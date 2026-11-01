#!/usr/bin/env python3
"""
go2_param_tool.py: read or set several Nav2 parameters in one go, for the
dashboard's Settings tab (scripts/runtime/go2_hub.py).

Runs inside go2-lidar3d on the session's DDS setup. One process for all the
parameters (the ros2 CLI takes ~1 s per call). Reads one JSON request from
argv[1], prints one JSON line:

  {"get": [["/node", "param"], ...]}
  {"set": [["/node", "param", value], ...]}      value: float or list of floats
  -> {"values": {"/node|param": value}, "errors": {"/node|param": "reason"}}

A set is followed by a get of the same parameters, so "values" is what Nav2
really holds afterwards.
"""

import json
import sys

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

TIMEOUT = 4.0


def to_value(v):
    pv = ParameterValue()
    if isinstance(v, list):
        pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        pv.double_array_value = [float(x) for x in v]
    elif isinstance(v, bool):
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = v
    else:
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(v)
    return pv


def from_value(pv):
    t = pv.type
    if t == ParameterType.PARAMETER_DOUBLE:
        return pv.double_value
    if t == ParameterType.PARAMETER_INTEGER:
        return pv.integer_value
    if t == ParameterType.PARAMETER_BOOL:
        return pv.bool_value
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(pv.double_array_value)
    if t == ParameterType.PARAMETER_STRING:
        return pv.string_value
    return None  # not set / unsupported


def call(node, cli, req):
    if not cli.wait_for_service(timeout_sec=TIMEOUT):
        return None
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=TIMEOUT)
    return fut.result()


def by_node(items):
    out = {}
    for it in items:
        out.setdefault(it[0], []).append(it)
    return out


def main():
    req = json.loads(sys.argv[1])
    rclpy.init()
    node = rclpy.create_node("go2_param_tool")
    values, errors = {}, {}
    try:
        for target, items in by_node(req.get("set", [])).items():
            cli = node.create_client(SetParameters, f"{target}/set_parameters")
            r = SetParameters.Request()
            r.parameters = [Parameter(name=p, value=to_value(v)) for _, p, v in items]
            res = call(node, cli, r)
            if res is None:
                for _, p, _ in items:
                    errors[f"{target}|{p}"] = f"{target} not reachable"
                continue
            for (_, p, _), result in zip(items, res.results):
                if not result.successful:
                    errors[f"{target}|{p}"] = result.reason or "refused"
        gets = req.get("get", []) + [[t, p] for t, p, _ in req.get("set", [])]
        for target, items in by_node(gets).items():
            cli = node.create_client(GetParameters, f"{target}/get_parameters")
            r = GetParameters.Request()
            r.names = [p for _, p in items]
            res = call(node, cli, r)
            if res is None:
                for _, p in items:
                    errors.setdefault(f"{target}|{p}", f"{target} not reachable")
                continue
            for (_, p), pv in zip(items, res.values):
                v = from_value(pv)
                if v is None:
                    errors.setdefault(f"{target}|{p}", "not declared (this drive mode?)")
                else:
                    values[f"{target}|{p}"] = v
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({"values": values, "errors": errors}), flush=True)


if __name__ == "__main__":
    main()
