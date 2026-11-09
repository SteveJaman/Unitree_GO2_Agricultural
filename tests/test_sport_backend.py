"""Unit tests for the sport_client backend in scripts/nodes/gait_supervisor.py.

The real SDK is never used: a fake unitree_sdk2py is injected into sys.modules
where a test needs one, and an import hook records any attempt to import it.
Needs rclpy (gait_supervisor imports it): run where ROS 2 is sourced.
"""
import importlib.abc
import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "nodes"))
import gait_supervisor as gs  # noqa: E402


class Log:
    def __init__(self):
        self.lines = []

    def _add(self, level):
        return lambda m: self.lines.append((level, m))

    def __getattr__(self, level):
        if level in ("info", "warn", "error", "fatal"):
            return self._add(level)
        raise AttributeError(level)

    def has(self, text, level=None):
        return any(text in m and (level is None or lv == level) for lv, m in self.lines)


class ImportSpy(importlib.abc.MetaPathFinder):
    """Records attempts to import unitree_sdk2py; blocks them unless a fake is installed."""

    def __init__(self):
        self.attempts = []

    def find_spec(self, name, path, target=None):
        if name.split(".")[0] == "unitree_sdk2py":
            self.attempts.append(name)
        return None


def clear_sdk():
    for k in [k for k in sys.modules if k.split(".")[0] == "unitree_sdk2py"]:
        del sys.modules[k]


def install_fake_sdk(has_foot=True, fail=0, init_error=None):
    """A fake unitree_sdk2py. Returns the list FootRaiseHeight calls land in."""
    clear_sdk()
    calls = []
    state = {"fail": fail}

    class SportClient:
        def SetTimeout(self, t):
            pass

        def Init(self):
            if init_error:
                raise init_error

    if has_foot:
        def FootRaiseHeight(self, d):
            if state["fail"] > 0:
                state["fail"] -= 1
                raise RuntimeError("dds timeout")
            calls.append(round(d, 4))
            return 0
        SportClient.FootRaiseHeight = FootRaiseHeight

    root = types.ModuleType("unitree_sdk2py")
    for name in ("core", "go2", "go2.sport"):
        sys.modules["unitree_sdk2py." + name] = types.ModuleType("unitree_sdk2py." + name)
    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelFactoryInitialize = lambda *a: None
    sc = types.ModuleType("unitree_sdk2py.go2.sport.sport_client")
    sc.SportClient = SportClient
    sys.modules.update({"unitree_sdk2py": root, "unitree_sdk2py.core.channel": channel,
                        "unitree_sdk2py.go2.sport.sport_client": sc})
    return calls


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def backend(dry_run, enable, clock=None, log=None):
    return gs.SportClientBackend(log or Log(), dry_run, enable, nominal_foot_height=0.09,
                                 max_rate_hz=2.0, clock=clock or Clock())


def test_both_gates_closed_logs_dry_run_and_never_imports_sdk():
    clear_sdk()
    spy = ImportSpy()
    sys.meta_path.insert(0, spy)
    try:
        log = Log()
        b = backend(True, False, log=log)
        assert b.send("HIGH_STEP") == "dry_run"
        assert log.has("[DRY_RUN] would send FootRaiseHeight(+0.030) for HIGH_STEP")
        assert spy.attempts == []
    finally:
        sys.meta_path.remove(spy)


def test_one_gate_open_still_inert_and_names_the_missing_gate():
    clear_sdk()
    spy = ImportSpy()
    sys.meta_path.insert(0, spy)
    try:
        log = Log()
        assert backend(True, True, log=log).send("CAREFUL") == "dry_run"      # hardware on, dry_run on
        assert log.has("dry_run is true") and not log.has("enable_hardware_backend is false")
        log2 = Log()
        assert backend(False, False, log=log2).send("CAREFUL") == "dry_run"   # dry_run off, hardware off
        assert log2.has("enable_hardware_backend is false") and not log2.has("dry_run is true")
        assert spy.attempts == []
    finally:
        sys.meta_path.remove(spy)


def test_both_gates_open_without_sdk_logs_fatal_and_does_not_crash():
    clear_sdk()
    sys.modules["unitree_sdk2py"] = None      # makes the import fail like a missing package
    try:
        log = Log()
        b = backend(False, True, log=log)
        assert b.send("CAREFUL") == "inactive"
        assert log.has("[FATAL] unitree_sdk2py not available; hardware backend inactive", "fatal")
        assert b.send("NORMAL") == "inactive"    # stays inactive, no retry storm
    finally:
        clear_sdk()


def test_missing_foot_raise_height_is_fatal_and_inert():
    install_fake_sdk(has_foot=False)
    log = Log()
    assert backend(False, True, log=log).send("CAREFUL") == "inactive"
    assert log.has("has no FootRaiseHeight", "fatal")
    clear_sdk()


def test_channel_init_failure_is_fatal_and_inert():
    install_fake_sdk(init_error=OSError("no interface eth9"))
    log = Log()
    assert backend(False, True, log=log).send("CAREFUL") == "inactive"
    assert log.has("channel init failed", "fatal")
    clear_sdk()


def test_sends_delta_from_nominal():
    calls = install_fake_sdk()
    clock = Clock()
    b = backend(False, True, clock=clock)
    assert b.send("NORMAL") == "sent"            # 0.04 - 0.09
    clock.t += 1.0
    assert b.send("CAREFUL") == "sent"           # 0.12 - 0.09
    assert calls == [-0.05, 0.03]
    clear_sdk()


def test_clamps_large_lift_to_nominal_plus_003():
    install_fake_sdk()
    log = Log()
    b = backend(False, True, log=log)
    assert abs(b.delta_for(0.50) - 0.03) < 1e-9          # 0.09 + 0.03 = 0.12 m lift
    assert log.has("[WARN] foot delta clamped from +0.410 to +0.030", "warn")
    assert abs(b.delta_for(0.0) - (-0.06)) < 1e-9        # minimum lift is 0.03 m, not 0
    clear_sdk()


def test_rate_limit_and_unchanged():
    calls = install_fake_sdk()
    clock = Clock()
    b = backend(False, True, clock=clock)
    assert b.send("CAREFUL") == "sent"
    assert b.send("CAREFUL") == "unchanged"
    clock.t += 0.2
    assert b.send("NORMAL") == "rate_limited"    # 2 Hz: 0.5 s between SDK calls
    clock.t += 0.4
    assert b.send("NORMAL") == "sent"
    assert calls == [0.03, -0.05]
    clear_sdk()


def test_three_failures_send_normal_once_then_stop():
    calls = install_fake_sdk(fail=3)
    clock = Clock()
    log = Log()
    b = backend(False, True, clock=clock, log=log)
    for gait in ("CAREFUL", "HIGH_STEP", "STOP"):
        assert b.send(gait) == "failed"
        clock.t += 1.0
    assert calls == [-0.05]                       # the one NORMAL after the third failure
    assert log.has("3 consecutive SDK failures", "error")
    assert b.send("CAREFUL", force=True) == "halted"
    assert calls == [-0.05]
    clear_sdk()


def test_override_normal_pauses_until_gait_changes():
    calls = install_fake_sdk()
    clock = Clock()
    b = backend(False, True, clock=clock)
    b.send("CAREFUL")
    clock.t += 1.0
    assert b.override_normal("CAREFUL") == "sent"
    clock.t += 1.0
    assert b.send("CAREFUL") == "paused"
    assert b.send("STOP") == "sent"               # a new gait from the supervisor resumes
    assert calls == [0.03, -0.05, 0.03]
    clear_sdk()


def test_heartbeat_staleness():
    assert gs.inputs_stale(10.0, (None, 9.5), 2.0)         # never arrived
    assert gs.inputs_stale(10.0, (7.0, 9.5), 2.0)          # odometry 3 s old
    assert not gs.inputs_stale(10.0, (9.0, 9.5), 2.0)


def test_lift_table_matches_sim_bridge_and_sdk_range():
    for gait, lift in gs.HARDWARE_LIFT.items():
        d = lift - 0.09
        assert gs.SDK_DELTA_MIN - 1e-9 <= d <= gs.SDK_DELTA_MAX + 1e-9, gait
    assert gs.HARDWARE_LIFT == {"NORMAL": 0.04, "HIGH_STEP": 0.12, "CAREFUL": 0.12, "STOP": 0.12}
