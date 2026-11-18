"""scripts/tools/ollama_mem_guard.py: when it unloads the VLM (no Ollama needed)."""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "tools"))
import ollama_mem_guard as g  # noqa: E402

GB = g.GB


def run(monkeypatch, held, free):
    calls = []
    monkeypatch.setattr(g, "runner_pids", lambda: [1])
    monkeypatch.setattr(g, "rss_anon", lambda pid: int(held * GB))
    monkeypatch.setattr(g, "mem_available", lambda: int(free * GB))
    monkeypatch.setattr(g, "unload_all", lambda url, log: calls.append(url) or True)
    args = argparse.Namespace(url="http://localhost:11434", max_gb=4.0, min_free_gb=1.5)
    _, unloaded = g.check(args, lambda m: None)
    return unloaded, calls


def test_unloads_when_llama_server_is_too_big(monkeypatch):
    assert run(monkeypatch, 4.2, 6.0) == (True, ["http://localhost:11434"])


def test_unloads_when_the_computer_runs_out_of_ram(monkeypatch):
    assert run(monkeypatch, 2.0, 1.0)[0]


def test_leaves_it_alone_otherwise(monkeypatch):
    assert run(monkeypatch, 3.5, 5.0) == (False, [])
    assert run(monkeypatch, 0.5, 1.0) == (False, [])        # low RAM, but not its fault
