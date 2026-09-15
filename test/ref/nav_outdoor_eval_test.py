"""ctest `nav_outdoor_eval` -- unit test for tools/nav_outdoor_eval.py's pure
math (window/step/radius), on synthetic traces with hand-computable answers.
No MF4, no board, no gen_fusion_trace: these are plain numpy-in, dict-out
functions, exercised directly.

Usage:  python nav_outdoor_eval_test.py
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "tools"))
import nav_outdoor_eval as ev   # noqa: E402  (path must be set up first)

failures = []


def fail(msg):
    failures.append(msg)
    print("FAIL: " + msg)


def check_close(name, got, want, tol=1e-9):
    if not math.isclose(got, want, abs_tol=tol):
        fail(f"{name}: got {got!r}, want {want!r} (tol {tol})")


def test_contiguous_runs():
    t = np.arange(10, dtype=np.float64)
    mask = np.array([False, True, True, False, True, False, False, True, True, True])
    runs = ev.contiguous_runs(mask, t)
    if len(runs) != 3:
        fail(f"contiguous_runs: expected 3 runs, got {len(runs)}: {runs}")
        return
    want = [
        {"t_start": 1.0, "t_end": 2.0, "duration_s": 1.0, "i_start": 1, "i_end": 2},
        {"t_start": 4.0, "t_end": 4.0, "duration_s": 0.0, "i_start": 4, "i_end": 4},
        {"t_start": 7.0, "t_end": 9.0, "duration_s": 2.0, "i_start": 7, "i_end": 9},
    ]
    for got, exp in zip(runs, want):
        for k, v in exp.items():
            check_close(f"contiguous_runs[{k}]", float(got[k]), float(v))


def test_sliding_window_radius():
    # Hand-traced in the flight-dev report: a single step from 0 to 10 at
    # t=3, window_s=3, gives radius = 7.5, 5.0, 7.5 for the three windows
    # that first reach full length (t=3,4,5).
    t = np.array([0, 1, 2, 3, 4, 5], dtype=np.float64)
    x = np.array([0, 0, 0, 10, 10, 10], dtype=np.float64)
    y = np.zeros_like(x)
    radius = ev.sliding_window_radius(t, x, y, window_s=3.0)
    want = np.array([7.5, 5.0, 7.5])
    if radius.shape != want.shape:
        fail(f"sliding_window_radius: shape {radius.shape}, want {want.shape} (got {radius})")
        return
    for i, (g, w) in enumerate(zip(radius, want)):
        check_close(f"sliding_window_radius[{i}]", float(g), float(w))

    # A stationary point has zero radius over any full window.
    t2 = np.arange(0, 20, dtype=np.float64)
    x2 = np.full_like(t2, 3.0)
    y2 = np.full_like(t2, -2.0)
    r2 = ev.sliding_window_radius(t2, x2, y2, window_s=5.0)
    if r2.size == 0:
        fail("sliding_window_radius: stationary case produced no windows")
    elif not np.allclose(r2, 0.0):
        fail(f"sliding_window_radius: stationary case not all zero: {r2}")


def test_tangent_plane():
    lat = np.array([52.0, 52.0009], dtype=np.float64)
    lon = np.array([13.0, 13.0], dtype=np.float64)
    north, east = ev.tangent_plane(lat, lon, origin_idx=0)
    check_close("tangent_plane north[0]", float(north[0]), 0.0)
    check_close("tangent_plane east[0]", float(east[0]), 0.0)
    # 0.0009 deg lat * 111132 m/deg = 100.0188 m
    check_close("tangent_plane north[1]", float(north[1]), 100.0188, tol=1e-6)
    check_close("tangent_plane east[1]", float(east[1]), 0.0, tol=1e-9)


def test_detect_dwells_and_step_episode():
    dt = 0.5
    # Phase A: dwell at rest (0,0), t in [0,5]
    tA = np.arange(0.0, 5.0 + dt, dt)
    # Move out: one sample well above the speed threshold
    tMove1 = np.array([tA[-1] + dt])
    # Phase B: dwell at the 0.5 m target, t offset by 6.0
    tB = tMove1[-1] + dt + np.arange(0.0, 5.0 + dt, dt)
    tMove2 = np.array([tB[-1] + dt])
    # Phase C: dwell back at the start
    tC = tMove2[-1] + dt + np.arange(0.0, 5.0 + dt, dt)

    t = np.concatenate([tA, tMove1, tB, tMove2, tC])
    speed = np.concatenate([
        np.zeros_like(tA), [2.0], np.zeros_like(tB), [2.0], np.zeros_like(tC),
    ])
    posN = np.concatenate([
        np.zeros_like(tA), [0.0], np.full_like(tB, 0.5), [0.5], np.zeros_like(tC),
    ])
    posE = np.zeros_like(t)

    dwells = ev.detect_dwells(t, speed, speed_thresh=0.15, min_dwell_s=3.0)
    if len(dwells) != 3:
        fail(f"detect_dwells: expected 3 dwells, got {len(dwells)}: {dwells}")
        return
    (a0, a1), (b0, b1), (c0, c1) = dwells
    check_close("dwell A start", t[a0], 0.0)
    check_close("dwell A end", t[a1], 5.0)
    check_close("dwell B start", t[b0], 6.0)
    check_close("dwell C end", t[c1], 17.0)

    ep = ev.step_episode_from_dwells(t, posN, posE, speed, dwells[0], dwells[1], dwells[2])
    check_close("step displacement_m", ep["displacement_m"], 0.5)
    if not ep["displacement_pass"]:
        fail(f"step displacement_pass should be True for a 0.5 m step, got {ep}")
    check_close("step closure_m", ep["closure_m"], 0.0)
    if not ep["closure_pass"]:
        fail("step closure_pass should be True for a 0.0 m closure")
    check_close("ground_speed_peak_mps", ep["ground_speed_peak_mps"], 2.0)

    # The auto-detector (step=2 over consecutive dwell triples) must find
    # exactly this one episode from the same trace end to end.
    class FakeRec:
        pass
    fake = FakeRec()
    fake.t = t
    fake.sig = {"NavVelNorth": speed, "NavVelEast": np.zeros_like(speed),
                "NavPosNorth": posN, "NavPosEast": posE}
    result = ev.step_response_metrics(fake)
    if not result["evaluated"] or len(result["episodes"]) != 1:
        fail(f"step_response_metrics auto-detect: expected 1 episode, got {result}")
    else:
        check_close("auto episode displacement_m", result["episodes"][0]["displacement_m"], 0.5)


def main() -> int:
    test_contiguous_runs()
    test_sliding_window_radius()
    test_tangent_plane()
    test_detect_dwells_and_step_episode()

    if failures:
        print(f"\n{len(failures)} FAILURE(S)")
        return 1
    print("all nav_outdoor_eval unit tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
