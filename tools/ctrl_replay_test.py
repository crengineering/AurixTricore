"""Vector-replay client for the flight-controller replay harness on the
AURIX target (src/asw/CtrlReplay.c, UDP port 5556).

Modes:
    python tools/ctrl_replay_test.py                       # smoke test
    python tools/ctrl_replay_test.py --csv in.csv          # replay vectors
    python tools/ctrl_replay_test.py --csv in.csv --out out.csv
    python tools/ctrl_replay_test.py --csv in.csv --expect ref.csv --tol 1e-4

CSV formats (comma-separated, optional header line starting with a letter):
    input : 16 columns  p_ned_soll[3], p_ned_ist[3], v_b_ist[3], psi_soll,
                        phi_ist[3], om_ist[3]
    ref   : 17 columns  T_soll, phi_soll[3], om_soll[3], tau_soll[3],
                        w_cmd[4], tau_I[3]
    output: 18 columns  (ref columns + t_exec_us)

Protocol: see docs/CTRL_REPLAY.md. The client runs in lockstep (send one
frame, wait for its reply) so the integrator state on the target stays
deterministic. A lost STEP packet aborts the run — a silent retry would
advance the PI integrator twice.

Exit code 0 on success, 1 on any failure (timeout, NAK, budget overrun,
comparison mismatch).
"""

import argparse
import csv
import socket
import struct
import sys

IN_FLOATS = 16
OUT_FLOATS = 17
IN_BYTES = IN_FLOATS * 4
OUT_BYTES = OUT_FLOATS * 4 + 4          # + uint32 t_exec ticks

CMD_RESET = b"\x00"
CMD_STATS = b"\x01"
NAK = 0xEE

TICKS_PER_US = 100.0                    # STM @ 100 MHz
BUDGET_US = 1000.0                      # 1 ms controller period

OUT_HEADER = ("T_soll,phi_soll_0,phi_soll_1,phi_soll_2,"
              "om_soll_0,om_soll_1,om_soll_2,"
              "tau_soll_0,tau_soll_1,tau_soll_2,"
              "w_cmd_0,w_cmd_1,w_cmd_2,w_cmd_3,"
              "tau_I_0,tau_I_1,tau_I_2,t_exec_us")


class ReplayClient:
    def __init__(self, ip, port, timeout):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)

    def _xfer(self, payload, expect_len):
        self.sock.sendto(payload, self.addr)
        data, _ = self.sock.recvfrom(2048)
        if len(data) == 1 and data[0] == NAK:
            raise RuntimeError("target answered NAK")
        if len(data) != expect_len:
            raise RuntimeError(
                f"unexpected reply length {len(data)}, expected {expect_len}")
        return data

    def wait_ready(self, seconds):
        """Poll STATS until the target answers (e.g. right after flashing)."""
        deadline = seconds
        while True:
            try:
                self._xfer(CMD_STATS, 16)
                return
            except (socket.timeout, OSError):
                deadline -= self.sock.gettimeout()
                if deadline <= 0:
                    raise RuntimeError(
                        f"target {self.addr[0]}:{self.addr[1]} not reachable")

    def reset(self):
        self._xfer(CMD_RESET, 1)

    def step(self, vec16):
        data = self._xfer(struct.pack("<16f", *vec16), OUT_BYTES)
        out = struct.unpack("<17f", data[:OUT_FLOATS * 4])
        (ticks,) = struct.unpack("<I", data[OUT_FLOATS * 4:])
        return out, ticks / TICKS_PER_US

    def stats(self):
        data = self._xfer(CMD_STATS, 16)
        count, tmin, tmax = struct.unpack("<III", data[:12])
        (mean,) = struct.unpack("<f", data[12:])
        return count, tmin / TICKS_PER_US, tmax / TICKS_PER_US, mean / TICKS_PER_US


def read_csv(path, ncols):
    rows = []
    with open(path, newline="") as f:
        for lineno, row in enumerate(csv.reader(f), 1):
            if not row or (lineno == 1 and not _is_number(row[0])):
                continue                       # skip header / blank lines
            if len(row) < ncols:
                sys.exit(f"{path}:{lineno}: expected {ncols} columns, "
                         f"got {len(row)}")
            rows.append([float(x) for x in row[:ncols]])
    return rows


def _is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def print_stats(client):
    count, tmin, tmax, tmean = client.stats()
    print(f"\nexecution time over {count} tick(s): "
          f"min {tmin:.2f} us / mean {tmean:.2f} us / max {tmax:.2f} us "
          f"(budget {BUDGET_US:.0f} us)")
    if tmax > BUDGET_US:
        print("FAIL: budget exceeded")
        return False
    return True


def run_smoke(client):
    """Hover + height-step sanity check with hand-computable expectations."""
    client.reset()

    # hover: setpoint == actual, everything at rest
    hover = [0.0] * IN_FLOATS
    out, t_us = client.step(hover)
    T, w = out[0], out[10:14]
    # T = m*g = 11.772 N; w = sqrt(5e4 * T) ~ 767.2 rad/s on all motors
    print(f"hover : T={T:.3f} N  w_cmd=[{w[0]:.1f} {w[1]:.1f} "
          f"{w[2]:.1f} {w[3]:.1f}] rad/s  t={t_us:.2f} us")
    ok = abs(T - 11.772) < 1e-3 and all(abs(x - 767.20) < 0.1 for x in w)

    # step: copter 1 m too low (z_ist = +1 in NED) -> more thrust
    low = list(hover)
    low[5] = 1.0                               # p_ned_ist[2]
    out, t_us = client.step(low)
    print(f"z-step: T={out[0]:.3f} N (expected 14.472)  t={t_us:.2f} us")
    ok = ok and abs(out[0] - 14.472) < 1e-3

    # integrator must move under a rate error and return to 0 after reset
    rate = [0.0] * IN_FLOATS
    rate[13] = 1.0                             # om_ist[0] = 1 rad/s
    out, _ = client.step(rate)
    ok = ok and abs(out[14]) > 0.0
    client.reset()
    out, _ = client.step(hover)
    ok = ok and out[14] == 0.0 and out[15] == 0.0 and out[16] == 0.0
    print(f"integrator move/reset: {'ok' if ok else 'FAIL'}")

    client.reset()                             # leave a clean state behind
    for _ in range(1000):                      # timing over a real run
        client.step(hover)
    return print_stats(client) and ok


def run_csv(client, args):
    vecs = read_csv(args.csv, IN_FLOATS)
    refs = read_csv(args.expect, OUT_FLOATS) if args.expect else None
    if refs is not None and len(refs) != len(vecs):
        sys.exit(f"vector count mismatch: {len(vecs)} inputs, "
                 f"{len(refs)} reference rows")

    client.reset()
    results = []
    worst = (0.0, -1, -1)                      # (abs error, row, col)
    for i, vec in enumerate(vecs):
        out, t_us = client.step(vec)
        results.append(list(out) + [t_us])
        if refs is not None:
            for j, (a, b) in enumerate(zip(out, refs[i])):
                err = abs(a - b)
                if err > worst[0]:
                    worst = (err, i, j)

    if args.out:
        with open(args.out, "w", newline="") as f:
            f.write(OUT_HEADER + "\n")
            w = csv.writer(f)
            w.writerows([f"{x:.9g}" for x in row] for row in results)
        print(f"{len(results)} output rows -> {args.out}")

    ok = print_stats(client)
    if refs is not None:
        print(f"comparison over {len(vecs)} ticks: max abs error "
              f"{worst[0]:.3e} (row {worst[1] + 1}, col {worst[2] + 1}), "
              f"tolerance {args.tol:g}")
        if worst[0] > args.tol:
            print("FAIL: tolerance exceeded")
            ok = False
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ip", default="192.168.0.10")
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--timeout", type=float, default=1.0,
                    help="per-frame reply timeout in s")
    ap.add_argument("--wait", type=float, default=15.0,
                    help="max s to wait for the target after power-up")
    ap.add_argument("--csv", help="input vector file (16 floats per row)")
    ap.add_argument("--out", help="write outputs (17 floats + t_exec_us)")
    ap.add_argument("--expect", help="reference outputs (17 floats per row)")
    ap.add_argument("--tol", type=float, default=1e-4,
                    help="max abs error vs. --expect")
    args = ap.parse_args()

    client = ReplayClient(args.ip, args.port, args.timeout)
    try:
        client.wait_ready(args.wait)
        ok = run_csv(client, args) if args.csv else run_smoke(client)
    except (RuntimeError, socket.timeout) as e:
        sys.exit(f"error: {e}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
