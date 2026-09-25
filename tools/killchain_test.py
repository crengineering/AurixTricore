#!/usr/bin/env python3
"""killchain_test.py -- SYS1-005 kill-chain bench recording (SYS2-SAF-001/-002/-003, PIL).

Drives the arming state machine (Motor.c) over XCP from this PC as the ONLY master,
logs the ESC/state block at 10 Hz and captures the DShot lines with the Nano 33 IoT
analyzer (sibling project C:/Users/chris/Projects/NanoLogicAnalyzer) at the decisive moments.
the firmware's link heartbeat (Xcp_linkAlive, 500 ms), so "link loss" is produced by
simply not sending anything for a few seconds -- with a known timestamp.

Phases
  P0  baseline           disarmed, zero frames flowing, ESCs answering
  P1  ARM                CAL_motorCmd=1 -> INIT (beeps) -> ARMED
  P2  manual setpoint    CAL_motorManual=1, CAL_motorManualSp_M1=<sp> -> M1 frames carry <sp>
  P3  DISARM             CAL_motorCmd=0 -> next frame zero, words cleared   (SAF-001)
  P4  link loss          ARM again, then silence -> frames stop, FAILSAFE   (SAF-002)
  P5  link back          polling resumes -> DISARMED, zeros, ESCs answering

Outputs (in --out): <stem>.csv (10 Hz log), <stem>_events.json, <stem>_cap_*.json
(analyzer captures), <stem>.meta.txt (firmware, git, metrics, SHA-256 of every file).

PRECONDITIONS: GUI disconnected (one XCP master), analyzer GUI closed (COM3 free),
NO MOTOR mounted unless you mean it. The script refuses to start unless the board
reports DISARMED with all three cal words at 0.
"""
import argparse
import csv
import hashlib
import json
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent.parent / "NanoLogicAnalyzer"))   # sibling project (generic Nano 33 IoT analyzer)
import xcp_read                     # noqa: E402  (connect, short_upload_chunked, load_symbols)
import logic_gui as lg              # noqa: E402  (Capture, decoders)

A2L = HERE.parent / "docs" / "AurixTricore.a2l"
ESC_BASE = 0x70030700
ESC_LEN = 0x74
CAL_WORDS = 0x70030140              # motorCmd, motorManual, motorManualSp[4]
CAL_LEN = 16


def a2l_addresses():
    """name -> address for every MEASUREMENT / CHARACTERISTIC in the A2L."""
    out = {}
    txt = A2L.read_text(encoding="utf-8", errors="replace")
    for m in re.finditer(r"/begin (MEASUREMENT|CHARACTERISTIC) (\w+) .*?(?:ECU_ADDRESS|VALUE) (0x[0-9A-Fa-f]+)", txt, re.S):
        out[m.group(2)] = int(m.group(3), 16)
    return out


class Board:
    def __init__(self):
        self.master = xcp_read.connect()      # pyxcp Master is a context manager: enter opens the socket
        self.x = self.master.__enter__()
        self.x.connect()
        self.syms = xcp_read.load_symbols()
        self.a = a2l_addresses()
        need = ["CAL_motorCmd", "CAL_motorManual", "CAL_motorManualSp_M1", "EscMotorState",
                "EscM1Alive", "EscM1TlmCount", "EscCrcFail"]
        missing = [n for n in need if n not in self.a]
        if missing:
            sys.exit(f"A2L lacks {missing}; regenerate docs/AurixTricore.a2l")
        if "g_dshotFrames" not in self.syms:
            sys.exit("g_dshotFrames not in the map (build first)")
        self.frames_addr = self.syms["g_dshotFrames"]

    def read(self, addr, n):
        return bytes(xcp_read.short_upload_chunked(self.x, addr, n))

    def write32(self, addr, value):
        self.x.setMta(addr, 0)
        self.x.download(int(value).to_bytes(4, "little"))

    def write16(self, addr, value):
        self.x.setMta(addr, 0)
        self.x.download(int(value).to_bytes(2, "little"))

    def sample(self):
        esc = self.read(ESC_BASE, ESC_LEN)
        cal = self.read(CAL_WORDS, CAL_LEN)
        fr = int.from_bytes(self.read(self.frames_addr, 4), "little")
        o = lambda name: self.a[name] - ESC_BASE
        s = {
            "state": esc[o("EscMotorState")],
            "motorCmd": int.from_bytes(cal[0:4], "little"),
            "manual": int.from_bytes(cal[4:8], "little"),
            "frames": fr,
            "crcFail": int.from_bytes(esc[o("EscCrcFail"):o("EscCrcFail") + 4], "little"),
        }
        for i in range(4):
            s[f"sp{i+1}"] = int.from_bytes(cal[8 + 2 * i:10 + 2 * i], "little")
            s[f"alive{i+1}"] = esc[o("EscM1Alive") + i]
            a = o("EscM1TlmCount") + 4 * i
            s[f"tlm{i+1}"] = int.from_bytes(esc[a:a + 4], "little")
        return s

    def version(self):
        v = self.read(0x70030004, 3)
        return f"{v[0]}.{v[1]}.{v[2]}"


class Analyzer:
    def __init__(self, port):
        import serial
        self.s = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(1.2)
        self.s.reset_input_buffer()
        self.cmd("l20000"); self.cmd("d0")

    def cmd(self, c, wait=0.4):
        self.s.write((c + "\n").encode()); time.sleep(wait)
        return self.s.read(400000).decode("ascii", "replace")

    def arm(self, trig, timeout_ms, between=None):
        """Configure trigger + timeout (two serial round trips, ~0.3 s each); `between` is called
        after each so the caller can keep its XCP heartbeat going."""
        self.cmd(f"w{timeout_ms}", 0.3)
        if between: between()
        self.cmd(trig, 0.3)
        if between: between()

    def start(self):
        """Fire the capture: returns immediately, the Nano waits for its trigger on its own."""
        self.s.reset_input_buffer()
        self.s.write(b"a\n")

    def collect(self, wait_s):
        time.sleep(wait_s)
        out = self.s.read(400000).decode("ascii", "replace")
        return self._parse(out)

    def capture(self, trig, timeout_ms):
        self.arm(trig, timeout_ms)
        self.start()
        return self.collect(timeout_ms / 1000.0 + 2.5)

    def _parse(self, out):
        lines = out.splitlines()
        hdr = [l for l in lines if l.startswith("#CAP")]
        if not hdr or "#END" not in lines:
            return None
        i = lines.index(hdr[0]); j = lines.index("#END")
        cap = lg.Capture.from_lines(hdr[0], lines[i + 1:j])
        cap.names = lg.DEFAULT_NAMES
        return cap

    def close(self):
        self.s.close()


def summarize(cap):
    """Per DShot channel: list of (throttle, telem, crc_ok) for complete frames; trigok."""
    if cap is None:
        return {"error": "no capture"}
    res = {"trigok": cap.meta.get("trigok"), "ns": cap.meta.get("ns")}
    for ch in range(4):
        fr = lg.decode_dshot(cap.transitions(ch), cap.t_end_us)
        res[f"M{ch+1}"] = [(f["throttle"], f["telem"], f["crc_ok"]) for f in fr if "error" not in f]
        res[f"M{ch+1}_partial"] = sum(1 for f in fr if "error" in f)
    return res


def sha16(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16].upper()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=r"C:\Users\chris\Projects\Measurement Data\20260925_SYS1-005_killchain")
    ap.add_argument("--stem", default=time.strftime("%Y-%m-%d_killchain"))
    ap.add_argument("--nano", default="COM3")
    ap.add_argument("--setpoint", type=int, default=100, help="manual DShot value for M1 in P2 (48..2047)")
    ap.add_argument("--silence", type=float, default=4.0, help="seconds without any XCP command in P4")
    ap.add_argument("--rate", type=float, default=10.0, help="log rate [Hz]")
    args = ap.parse_args()

    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    stem = out / args.stem
    b = Board()
    nano = Analyzer(args.nano)
    rows, events, caps, metrics = [], [], {}, {}
    t0 = time.perf_counter()
    now = lambda: time.perf_counter() - t0
    phase_box = ["P0"]

    def log(n_s, ph):
        end = now() + n_s
        while now() < end:
            s = b.sample(); s["t"] = round(now(), 3); s["phase"] = ph
            rows.append(s)
            time.sleep(max(0.0, 1.0 / args.rate))
        return rows[-1]

    def event(name, **kw):
        events.append({"t": round(now(), 4), "event": name, **kw})
        print(f"[{now():8.3f}] {name} {kw if kw else ''}")

    def wait_state(target, limit_s, ph):
        end = now() + limit_s
        while now() < end:
            s = b.sample(); s["t"] = round(now(), 3); s["phase"] = ph
            rows.append(s)
            if s["state"] == target:
                return s["t"]
            time.sleep(0.02)
        return None

    def finish_cap(name, c, trig):
        summ = summarize(c)
        caps[name] = summ
        if c is not None:
            json.dump(c.to_json(), open(f"{stem}_cap_{name}.json", "w"))
        event(f"capture {name}", trig=trig, result={k: v for k, v in summ.items() if not k.endswith("_partial")})
        return summ

    def in_thread(fn):
        """Run a blocking analyzer call in a thread while this thread keeps the XCP heartbeat."""
        box = {}
        th = threading.Thread(target=lambda: box.update(r=fn()))
        th.start()
        while th.is_alive():
            log(0.1, phase_box[0])
        th.join()
        return box.get("r")

    def cap_live(name, trig, timeout_ms, ph):
        """Capture while XCP polling continues (the polling is the link heartbeat)."""
        phase_box[0] = ph
        c = in_thread(lambda: nano.capture(trig, timeout_ms))
        return finish_cap(name, c, trig)

    def cap(name, trig, timeout_ms):
        c = nano.capture(trig, timeout_ms)
        summ = summarize(c)
        caps[name] = summ
        if c is not None:
            json.dump(c.to_json(), open(f"{stem}_cap_{name}.json", "w"))
        event(f"capture {name}", trig=trig, result={k: v for k, v in summ.items() if not k.endswith("_partial")})
        return summ

    fw = b.version()
    event("start", fw=fw)
    s0 = b.sample()
    if s0["state"] != 0 or s0["motorCmd"] != 0 or s0["manual"] != 0 or any(s0[f"sp{i}"] for i in (1, 2, 3, 4)):
        nano.close(); sys.exit(f"precondition failed: {s0}")
    try:
        # P0 baseline
        log(5.0, "P0")
        cap_live("P0_frames", "t0F", 2000, "P0")
        # P1 ARM
        event("write CAL_motorCmd=1"); b.write32(b.a["CAL_motorCmd"], 1)
        t_arm = now()
        t_init = wait_state(1, 1.0, "P1"); event("state INIT", latency_s=None if t_init is None else round(t_init - t_arm, 3))
        t_armed = wait_state(2, 8.0, "P1"); event("state ARMED", after_s=None if t_armed is None else round(t_armed - t_arm, 3))
        metrics["arm_to_init_s"] = None if t_init is None else round(t_init - t_arm, 3)
        metrics["arm_to_armed_s"] = None if t_armed is None else round(t_armed - t_arm, 3)
        log(1.0, "P1")
        # P2 manual setpoint on M1
        event("write CAL_motorManual=1"); b.write32(b.a["CAL_motorManual"], 1)
        event(f"write CAL_motorManualSp_M1={args.setpoint}"); b.write16(b.a["CAL_motorManualSp_M1"], args.setpoint)
        log(1.0, "P2")
        cap_live("P2_setpoint", "t0F", 2000, "P2")
        log(1.0, "P2")
        # P3 DISARM
        event("write CAL_motorCmd=0 (DISARM)"); b.write32(b.a["CAL_motorCmd"], 0)
        t_dis = now()
        t_d0 = wait_state(0, 1.0, "P3"); event("state DISARMED", latency_s=None if t_d0 is None else round(t_d0 - t_dis, 3))
        metrics["disarm_to_disarmed_s"] = None if t_d0 is None else round(t_d0 - t_dis, 3)
        cap_live("P3_after_disarm", "t0F", 2000, "P3")
        s = log(1.0, "P3"); metrics["words_after_disarm"] = {k: s[k] for k in ("motorCmd", "manual", "sp1", "sp2", "sp3", "sp4")}
        # P4 link loss
        event("write CAL_motorCmd=1 (ARM again)"); b.write32(b.a["CAL_motorCmd"], 1)
        t_arm2 = now()
        t_armed2 = wait_state(2, 8.0, "P4"); event("state ARMED", after_s=None if t_armed2 is None else round(t_armed2 - t_arm2, 3))
        s_before = log(1.0, "P4")
        phase_box[0] = "P4"
        in_thread(lambda: nano.arm("t0F", 300))                  # window 0.3 s, armed while the link is alive
        event("XCP SILENCE begins", silence_s=args.silence)
        t_sil = now()
        time.sleep(0.10)
        nano.start()                                              # +0.10 s: frames must still be there (trigok=1)
        c1 = nano.collect(0.9)
        finish_cap("P4_silence_+0.1s_window0.3s", c1, "t0F")
        nano.arm("t0F", 1500)                                     # ~0.6 s of serial, still silent on XCP
        t_c2 = now() - t_sil
        nano.start()                                              # frames must be GONE (trigok=0 expected)
        c2 = nano.collect(2.2)
        finish_cap(f"P4_silence_+{t_c2:.1f}s_window1.5s", c2, "t0F")
        while now() - t_sil < args.silence:
            time.sleep(0.05)
        event("XCP resumes", silence_actual_s=round(now() - t_sil, 3))
        s_after = b.sample(); s_after["t"] = round(now(), 3); s_after["phase"] = "P5"; rows.append(s_after)
        event("first sample after silence", state=s_after["state"], motorCmd=s_after["motorCmd"], manual=s_after["manual"])
        metrics["frames_during_silence"] = s_after["frames"] - s_before["frames"]
        metrics["frames_expected_if_streaming"] = int(round((s_after["t"] - s_before["t"]) * 1000))
        metrics["state_first_sample_after_silence"] = s_after["state"]
        metrics["words_after_failsafe"] = {k: s_after[k] for k in ("motorCmd", "manual", "sp1")}
        # P5 recovery
        t_rec = wait_state(0, 2.0, "P5"); event("state DISARMED after link back", after_s=None if t_rec is None else round(t_rec - s_after["t"], 3))
        s_end = log(3.0, "P5")
        cap_live("P5_recovered", "t0F", 2000, "P5")
        metrics["alive_after_recovery"] = [s_end[f"alive{i}"] for i in (1, 2, 3, 4)]
        metrics["frames_rate_P5_per_s"] = round((s_end["frames"] - s_after["frames"]) / max(1e-3, s_end["t"] - s_after["t"]))
        metrics["crcFail_end"] = s_end["crcFail"]
    finally:
        # always leave the board disarmed, whatever happened
        try:
            b.write32(b.a["CAL_motorCmd"], 0); b.write32(b.a["CAL_motorManual"], 0); b.write16(b.a["CAL_motorManualSp_M1"], 0)
            event("final DISARM written")
        except Exception as e:  # noqa: BLE001
            event("final DISARM FAILED", error=str(e))
        nano.close()
        try:
            b.x.disconnect()
        except Exception:
            pass
        try:
            b.master.__exit__(None, None, None)
        except Exception:
            pass

    # ---- files -------------------------------------------------------------
    csv_path = f"{stem}.csv"
    with open(csv_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    json.dump(events, open(f"{stem}_events.json", "w"), indent=1)
    git = subprocess.run(["git", "-C", str(HERE.parent), "log", "-1", "--format=%h %s"], capture_output=True, text=True).stdout.strip()
    branch = subprocess.run(["git", "-C", str(HERE.parent), "branch", "--show-current"], capture_output=True, text=True).stdout.strip()
    files = sorted(p for p in out.glob(f"{Path(stem).name}*") if not p.name.endswith(".meta.txt"))
    meta = [f"SYS1-005 kill-chain bench recording  {time.strftime('%Y-%m-%d %H:%M:%S')}",
            f"firmware {fw}  branch {branch}  head {git}  (working tree may be uncommitted)",
            f"setpoint M1 = {args.setpoint}  silence = {args.silence} s  log rate = {args.rate} Hz  analyzer {args.nano}",
            "", "METRICS", json.dumps(metrics, indent=1), "", "CAPTURES", json.dumps(caps, indent=1), "", "FILES (sha256[:16])"]
    meta += [f"{sha16(p)}  {p.name}" for p in files]
    Path(f"{stem}.meta.txt").write_text("\n".join(meta), encoding="utf-8")
    print("\n".join(meta))


if __name__ == "__main__":
    main()
