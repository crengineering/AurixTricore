"""Disarmed bench check of the controller words, fw 1.19.33. NEVER writes motorCmd / motorManual."""
import struct
import sys
import time

sys.path.insert(0, r"C:\Users\chris\Projects\AurixTricore\tools")
import xcp_read  # noqa: E402

CAL = 0x70030100
ESC = 0x70030700
FUS = 0x70030500
DATA = 0x70030000


def rd(x, addr, fmt):
    n = struct.calcsize("<" + fmt)
    return struct.unpack("<" + fmt, xcp_read.short_upload_chunked(x, addr, n))


def wr(x, addr, fmt, *vals):
    x.setMta(addr, 0)
    x.download(struct.pack("<" + fmt, *vals))


def show(x, tag):
    state = rd(x, ESC + 0x6C, "B")[0]
    mode, thrust, tilt, rpmmin = rd(x, CAL + 0x54, "Ifff")
    sp = rd(x, CAL + 0x88, "3f")
    rpm = rd(x, ESC + 0xB0, "4f")
    tau = rd(x, ESC + 0xC0, "3f")
    om = rd(x, ESC + 0xCC, "3f")
    st, trips = rd(x, ESC + 0xE4, "2I")
    spf = rd(x, ESC + 0xEC, "3f")
    reason, modeEff = rd(x, ESC + 0xF8, "2I")
    phi = [v * 57.29578 for v in rd(x, FUS + 0xC0, "3f")]
    print(f"[{tag}] motorState={state} CAL mode={mode} thrust={thrust:.2f} tilt={tilt:.0f} rpmMin={rpmmin:.0f} "
          f"sp={sp[0]:.1f}/{sp[1]:.1f}/{sp[2]:.1f}")
    print(f"      att={phi[0]:.2f}/{phi[1]:.2f}/{phi[2]:.2f} deg  CtrlAttSp={spf[0]:.2f}/{spf[1]:.2f}/{spf[2]:.2f}  "
          f"om_soll={om[0]:.3f}/{om[1]:.3f}/{om[2]:.3f}  tau={tau[0]:.4f}/{tau[1]:.4f}/{tau[2]:.4f}")
    print(f"      rpm={rpm[0]:.0f}/{rpm[1]:.0f}/{rpm[2]:.0f}/{rpm[3]:.0f}  status={st} trips={trips} "
          f"tripReason={reason} CtrlMode={modeEff}")
    return state, mode, thrust, sp, st, trips, modeEff


with xcp_read.connect() as x:
    x.connect()
    ver = rd(x, DATA + 4, "3B")
    print("fw", ver)
    s = show(x, "boot state")
    assert s[0] == 0, "board is not DISARMED - stopping, nothing written"

    # words that only live while ARMED: write them disarmed, the firmware must take them back
    wr(x, CAL + 0x58, "f", 8.0)             # thrust
    wr(x, CAL + 0x88, "3f", 10.0, -5.0, 90.0)
    wr(x, CAL + 0x54, "I", 1)               # ctrlMode = drive (disarmed: must be refused)
    time.sleep(0.2)
    s = show(x, "after writing thrust 8, sp 10/-5/90, mode 1 while DISARMED")
    ok = (s[1] == 0 and s[2] == 0.0 and s[3][0] == 0.0 and s[3][1] == 0.0 and s[3][2] == -1.0 and s[6] == 0)
    print("RESET WHILE DISARMED:", "PASS" if ok else "FAIL")
    print("no trip counted for a refused engage:", "PASS" if s[5] == 0 else "FAIL (trips=%d)" % s[5])

    # gains are ordinary cal words: write, read back, restore
    kp = rd(x, CAL + 0x70, "3f")
    wr(x, CAL + 0x70, "f", 0.07)
    time.sleep(0.2)
    kp2 = rd(x, CAL + 0x70, "3f")
    tau = rd(x, ESC + 0xC0, "3f")
    om = rd(x, ESC + 0xCC, "3f")
    print(f"gain write: tauKp roll {kp[0]:.3f} -> {kp2[0]:.3f}; tau_x/om_x = {tau[0] / om[0] if om[0] else float('nan'):.3f}")
    wr(x, CAL + 0x70, "f", kp[0])
    time.sleep(0.1)
    print("gain restored:", rd(x, CAL + 0x70, "3f")[0])

    # load, a few windows
    for _ in range(5):
        pm = rd(x, DATA + 0x6C, "6H")
        print("load per mille core0/core1:", pm[0], pm[1])
        time.sleep(0.5)
    cs = rd(x, 0xB00F0000, "8I")
    print("core1 longest dispatch us:", cs[5], " core0:", cs[1])
    show(x, "end")
    x.disconnect()
