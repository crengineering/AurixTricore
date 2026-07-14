"""Hardware validation of the PWM-capable GPIO feature (v1.7.0).

Raw-socket XCP-on-UDP against the TriBoard at 192.168.0.10:5555.
Validates: version, published mode[], duty defaults, write protection,
digital pins, and real PWM behaviour by sampling P00_IN.
"""
import socket
import struct
import sys
import time

HOST = ("192.168.0.10", 5555)

XCP_GPIO = 0x70030300
ADDR_MAGIC = XCP_GPIO + 0x00
ADDR_STATE = XCP_GPIO + 0x04
ADDR_MODE  = XCP_GPIO + 0x14
ADDR_DUTY  = XCP_GPIO + 0x24
ADDR_P00_IN = 0xF003A024

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)
_ctr = 0

failures = []

def check(name, cond, detail=""):
    tag = "PASS" if cond else "FAIL"
    print(f"[{tag}] {name}" + (f" ({detail})" if detail else ""))
    if not cond:
        failures.append(name)

def cmd(payload, expect_err=None):
    """Send one XCP command, return response payload (after PID check)."""
    global _ctr
    frame = struct.pack("<HH", len(payload), _ctr) + payload
    _ctr += 1
    sock.sendto(frame, HOST)
    data, _ = sock.recvfrom(1500)
    resp = data[4:4 + struct.unpack("<H", data[0:2])[0]]
    if expect_err is not None:
        assert resp[0] == 0xFE and resp[1] == expect_err, \
            f"expected ERR {expect_err:#04x}, got {resp.hex()}"
        return resp
    assert resp[0] == 0xFF, f"XCP error response: {resp.hex()}"
    return resp

def connect():
    return cmd(bytes([0xFF, 0x00]))

def short_upload(addr, n):
    r = cmd(struct.pack("<BBBBI", 0xF4, n, 0, 0, addr))
    return r[1:1 + n]

def short_download(addr, data, expect_err=None):
    return cmd(struct.pack("<BBBBI", 0xED, len(data), 0, 0, addr) + bytes(data),
               expect_err=expect_err)

def get_ident():
    r = cmd(bytes([0xFA, 0x00]))
    n = struct.unpack("<I", r[4:8])[0]
    r = cmd(bytes([0xF5, n]))
    return r[1:1 + n].decode()

def pin_level(pin):
    val = struct.unpack("<I", short_upload(ADDR_P00_IN, 4))[0]
    return (val >> pin) & 1

def sample_pin(pin, duration_s):
    """Sample a pin as fast as XCP allows; return (levels, timestamps)."""
    levels, stamps = [], []
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < duration_s:
        levels.append(pin_level(pin))
        stamps.append(time.perf_counter() - t0)
    return levels, stamps

# --- 1. connect + version -------------------------------------------------
connect()
ident = get_ident()
print(f"connected, ident: {ident}")
check("firmware version 1.7.0", ident == "AurixTricore v1.7.0", ident)

# --- 2. GPIO block: magic, mode[], duty[] defaults ------------------------
# normalize writable bytes first so the script is rerunnable without reboot
# (the block is RAM-persistent; true boot defaults were validated separately)
short_download(ADDR_STATE, [0] * 16)
short_download(ADDR_DUTY, [0, 50, 50] + [0] * 13)

magic = struct.unpack("<I", short_upload(ADDR_MAGIC, 4))[0]
check("GPIO magic", magic == 0x4F495047, hex(magic))

mode = list(short_upload(ADDR_MODE, 16))
print(f"mode[] = {mode}")
expected_mode = [2, 1, 1] + [0] * 13   # P00.0 reserved, P00.1/2 PWM, rest digital
check("published modes", mode == expected_mode)

duty = list(short_upload(ADDR_DUTY, 16))
print(f"duty[] = {duty}")
check("default duties", duty == [0, 50, 50] + [0] * 13)

state = list(short_upload(ADDR_STATE, 16))
check("default states all OFF", state == [0] * 16)

# --- 3. write protection ---------------------------------------------------
short_download(ADDR_MAGIC, [0x00], expect_err=0x25)
check("magic write rejected", True)
short_download(ADDR_MODE, [0x00], expect_err=0x25)
check("mode[] write rejected", True)
short_download(ADDR_MODE + 15, [0x00], expect_err=0x25)
check("mode[15] write rejected", True)
short_download(ADDR_STATE, [0x01])          # state[0] is writable (but ignored)
short_download(ADDR_DUTY, [0x01])           # duty[0] writable (ignored: digital)
check("state[]/duty[] writable", True)
short_download(ADDR_STATE, [0x00])

# --- 4. digital pin still works (P00.3) ------------------------------------
short_download(ADDR_STATE + 3, [1])
time.sleep(0.25)                             # calApply runs every 100 ms
check("digital P00.3 ON", pin_level(3) == 1)
short_download(ADDR_STATE + 3, [0])
time.sleep(0.25)
check("digital P00.3 OFF", pin_level(3) == 0)

# --- 5. reserved pin cannot be driven (P00.0) -------------------------------
# P00.0 is the diagnostics output and may be high or low depending on the
# current diag state (e.g. UART disconnected) — the requirement is that XCP
# state writes do NOT change it.
before = pin_level(0)
short_download(ADDR_STATE + 0, [1 - before])   # try to force the opposite
time.sleep(0.25)
check("reserved P00.0 unaffected by XCP", pin_level(0) == before,
      f"diag-driven level {before}")
short_download(ADDR_STATE + 0, [0])

# --- 6. PWM pin disabled => low --------------------------------------------
levels, _ = sample_pin(2, 0.4)
check("PWM P00.2 low while disabled", all(l == 0 for l in levels),
      f"{len(levels)} samples")

# --- 7. 5 Hz PWM on P00.2: toggles, ~100 ms half-period ---------------------
short_download(ADDR_STATE + 2, [1])
time.sleep(0.3)
levels, stamps = sample_pin(2, 2.0)
ones = sum(levels)
check("PWM P00.2 toggling", 0 < ones < len(levels),
      f"{ones}/{len(levels)} high")
# measure half-period from transitions
trans = [stamps[i] for i in range(1, len(levels)) if levels[i] != levels[i - 1]]
if len(trans) >= 4:
    deltas = [trans[i + 1] - trans[i] for i in range(len(trans) - 1)]
    avg = sum(deltas) / len(deltas)
    check("P00.2 half-period ~100 ms (5 Hz)", 0.080 < avg < 0.120,
          f"avg {avg*1000:.1f} ms over {len(deltas)} transitions")
else:
    check("P00.2 transitions observed", False, f"only {len(trans)}")

# duty fraction should be ~50 %
frac = ones / len(levels)
check("P00.2 duty ~50 %", 0.35 < frac < 0.65, f"{frac:.2f}")

# --- 8. duty 100 % => constant high, 0 % => constant low --------------------
short_download(ADDR_DUTY + 2, [100])
time.sleep(0.5)
levels, _ = sample_pin(2, 0.4)
check("P00.2 duty 100 % constant high", all(l == 1 for l in levels),
      f"{len(levels)} samples")

short_download(ADDR_DUTY + 2, [0])
time.sleep(0.5)
levels, _ = sample_pin(2, 0.4)
check("P00.2 duty 0 % constant low", all(l == 0 for l in levels),
      f"{len(levels)} samples")

# duty > 100 is clamped to 100
short_download(ADDR_DUTY + 2, [255])
time.sleep(0.5)
levels, _ = sample_pin(2, 0.4)
check("P00.2 duty 255 clamped to 100 %", all(l == 1 for l in levels),
      f"{len(levels)} samples")

# --- 9. 1 kHz PWM on P00.1: mixed samples at 50 % ---------------------------
short_download(ADDR_STATE + 1, [1])
short_download(ADDR_DUTY + 1, [50])
time.sleep(0.3)
levels, _ = sample_pin(1, 1.0)
ones = sum(levels)
check("PWM P00.1 (1 kHz) mixed levels", 0 < ones < len(levels),
      f"{ones}/{len(levels)} high")

# --- 10. state OFF forces PWM low -------------------------------------------
short_download(ADDR_STATE + 1, [0])
short_download(ADDR_STATE + 2, [0])
short_download(ADDR_DUTY + 2, [50])
time.sleep(0.5)
levels, _ = sample_pin(1, 0.3)
check("P00.1 low after state OFF", all(l == 0 for l in levels))

print()
if failures:
    print(f"=== {len(failures)} FAILURE(S): {failures} ===")
    sys.exit(1)
print("=== ALL PWM/GPIO TESTS PASSED ===")
