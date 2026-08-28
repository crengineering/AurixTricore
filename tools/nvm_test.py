"""NVM persistence validation (SW >= v1.6.0): pyXCP master vs. DFLASH storage.

The persistent parameter block (Xcp_Nvm @ 0x70030200) is strictly separated
from the RAM-only calibration block. The test also proves the separation:
a calibration value written before the save must NOT survive the reset.

Usage:
    python tools/nvm_test.py save      # write markers + SAVE command
    <power-cycle or reset the board>
    python tools/nvm_test.py verify    # NVM marker survived, cal marker not
    python tools/nvm_test.py restore   # DFLT + SAVE: persist the defaults again

⚠️ Between `save` and `verify` the board must be RESET, not RE-FLASHED with
`flash.bat` -- that script passes `-erase on`, which wipes DFLASH and takes
the record you just wrote with it, so `verify` fails for that reason alone
on freshly-erased flash (found doing this, docs/MEMORY_PLACEMENT.md T5). A
reset that does NOT erase:
    AURIXFlasher.exe -hex <hex> -erase off -prog off -ver off
(connects by reset&halt, starts by reset, no programming step -- ~450 ms).
"""
import struct
import sys
import time

from pyxcp.master import Master
from pyxcp.config import create_application_from_config

CAL_BASE       = 0x70030100
CAL_DTSMIN     = CAL_BASE + 0x04    # float32, RAM only — must NOT persist
NVM_BASE       = 0x70030200
NVM_COMMAND    = NVM_BASE + 0x04    # uint32 command word (handshake)
NVM_USERVALUE  = NVM_BASE + 0x08    # uint32, persistent marker target
DIAG_STATUS    = 0x70030024         # uint32, bit 12 = NVM fault
DIAG_NVM_FAULT = 1 << 12

CMD_SAVE       = 0x45564153         # "SAVE"
CMD_DEFAULTS   = 0x544C4644         # "DFLT"

NVM_MARKER     = 0xC0FFEE01         # distinctive userValue (default is 0)
CAL_MARKER     = -39.5              # distinctive dtsMin (default is -40.0)

CONF = create_application_from_config({
    "Transport": {
        "Eth": {
            "host": "192.168.0.10",
            "port": 5555,
            "protocol": "UDP",
            "ipv6": False,
        },
    },
})


def read_u32(x, addr):
    return int.from_bytes(bytes(x.shortUpload(4, addr, 0)), "little")


def read_f32(x, addr):
    return struct.unpack("<f", bytes(x.shortUpload(4, addr, 0)))[0]


def write_u32(x, addr, value):
    x.setMta(addr, 0)
    x.download(struct.pack("<I", value))


def write_f32(x, addr, value):
    x.setMta(addr, 0)
    x.download(struct.pack("<f", value))


def run_command(x, cmd, name):
    """Write an NVM command and wait for the firmware handshake (word -> 0)."""
    write_u32(x, NVM_COMMAND, cmd)
    deadline = time.time() + 2.0
    while time.time() < deadline:
        if read_u32(x, NVM_COMMAND) == 0:
            print(f"{name} acknowledged (command word cleared)")
            return
        time.sleep(0.05)
    raise SystemExit(f"FAIL: {name} not acknowledged within 2 s")


def check_no_fault(x):
    diag = read_u32(x, DIAG_STATUS)
    if diag & DIAG_NVM_FAULT:
        raise SystemExit(f"FAIL: DIAG_NVM_FAULT set (diagStatus=0x{diag:08X})")
    print(f"diagStatus ok (0x{diag:08X}, bit 12 clear)")


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("save", "verify", "restore"):
        raise SystemExit(__doc__)
    mode = sys.argv[1]

    with Master("eth", config=CONF) as x:
        x.connect()

        if mode == "save":
            print(f"userValue before: 0x{read_u32(x, NVM_USERVALUE):08X}")
            write_u32(x, NVM_USERVALUE, NVM_MARKER)
            write_f32(x, CAL_DTSMIN, CAL_MARKER)    # RAM-only control sample
            run_command(x, CMD_SAVE, "SAVE")
            check_no_fault(x)
            print(f"\nNVM marker userValue=0x{NVM_MARKER:08X} persisted, "
                  f"cal marker dtsMin={CAL_MARKER} in RAM only."
                  "\nPower-cycle the board, then run: python tools/nvm_test.py verify")

        elif mode == "verify":
            user = read_u32(x, NVM_USERVALUE)
            dts  = read_f32(x, CAL_DTSMIN)
            print(f"userValue after reset: 0x{user:08X} (expected 0x{NVM_MARKER:08X})")
            print(f"dtsMin    after reset: {dts:.2f} (expected -40.00 = default)")
            check_no_fault(x)
            if user != NVM_MARKER:
                raise SystemExit("FAIL: NVM marker did not survive the reset")
            if abs(dts - (-40.0)) > 1e-3:
                raise SystemExit("FAIL: cal value persisted although it is RAM-only")
            print("\nNVM PERSISTENCE + RAM/NVM SEPARATION VALIDATION PASSED"
                  "\nRestore defaults with: python tools/nvm_test.py restore")

        else:  # restore
            run_command(x, CMD_DEFAULTS, "DFLT")
            run_command(x, CMD_SAVE, "SAVE")
            check_no_fault(x)
            print(f"defaults restored and persisted "
                  f"(userValue=0x{read_u32(x, NVM_USERVALUE):08X})")

        x.disconnect()


if __name__ == "__main__":
    main()
