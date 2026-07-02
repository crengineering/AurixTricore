"""Milestone 6 validation: pyXCP master against the TC399 XCP slave."""
import time
from pyxcp.master import Master
from pyxcp.config import create_application_from_config

TICK_ADDR = 0x70000170  # g_TickCount_1ms (from AurixTricore.map)

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

with Master("eth", config=CONF) as x:
    res = x.connect()
    print("CONNECT ok")
    print("  maxCto:", res.maxCto, " maxDto:", res.maxDto)
    print("  byteOrder:", res.commModeBasic.byteOrder)

    status = x.getStatus()
    print("GET_STATUS ok:", status.sessionStatus)

    raw1 = x.shortUpload(4, TICK_ADDR, 0)
    t1 = int.from_bytes(bytes(raw1), "little")
    time.sleep(0.5)
    raw2 = x.shortUpload(4, TICK_ADDR, 0)
    t2 = int.from_bytes(bytes(raw2), "little")
    print(f"SHORT_UPLOAD g_TickCount_1ms: {t1} -> {t2} (delta {t2 - t1} in ~500 ms)")

    x.setMta(TICK_ADDR, 0)
    raw3 = x.upload(4)
    print("SET_MTA + UPLOAD ok:", int.from_bytes(bytes(raw3), "little"))

    x.disconnect()
    print("DISCONNECT ok")

    assert t2 > t1, "tick counter did not advance!"
    print("\nMILESTONE 6 VALIDATION PASSED")
