# Board Diagnostics (XCP)

Firmware ab **v1.2.0**. Der Diagnose-Task läuft alle 100 ms auf CPU0,
vergleicht die Messwerte gegen kalibrierbare Grenzwerte und meldet
Verletzungen als 32-Bit-Bitmaske `diagStatus` im zyklischen XCP-Datenblock.
Die AurixGUI (Tab *Ethernet → Diagnose*) interpretiert die Bits
automatisch.

Hinweis zur Arbeitsteilung: harte Versorgungsfehler behandelt die PMS-Hardware
selbst (SMU-Alarm/Reset). Dieses Modul dient der **Beobachtung und
Frühwarnung** — es greift nicht ein.

## Statuswort `diagStatus` (uint32, little-endian)

Adresse: `Xcp_Data + 0x24` (Basis `0x70030000`, also `0x70030024`).
`0x00000000` = alles in Ordnung.

| Bit | Maske | Bedeutung | Vergleich |
|----:|---|---|---|
| 0 | `0x00000001` | DTS-Temperatur zu niedrig (PMS-Sensor) | `dieTempC < dtsMin` |
| 1 | `0x00000002` | DTS-Temperatur zu hoch (PMS-Sensor) | `dieTempC > dtsMax` |
| 2 | `0x00000004` | DTSC-Temperatur zu niedrig (SCU-Sensor) | `dtscTempC < dtscMin` |
| 3 | `0x00000008` | DTSC-Temperatur zu hoch (SCU-Sensor) | `dtscTempC > dtscMax` |
| 4 | `0x00000010` | VDD-Unterspannung (1,25-V-Kernspannung) | `vddCore < vddMin` |
| 5 | `0x00000020` | VDD-Überspannung | `vddCore > vddMax` |
| 6 | `0x00000040` | VDDP3-Unterspannung (3,3-V-I/O) | `vddp3 < vddp3Min` |
| 7 | `0x00000080` | VDDP3-Überspannung | `vddp3 > vddp3Max` |
| 8 | `0x00000100` | VEXT-Unterspannung (5-V-Boardversorgung) | `vext < vextMin` |
| 9 | `0x00000200` | VEXT-Überspannung | `vext > vextMax` |
| 10 | `0x00000400` | Temperatursensoren unplausibel | `|DTS − DTSC| > tempDeltaMax` |
| 11 | `0x00000800` | UART-Verbindung getrennt (kein Heartbeat, ab v1.4.0) | > 2 s kein RX-Byte |
| 31 | `0x80000000` | Kalibrierblock ungültig — Defaults wurden neu geladen | Magic-Wort zerstört |

Bits 12–30 sind reserviert (immer 0).

## UART-Heartbeat (Bit 11)

Die Firmware kann ein gezogenes USB-Kabel an X109 nicht direkt erkennen —
der USB-UART-Bridge-Baustein wird aus dem USB-Kabel versorgt. Ohne Kabel
zieht sein toter TX-Ausgang die RX-Leitung (P14.1) auf Low: ein Dauer-Break,
das den ASCLIN-Empfänger mit 0x00-Müllframes flutet. Deshalb sendet die
AurixGUI bei geöffnetem COM-Port alle 500 ms das Heartbeat-Byte **`'H'`
(0x48)**, und nur dieses Byte zählt als Lebenszeichen (ab v1.4.1 — v1.4.0
wertete jedes RX-Byte, wodurch das Break-Garbage die Erkennung aushebelte).

Die Firmware leert im 100-ms-Diagnose-Task das RX-FIFO; kommt länger als
**2 s** kein `'H'`, gilt die UART-Verbindung als getrennt und Bit 11 wird
(nach `debounceSec`) gesetzt. Jedes empfangene `'H'` löscht das Bit sofort —
im Terminalprogramm also `H` tippen.

Nach einem Reset startet der Zähler „getrennt“: Ohne GUI ist Bit 11 also
dauerhaft aktiv. Das ist gewollt — es zeigt, dass niemand am UART lauscht.

## Entprellung (Debounce)

Eine Verletzung muss **`debounceSec` Sekunden ununterbrochen** anliegen,
bevor das Bit gesetzt wird (Auflösung 0,1 s, z. B. `1.5`). Sobald der Wert
wieder im gültigen Bereich liegt, wird das Bit **sofort** gelöscht.
`debounceSec` wird firmwareseitig auf 0…60 s begrenzt; `0.0` = sofort melden.

## Kalibrierblock (per XCP schreibbar)

Basisadresse `0x70030100`, 64 Bytes, alle Werte `float32` little-endian.
Schreibzugriffe per XCP (`DOWNLOAD`/`SHORT_DOWNLOAD`) sind **nur** innerhalb
dieses Blocks erlaubt (ab Offset 0x04 — das Magic-Wort setzt nur die
Firmware). Alle anderen Adressen antworten mit `ERR_WRITE_PROTECTED (0x25)`.
Der Block liegt im RAM: Nach Reset gelten wieder die Defaults.

| Offset | Name | Einheit | Default | Herkunft |
|---:|---|---|---:|---|
| 0x00 | `magic` | – | `0x4C414358` | nur Firmware |
| 0x04 | `dtsMin` | °C | −40.0 | Betriebsbereich |
| 0x08 | `dtsMax` | °C | 105.0 | Warnschwelle unter Tj,max = 150 °C |
| 0x0C | `dtscMin` | °C | −40.0 | Betriebsbereich |
| 0x10 | `dtscMax` | °C | 105.0 | Warnschwelle |
| 0x14 | `vddMin` | V | 1.17 | Datenblatt-Betriebsbereich 1,25 V ± ~6 % |
| 0x18 | `vddMax` | V | 1.32 | |
| 0x1C | `vddp3Min` | V | 3.13 | Datenblatt 3,3 V ± 5 % |
| 0x20 | `vddp3Max` | V | 3.47 | |
| 0x24 | `vextMin` | V | 4.50 | 5-V-System |
| 0x28 | `vextMax` | V | 5.50 | |
| 0x2C | `tempDeltaMax` | K | 10.0 | Plausibilität DTS ↔ DTSC |
| 0x30 | `debounceSec` | s | 1.0 | Entprellzeit, 0,1-s-Auflösung |
| 0x34 | `fsVdd` | V | 1.455 | Monitor-ADC-Endwert, empirisch 2026-07-02 |
| 0x38 | `fsVddp3` | V | 3.825 | (Schienen auf Nominalwert angenommen) |
| 0x3C | `fsVext` | V | 5.903 | |

Die `fs*`-Werte skalieren die 8-Bit-Rohwerte in Volt
(`U = raw · fs / 255`) — eine Änderung wirkt direkt auf die Messwerte
*und* damit auf die Diagnose.

## Kalibrieren

**GUI:** Tab *Ethernet → Kalibrierung* — Werte lesen, editieren,
schreiben.

**pyXCP:**
```python
import struct
with Master("eth", config=CONF) as x:
    x.connect()
    x.setMta(0x70030100 + 0x14, 0)                  # vddMin
    x.download(struct.pack("<f", 1.20))
    x.disconnect()
```

## Validierungstrick

Zum Testen der Kette einen Grenzwert absichtlich verletzen, z. B.
`vddMin = 2.0` schreiben → nach `debounceSec` erscheint Bit 4
(VDD-Unterspannung). Wert zurückschreiben → Bit verschwindet sofort.
