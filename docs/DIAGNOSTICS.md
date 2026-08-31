# Board Diagnostics (XCP)

**ASPICE:** SWE.3 — detailed design, diagnostics + cal blocks · baseline — parent TBD (serves SYS1-008 groundstation visibility) · process: QuadSE/requirements/README.md

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
| 12 | `0x00001000` | NVM-Fehler (ab v1.5.0) — DFLASH-Datensatz beim Boot korrupt oder letztes `SAVE` fehlgeschlagen | wird durch erfolgreiches `SAVE` gelöscht |
| 13 | `0x00002000` | I2C0 **SCL dauerhaft LOW** (ab v1.13.0) — Kurzschluss nach GND oder Slave hängt in Clock-Stretching | > 0,3 s bei ruhendem Bus |
| 14 | `0x00004000` | I2C0 **SDA dauerhaft LOW** — Kurzschluss nach GND oder Slave mitten im Byte blockiert | > 0,3 s bei ruhendem Bus |
| 15 | `0x00008000` | **BMP581 antwortet nie** seit dem Start — nicht verdrahtet, falsche Adresse oder defekt (Leitungsbruch) | kein ACK seit Boot |
| 16 | `0x00010000` | **BMP581 Kommunikations-Timeout** — hat geantwortet, ist jetzt still (Leitung ab, Stecker locker) | > 1 s kein erfolgreicher Read |
| 17 | `0x00020000` | **BMP581 Daten eingefroren** — Bus gesund, Wert ändert sich nicht mehr | > 5 s bit-identischer Wert |
| 18 | `0x00040000` | **BMP581 unplausibel** — Druck außerhalb 300–1200 hPa oder Temperatur außerhalb −40…85 °C | > 1 s außerhalb |
| 19 | `0x00080000` | **IMU antwortet nie** seit dem Start | kein ACK seit Boot |
| 20 | `0x00100000` | **IMU Kommunikations-Timeout** | > 1 s kein erfolgreicher Read |
| 21 | `0x00200000` | **IMU Daten eingefroren** — genau der Fehler vom 30.07.2026 (Register standen still, Bus meldete OK) | > 5 s bit-identischer Wert |
| 22 | `0x00400000` | **IMU unplausibel** — \|a\| außerhalb 0,05…13 g oder Temperatur außerhalb −40…85 °C | > 1 s außerhalb |
| 23 | `0x00800000` | **MMC5983MA antwortet nie** seit dem Start (ab v1.15.0) — Verdrahtung, oder **CS nicht auf +3V3 gelegt** (häufigste Ursache, siehe MMC5983MA.md §3) | kein ACK seit Boot |
| 24 | `0x01000000` | **MMC5983MA Kommunikations-Timeout** | > 1 s kein erfolgreicher Read |
| 25 | `0x02000000` | **MMC5983MA Daten eingefroren** | > 5 s bit-identischer Wert |
| 26 | `0x04000000` | **MMC5983MA unplausibel** — \|B\| außerhalb 0,15…2,0 G. Das ist der schärfste Test im ganzen Wort: \|B\| ist eine Eigenschaft des **Ortes**, nicht der Lage, muss also beim Drehen des Boards konstant bleiben (~0,48 G in München). Ein Wert, der um einen glatten Faktor daneben liegt, bedeutet falsche Skalierung oder falsch zusammengesetztes 18-Bit-Wort | > 1 s außerhalb |
| 27 | `0x08000000` | **GNSS antwortet nie** (`DIAG_GNSS_NO_RESPONSE`, `Diagnostics.h:95`) | s. `Diagnostics.c` |
| 28 | `0x10000000` | **GNSS Kommunikations-Timeout** (`DIAG_GNSS_TIMEOUT`) | s. `Diagnostics.c` |
| 29 | `0x20000000` | **GNSS Daten eingefroren** (`DIAG_GNSS_STUCK_DATA`) | s. `Diagnostics.c` |
| 30 | `0x40000000` | **GNSS unplausibel** (`DIAG_GNSS_IMPLAUSIBLE`) | s. `Diagnostics.c` |
| 31 | `0x80000000` | Kalibrierblock ungültig — Defaults wurden neu geladen | Magic-Wort zerstört |

## Resource budgets (SSoT — korrigiert 2026-08-29)

Diese Tabelle ist die **einzige** maßgebliche Buchführung; andere Dokumente
verweisen hierher statt eigene Stände zu führen.

| Ressource | Stand | Konsequenz |
|---|---|---|
| `diagStatus` | **VOLL — 32/32 Bits belegt** (0–26 Board+Peripherie, 27–30 GNSS seit der NEO-M9N-Integration, 31 Cal) | Das **nächste** Gerät (Flight-IMU-Slots, 4× ESC-Telemetrie) erzwingt die Migration der gerätespezifischen Fehler in ein Status-Array pro Peripherie (`PeriphDiag_Id`-indiziert); `diagStatus` bleibt dann für Board-Fehler. Ändert `Xcp_Data`, A2L und die `BIT_MASK`-Zeilen der GUI gemeinsam |
| `Xcp_Data` | letztes Feld endet **`0xF8`** → **8 Bytes frei** vor `Xcp_Cal` @`0x…100` (IMU_INTERRUPT.md §, REFACTORING_PLAN.md §) | praktisch voll — jedes echte Gerät braucht mehr; Erweiterung heißt Block-Umbau per CODEMAP |

*(Der frühere Hinweis an dieser Stelle — „Bits 27–30 sind die letzten vier
freien" — war seit der GNSS-Integration veraltet; ebenso ist der Kommentar
`Diagnostics.h:101` „Still to come are the GNSS" stale.)*

> **Ab v1.14.0:** Der Barometer-Slot (Bits 15–18) wird vom **BMP581** (I2C0
> `0x47`) bedient — die Bitbelegung ist unverändert. Der **IMU-Slot (Bits
> 19–22) ist stillgelegt**: seit dem 31.07.2026 ist kein IMU verbaut, und der
> Slot wird per `PeriphDiag_setFitted(PERIPH_DIAG_IMU, FALSE)` als *nicht
> bestückt* deklariert. Ein nicht bestücktes Peripheriegerät setzt **keine
> Bits** — sonst stünde `IMU antwortet nie` dauerhaft an und ein permanent
> rotes Diagnosewort liest niemand mehr. Die Bits kommen zurück, sobald der
> ICM-42688-P (QSPI0) bestückt und als bestückt deklariert wird.

### Peripherie-Diagnose lesen (ab v1.13.0)

Die vier Fehlerarten je Gerät sind absichtlich getrennt, weil sie
unterschiedliche Reparaturen bedeuten:

* **NO_RESPONSE** — hat seit dem Boot nie geantwortet: Verdrahtung, Adresse
  oder Bauteil defekt. Bewusst getrennt von TIMEOUT, damit „hat nie
  funktioniert" nicht mit „lief und ist ausgefallen" verwechselt wird.
* **TIMEOUT** — lief und ist jetzt still: Leitung ab, Stecker locker,
  Spannungseinbruch.
* **STUCK_DATA** — Bus in Ordnung, Werte stehen still. Eigenes Bit, weil genau
  dieser Fall am 30.07.2026 tagelang unentdeckt blieb: der MPU-6050 hat jeden
  Read sauber quittiert, während seine Sensorregister eingefroren waren.
  Weder Busstatus noch Present-Flag zeigen so etwas an.
* **IMPLAUSIBLE** — antwortet und ändert sich, liegt aber außerhalb des
  physikalisch Möglichen: Skalierung, Kalibrierung oder Messelement defekt.

> ⚠️ **Beim GNSS (Bit 30) bedeutet IMPLAUSIBLE etwas anderes:** „der Empfänger
> hat nicht jedes Konfigurationskommando quittiert“ (`cfgOk`, siehe
> `Cpu0_Main.c` Task_Measure100ms). Grund: eine abgelehnte `CFG-VALSET` ändert
> **nichts** Sichtbares auf der Leitung — ein still unkonfigurierter Empfänger
> sähe kerngesund aus. Das Bit ist *nicht* an `navOk` gekoppelt: „kein
> Satellit“ ist der normale Zustand in Innenräumen, kein Fehler. Ein dauerhaft
> rotes Diagnosewort liest niemand mehr. Details: `docs/GNSS_UBX.md`.

**Kurzschluss vs. Leitungsbruch:** Bei ruhendem Bus müssen beide Leitungen über
die Pull-ups HIGH sein. Eine Leitung dauerhaft LOW (Bit 13/14) ist ein
Kurzschluss nach GND oder ein blockierender Slave. Ein **Leitungsbruch** sieht
genau umgekehrt aus — Bus perfekt im Leerlauf, aber niemand quittiert, also
NO_RESPONSE bzw. TIMEOUT ohne Bus-Bit. Sind **beide** Sensoren betroffen,
liegt es an der gemeinsamen Verdrahtung; ist nur einer betroffen, an dessen
eigener Leitung.

Die Zeitgrenzen sind Konstanten in `PeriphDiag.c` und nicht kalibrierbar — der
64-Byte-Kalibrierblock ist voll (siehe `Nvm.h` für den gleichen Grund beim
QNH-Parameter).

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
dieses Blocks und des NVM-Blocks (s. u.) erlaubt (jeweils ab Offset 0x04 —
die Magic-Wörter setzt nur die Firmware). Alle anderen Adressen antworten
mit `ERR_WRITE_PROTECTED (0x25)`. ⚠️ pyXCP nennt denselben Code
`ERR_ACCESS_LOCKED` in seiner eigenen Fehlertabelle — gleicher Wert `0x25`,
andere Bezeichnung; nicht mit einem zweiten Fehler verwechseln (gefunden bei
der T5-Verifikation auf Hardware, docs/MEMORY_PLACEMENT.md).
Der Block liegt **nur im RAM**: Nach einem Reset gelten wieder die
Defaults. Persistente Parameter leben strikt getrennt im NVM-Block.

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

## NVM-Block: persistente Parameter (ab v1.6.0)

Strikt getrennt vom Kalibrierblock gibt es den **persistenten
Parameterblock** `Xcp_Nvm` an Basisadresse `0x70030200` (44 Bytes,
little-endian, per XCP ab Offset 0x04 schreibbar). **Nur** dieser Block
wird im on-chip DFLASH gespeichert (EEPROM-Emulation, zwei 4-KB-Sektoren
ab `0xAF000000` im Ping-Pong-Verfahren — ein Stromausfall während des
Speicherns kann den letzten gültigen Datensatz nie zerstören). Jeder
Datensatz trägt Magic, Layout-Version, Sequenznummer und CRC-32; beim
Boot gewinnt der neueste gültige Datensatz, sonst gelten die Defaults.

| Offset | Name | Default | Bedeutung |
|---:|---|---:|---|
| 0x00 | `magic` | `0x4D564E58` | nur Firmware ("XNVM") |
| 0x04 | `command` | 0 | NVM-Kommando (s. u.) |
| 0x08 | `userValue` | 0 | erster persistenter Parameter (freies `uint32`) |
| 0x0C | `seaLevelPa` | 101325 | Referenzdruck auf Meereshöhe (QNH) für die Baro-Höhe |
| 0x10 | `magOffX` | 0.0 | Hard-Iron-Offset X [Gauss], **Sensorachsen** |
| 0x14 | `magOffY` | 0.0 | Hard-Iron-Offset Y |
| 0x18 | `magOffZ` | 0.0 | Hard-Iron-Offset Z |
| 0x1C | `magScaleX` | 1.0 | Soft-Iron-Skalierung X (dimensionslos) |
| 0x20 | `magScaleY` | 1.0 | Soft-Iron-Skalierung Y |
| 0x24 | `magScaleZ` | 1.0 | Soft-Iron-Skalierung Z |
| 0x28 | `magDeclDeg` | 3.9 | Missweisung [Grad, Ost positiv] — macht aus magnetisch Nord **rechtweisend** Nord |

Die Magnetometer-Kalibrierung (ab v1.19.x, Layout-Version 4) ist
**boardspezifisch**, nicht designspezifisch: die Offsets sind die Magnetik der
Platine selbst, wie der MMC5983MA sie sieht. Deshalb gehören sie in den Flash
und nicht in einen Header. Alle Werte sind so vorbelegt, dass sie **nichts
verändern** (Offset 0, Skalierung 1) — ein unkalibriertes Board verhält sich
exakt wie vor Einführung der Felder. Ermittelt werden sie mit
`tools/mag_cal.py` (siehe `docs/FUSION.md` §4).

Neue persistente Parameter werden hier angehängt (Layout-Version in
`Nvm.c` erhöhen; alte Datensätze werden dann ignoriert, kein Fehler).
⚠️ Die CRC deckt den gesamten Block ab, eine Größenänderung entwertet also
gespeicherte Datensätze — genau dafür ist die Layout-Version da.

Gesteuert wird über das `command`-Wort (`0x70030204`):

| Wert | ASCII | Wirkung |
|---|---|---|
| `0x45564153` | `SAVE` | NVM-Block in den DFLASH speichern |
| `0x544C4644` | `DFLT` | NVM-Defaults ins RAM laden (speichert **nicht**) |

Die Firmware führt das Kommando im 100-ms-Task aus und setzt das Wort
danach auf `0` zurück (Handshake fürs Tool). Erfolg/Misserfolg zeigt
Diagnose-Bit 12: gesetzt = Datensatz beim Boot korrupt oder letztes
`SAVE` fehlgeschlagen; ein erfolgreiches `SAVE` löscht es. Ein leerer
(fabrikneuer) DFLASH ist **kein** Fehler.

Zurück zu den Defaults dauerhaft: erst `DFLT`, dann `SAVE`.
Validierung (inkl. Nachweis der RAM/NVM-Trennung):
`python tools/nvm_test.py` (siehe Skript-Hilfe).

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
