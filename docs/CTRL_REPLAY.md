# Flight-Controller Vector Replay

**ASPICE:** SWE.5/6 — SW integration/qualification harness (vector replay, UDP 5556) · realizes SYS2-VER-001, SYS2-IF-001 · process: QuadSE/requirements/README.md

Vektor-Replay-Test für den in Simulink verifizierten Kaskadenregler
(`flight_ctrl.c`) auf dem TC399: der Host schickt pro Reglertakt einen
Eingangsdatensatz per UDP, das Target ruft die vier Reglerstufen auf und
schickt die Ausgänge samt Ausführungszeit zurück.

## Dateien

| Datei | Rolle |
|---|---|
| `src/asw/flight_ctrl.c/.h` | **Byte-identische** Kopie der verifizierten Referenz aus `../Quadrocopter/sil/`. Keine Änderungen — nie editieren; bei Reglerupdates neu herüberkopieren. |
| `src/asw/CtrlReplay.c/.h` | Replay-Schicht: UDP-Server (Port 5556), Parametersatz, Zeitmessung |
| `src/bsw/SysTime.c/.h` | BSW-Zeitdienst (STM0-Ticks, 10 ns) mit iLLD-freier Schnittstelle |
| `tools/ctrl_replay_test.py` | Host-Client: Smoke-Test, CSV-Replay, Referenzvergleich |
| `test_replay.bat` | build.bat → flash.bat → Smoke-Test |

## Abweichungen / Entscheidungen

- **Frame-Größe:** Die Aufgabenstellung nannte „14 Floats" für den
  Eingangsdatensatz, die dort aufgezählten Felder ergeben aber 16 Floats
  (3+3+3+1+3+3) — und alle 16 werden von den Reglerfunktionen benötigt.
  Implementiert sind **16 Floats (64 Byte)**.
- **`flight_ctrl.c` unverändert:** Es waren keinerlei Anpassungen nötig
  (C99, float, `sqrtf` — alles vom TASKING-Compiler/der FPU abgedeckt).
  MISRA-Findings in dieser Datei werden über `tools/misra_baseline.txt`
  grandfathered statt gefixt, damit die Datei identisch zur verifizierten
  Referenz bleibt.
- **Parameter:** `ctrl_params_t` liegt als `const` in `CtrlReplay.c`, der
  Initialisierer kommt aus `src/asw/flight_ctrl_params.h` (derselbe Satz, den
  `FlightTask.c` als Basis nimmt) und ist ein Spiegel des Parametersatzes im
  Quadrocopter-Repo (`quadsim/params.py`, früher `quad_params.m` via
  `flight_ctrl_lct.c`). Bei Parameteränderungen dort muss dieser Block
  nachgezogen werden, sonst sind Vektorvergleiche wertlos.
- **Layering (dokumentierte Ausnahmen zu `src/asw/README.md`):**
  1. `Cpu0_Main.c` (BSW) ruft `CtrlReplay_init()` — einmaliger
     Init-Callout in die ASW.
  2. Die Replay-ASW läuft auf CPU0 statt CPU1–5, weil sie im
     lwIP-Poll-Kontext leben muss (NO_SYS=1, Stack gehört CPU0).
  3. `CtrlReplay.c` nutzt lwIP direkt (Library, kein iLLD); die
     iLLD-Zeitbasis ist über den BSW-Dienst `SysTime` gekapselt.
  4. `Cpu1_Main.c` (BSW) registriert `FlightTask_step()` (ASW) als
     Scheduler-Task hinter `NavTask_step` — zweiter BSW→ASW-Callout, seit
     fw 1.19.32 (Regler auf dem Motorpfad, `src/asw/FlightTask.h`). Die
     Gegenrichtung bleibt regelkonform: `FlightTask.c` ruft nur BSW
     (`NavState`, `CtrlLatch`, `Diagnostics`), die BSW liest das Ergebnis
     über `src/bsw/CtrlLatch.h`, nie über einen ASW-Header.

## Protokoll (UDP 5556, roh, alles little-endian)

| Frame | Request | Response |
|---|---|---|
| STEP | 64 B = 16×float32: `p_ned_soll[3], p_ned_ist[3], v_b_ist[3], psi_soll, phi_ist[3], om_ist[3]` | 72 B = 17×float32: `T_soll, phi_soll[3], om_soll[3], tau_soll[3], w_cmd[4], tau_I[3]` + uint32 `t_exec` [STM-Ticks, 10 ns] |
| RESET | 1 B: `0x00` | 1 B: `0x00` — Integrator **und** Zeitstatistik auf null |
| STATS | 1 B: `0x01` | 16 B: uint32 `count`, `min`, `max` [Ticks], float32 `mean` [Ticks] |
| sonstiges | — | 1 B: `0xEE` (NAK) |

Der Integratorzustand (`ctrl_state_t`) bleibt zwischen STEP-Frames
erhalten. Der Host muss lockstep fahren (senden → Antwort abwarten):
UDP garantiert keine Reihenfolge, und ein wiederholter STEP würde den
PI-Integrator doppelt fortschreiben — deshalb bricht das Python-Tool bei
Timeout ab statt still zu wiederholen.

Die Zeitmessung (`SysTime_getTicks`, STM0 @ 100 MHz) umschließt exakt die
vier Reglerauf­rufe — ohne UDP-Empfang, Deserialisierung oder Versand.
Budget: 1000 µs (Ts = 1 ms).

## Testablauf

```bat
test_replay.bat            :: inkrementell bauen, flashen, Smoke-Test
test_replay.bat clean      :: dito mit Clean-Build (nach Header-Änderungen)

:: Vektorlauf gegen Simulink-Referenz:
python tools\ctrl_replay_test.py --csv in.csv --out out.csv ^
                                 --expect ref.csv --tol 1e-4
```

CSV-Formate: Eingang 16 Spalten, Referenz 17 Spalten (Reihenfolge wie im
Protokoll), optionale Headerzeile. `--out` schreibt 17 Ausgänge +
`t_exec_us` pro Takt. Exit-Code 0 = bestanden (inkl. Budget- und
Toleranzprüfung), 1 = Fehler.

Der Smoke-Test prüft handrechenbare Punkte: Hover (T = m·g = 11,772 N,
w_cmd ≈ 767,2 rad/s auf allen Motoren), Höhensprung (+1 m zu tief →
T = 14,472 N), Integrator-Bewegung und -Reset, danach 1000 Takte für die
Timing-Statistik.
