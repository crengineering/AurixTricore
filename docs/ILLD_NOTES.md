# iLLD Notes — TC39x

Distilled reference for the iLLD subset this project actually uses. **Read this
instead of grepping `Libraries/iLLD/` (801 files).** Only open vendor source when
this file has a gap — and when you do, write the answer back here.

Vendor root: `Libraries/iLLD/TC3xx/Tricore/<Module>/`
Build exclusion (`.cproject`): `SCR|MCS|HSM` only — everything else compiles.

---

## 0. Traps index — read this first

Each of these cost real debugging time on this project. They are not in the vendor docs.

| # | Module | Trap |
|---|---|---|
| T1 | Port | `IfxPort_State` is an **OMR bitmask, not a boolean**. `FALSE`/`0` = *leave unchanged*. |
| T2 | I2C | `initConfig()` hardcodes `stopOnPacketEnd = 0` → master **never emits STOP**. |
| T3 | I2C | `IfxI2c_I2c_deviceConfig.enableRepeatedStart` is **dead code** — stored, never read. |
| T4 | I2C | `read2()`/`write2()` spin **unbounded** — a stuck slave hangs the core forever. |
| T5 | I2C | `initModule()` does **not** reset the kernel/FIFOs. Re-init after a slave brown-out does nothing. |
| T6 | STM | Each core must use **its own** `MODULE_STMn`. |
| T7 | Qspi | `exchange()` is **async**; poll `getStatus()` before touching the RX buffer. |
| T8 | All | Unsuffixed vendor constants trip cppcheck/MISRA — see `tools/misra_baseline.txt`. |
| T9 | Build | Header-only changes leave stale `.src` — delete `.src` + `.o` + `.d`, not just `.o`. |
| T10 | ScuEru | Only 18 pads reach the ERU on the 516 package, and this board commits most of them. |
| T11 | ScuEru | Only **OGU0-3** have an SRC node; `OutputChannel_4..7` / `InputNodePointer_4..7` are silently dead. |
| T12 | ScuEru | The **input** channel is fixed by the pad (`req->channelId`); `initReqPin` also sets the input mux. |
| T13 | ScuEru | The glitch filter (`SCU_EIFILT`) is **global to all ERU inputs**, not per channel. |
| T14 | Cpu | `Ifx__dsync` is **undefined for TASKING** in this tree — see §3. |

---

## 1. IfxPort — GPIO

`Port/Std/IfxPort.h` · all hot functions are `IFX_INLINE`.

```c
void     IfxPort_setPinState (Ifx_P *port, uint8 pin, IfxPort_State action);
boolean  IfxPort_getPinState (Ifx_P *port, uint8 pin);
void     IfxPort_setPinHigh  (Ifx_P *port, uint8 pin);   // = setPinState(..., _high)
void     IfxPort_setPinLow   (Ifx_P *port, uint8 pin);
void     IfxPort_togglePin   (Ifx_P *port, uint8 pin);
void     IfxPort_setPinMode         (Ifx_P *port, uint8 pin, IfxPort_Mode mode);
void     IfxPort_setPinModeOutput   (Ifx_P *port, uint8 pin, IfxPort_OutputMode, IfxPort_OutputIdx);
void     IfxPort_setPinModeInput    (Ifx_P *port, uint8 pin, IfxPort_InputMode);
void     IfxPort_setPinPadDriver    (Ifx_P *port, uint8 pin, IfxPort_PadDriver);
```

### T1 — the OMR encoding

```c
IFX_INLINE void IfxPort_setPinState(Ifx_P *port, uint8 pinIndex, IfxPort_State action)
{ port->OMR.U = action << pinIndex; }          // one register write, atomic, no RMW
```

`OMR` has **separate SET (bits 0-15) and CLEAR (bits 16-31) fields**, so the enum
is a two-bit mask, not a level:

| Enum | Value | Effect |
|---|---|---|
| `IfxPort_State_notChanged` | `(0<<16)\|(0<<0)` = 0 | **no change** |
| `IfxPort_State_high` | `(0<<16)\|(1<<0)` = 1 | drive high |
| `IfxPort_State_low` | `(1<<16)\|(0<<0)` | drive low |
| `IfxPort_State_toggled` | `(1<<16)\|(1<<0)` | toggle |

Passing `FALSE` (0) silently does nothing; `TRUE` (1) means *high*, not *on*.
Board LEDs are active-low → `LED_ON = IfxPort_State_low`.

**Pad self-test** (no meter needed, from the IMU bring-up): configure as output,
drive high then low, read back with `IfxPort_getPinState` after each. A pad
clamped to 5V/GND reads its clamped value regardless of what you drive. This is
how P22.7/P22.8 were proven unusable on this board. Always include an unwired
control pin to prove the method works.

---

## 2. IfxStm — system timer

`Stm/Std/IfxStm.h` · **~100 MHz after PLL init** → `50e6 ticks ≈ 500 ms`.

```c
uint32   IfxStm_getLower     (Ifx_STM *stm);            // = stm->TIM0.U, free-running 32-bit
void     IfxStm_waitTicks    (Ifx_STM *stm, uint32 ticks);   // blocking
float32  IfxStm_getFrequency (Ifx_STM *stm);
sint32   IfxStm_getTicksFromMilliseconds(Ifx_STM *stm, uint32 ms);
sint32   IfxStm_getTicksFromMicroseconds(Ifx_STM *stm, uint32 us);
boolean  IfxStm_initCompare       (Ifx_STM *stm, const IfxStm_CompareConfig *cfg);
void     IfxStm_initCompareConfig (IfxStm_CompareConfig *cfg);
void     IfxStm_increaseCompare   (Ifx_STM *stm, IfxStm_Comparator, uint32 ticks);
void     IfxStm_clearCompareFlag  (Ifx_STM *stm, IfxStm_Comparator);
```

**Wraparound is safe** if you subtract before comparing — unsigned 32-bit
arithmetic handles the rollover. This is the idiom `waitTicks` itself uses:

```c
uint32 begin = IfxStm_getLower(stm);
while ((IfxStm_getLower(stm) - begin) < ticks) {}   // correct across 0xFFFFFFFF
```

Never write `while (IfxStm_getLower(stm) < begin + ticks)` — that breaks on wrap.

**T6**: `MODULE_STM0`→CPU0 … `MODULE_STM5`→CPU5. Cross-core access works but adds
a bus dependency for nothing.

Beware `getTicksFromMilliseconds`: it computes `(freq/1000) * ms` — integer
division **first**, and returns `sint32`, so it overflows above ~21 s at 100 MHz.

---

## 3. IfxCpu — cores, sync, mutexes

```c
IfxCpu_Id IfxCpu_getCoreId(void);
void      IfxCpu_enableInterrupts(void);
boolean   IfxCpu_disableInterrupts(void);
void      IfxCpu_restoreInterrupts(boolean enabled);
boolean   IfxCpu_acquireMutex(IfxCpu_mutexLock *lock);   // TRUE = acquired
void      IfxCpu_releaseMutex(IfxCpu_mutexLock *lock);
void      IfxCpu_initCSA(uint32 *csaBegin, uint32 *csaEnd);
```

Sync barrier (`IfxCpu_Irq.h` / `Ifx_Cfg.h`): **every one of the six `coreN_main`
must call both** `IfxCpu_emitEvent(&cpuSyncEvent)` and
`IfxCpu_waitEvent(&cpuSyncEvent, timeout)` before any application code. Skipping
either on *any* core → watchdog reset at the timeout. Cores 4-5 sync then idle.

Cross-core data: **the real rule is LMU + non-cached alias (or LMU + `DSYNC`
per write), not "cross-core DSPR reads bypass the cache"** — that older claim
described the reader only, was never Infineon's actual mechanism, and must not
be extended to a new crossing. See `SharedRam.h` (`src/bsw/`) and
`docs/REFACTORING_PLAN.md` §2.4 for the vendor-sourced rule: exactly one
writer per object, every field `volatile` and 32-bit (LMU has no sub-word
write), and `store payload -> Ifx__dsync() -> publish` for anything with more
than one field a reader must see consistently.

### T14 — `Ifx__dsync` is not declared for TASKING

`IfxCpu_IntrinsicsDcc.h:1349-1351`, `...Gcc.h`, `...Gnuc.h` and
`...HighTec.h:354` all declare `Ifx__dsync` (mapping to the compiler's own
`__dsync`/`__builtin_tricore_dsync`). `IfxCpu_IntrinsicsTasking.h` has **no
`dsync` symbol at all**, and the project's build config is TriCore Debug
(TASKING). The TASKING compiler still recognises `__dsync()` as a built-in
intrinsic with no declaration needed — `IfxFlash.c`/`IfxFlash.h` already call
it bare and that TU builds today — so the fix is a one-line wrapper, not a
missing header:
```c
#ifndef Ifx__dsync
#define Ifx__dsync()   __dsync()
#endif
```
A missing barrier here is **silent**: it does not fail the build, it fails as
a rare torn cross-core snapshot under load. Never trust "it compiled" for this
one — inspect the generated `.src` for the actual `DSYNC` instruction.

---

## 4. IfxScuWdt — watchdogs

```c
uint16 IfxScuWdt_getCpuWatchdogPassword(void);
uint16 IfxScuWdt_getSafetyWatchdogPassword(void);
void   IfxScuWdt_disableCpuWatchdog(uint16 password);
void   IfxScuWdt_disableSafetyWatchdog(uint16 password);
void   IfxScuWdt_clearSafetyEndinit(uint16 password);   // unlock ENDINIT-protected SFRs
void   IfxScuWdt_setSafetyEndinit(uint16 password);     // re-lock — always pair
```

Both watchdogs must be disabled. CPU watchdog: **per core**. Safety watchdog:
**CPU0 only**. ENDINIT-protected registers (SCU, clock, some port config) need a
`clearSafetyEndinit` / `setSafetyEndinit` pair around the write — leaving it
cleared is itself a fault.

---

## 5. IfxI2c — the module with all the landmines

`I2c/I2c/IfxI2c_I2c.h` (driver) + `I2c/Std/IfxI2c.h` (register layer).

```c
void              IfxI2c_I2c_initConfig      (IfxI2c_I2c_Config *cfg, Ifx_I2C *i2c);
void              IfxI2c_I2c_initModule      (IfxI2c_I2c *i2c, const IfxI2c_I2c_Config *cfg);
void              IfxI2c_I2c_initDeviceConfig(IfxI2c_I2c_deviceConfig *cfg, IfxI2c_I2c *i2c);
void              IfxI2c_I2c_initDevice      (IfxI2c_I2c_Device *dev, const IfxI2c_I2c_deviceConfig *cfg);
IfxI2c_I2c_Status IfxI2c_I2c_read2 (IfxI2c_I2c_Device *dev, volatile uint8 *data, Ifx_SizeT size);
IfxI2c_I2c_Status IfxI2c_I2c_write2(IfxI2c_I2c_Device *dev, volatile uint8 *data, Ifx_SizeT size);
void              IfxI2c_resetModule(Ifx_I2C *i2c);
void              IfxI2c_resetFifo  (Ifx_I2C *i2c);
```

Device address goes in **shifted left by 1** (`addr << 1`).

### T2 — no STOP is ever generated

`IfxI2c_I2c.c:82` — `initConfig()` sets `addrFifoCfg.addressConfig.stopOnPacketEnd = 0`,
which lands in `ADDRCFG.B.SOPE` (`IfxI2c.c:347`). The master therefore holds the
bus after every transfer.

Consequences seen on this board: the MPU-6050 freezes its data registers
mid-burst and latches them **forever** (all-zero if the freeze happened after
`DEVICE_RESET`) — indistinguishable from a dead die. The BMP388 has no read-hold
behaviour, so it masked the bug entirely. Set `stopOnPacketEnd = 1` explicitly
after `initConfig()`.

### T3 — `enableRepeatedStart` is dead code

Present on both `IfxI2c_I2c_deviceConfig` and `IfxI2c_I2c_Device`, defaulted to
`TRUE`, copied device→config at `IfxI2c_I2c.c:102`/`112` — and **never read by
`read2()` or `write2()`**. Setting it has no effect. Control STOP via SOPE (T2).

### T4 — unbounded spins

`read2()`/`write2()` busy-wait on FIFO/protocol flags with no deadline. One
unresponsive slave hangs CPU0 permanently. This project **replaced them** with an
in-house transfer engine (10 ms deadline on every wait) built from the register
layer: `IfxI2c_getProtocolInterruptSourceStatus`, `IfxI2c_getDtrinterruptSourceStatus`,
`IfxI2c_isFifoRequest`, `IfxI2c_writeFifo`, `IfxI2c_setTransmitPacketSize`,
`IfxI2c_setReceivePacketSize`, `IfxI2c_clear*InterruptSource*`, `IfxI2c_getBusStatus`.
Verified hang-immune under 60 s of live wire-wiggling. **Use the in-house engine,
not `read2`/`write2`.**

### T5 — recovery must reset the kernel, not just re-init

`IfxI2c_I2c_initModule()` does **not** clear the I2C kernel or FIFOs. After a
slave brown-out the RX path stays wedged forever and one wedged master kills
*every* device on the bus.

Tell-tale signature: **writes succeed, reads fail, and you never see a NAK.**

Recovery sequence: 9 SCL pulses (bit-banged) → `IfxI2c_resetModule()` →
`IfxI2c_resetFifo()` → re-init. Without the 9-pulse step, every warm reset makes
all sensors vanish.

### Cost model

100 kHz blocking driver → bus time *is* CPU time. 9 bits/byte × 10 µs/bit.
Measured 9.4% CPU0 load, ~91% of it literal wire time. 400 kHz would give ~2.5%.

---

## 6. IfxQspi_SpiMaster — SPI

`Qspi/SpiMaster/IfxQspi_SpiMaster.h`

```c
void            IfxQspi_SpiMaster_initModuleConfig (IfxQspi_SpiMaster_Config *cfg, Ifx_QSPI *qspi);
void            IfxQspi_SpiMaster_initModule       (IfxQspi_SpiMaster *h, const IfxQspi_SpiMaster_Config *cfg);
void            IfxQspi_SpiMaster_initChannelConfig(IfxQspi_SpiMaster_ChannelConfig *c, IfxQspi_SpiMaster *h);
IfxQspi_Status  IfxQspi_SpiMaster_initChannel      (IfxQspi_SpiMaster_Channel *ch, const IfxQspi_SpiMaster_ChannelConfig *c);
IfxQspi_Status  IfxQspi_SpiMaster_exchange         (IfxQspi_SpiMaster_Channel *ch, const void *src, void *dest, Ifx_SizeT count);
IfxQspi_Status  IfxQspi_SpiMaster_getStatus        (IfxQspi_SpiMaster_Channel *ch);
```

`IfxQspi_Status` = `{ ok, busy, unknown }`.

### T7 — exchange is asynchronous

`exchange()` starts the transfer and returns. The RX buffer is **not valid** until
`getStatus()` stops returning `IfxQspi_Status_busy`. Always poll with your own
timeout — do not add another unbounded spin.

Three ISRs must be wired or the transfer never completes:
`IfxQspi_SpiMaster_isrTransmit`, `_isrReceive`, `_isrError`.

Pin mapping via `IfxQspi_PinMap.h` tables. **On this board SCLK is P20.13** —
P22.7/P22.8 are electrically unusable (see T1 pad self-test). Hole numbers in
`docs/PINNING.md` §X702 are unverified; `PINNING.md` is otherwise the SSoT for
pin allocation — read the whole section before contradicting it.

---

## 7. IfxFlash — DFLASH (NVM)

```c
void    IfxFlash_eraseSector  (uint32 sectorAddr);
void    IfxFlash_enterPageMode(uint32 pageAddr);
void    IfxFlash_loadPage2X32 (uint32 pageAddr, uint32 wordL, uint32 wordU);
void    IfxFlash_writePage    (uint32 pageAddr);
boolean IfxFlash_waitUnbusy   (uint32 flash, IfxFlash_FlashType type);
```

Sequence: `eraseSector` → `waitUnbusy` → `enterPageMode` → `waitUnbusy` →
N × `loadPage2X32` → `writePage` → `waitUnbusy`. Command sequences must run from
**RAM or a different flash bank** than the one being programmed.

Project usage: 2-sector ping-pong with CRC for `Xcp_Nvm` @ `0x70030200`. That is
the *only* block in DFLASH — the cal block is RAM-only. Board has no FRAM.

---

## 8. IfxGtm — TOM PWM

```c
void    IfxGtm_enable               (Ifx_GTM *gtm);
void    IfxGtm_Cmu_enableClocks     (Ifx_GTM *gtm, uint32 clkMask);
void    IfxGtm_Cmu_setGclkFrequency (Ifx_GTM *gtm, float32 freq);
float32 IfxGtm_Cmu_getModuleFrequency(Ifx_GTM *gtm);
float32 IfxGtm_Cmu_getFxClkFrequency (Ifx_GTM *gtm, IfxGtm_Cmu_Fxclk, boolean assumeEnabled);
void    IfxGtm_Tom_Pwm_initConfig   (IfxGtm_Tom_Pwm_Config *cfg, Ifx_GTM *gtm);
boolean IfxGtm_Tom_Pwm_init         (IfxGtm_Tom_Pwm_Driver *drv, const IfxGtm_Tom_Pwm_Config *cfg);
void    IfxGtm_Tom_Pwm_start        (IfxGtm_Tom_Pwm_Driver *drv, boolean immediate);
void    IfxGtm_Tom_Ch_setCompareOneShadow(...);   // duty update, glitch-free
```

Order matters: `IfxGtm_enable` → `Cmu_setGclkFrequency` → `Cmu_enableClocks` →
per-channel init. Clocks before channels, always.

Duty updates at runtime go through `setCompareOneShadow` — the shadow register
transfers at the next period boundary, so no glitch. Don't write CM1 directly.

DShot300 on P22.0–3 (ATOM0/TIM) uses open-drain + 1.5k pull-up as a 3.3V→5V
level shift. See `docs/PINNING.md` §2.6 (DSHOT_PINS.md was retired and folded in there).

---

## 9. IfxDts — die temperature

```c
void    IfxDts_Dts_initModuleConfig(IfxDts_Dts_Config *cfg);
void    IfxDts_Dts_initModule      (const IfxDts_Dts_Config *cfg);
float32 IfxDts_Dts_getTemperatureCelsius(void);
float32 IfxDts_Dts_convertToCelsius(uint16 dtsValue);
```

Single module, no instance pointer. Feeds `Xcp_Data`.

---

## 10. IfxScuEru — external interrupts (ERU)

`Scu/Std/IfxScuEru.{c,h}` + the REQ pin objects in
`_PinMap/TC39xB/IfxScu_PinMap_TC39xB_516.{c,h}`. **Not `.cproject`-excluded** —
unlike `Qspi`/`I2c`, nothing has to be un-excluded.

```c
void IfxScuEru_initReqPin(IfxScu_Req_In *req, IfxPort_InputMode mode); /* inline */
void IfxScuEru_enableRisingEdgeDetection (IfxScuEru_InputChannel  ch);
void IfxScuEru_disableFallingEdgeDetection(IfxScuEru_InputChannel ch);
void IfxScuEru_enableTriggerPulse        (IfxScuEru_InputChannel  ch);
void IfxScuEru_connectTrigger            (IfxScuEru_InputChannel  ch,
                                          IfxScuEru_InputNodePointer trigSel);
void IfxScuEru_setFlagPatternDetection   (IfxScuEru_OutputChannel out,
                                          IfxScuEru_InputChannel  ch, boolean state);
void IfxScuEru_enablePatternDetectionTrigger(IfxScuEru_OutputChannel out);
void IfxScuEru_setInterruptGatingPattern (IfxScuEru_OutputChannel out,
                                          IfxScuEru_InterruptGatingPattern pattern);
```

Working order is the example in `IfxScuEru.h:89-111`; then
`IfxSrc_init(&SRC_SCUERU<n>, IfxSrc_Tos_cpuX, srpn)` + `IfxSrc_enable`.
⚠️ **That vendor example also calls `IfxScuEru_enableAutoClear()`, which is
wrong for a pulsed signal — see T14. Omitted from the list above on purpose.**

**T10 — only 8 pads on the 516 package reach the ERU, and most are taken.**
The complete list is `IfxScu_PinMap_TC39xB_516.h:112-128` (18 objects, 8 input
channels). On *this* board nearly all are board-committed: P15.4/P15.5 (EEPROM),
P14.3 (TLF35584 WDI), P14.1 (console RX), P33.7 (LED + DAP_SCR), P20.0 (JTAG),
P20.9 (ERAY-B ERRN), P15.1 (LIN1 RXD), P11.10 (RGMII). The genuinely free ones
are **P10.7 (REQ0C, ch0)**, **P10.8 (REQ1C, ch1)**, **P10.3 (REQ3A, ch3)** and
P15.8 (REQ5A, ch5 — carries the Eth-MDINT footprint stub). Check
`docs/PINNING.md` before assuming any of them is still free.

**T11 — only OGU0-3 have a service-request node.** `SRC_SCUERU0..3` at
`0xF0038880/84/88/8C` (`IfxSrc_reg.h:3489-3514`). `IfxScuEru_InputNodePointer_4`
… `_7` and `IfxScuEru_OutputChannel_4..7` exist in the enums but cannot raise an
interrupt on TC39x — a config using them is silently dead.

**T12 — the input channel is fixed by the pad, the output channel is not.**
`req->channelId` in the pin object *is* the `IfxScuEru_InputChannel`
(REQ**0**C_P10_7 → channel 0). `initReqPin` also programs the external-input
mux from `req->select` (`Ifx_RxSel_c` for that pin), so do not call
`IfxScuEru_selectExternalInput` yourself as well.

**T13 — the glitch filter is global, not per channel, and ENDINIT-protected.**
`IfxScuEru_setInputFilterDepth`/`setInputFilterPredivider` write `SCU_EIFILT`
for *all* ERU inputs. Enabling it for one signal changes behaviour for every
future ERU user. Leave it disabled unless a finding forces it.

**T14 — `IfxScuEru_enableAutoClear()` is a misnomer; it does NOT self-clear
after one edge, and combining it with edge detection double-counts a pulse.**
The bit it writes is `EICRn.LDENx` = **"Level Detection Enable"**
(`Ifx_SCU_EICR_Bits`, `Libraries/Infra/Sfr/TC39xB/IfxScu_regdef.h:368` — the
regdef header, generated from Infineon's own register database, is
authoritative; the wrapper function name is not). With `LDENx` enabled,
`INTFx` tracks the pin's **level**: it is *set* by the edge you enabled
(`RENx`/`FENx`) and *cleared* by the **opposite** edge, or by software
(`FMR.FCx`) — confirmed against Infineon's community KB article "How to use
and configure the External Request Unit for AURIX" (no full peripheral UM in
this repo, `docs/ILLD_NOTES.md` intro). The OGU's pattern-detection stage
fires on every **transition** of `INTFx`, not only the set. For a signal that
is genuinely held at a level (a button, say) that is the wanted behaviour —
one interrupt while it's pressed, roughly. For a **self-terminating pulse**
(any data-ready line with a finite `INT_TPULSE_DURATION`) it produces **two**
interrupts per pulse: one on the rising edge (the real event) and a second
when `INTFx` clears on the falling edge, `TPULSE_DURATION` later. Found on the
ICM-42688-P INT1 line (`docs/IMU_INTERRUPT.md`): a bimodal ~107 µs / ~878 µs
interval distribution, the two populations summing to the true ~985 µs sample
period. `IfxScuEru_enableTriggerPulse()` (`EICRn.EIENx`) already generates a
genuine one-shot trigger per edge on its own — `LDENx` is not needed for a
pulsed signal and is actively wrong for one. **For a pulse: `RENx`/`FENx` +
`EIENx` only, never `LDENx`.**

**Pad note:** a 3.3 V peripheral driving a VEXT (5 V) pad still needs
`IfxPort_setPinPadDriver(port, pin, IfxPort_PadDriver_ttlSpeed1)` *after*
`initReqPin`, and must use `IfxPort_InputMode_pullDown`/`noPullDevice` — the
internal pull-**up** goes to 5 V and fights a 3.3 V push-pull output.

First use: `docs/IMU_INTERRUPT.md` (ICM-42688-P data-ready on P10.7).

---

## 11. Cross-cutting

**`DEVICE_TC39XB` must be defined** in the TASKING preprocessing symbols.
Without it `IfxPort.c` takes the wrong `#else` branch and GPIO silently fails.

**Includes must be root-relative** (`Libraries/iLLD/TC3xx/Tricore/...`) — this
has bitten every new peripheral bring-up.

**MISRA (T8)**: vendor headers use unsuffixed constants and `1u << n` patterns
that cppcheck 12.2 flags. Legacy findings are grandfathered in
`tools/misra_baseline.txt`; only *new* violations fail CI. Note MISRA 8.7 *does*
fire on pool drivers that are compiled but never called — an earlier assumption
that it wouldn't was wrong (confirmed on PR #9).

**Stale intermediates (T9)**: after changing a header, `amk` may reuse a stale
`.src`. Delete `.src` + `.o` + `.d` for affected files, then verify the version
string landed in the ELF.

---

## Gaps

Not yet distilled — expect to read vendor source (and then extend this file):
MCMCAN, EVADC, ASCLIN beyond raw `readRxData`/`writeTxData`, GETH/lwIP port,
DMA, SENT/PSI5. *(ERU was a gap — distilled in §10 on 2026-08-27.)*
