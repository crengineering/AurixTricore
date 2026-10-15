/*
 * am32_eeprom_writer_nano33iot.ino — read the AM32 EEPROM page of ONE GOKU G55M
 * channel through its bootloader and rewrite ONLY byte 26 (motor_kv) and
 * byte 27 (motor_poles). Everything else on the page is written back as read.
 * Derived from am32_bl_reader_nano33iot.ino (2026-09-16); same wiring, same
 * protocol core, plus the three write commands.
 *
 * WHY (thrust rig 2026-10-15): AM32 scales its "low RPM power protect" duty
 * cap with the KV SETTING, kv = byte26 * 40 + 20. Factory default byte 26 = 55
 * = 2220 KV (am32-configurator public/eeprom-defaults/DEFAULT/v2.bin). With
 * the real SunnySky X2216 (880 KV, 14 poles) that cap locks the motor at
 * ~38 % duty / 4200 rpm from DShot throttle ~800 upward. Setting the true KV
 * moves the cap below the motor's operating line and it never bites again.
 *
 *   duty_max = map(k_erpm, kv/100/(32/poles), kv/12/(32/poles), 400, 2000)
 *   byte26 = 55 (2220 KV): cap 20 % below 11 k eRPM, 100 % only above 92 k eRPM
 *   byte26 = 21 ( 860 KV): cap 20 % below  4 k eRPM, 100 % above 35 k eRPM
 *
 * Wiring (one channel at a time, ESC signal lead OFF the AURIX breakout — the
 * AURIX streams idle-low DShot even when disarmed and the bootloader would
 * start the motor firmware within 40 ms):
 *   Nano TX1 (PB22) --[ 1 kΩ series ]--+
 *   Nano RX0 (PB23) -------------------+---- ESC pad 1..4 (one at a time)
 *   Nano GND ----------------------------- ESC pad G
 *   ESC XT60 <- lab supply 12 V, 0,5 A limit. No motor. Pads V, C, T untouched.
 *
 * Order: Nano on USB with this sketch running FIRST (TX idles high = "linker
 * present"), serial monitor 115200, send single characters:
 *   l   RX line level (expect 1 with the ESC powered or not)
 *   G   HUNT: greet every ~250 ms for 30 s while you switch the ESC on; on
 *       contact read the whole 1 KB EEPROM page and decode the settings
 *   W   arm the write: shows old -> new bytes 26/27, waits for Y
 *   Y   execute within 15 s of W: erase + program the page in 16 chunks of 64,
 *       read it back, compare byte for byte (byte 2 excluded: the bootloader
 *       stamps its own version there on every EEPROM write)
 *   e   read + decode the first 48 bytes only (quick check, as in the reader)
 *   h   help
 * Then power the ESC off, move the wires to the next pad, G again.
 *
 * If Y reports a failed chunk: do NOT power-cycle, the page may be half
 * written. Send Y again — it re-erases and rewrites the whole page. Only when
 * the bootloader itself is gone (greeting silent) power-cycle and start at G.
 *
 * Protocol (AM32-bootloader master bootloader/main.c + Mcu/f421/Src/eeprom.c,
 * am32-configurator master src/communication/direct.ts, read 2026-10-15):
 *   19200 8N1 half duplex; every byte we send comes back as echo and is dropped.
 *   greeting    12x 00, 0D, "BLHeli", F4 7D   -> '4','7','1', pin, sig_hi, sig_lo, x, x, 30
 *   SET_ADDRESS FF 00 hi lo crc              -> 30
 *   READ_FLASH  03 n crc                     -> n bytes, crc, 30
 *   SET_BUFFER  FE 00 00 n crc               -> (no ack)   then: n bytes, crc -> 30 / C2
 *   PROG_FLASH  01 01 crc                    -> 30 (flash == buffer, memcmp) / C1
 *   ACK 30 = ok, C1 = bad command / not writable / verify failed, C2 = bad CRC.
 *   CRC16 reflected 0xA001, init 0, low byte first (configurator makeCRC()).
 *   save_flash_nolib(): erases the 1 KB page ONLY if the address is page-aligned,
 *   refuses address/length not multiple of 4, programs 32-bit words, returns
 *   memcmp()==0. Hence: chunk 0 at 0x7C00 erases, chunks 1..15 fill the rest.
 *   EEprom_t is 192 bytes (incl. tune[128] + CAN block); the page is 1024.
 *   The bootloader stays alive as long as the line idles high (it only leaves
 *   on a line held LOW > 20 ms or > 100 bad commands).
 */

// ---------------- what to write ----------------
#define NEW_KV_BYTE   21u    // 21*40+20 = 860 KV, nearest step below 880 (22 = 900 KV)
#define NEW_POLES     14u    // X2216-III 12N14P

#define OFF_KV        26u
#define OFF_POLES     27u
#define PAGE_SIZE     1024u
#define CHUNK         64u

static const uint32_t BL_BAUD = 19200;

// ---------------- pins ----------------
static void rxPullOff()
{
  // The Nano 33 IoT's internal pull on the SERCOM RX pin acts as a pull-DOWN
  // (measured 2026-09-16). Remove it; the 1 kΩ from TX holds the idle high.
  const PinDescription &pd = g_APinDescription[0];   // D0 = RX0 = PB23
  PORT->Group[pd.ulPort].PINCFG[pd.ulPin].reg = PORT_PINCFG_INEN | PORT_PINCFG_PMUXEN;
}

static int rxLevel()
{
  const PinDescription &pd = g_APinDescription[0];
  return (PORT->Group[pd.ulPort].IN.reg & (1ul << pd.ulPin)) ? 1 : 0;
}

// ---------------- CRC16 (init 0, reflected 0xA001) ----------------
static uint16_t crc16(const uint8_t *p, size_t n)
{
  uint16_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    uint8_t xb = p[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (((xb & 1) ^ (crc & 1)) != 0) crc = (crc >> 1) ^ 0xA001; else crc >>= 1;
      xb >>= 1;
    }
  }
  return crc;
}

static void appendCrc(uint8_t *buf, size_t n)      // buf must have n+2 bytes of room
{
  uint16_t c = crc16(buf, n);
  buf[n] = (uint8_t)(c & 0xFF);
  buf[n + 1] = (uint8_t)(c >> 8);
}

// ---------------- half-duplex exchange ----------------
static void hexdump(const uint8_t *b, size_t n)
{
  for (size_t i = 0; i < n; i++) {
    if (b[i] < 16) Serial.print('0');
    Serial.print(b[i], HEX);
    Serial.print(i % 16 == 15 ? '\n' : ' ');
  }
  if (n % 16) Serial.println();
}

// Send tx[0..n) byte by byte while draining the echo (the SAMD21 RX ring holds
// only 64 bytes, a 66-byte payload would overflow it if we drained afterwards),
// then collect the reply until `gapMs` of silence, or `firstMs` without any
// reply byte. expectReply=false: return right after the echo. Returns reply length.
static size_t xfer(const uint8_t *tx, size_t n, uint8_t *rx, size_t rxMax,
                   uint32_t firstMs = 400, uint32_t gapMs = 60, bool expectReply = true)
{
  while (Serial1.available()) Serial1.read();

  size_t echoLeft = n, echoBad = 0, got = 0;
  for (size_t i = 0; i < n; i++) {
    Serial1.write(tx[i]);
    while (Serial1.available()) {
      uint8_t c = (uint8_t)Serial1.read();
      if (echoLeft) { if (c != tx[n - echoLeft]) echoBad++; echoLeft--; }
      else if (got < rxMax) rx[got++] = c;
    }
  }
  Serial1.flush();                                   // last stop bit left the pin

  uint32_t t0 = millis(), last = millis();
  for (;;) {
    if (Serial1.available()) {
      uint8_t c = (uint8_t)Serial1.read();
      last = millis();
      if (echoLeft) { if (c != tx[n - echoLeft]) echoBad++; echoLeft--; }
      else if (got < rxMax) rx[got++] = c;
      if (echoLeft == 0 && !expectReply) break;
    } else {
      if (echoLeft != 0 && millis() - t0 > 100) break;               // echo never came
      if (!expectReply && echoLeft == 0) break;
      if (got == 0 && echoLeft == 0 && millis() - last > firstMs) break;
      if (got > 0 && millis() - last > gapMs) break;
    }
  }
  if (echoLeft || echoBad) {
    Serial.print(F("  [echo] missing=")); Serial.print(echoLeft);
    Serial.print(F(" mismatched=")); Serial.println(echoBad);
    if (echoLeft == n) Serial.println(F("  -> no echo at all: RX is not on the same node as TX/1k, or RX wire open"));
  }
  return got;
}

// ---------------- bootloader commands ----------------
static uint16_t g_signature = 0;
static uint16_t g_eeprom    = 0x7C00;               // AT32F421, 1 KB pages, page-aligned

static bool greeting(bool verbose = true, uint32_t firstMs = 400)
{
  static const uint8_t init[21] = {0,0,0,0,0,0,0,0,0,0,0,0, 0x0D,'B','L','H','e','l','i', 0xF4, 0x7D};
  uint8_t rx[64];
  size_t n = xfer(init, sizeof(init), rx, sizeof(rx), firstMs);
  if (n == 0 && !verbose) return false;              // hunting mode: stay quiet
  Serial.print(F("[greeting] reply ")); Serial.print(n); Serial.println(F(" bytes:"));
  hexdump(rx, n);
  if (n == 0) {
    Serial.println(F("  -> silence. Bootloader not listening: ESC powered before the Nano held the line high"));
    Serial.println(F("     (power-cycle the ESC now, then retry), or the app already took over, or wiring."));
    return false;
  }
  if (n < 9 || rx[8] != 0x30) {
    Serial.println(F("  -> unexpected reply (no ACK 0x30 at byte 8)"));
    return false;
  }
  g_signature = ((uint16_t)rx[4] << 8) | rx[5];
  Serial.print(F("  boot string '")); Serial.write(rx, 3); Serial.print(F("'  input-pin code=0x")); Serial.print(rx[3], HEX);
  Serial.print(F("  MCU signature=0x")); Serial.println(g_signature, HEX);
  if (g_signature != 0x1F06) {
    Serial.println(F("  -> NOT the AT32F421 signature (0x1F06). Page size/EEPROM address unknown: writing is BLOCKED."));
    return false;
  }
  Serial.println(F("  -> bootloader is alive and talking to us."));
  return true;
}

static bool setAddress(uint16_t addr)
{
  uint8_t cmd[6] = {0xFF, 0x00, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF), 0, 0};
  appendCrc(cmd, 4);
  uint8_t rx[8];
  size_t n = xfer(cmd, sizeof(cmd), rx, sizeof(rx));
  if (n < 1 || rx[0] != 0x30) {
    Serial.print(F("  [set_address 0x")); Serial.print(addr, HEX); Serial.print(F("] failed, reply: ")); hexdump(rx, n);
    return false;
  }
  return true;
}

static size_t readFlash(uint8_t count, uint8_t *out)
{
  uint8_t cmd[4] = {0x03, count, 0, 0};
  appendCrc(cmd, 2);
  uint8_t rx[260];
  size_t n = xfer(cmd, sizeof(cmd), rx, sizeof(rx), 600, 60);
  if (n != (size_t)count + 3) {
    Serial.print(F("  [read_flash] expected ")); Serial.print(count + 3); Serial.print(F(" bytes, got ")); Serial.println(n);
    hexdump(rx, n);
    return 0;
  }
  uint16_t want = crc16(rx, count), have = ((uint16_t)rx[count + 1] << 8) | rx[count];
  if (want != have) { Serial.println(F("  [read_flash] CRC mismatch")); return 0; }
  if (rx[count + 2] != 0x30) { Serial.println(F("  [read_flash] no ACK")); return 0; }
  memcpy(out, rx, count);
  return count;
}

// One 64-byte chunk: SET_ADDRESS -> SET_BUFFER header (no ack) -> payload+crc
// (ack) -> PROG_FLASH (ack after erase/program/verify). Mirrors the
// configurator's writeBufferToAddress().
static bool writeChunk(uint16_t addr, const uint8_t *data)
{
  uint8_t rx[8];
  if (!setAddress(addr)) return false;
  delay(5);

  uint8_t hdr[6] = {0xFE, 0x00, 0x00, (uint8_t)CHUNK, 0, 0};
  appendCrc(hdr, 4);
  xfer(hdr, sizeof(hdr), rx, sizeof(rx), 0, 0, false);   // no ack by design
  delay(2);                                               // same gap the two USB writes of the configurator leave

  uint8_t pl[CHUNK + 2];
  memcpy(pl, data, CHUNK);
  appendCrc(pl, CHUNK);
  size_t n = xfer(pl, sizeof(pl), rx, sizeof(rx), 600);
  if (n < 1 || rx[0] != 0x30) {
    Serial.print(F("  [send_buffer @0x")); Serial.print(addr, HEX); Serial.print(F("] no ACK, reply: ")); hexdump(rx, n);
    return false;
  }
  delay(5);

  uint8_t prog[4] = {0x01, 0x01, 0, 0};
  appendCrc(prog, 2);
  n = xfer(prog, sizeof(prog), rx, sizeof(rx), 2000);     // page erase + program + memcmp
  if (n < 1 || rx[0] != 0x30) {
    Serial.print(F("  [prog_flash @0x")); Serial.print(addr, HEX); Serial.print(F("] failed (C1 = verify/not writable), reply: ")); hexdump(rx, n);
    return false;
  }
  return true;
}

// ---------------- page read / decode ----------------
static uint8_t g_page[PAGE_SIZE];
static bool    g_pageValid = false;
static uint32_t g_armedAt  = 0;

static bool readPage(uint8_t *page)
{
  for (uint16_t off = 0; off < PAGE_SIZE; off += CHUNK) {
    if (!setAddress(g_eeprom + off)) return false;
    delay(200);                                        // the configurator waits 200 ms here too
    if (!readFlash(CHUNK, page + off)) return false;
    Serial.print('.');
  }
  Serial.println();
  return true;
}

static void decode(const uint8_t *b)
{
  uint16_t kv = (uint16_t)b[OFF_KV] * 40u + 20u;
  uint8_t  poles = b[OFF_POLES];
  Serial.print(F("  firmware ")); Serial.print(b[3]); Serial.print('.'); Serial.print(b[4]);
  Serial.print(F("  eeprom_version ")); Serial.print(b[1]);
  Serial.print(F("  bootloader ")); Serial.print(b[2]);
  Serial.print(F("  bi_direction(3D) ")); Serial.print(b[18]);
  Serial.print(F("  pwm_frequency ")); Serial.print(b[24]);
  Serial.print(F("  startup_power ")); Serial.println(b[25]);
  Serial.print(F("  byte26 motor_kv = ")); Serial.print(b[OFF_KV]); Serial.print(F(" -> ")); Serial.print(kv); Serial.print(F(" KV"));
  Serial.print(F("   byte27 motor_poles = ")); Serial.println(poles);
  Serial.print(F("  temp limit ")); Serial.print(b[43]); Serial.print(b[43] > 140 || b[43] < 70 ? F(" (off)") : F(" degC"));
  Serial.print(F("   current limit ")); Serial.print(b[44]); Serial.println(b[44] == 0 || b[44] > 100 ? F(" (off)") : F(" (x2 A)"));
  if (poles != 0 && (32u / poles) != 0) {
    uint16_t div = 32u / poles;
    uint16_t lo = kv / 100u / div, hi = kv / 12u / div;
    Serial.print(F("  low-RPM power protect: duty cap 20 % below ")); Serial.print(lo);
    Serial.print(F(" k eRPM (")); Serial.print((uint32_t)lo * 1000u / (poles / 2u));
    Serial.print(F(" rpm), 100 % above ")); Serial.print(hi);
    Serial.print(F(" k eRPM (")); Serial.print((uint32_t)hi * 1000u / (poles / 2u)); Serial.println(F(" rpm)"));
  }
}

static void hunt()
{
  g_pageValid = false;
  Serial.println(F("[hunt] sending greetings for 30 s - switch the ESC supply ON now ..."));
  uint32_t t0 = millis();
  uint16_t tries = 0;
  bool ok = false;
  while (millis() - t0 < 30000) {
    tries++;
    if (greeting(false, 120)) { ok = true; break; }
    if (tries % 4 == 0) Serial.print('.');
    delay(130);
  }
  Serial.println();
  if (!ok) { Serial.println(F("[hunt] no reply in 30 s. Check pad wiring (continuity RX0 <-> pad), then retry.")); return; }
  Serial.print(F("[hunt] connected after ")); Serial.print(tries); Serial.println(F(" greetings. Reading the 1 KB page:"));
  delay(50);
  if (!readPage(g_page)) { Serial.println(F("[page] read failed - send G again")); return; }
  g_pageValid = true;
  Serial.println(F("[page] first 48 bytes (EEprom_t header):"));
  hexdump(g_page, 48);
  decode(g_page);
  Serial.println(F("  -> page read OK. W arms the write of bytes 26/27."));
}

static void readShort()
{
  if (!greeting()) return;
  delay(50);
  if (!setAddress(g_eeprom)) return;
  delay(200);
  uint8_t buf[48];
  if (!readFlash(48, buf)) return;
  Serial.print(F("[eeprom @0x")); Serial.print(g_eeprom, HEX); Serial.println(F("] 48 bytes:"));
  hexdump(buf, 48);
  decode(buf);
}

// ---------------- write ----------------
static void arm()
{
  if (!g_pageValid) { Serial.println(F("[W] no valid page read in this power cycle - send G first")); return; }
  if (g_page[3] != 2) { Serial.println(F("[W] firmware major is not 2 - layout unknown, write BLOCKED")); return; }
  if (g_page[OFF_KV] == NEW_KV_BYTE && g_page[OFF_POLES] == NEW_POLES) {
    Serial.println(F("[W] bytes 26/27 already have the target values - nothing to write")); return;
  }
  Serial.print(F("[W] byte26 motor_kv   ")); Serial.print(g_page[OFF_KV]); Serial.print(F(" -> ")); Serial.print(NEW_KV_BYTE);
  Serial.print(F("  (")); Serial.print((uint16_t)g_page[OFF_KV] * 40u + 20u); Serial.print(F(" -> ")); Serial.print((uint16_t)NEW_KV_BYTE * 40u + 20u); Serial.println(F(" KV)"));
  Serial.print(F("[W] byte27 motor_poles ")); Serial.print(g_page[OFF_POLES]); Serial.print(F(" -> ")); Serial.println(NEW_POLES);
  Serial.println(F("[W] all other 1022 bytes written back unchanged (byte 2 is re-stamped by the bootloader)."));
  Serial.println(F("[W] ARMED - send Y within 15 s to erase + program the page, anything else disarms."));
  g_armedAt = millis();
}

static void execute()
{
  if (g_armedAt == 0 || millis() - g_armedAt > 15000) { Serial.println(F("[Y] not armed (W first, then Y within 15 s)")); g_armedAt = 0; return; }
  g_armedAt = 0;

  Serial.println(F("[Y] checking the bootloader is still there ..."));
  if (!greeting(false, 400)) { Serial.println(F("[Y] bootloader silent - power-cycle and start at G. Nothing written.")); return; }
  delay(50);

  static uint8_t newPage[PAGE_SIZE];
  memcpy(newPage, g_page, PAGE_SIZE);
  newPage[OFF_KV]    = NEW_KV_BYTE;
  newPage[OFF_POLES] = NEW_POLES;

  Serial.print(F("[Y] writing 16 chunks "));
  for (uint16_t off = 0; off < PAGE_SIZE; off += CHUNK) {
    bool ok = false;
    for (uint8_t attempt = 1; attempt <= 3 && !ok; attempt++) {
      ok = writeChunk(g_eeprom + off, newPage + off);
      if (!ok) { Serial.print(F(" [retry chunk @0x")); Serial.print(g_eeprom + off, HEX); Serial.print(']'); delay(20); }
    }
    if (!ok) {
      Serial.println();
      Serial.println(F("[Y] FAILED - the page may be half written. Do NOT power-cycle: send W then Y again to rewrite the whole page."));
      g_armedAt = 0;
      return;
    }
    Serial.print('#');
    delay(5);
  }
  Serial.println(F(" done. Reading back:"));

  static uint8_t verify[PAGE_SIZE];
  if (!readPage(verify)) { Serial.println(F("[Y] read-back failed - send G to re-read and judge, do not power-cycle yet")); return; }

  uint16_t diffs = 0;
  for (uint16_t i = 0; i < PAGE_SIZE; i++) {
    if (i == 2) continue;                              // bootloader version stamp
    if (verify[i] != newPage[i]) {
      diffs++;
      if (diffs <= 8) { Serial.print(F("  mismatch @")); Serial.print(i); Serial.print(F(": wrote ")); Serial.print(newPage[i]); Serial.print(F(" read ")); Serial.println(verify[i]); }
    }
  }
  Serial.println(F("[verify] first 48 bytes now:"));
  hexdump(verify, 48);
  decode(verify);
  if (diffs == 0 && verify[OFF_KV] == NEW_KV_BYTE && verify[OFF_POLES] == NEW_POLES) {
    Serial.println(F("[verify] OK - page identical except bytes 26/27 (and the bootloader stamp in byte 2)."));
    Serial.println(F("         Power the ESC off, move to the next pad, G again."));
    memcpy(g_page, verify, PAGE_SIZE);
  } else {
    Serial.print(F("[verify] FAILED, ")); Serial.print(diffs); Serial.println(F(" mismatching bytes. Send W + Y again before power-cycling."));
  }
}

static void help()
{
  Serial.println(F("--- AM32 EEPROM writer (Nano 33 IoT, 19200 one-wire on TX1/RX0) --- writes ONLY bytes 26/27"));
  Serial.print  (F("target: byte26 motor_kv = ")); Serial.print(NEW_KV_BYTE); Serial.print(F(" (")); Serial.print((uint16_t)NEW_KV_BYTE * 40u + 20u);
  Serial.print  (F(" KV), byte27 motor_poles = ")); Serial.println(NEW_POLES);
  Serial.println(F("G: HUNT - greet for 30 s while you power the ESC, then read + decode the 1 KB page (do this first)"));
  Serial.println(F("W: arm the write (shows old -> new)   Y: execute within 15 s   e: quick 48-byte read   l: RX line level   h: help"));
  Serial.println(F("Order: this sketch running -> ESC lead OFF the AURIX -> power ESC -> G -> W -> Y. One pad at a time."));
}

void setup()
{
  Serial1.begin(BL_BAUD);                            // FIRST: TX idles HIGH from here on
  rxPullOff();
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }
  help();
}

void loop()
{
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\r' || c == '\n' || c == ' ') return;    // ignore line endings from the monitor
  switch (c) {
    case 'l': Serial.print(F("[line] RX = ")); Serial.println(rxLevel()); break;
    case 'g': greeting(); break;
    case 'G': hunt(); break;
    case 'e': readShort(); break;
    case 'W': arm(); break;
    case 'Y': execute(); break;
    case 'h': help(); break;
    default:  if (g_armedAt) { Serial.println(F("[W] disarmed")); g_armedAt = 0; } break;
  }
}
