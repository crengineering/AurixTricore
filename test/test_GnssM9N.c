/* test_GnssM9N.c -- host tests for the NEO-M9N driver.
 *
 * Scope note (2026-08-21): the driver is now UBX-only. The NMEA decoder and
 * its ~30 tests were removed when UBX-NAV-PVT replaced $GNGGA as the source
 * of navigation data -- NAV-PVT supersedes GGA in every respect (position at
 * 1.1 cm instead of 1.85 m, velocity, and accuracy estimates, all from one
 * atomic epoch). See docs/GNSS_UBX.md and the driver header.
 *
 * Include-by-source: the interesting logic (framing, checksum, field
 * extraction) is static, so the whole .c is pulled in and the tests can see
 * the driver's file statics. Listing GnssM9N.c in CMakeLists as well would
 * give duplicate symbols at link.
 *
 * What is deliberately asserted here: the BYTES a function puts on the wire
 * and the VALUES it extracts -- never merely that it returned TRUE. A status
 * code the fake can never make fail is not a test; that mistake shipped a
 * frame with a wrong key byte and a wrong checksum once already.
 */

#include "unity.h"
#include <string.h>

/* Only for FakeStm_setAutoAdvance: the TX deadline is measured with SysTime,
 * so a test that exercises it has to make the fake clock move. */
#include "IfxStm.h"

#include "GnssM9N.c"

/* --- fixtures ---------------------------------------------------------- */

/* UBX-ACK-ACK for a CFG-VALSET, exactly as it comes off the wire, minus the
 * two sync bytes the state machine consumes before entering the UBX state.
 * Checksums verified against docs/GNSS_UBX.md section 2. */
static const uint8 ACK_BODY[8] = {
    0x05u, 0x01u, 0x02u, 0x00u, 0x06u, 0x8Au, 0x98u, 0xC1u
};

/* The CFG-VALSET example from docs/GNSS_UBX.md section 1 (GSV off, RAM layer).
 * Kept as a fixed reference for the framing even though the driver no longer
 * sends this particular key -- it is the frame the note documents. */
static const uint8 GSV_OFF_FRAME[17] = {
    0xB5u, 0x62u, 0x06u, 0x8Au, 0x09u, 0x00u,
    0x00u, 0x01u, 0x00u, 0x00u, 0xC5u, 0x00u, 0x91u, 0x20u, 0x00u,
    0x10u, 0xFDu
};

/* What the driver actually sends at boot, both RAM layer. */
static const uint8 NMEA_OFF_FRAME[17] = {
    0xB5u, 0x62u, 0x06u, 0x8Au, 0x09u, 0x00u,
    0x00u, 0x01u, 0x00u, 0x00u, 0x02u, 0x00u, 0x74u, 0x10u, 0x00u,
    0x20u, 0xB7u
};
static const uint8 NAVPVT_ON_FRAME[17] = {
    0xB5u, 0x62u, 0x06u, 0x8Au, 0x09u, 0x00u,
    0x00u, 0x01u, 0x00u, 0x00u, 0x07u, 0x00u, 0x91u, 0x20u, 0x01u,
    0x53u, 0x48u
};

/* Two-byte values: note the length field reads 0x0A, not 0x09. */
static const uint8 RATE_MEAS_FRAME[18] = {
    0xB5u, 0x62u, 0x06u, 0x8Au, 0x0Au, 0x00u,
    0x00u, 0x01u, 0x00u, 0x00u, 0x01u, 0x00u, 0x21u, 0x30u, 0x64u, 0x00u,
    0x51u, 0xB9u
};
static const uint8 RATE_NAV_FRAME[18] = {
    0xB5u, 0x62u, 0x06u, 0x8Au, 0x0Au, 0x00u,
    0x00u, 0x01u, 0x00u, 0x00u, 0x02u, 0x00u, 0x21u, 0x30u, 0x01u, 0x00u,
    0xEFu, 0xF9u
};
static const uint8 DYNMODEL_FRAME[17] = {
    0xB5u, 0x62u, 0x06u, 0x8Au, 0x09u, 0x00u,
    0x00u, 0x01u, 0x00u, 0x00u, 0x21u, 0x00u, 0x11u, 0x20u, 0x06u,
    0xF2u, 0x4Fu
};

/* --- helpers ----------------------------------------------------------- */

/* Put the driver back into its power-on state.
 *
 * This is what GnssM9N_init() *looks* like it does -- but its reset block is
 * guarded by "initModule returned noError", and with the NULL-buffer
 * configuration this driver uses on purpose, iLLD always returns
 * configurationError there. The block is dead code on the host AND on
 * silicon, so the tests cannot lean on it. Reaching in directly is only
 * possible because this file includes the driver source. */
static void resetDriverState(void)
{
    g_len                       = 0u;
    g_errors                    = 0u;
    g_bytes                     = 0u;
    g_sentences                 = 0u;
    g_poll_counter              = GNSS_NOT_PRESENT_TICKS;
    g_ring_head                 = 0u;
    g_ring_tail                 = 0u;
    g_ring_buf_overflow_counter = 0u;
    g_ubx_payload_len           = 0u;
    g_ubx_ack                   = 0u;
    g_ubx_nak                   = 0u;
    g_ubx_navpvt                = 0u;
    g_tx_discards               = 0u;
    g_detect_ack                = 0u;
    gsv_cfg_sent                = FALSE;
    g_cfg_expected_acks         = 0u;
    parse_state                 = GNSS_IDLE;
    last_byte                   = 0u;
    memset(&g_nav, 0, sizeof(g_nav));
    memset(g_buffer, 0, sizeof(g_buffer));

    /* bring-up instrumentation mirrors (issue #16) */
    g_gnssCfgSent         = 0u;
    g_gnssCfgExpectedAcks = 0u;
    g_gnssCfgAcked        = 0u;
    g_gnssCfgNaked        = 0u;
    g_gnssCfgOk           = 0u;
    g_gnssTxDiscards      = 0u;
    g_gnssUbxNavPvt       = 0u;
    g_gnssUbxSyncCount    = 0u;
}

void setUp(void)
{
    FakeAsclin_reset();
    FakeStm_reset();
    resetDriverState();
}

void tearDown(void) {}

/* Drain the simulated wire the way the hardware would: one interrupt per FIFO
 * fill, until nothing is left. The ISR reads the fill level once per entry,
 * so a single call moves at most 16 bytes. */
static void pumpIsr(void)
{
    while (FakeAsclin_pending() > 0u)
    {
        asclin4IsrReceive();
    }
}

/* Frame a UBX message body (class, id, length, payload, checksum) the way it
 * lands in g_buffer -- i.e. without the sync bytes. Returns total length. */
static uint8 buildUbx(uint8 *out, uint8 cls, uint8 id,
                      const uint8 *payload, uint8 n)
{
    uint8 ck_a = 0u;
    uint8 ck_b = 0u;
    uint8 i;

    out[0] = cls;
    out[1] = id;
    out[2] = n;
    out[3] = 0u;
    for (i = 0u; i < n; i++) { out[4u + i] = payload[i]; }
    for (i = 0u; i < (4u + n); i++)
    {
        ck_a = (uint8)(ck_a + out[i]);
        ck_b = (uint8)(ck_b + ck_a);
    }
    out[4u + n] = ck_a;
    out[5u + n] = ck_b;
    return (uint8)(6u + n);
}

/* Feed a UBX frame in through the ISR, sync bytes included, as the receiver
 * would send it. */
static void pushUbxFrame(const uint8 *body, uint8 len)
{
    uint8 wire[128];

    wire[0] = 0xB5u;
    wire[1] = 0x62u;
    memcpy(&wire[2], body, len);
    FakeAsclin_pushRxBytes(wire, (uint32)len + 2u);
}

/* =======================================================================
 * gnss_checksumUbx
 * ======================================================================= */

void test_checksum_matches_the_documented_ack(void)
{
    uint8 ck_a = 0u;
    uint8 ck_b = 0u;

    /* over class..payload = the first 6 bytes of the ACK body */
    gnss_checksumUbx(ACK_BODY, 6u, &ck_a, &ck_b);

    TEST_ASSERT_EQUAL_UINT8(0x98u, ck_a);
    TEST_ASSERT_EQUAL_UINT8(0xC1u, ck_b);
}

/* ck_b accumulates the running ck_a, which is what makes the pair sensitive
 * to byte ORDER -- a plain sum is not. */
void test_checksum_detects_reordered_bytes(void)
{
    uint8 swapped[6];
    uint8 a1 = 0u, b1 = 0u, a2 = 0u, b2 = 0u;

    memcpy(swapped, ACK_BODY, 6u);
    swapped[4] = ACK_BODY[5];
    swapped[5] = ACK_BODY[4];

    gnss_checksumUbx(ACK_BODY, 6u, &a1, &b1);
    gnss_checksumUbx(swapped,  6u, &a2, &b2);

    TEST_ASSERT_EQUAL_UINT8(a1, a2);            /* plain sum cannot tell   */
    TEST_ASSERT_NOT_EQUAL_UINT8(b1, b2);        /* the running sum can     */
}

/* =======================================================================
 * gnss_sendUbx / gnss_cfgValsetU1  -- what actually goes on the wire
 * ======================================================================= */

void test_sendUbx_emits_the_documented_gsv_off_frame(void)
{
    uint8 payload[9] = {
        0x00u, 0x01u, 0x00u, 0x00u, 0xC5u, 0x00u, 0x91u, 0x20u, 0x00u
    };

    TEST_ASSERT_EQUAL(TRUE, gnss_sendUbx(0x06u, 0x8Au, payload, 9u));

    TEST_ASSERT_EQUAL_UINT32(17u, FakeAsclin_txCount());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(GSV_OFF_FRAME, FakeAsclin_txData(), 17);
}

/* The helper must build that payload itself -- this is what catches a
 * byte-swapped key or a wrong layer. */
void test_cfgValset_builds_the_nmea_off_frame(void)
{
    TEST_ASSERT_EQUAL(TRUE, gnss_cfgValsetU1(0u, CFG_UART1OUTPROT_NMEA));

    TEST_ASSERT_EQUAL_UINT32(17u, FakeAsclin_txCount());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(NMEA_OFF_FRAME, FakeAsclin_txData(), 17);
}

/* Value 0 disables. A 1 here would ENABLE NMEA -- the opposite of the intent,
 * and nothing else in the system would notice. */
void test_cfgValset_nmea_off_really_carries_zero(void)
{
    (void)gnss_cfgValsetU1(0u, CFG_UART1OUTPROT_NMEA);
    TEST_ASSERT_EQUAL_UINT8(0x00u, FakeAsclin_txData()[14]);
}

void test_cfgValset_builds_the_navpvt_on_frame(void)
{
    TEST_ASSERT_EQUAL(TRUE, gnss_cfgValsetU1(1u, CFG_MSGOUT_UBX_NAV_PVT_UART1));

    TEST_ASSERT_EQUAL_UINT32(17u, FakeAsclin_txCount());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(NAVPVT_ON_FRAME, FakeAsclin_txData(), 17);
}

void test_cfgValset_writes_the_key_little_endian(void)
{
    const uint8 *tx;

    TEST_ASSERT_EQUAL(TRUE, gnss_cfgValsetU1(1u, CFG_MSGOUT_UBX_NAV_PVT_UART1));
    tx = FakeAsclin_txData();

    /* 0x20910007 -> 07 00 91 20 */
    TEST_ASSERT_EQUAL_UINT8(0x07u, tx[10]);
    TEST_ASSERT_EQUAL_UINT8(0x00u, tx[11]);
    TEST_ASSERT_EQUAL_UINT8(0x91u, tx[12]);
    TEST_ASSERT_EQUAL_UINT8(0x20u, tx[13]);
    TEST_ASSERT_EQUAL_UINT8(0x01u, tx[14]);     /* value = 1 epoch          */
}

void test_cfgValsetU2_builds_the_rate_meas_frame(void)
{
    TEST_ASSERT_EQUAL(TRUE, gnss_cfgValsetU2(UBX_MEAS_RATE, CFG_RATE_MEAS));

    TEST_ASSERT_EQUAL_UINT32(18u, FakeAsclin_txCount());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(RATE_MEAS_FRAME, FakeAsclin_txData(), 18);
}

void test_cfgValsetU2_builds_the_rate_nav_frame(void)
{
    TEST_ASSERT_EQUAL(TRUE, gnss_cfgValsetU2(1u, CFG_RATE_NAV));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(RATE_NAV_FRAME, FakeAsclin_txData(), 18);
}

/* A U2 value is little-endian like the key. 100 = 0x0064 -> 64 00. */
void test_cfgValsetU2_writes_the_value_little_endian(void)
{
    const uint8 *tx;

    TEST_ASSERT_EQUAL(TRUE, gnss_cfgValsetU2(1000u, CFG_RATE_MEAS));
    tx = FakeAsclin_txData();

    TEST_ASSERT_EQUAL_UINT8(0x0Au, tx[4]);      /* payload length = 10   */
    TEST_ASSERT_EQUAL_UINT8(0xE8u, tx[14]);     /* 1000 = 0x03E8         */
    TEST_ASSERT_EQUAL_UINT8(0x03u, tx[15]);
}

/* AIR1, not the PORT default -- a ground model constrains altitude and
 * rejects real acceleration as implausible. */
void test_cfgValset_builds_the_dynmodel_frame(void)
{
    TEST_ASSERT_EQUAL(TRUE,
        gnss_cfgValsetU1(UBX_DYNMODEL_AIR1, CFG_NAVSPG_DYNMODEL));

    TEST_ASSERT_EQUAL_UINT32(17u, FakeAsclin_txCount());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(DYNMODEL_FRAME, FakeAsclin_txData(), 17);
    TEST_ASSERT_EQUAL_UINT8(6u, FakeAsclin_txData()[14]);   /* AIR1, not 0 */
}

/* Every CFG-VALSET must carry msgId 0x8A. A wrong id here silently becomes a
 * different CFG message -- 0x09 is UBX-CFG-CFG, clear/save/load -- and the
 * receiver NAKs it without any other visible effect. */
void test_every_valset_uses_the_right_class_and_id(void)
{
    const uint8 *tx;

    gnss_cfgValsetU1(0u, CFG_UART1OUTPROT_NMEA);
    tx = FakeAsclin_txData();
    TEST_ASSERT_EQUAL_UINT8(0x06u, tx[2]);
    TEST_ASSERT_EQUAL_UINT8(0x8Au, tx[3]);

    FakeAsclin_reset();
    gnss_cfgValsetU2(UBX_MEAS_RATE, CFG_RATE_MEAS);
    tx = FakeAsclin_txData();
    TEST_ASSERT_EQUAL_UINT8(0x06u, tx[2]);
    TEST_ASSERT_EQUAL_UINT8(0x8Au, tx[3]);
}

void test_sendUbx_rejects_an_oversized_payload(void)
{
    uint8 payload[8] = {0};

    TEST_ASSERT_EQUAL(FALSE,
        gnss_sendUbx(0x06u, 0x8Au, payload, (uint8)(GNSS_UBX_MAX_PAYLOAD + 1u)));
    TEST_ASSERT_EQUAL_UINT32(0u, FakeAsclin_txCount());
}

/* A TX FIFO that never drains must not hang the 100 ms task. Regression guard
 * for the unbounded-spin class of bug that wedged CPU0 in the I2C driver. */
void test_txByte_gives_up_when_the_fifo_never_drains(void)
{
    FakeStm_setAutoAdvance(GNSS_TX_DEADLINE_TICKS / 4u);
    FakeAsclin_setTxBlocked(TRUE);

    TEST_ASSERT_EQUAL(FALSE, gnss_txByte(0x42u));   /* returns, not spins */
    TEST_ASSERT_EQUAL_UINT32(0u, FakeAsclin_txCount());
    TEST_ASSERT_TRUE(g_tx_discards > 0u);
}

/* =======================================================================
 * gnss_ubx_decode
 * ======================================================================= */

void test_decode_accepts_the_documented_ack_frame(void)
{
    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(ACK_BODY, 8u, 2u, &g_nav));
    TEST_ASSERT_EQUAL_UINT8(1u, g_ubx_ack);
    TEST_ASSERT_EQUAL_UINT32(0u, g_ubx_nak);
}

void test_decode_rejects_a_corrupted_payload(void)
{
    uint8 bad[8];

    memcpy(bad, ACK_BODY, sizeof(bad));
    bad[4] = 0x07u;                              /* flip one payload byte  */

    TEST_ASSERT_EQUAL(FALSE, gnss_ubx_decode(bad, 8u, 2u, &g_nav));
    TEST_ASSERT_EQUAL_UINT8(0u, g_ubx_ack);      /* and it was not counted */
}

/* A single wrong checksum byte must be enough to reject -- an && instead of
 * an || here would let half-corrupt frames through. */
void test_decode_rejects_when_only_one_checksum_byte_is_wrong(void)
{
    uint8 bad[8];

    memcpy(bad, ACK_BODY, sizeof(bad));
    bad[6] = (uint8)(bad[6] ^ 0xFFu);            /* CK_A only              */
    TEST_ASSERT_EQUAL(FALSE, gnss_ubx_decode(bad, 8u, 2u, &g_nav));

    memcpy(bad, ACK_BODY, sizeof(bad));
    bad[7] = (uint8)(bad[7] ^ 0xFFu);            /* CK_B only              */
    TEST_ASSERT_EQUAL(FALSE, gnss_ubx_decode(bad, 8u, 2u, &g_nav));
}

void test_decode_counts_a_nak_separately_from_an_ack(void)
{
    uint8 pl[2] = { 0x06u, 0x8Au };
    uint8 f[16];
    uint8 len = buildUbx(f, 0x05u, 0x00u, pl, 2u);

    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(f, len, 2u, &g_nav));
    TEST_ASSERT_EQUAL_UINT8 (0u, g_ubx_ack);
    TEST_ASSERT_EQUAL_UINT32(1u, g_ubx_nak);
}

/* Every NAV-PVT field, with values chosen so a shifted offset or a
 * sign-extended byte cannot produce a passing result. */
void test_decode_extracts_every_navpvt_field(void)
{
    uint8 pl[92];
    uint8 f[110];
    uint8 len;

    memset(pl, 0, sizeof(pl));
    /* iTOW      123456789 */ pl[0]=0x15u;  pl[1]=0xCDu;  pl[2]=0x5Bu;  pl[3]=0x07u;
    /* year           2026 */ pl[4]=0xEAu;  pl[5]=0x07u;
    pl[6]=8u; pl[7]=21u; pl[8]=17u; pl[9]=45u; pl[10]=30u;
    pl[11]=0x07u;                                /* valid: date+time+resolved */
    pl[20]=3u;                                   /* fixType = 3D fix      */
    pl[21]=0x01u;                                /* flags.gnssFixOK       */
    pl[23]=12u;                                  /* numSV                 */
    /* lon       115678900 */ pl[24]=0xB4u; pl[25]=0x1Eu; pl[26]=0xE5u; pl[27]=0x06u;
    /* lat       481234567 */ pl[28]=0x87u; pl[29]=0x0Eu; pl[30]=0xAFu; pl[31]=0x1Cu;
    /* hMSL         520000 */ pl[36]=0x40u; pl[37]=0xEFu; pl[38]=0x07u; pl[39]=0x00u;
    /* hAcc           2500 */ pl[40]=0xC4u; pl[41]=0x09u; pl[42]=0x00u; pl[43]=0x00u;
    /* vAcc           4100 */ pl[44]=0x04u; pl[45]=0x10u; pl[46]=0x00u; pl[47]=0x00u;
    /* gSpeed        13890 */ pl[60]=0x42u; pl[61]=0x36u; pl[62]=0x00u; pl[63]=0x00u;
    /* headMot     9012345 */ pl[64]=0x79u; pl[65]=0x84u; pl[66]=0x89u; pl[67]=0x00u;
    /* pDOP            180 */ pl[76]=0xB4u; pl[77]=0x00u;

    len = buildUbx(f, 0x01u, 0x07u, pl, 92u);
    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(f, len, 92u, &g_nav));

    TEST_ASSERT_EQUAL_UINT32(1u,          g_ubx_navpvt);
    TEST_ASSERT_EQUAL_UINT32(123456789u,  g_nav.iTOW);
    TEST_ASSERT_EQUAL_UINT16(2026u,       g_nav.year);
    TEST_ASSERT_EQUAL_UINT8 (8u,          g_nav.month);
    TEST_ASSERT_EQUAL_UINT8 (21u,         g_nav.day);
    TEST_ASSERT_EQUAL_UINT8 (17u,         g_nav.hour);
    TEST_ASSERT_EQUAL_UINT8 (45u,         g_nav.min);
    TEST_ASSERT_EQUAL_UINT8 (30u,         g_nav.sec);
    TEST_ASSERT_EQUAL_UINT8 (3u,          g_nav.fixQuality);
    TEST_ASSERT_EQUAL_UINT8 (12u,         g_nav.numSats);
    TEST_ASSERT_EQUAL_UINT8 (1u,          g_nav.fixOk);
    TEST_ASSERT_EQUAL_INT32 (115678900,   g_nav.lon);
    TEST_ASSERT_EQUAL_INT32 (481234567,   g_nav.lat);
    TEST_ASSERT_EQUAL_INT32 (520000,      g_nav.hMSL);
    TEST_ASSERT_EQUAL_UINT32(2500u,       g_nav.hAcc);
    TEST_ASSERT_EQUAL_UINT32(4100u,       g_nav.vAcc);
    TEST_ASSERT_EQUAL_INT32 (13890,       g_nav.gSpeed);
    TEST_ASSERT_EQUAL_INT32 (9012345,     g_nav.headMot);
    TEST_ASSERT_EQUAL_UINT16(180u,        g_nav.pDOP);
    TEST_ASSERT_EQUAL_UINT8 (1u,          g_nav.timeOk);
    TEST_ASSERT_EQUAL_UINT8 (1u,          g_nav.navOk);
}

/* =======================================================================
 * validity gating
 * ======================================================================= */

/* With no satellite the receiver reports a free-running internal clock (seen
 * as 2020-08-02 on 2026-08-22) and clears validDate. Publishing that would
 * look exactly like a real timestamp, so the calendar fields are zeroed. */
void test_decode_zeroes_the_date_when_validdate_is_clear(void)
{
    uint8 pl[92];
    uint8 f[110];
    uint8 len;

    memset(pl, 0, sizeof(pl));
    pl[4]=0xE4u; pl[5]=0x07u;                    /* year 2020, but...     */
    pl[6]=8u; pl[7]=2u;
    pl[11]=0x00u;                                /* ...validDate is clear */

    len = buildUbx(f, 0x01u, 0x07u, pl, 92u);
    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(f, len, 92u, &g_nav));

    TEST_ASSERT_EQUAL_UINT16(0u, g_nav.year);
    TEST_ASSERT_EQUAL_UINT8 (0u, g_nav.month);
    TEST_ASSERT_EQUAL_UINT8 (0u, g_nav.day);
    TEST_ASSERT_EQUAL_UINT8 (0u, g_nav.timeOk);
}

/* iTOW is GPS time of week, not UTC -- it must survive an unresolved UTC. */
void test_decode_keeps_itow_when_utc_is_unresolved(void)
{
    uint8 pl[92];
    uint8 f[110];
    uint8 len;

    memset(pl, 0, sizeof(pl));
    pl[0]=0x15u; pl[1]=0xCDu; pl[2]=0x5Bu; pl[3]=0x07u;
    pl[11]=0x00u;

    len = buildUbx(f, 0x01u, 0x07u, pl, 92u);
    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(f, len, 92u, &g_nav));

    TEST_ASSERT_EQUAL_UINT32(123456789u, g_nav.iTOW);
}

/* validDate and validTime set but fullyResolved clear = seconds-level
 * uncertainty remains, which is useless for timestamping. */
void test_decode_requires_fullyresolved_for_timeok(void)
{
    uint8 pl[92];
    uint8 f[110];
    uint8 len;

    memset(pl, 0, sizeof(pl));
    pl[11] = 0x03u;                              /* date + time, not resolved */

    len = buildUbx(f, 0x01u, 0x07u, pl, 92u);
    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(f, len, 92u, &g_nav));

    TEST_ASSERT_EQUAL_UINT8(1u, g_nav.validDate);
    TEST_ASSERT_EQUAL_UINT8(1u, g_nav.validTime);
    TEST_ASSERT_EQUAL_UINT8(0u, g_nav.fullyResolved);
    TEST_ASSERT_EQUAL_UINT8(0u, g_nav.timeOk);
}

/* navOk is the fusion gate. A 2D fix has no trustworthy altitude. */
void test_navok_rejects_a_2d_fix(void)
{
    uint8 pl[92];
    uint8 f[110];
    uint8 len;

    memset(pl, 0, sizeof(pl));
    pl[20] = 2u;                                 /* 2D fix    */
    pl[21] = 0x01u;                              /* gnssFixOK */

    len = buildUbx(f, 0x01u, 0x07u, pl, 92u);
    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(f, len, 92u, &g_nav));

    TEST_ASSERT_EQUAL_UINT8(1u, g_nav.fixOk);    /* the receiver is happy */
    TEST_ASSERT_EQUAL_UINT8(0u, g_nav.navOk);    /* we are not            */
}

/* A fix can be flagged valid and still be far out while reacquiring. */
void test_navok_rejects_a_poor_accuracy_fix(void)
{
    uint8 pl[92];
    uint8 f[110];
    uint8 len;

    memset(pl, 0, sizeof(pl));
    pl[20] = 3u;
    pl[21] = 0x01u;
    /* hAcc = 50000 mm = 50 m, past GNSS_HACC_USABLE_MM */
    pl[40]=0x50u; pl[41]=0xC3u; pl[42]=0x00u; pl[43]=0x00u;

    len = buildUbx(f, 0x01u, 0x07u, pl, 92u);
    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(f, len, 92u, &g_nav));

    TEST_ASSERT_EQUAL_UINT8(1u, g_nav.fixOk);
    TEST_ASSERT_EQUAL_UINT8(0u, g_nav.navOk);
}

/* Time and position validity are independent: the receiver can know the time
 * from a single satellite while having no position at all. */
void test_navok_is_independent_of_time_validity(void)
{
    uint8 pl[92];
    uint8 f[110];
    uint8 len;

    memset(pl, 0, sizeof(pl));
    pl[11] = 0x00u;                              /* UTC unresolved        */
    pl[20] = 3u;
    pl[21] = 0x01u;
    pl[40]=0xC4u; pl[41]=0x09u;                  /* hAcc 2.5 m            */

    len = buildUbx(f, 0x01u, 0x07u, pl, 92u);
    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(f, len, 92u, &g_nav));

    TEST_ASSERT_EQUAL_UINT8(0u, g_nav.timeOk);
    TEST_ASSERT_EQUAL_UINT8(1u, g_nav.navOk);
}

/* cfgOk drives DIAG_GNSS_IMPLAUSIBLE: FALSE until every CFG-VALSET we sent
 * has been acknowledged, so a silently rejected key raises a fault. */
void test_cfgok_is_false_until_every_command_is_acked(void)
{
    GnssM9N_Sample sample = {0};
    uint8  expected;
    uint16 i;

    /* nothing sent yet */
    (void)GnssM9N_read(&sample);
    TEST_ASSERT_EQUAL_UINT8(0u, sample.cfgOk);

    /* data arrives -> presence -> the whole config burst goes out */
    pushUbxFrame(ACK_BODY, 8u);
    pumpIsr();
    for (i = 0u; i < 3u; i++) { (void)GnssM9N_read(&sample); }
    TEST_ASSERT_TRUE(gsv_cfg_sent != FALSE);

    /* Counted, not compared against a literal: adding a key must not
     * silently invalidate the check. */
    expected = g_cfg_expected_acks;
    TEST_ASSERT_TRUE(expected > 1u);

    /* one ACK short -> still not OK */
    while (g_ubx_ack < (expected - 1u))
    {
        pushUbxFrame(ACK_BODY, 8u);
        pumpIsr();
        (void)GnssM9N_read(&sample);
    }
    TEST_ASSERT_EQUAL_UINT8(0u, sample.cfgOk);

    /* the last one arrives */
    pushUbxFrame(ACK_BODY, 8u);
    pumpIsr();
    (void)GnssM9N_read(&sample);
    TEST_ASSERT_EQUAL_UINT8(1u, sample.cfgOk);
}

/* Negative coordinates: every payload byte above 127 must be widened before
 * it is shifted, or the value sign-extends into nonsense. */
void test_decode_handles_negative_coordinates(void)
{
    uint8 pl[92];
    uint8 f[110];
    uint8 len;

    memset(pl, 0, sizeof(pl));
    /* lon = -115678900 -> 0xF91AE14C */
    pl[24]=0x4Cu; pl[25]=0xE1u; pl[26]=0x1Au; pl[27]=0xF9u;
    /* velD is not published but hMSL below sea level is the same problem */
    /* hMSL = -17000 -> 0xFFFFBD98 */
    pl[36]=0x98u; pl[37]=0xBDu; pl[38]=0xFFu; pl[39]=0xFFu;

    len = buildUbx(f, 0x01u, 0x07u, pl, 92u);
    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(f, len, 92u, &g_nav));

    TEST_ASSERT_EQUAL_INT32(-115678900, g_nav.lon);
    TEST_ASSERT_EQUAL_INT32(-17000,     g_nav.hMSL);
}

/* fixType alone must not be trusted: invalidLlh set means the position is
 * unusable even with a 3D fix (docs/GNSS_UBX.md section 7). */
void test_decode_clears_fixok_when_llh_is_invalid(void)
{
    uint8 pl[92];
    uint8 f[110];
    uint8 len;

    memset(pl, 0, sizeof(pl));
    pl[20] = 3u;            /* 3D fix     */
    pl[21] = 0x01u;         /* gnssFixOK  */
    pl[78] = 0x01u;         /* invalidLlh */

    len = buildUbx(f, 0x01u, 0x07u, pl, 92u);
    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(f, len, 92u, &g_nav));

    TEST_ASSERT_EQUAL_UINT8(3u, g_nav.fixQuality);
    TEST_ASSERT_EQUAL_UINT8(0u, g_nav.fixOk);
}

void test_decode_clears_fixok_when_gnssfixok_is_clear(void)
{
    uint8 pl[92];
    uint8 f[110];
    uint8 len;

    memset(pl, 0, sizeof(pl));
    pl[20] = 3u;            /* 3D fix, but flags.gnssFixOK not set */

    len = buildUbx(f, 0x01u, 0x07u, pl, 92u);
    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(f, len, 92u, &g_nav));

    TEST_ASSERT_EQUAL_UINT8(0u, g_nav.fixOk);
}

/* An unknown class/id is well formed but not ours -- accepted, not counted
 * by the class-specific counters, but it IS a complete, checksum-valid UBX
 * frame, so it counts toward g_sentences (GnssSentences, issue #16). */
void test_decode_ignores_an_unhandled_message(void)
{
    uint8 pl[4] = { 0x01u, 0x02u, 0x03u, 0x04u };
    uint8 f[16];
    uint8 len = buildUbx(f, 0x0Au, 0x04u, pl, 4u);   /* UBX-MON-VER-ish */

    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(f, len, 4u, &g_nav));
    TEST_ASSERT_EQUAL_UINT8 (0u, g_ubx_ack);
    TEST_ASSERT_EQUAL_UINT32(0u, g_ubx_nak);
    TEST_ASSERT_EQUAL_UINT32(0u, g_ubx_navpvt);
    TEST_ASSERT_EQUAL_UINT32(1u, g_sentences);
}

/* =======================================================================
 * g_sentences (GnssSentences, issue #16) -- was dead, now counts every
 * complete, checksum-valid UBX frame, any class.
 * ======================================================================= */

void test_sentences_counts_every_class_of_valid_frame(void)
{
    uint8 pl[92];
    uint8 f[110];
    uint8 len;

    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(ACK_BODY, 8u, 2u, &g_nav));
    TEST_ASSERT_EQUAL_UINT32(1u, g_sentences);

    memset(pl, 0, sizeof(pl));
    len = buildUbx(f, 0x01u, 0x07u, pl, 92u);
    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(f, len, 92u, &g_nav));
    TEST_ASSERT_EQUAL_UINT32(2u, g_sentences);
}

/* A corrupted checksum must not count -- g_sentences is a health signal, not
 * a byte counter (that is GnssRxBytes). */
void test_sentences_does_not_count_a_bad_checksum(void)
{
    uint8 bad[8];

    memcpy(bad, ACK_BODY, sizeof(bad));
    bad[4] = 0x07u;

    TEST_ASSERT_EQUAL(FALSE, gnss_ubx_decode(bad, 8u, 2u, &g_nav));
    TEST_ASSERT_EQUAL_UINT32(0u, g_sentences);
}

/* =======================================================================
 * bring-up instrumentation mirrors (issue #16)
 * ======================================================================= */

/* Each mirror must track its source static exactly -- a mirror that lags
 * or races the value it reflects would be worse than not having it, since
 * it would misreport what the receiver actually did. */
void test_cfg_mirrors_track_the_internal_counters(void)
{
    GnssM9N_Sample sample = {0};
    uint16         i;

    /* Same bring-up sequence as test_cfgok_is_false_until_every_command_is_acked:
     * presence needs two GnssM9N_read() calls before the one-shot config
     * burst goes out (GnssM9N_timeout()'s first call always reports absent). */
    pushUbxFrame(ACK_BODY, 8u);
    pumpIsr();
    for (i = 0u; i < 3u; i++) { (void)GnssM9N_read(&sample); }

    TEST_ASSERT_EQUAL_UINT8 (1u,                  g_gnssCfgSent);
    TEST_ASSERT_EQUAL_UINT8 (g_cfg_expected_acks, g_gnssCfgExpectedAcks);
    TEST_ASSERT_EQUAL_UINT8 (g_ubx_ack,           g_gnssCfgAcked);
    TEST_ASSERT_EQUAL_UINT32(g_ubx_nak,           g_gnssCfgNaked);
    TEST_ASSERT_EQUAL_UINT8 (sample.cfgOk,        g_gnssCfgOk);
    TEST_ASSERT_TRUE(g_gnssCfgExpectedAcks > 1u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_gnssCfgOk);   /* only one ACK so far */

    /* drain the remaining ACKs */
    while (g_ubx_ack < g_cfg_expected_acks)
    {
        pushUbxFrame(ACK_BODY, 8u);
        pumpIsr();
        (void)GnssM9N_read(&sample);
    }
    TEST_ASSERT_EQUAL_UINT8(1u, g_gnssCfgOk);
    TEST_ASSERT_EQUAL_UINT8(g_ubx_ack, g_gnssCfgAcked);
}

void test_tx_discard_mirror_tracks_the_internal_counter(void)
{
    FakeStm_setAutoAdvance(GNSS_TX_DEADLINE_TICKS / 4u);
    FakeAsclin_setTxBlocked(TRUE);

    (void)gnss_txByte(0x42u);

    TEST_ASSERT_TRUE(g_tx_discards > 0u);
    TEST_ASSERT_EQUAL_UINT8(g_tx_discards, g_gnssTxDiscards);
}

void test_navpvt_mirror_tracks_the_internal_counter(void)
{
    uint8 pl[92];
    uint8 f[110];
    uint8 len;

    memset(pl, 0, sizeof(pl));
    len = buildUbx(f, 0x01u, 0x07u, pl, 92u);

    TEST_ASSERT_EQUAL(TRUE, gnss_ubx_decode(f, len, 92u, &g_nav));
    TEST_ASSERT_EQUAL_UINT32(1u, g_gnssUbxNavPvt);
    TEST_ASSERT_EQUAL_UINT32(g_ubx_navpvt, g_gnssUbxNavPvt);
}

/* Answers "is this UBX at all" on its own: a sync pair increments the count
 * even when the frame that follows is garbage and never decodes. */
void test_sync_count_increments_even_when_the_frame_is_garbage(void)
{
    GnssM9N_Sample sample = {0};
    uint8 junk[8] = { 0xB5u, 0x62u, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u };

    FakeAsclin_pushRxBytes(junk, sizeof(junk));
    pumpIsr();
    (void)GnssM9N_read(&sample);

    TEST_ASSERT_EQUAL_UINT32(1u, g_gnssUbxSyncCount);
}

/* An NMEA-only stream (leading '$', no 0xB5 0x62 ever) must leave the sync
 * counter at zero -- this is the exact signature that would explain
 * GnssSentences stuck at 0 with GnssErrors also 0. */
void test_sync_count_stays_zero_on_an_nmea_stream(void)
{
    GnssM9N_Sample sample = {0};
    const uint8 nmea[] = "$GNGGA,120000.00,4807.038,N*47\r\n";

    FakeAsclin_pushRxBytes(nmea, (uint32)(sizeof(nmea) - 1u));
    pumpIsr();
    (void)GnssM9N_read(&sample);

    TEST_ASSERT_EQUAL_UINT32(0u, g_gnssUbxSyncCount);
}

/* =======================================================================
 * ISR + ring buffer
 * ======================================================================= */

void test_isr_moves_fifo_bytes_into_the_ring(void)
{
    FakeAsclin_pushRxBytes(ACK_BODY, 8u);
    pumpIsr();

    TEST_ASSERT_EQUAL_UINT32(8u, g_bytes);
    TEST_ASSERT_EQUAL_UINT16(8u, g_ring_head);
}

void test_isr_needs_several_entries_for_a_long_burst(void)
{
    uint8 blob[100];
    memset(blob, 0x41, sizeof(blob));

    FakeAsclin_pushRxBytes(blob, sizeof(blob));
    asclin4IsrReceive();                    /* one entry drains <= 16 */

    TEST_ASSERT_EQUAL_UINT32(16u, g_bytes);
    TEST_ASSERT_TRUE(FakeAsclin_pending() > 0u);
}

/* =======================================================================
 * GnssM9N_read -- end to end through the state machine
 * ======================================================================= */

void test_read_finds_an_ack_in_the_stream(void)
{
    GnssM9N_Sample sample = {0};

    pushUbxFrame(ACK_BODY, 8u);
    pumpIsr();
    (void)GnssM9N_read(&sample);

    TEST_ASSERT_EQUAL_UINT8(1u, g_ubx_ack);
}

/* A frame will not always arrive whole: at 38400 with a 100 ms consumer a
 * NAV-PVT spans several read() calls. The state must survive the gap. */
void test_read_reassembles_a_frame_split_across_two_calls(void)
{
    GnssM9N_Sample sample = {0};
    uint8 wire[10];

    wire[0] = 0xB5u; wire[1] = 0x62u;
    memcpy(&wire[2], ACK_BODY, 8u);

    FakeAsclin_pushRxBytes(wire, 4u);       /* sync + class + id       */
    pumpIsr();
    (void)GnssM9N_read(&sample);
    TEST_ASSERT_EQUAL_UINT8(0u, g_ubx_ack); /* not yet complete        */

    FakeAsclin_pushRxBytes(&wire[4], 6u);   /* the rest, 100 ms later  */
    pumpIsr();
    (void)GnssM9N_read(&sample);

    TEST_ASSERT_EQUAL_UINT8(1u, g_ubx_ack);
}

/* Presence: absent until bytes arrive, present from the poll after the drain,
 * absent again once GNSS_NOT_PRESENT_TICKS polls pass with an empty ring. */
void test_read_reports_absent_until_data_arrives(void)
{
    GnssM9N_Sample sample = {0};

    TEST_ASSERT_EQUAL(FALSE, GnssM9N_read(&sample));
    TEST_ASSERT_EQUAL(FALSE, GnssM9N_read(&sample));
    TEST_ASSERT_EQUAL_UINT32(0u, sample.rxBytes);

    pushUbxFrame(ACK_BODY, 8u);
    pumpIsr();

    (void)GnssM9N_read(&sample);            /* this drain clears it    */
    TEST_ASSERT_EQUAL(TRUE, GnssM9N_read(&sample));
}

void test_read_reports_absent_again_after_the_timeout(void)
{
    GnssM9N_Sample sample = {0};
    uint16         i;

    pushUbxFrame(ACK_BODY, 8u);
    pumpIsr();
    (void)GnssM9N_read(&sample);
    TEST_ASSERT_EQUAL(TRUE, GnssM9N_read(&sample));

    for (i = 0u; i < GNSS_NOT_PRESENT_TICKS; i++)
    {
        (void)GnssM9N_read(&sample);
    }
    TEST_ASSERT_EQUAL(FALSE, GnssM9N_read(&sample));
    TEST_ASSERT_EQUAL(FALSE, GnssM9N_read(&sample));
}

/* The scaled values must be republished on EVERY call, not only when a frame
 * arrived -- NAV-PVT lands at 1 Hz while read() runs at 100 ms. */
void test_read_republishes_the_last_fix_between_frames(void)
{
    GnssM9N_Sample sample = {0};
    uint8 pl[92];
    uint8 f[110];
    uint8 len;
    uint8 i;

    memset(pl, 0, sizeof(pl));
    pl[21]=0x01u;
    pl[28]=0x87u; pl[29]=0x0Eu; pl[30]=0xAFu; pl[31]=0x1Cu;   /* lat */
    len = buildUbx(f, 0x01u, 0x07u, pl, 92u);

    pushUbxFrame(f, len);
    pumpIsr();
    (void)GnssM9N_read(&sample);

    /* nine more polls with nothing on the wire */
    for (i = 0u; i < 9u; i++)
    {
        (void)GnssM9N_read(&sample);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, 48.1234567f, sample.latDeg);
        TEST_ASSERT_EQUAL_UINT8(1u, sample.fixOk);
    }
}

/* Garbage must not wedge the state machine: a bogus length is rejected and
 * the next good frame is still found. */
void test_read_recovers_from_an_oversized_length(void)
{
    GnssM9N_Sample sample = {0};
    uint8 junk[8] = { 0xB5u, 0x62u, 0x01u, 0x07u, 0xFFu, 0xFFu, 0x00u, 0x00u };

    FakeAsclin_pushRxBytes(junk, sizeof(junk));
    pumpIsr();
    (void)GnssM9N_read(&sample);

    pushUbxFrame(ACK_BODY, 8u);
    pumpIsr();
    (void)GnssM9N_read(&sample);

    TEST_ASSERT_EQUAL_UINT8(1u, g_ubx_ack);
}

/* read() must never touch the peripheral -- the ISR owns it. */
void test_read_does_not_touch_the_hardware_fifo(void)
{
    GnssM9N_Sample sample = {0};

    FakeAsclin_pushRxBytes(ACK_BODY, 8u);   /* no pumpIsr on purpose */
    (void)GnssM9N_read(&sample);

    TEST_ASSERT_EQUAL_UINT32(8u, FakeAsclin_pending());
    TEST_ASSERT_EQUAL_UINT32(0u, g_bytes);
}

/* =======================================================================
 * GnssM9N_init
 * ======================================================================= */

void test_init_reports_ok_when_the_module_is_clocked(void)
{
    FakeAsclin_forceClockSource(IfxAsclin_ClockSource_ascFastClock);
    TEST_ASSERT_EQUAL(TRUE, GnssM9N_init());
}

void test_init_reports_failure_when_the_module_is_unclocked(void)
{
    FakeAsclin_forceClockSource(IfxAsclin_ClockSource_noClock);
    TEST_ASSERT_EQUAL(FALSE, GnssM9N_init());
}

void test_init_requests_the_gnss_baudrate(void)
{
    (void)GnssM9N_init();
    TEST_ASSERT_EQUAL_FLOAT((float)UART_SPEED_38400,
                            FakeAsclin_Asc_lastConfig()->baudrate.baudrate);
}

void test_init_configures_the_rx_interrupt(void)
{
    (void)GnssM9N_init();
    TEST_ASSERT_EQUAL_UINT16(ISR_PRIORITY_ASCLIN4_RX,
                             FakeAsclin_Asc_lastConfig()->interrupt.rxPriority);
}

/* The reset used to be dead code: it sat behind "initModule() == noError",
 * and iLLD returns configurationError whenever tx/rxBuffer is NULL_PTR --
 * which this driver passes on purpose. Fixed 2026-08-22; this test is the
 * inverted version of the one that pinned the bug. */
void test_init_resets_driver_state(void)
{
    FakeAsclin_reset();

    g_bytes           = 4242u;
    g_sentences       = 99u;
    g_ring_head       = 17u;
    g_ubx_ack         = 5u;
    parse_state       = GNSS_UBX;
    g_ubx_payload_len = 77u;
    g_nav.numSats     = 9u;

    (void)GnssM9N_init();

    TEST_ASSERT_EQUAL_UINT32(0u, g_bytes);
    TEST_ASSERT_EQUAL_UINT32(0u, g_sentences);
    TEST_ASSERT_EQUAL_UINT16(0u, g_ring_head);
    TEST_ASSERT_EQUAL_UINT8 (0u, g_ubx_ack);
    TEST_ASSERT_EQUAL(GNSS_IDLE, parse_state);
    TEST_ASSERT_EQUAL_UINT16(0u, g_ubx_payload_len);
    TEST_ASSERT_EQUAL_UINT8 (0u, g_nav.numSats);
}

int main(void)
{
    UNITY_BEGIN();

    /* checksum */
    RUN_TEST(test_checksum_matches_the_documented_ack);
    RUN_TEST(test_checksum_detects_reordered_bytes);

    /* transmit */
    RUN_TEST(test_sendUbx_emits_the_documented_gsv_off_frame);
    RUN_TEST(test_cfgValset_builds_the_nmea_off_frame);
    RUN_TEST(test_cfgValset_nmea_off_really_carries_zero);
    RUN_TEST(test_cfgValset_builds_the_navpvt_on_frame);
    RUN_TEST(test_cfgValsetU2_builds_the_rate_meas_frame);
    RUN_TEST(test_cfgValsetU2_builds_the_rate_nav_frame);
    RUN_TEST(test_cfgValsetU2_writes_the_value_little_endian);
    RUN_TEST(test_cfgValset_builds_the_dynmodel_frame);
    RUN_TEST(test_every_valset_uses_the_right_class_and_id);
    RUN_TEST(test_cfgValset_writes_the_key_little_endian);
    RUN_TEST(test_sendUbx_rejects_an_oversized_payload);
    RUN_TEST(test_txByte_gives_up_when_the_fifo_never_drains);

    /* decode */
    RUN_TEST(test_decode_accepts_the_documented_ack_frame);
    RUN_TEST(test_decode_rejects_a_corrupted_payload);
    RUN_TEST(test_decode_rejects_when_only_one_checksum_byte_is_wrong);
    RUN_TEST(test_decode_counts_a_nak_separately_from_an_ack);
    RUN_TEST(test_decode_extracts_every_navpvt_field);

    /* validity gating */
    RUN_TEST(test_decode_zeroes_the_date_when_validdate_is_clear);
    RUN_TEST(test_decode_keeps_itow_when_utc_is_unresolved);
    RUN_TEST(test_decode_requires_fullyresolved_for_timeok);
    RUN_TEST(test_navok_rejects_a_2d_fix);
    RUN_TEST(test_navok_rejects_a_poor_accuracy_fix);
    RUN_TEST(test_navok_is_independent_of_time_validity);
    RUN_TEST(test_cfgok_is_false_until_every_command_is_acked);
    RUN_TEST(test_decode_handles_negative_coordinates);
    RUN_TEST(test_decode_clears_fixok_when_llh_is_invalid);
    RUN_TEST(test_decode_clears_fixok_when_gnssfixok_is_clear);
    RUN_TEST(test_decode_ignores_an_unhandled_message);

    /* g_sentences (GnssSentences, issue #16) */
    RUN_TEST(test_sentences_counts_every_class_of_valid_frame);
    RUN_TEST(test_sentences_does_not_count_a_bad_checksum);

    /* bring-up instrumentation mirrors (issue #16) */
    RUN_TEST(test_cfg_mirrors_track_the_internal_counters);
    RUN_TEST(test_tx_discard_mirror_tracks_the_internal_counter);
    RUN_TEST(test_navpvt_mirror_tracks_the_internal_counter);
    RUN_TEST(test_sync_count_increments_even_when_the_frame_is_garbage);
    RUN_TEST(test_sync_count_stays_zero_on_an_nmea_stream);

    /* ISR + ring */
    RUN_TEST(test_isr_moves_fifo_bytes_into_the_ring);
    RUN_TEST(test_isr_needs_several_entries_for_a_long_burst);

    /* read */
    RUN_TEST(test_read_finds_an_ack_in_the_stream);
    RUN_TEST(test_read_reassembles_a_frame_split_across_two_calls);
    RUN_TEST(test_read_reports_absent_until_data_arrives);
    RUN_TEST(test_read_reports_absent_again_after_the_timeout);
    RUN_TEST(test_read_republishes_the_last_fix_between_frames);
    RUN_TEST(test_read_recovers_from_an_oversized_length);
    RUN_TEST(test_read_does_not_touch_the_hardware_fifo);

    /* init */
    RUN_TEST(test_init_reports_ok_when_the_module_is_clocked);
    RUN_TEST(test_init_reports_failure_when_the_module_is_unclocked);
    RUN_TEST(test_init_requests_the_gnss_baudrate);
    RUN_TEST(test_init_configures_the_rx_interrupt);
    RUN_TEST(test_init_resets_driver_state);

    return UNITY_END();
}
