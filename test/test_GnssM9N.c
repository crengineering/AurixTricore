/* Unit tests for the GNSS NEO-M9N driver.
 *
 * ---------------------------------------------------------------------------
 * WHY THIS FILE #includes A .c FILE
 * ---------------------------------------------------------------------------
 * Everything that is worth testing in GnssM9N.c is `static`: the checksum, the
 * ASCII conversion, the field parser and the decoder. Linking against the
 * object file would leave only GnssM9N_init/_read reachable -- i.e. only the
 * two functions that need hardware, and none of the pure logic.
 *
 * So this test is built as a single translation unit that pulls the driver in
 * by source. GnssM9N.c is deliberately NOT listed in CMakeLists any more;
 * listing it as well would give duplicate symbols at link time.
 *
 * The cost is one unusual #include. What it buys:
 *   - the internal helpers are testable without weakening them to extern,
 *   - the module-level statics are reachable, which is the only way to get a
 *     clean slate between tests (see resetDriverState below and the note on
 *     GnssM9N_init),
 *   - the shipped firmware source stays byte-identical -- no test hooks, no
 *     GNSS_STATIC macro, nothing that only exists for the test build.
 *
 * ---------------------------------------------------------------------------
 * TESTABILITY OF THE DRIVER AS DESIGNED -- summary
 * ---------------------------------------------------------------------------
 * Good:
 *   + The parsing chain is pure-ish: gnss_checksum, convert_ascii_to_int and
 *     GnssM9N_decode take a buffer in and hand a result out. No register
 *     access, no timing, no waiting. That is why the bulk of the tests below
 *     are plain "string in, number out" and need no fake at all.
 *   + Splitting the ISR (bytes -> ring buffer) from GnssM9N_read (ring buffer
 *     -> sentence -> decode) means the framing logic can be driven by calling
 *     asclin4IsrReceive() as an ordinary function. A driver that parsed inside
 *     the ISR would not be testable this way.
 *   + The decoder writes through a caller-supplied GnssM9N_Nav*, so a test
 *     owns the output and can prove that a rejected sentence leaves it alone.
 *
 * Awkward, and what it costs the tests (details at each test):
 *   - nmea_sentence_parser is NOT pure: it carries its cursor in two file-level
 *     statics. Its result therefore depends on what was called before it. Every
 *     test of it has to reset that cursor by hand, and the fields must be asked
 *     for in ascending order or the answer is silently FALSE.
 *   - convert_ascii_to_int returns 0 both for "the digit zero" and for "not a
 *     hex character". A caller cannot tell the two apart, so gnss_checksum
 *     silently accepts a malformed checksum field as 0x00.
 *   - GnssM9N_init reports its result through a *side channel* (it re-reads the
 *     peripheral clock source) rather than through the iLLD status, and its
 *     whole software-reset block sits behind a condition that is never true on
 *     silicon. Tests therefore cannot rely on init for a clean slate.
 *   - GnssM9N_read hardcodes its return to TRUE, so there is no observable
 *     "GNSS not present" state to assert on.
 * The last two are driver findings, not test problems; the tests below pin the
 * behaviour as it is today so a later fix shows up as a failing test.
 */

#include "unity.h"
#include <string.h>

/* Include-by-source: see the header comment. Everything after this line can
 * see the driver's statics. */
#include "GnssM9N.c"

/* --- fixtures ---------------------------------------------------------- */

/* A textbook GNGGA: fix quality 1 (GPS fix), 8 satellites used. */
#define GGA_8SATS   "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59"
/* Same sentence, differential fix (2) and 12 satellites. */
#define GGA_12SATS  "$GNGGA,123519,4807.038,N,01131.000,E,2,12,0.9,545.4,M,46.9,M,,*51"
/* 33 satellites -- above GNSS_MAX_SATELLITES, must be rejected. */
#define GGA_33SATS  "$GNGGA,123519,4807.038,N,01131.000,E,1,33,0.9,545.4,M,46.9,M,,*51"
/* Cold start: no fix, empty position fields, quality 0 and 00 satellites. */
#define GGA_NOFIX   "$GNGGA,,,,,,0,00,99.99,,,,,,*56"
/* A sentence the driver does not decode, but whose checksum is valid. */
#define RMC_VALID   "$GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*74"
/* Minimal well-formed GGA: field N is the digit N. */
#define GGA_TINY    "$GNGGA,1,2,3,4,5,6,7*54"

#define LEN(s)  ((uint8)(sizeof(s) - 1u))

/* --- helpers ----------------------------------------------------------- */

/* Put the driver back into its power-on state.
 *
 * This is what GnssM9N_init() *looks* like it does -- but its reset block is
 * guarded by "initModule returned noError", and with the polled/NULL-buffer
 * configuration the driver uses, iLLD always returns configurationError there
 * (IfxAsclin_Asc.c sets it in the else branch of "did the application supply a
 * buffer"). The block is therefore dead code on the host AND on silicon, so
 * the tests cannot lean on it. Reaching in directly is only possible because
 * this file includes the driver source. */
static void resetDriverState(void)
{
    g_len                       = 0u;
    g_errors                    = 0u;
    g_bytes                     = 0u;
    g_sentences                 = 0u;
    g_timeout                   = TRUE;
    g_poll_counter              = GNSS_NOT_PRESENT_TICKS;
    g_ring_head                 = 0u;
    g_ring_tail                 = 0u;
    g_ring_buf_overflow_counter = 0u;
    g_nav.numSats               = 0u;
    g_nav.fixQuality            = 0u;
    nmea_parser_index           = 0u;
    nmea_parser_bytes           = 0u;
    memset(g_buffer, 0, sizeof(g_buffer));
}

/* Drain the simulated wire the way the hardware would: one interrupt per FIFO
 * fill, until nothing is left. The ISR reads the fill level once per entry, so
 * a single call moves at most 16 bytes. */
static void pumpIsr(void)
{
    while (FakeAsclin_pending() > 0u)
    {
        asclin4IsrReceive();
    }
}

/* Rewind the field parser. Needed before every direct nmea_sentence_parser
 * call because the cursor lives in file statics, not in the arguments. */
static void rewindParser(void)
{
    nmea_parser_index = 0u;
    nmea_parser_bytes = 0u;
}

void setUp(void)
{
    FakeAsclin_reset();
    (void)GnssM9N_init();
    resetDriverState();      /* init does not do it -- see resetDriverState */
}

void tearDown(void)
{
}

/* =======================================================================
 * convert_ascii_to_int
 *
 * Testability: ideal. One char in, one uint8 out, no state, no hardware.
 * The only design wrinkle is the return value doubling as the error value.
 * ======================================================================= */

void test_convert_ascii_decimal_digits(void)
{
    char c;
    for (c = '0'; c <= '9'; c++)
    {
        TEST_ASSERT_EQUAL_UINT8((uint8)(c - '0'), convert_ascii_to_int(c));
    }
}

void test_convert_ascii_uppercase_hex(void)
{
    TEST_ASSERT_EQUAL_UINT8(10u, convert_ascii_to_int('A'));
    TEST_ASSERT_EQUAL_UINT8(11u, convert_ascii_to_int('B'));
    TEST_ASSERT_EQUAL_UINT8(15u, convert_ascii_to_int('F'));
}

/* NMEA mandates uppercase hex, so this is a legal simplification -- but it is
 * a silent one: 'a' is not rejected, it is read as 0. Pinned here so that if
 * someone later adds lowercase support, this test tells them the checksum
 * behaviour changed too. */
void test_convert_ascii_lowercase_hex_reads_as_zero(void)
{
    TEST_ASSERT_EQUAL_UINT8(0u, convert_ascii_to_int('a'));
    TEST_ASSERT_EQUAL_UINT8(0u, convert_ascii_to_int('f'));
}

/* The design cost, made visible: the function cannot say "that was not a hex
 * digit", it can only say 0 -- which is also the answer for '0'. */
void test_convert_ascii_rejects_by_returning_zero(void)
{
    TEST_ASSERT_EQUAL_UINT8(0u, convert_ascii_to_int('G'));
    TEST_ASSERT_EQUAL_UINT8(0u, convert_ascii_to_int('*'));
    TEST_ASSERT_EQUAL_UINT8(0u, convert_ascii_to_int(' '));
    TEST_ASSERT_EQUAL_UINT8(0u, convert_ascii_to_int('\0'));
    /* indistinguishable from a real zero: */
    TEST_ASSERT_EQUAL_UINT8(convert_ascii_to_int('0'), convert_ascii_to_int('Z'));
}

/* =======================================================================
 * gnss_checksum
 *
 * Testability: very good. Pure function over a caller-owned buffer, so a test
 * can hand it any byte sequence, including ones the hardware would never
 * produce. No fake involved.
 * ======================================================================= */

void test_checksum_accepts_valid_sentences(void)
{
    TEST_ASSERT_EQUAL(TRUE, gnss_checksum(GGA_8SATS,  LEN(GGA_8SATS)));
    TEST_ASSERT_EQUAL(TRUE, gnss_checksum(GGA_12SATS, LEN(GGA_12SATS)));
    TEST_ASSERT_EQUAL(TRUE, gnss_checksum(GGA_NOFIX,  LEN(GGA_NOFIX)));
    TEST_ASSERT_EQUAL(TRUE, gnss_checksum(RMC_VALID,  LEN(RMC_VALID)));
    TEST_ASSERT_EQUAL(TRUE, gnss_checksum(GGA_TINY,   LEN(GGA_TINY)));
}

/* One flipped payload byte must break the XOR. */
void test_checksum_detects_corrupted_payload(void)
{
    char corrupted[] = GGA_8SATS;
    corrupted[7] = '9';                       /* was '1' in the time field */
    TEST_ASSERT_EQUAL(FALSE, gnss_checksum(corrupted, LEN(GGA_8SATS)));
}

/* ...and a flipped checksum digit must break it too. */
void test_checksum_detects_wrong_checksum_field(void)
{
    char wrongHigh[] = GGA_8SATS;
    char wrongLow[]  = GGA_8SATS;
    wrongHigh[LEN(GGA_8SATS) - 2u] = '6';     /* *59 -> *69 */
    wrongLow [LEN(GGA_8SATS) - 1u] = '8';     /* *59 -> *58 */
    TEST_ASSERT_EQUAL(FALSE, gnss_checksum(wrongHigh, LEN(GGA_8SATS)));
    TEST_ASSERT_EQUAL(FALSE, gnss_checksum(wrongLow,  LEN(GGA_8SATS)));
}

/* The XOR must span exactly "everything after '$', everything before '*'".
 * Constructed by hand rather than reusing a fixture, so the test fails if the
 * loop bounds drift by one in either direction. */
void test_checksum_covers_exactly_dollar_to_star(void)
{
    /* 'A'^'B' = 0x03 */
    TEST_ASSERT_EQUAL(TRUE,  gnss_checksum("$AB*03", 6u));
    /* Same payload, but claiming the '$' was included ('$'^'A'^'B' = 0x27). */
    TEST_ASSERT_EQUAL(FALSE, gnss_checksum("$AB*27", 6u));
    /* Same payload, but claiming the '*' was included ('A'^'B'^'*' = 0x29). */
    TEST_ASSERT_EQUAL(FALSE, gnss_checksum("$AB*29", 6u));
}

/* Consequence of convert_ascii_to_int's error handling: a checksum field made
 * of junk is read as 0x00 instead of being rejected outright. A payload that
 * genuinely XORs to zero would then pass with any junk suffix. Documented, not
 * endorsed. */
void test_checksum_reads_junk_checksum_field_as_zero(void)
{
    /* 'A'^'A' = 0x00, and "ZZ" also converts to 0x00 -> accepted. */
    TEST_ASSERT_EQUAL(TRUE, gnss_checksum("$AA*ZZ", 6u));
    TEST_ASSERT_EQUAL(TRUE, gnss_checksum("$AA*00", 6u));
}

/* =======================================================================
 * nmea_sentence_parser
 *
 * Testability: the worst of the four. The signature promises a pure function
 * -- buffer in, value out -- but the cursor (nmea_parser_index /
 * nmea_parser_bytes) lives in file-level statics. Consequences:
 *   - every test must rewind that cursor itself (rewindParser),
 *   - the tests are order-dependent by construction,
 *   - it cannot be called from two contexts (e.g. ISR and task) at all.
 * Moving the cursor into a small parser-state struct passed by the caller
 * would make all of that go away and cost nothing at runtime.
 * ======================================================================= */

void test_parser_extracts_a_field(void)
{
    uint8 value = 0xFFu;

    rewindParser();
    TEST_ASSERT_EQUAL(TRUE, nmea_sentence_parser(GGA_8SATS, LEN(GGA_8SATS),
                                                 GNGGA_SATELLITES_USED, &value));
    TEST_ASSERT_EQUAL_UINT8(8u, value);
}

/* Multi-digit fields accumulate left to right. */
void test_parser_accumulates_multiple_digits(void)
{
    uint8 value = 0u;

    rewindParser();
    TEST_ASSERT_EQUAL(TRUE, nmea_sentence_parser(GGA_12SATS, LEN(GGA_12SATS),
                                                 GNGGA_SATELLITES_USED, &value));
    TEST_ASSERT_EQUAL_UINT8(12u, value);
}

/* The intended usage: successive calls share the cursor, so asking for field 6
 * and then field 7 walks the sentence once instead of twice. That is the
 * upside of the shared state -- and the reason GnssM9N_decode must query in
 * ascending order. */
void test_parser_ascending_calls_share_the_cursor(void)
{
    uint8 quality = 0u;
    uint8 sats    = 0u;

    rewindParser();
    TEST_ASSERT_EQUAL(TRUE, nmea_sentence_parser(GGA_12SATS, LEN(GGA_12SATS),
                                                 GNGGA_FIX_QUALITY, &quality));
    TEST_ASSERT_EQUAL(TRUE, nmea_sentence_parser(GGA_12SATS, LEN(GGA_12SATS),
                                                 GNGGA_SATELLITES_USED, &sats));
    TEST_ASSERT_EQUAL_UINT8(2u,  quality);
    TEST_ASSERT_EQUAL_UINT8(12u, sats);
}

/* ...and the downside. Asking for an earlier field after a later one returns
 * FALSE, because the cursor has already run past it. Nothing in the signature
 * warns the caller. */
void test_parser_descending_calls_fail_silently(void)
{
    uint8 sats    = 0u;
    uint8 quality = 0xAAu;

    rewindParser();
    TEST_ASSERT_EQUAL(TRUE, nmea_sentence_parser(GGA_12SATS, LEN(GGA_12SATS),
                                                 GNGGA_SATELLITES_USED, &sats));
    TEST_ASSERT_EQUAL(FALSE, nmea_sentence_parser(GGA_12SATS, LEN(GGA_12SATS),
                                                  GNGGA_FIX_QUALITY, &quality));
    TEST_ASSERT_EQUAL_UINT8(0xAAu, quality);      /* left untouched */
}

/* An empty field yields FALSE and must not clobber the caller's variable --
 * this is what lets GnssM9N_decode keep the previous fix on a partial burst. */
void test_parser_empty_field_leaves_value_untouched(void)
{
    uint8 value = 0x5Au;

    rewindParser();
    /* field 1 (time) is empty in the cold-start sentence */
    TEST_ASSERT_EQUAL(FALSE, nmea_sentence_parser(GGA_NOFIX, LEN(GGA_NOFIX),
                                                  1u, &value));
    TEST_ASSERT_EQUAL_UINT8(0x5Au, value);
}

/* A field index past the end of the sentence must fail rather than run off. */
void test_parser_field_beyond_sentence_fails(void)
{
    uint8 value = 0x11u;

    rewindParser();
    TEST_ASSERT_EQUAL(FALSE, nmea_sentence_parser(GGA_TINY, LEN(GGA_TINY),
                                                  99u, &value));
    TEST_ASSERT_EQUAL_UINT8(0x11u, value);
}

/* Non-digit characters inside a field are skipped, not rejected: "4807.038"
 * parses as the digit sequence 4807038. Combined with the uint8 accumulator
 * that wraps silently. This is fine for the two fields the driver actually
 * reads (both are <= 2 digits) but makes the helper unsafe to reuse for
 * anything wider -- recorded here as a limitation, not as a specification. */
void test_parser_overflows_silently_on_wide_fields(void)
{
    uint8 value = 0u;

    rewindParser();
    /* field 1 = "123519" -> truncated to 8 bits at every step -> 127 */
    TEST_ASSERT_EQUAL(TRUE, nmea_sentence_parser(GGA_8SATS, LEN(GGA_8SATS),
                                                 1u, &value));
    TEST_ASSERT_EQUAL_UINT8(127u, value);
}

/* =======================================================================
 * GnssM9N_decode
 *
 * Testability: good. It is the whole sentence-to-navigation-data step with no
 * hardware in it, and it writes through a caller-owned GnssM9N_Nav*, so a test
 * can prove both what it sets and what it leaves alone. The one blemish is the
 * hidden dependency on the parser cursor -- decode resets it on entry, which
 * is what makes these tests independent of each other.
 * ======================================================================= */

void test_decode_valid_gga_sets_sats_and_quality(void)
{
    GnssM9N_Nav nav = { 0u, 0u };

    TEST_ASSERT_EQUAL(TRUE, GnssM9N_decode(GGA_8SATS, LEN(GGA_8SATS), &nav));
    TEST_ASSERT_EQUAL_UINT8(8u, nav.numSats);
    TEST_ASSERT_EQUAL_UINT8(1u, nav.fixQuality);
}

void test_decode_reports_differential_fix(void)
{
    GnssM9N_Nav nav = { 0u, 0u };

    TEST_ASSERT_EQUAL(TRUE, GnssM9N_decode(GGA_12SATS, LEN(GGA_12SATS), &nav));
    TEST_ASSERT_EQUAL_UINT8(12u, nav.numSats);
    TEST_ASSERT_EQUAL_UINT8(2u,  nav.fixQuality);
}

/* Cold start: valid sentence, zero satellites, quality 0. Decoding succeeds --
 * "no fix" is data, not an error. */
void test_decode_cold_start_is_valid_data(void)
{
    GnssM9N_Nav nav = { 9u, 9u };

    TEST_ASSERT_EQUAL(TRUE, GnssM9N_decode(GGA_NOFIX, LEN(GGA_NOFIX), &nav));
    TEST_ASSERT_EQUAL_UINT8(0u, nav.numSats);
    TEST_ASSERT_EQUAL_UINT8(0u, nav.fixQuality);
}

/* Decode must call the checksum first: a corrupted sentence leaves nav alone,
 * so a garbled burst can never move numSats. */
void test_decode_rejects_bad_checksum_without_touching_nav(void)
{
    GnssM9N_Nav nav = { 7u, 3u };
    char corrupted[] = GGA_8SATS;

    corrupted[LEN(GGA_8SATS) - 1u] = '8';        /* *59 -> *58 */
    TEST_ASSERT_EQUAL(FALSE, GnssM9N_decode(corrupted, LEN(GGA_8SATS), &nav));
    TEST_ASSERT_EQUAL_UINT8(7u, nav.numSats);
    TEST_ASSERT_EQUAL_UINT8(3u, nav.fixQuality);
}

/* The framing guard: '*' must sit exactly three characters from the end. */
void test_decode_rejects_missing_star_delimiter(void)
{
    GnssM9N_Nav nav = { 7u, 3u };
    char noStar[] = GGA_8SATS;

    noStar[LEN(GGA_8SATS) - 3u] = 'X';
    TEST_ASSERT_EQUAL(FALSE, GnssM9N_decode(noStar, LEN(GGA_8SATS), &nav));
    TEST_ASSERT_EQUAL_UINT8(7u, nav.numSats);
}

/* Short buffers must be rejected before any indexing happens -- buffer[len-3]
 * would underflow otherwise. */
void test_decode_rejects_short_buffers(void)
{
    GnssM9N_Nav nav = { 7u, 3u };

    TEST_ASSERT_EQUAL(FALSE, GnssM9N_decode("",       0u, &nav));
    TEST_ASSERT_EQUAL(FALSE, GnssM9N_decode("$G",     2u, &nav));
    TEST_ASSERT_EQUAL(FALSE, GnssM9N_decode("$AB*03", 5u, &nav));
    TEST_ASSERT_EQUAL_UINT8(7u, nav.numSats);
}

/* GNRMC has a valid checksum but is not a talker the decoder handles; it must
 * fall through the default branch without writing anything. */
void test_decode_ignores_other_talkers(void)
{
    GnssM9N_Nav nav = { 7u, 3u };

    TEST_ASSERT_EQUAL(FALSE, GnssM9N_decode(RMC_VALID, LEN(RMC_VALID), &nav));
    TEST_ASSERT_EQUAL_UINT8(7u, nav.numSats);
    TEST_ASSERT_EQUAL_UINT8(3u, nav.fixQuality);
}

/* Plausibility clamp: GNSS_MAX_SATELLITES rejects an impossible count.
 *
 * Note the asymmetry this test pins down -- numSats is protected, but
 * fixQuality is written before the clamp is evaluated, so a sentence rejected
 * as implausible still updates the fix quality. If that is not intended, this
 * test is where it will show up. */
void test_decode_rejects_implausible_satellite_count(void)
{
    GnssM9N_Nav nav = { 7u, 3u };

    TEST_ASSERT_EQUAL(FALSE, GnssM9N_decode(GGA_33SATS, LEN(GGA_33SATS), &nav));
    TEST_ASSERT_EQUAL_UINT8(7u, nav.numSats);      /* clamped away */
    TEST_ASSERT_EQUAL_UINT8(1u, nav.fixQuality);   /* written anyway */
}

/* decode resets the parser cursor on entry, so two sentences in a row decode
 * identically. Without that reset the second call would silently return FALSE
 * -- exactly the failure mode test_parser_descending_calls_fail_silently
 * shows. */
void test_decode_is_repeatable_back_to_back(void)
{
    GnssM9N_Nav nav = { 0u, 0u };

    TEST_ASSERT_EQUAL(TRUE, GnssM9N_decode(GGA_8SATS,  LEN(GGA_8SATS),  &nav));
    TEST_ASSERT_EQUAL(TRUE, GnssM9N_decode(GGA_12SATS, LEN(GGA_12SATS), &nav));
    TEST_ASSERT_EQUAL_UINT8(12u, nav.numSats);
    TEST_ASSERT_EQUAL(TRUE, GnssM9N_decode(GGA_8SATS,  LEN(GGA_8SATS),  &nav));
    TEST_ASSERT_EQUAL_UINT8(8u, nav.numSats);
}

/* =======================================================================
 * asclin4IsrReceive
 *
 * Testability: better than it looks. IFX_INTERRUPT only decorates the
 * function; the body is ordinary C, so the host build calls it directly and
 * the fake plays the role of the FIFO. The only thing a host test cannot
 * reproduce is genuine concurrency -- the ring buffer's single-producer /
 * single-consumer claim is argued, not proven, by these tests.
 * ======================================================================= */

void test_isr_moves_fifo_bytes_into_the_ring(void)
{
    FakeAsclin_pushRx("$GNGGA");
    pumpIsr();

    TEST_ASSERT_EQUAL_UINT32(6u, g_bytes);
    TEST_ASSERT_EQUAL_UINT16(6u, g_ring_head);
    TEST_ASSERT_EQUAL_UINT16(0u, g_ring_tail);
    TEST_ASSERT_EQUAL_UINT8(0u, g_ring_buf_overflow_counter);
    TEST_ASSERT_EQUAL_CHAR('$', g_ring_buffer[0]);
    TEST_ASSERT_EQUAL_CHAR('A', g_ring_buffer[5]);
}

/* The ISR reads the fill level once per entry, and the fake caps it at the
 * 16-entry hardware depth -- so a longer burst needs several interrupts, just
 * as it does on silicon. */
void test_isr_needs_several_entries_for_a_long_burst(void)
{
    FakeAsclin_pushRx(GGA_8SATS);            /* 68 bytes > 16-deep FIFO */
    asclin4IsrReceive();
    TEST_ASSERT_EQUAL_UINT32(16u, g_bytes);

    pumpIsr();
    TEST_ASSERT_EQUAL_UINT32((uint32)LEN(GGA_8SATS), g_bytes);
}

/* Ring full: the ISR must drop the byte and count it, never overwrite unread
 * data. Capacity is RING_BUFFER_SIZE-1 because head==tail means empty. */
void test_isr_counts_overflow_and_keeps_unread_data(void)
{
    char   burst[RING_BUFFER_SIZE + 64u];
    uint16 i;

    for (i = 0u; i < (RING_BUFFER_SIZE + 63u); i++)
    {
        burst[i] = 'A';
    }
    burst[RING_BUFFER_SIZE + 63u] = '\0';

    FakeAsclin_pushRx(burst);
    pumpIsr();

    /* every byte was taken off the FIFO ... */
    TEST_ASSERT_EQUAL_UINT32((uint32)(RING_BUFFER_SIZE + 63u), g_bytes);
    /* ... but only RING_BUFFER_SIZE-1 fitted, the rest were counted as lost */
    TEST_ASSERT_EQUAL_UINT8((uint8)((RING_BUFFER_SIZE + 63u) - (RING_BUFFER_SIZE - 1u)),
                            g_ring_buf_overflow_counter);
    TEST_ASSERT_EQUAL_UINT16(RING_BUFFER_SIZE - 1u, g_ring_head);
    TEST_ASSERT_EQUAL_UINT16(0u, g_ring_tail);
}

/* =======================================================================
 * GnssM9N_read
 *
 * Testability: good for the framing, poor for the return value. The framing
 * (ring buffer -> CR/LF-delimited sentence -> decode) is fully observable
 * through the GnssM9N_Sample the caller supplies. The boolean it returns is
 * not testable, because it is not computed -- see the test at the end.
 * ======================================================================= */

/* End-to-end: bytes on the wire, through the ISR, out as navigation data. */
void test_read_decodes_a_sentence_end_to_end(void)
{
    GnssM9N_Sample sample = { 0u, 0u, 0u, 0u, 0u };

    FakeAsclin_pushRx(GGA_8SATS "\r\n");
    pumpIsr();

    TEST_ASSERT_EQUAL(TRUE, GnssM9N_read(&sample));
    TEST_ASSERT_EQUAL_UINT8(8u, sample.numSats);
    TEST_ASSERT_EQUAL_UINT8(1u, sample.fixType);
    TEST_ASSERT_EQUAL_UINT32(1u, sample.sentences);
    TEST_ASSERT_EQUAL_UINT32((uint32)LEN(GGA_8SATS) + 2u, sample.rxBytes);
    TEST_ASSERT_EQUAL_UINT16(0u, sample.errors);
}

/* The trailing '\n' after '\r' must not be counted as a second sentence. */
void test_read_counts_crlf_as_one_sentence(void)
{
    GnssM9N_Sample sample = { 0u, 0u, 0u, 0u, 0u };

    FakeAsclin_pushRx(GGA_8SATS "\r\n" GGA_12SATS "\r\n");
    pumpIsr();
    (void)GnssM9N_read(&sample);

    TEST_ASSERT_EQUAL_UINT32(2u, sample.sentences);
    TEST_ASSERT_EQUAL_UINT8(12u, sample.numSats);   /* newest wins */
}

/* A sentence split across two calls (the normal case at 38400 baud, where a
 * burst spans several task periods) must still decode once complete. */
void test_read_reassembles_a_split_sentence(void)
{
    GnssM9N_Sample sample = { 0u, 0u, 0u, 0u, 0u };

    FakeAsclin_pushRx("$GNGGA,123519,4807.038,N,0113");
    pumpIsr();
    (void)GnssM9N_read(&sample);
    TEST_ASSERT_EQUAL_UINT32(0u, sample.sentences);
    TEST_ASSERT_EQUAL_UINT8(0u, sample.numSats);

    FakeAsclin_pushRx("1.000,E,1,08,0.9,545.4,M,46.9,M,,*59\r\n");
    pumpIsr();
    (void)GnssM9N_read(&sample);
    TEST_ASSERT_EQUAL_UINT32(1u, sample.sentences);
    TEST_ASSERT_EQUAL_UINT8(8u, sample.numSats);
}

/* A corrupted sentence still counts as a sentence (it was framed), but must
 * not move the navigation data. */
void test_read_keeps_last_fix_when_a_sentence_is_corrupted(void)
{
    GnssM9N_Sample sample = { 0u, 0u, 0u, 0u, 0u };

    FakeAsclin_pushRx(GGA_8SATS "\r\n");
    pumpIsr();
    (void)GnssM9N_read(&sample);
    TEST_ASSERT_EQUAL_UINT8(8u, sample.numSats);

    /* same sentence as GGA_12SATS but with a deliberately wrong checksum */
    FakeAsclin_pushRx("$GNGGA,123519,4807.038,N,01131.000,E,2,12,0.9,545.4,M,46.9,M,,*00\r\n");
    pumpIsr();
    (void)GnssM9N_read(&sample);

    TEST_ASSERT_EQUAL_UINT32(2u, sample.sentences);
    TEST_ASSERT_EQUAL_UINT8(8u, sample.numSats);    /* unchanged */
    TEST_ASSERT_EQUAL_UINT8(1u, sample.fixType);
}

/* A sentence longer than the assembly buffer must not overrun it, and the
 * next good sentence must still decode.
 *
 * FINDING while writing this test: the recovery is not "throw the line away".
 * When g_buffer is full the driver sets g_len = 0 and keeps collecting, so the
 * *tail* of the oversized line becomes a sentence of its own and is counted in
 * sample.sentences (it fails the checksum, so nav data is safe). Dropping
 * bytes until the next CR/LF would be the cleaner recovery. Asserted as-is. */
void test_read_discards_an_oversized_sentence(void)
{
    GnssM9N_Sample sample = { 0u, 0u, 0u, 0u, 0u };
    char   monster[GNSSM9N_BUFFER_SIZE + 32u];
    uint16 i;

    monster[0] = '$';
    for (i = 1u; i < (GNSSM9N_BUFFER_SIZE + 30u); i++)
    {
        monster[i] = 'X';
    }
    monster[GNSSM9N_BUFFER_SIZE + 30u] = '\r';
    monster[GNSSM9N_BUFFER_SIZE + 31u] = '\0';

    FakeAsclin_pushRx(monster);
    FakeAsclin_pushRx(GGA_8SATS "\r\n");
    pumpIsr();
    (void)GnssM9N_read(&sample);

    TEST_ASSERT_EQUAL_UINT8(8u, sample.numSats);
    TEST_ASSERT_EQUAL_UINT8(1u, sample.fixType);
    /* the truncated tail was framed as a sentence of its own -> 2, not 1 */
    TEST_ASSERT_EQUAL_UINT32(2u, sample.sentences);
}

/* Architectural check: GnssM9N_read must never touch the peripheral. If
 * somebody re-introduces a FIFO drain there, the bytes would disappear from
 * the wire without an interrupt and this fails. */
void test_read_does_not_touch_the_hardware_fifo(void)
{
    GnssM9N_Sample sample = { 0u, 0u, 0u, 0u, 0u };

    FakeAsclin_pushRx(GGA_8SATS "\r\n");
    (void)GnssM9N_read(&sample);            /* no pumpIsr() on purpose */

    TEST_ASSERT_EQUAL_UINT32((uint32)LEN(GGA_8SATS) + 2u, FakeAsclin_pending());
    TEST_ASSERT_EQUAL_UINT32(0u, sample.rxBytes);
    TEST_ASSERT_EQUAL_UINT32(0u, sample.sentences);
}

/* Calling read twice with nothing new must not re-decode or re-count. */
void test_read_is_idempotent_when_the_ring_is_empty(void)
{
    GnssM9N_Sample first  = { 0u, 0u, 0u, 0u, 0u };
    GnssM9N_Sample second = { 0u, 0u, 0u, 0u, 0u };

    FakeAsclin_pushRx(GGA_8SATS "\r\n");
    pumpIsr();
    (void)GnssM9N_read(&first);
    (void)GnssM9N_read(&second);

    TEST_ASSERT_EQUAL_UINT32(first.sentences, second.sentences);
    TEST_ASSERT_EQUAL_UINT32(first.rxBytes,   second.rxBytes);
    TEST_ASSERT_EQUAL_UINT8 (first.numSats,   second.numSats);
}

/* FINDING, pinned deliberately.
 *
 * The return value is meant to say "is the GNSS there?" -- g_timeout and
 * g_poll_counter exist for exactly that. But the function assigns
 * status = TRUE unconditionally after the g_timeout check, so the check is
 * dead code and the result carries no information: read() reports TRUE with
 * no receiver attached and nothing ever received.
 *
 * The test asserts today's behaviour so the file stays green, and names the
 * problem so it is not mistaken for intent. When the presence logic is wired
 * up, this test should be replaced by one that asserts FALSE here. */
void test_read_return_value_is_currently_hardcoded_true(void)
{
    GnssM9N_Sample sample = { 0u, 0u, 0u, 0u, 0u };

    TEST_ASSERT_EQUAL(TRUE, g_timeout);          /* "not seen yet" */
    TEST_ASSERT_EQUAL(TRUE, GnssM9N_read(&sample));
    TEST_ASSERT_EQUAL_UINT32(0u, sample.rxBytes);
}

/* =======================================================================
 * GnssM9N_init
 *
 * Testability: the weakest of the module. It reports success through a side
 * channel (re-reading the peripheral clock source) instead of through the
 * value iLLD hands it, and its software-reset block is unreachable. Both are
 * pinned below.
 * ======================================================================= */

/* Success is "the module ended up on the fast ASC clock". */
void test_init_reports_ok_when_the_module_is_clocked(void)
{
    FakeAsclin_reset();
    TEST_ASSERT_EQUAL(TRUE, GnssM9N_init());
}

/* ...and fails when it is not. This is the only real assertion available,
 * since the iLLD status cannot be used (see the last test). */
void test_init_reports_failure_when_the_module_is_unclocked(void)
{
    FakeAsclin_reset();
    FakeAsclin_forceClockSource(IfxAsclin_ClockSource_noClock);
    TEST_ASSERT_EQUAL(FALSE, GnssM9N_init());
}

/* The ISR must be wired to the right SRPN and the right core, otherwise no
 * byte is ever seen. Checked against the config the driver handed to iLLD. */
void test_init_configures_the_rx_interrupt(void)
{
    const IfxAsclin_Asc_Config *cfg;

    FakeAsclin_reset();
    (void)GnssM9N_init();
    cfg = FakeAsclin_Asc_lastConfig();

    TEST_ASSERT_EQUAL_UINT16(ISR_PRIORITY_ASCLIN4_RX, cfg->interrupt.rxPriority);
    TEST_ASSERT_EQUAL(IfxSrc_Tos_cpu0, cfg->interrupt.typeOfService);
}

/* The GNSS ships 38400 baud by default; a wrong rate looks exactly like a
 * dead receiver, so it is worth an assertion. */
void test_init_requests_the_gnss_baudrate(void)
{
    FakeAsclin_reset();
    (void)GnssM9N_init();
    TEST_ASSERT_EQUAL_FLOAT((float32)UART_SPEED_38400,
                            FakeAsclin_Asc_lastConfig()->baudrate.baudrate);
}

/* FINDING, pinned deliberately.
 *
 * GnssM9N_init guards its software reset with
 * "IfxAsclin_Asc_initModule() == IfxAsclin_Status_noError". iLLD sets
 * configurationError whenever txBuffer or rxBuffer is NULL_PTR -- and this
 * driver passes NULL_PTR for both on purpose, because it uses the hardware
 * FIFO plus its own ring buffer instead of the iLLD software FIFOs. So the
 * condition is never true: the counters, the ring indices and g_timeout are
 * never cleared, on the host or on the board. It only goes unnoticed because
 * the statics start at zero after reset.
 *
 * The test proves it by dirtying the state and calling init. When the guard is
 * fixed (drop it, or key the reset off the clock-source check instead), this
 * test fails and should be inverted. */
void test_init_does_not_actually_reset_driver_state(void)
{
    FakeAsclin_reset();

    g_bytes     = 4242u;
    g_sentences = 99u;
    g_ring_head = 17u;

    (void)GnssM9N_init();

    TEST_ASSERT_EQUAL_UINT32(4242u, g_bytes);
    TEST_ASSERT_EQUAL_UINT32(99u,   g_sentences);
    TEST_ASSERT_EQUAL_UINT16(17u,   g_ring_head);
}

void test_send_Ubx(void)
{
    uint8 payload[9] = {0x00, 0x01, 0x00, 0x00, 0xC5, 0x00, 0x90, 0x20, 0x00};
    TEST_ASSERT_EQUAL(TRUE, gnss_sendUbx (0x06, 0x8A, payload, 9));
}

int main(void)
{
    UNITY_BEGIN();

    /* convert_ascii_to_int */
    RUN_TEST(test_convert_ascii_decimal_digits);
    RUN_TEST(test_convert_ascii_uppercase_hex);
    RUN_TEST(test_convert_ascii_lowercase_hex_reads_as_zero);
    RUN_TEST(test_convert_ascii_rejects_by_returning_zero);

    /* gnss_checksum */
    RUN_TEST(test_checksum_accepts_valid_sentences);
    RUN_TEST(test_checksum_detects_corrupted_payload);
    RUN_TEST(test_checksum_detects_wrong_checksum_field);
    RUN_TEST(test_checksum_covers_exactly_dollar_to_star);
    RUN_TEST(test_checksum_reads_junk_checksum_field_as_zero);

    /* nmea_sentence_parser */
    RUN_TEST(test_parser_extracts_a_field);
    RUN_TEST(test_parser_accumulates_multiple_digits);
    RUN_TEST(test_parser_ascending_calls_share_the_cursor);
    RUN_TEST(test_parser_descending_calls_fail_silently);
    RUN_TEST(test_parser_empty_field_leaves_value_untouched);
    RUN_TEST(test_parser_field_beyond_sentence_fails);
    RUN_TEST(test_parser_overflows_silently_on_wide_fields);

    /* GnssM9N_decode */
    RUN_TEST(test_decode_valid_gga_sets_sats_and_quality);
    RUN_TEST(test_decode_reports_differential_fix);
    RUN_TEST(test_decode_cold_start_is_valid_data);
    RUN_TEST(test_decode_rejects_bad_checksum_without_touching_nav);
    RUN_TEST(test_decode_rejects_missing_star_delimiter);
    RUN_TEST(test_decode_rejects_short_buffers);
    RUN_TEST(test_decode_ignores_other_talkers);
    RUN_TEST(test_decode_rejects_implausible_satellite_count);
    RUN_TEST(test_decode_is_repeatable_back_to_back);

    /* asclin4IsrReceive */
    RUN_TEST(test_isr_moves_fifo_bytes_into_the_ring);
    RUN_TEST(test_isr_needs_several_entries_for_a_long_burst);
    RUN_TEST(test_isr_counts_overflow_and_keeps_unread_data);

    /* GnssM9N_read */
    RUN_TEST(test_read_decodes_a_sentence_end_to_end);
    RUN_TEST(test_read_counts_crlf_as_one_sentence);
    RUN_TEST(test_read_reassembles_a_split_sentence);
    RUN_TEST(test_read_keeps_last_fix_when_a_sentence_is_corrupted);
    RUN_TEST(test_read_discards_an_oversized_sentence);
    RUN_TEST(test_read_does_not_touch_the_hardware_fifo);
    RUN_TEST(test_read_is_idempotent_when_the_ring_is_empty);
    RUN_TEST(test_read_return_value_is_currently_hardcoded_true);

    /* GnssM9N_init */
    RUN_TEST(test_init_reports_ok_when_the_module_is_clocked);
    RUN_TEST(test_init_reports_failure_when_the_module_is_unclocked);
    RUN_TEST(test_init_configures_the_rx_interrupt);
    RUN_TEST(test_init_requests_the_gnss_baudrate);
    RUN_TEST(test_init_does_not_actually_reset_driver_state);

    /* send */
    RUN_TEST(test_send_Ubx);

    return UNITY_END();
}
