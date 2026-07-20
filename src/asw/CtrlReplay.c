/* CtrlReplay.c
 *
 * UDP vector-replay harness for the verified flight controller
 * (flight_ctrl.c — unchanged Simulink-verified reference).
 *
 * Protocol (UDP port 5556, raw frames, everything little-endian):
 *
 *   STEP   in : 64 bytes = 16 x float32
 *               p_ned_soll[3], p_ned_ist[3], v_b_ist[3], psi_soll,
 *               phi_ist[3], om_ist[3]
 *          out: 72 bytes = 17 x float32 + 1 x uint32
 *               T_soll, phi_soll[3], om_soll[3], tau_soll[3],
 *               w_cmd[4], tau_I[3], t_exec [STM ticks, 10 ns]
 *   RESET  in : 1 byte 0x00   -> out: 1 byte 0x00
 *               clears the integrator state AND the timing statistics
 *   STATS  in : 1 byte 0x01   -> out: 16 bytes
 *               uint32 count, uint32 min, uint32 max [ticks],
 *               float32 mean [ticks]
 *   other  -> out: 1 byte 0xEE (NAK)
 *
 * The execution time covers exactly the four controller calls, nothing
 * else. Frames are processed in the CPU0 main-loop lwIP poll context
 * (NO_SYS=1), so calling udp_sendto here is safe.
 */

#include <stdint.h>
#include <string.h>

#include "CtrlReplay.h"
#include "flight_ctrl.h"
#include "SysTime.h"
#include "lwip/udp.h"

#define CTRLREPLAY_PORT         5556u

#define CTRLREPLAY_IN_BYTES     64u     /* 16 x float32                     */
#define CTRLREPLAY_OUT_BYTES    72u     /* 17 x float32 + uint32 t_exec     */
#define CTRLREPLAY_STATS_BYTES  16u

#define CTRLREPLAY_CMD_RESET    0x00u
#define CTRLREPLAY_CMD_STATS    0x01u
#define CTRLREPLAY_NAK          0xEEu

/* ------------------------------------------------------------------ */

static struct udp_pcb *s_pcb;
static ctrl_state_t    s_state;

/* execution-time statistics in STM ticks (10 ns) */
static uint32_t s_statCount;
static uint32_t s_statMin;
static uint32_t s_statMax;
static uint64_t s_statSum;

/* little-endian u32 store without pointer-type reinterpretation */
static void ctrlReplayPutU32(uint8_t *dst, uint32_t v)
{
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
    dst[2] = (uint8_t)((v >> 16) & 0xFFu);
    dst[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void ctrlReplayResetAll(void)
{
    ctrl_reset(&s_state);
    s_statCount = 0u;
    s_statMin   = 0xFFFFFFFFu;
    s_statMax   = 0u;
    s_statSum   = 0u;
}

/* Run one controller tick: unpack 16 floats, call the four stages in
 * cascade order, pack 17 floats + execution time. */
static void ctrlReplayStep(const uint8_t *in, uint8_t *out)
{
    /* Parameter set — mirror of ../Quadrocopter/quad_params.m via
     * flight_ctrl_lct.c (SiL wrapper). Keep in sync with the Simulink
     * reference; a mismatch invalidates every vector comparison. */
    static const ctrl_params_t s_params = {
        /* Physik */
        1.20f,          /* m     */
        9.81f,          /* g     */
        1200.0f,        /* w_max */

        /* Abtastzeit */
        0.001f,         /* Ts    */

        /* Position horizontal */
        1.44f,          /* ned_xy_kP */
        1.92f,          /* ned_xy_kD */

        /* Position vertikal */
        2.7f,           /* ned_z_kP  */
        2.5f,           /* ned_z_kD  */

        /* Lage */
        { 3.0f, 3.0f, 3.0f },                  /* om_kP       */

        /* Raten */
        { 0.100f, 0.100f, 0.216f },            /* tau_kP      */
        { 0.125f, 0.125f, 0.324f },            /* tau_kI      */
        { 1.3f,   1.3f,   0.08f  },            /* tau_sat_up  */
        { -1.3f, -1.3f,  -0.08f  },            /* tau_sat_low */

        /* Mixer-Inverse, Zeilen-Major */
        {
            5.000000000e+04f, -3.142696805e+05f,  3.142696805e+05f,  5.000000000e+06f,
            5.000000000e+04f, -3.142696805e+05f, -3.142696805e+05f, -5.000000000e+06f,
            5.000000000e+04f,  3.142696805e+05f, -3.142696805e+05f,  5.000000000e+06f,
            5.000000000e+04f,  3.142696805e+05f,  3.142696805e+05f, -5.000000000e+06f
        }
    };

    real32_T u[16];
    real32_T y[17];
    real32_T T_soll;
    real32_T phi_soll[3];
    real32_T om_soll[3];
    real32_T tau_soll[3];
    real32_T w_cmd[4];
    uint32_t t0;
    uint32_t dt;

    /* serialization: raw little-endian float32, host and target share
     * the byte order */
    /* cppcheck-suppress misra-c2012-21.15 ; deviation: byte-buffer to
     * float array copy is the intended deserialization */
    (void)memcpy(u, in, (size_t)CTRLREPLAY_IN_BYTES);

    t0 = SysTime_getTicks();

    pos_ctrl_step(&u[0], &u[3], &u[6], u[9], &T_soll, phi_soll, &s_params);
    att_ctrl_step(phi_soll, &u[10], om_soll, &s_params);
    rate_ctrl_step(om_soll, &u[13], tau_soll, &s_state, &s_params);
    mixer_step(T_soll, tau_soll, w_cmd, &s_params);

    dt = SysTime_getTicks() - t0;

    s_statCount++;
    s_statSum += dt;
    if (dt < s_statMin) { s_statMin = dt; }
    if (dt > s_statMax) { s_statMax = dt; }

    y[0]  = T_soll;
    y[1]  = phi_soll[0];  y[2]  = phi_soll[1];  y[3]  = phi_soll[2];
    y[4]  = om_soll[0];   y[5]  = om_soll[1];   y[6]  = om_soll[2];
    y[7]  = tau_soll[0];  y[8]  = tau_soll[1];  y[9]  = tau_soll[2];
    y[10] = w_cmd[0];     y[11] = w_cmd[1];     y[12] = w_cmd[2];
    y[13] = w_cmd[3];
    y[14] = s_state.tau_I[0];
    y[15] = s_state.tau_I[1];
    y[16] = s_state.tau_I[2];

    /* cppcheck-suppress misra-c2012-21.15 ; deviation: float array to
     * byte-buffer copy is the intended serialization */
    (void)memcpy(out, y, sizeof(y));
    ctrlReplayPutU32(&out[sizeof(y)], dt);
}

static void ctrlReplayStats(uint8_t *out)
{
    uint32_t minv     = (s_statCount == 0u) ? 0u : s_statMin;
    float    mean     = 0.0f;
    uint32_t meanBits = 0u;

    if (s_statCount != 0u)
    {
        mean = (float)s_statSum / (float)s_statCount;
    }
    /* cppcheck-suppress misra-c2012-21.15 ; deviation: reading the
     * float32 bit pattern for serialization */
    (void)memcpy(&meanBits, &mean, 4u);

    ctrlReplayPutU32(&out[0],  s_statCount);
    ctrlReplayPutU32(&out[4],  minv);
    ctrlReplayPutU32(&out[8],  s_statMax);
    ctrlReplayPutU32(&out[12], meanBits);
}

static void ctrlReplaySend(const ip_addr_t *addr, uint16_t port,
                           const uint8_t *data, uint16_t len)
{
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);

    if (p != NULL)
    {
        (void)memcpy(p->payload, data, len);
        (void)udp_sendto(s_pcb, p, addr, port);
        (void)pbuf_free(p);
    }
}

static void ctrlReplayRecv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                           const ip_addr_t *addr, u16_t port)
{
    uint8_t rx[CTRLREPLAY_IN_BYTES];
    uint8_t tx[CTRLREPLAY_OUT_BYTES];
    uint16_t txLen = 1u;

    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(pcb);

    if (p != NULL)
    {
        if (p->tot_len == CTRLREPLAY_IN_BYTES)
        {
            (void)pbuf_copy_partial(p, rx, CTRLREPLAY_IN_BYTES, 0u);
            ctrlReplayStep(rx, tx);
            txLen = CTRLREPLAY_OUT_BYTES;
        }
        else if (p->tot_len == 1u)
        {
            (void)pbuf_copy_partial(p, rx, 1u, 0u);

            if (rx[0] == CTRLREPLAY_CMD_RESET)
            {
                ctrlReplayResetAll();
                tx[0] = CTRLREPLAY_CMD_RESET;
            }
            else if (rx[0] == CTRLREPLAY_CMD_STATS)
            {
                ctrlReplayStats(tx);
                txLen = CTRLREPLAY_STATS_BYTES;
            }
            else
            {
                tx[0] = CTRLREPLAY_NAK;
            }
        }
        else
        {
            tx[0] = CTRLREPLAY_NAK;
        }

        (void)pbuf_free(p);
        ctrlReplaySend(addr, port, tx, txLen);
    }
}

void CtrlReplay_init(void)
{
    ctrlReplayResetAll();

    s_pcb = udp_new();

    if (s_pcb != NULL)
    {
        err_t bindResult = udp_bind(s_pcb, IP_ADDR_ANY, CTRLREPLAY_PORT);

        if (bindResult == (err_t)ERR_OK)
        {
            udp_recv(s_pcb, ctrlReplayRecv, NULL);
        }
        else
        {
            udp_remove(s_pcb);
            s_pcb = NULL;
        }
    }
}
