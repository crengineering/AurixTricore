/* FlightTask.c
 *
 * Flight controller on the real signal path -- see FlightTask.h.
 */

#include "FlightTask.h"
#include "flight_ctrl.h"
#include "flight_ctrl_params.h"
#include "NavState.h"
#include "CtrlLatch.h"
#include "Diagnostics.h"

#define FLIGHTTASK_RAD_S_TO_RPM   (9.5492966f)     /* 60 / (2 pi) */
#define FLIGHTTASK_DEG_TO_RAD     (0.017453293f)
#define FLIGHTTASK_PI             (3.14159265f)

/* Fallbacks for calibration words outside their range (also catches NaN:
 * every range check below is written so that a NaN fails it). */
#define FLIGHTTASK_TILT_MAX_DEG   (30.0f)
#define FLIGHTTASK_RPM_MIN        (1300.0f)
#define FLIGHTTASK_RPM_MIN_LIMIT  (3000.0f)        /* an idle floor above this is a typo        */
#define FLIGHTTASK_RPM_MIN_LOWEST (129.0f)         /* below this the motor does not turn (table) */
#define FLIGHTTASK_OM_KP_LIMIT    (20.0f)          /* 1/s        - verified set: 3               */
#define FLIGHTTASK_TAU_KP_LIMIT   (1.0f)           /* Nm/(rad/s) - verified set: 0.1 / 0.216     */
#define FLIGHTTASK_TAU_KI_LIMIT   (2.0f)           /* Nm/rad     - verified set: 0.125 / 0.324   */
#define FLIGHTTASK_THRUST_LIMIT_N (40.0f)          /* 4 motors at w_lim give ~35 N              */
#define FLIGHTTASK_TS_MIN_S       (0.0005f)
#define FLIGHTTASK_TS_MAX_S       (0.005f)
#define FLIGHTTASK_ANGLE_LIMIT    (7.0f)           /* rad; estimate beyond this is not an angle */
#define FLIGHTTASK_RATE_LIMIT     (40.0f)          /* rad/s; gyro full scale is 34.9            */

/* value if 0 <= value <= hi, else fallback */
static real32_T flightTask_range(real32_T value, real32_T hi, real32_T fallback)
{
    real32_T out = fallback;

    if ((value >= 0.0f) && (value <= hi))
    {
        out = value;
    }
    return out;
}

/* value limited to +/- lim; a NaN gives 0 */
static real32_T flightTask_clampSym(real32_T value, real32_T lim)
{
    real32_T out = 0.0f;

    if (value > lim)
    {
        out = lim;
    }
    else if (value < -lim)
    {
        out = -lim;
    }
    else if ((value >= -lim) && (value <= lim))
    {
        out = value;
    }
    else
    {
        /* NaN */
    }
    return out;
}

static real32_T flightTask_abs(real32_T x)
{
    return (x < 0.0f) ? -x : x;
}

/* One controller tick on a fresh navigation sample. */
static void flightTask_tick(const NavState_t *nav)
{
    static ctrl_params_t s_params = FLIGHT_CTRL_PARAMS_REF;
    static ctrl_state_t  s_state;                   /* zero-initialised = ctrl_reset */

    real32_T phi_ist[3];
    real32_T phi_soll[3] = { 0.0f, 0.0f, 0.0f };
    real32_T om_soll[3]  = { 0.0f, 0.0f, 0.0f };
    real32_T tau_soll[3] = { 0.0f, 0.0f, 0.0f };
    real32_T w_cmd[4]    = { 0.0f, 0.0f, 0.0f, 0.0f };
    real32_T rpm[4]      = { 0.0f, 0.0f, 0.0f, 0.0f };
    uint32   status      = 0u;
    uint8    i;

    const boolean  drive   = ((g_xcpCal.ctrlMode == MOTOR_CTRL_DRIVE)
                              && (g_xcpCal.motorManual != MOTOR_CMD_MANUAL)) ? TRUE : FALSE;
    const real32_T tiltMax = FLIGHTTASK_DEG_TO_RAD
                           * flightTask_range(g_xcpCal.ctrlTiltMaxDeg, 90.0f, FLIGHTTASK_TILT_MAX_DEG);
    const real32_T rpmMinW = flightTask_range(g_xcpCal.ctrlRpmMin, FLIGHTTASK_RPM_MIN_LIMIT, FLIGHTTASK_RPM_MIN);
    const real32_T rpmMin  = (rpmMinW >= FLIGHTTASK_RPM_MIN_LOWEST) ? rpmMinW : FLIGHTTASK_RPM_MIN;
    const real32_T thrust  = flightTask_range(g_xcpCal.ctrlThrustN, FLIGHTTASK_THRUST_LIMIT_N, 0.0f);
    boolean        navOk   = TRUE;

    phi_ist[0] = nav->ahrs.rollRad;
    phi_ist[1] = nav->ahrs.pitchRad;
    phi_ist[2] = nav->ahrs.yawRad;

    if ((nav->imuPresent == 0u) || (nav->ahrs.state != (uint8)AHRS_RUNNING))
    {
        navOk = FALSE;
    }
    for (i = 0u; i < 3u; i++)
    {
        /* negated "inside the band" so that a NaN counts as outside */
        if (!(flightTask_abs(phi_ist[i]) <= FLIGHTTASK_ANGLE_LIMIT)
            || !(flightTask_abs(nav->ahrs.rate[i]) <= FLIGHTTASK_RATE_LIMIT))
        {
            navOk = FALSE;
        }
    }

    if (navOk == FALSE)
    {
        status |= CTRLLATCH_ST_NAV_INVALID;
    }
    else if ((flightTask_abs(phi_ist[0]) > tiltMax) || (flightTask_abs(phi_ist[1]) > tiltMax))
    {
        status |= CTRLLATCH_ST_TILT;
    }
    else
    {
        /* estimate usable and inside the tilt limit */
    }

    if (status == 0u)
    {
        for (i = 0u; i < 3u; i++)
        {
            s_params.om_kP[i]  = flightTask_range(g_xcpCal.ctrlOmKp[i],  FLIGHTTASK_OM_KP_LIMIT,  0.0f);
            s_params.tau_kP[i] = flightTask_range(g_xcpCal.ctrlTauKp[i], FLIGHTTASK_TAU_KP_LIMIT, 0.0f);
            s_params.tau_kI[i] = flightTask_range(g_xcpCal.ctrlTauKi[i], FLIGHTTASK_TAU_KI_LIMIT, 0.0f);
        }
        /* the IMU clocks this task (1014 Hz), not a fixed 1 ms */
        s_params.Ts = ((nav->dtS >= FLIGHTTASK_TS_MIN_S) && (nav->dtS <= FLIGHTTASK_TS_MAX_S))
                    ? nav->dtS : 0.001f;

        /* attitude setpoint from the operator, roll/pitch never beyond the tilt limit */
        phi_soll[0] = flightTask_clampSym(g_xcpCal.ctrlAttSpDeg[0] * FLIGHTTASK_DEG_TO_RAD, tiltMax);
        phi_soll[1] = flightTask_clampSym(g_xcpCal.ctrlAttSpDeg[1] * FLIGHTTASK_DEG_TO_RAD, tiltMax);
        phi_soll[2] = phi_ist[2];       /* no heading setpoint: yaw rate setpoint 0 */
        {
            const real32_T yawSpDeg = g_xcpCal.ctrlAttSpDeg[2];

            if ((yawSpDeg >= 0.0f) && (yawSpDeg <= 360.0f))
            {
                /* shortest way round: the error the attitude loop sees is within +/- pi */
                real32_T e = (yawSpDeg * FLIGHTTASK_DEG_TO_RAD) - phi_ist[2];

                if (e > FLIGHTTASK_PI)
                {
                    e -= 2.0f * FLIGHTTASK_PI;
                }
                else if (e < -FLIGHTTASK_PI)
                {
                    e += 2.0f * FLIGHTTASK_PI;
                }
                else
                {
                    /* already the short way */
                }
                phi_soll[2] = phi_ist[2] + e;
            }
        }

        att_ctrl_step(phi_soll, phi_ist, om_soll, &s_params);
        rate_ctrl_step(om_soll, nav->ahrs.rate, tau_soll, &s_state, &s_params);
        mixer_step(thrust, tau_soll, w_cmd, &s_params);

        for (i = 0u; i < 4u; i++)
        {
            rpm[i] = w_cmd[i] * FLIGHTTASK_RAD_S_TO_RPM;
            if (rpm[i] < rpmMin)
            {
                rpm[i] = rpmMin;        /* keep every rotor turning; 0 would be a motor stop */
            }
        }
    }

    if ((drive != FALSE) && (status == 0u))
    {
        status |= CTRLLATCH_ST_DRIVE;
    }
    else
    {
        ctrl_reset(&s_state);           /* shadow / unusable: the integrator must not wind up */
    }

    CtrlLatch_publish(rpm, tau_soll, om_soll, s_state.tau_I, phi_soll, status);
}

void FlightTask_step(void)
{
    static NavState_t s_nav;            /* static: ~200 bytes, not on the stack */
    static uint32     s_lastGen;

    if ((NavState_get(&s_nav) != FALSE) && (s_nav.gen != s_lastGen))
    {
        s_lastGen = s_nav.gen;
        flightTask_tick(&s_nav);
    }
}
