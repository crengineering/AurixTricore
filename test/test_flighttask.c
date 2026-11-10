#include <math.h>
#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "../src/asw/FlightTask.h"
#include "../src/bsw/NavState.h"
#include "../src/bsw/CtrlLatch.h"
#include "../src/bsw/Diagnostics.h"

/* FlightTask: the flight controller on the real signal path (attitude + rate
 * loop + mixer, stage 1 for the attitude rig). The test plays NavTask
 * (NavState_publish) and reads what the controller hands to CPU0 (CtrlLatch).
 *
 * The SIGN tests are the point: a wrong sign between estimate, controller and
 * mixer flips the aircraft on the rig. Motor arrangement, seen from above
 * (confirmed on the frame 2026-11-02):
 *   M1 front-right CCW, M2 rear-right CW, M3 rear-left CCW, M4 front-left CW.
 * AHRS: roll + = right side down, pitch + = nose up. */

volatile Xcp_Cal g_xcpCal;                 /* Diagnostics.c is target code */

#define M1 0
#define M2 1
#define M3 2
#define M4 3

#define HOVER_N        (1.20f * 9.81f)
#define HOVER_RPM      (4805.0f)           /* sqrt(HOVER_N * 2.1515e4) * 60 / 2pi */

static Ahrs_Values  s_ahrs;
static FusionValues s_fusion;
static CtrlLatch_t  s_out;

/* one IMU sample through NavTask's publish and the controller */
static void tick(void)
{
    static const float32 zero[3] = { 0.0f, 0.0f, 0.0f };

    NavState_publish(&s_ahrs, &s_fusion, 0.000986f, TRUE, zero, zero, 25.0f, 0.0f);
    FlightTask_step();
    TEST_ASSERT_TRUE(CtrlLatch_get(&s_out));
}

void setUp(void)
{
    s_ahrs.rollRad  = 0.0f;
    s_ahrs.pitchRad = 0.0f;
    s_ahrs.yawRad   = 1.0f;
    s_ahrs.rate[0]  = 0.0f;
    s_ahrs.rate[1]  = 0.0f;
    s_ahrs.rate[2]  = 0.0f;
    s_ahrs.state    = (uint8)AHRS_RUNNING;

    g_xcpCal.motorManual    = 0u;
    g_xcpCal.ctrlMode       = 0u;          /* shadow: also resets the integrator on the next tick */
    g_xcpCal.ctrlThrustN    = HOVER_N;
    g_xcpCal.ctrlTiltMaxDeg = 30.0f;
    g_xcpCal.ctrlRpmMin     = 1300.0f;
    g_xcpCal.ctrlAttSpDeg[0] = 0.0f;
    g_xcpCal.ctrlAttSpDeg[1] = 0.0f;
    g_xcpCal.ctrlAttSpDeg[2] = MOTOR_CTRL_YAW_HOLD;
    for (uint32 i = 0u; i < 3u; i++)
    {
        g_xcpCal.ctrlOmKp[i]  = 3.0f;
        g_xcpCal.ctrlTauKp[i] = 0.05f;
        g_xcpCal.ctrlTauKi[i] = 0.0625f;
    }
    tick();
}

void tearDown(void)
{
}

void test_level_hover_all_equal(void)
{
    for (uint32 i = 0u; i < 4u; i++)
    {
        TEST_ASSERT_FLOAT_WITHIN(5.0f, HOVER_RPM, s_out.rpm[i]);
    }
    for (uint32 i = 0u; i < 3u; i++)
    {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.tau[i]);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.omSoll[i]);
    }
    TEST_ASSERT_EQUAL_UINT32(0u, s_out.status);
}

void test_right_side_down_speeds_up_the_right_motors(void)
{
    s_ahrs.rollRad = 0.1f;
    tick();
    TEST_ASSERT_TRUE(s_out.omSoll[0] < 0.0f);
    TEST_ASSERT_TRUE(s_out.tau[0] < 0.0f);
    TEST_ASSERT_TRUE(s_out.rpm[M1] > HOVER_RPM);
    TEST_ASSERT_TRUE(s_out.rpm[M2] > HOVER_RPM);
    TEST_ASSERT_TRUE(s_out.rpm[M3] < HOVER_RPM);
    TEST_ASSERT_TRUE(s_out.rpm[M4] < HOVER_RPM);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, s_out.rpm[M1], s_out.rpm[M2]);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, s_out.rpm[M3], s_out.rpm[M4]);
}

void test_nose_up_speeds_up_the_rear_motors(void)
{
    s_ahrs.pitchRad = 0.1f;
    tick();
    TEST_ASSERT_TRUE(s_out.tau[1] < 0.0f);
    TEST_ASSERT_TRUE(s_out.rpm[M2] > HOVER_RPM);
    TEST_ASSERT_TRUE(s_out.rpm[M3] > HOVER_RPM);
    TEST_ASSERT_TRUE(s_out.rpm[M1] < HOVER_RPM);
    TEST_ASSERT_TRUE(s_out.rpm[M4] < HOVER_RPM);
}

void test_rate_is_damped(void)
{
    s_ahrs.rate[0] = 1.0f;                 /* rolling right-side-down, still level */
    tick();
    TEST_ASSERT_TRUE(s_out.tau[0] < 0.0f);
    TEST_ASSERT_TRUE(s_out.rpm[M1] > s_out.rpm[M4]);

    s_ahrs.rate[0] = 0.0f;
    s_ahrs.rate[1] = 1.0f;                 /* pitching nose-up */
    tick();
    TEST_ASSERT_TRUE(s_out.tau[1] < 0.0f);
    TEST_ASSERT_TRUE(s_out.rpm[M2] > s_out.rpm[M1]);
}

/* CW yaw rate (r > 0, NED) must be answered with CCW torque: the CW rotors
 * (M2, M4) speed up -- their reaction torque on the frame is CCW. */
void test_yaw_rate_speeds_up_the_cw_rotors(void)
{
    s_ahrs.rate[2] = 1.0f;
    tick();
    TEST_ASSERT_TRUE(s_out.tau[2] < 0.0f);
    TEST_ASSERT_TRUE(s_out.rpm[M2] > s_out.rpm[M1]);
    TEST_ASSERT_TRUE(s_out.rpm[M4] > s_out.rpm[M3]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.omSoll[2]);          /* heading itself is not held */

    g_xcpCal.ctrlTauKp[2] = 0.0f;                           /* rig default: yaw off */
    g_xcpCal.ctrlTauKi[2] = 0.0f;
    tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.tau[2]);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, s_out.rpm[M1], s_out.rpm[M2]);
}

void test_integrator_runs_only_in_drive(void)
{
    s_ahrs.rollRad = 0.1f;
    for (uint32 n = 0u; n < 200u; n++) { tick(); }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.tauI[0]);            /* shadow */
    TEST_ASSERT_EQUAL_UINT32(0u, s_out.status & CTRLLATCH_ST_DRIVE);

    g_xcpCal.ctrlMode = MOTOR_CTRL_DRIVE;
    for (uint32 n = 0u; n < 200u; n++) { tick(); }
    TEST_ASSERT_TRUE(s_out.tauI[0] < -0.001f);               /* e = -0.3 rad/s, kI 0.0625, 0.2 s */
    TEST_ASSERT_FLOAT_WITHIN(0.0005f, -0.3f * 0.0625f * 200.0f * 0.000986f, s_out.tauI[0]);
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_DRIVE, s_out.status);

    g_xcpCal.motorManual = MOTOR_CMD_MANUAL;                 /* manual wins: back to shadow */
    tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.tauI[0]);
    TEST_ASSERT_EQUAL_UINT32(0u, s_out.status);
}

void test_tilt_limit(void)
{
    g_xcpCal.ctrlMode = MOTOR_CTRL_DRIVE;
    s_ahrs.rollRad = 0.51f;                /* 29.2 deg */
    tick();
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_DRIVE, s_out.status);

    s_ahrs.rollRad = -0.54f;               /* 30.9 deg */
    tick();
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_TILT, s_out.status);
    for (uint32 i = 0u; i < 4u; i++) { TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.rpm[i]); }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.tauI[0]);

    s_ahrs.rollRad  = 0.0f;
    s_ahrs.pitchRad = 0.54f;
    tick();
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_TILT, s_out.status);

    g_xcpCal.ctrlTiltMaxDeg = NAN;         /* unusable limit reads as 30 deg */
    tick();
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_TILT, s_out.status);
    s_ahrs.pitchRad = 0.51f;
    tick();
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_DRIVE, s_out.status);
}

void test_invalid_estimate(void)
{
    static const float32 zero[3] = { 0.0f, 0.0f, 0.0f };

    g_xcpCal.ctrlMode = MOTOR_CTRL_DRIVE;

    s_ahrs.state = (uint8)AHRS_NO_SENSOR;
    tick();
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_NAV_INVALID, s_out.status);
    for (uint32 i = 0u; i < 4u; i++) { TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.rpm[i]); }

    s_ahrs.state   = (uint8)AHRS_RUNNING;
    s_ahrs.rollRad = NAN;
    tick();
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_NAV_INVALID, s_out.status);

    s_ahrs.rollRad = 0.0f;
    s_ahrs.rate[1] = NAN;
    tick();
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_NAV_INVALID, s_out.status);

    s_ahrs.rate[1] = 0.0f;
    NavState_publish(&s_ahrs, &s_fusion, 0.000986f, FALSE, zero, zero, 25.0f, 0.0f);   /* IMU absent */
    FlightTask_step();
    TEST_ASSERT_TRUE(CtrlLatch_get(&s_out));
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_NAV_INVALID, s_out.status);

    tick();
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_DRIVE, s_out.status);
}

void test_idle_floor_and_bad_calibration(void)
{
    g_xcpCal.ctrlThrustN = 0.0f;
    tick();
    for (uint32 i = 0u; i < 4u; i++) { TEST_ASSERT_EQUAL_FLOAT(1300.0f, s_out.rpm[i]); }

    g_xcpCal.ctrlRpmMin = 5000.0f;         /* out of range reads as 1300 */
    tick();
    TEST_ASSERT_EQUAL_FLOAT(1300.0f, s_out.rpm[M1]);
    g_xcpCal.ctrlRpmMin = 0.0f;            /* a floor of 0 would be a motor stop: reads as 1300 */
    tick();
    TEST_ASSERT_EQUAL_FLOAT(1300.0f, s_out.rpm[M1]);
    g_xcpCal.ctrlRpmMin = 2000.0f;         /* a sensible floor is taken */
    tick();
    TEST_ASSERT_EQUAL_FLOAT(2000.0f, s_out.rpm[M1]);
    g_xcpCal.ctrlRpmMin = 1300.0f;

    g_xcpCal.ctrlThrustN = 41.0f;          /* beyond the thrust range reads as 0 N */
    tick();
    TEST_ASSERT_EQUAL_FLOAT(1300.0f, s_out.rpm[M1]);

    g_xcpCal.ctrlThrustN  = HOVER_N;       /* a gain a thousand times too high reads as 0 */
    g_xcpCal.ctrlTauKp[1] = 50.0f;
    g_xcpCal.ctrlTauKi[1] = 50.0f;
    s_ahrs.pitchRad = 0.1f;
    tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.tau[1]);
    g_xcpCal.ctrlTauKp[1] = 0.05f;
    g_xcpCal.ctrlTauKi[1] = 0.0625f;
    s_ahrs.pitchRad = 0.0f;

    g_xcpCal.ctrlThrustN  = NAN;           /* reads as 0 N */
    g_xcpCal.ctrlTauKp[0] = NAN;           /* reads as 0 */
    g_xcpCal.ctrlTauKi[0] = -1.0f;         /* reads as 0 */
    s_ahrs.rollRad = 0.1f;
    tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.tau[0]);
    for (uint32 i = 0u; i < 4u; i++) { TEST_ASSERT_EQUAL_FLOAT(1300.0f, s_out.rpm[i]); }
}

/* Operator setpoints: +roll setpoint = right side down wanted -> the LEFT motors
 * speed up; +pitch = nose up wanted -> the FRONT motors speed up. */
void test_roll_pitch_setpoints(void)
{
    g_xcpCal.ctrlAttSpDeg[0] = 10.0f;
    tick();
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 0.1745329f, s_out.phiSoll[0]);
    TEST_ASSERT_TRUE(s_out.omSoll[0] > 0.0f);
    TEST_ASSERT_TRUE(s_out.rpm[M3] > s_out.rpm[M2]);
    TEST_ASSERT_TRUE(s_out.rpm[M4] > s_out.rpm[M1]);

    s_ahrs.rollRad = 0.1745329f;           /* arrived: no demand left */
    tick();
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 0.0f, s_out.omSoll[0]);

    s_ahrs.rollRad = 0.0f;
    g_xcpCal.ctrlAttSpDeg[0] = 0.0f;
    g_xcpCal.ctrlAttSpDeg[1] = 10.0f;
    tick();
    TEST_ASSERT_TRUE(s_out.omSoll[1] > 0.0f);
    TEST_ASSERT_TRUE(s_out.rpm[M1] > s_out.rpm[M2]);
    TEST_ASSERT_TRUE(s_out.rpm[M4] > s_out.rpm[M3]);

    /* never beyond the tilt limit; NaN reads as level */
    g_xcpCal.ctrlAttSpDeg[0] = 80.0f;
    g_xcpCal.ctrlAttSpDeg[1] = -80.0f;
    tick();
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f,  0.5235988f, s_out.phiSoll[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, -0.5235988f, s_out.phiSoll[1]);
    g_xcpCal.ctrlAttSpDeg[0] = NAN;
    tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.phiSoll[0]);
}

/* Heading setpoint: 0..360 deg is held the short way round; anything else keeps
 * the current heading (rate setpoint 0). yaw is 1.0 rad = 57.3 deg in setUp. */
void test_yaw_setpoint(void)
{
    tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.omSoll[2]);                  /* -1: hold */
    TEST_ASSERT_EQUAL_FLOAT(1.0f, s_out.phiSoll[2]);

    g_xcpCal.ctrlAttSpDeg[2] = 90.0f;                                /* 32.7 deg to the right */
    tick();
    TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 3.0f * (1.5707963f - 1.0f), s_out.omSoll[2]);
    TEST_ASSERT_TRUE(s_out.tau[2] > 0.0f);                           /* CW torque: CCW rotors up */
    TEST_ASSERT_TRUE(s_out.rpm[M1] > s_out.rpm[M2]);

    s_ahrs.yawRad = 6.1086524f;                                      /* 350 deg, setpoint 10 deg */
    g_xcpCal.ctrlAttSpDeg[2] = 10.0f;
    tick();
    TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 3.0f * 0.3490659f, s_out.omSoll[2]);    /* +20 deg, not -340 */

    s_ahrs.yawRad = 0.1745329f;                                      /* 10 deg, setpoint 350 deg */
    g_xcpCal.ctrlAttSpDeg[2] = 350.0f;
    tick();
    TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, -3.0f * 0.3490659f, s_out.omSoll[2]);

    g_xcpCal.ctrlAttSpDeg[2] = 400.0f;                               /* not a heading: hold */
    tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.omSoll[2]);
    g_xcpCal.ctrlAttSpDeg[2] = NAN;
    tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s_out.omSoll[2]);
}

void test_no_new_sample_no_publish(void)
{
    const uint32 gen = g_ctrlLatch.gen;

    FlightTask_step();
    FlightTask_step();
    TEST_ASSERT_EQUAL_UINT32(gen, g_ctrlLatch.gen);
    tick();
    TEST_ASSERT_EQUAL_UINT32(gen + 2u, g_ctrlLatch.gen);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_level_hover_all_equal);
    RUN_TEST(test_right_side_down_speeds_up_the_right_motors);
    RUN_TEST(test_nose_up_speeds_up_the_rear_motors);
    RUN_TEST(test_rate_is_damped);
    RUN_TEST(test_yaw_rate_speeds_up_the_cw_rotors);
    RUN_TEST(test_integrator_runs_only_in_drive);
    RUN_TEST(test_tilt_limit);
    RUN_TEST(test_invalid_estimate);
    RUN_TEST(test_idle_floor_and_bad_calibration);
    RUN_TEST(test_roll_pitch_setpoints);
    RUN_TEST(test_yaw_setpoint);
    RUN_TEST(test_no_new_sample_no_publish);
    return UNITY_END();
}
