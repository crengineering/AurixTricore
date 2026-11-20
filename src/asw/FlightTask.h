/* FlightTask.h
 *
 * The flight controller on the real signal path: attitude + rate loop + mixer
 * (flight_ctrl.c) fed from the navigation state, once per IMU sample, on the
 * flight core (CPU1). Output: the four rotor-speed requests in the CtrlLatch
 * (LMU), which Task_Dshot (CPU0) hands to Motor_task.
 *
 * Stage 1 (attitude rig): no position/altitude loop. Collective thrust is the
 * calibration word ctrlThrustN, the attitude setpoint is ctrlAttSpDeg[] (after
 * every release: level, keep the heading). Loop gains come from Xcp_Cal every tick.
 *
 * The task always computes and publishes (shadow). Whether the request reaches
 * the motors is decided on CPU0 by Xcp_Cal.ctrlMode (motor.c); here ctrlMode
 * only decides whether the rate integrator runs.
 */

#ifndef FLIGHTTASK_H
#define FLIGHTTASK_H

/* Register on CPU1 AFTER NavTask_step, same dispatch period. Does nothing
 * unless NavTask_step has published a new sample since the last call. */
void FlightTask_step(void);

#endif /* FLIGHTTASK_H */
