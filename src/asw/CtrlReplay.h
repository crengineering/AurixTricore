/* CtrlReplay.h
 *
 * UDP vector-replay harness for the verified flight controller
 * (flight_ctrl.c). Protocol and architecture: docs/CTRL_REPLAY.md.
 *
 * Runs on CPU0 in the lwIP poll context. Called once from core0_main
 * after the lwIP stack is up (documented deviation from the BSW/ASW
 * layering rules — see docs/CTRL_REPLAY.md).
 */

#ifndef CTRLREPLAY_H
#define CTRLREPLAY_H

void CtrlReplay_init(void);

#endif /* CTRLREPLAY_H */
