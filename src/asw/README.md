# Application Software (ASW)

Flight-control application code. Today that is the cascade controller
(`flight_ctrl.c`) and the replay endpoint that feeds it (`CtrlReplay.c`);
the state estimator lands here as well.

`flight_ctrl.c` is byte-identical to its original in the Quadrocopter
repository — the same translation unit runs as a Simulink S-function, as a
PC reference and here on the target. Never edit it in place to fix a
target-only problem; change it there and copy it across, or the replay
comparison stops meaning anything.

Dependency rules:

1. ASW may call BSW (`src/bsw/`); BSW never includes an ASW header.
2. ASW never includes iLLD/SFR headers directly — if hardware access is
   missing, extend the corresponding BSW service instead.
3. BSW owns CPU0 (communication, cyclic services); ASW compute runs on
   CPU1…CPU5.
4. ASW uses the BSW `Nvm` service for persistent settings. An ASW-owned
   XCP block does not exist yet — `0x70030300` is taken by `Xcp_Gpio`, so
   pick a free address when one is needed and add it to the map in the
   top-level README.

The split exists so the compute half stays host-testable without hardware
(see `docs/TESTING_PLAN.md` §A1).
