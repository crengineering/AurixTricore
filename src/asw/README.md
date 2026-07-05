# Application Software (ASW)

Vision pipeline sources land here from milestone M1 onwards
(see `docs/OBJECT_DETECTION.md`).

Dependency rules:

1. ASW may call BSW (`src/bsw/`); BSW never includes an ASW header.
2. ASW never includes iLLD/SFR headers directly — if hardware access is
   missing, extend the corresponding BSW service instead.
3. BSW owns CPU0 (communication, cyclic services); ASW compute runs on
   CPU1…CPU5.
4. ASW publishes results via its own XCP block (`Xcp_Vision`,
   0x70030300) and uses the BSW `Nvm` service for persistent settings.
