# Third-Party Components

This repository vendors third-party source so it builds out of the box in AURIX
Development Studio. Every component below is redistributed under a **permissive**
licence that allows source and binary redistribution, and every original licence
header is preserved unmodified in the files themselves.

The MIT licence in [`LICENSE`](LICENSE) applies only to the original work in this
repository — see [Scope](#scope) below.

## Components

| Path | Component | Version | Copyright | Licence (SPDX) | Upstream |
|---|---|---|---|---|---|
| `Libraries/iLLD/` | Infineon Low-Level Driver | iLLD 1.20.0 | © Infineon Technologies AG | `BSL-1.0` | [AURIX Development Studio](https://www.infineon.com/cms/en/tools/aurix-development-studio/) |
| `Libraries/Infra/` | Infineon SSW / SFR infrastructure | iLLD 1.20.0 | © Infineon Technologies AG | `BSL-1.0` | ships with iLLD |
| `Libraries/Service/` | Infineon CpuGeneric service layer | iLLD 1.20.0 | © Infineon Technologies AG | `BSL-1.0` | ships with iLLD |
| `Configurations/` | Infineon-derived startup, PLL init, boot-mode header | iLLD 1.20.0 | © Infineon Technologies AG | `BSL-1.0` | ships with iLLD |
| `Libraries/Ethernet/lwip/src/` | lwIP TCP/IP stack | 2.1.x | © Swedish Institute of Computer Science and contributors | `BSD-3-Clause` | [savannah.nongnu.org/projects/lwip](https://savannah.nongnu.org/projects/lwip/) |
| `Libraries/Ethernet/lwip/port/` | Infineon GETH port for lwIP | — | © Infineon Technologies AG | `BSL-1.0` | ships with iLLD |
| `Libraries/Ethernet/` (PHY) | RTL8211F PHY driver | — | © Infineon Technologies AG | `BSL-1.0` | ships with iLLD |
| `test/unity/` | Unity unit-test framework | 2.7.0 | © 2007-26 Mike Karlesky, Mark VanderVoord, Greg Williams | `MIT` | [ThrowTheSwitch/Unity](https://github.com/ThrowTheSwitch/Unity) |

Full licence texts:

- Boost Software License 1.0 — reproduced in the header of every Infineon file;
  canonical text at <https://www.boost.org/LICENSE_1_0.txt>
- BSD-3-Clause (lwIP) — [`Libraries/Ethernet/lwip/COPYING`](Libraries/Ethernet/lwip/COPYING)
- MIT (Unity) — [`test/unity/LICENSE.txt`](test/unity/LICENSE.txt)

## Scope

`LICENSE` (MIT) covers the original work in this repository:

```
src/          all BSW and ASW sources
tools/        Python validation, A2L generation, MISRA gate
docs/         notes, A2L file, HTML explainers
configs/      GUI plot configurations
test/         unit tests and fakes, EXCLUDING test/unity/
.github/      CI workflows
*.lsl, *.bat  linker scripts and build/flash scripts
```

Everything under `Libraries/` and `Configurations/` is third-party and stays
under the licences listed above. MIT does not, and cannot, relicense it.

## Not redistributed

Infineon datasheets, reference manuals, board user manuals and errata sheets are
**not** included in this repository and are excluded via `.gitignore`. Obtain
them from Infineon. The notes under `docs/` are original summaries written while
working from those documents; they paraphrase behaviour and cite register names
and addresses (functional facts required to program the part) rather than
reproducing vendor text, tables or figures.

## Toolchain

The TASKING VX-toolset for TriCore and AURIX Development Studio are **not**
redistributed here. They are installed separately and used under Infineon's /
TASKING's own terms. No toolchain binaries, headers or licence files are part of
this repository.

## Trademarks

AURIX™, TriCore™ and Infineon® are trademarks of Infineon Technologies AG.
Their use here is descriptive — to identify the hardware this software targets —
and implies no affiliation with, sponsorship by, or endorsement from Infineon
Technologies AG. The same applies to any other product or company name that
appears in this repository.
