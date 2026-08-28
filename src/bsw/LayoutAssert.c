/**********************************************************************************************************************
 * \file LayoutAssert.c
 * \brief Compiles the generated struct-offset guard -- see LayoutAssert_gen.h.
 *
 * This file holds ONE #include and NOTHING else -- no logic. It is the sole
 * translation unit that compiles `LayoutAssert_gen.h` (`tools/gen_a2l.py`,
 * docs/MEMORY_PLACEMENT.md part 6): a generated set of compile-time
 * assertions, one per struct field in every XCP block plus one `sizeof` per
 * block, that fail the build -- naming the exact field, at the exact line --
 * if the compiler's own layout ever disagrees with the offset
 * `tools/gen_a2l.py` assumed when it last wrote `docs/AurixTricore.a2l`.
 *
 * If this file stops being compiled (e.g. excluded from a build config, or
 * `.cproject` narrowed to skip it), the guard silently stops running -- a
 * generated header that nothing includes is not a guard. Keep this file
 * unexcluded.
 *********************************************************************************************************************/
#include "LayoutAssert_gen.h"
