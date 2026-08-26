/* Kleine Helfer, die mehrere Estimator-Tests brauchen: Zufallszahlen mit
 * festem Startwert (damit ein Fehlschlag reproduzierbar ist), Endlichkeits-
 * pruefungen und 3x3-Lineare-Algebra fuer die Rotations-Invarianten.
 *
 * Deliberately header-only and dependency-free: the tests must not need numpy,
 * BLAS or anything else that is not already in the tree.
 */
#ifndef TEST_UTIL_MATH_H
#define TEST_UTIL_MATH_H

#include <math.h>
#include <stddef.h>

/* ---- deterministic PRNG (xorshift32) ---------------------------------- */

typedef struct { unsigned int s; } Rng;

static inline void rngSeed(Rng *r, unsigned int seed)
{
    r->s = (seed != 0u) ? seed : 0xC0FFEEu;
}

static inline unsigned int rngNext(Rng *r)
{
    unsigned int x = r->s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    r->s = x;
    return x;
}

/** uniform in [lo, hi) */
static inline float rngF(Rng *r, float lo, float hi)
{
    return lo + (hi - lo) * ((float)(rngNext(r) >> 8) / 16777216.0f);
}

/** approximately standard normal (sum of 12 uniforms, Irwin-Hall) */
static inline float rngN(Rng *r)
{
    float s = 0.0f;
    int i;
    for (i = 0; i < 12; ++i) { s += rngF(r, 0.0f, 1.0f); }
    return s - 6.0f;
}

/* ---- finiteness -------------------------------------------------------- */

static inline int isFiniteF(float v)
{
    return isfinite(v) != 0;    /* false for NaN and for both infinities */
}

static inline int allFinite(const float *v, size_t n)
{
    size_t i;
    for (i = 0u; i < n; ++i) { if (!isFiniteF(v[i])) { return 0; } }
    return 1;
}

/* ---- 3x3, row major ---------------------------------------------------- */

static inline float det3(const float m[9])
{
    return m[0] * (m[4] * m[8] - m[5] * m[7])
         - m[1] * (m[3] * m[8] - m[5] * m[6])
         + m[2] * (m[3] * m[7] - m[4] * m[6]);
}

static inline float norm3(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/** Rotation matrix of a body->NED quaternion (w, x, y, z), row major.
 *  Textbook form; nothing in the firmware is consulted for it. */
static inline void quatToR(const float q[4], float R[9])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    R[0] = 1.0f - 2.0f * (y * y + z * z);
    R[1] = 2.0f * (x * y - w * z);
    R[2] = 2.0f * (x * z + w * y);
    R[3] = 2.0f * (x * y + w * z);
    R[4] = 1.0f - 2.0f * (x * x + z * z);
    R[5] = 2.0f * (y * z - w * x);
    R[6] = 2.0f * (x * z - w * y);
    R[7] = 2.0f * (y * z + w * x);
    R[8] = 1.0f - 2.0f * (x * x + y * y);
}

static inline void mat3vec(const float m[9], const float v[3], float o[3])
{
    o[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    o[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    o[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

/** Transpose-times-vector, i.e. the inverse rotation for an orthonormal m. */
static inline void mat3Tvec(const float m[9], const float v[3], float o[3])
{
    o[0] = m[0] * v[0] + m[3] * v[1] + m[6] * v[2];
    o[1] = m[1] * v[0] + m[4] * v[1] + m[7] * v[2];
    o[2] = m[2] * v[0] + m[5] * v[1] + m[8] * v[2];
}

#endif /* TEST_UTIL_MATH_H */
