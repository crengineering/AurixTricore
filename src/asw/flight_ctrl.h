/* flight_ctrl.h
 *
 * Kaskadenregler fuer Quadrocopter, C99, float.
 * Vier Stufen, jeweils eine Funktion:
 *
 *   pos_ctrl_step   Position NED -> Gesamtschub T und Lage-Sollwinkel
 *   att_ctrl_step   Lage-Sollwinkel -> Soll-Drehraten
 *   rate_ctrl_step  Soll-Drehraten -> Soll-Momente   (einziger Zustand: I-Anteil)
 *   mixer_step      Schub + Momente -> vier Motordrehzahlen
 *
 * Konventionen:
 *   NED-Frame: z zeigt nach unten, Hoehe = -z.
 *   Winkel [phi, theta, psi] = [Roll, Pitch, Yaw] in rad.
 *   Drehraten [p, q, r] in rad/s (Body-Frame).
 *   Reglerverstaerkungen sind zeitkontinuierlich; Ts geht nur in den
 *   I-Anteil des Ratenreglers ein (Backward Euler).
 */

#ifndef FLIGHT_CTRL_H
#define FLIGHT_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

typedef float real32_T;

/* ------------------------------------------------------------------ */
/* Parameter (konstant zur Laufzeit)                                   */
/* ------------------------------------------------------------------ */

typedef struct {
    /* Physik */
    real32_T m;                 /* kg      Abflugmasse                 */
    real32_T g;                 /* m/s^2   Erdbeschleunigung           */
    real32_T w_max;             /* rad/s   max. Rotordrehzahl          */

    /* Abtastzeit */
    real32_T Ts;                /* s       Reglertakt                  */

    /* Positionsregler horizontal (x, y) */
    real32_T ned_xy_kP;         /* 1/s^2                               */
    real32_T ned_xy_kD;         /* 1/s                                 */

    /* Positionsregler vertikal (z) */
    real32_T ned_z_kP;          /* N/m                                 */
    real32_T ned_z_kD;          /* N/(m/s)                             */

    /* Lageregler */
    real32_T om_kP[3];          /* 1/s                                 */

    /* Ratenregler */
    real32_T tau_kP[3];         /* Nm/(rad/s)                          */
    real32_T tau_kI[3];         /* Nm/rad                              */
    real32_T tau_sat_up[3];     /* Nm   obere Stellgrenze              */
    real32_T tau_sat_low[3];    /* Nm   untere Stellgrenze             */

    /* Mixer: invertierte Mischermatrix, Zeilen-Major.
     * [w1^2 w2^2 w3^2 w4^2]^T = MIX_inv * [T tau_x tau_y tau_z]^T   */
    real32_T MIX_inv[16];
} ctrl_params_t;

/* ------------------------------------------------------------------ */
/* Zustand (wird von rate_ctrl_step veraendert)                        */
/* ------------------------------------------------------------------ */

typedef struct {
    real32_T tau_I[3];          /* Nm   Integratorzustand Ratenregler  */
} ctrl_state_t;

/* ------------------------------------------------------------------ */
/* Funktionen                                                          */
/* ------------------------------------------------------------------ */

/* Zustand auf null setzen. Vor dem ersten Takt aufrufen. */
void ctrl_reset(ctrl_state_t *st);

/* Stufe 1: Positionsregler.
 *   p_ned_soll  [3]  Sollposition   [x, y, z] in m (NED)
 *   p_ned_ist   [3]  Istposition    [x, y, z] in m (NED)
 *   v_b_ist     [3]  Geschwindigkeit [u, v, w] in m/s (Body, siehe Hinweis)
 *   psi_soll         Soll-Gierwinkel in rad (unabhaengige Fuehrungsgroesse)
 *   T_soll      out  Gesamtschub in N
 *   phi_soll    [3] out  Soll-Lagewinkel [phi, theta, psi] in rad
 *
 * Hinweis: v_b_ist wird als Daempfungsterm verwendet. Streng genommen
 * muesste die Geschwindigkeit im NED-Frame vorliegen (R * v_b). Fuer
 * kleine Winkel und psi ~ 0 ist die Naeherung zulaessig.
 */
void pos_ctrl_step(const real32_T p_ned_soll[3],
                   const real32_T p_ned_ist[3],
                   const real32_T v_b_ist[3],
                   real32_T psi_soll,
                   real32_T *T_soll,
                   real32_T phi_soll[3],
                   const ctrl_params_t *p);

/* Stufe 2: Lageregler. Proportional, zustandsfrei. */
void att_ctrl_step(const real32_T phi_soll[3],
                   const real32_T phi_ist[3],
                   real32_T om_soll[3],
                   const ctrl_params_t *p);

/* Stufe 3: Ratenregler. PI mit Backward Euler und Clamping-Anti-Windup. */
void rate_ctrl_step(const real32_T om_soll[3],
                    const real32_T om_ist[3],
                    real32_T tau_soll[3],
                    ctrl_state_t *st,
                    const ctrl_params_t *p);

/* Stufe 4: Mixer. Invertierte Mischermatrix, Begrenzung auf [0, w_max^2],
 * anschliessend Wurzel. */
void mixer_step(real32_T T_soll,
                const real32_T tau_soll[3],
                real32_T w_cmd[4],
                const ctrl_params_t *p);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_CTRL_H */
