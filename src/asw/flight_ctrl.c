/* flight_ctrl.c
 *
 * Implementierung des Kaskadenreglers. Siehe flight_ctrl.h fuer die
 * Schnittstellen und Konventionen.
 */

#include "flight_ctrl.h"
#include <math.h>

/* ------------------------------------------------------------------ */

static real32_T sat(real32_T x, real32_T lo, real32_T hi)
{
    if (x > hi) { return hi; }
    if (x < lo) { return lo; }
    return x;
}

/* ------------------------------------------------------------------ */

void ctrl_reset(ctrl_state_t *st)
{
    st->tau_I[0] = 0.0f;
    st->tau_I[1] = 0.0f;
    st->tau_I[2] = 0.0f;
}

/* ------------------------------------------------------------------ */
/* Stufe 1: Position -> Schub und Lage-Sollwinkel                      */
/* ------------------------------------------------------------------ */

void pos_ctrl_step(const real32_T p_ned_soll[3],
                   const real32_T p_ned_ist[3],
                   const real32_T v_b_ist[3],
                   real32_T psi_soll,
                   real32_T *T_soll,
                   real32_T phi_soll[3],
                   const ctrl_params_t *p)
{
    real32_T e_z, T, T_spec;
    real32_T e_x, e_y, acc_x, acc_y;

    /* --- Hoehe: PD auf den Schub -----------------------------------
     * NED: z zeigt nach unten. Ist der Copter zu tief, gilt
     * z_ist > z_soll, also e_z > 0 -> mehr Schub.
     * Sinkt er (w > 0), bremst der D-Anteil ebenfalls mit mehr Schub.
     * T_hover = m*g wirkt als Vorsteuerung.
     */
    e_z = p_ned_ist[2] - p_ned_soll[2];
    T   = p->m * p->g
        + p->ned_z_kP * e_z
        + p->ned_z_kD * v_b_ist[2];

    /* Schub darf nicht negativ werden. Obergrenze uebernimmt der Mixer. */
    if (T < 0.0f) { T = 0.0f; }
    *T_soll = T;

    /* --- Horizontal: PD auf die Sollbeschleunigung ------------------ */
    e_x = p_ned_soll[0] - p_ned_ist[0];
    e_y = p_ned_soll[1] - p_ned_ist[1];

    acc_x = p->ned_xy_kP * e_x - p->ned_xy_kD * v_b_ist[0];
    acc_y = p->ned_xy_kP * e_y - p->ned_xy_kD * v_b_ist[1];

    /* --- Achsentausch und Schubnormierung ---------------------------
     * Kopplung: x_ddot = -(T/m)*theta,  y_ddot = +(T/m)*phi
     * Umkehrung ergibt den Sollwinkel. Es wird durch den tatsaechlichen
     * spezifischen Schub geteilt, nicht durch g -- sonst uebersteuert
     * die Horizontalbewegung, sobald der Hoehenregler nachdrueckt.
     */
    T_spec = T / p->m;
    if (T_spec < 0.1f * p->g) {        /* Division absichern */
        T_spec = 0.1f * p->g;
    }

    phi_soll[0] =  acc_y / T_spec;     /* Roll  aus y */
    phi_soll[1] = -acc_x / T_spec;     /* Pitch aus x */
    phi_soll[2] =  psi_soll;           /* Yaw unabhaengig vorgegeben */
}

/* ------------------------------------------------------------------ */
/* Stufe 2: Lage -> Soll-Drehraten                                     */
/* ------------------------------------------------------------------ */

void att_ctrl_step(const real32_T phi_soll[3],
                   const real32_T phi_ist[3],
                   real32_T om_soll[3],
                   const ctrl_params_t *p)
{
    int i;
    for (i = 0; i < 3; i++) {
        om_soll[i] = p->om_kP[i] * (phi_soll[i] - phi_ist[i]);
    }
}

/* ------------------------------------------------------------------ */
/* Stufe 3: Drehraten -> Momente (PI, Backward Euler, Clamping)        */
/* ------------------------------------------------------------------ */

void rate_ctrl_step(const real32_T om_soll[3],
                    const real32_T om_ist[3],
                    real32_T tau_soll[3],
                    ctrl_state_t *st,
                    const ctrl_params_t *p)
{
    int i;

    for (i = 0; i < 3; i++) {
        real32_T e, I_new, u, u_sat;

        e = om_soll[i] - om_ist[i];

        /* Backward Euler: I[k] = I[k-1] + kI * Ts * e[k] */
        I_new = st->tau_I[i] + p->tau_kI[i] * p->Ts * e;

        u     = p->tau_kP[i] * e + I_new;
        u_sat = sat(u, p->tau_sat_low[i], p->tau_sat_up[i]);

        /* Clamping: Integrator nur uebernehmen, wenn die Stellgroesse
         * nicht gesaettigt ist oder der Fehler aus der Saettigung
         * herausfuehrt. */
        if ((u == u_sat) || ((e > 0.0f) != (u > u_sat))) {
            st->tau_I[i] = I_new;
        }

        tau_soll[i] = u_sat;
    }
}

/* ------------------------------------------------------------------ */
/* Stufe 4: Mixer                                                      */
/* ------------------------------------------------------------------ */

void mixer_step(real32_T T_soll,
                const real32_T tau_soll[3],
                real32_T w_cmd[4],
                const ctrl_params_t *p)
{
    real32_T wrench[4];
    real32_T w_sq_max;
    int i, j;

    wrench[0] = T_soll;
    wrench[1] = tau_soll[0];
    wrench[2] = tau_soll[1];
    wrench[3] = tau_soll[2];

    w_sq_max = p->w_max * p->w_max;

    for (i = 0; i < 4; i++) {
        real32_T w_sq = 0.0f;
        for (j = 0; j < 4; j++) {
            w_sq += p->MIX_inv[i * 4 + j] * wrench[j];
        }
        w_sq = sat(w_sq, 0.0f, w_sq_max);
        w_cmd[i] = sqrtf(w_sq);
    }
}
