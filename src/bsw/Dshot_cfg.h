/*
 *  Code from crengineering
 *  Last modified: 19.09.2026
 *  Last Modified by: crengineering
 */

#ifndef DSHOT_CFG_H
#define DSHOT_CFG_H


/* ---------------------------------------------------------------------------
 * Per-pin Dshot motor configuration
 * ------------------------------------------------------------------------ */

typedef enum {
    DSHOT_M1 = 0,
    DSHOT_M2,
    DSHOT_M3,
    DSHOT_M4,
    DSHOT_MEND
} Dshot_Motor_t;

typedef struct
{
    IfxGtm_Atom_Ch       atomChannel;
    IfxGtm_Atom_ToutMap *pin;
} Dshot_MotorCfg;

/*                    ATOM Channel      &PIN OUT                           */
#define DSHOT_MOTOR_CFG                                                    \
{                                                                          \
    [DSHOT_M1]  = { IfxGtm_Atom_Ch_0,  &IfxGtm_ATOM0_0_TOUT48_P22_1_OUT},  \
    [DSHOT_M2]  = { IfxGtm_Atom_Ch_1,  &IfxGtm_ATOM0_1_TOUT47_P22_0_OUT},  \
    [DSHOT_M3]  = { IfxGtm_Atom_Ch_3,  &IfxGtm_ATOM0_3_TOUT49_P22_2_OUT},  \
    [DSHOT_M4]  = { IfxGtm_Atom_Ch_4,  &IfxGtm_ATOM0_4_TOUT50_P22_3_OUT},  \
}
#endif /* DSHOT_CFG_H */
