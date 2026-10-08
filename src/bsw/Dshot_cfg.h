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

typedef struct
{
    IfxGtm_Atom_Ch       atomChannel;
    IfxGtm_Atom_ToutMap *pin;
    IfxGtm_Tim_Ch        timChannel;
    IfxGtm_Tim_TinMap   *pinIn;
} Dshot_MotorCfg;

/*                  ATOM Channel       &PIN OUT                          TIM Channel       &PIN IN*/
#define DSHOT_MOTOR_CFG                                                                                               \
{                                                                                                                     \
    [DSHOT_M1]  = { IfxGtm_Atom_Ch_0,  &IfxGtm_ATOM0_0_TOUT48_P22_1_OUT, IfxGtm_Tim_Ch_0,  &IfxGtm_TIM0_0_P22_1_IN},  \
    [DSHOT_M2]  = { IfxGtm_Atom_Ch_1,  &IfxGtm_ATOM0_1_TOUT47_P22_0_OUT, IfxGtm_Tim_Ch_1,  &IfxGtm_TIM0_1_P22_0_IN},  \
    [DSHOT_M3]  = { IfxGtm_Atom_Ch_3,  &IfxGtm_ATOM0_3_TOUT49_P22_2_OUT, IfxGtm_Tim_Ch_3,  &IfxGtm_TIM0_3_P22_2_IN},  \
    [DSHOT_M4]  = { IfxGtm_Atom_Ch_4,  &IfxGtm_ATOM0_4_TOUT50_P22_3_OUT, IfxGtm_Tim_Ch_4,  &IfxGtm_TIM0_4_P22_3_IN},  \
}
#endif /* DSHOT_CFG_H */
