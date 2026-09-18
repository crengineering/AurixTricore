/*
 *  Code from crengineering
 *  Last modified: 16.09.2026
 *  Last Modified by: crengineering
 */

#ifndef DSHOT_H
#define DSHOT_H

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "Ifx_Types.h"
#include "IfxPort.h"

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/
typedef struct
{
  sint8  temperature;
  uint16 voltage;
  uint16 current;
  uint16 mAh;
  uint16 eRPM;
}Esc_telemetry;


/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

void Dshot_init(void);
void Dshot_task(void);
/* Copies the last CRC-valid telemetry packet; returns the driver's CRC-ok packet
 * count so the caller can tell whether a new packet arrived since it last asked. */
uint32 Dshot_getTelemetry(Esc_telemetry *out);

#endif /* DSHOT_H */
