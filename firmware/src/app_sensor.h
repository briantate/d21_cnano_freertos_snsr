/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app_sensor.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_SENSOR_Initialize" and "APP_SENSOR_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_SENSOR_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef _APP_SENSOR_H
#define _APP_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "configuration.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

typedef enum
{
    APP_SENSOR_STATE_INIT=0,
    APP_SENSOR_STATE_SERVICE_TASKS,
} APP_SENSOR_STATES;

typedef struct
{
    APP_SENSOR_STATES state;
} APP_SENSOR_DATA;

void APP_SENSOR_Initialize ( void );
void APP_SENSOR_Tasks( void );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _APP_SENSOR_H */
