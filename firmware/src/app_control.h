/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app_control.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_CONTROL_Initialize" and "APP_CONTROL_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_CONTROL_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef _APP_CONTROL_H
#define _APP_CONTROL_H

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
    APP_CONTROL_STATE_INIT=0,
    APP_CONTROL_STATE_SERVICE_TASKS,
} APP_CONTROL_STATES;

typedef struct
{
    APP_CONTROL_STATES state;
} APP_CONTROL_DATA;

void APP_CONTROL_Initialize ( void );
void APP_CONTROL_Tasks( void );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _APP_CONTROL_H */
