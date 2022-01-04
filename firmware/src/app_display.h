/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app_display.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_DISPLAY_Initialize" and "APP_DISPLAY_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_DISPLAY_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef _APP_DISPLAY_H
#define _APP_DISPLAY_H

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
    APP_DISPLAY_STATE_INIT=0,
    APP_DISPLAY_STATE_SERVICE_TASKS,
} APP_DISPLAY_STATES;

typedef struct
{
    APP_DISPLAY_STATES state;
} APP_DISPLAY_DATA;

void APP_DISPLAY_Initialize ( void );
void APP_DISPLAY_Tasks( void );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _APP_DISPLAY_H */
