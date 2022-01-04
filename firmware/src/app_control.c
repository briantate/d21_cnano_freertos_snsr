/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_control.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/
#include "app_control.h"
#include "FreeRTOS.h"
#include "task.h"

APP_CONTROL_DATA app_controlData;

void APP_CONTROL_Initialize ( void )
{
    app_controlData.state = APP_CONTROL_STATE_INIT;
}

void APP_CONTROL_Tasks ( void )
{
    vTaskSuspend(NULL); //Temporary, task not currently used

    switch ( app_controlData.state )
    {
        case APP_CONTROL_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized)
            {

                app_controlData.state = APP_CONTROL_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_CONTROL_STATE_SERVICE_TASKS:
        {

            break;
        }

        default:
        {
            break;
        }
    }
}
