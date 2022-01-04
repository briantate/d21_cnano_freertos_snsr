/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_display.c

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
#include "app_display.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp/bsp.h"
#include "queue.h"
#include "sensor.h"

extern QueueHandle_t sensorDataQueue;

sensorData_t snsrData;

APP_DISPLAY_DATA app_displayData;
uint32_t blinkRate;

void APP_DISPLAY_Initialize ( void )
{
    app_displayData.state = APP_DISPLAY_STATE_INIT;
    printf("Display Init\r\n");
}

void APP_DISPLAY_Tasks ( void )
{
    static uint32_t measurementCount = 0;

    switch ( app_displayData.state )
    {
        case APP_DISPLAY_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized)
            {

                app_displayData.state = APP_DISPLAY_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_DISPLAY_STATE_SERVICE_TASKS:
        {
            if(NULL != sensorDataQueue)
            {
                xQueueReceive(sensorDataQueue, &snsrData, 1000);
            }
            //wait on queue
            LED_Toggle();
            printf("\033[K");
            printf("%u\r\n", (unsigned int)++measurementCount);           
            printf("\033[K");
            printf("ax:%d\tay:%d\taz:%d\r\n", snsrData.bmi160_accel.x, 
                    snsrData.bmi160_accel.y, 
                    snsrData.bmi160_accel.z);
            printf("\033[K");
            printf("gx:%d\tgy:%d\tgz:%d\r\n", snsrData.bmi160_gyro.x, 
                    snsrData.bmi160_gyro.y, 
                    snsrData.bmi160_gyro.z);
            printf("\033[F");
            printf("\033[F");
            printf("\033[F");
            break;
        }

        default:
        {
            break;
        }
    }
}
