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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app_display.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp/bsp.h"
#include "queue.h"
#include "sensor.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
extern QueueHandle_t sensorDataQueue;

sensorData_t snsrData;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_DISPLAY_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DISPLAY_DATA app_displayData;
uint32_t blinkRate;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_DISPLAY_Initialize ( void )

  Remarks:
    See prototype in app_display.h.
 */

void APP_DISPLAY_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_displayData.state = APP_DISPLAY_STATE_INIT;
    blinkRate = 500;
    printf("Display Init\r\n");
}


/******************************************************************************
  Function:
    void APP_DISPLAY_Tasks ( void )

  Remarks:
    See prototype in app_display.h.
 */

void APP_DISPLAY_Tasks ( void )
{
    static uint32_t measurementCount = 0;
    /* Check the application's current state. */
    switch ( app_displayData.state )
    {
        /* Application's initial state. */
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
            printf("%u\r\n", ++measurementCount);           
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

        /* TODO: implement your application state machine.*/


        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
  //  vTaskDelay(blinkRate/portTICK_PERIOD_MS);
}


/*******************************************************************************
 End of File
 */
