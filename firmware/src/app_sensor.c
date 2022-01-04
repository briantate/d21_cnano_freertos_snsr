/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_sensor.c

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

#include "app_sensor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "driver/driver_common.h"
#include "sensor.h"
#include "driver/i2c/drv_i2c.h"
#include <stdio.h>
#include <stdint.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_SENSOR_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/
#define SNSR_BUF_SIZE 128
#define BMI160_DEV_ADDR 0b1101000

APP_SENSOR_DATA app_sensorData;
DRV_HANDLE sensorHandle;
QueueHandle_t sensorDataQueue;

sensorData_t sensorDataStruct; 

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

static void init_sensor_interface(void);
static void init_bmi160(void);
static void init_bmi160_sensor_driver_interface(void);

int8_t bmi160_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi160_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
void   bmi160_delay_ms(uint32_t period);

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_SENSOR_Initialize ( void )

  Remarks:
    See prototype in app_sensor.h.
 */

void APP_SENSOR_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_sensorData.state = APP_SENSOR_STATE_INIT;
    sensorDataQueue = xQueueCreate( 2, sizeof(sensorData_t));
    printf("sensor task init\r\n");
    if(NULL == sensorDataQueue)
        printf("sensor data queue not created\r\n");
}


/******************************************************************************
  Function:
    void APP_SENSOR_Tasks ( void )

  Remarks:
    See prototype in app_sensor.h.
 */

void APP_SENSOR_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( app_sensorData.state )
    {
        /* Application's initial state. */
        case APP_SENSOR_STATE_INIT:
        {
            init_bmi160_sensor_driver_interface();
            init_sensor_interface();
            vTaskDelay(200/portTICK_PERIOD_MS); //per bosch example code
            init_bmi160();
            app_sensorData.state = APP_SENSOR_STATE_SERVICE_TASKS;
            break;
        }

        case APP_SENSOR_STATE_SERVICE_TASKS:
        {
            bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), 
                    &sensorDataStruct.bmi160_accel, 
                    &sensorDataStruct.bmi160_gyro, 
                    &sensorDataStruct.bmi160dev);

            //send data to queue
            
            if(NULL != sensorDataQueue)
            {
                xQueueSend(sensorDataQueue, &sensorDataStruct, 10);
            }
            
            vTaskDelay(1000/portTICK_PERIOD_MS); //ToDo: synchronize a different way

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
}

static void init_sensor_interface(void)
{
    sensorHandle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (sensorHandle == DRV_HANDLE_INVALID)
        printf("DRV_I2C_OPEN fail\r\n");
}

static void init_bmi160(void)
{
    int8_t rslt;

    rslt = bmi160_init(&sensorDataStruct.bmi160dev);

    if (rslt == BMI160_OK)
    {
        printf("BMI160 initialization success !\r\n");
        printf("Chip ID 0x%X\r\n", sensorDataStruct.bmi160dev.chip_id);
    }
    else
    {
        printf("BMI160 initialization failure !\r\n");
        exit(0); //ToDo implement error codes?
    }

//    /* Select the Output data rate, range of accelerometer sensor */
//    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    sensorDataStruct.bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
//    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    sensorDataStruct.bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    sensorDataStruct.bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
//
//    /* Select the power mode of accelerometer sensor */
    sensorDataStruct.bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
//
//    /* Select the Output data rate, range of Gyroscope sensor */
//    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    sensorDataStruct.bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
    sensorDataStruct.bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    sensorDataStruct.bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
//
//    /* Select the power mode of Gyroscope sensor */
    sensorDataStruct.bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&sensorDataStruct.bmi160dev);
    if (rslt == BMI160_OK)
    {
        printf("BMI160 set config success !\r\n");
    }
    else
    {
        printf("BMI160 set config failure !\r\n");
        exit(0); //ToDo implement error codes?
    }
}

static void init_bmi160_sensor_driver_interface(void)
{
    /* link read/write/delay function of host system to appropriate
    * bmi160 function call prototypes */
    sensorDataStruct.bmi160dev.write = bmi160_write;
    sensorDataStruct.bmi160dev.read = bmi160_read;
    sensorDataStruct.bmi160dev.delay_ms = bmi160_delay_ms;
    sensorDataStruct.bmi160dev.id = BMI160_DEV_ADDR;
    sensorDataStruct.bmi160dev.intf = BMI160_I2C_INTF;
}

#define BMI_RX_BUF_SIZE 64
uint8_t bmiRxBuf[BMI_RX_BUF_SIZE];

int8_t bmi160_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{   
    if(false == DRV_I2C_WriteReadTransfer(sensorHandle, dev_addr, &reg_addr,1, data, (size_t)len))
        return BMI160_E_COM_FAIL;
    else
        return BMI160_OK;
}

int8_t bmi160_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    static uint8_t sensorBuffer[SNSR_BUF_SIZE];
    
    if ((len + 1) > SNSR_BUF_SIZE)
        return BMI160_E_COM_FAIL;
    sensorBuffer[0] = reg_addr;
    memcpy(&sensorBuffer[1], data, len);
    
    if (false == DRV_I2C_WriteTransfer(sensorHandle, dev_addr, sensorBuffer, (size_t)len+1)) 
        return BMI160_E_COM_FAIL;
    else
        return BMI160_OK;
}
void   bmi160_delay_ms(uint32_t period)
{
    vTaskDelay(period/portTICK_PERIOD_MS);
}


/*******************************************************************************
 End of File
 */
