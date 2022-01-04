/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _SENSOR_H    /* Guard against multiple inclusion */
#define _SENSOR_H

#include "bosch/bmi160.h"
#include "bosch/bmi160_defs.h"


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif
    
typedef struct sensorDataStructure
{
    struct bmi160_sensor_data bmi160_accel;
    struct bmi160_sensor_data bmi160_gyro;
    struct bmi160_dev bmi160dev;
}sensorData_t;

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _SENSOR_H */

/* *****************************************************************************
 End of File
 */
