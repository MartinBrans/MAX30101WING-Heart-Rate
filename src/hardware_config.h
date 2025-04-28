
#ifndef _HARDWARE_CONFIG_H
#define _HARDWARE_CONFIG_H

#include "MAX30101.h"
#include "max32630fthr.h"


//IC configuration functions
bool op_sensor_config(MAX30101 &op_sensor);
void pmic_config(I2C & i2c_bus, DigitalOut & pmic_en);

/* Op Sensor FIFO nearly full callback */
// volatile bool op_sensorIntFlag = 0;
// void op_sensor_callback() {op_sensorIntFlag = 1;}
// Not actually useful anymore since the BLE library does the interrupts now

#endif
