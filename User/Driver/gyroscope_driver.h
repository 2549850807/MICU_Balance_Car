#ifndef __GYROSCOPE_DRIVER_H__
#define __GYROSCOPE_DRIVER_H__

#include "main.h"
#include "icm20608.h"
#include "bno08x_hal.h"

float convert_to_continuous_yaw(float current_yaw);
void PreprocessForMadgwick(Vector3f *gyro, Vector3f *accel);

#endif

