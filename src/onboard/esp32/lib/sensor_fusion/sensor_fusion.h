#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <MPU6050.h>

void sensor_fuse();
extern float past_accelerations[3];
#endif