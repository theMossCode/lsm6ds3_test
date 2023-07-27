#ifndef IMU_H
#define IMU_H

#include <stdint.h>

int imu_init(double odr);
int fetch_imu_accl(float *x, float *y, float *z);

#endif