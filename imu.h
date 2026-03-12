#ifndef IMU_H
#define IMU_H

#include <stdint.h>

// Integrated yaw angle in degrees (updated by imu_update)
extern volatile float yaw_deg;

void twi_init(void);
void mpu_init(void);
void calibrate_gyro(void);

// Call frequently in main loop to integrate gyro into yaw_deg
void imu_update(void);

#endif // IMU_H
