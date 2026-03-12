#ifndef PID_H
#define PID_H

// Reset PID integrator and derivative state (call before a new straight run)
void pid_reset(void);

// Compute PID correction from yaw error (degrees) and command servo accordingly
void servo_from_yaw_error(float err_deg);

#endif // PID_H
