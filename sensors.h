#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

void adc_init(void);

// Returns distance in cm from the front IR sensor (max IR_MAX_CM)
float ir_front_distance_cm(void);

// Returns distance in cm from the upward-facing ultrasonic sensor, or -1 on error
float upbar_distance_cm(void);

// Returns true when the upbar is consistently detected within UPBAR_THRESHOLD_CM
bool upbar_detected(void);

#endif // SENSORS_H
