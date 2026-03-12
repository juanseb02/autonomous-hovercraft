#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>

// ── Servo ─────────────────────────────────────────────────────────────────────
void servo_init(void);
void servo_write_us(int us);
void set_servo_angle(int16_t angle);

// ── Fans ──────────────────────────────────────────────────────────────────────
void setupFans(void);
void runFans(uint8_t thrust, uint8_t lift);
void stopFans(void);

// ── High-level Motion ─────────────────────────────────────────────────────────

// Halt thrust, blink LED forever (maze complete)
void finish_stop(void);

// Rotate in place by delta_deg (positive = left, negative = right)
void turn_by_yaw(float delta_deg);

// Drive forward advance_cm then turn
void advance_then_turn(bool goLeft, float advance_cm);

// Drive straight until front IR detects a wall within OBSTACLE_THRESHOLD_CM
void drive_straight_until_wall(void);

// Scan left/right with IR and choose which way to turn
void handle_intersection(void);

#endif // MOTION_H
