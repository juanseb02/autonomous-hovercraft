#ifndef CONFIG_H
#define CONFIG_H

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// ── Pin Definitions ──────────────────────────────────────────────────────────
#define SERVO_PIN        PB1
#define THRUST_FAN_PIN   PD4
#define LIFT_FAN_PIN     PD7
#define LED_DEBUG        PB5
#define US_TRIG          PB3
#define US_ECHO          PD2
#define IR_FRONT_CH      0

// ── Navigation Thresholds ─────────────────────────────────────────────────────
#define PATH_WIDTH_CM          61.0f
#define OBSTACLE_THRESHOLD_CM  60.0f
#define DEADEND_MAX_CM         15.0f
#define MIN_FRONT_CM           10.0f

// ── Upbar Detection ───────────────────────────────────────────────────────────
#define UPBAR_THRESHOLD_CM     25.0f
#define UPBAR_CONFIRM_COUNT    3

// ── Turning ───────────────────────────────────────────────────────────────────
#define TURN_ANGLE_DEG         90.0f
#define TURN_TIMEOUT_MS        3000UL

// ── Fan Duty Cycles (0–100) ───────────────────────────────────────────────────
#define THRUST_CRUISE_DUTY     80
#define THRUST_TURN_DUTY       65
#define THRUST_SCAN_DUTY       0
#define LIFT_DUTY              100

// ── Servo Angles ──────────────────────────────────────────────────────────────
#define ANG_LEFT_DEG           0
#define ANG_MID_DEG            90
#define ANG_RIGHT_DEG          180
#define LEFT_SCAN_ANGLE_DEG    20
#define RIGHT_SCAN_ANGLE_DEG   160

// ── Timing ────────────────────────────────────────────────────────────────────
#define SCAN_SETTLE_MS         600
#define SONAR_PERIOD_MS        60

// ── IR Sensor ─────────────────────────────────────────────────────────────────
#define IR_MAX_CM              80.0f
#define IR_SAMPLES             10

// ── IMU / PID ─────────────────────────────────────────────────────────────────
#define CF_ALPHA               0.98f
#define YAW_DEADBAND_DPS       0.05f
#define SERVO_CENTER_US        1500
#define SERVO_MAX_OFFSET_US    400

#define YAW_KP                 3.0f
#define YAW_KI                 0.1f
#define YAW_KD                 0.8f
#define YAW_I_MAX              30.0f
#define YAW_D_LPF_ALPHA        0.7f

#endif // CONFIG_H
