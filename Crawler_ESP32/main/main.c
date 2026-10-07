/*
  Self-Driving RC Car - pure C, ESP-IDF, for the Arduino Nano ESP32 (ESP32-S3)

  Ported from the updated ATmega328P/Arduino-Nano version, keeping the SAME
  physical wiring: every sensor/servo/motor/start wire plugs into the exact
  same D-pin / A-pin header position it did on the old Nano. Only the
  #defines below had to change, mapping each Arduino pin LABEL to the real
  GPIO number the Nano ESP32 board actually uses underneath that label
  (the Nano ESP32's silkscreen numbers are not the chip's own GPIO
  numbers - they're a separate mapping specific to this board):

    Arduino label -> ESP32-S3 GPIO      Used for
    D2  -> GPIO5                        Sensor 1 (Left)  TRIG
    D10 -> GPIO21                       Sensor 1 (Left)  ECHO
    D4  -> GPIO7                        Sensor 2 (Mid)   TRIG
    D7  -> GPIO10                       Sensor 2 (Mid)   ECHO
    D8  -> GPIO17                       Sensor 3 (Right) TRIG
    D5  -> GPIO8                        Sensor 3 (Right) ECHO
    D9  -> GPIO18                       Steering servo (PWM)
    D3  -> GPIO6                        Drive motor DRV8871 IN1 (PWM)
    D11 -> GPIO38                       Drive motor DRV8871 IN2 (PWM)
    A0  -> GPIO1                        Start trigger (digital in)
    A1  -> GPIO2                        Battery sense (analog in)
    D6  -> GPIO9                        Status LED

  Since wiring is unchanged, this is a drop-in replacement on the same
  physical harness - move the board, not the wires.

  *** IMPORTANT HARDWARE WARNING ***
  ESP32-S3 GPIOs are 3.3V-tolerant ONLY. The HY-SRF05's ECHO output swings
  to 5V - put a voltage divider (e.g. 1k/2k resistors) or a logic-level
  shifter on every ECHO line before wiring it up, or you risk damaging the
  pin/board. TRIG lines are fine driven at 3.3V.

  NOTE: the rear (4th) HY-SRF05 sensor has been physically removed, same as
  in the original AVR version. There is no way to check the rear is clear
  before reversing - all recovery reversing is done "blind".

  Behavior:

    DRIVE - normal state. Centers itself between the two side boards with a
    PID on the left/right sensor difference, and slows down as the front
    sensor sees a wall/corner getting close.

    TURN - triggered the instant the front sensor sees a corner wall inside
    CORNER_TRIGGER_DISTANCE_CM. Reduced fixed speed, held for
    TURN_DURATION_MS (extended a bit if the front is still blocked once that
    expires), then hands back control to DRIVE.

    Turn direction (decided fresh at EVERY corner, no learning): the track
    can run in any rotation, clockwise, counter-clockwise or mixed. At the
    start of each corner the left/right difference is averaged over
    TURN_DECIDE_MS. While that window runs, the car steers provisionally by
    the running sign of the average; once it ends, the direction is locked
    for the rest of that corner only. Nothing is remembered between
    corners. The turn PID only supplies the steering MAGNITUDE (with a
    minimum of TURN_MIN_STEER_DEG), the sign comes from the decision.

  Recovery (reverse + wiggle) triggers on either:
    1. Front emergency stop - front sensor inside STOP_DISTANCE_CM.
    2. Stuck detection - all three front sensors report essentially
       unchanged distances for STUCK_CHECK_WINDOW_MS while in DRIVE and
       commanding forward motion (e.g. wedged sideways, wheels spinning).

  Either trigger: stop, reverse for REVERSE_TIME_MS while continuously
  wiggling the steering hard left/right, then return to DRIVE.

  Kickstart: every transition from "stopped" to "commanded-moving" (either
  direction) is driven at full power (KICKSTART_SPEED) for a short fixed
  burst (KICKSTART_MS) before dropping to the requested speed.

  ---------------------------------------------------------------------
  Porting notes (AVR/Arduino -> ESP-IDF C):
  - DDRx/PORTx/PINx -> gpio_config()/gpio_set_level()/gpio_get_level().
  - Custom Timer0 micros() -> esp_timer_get_time() (already in microseconds).
  - Trigger pulse / echo pulse-width bit-banging -> a small busy-wait
    helper (measure_echo_pulse_us) built on esp_timer_get_time().
  - Timer1 fast-PWM servo drive -> one LEDC channel at 50Hz. ESP32-S3's
    LEDC hardware maxes out at 14-bit duty resolution (unlike the classic
    ESP32's 20-bit), so SERVO_LEDC_RES_BITS is 14-bit here, and
    SERVO_LEDC_DUTY_MAX is derived FROM that resolution automatically
    rather than hardcoded, so the two can never drift out of sync again if
    the resolution ever changes.
  - Timer2 fast-PWM motor drive -> two more LEDC channels at ~1kHz with
    8-bit duty resolution (0-255), matching the original speed scale 1:1.
  - AVR's 10-bit ADC -> adc_oneshot driver configured for ADC_BITWIDTH_10,
    so analog reads stay on the same 0-1023 scale and BATTERY_LOW_THRESHOLD
    (860 counts) is still valid as-is. Battery sense moved from A1's own
    ADC channel on the AVR to A1's equivalent ADC1 channel on the S3
    (GPIO2 = ADC1_CHANNEL_1).
  - delay_ms()/delay_us() -> vTaskDelay(pdMS_TO_TICKS(ms)) for millisecond
    waits and esp_rom_delay_us() for the short microsecond trigger-pulse
    waits.
  - main()'s while(1) -> app_main()'s while(1), run as ESP-IDF's default
    main task under FreeRTOS.
  - Servo direction inverted (SERVO_INVERT below) at the person's request -
    flips which pulse width each end of the angle range maps to, without
    touching the PID/sensor/state-machine math at all.
  ---------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_rom_sys.h" // esp_rom_delay_us

// ---------- Pin assignments (same wiring as the original AVR Nano board) ----------
#define PIN_TRIG_LEFT     5   // D2
#define PIN_ECHO_LEFT     21  // D10
#define PIN_TRIG_MID      7   // D4
#define PIN_ECHO_MID      10  // D7
#define PIN_TRIG_RIGHT    17  // D8
#define PIN_ECHO_RIGHT    8   // D5

#define PIN_SERVO         18  // D9

#define PIN_MOTOR_IN1     6   // D3
#define PIN_MOTOR_IN2     38  // D11

#define PIN_START_TRIGGER 1   // A0
#define PIN_LED           9   // D6

#define BATTERY_ADC_UNIT     ADC_UNIT_1
#define BATTERY_ADC_CHANNEL  ADC_CHANNEL_1 // GPIO2 (A1) on ESP32-S3's ADC1

// ---------- LEDC (PWM) assignments ----------
#define LEDC_MODE            LEDC_LOW_SPEED_MODE // only mode available on ESP32-S3

#define SERVO_LEDC_TIMER      LEDC_TIMER_0
#define SERVO_LEDC_CHANNEL    LEDC_CHANNEL_0
#define SERVO_LEDC_FREQ_HZ    50
#define SERVO_LEDC_RES_BITS   LEDC_TIMER_14_BIT           // ESP32-S3 LEDC max
#define SERVO_LEDC_DUTY_MAX   ((1u << SERVO_LEDC_RES_BITS) - 1u) // derived, not hardcoded - can't drift out of sync with the resolution above
#define SERVO_PERIOD_US       20000u // 1 / 50Hz

#define MOTOR_LEDC_TIMER      LEDC_TIMER_1
#define MOTOR_LEDC_CHANNEL_1  LEDC_CHANNEL_1 // IN1
#define MOTOR_LEDC_CHANNEL_2  LEDC_CHANNEL_2 // IN2
#define MOTOR_LEDC_FREQ_HZ    1000
#define MOTOR_LEDC_RES_BITS   LEDC_TIMER_8_BIT // 0-255 duty, same scale as the original

// ---------- Tuning constants (matching the updated AVR version) ----------
#define MAX_DISTANCE_CM        150u    // ignore/clamp anything farther than this
#define ECHO_TIMEOUT_US        9000UL  // ~150cm round-trip timeout

#define STOP_DISTANCE_CM       5u     // genuine imminent collision - stop/steer/reverse
#define CORNER_SLOW_DISTANCE_CM 100u    // front wall closer than this -> start slowing, still in DRIVE state
#define CORNER_TRIGGER_DISTANCE_CM 80u // front wall closer than this -> commit to a hard-lock TURN

#define TURN_SPEED              200u    // fixed, slow speed while executing a hard-lock turn
#define TURN_DURATION_MS        600u   // how long to hold the turn through a corner - THE main knob to tune
#define TURN_MAX_EXTRA_MS       800u   // if still blocked after TURN_DURATION_MS, keep turning up to this much longer

#define SERVO_MIN_US      1000u  // full-left pulse width  - calibrate to your linkage
#define SERVO_MAX_US      2000u  // full-right pulse width - calibrate to your linkage
#define SERVO_CENTER_DEG  90u
#define SERVO_MIN_DEG     45u    // maps to SERVO_MIN_US (or SERVO_MAX_US if SERVO_INVERT)
#define SERVO_MAX_DEG     135u   // maps to SERVO_MAX_US (or SERVO_MIN_US if SERVO_INVERT)

// Set to 1 if the servo turns the wrong way (e.g. commanding "steer right"
// visibly steers left). This just flips which pulse width each end of the
// angle range maps to - it doesn't touch the PID, sensor, or state-machine
// math at all, so "more room on the right -> steer right" logic upstream
// stays correct regardless of which way the linkage/horn is mounted.
#define SERVO_INVERT      1u

#define DRIVE_SPEED       255u   // 0-255 forward PWM speed on a clear straight
#define MIN_SPEED         200u   // speed floor so the car doesn't stall approaching a corner
#define TURN_SPEED_REDUCTION 50u // max PWM cut for hard PID steering corrections on a straight
#define REVERSE_SPEED     100u   // 0-255 reverse PWM speed used during recovery

// ---------- Kickstart tuning ----------
#define KICKSTART_SPEED    255u
#define KICKSTART_MS       120u
#define RECOVERY_SETTLE_MS 100u

// ---------- Recovery (reverse + steering wiggle) tuning ----------
#define REVERSE_TIME_MS        600u    // total time spent reversing during a recovery
#define WIGGLE_HALF_PERIOD_MS  200u    // how often the steering flips side while reversing

// ---------- Stuck detection tuning ----------
#define STUCK_CHECK_WINDOW_MS   2000u   // how long with no real change before calling it "stuck"
#define STUCK_DISTANCE_DELTA_CM 3u      // combined left+mid+right movement below this = not moving

// ---------- Battery sense ----------
// ADC counts, 10-bit scale (0-1023) via adc_oneshot configured for
// ADC_BITWIDTH_10 below - matches the original AVR threshold of 860 counts
// (~7V through the original divider). Re-check against your actual divider.
#define BATTERY_LOW_THRESHOLD 860

// ---------- micros() ----------
static inline uint32_t micros(void) {
    return (uint32_t)esp_timer_get_time(); // esp_timer_get_time() is already in microseconds
}

static void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// ---------- GPIO setup ----------
static void gpio_setup(void) {
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << PIN_TRIG_LEFT) | (1ULL << PIN_TRIG_MID) |
                         (1ULL << PIN_TRIG_RIGHT) | (1ULL << PIN_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_conf);

    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << PIN_ECHO_LEFT) | (1ULL << PIN_ECHO_MID) |
                         (1ULL << PIN_ECHO_RIGHT) | (1ULL << PIN_START_TRIGGER),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_conf);

    gpio_set_level(PIN_TRIG_LEFT, 0);
    gpio_set_level(PIN_TRIG_MID, 0);
    gpio_set_level(PIN_TRIG_RIGHT, 0);
    gpio_set_level(PIN_LED, 0);
}

// ---------- LEDC (servo + motor PWM) setup ----------
static void ledc_setup(void) {
    ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = SERVO_LEDC_TIMER,
        .duty_resolution = SERVO_LEDC_RES_BITS,
        .freq_hz = SERVO_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&servo_timer);

    ledc_channel_config_t servo_channel = {
        .speed_mode = LEDC_MODE,
        .channel = SERVO_LEDC_CHANNEL,
        .timer_sel = SERVO_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_SERVO,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&servo_channel);

    ledc_timer_config_t motor_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = MOTOR_LEDC_TIMER,
        .duty_resolution = MOTOR_LEDC_RES_BITS,
        .freq_hz = MOTOR_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&motor_timer);

    ledc_channel_config_t motor_channel_1 = {
        .speed_mode = LEDC_MODE,
        .channel = MOTOR_LEDC_CHANNEL_1,
        .timer_sel = MOTOR_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_MOTOR_IN1,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&motor_channel_1);

    ledc_channel_config_t motor_channel_2 = {
        .speed_mode = LEDC_MODE,
        .channel = MOTOR_LEDC_CHANNEL_2,
        .timer_sel = MOTOR_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_MOTOR_IN2,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&motor_channel_2);
}

// ---------- ADC setup (battery sense) ----------
static adc_oneshot_unit_handle_t battery_adc_handle;

static void adc_setup(void) {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = BATTERY_ADC_UNIT,
    };
    adc_oneshot_new_unit(&unit_cfg, &battery_adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_10, // matches the original AVR 0-1023 scale
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(battery_adc_handle, BATTERY_ADC_CHANNEL, &chan_cfg);
}

// ---------- Servo ----------
static void set_servo_angle(uint8_t angle_deg) {
    if (angle_deg < SERVO_MIN_DEG) angle_deg = SERVO_MIN_DEG;
    if (angle_deg > SERVO_MAX_DEG) angle_deg = SERVO_MAX_DEG;

    uint32_t span_deg = SERVO_MAX_DEG - SERVO_MIN_DEG;
    uint32_t span_us  = SERVO_MAX_US - SERVO_MIN_US;
    uint32_t offset_us = (uint32_t)(angle_deg - SERVO_MIN_DEG) * span_us / span_deg;

#if SERVO_INVERT
    uint32_t pulse_us = SERVO_MAX_US - offset_us; // SERVO_MIN_DEG -> SERVO_MAX_US, SERVO_MAX_DEG -> SERVO_MIN_US
#else
    uint32_t pulse_us = SERVO_MIN_US + offset_us; // SERVO_MIN_DEG -> SERVO_MIN_US, SERVO_MAX_DEG -> SERVO_MAX_US
#endif

    uint32_t duty = (uint32_t)((uint64_t)pulse_us * SERVO_LEDC_DUTY_MAX / SERVO_PERIOD_US);
    ledc_set_duty(LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, SERVO_LEDC_CHANNEL);
}

// ---------- Motor (DRV8871 via LEDC PWM) ----------
static uint8_t kickstart_needed = 1; // car starts stationary, so arm it from boot

static inline void motor_pwm(uint8_t in1, uint8_t in2) {
    ledc_set_duty(LEDC_MODE, MOTOR_LEDC_CHANNEL_1, in1);
    ledc_update_duty(LEDC_MODE, MOTOR_LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_MODE, MOTOR_LEDC_CHANNEL_2, in2);
    ledc_update_duty(LEDC_MODE, MOTOR_LEDC_CHANNEL_2);
}

static void drive_forward(uint8_t speed) {
    if (kickstart_needed && speed > 0) {
        motor_pwm(KICKSTART_SPEED, 0);
        delay_ms(KICKSTART_MS);
        kickstart_needed = 0;
    }
    motor_pwm(speed, 0);
}

static void drive_reverse(uint8_t speed) {
    if (kickstart_needed && speed > 0) {
        motor_pwm(0, KICKSTART_SPEED);
        delay_ms(KICKSTART_MS);
        kickstart_needed = 0;
    }
    motor_pwm(0, speed);
}

static void motor_stop(void) {
    motor_pwm(0, 0);
    kickstart_needed = 1;
}

// ---------- Ultrasonic sensor: trigger + echo timing ----------
static void trigger_pulse(gpio_num_t trig_pin) {
    gpio_set_level(trig_pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trig_pin, 0);
}

static uint32_t measure_echo_pulse_us(gpio_num_t echo_pin) {
    uint32_t t0 = micros();
    while (gpio_get_level(echo_pin) == 0) {
        if (micros() - t0 > ECHO_TIMEOUT_US) return 0;
    }
    uint32_t start = micros();
    while (gpio_get_level(echo_pin) == 1) {
        if (micros() - start > ECHO_TIMEOUT_US) return 0;
    }
    return micros() - start;
}

static long read_distance_cm(gpio_num_t trig_pin, gpio_num_t echo_pin) {
    trigger_pulse(trig_pin);
    uint32_t duration = measure_echo_pulse_us(echo_pin);

    if (duration == 0) {
        return MAX_DISTANCE_CM;
    }

    long distance = (long)(duration / 58);
    if (distance > (long)MAX_DISTANCE_CM) distance = MAX_DISTANCE_CM;
    return distance;
}

// ---------- small helpers (map / constrain) ----------
static long constrain_long(long x, long lo, long hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static long map_long(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static float constrain_float(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// ---------- Steering PID ----------
#define STEER_KP  0.2f   // degrees of steering per cm of left/right imbalance
#define STEER_KI  0.02f  // corrects any steady drift/bias - keep small
#define STEER_KD  0.0f   // damps oscillation from sudden sensor jumps
#define STEER_INTEGRAL_LIMIT 150.0f // anti-windup clamp on the integral term (cm*s)

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
} SteeringPID;

static SteeringPID steer_pid = {
    .kp = STEER_KP,
    .ki = STEER_KI,
    .kd = STEER_KD,
    .integral = 0.0f,
    .prev_error = 0.0f
};

static void pid_reset(SteeringPID *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static float pid_update(SteeringPID *pid, float error, float dt) {
    if (dt <= 0.0f) dt = 0.001f;

    pid->integral += error * dt;
    pid->integral = constrain_float(pid->integral, -STEER_INTEGRAL_LIMIT, STEER_INTEGRAL_LIMIT);

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}

// ---------- Sensor smoothing ----------
#define SENSOR_FILTER_ALPHA 0.9f // slight filtering

static float filtered_diff = 0.0f;
static uint8_t filtered_diff_valid = 0;

static float filter_update(float raw_value) {
    if (!filtered_diff_valid) {
        filtered_diff = raw_value;
        filtered_diff_valid = 1;
    } else {
        filtered_diff = (SENSOR_FILTER_ALPHA * raw_value) + ((1.0f - SENSOR_FILTER_ALPHA) * filtered_diff);
    }
    return filtered_diff;
}

// ---------- Turn PID ----------
#define TURN_KP  0.5f
#define TURN_KI  0.0f
#define TURN_KD  0.0f

static SteeringPID turn_pid = {
    .kp = TURN_KP,
    .ki = TURN_KI,
    .kd = TURN_KD,
    .integral = 0.0f,
    .prev_error = 0.0f
};

// ---------- Per-corner direction decision (no learning between corners) ----------
#define TURN_DECIDE_MS        150u  // average left/right difference over this long at the start of each corner
#define TURN_MIN_STEER_DEG    40u   // minimum steering angle away from center during a turn

static long    turn_diff_accum = 0; // running sum of (right-left) for the current corner
static uint8_t turn_decided    = 0; // direction locked for this corner?
static uint8_t turn_right      = 0; // locked direction for this corner (valid when turn_decided)

static void reset_steering(void) {
    pid_reset(&steer_pid);
    pid_reset(&turn_pid);
    filtered_diff_valid = 0;
}

// ---------- Stuck detector ----------
static uint32_t stuck_window_start_us = 0;
static long stuck_ref_left = 0, stuck_ref_mid = 0, stuck_ref_right = 0;
static uint8_t stuck_window_valid = 0;

static void reset_stuck_detector(void) {
    stuck_window_valid = 0;
}

static uint8_t stuck_check_and_update(uint32_t now_us, long dist_left, long dist_mid, long dist_right) {
    if (!stuck_window_valid) {
        stuck_window_start_us = now_us;
        stuck_ref_left = dist_left;
        stuck_ref_mid = dist_mid;
        stuck_ref_right = dist_right;
        stuck_window_valid = 1;
        return 0;
    }

    if ((now_us - stuck_window_start_us) < ((uint32_t)STUCK_CHECK_WINDOW_MS * 1000UL)) {
        return 0;
    }

    long total_change = labs(dist_left - stuck_ref_left)
                       + labs(dist_mid  - stuck_ref_mid)
                       + labs(dist_right - stuck_ref_right);

    stuck_window_start_us = now_us;
    stuck_ref_left = dist_left;
    stuck_ref_mid = dist_mid;
    stuck_ref_right = dist_right;

    return (total_change < (long)STUCK_DISTANCE_DELTA_CM) ? 1 : 0;
}

// ---------- Recovery: reverse while wiggling the steering ----------
static void recover_reverse_and_wiggle(void) {
    motor_stop();
    drive_reverse(REVERSE_SPEED);

    uint32_t recover_start_us = micros();
    uint8_t steer_right = 1; // arbitrary starting side - which way the wiggle starts doesn't matter

    while ((micros() - recover_start_us) < ((uint32_t)REVERSE_TIME_MS * 1000UL)) {
        set_servo_angle(steer_right ? SERVO_MAX_DEG : SERVO_MIN_DEG);
        delay_ms(WIGGLE_HALF_PERIOD_MS);
        steer_right = !steer_right;
    }

    motor_stop();
    delay_ms(RECOVERY_SETTLE_MS);
    set_servo_angle(SERVO_CENTER_DEG);

    reset_steering();
    reset_stuck_detector();
}

// ---------- Drive state machine ----------
typedef enum {
    STATE_DRIVE,
    STATE_TURN
} DriveState;

static uint16_t read_battery_adc(void) {
    int raw = 0;
    adc_oneshot_read(battery_adc_handle, BATTERY_ADC_CHANNEL, &raw);
    return (uint16_t)raw;
}

static void battery_low_warning(void) {
    if (read_battery_adc() < BATTERY_LOW_THRESHOLD) {
        gpio_set_level(PIN_LED, 1);
        delay_ms(250);
        gpio_set_level(PIN_LED, 0);
        delay_ms(250);
    } else {
        gpio_set_level(PIN_LED, 0);
    }
}

void app_main(void) {
    gpio_setup();
    ledc_setup();
    adc_setup();

    motor_stop();
    set_servo_angle(SERVO_CENTER_DEG);

    // Wait for the remote/start module to drive PIN_START_TRIGGER HIGH
    // Set != 0 for bypassing of start module
    // == 0 Start module connected

    while (gpio_get_level(PIN_START_TRIGGER) != 0) {
        delay_ms(20);
        battery_low_warning();
    }

    uint32_t last_pid_us = micros();
    DriveState state = STATE_DRIVE;
    uint32_t turn_start_us = 0;
    reset_stuck_detector();

    while (1) {
        // ---- Stop/resume ----
        // Set != 0 for bypassing of start module
        // == 0 Start module connected
        if (gpio_get_level(PIN_START_TRIGGER) != 0) {
            motor_stop();
            set_servo_angle(SERVO_CENTER_DEG);
            reset_steering();
            reset_stuck_detector();
            state = STATE_DRIVE;

            while (gpio_get_level(PIN_START_TRIGGER) == 0) {
                delay_ms(20);
                battery_low_warning();
            }

            last_pid_us = micros();
            continue;
        }

        long dist_left  = read_distance_cm(PIN_TRIG_LEFT, PIN_ECHO_LEFT);
        delay_ms(10);

        long dist_mid   = read_distance_cm(PIN_TRIG_MID, PIN_ECHO_MID);
        delay_ms(10);

        long dist_right = read_distance_cm(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT);
        delay_ms(10);

        // ---- Safety net: applies in either state ----
        if (dist_mid < STOP_DISTANCE_CM) {
            recover_reverse_and_wiggle();
            last_pid_us = micros();
            state = STATE_DRIVE;
            continue;
        }

        if (state == STATE_DRIVE) {
            if (stuck_check_and_update(micros(), dist_left, dist_mid, dist_right)) {
                recover_reverse_and_wiggle();
                last_pid_us = micros();
                continue;
            }

            if (dist_mid < CORNER_TRIGGER_DISTANCE_CM) {
                state = STATE_TURN;
                turn_start_us = micros();
                reset_steering();
                reset_stuck_detector();
                turn_diff_accum = 0;
                turn_decided = 0;
                continue;
            }

            uint32_t now = micros();
            float dt = (float)(now - last_pid_us) / 1000000.0f;
            last_pid_us = now;

            long diff = constrain_long(dist_right - dist_left, -(long)MAX_DISTANCE_CM, (long)MAX_DISTANCE_CM);
            float smoothed_diff = filter_update((float)diff);
            float offset = pid_update(&steer_pid, smoothed_diff, dt);

            int angle = (int)SERVO_CENTER_DEG + (int)offset;
            if (angle < (int)SERVO_MIN_DEG) angle = (int)SERVO_MIN_DEG;
            if (angle > (int)SERVO_MAX_DEG) angle = (int)SERVO_MAX_DEG;
            set_servo_angle((uint8_t)angle);

            long steer_amount = (offset < 0.0f) ? (long)(-offset) : (long)offset;
            long turn_speed = (long)DRIVE_SPEED - map_long(
                constrain_long(steer_amount, 0, (long)(SERVO_MAX_DEG - SERVO_CENTER_DEG)),
                0, (long)(SERVO_MAX_DEG - SERVO_CENTER_DEG),
                0, (long)TURN_SPEED_REDUCTION);

            long front_speed = (dist_mid < CORNER_SLOW_DISTANCE_CM)
                ? map_long(constrain_long(dist_mid, STOP_DISTANCE_CM, CORNER_SLOW_DISTANCE_CM),
                           STOP_DISTANCE_CM, CORNER_SLOW_DISTANCE_CM,
                           MIN_SPEED, DRIVE_SPEED)
                : (long)DRIVE_SPEED;

            long speed = (turn_speed < front_speed) ? turn_speed : front_speed;
            if (speed < MIN_SPEED) speed = MIN_SPEED;

            drive_forward((uint8_t)speed);

        } else { // STATE_TURN
            uint32_t now = micros();
            float dt = (float)(now - last_pid_us) / 1000000.0f;
            last_pid_us = now;

            uint32_t elapsed_ms = (now - turn_start_us) / 1000UL;

            long diff = constrain_long(dist_right - dist_left, -(long)MAX_DISTANCE_CM, (long)MAX_DISTANCE_CM);
            float offset = pid_update(&turn_pid, (float)diff, dt);

            // Decide this corner's direction: average the side difference for
            // TURN_DECIDE_MS, then lock it for the rest of this corner.
            if (!turn_decided) {
                turn_diff_accum += diff;
                if (elapsed_ms >= TURN_DECIDE_MS) {
                    turn_right = (turn_diff_accum >= 0); // more room on the right -> turn right
                    turn_decided = 1;
                }
            }
            uint8_t steer_right = turn_decided ? turn_right : (turn_diff_accum >= 0);

            // Only the PID's magnitude is used; the sign comes from the decision above.
            float magnitude = (offset < 0.0f) ? -offset : offset;
            if (magnitude < (float)TURN_MIN_STEER_DEG) magnitude = (float)TURN_MIN_STEER_DEG;
            offset = steer_right ? magnitude : -magnitude;

            int angle = (int)SERVO_CENTER_DEG + (int)offset;
            if (angle < (int)SERVO_MIN_DEG) angle = (int)SERVO_MIN_DEG;
            if (angle > (int)SERVO_MAX_DEG) angle = (int)SERVO_MAX_DEG;
            set_servo_angle((uint8_t)angle);

            drive_forward(TURN_SPEED);

            if (elapsed_ms >= TURN_DURATION_MS) {
                if (dist_mid >= CORNER_TRIGGER_DISTANCE_CM ||
                    elapsed_ms >= (TURN_DURATION_MS + TURN_MAX_EXTRA_MS)) {
                    state = STATE_DRIVE;
                    reset_steering();
                    reset_stuck_detector();
                    last_pid_us = micros();
                }
            }
        }
    }
}
