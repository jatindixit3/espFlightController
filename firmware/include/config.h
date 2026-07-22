#pragma once
#include <stdint.h>

// ============================================================================
// Hardware pin map - Seeed XIAO ESP32-S3 (non-Sense)
// See docs/HARDWARE.md for the full wiring rationale.
// ============================================================================

// Motors - bidirectional DShot300 via RMT, one channel per motor.
// Motor order follows Betaflight QuadX convention (M1=rear-right, M2=front-right,
// M3=rear-left, M4=front-left) but physical wiring order does not matter - the
// configurator's motor test tab is how you verify/resolve wiring to physical position.
constexpr int PIN_MOTOR1 = 1;   // D0 / GPIO1
constexpr int PIN_MOTOR2 = 2;   // D1 / GPIO2
constexpr int PIN_MOTOR3 = 3;   // D2 / GPIO3
constexpr int PIN_MOTOR4 = 4;   // D3 / GPIO4
constexpr int MOTOR_PINS[4] = {PIN_MOTOR1, PIN_MOTOR2, PIN_MOTOR3, PIN_MOTOR4};
constexpr int MOTOR_COUNT = 4;

// IMU - MPU6050 over I2C
constexpr int PIN_I2C_SDA = 5;  // D4 / GPIO5
constexpr int PIN_I2C_SCL = 6;  // D5 / GPIO6
constexpr uint32_t I2C_CLOCK_HZ = 400000; // MPU6050 Fast-mode ceiling

// ESC telemetry - single wire UART RX (voltage/current/temp/eRPM backup)
constexpr int PIN_ESC_TELEMETRY_RX = 43; // D6 / GPIO43
constexpr int ESC_TELEMETRY_UART_NUM = 1;
constexpr uint32_t ESC_TELEMETRY_BAUD = 115200;

// RC receiver - SBUS, inverted UART RX only
constexpr int PIN_SBUS_RX = 44; // D7 / GPIO44
constexpr int SBUS_UART_NUM = 2;
constexpr uint32_t SBUS_BAUD = 100000;

// Status indicators
constexpr int PIN_LED_DATA = 7;  // D8 / GPIO7
constexpr int PIN_BUZZER = 8;    // D9 / GPIO8
constexpr int LED_COUNT = 8;     // adjust to your strip's actual LED count

// Spare pin reserved for future use (arm-status LED, debug UART, extra sensor)
constexpr int PIN_SPARE = 9;     // D10 / GPIO9

// ============================================================================
// Timing
// ============================================================================
constexpr uint32_t FLIGHT_LOOP_HZ = 1000;         // gyro/PID/mixer/motor loop rate
constexpr uint32_t FLIGHT_LOOP_PERIOD_US = 1000000UL / FLIGHT_LOOP_HZ;
constexpr uint32_t ESC_TELEMETRY_TASK_HZ = 100;
constexpr uint32_t INDICATOR_TASK_HZ = 50;
constexpr uint32_t MSP_TASK_HZ = 100;
constexpr uint32_t BLACKBOX_DEFAULT_RATE_DIVIDER = 2; // log every Nth loop by default

// ============================================================================
// Safety limits
// ============================================================================
constexpr uint16_t DSHOT_MIN_THROTTLE = 48;    // DShot value just above the 0-47 command range
constexpr uint16_t DSHOT_MAX_THROTTLE = 2047;
constexpr float GYRO_CALIBRATION_STILLNESS_ACCEL_TOLERANCE_G = 0.05f; // accel vector must read 1g +/- this
constexpr float GYRO_CALIBRATION_STILLNESS_GYRO_TOLERANCE_DPS = 2.0f; // gyro noise must be below this while "still"
constexpr uint32_t GYRO_CALIBRATION_DURATION_MS = 2000;
constexpr uint32_t SBUS_FAILSAFE_TIMEOUT_MS = 200; // no valid SBUS frame in this window -> failsafe

// Flash partition for blackbox (must match partitions.csv)
constexpr const char* BLACKBOX_PARTITION_NAME = "blackbox";
