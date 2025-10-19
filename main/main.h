#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ########################## Global Device Definitions ##########################

// ########################## I2C Settings ##########################
#define I2C_PORT_NUM I2C_NUM_0

// ADS1115 functionality moved to ads1115_helper component

// ########################## SPI Settings ##########################
// SPI2 bus shared by: ABP pressure sensor, HMI display
#define SPI2_MOSI_PIN 9
#define SPI2_MISO_PIN 10
#define SPI2_SCLK_PIN 11

// ########################## Solar Tracking System Settings ##########################
// Photoresistor layout:
// [Ch0: Left-Top]  [Ch1: Right-Top]
// [Ch2: Left-Bot]  [Ch3: Right-Bot]

typedef struct {
    float left_top;     // Ch0
    float right_top;    // Ch1
    float left_bottom;  // Ch2
    float right_bottom; // Ch3
    float yaw_error;    // (left_avg - right_avg)
    float pitch_error;  // (top_avg - bottom_avg)
} photoresistor_readings_t;

// PWM/Servo Settings
#define SERVO_YAW_GPIO_PIN 25   // GPIO for yaw servo
#define SERVO_PITCH_GPIO_PIN 26 // GPIO for pitch servo

#define SERVO_PWM_FREQUENCY 50 // 50Hz for standard servos
#define SERVO_PWM_RESOLUTION LEDC_TIMER_14_BIT
#define SERVO_PWM_TIMER LEDC_TIMER_0
#define SERVO_YAW_CHANNEL LEDC_CHANNEL_0
#define SERVO_PITCH_CHANNEL LEDC_CHANNEL_1

// Servo position limits (in PWM duty cycle)
#define SERVO_MIN_DUTY 410     // ~1ms pulse width (0 degrees)
#define SERVO_MAX_DUTY 2048    // ~2ms pulse width (180 degrees)
#define SERVO_CENTER_DUTY 1229 // ~1.5ms pulse width (90 degrees)

// Solar tracking parameters
#define PHOTORESISTOR_THRESHOLD 0.050f // 50mV difference threshold
#define SERVO_STEP_SIZE 13             // PWM steps per adjustment
#define MAX_SERVO_ADJUSTMENT 250       // Maximum PWM adjustment per cycle
#define TRACKING_INTERVAL_MS 5000      // Check every 5 seconds

// ########################## Debug display settings ##########################
// Debuging display variables
#define SERIAL_DISPLAY_INTERVAL_MS 2000 // Update display every 2 seconds

extern SemaphoreHandle_t xDisplayDataMutex;

extern float latest_sht_temp;
extern float latest_sht_hum;
extern float latest_bmp_temp;
extern float latest_bmp_press;
extern float latest_bmp_hum; // If BME280
extern float latest_as5600_angle;
extern uint16_t latest_as5600_raw;

// ########################## FUNCTION DECLARATIONS ################################

// ADS1115 functions moved to ads1115_helper

/**
 * @brief Initialize SPI2 bus (shared by ABP pressure sensor and HMI display)
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t spi_bus_init(void);

#endif // MAIN_H