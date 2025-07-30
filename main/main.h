#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "i2cdev.h"
#include "ads111x.h"
#include "ina219.h"
#include "as5600.h"

// ########################## Global Device Definitions ##########################

// ########################## I2C Settings ##########################
#define I2C_PORT_NUM I2C_NUM_0

// ########################## ADS1115 Settings ##########################
#define GAIN ADS111X_GAIN_4V096 // +-4.096V
#define ADS1115_DEVICE_COUNT 3
#define ADS1115_RETRY_DELAY_START_MS 2000
#define ADS1115_RETRY_DELAY_MAX_MS 60000

// ADS1115 addresses
static const uint8_t ads1115_addresses[ADS1115_DEVICE_COUNT] = {
    ADS111X_ADDR_GND, // 0x48 - Photoresistor array (Dev#0)
    ADS111X_ADDR_VCC, // 0x49 - Moisture sensors (Dev#1)
    ADS111X_ADDR_SDA  // 0x4A - Zone5 moisture + pressure (Dev#2)
};

typedef struct {
    i2c_dev_t device;             // I2C device handle
    bool initialized;             // Device initialization status
    uint32_t last_retry_time;     // Last retry attempt timestamp
    uint8_t retry_count;          // Number of retry attempts
    uint32_t next_retry_delay_ms; // Next retry delay (exponential backoff)
    const char *name;             // Device name for logging
} ads1115_device_t;

// Single array containing all device data
extern ads1115_device_t ads1115_devices[ADS1115_DEVICE_COUNT];

// ADS1115 display arrays - organized by device and channel
extern float latest_ads_voltages[ADS1115_DEVICE_COUNT][4];

// ########################## INA219 Settings ##########################

// Define INA219 sensor types
typedef enum {
    INA219_TYPE_GENERIC, // Generic INA219 with 0.1Ω shunt resistor (3.2A max)
    INA219_TYPE_DFROBOT  // DFRobot SEN0291 with 0.01Ω shunt resistor (8A max)
} ina219_sensor_type_t;

// INA219 sensor configuration structure
typedef struct {
    ina219_t dev;              // Device handle
    ina219_sensor_type_t type; // Sensor type
    uint8_t addr;              // I2C address
    float shunt_ohms;          // Shunt resistor value in ohms
    ina219_gain_t gain;        // Gain setting
    const char *name;          // Human-readable name
    bool initialized;          // Whether initialization succeeded
} ina219_sensor_t;

// INA219 global variables
#define INA219_DEVICE_COUNT 1 // Define number of INA219 modules (planning for 2, 1 for now)
extern float latest_ina219_voltage[INA219_DEVICE_COUNT];
extern float latest_ina219_current[INA219_DEVICE_COUNT];
extern float latest_ina219_power[INA219_DEVICE_COUNT];

// INA219 sensor configurations
extern ina219_sensor_t ina219_devices[INA219_DEVICE_COUNT];

// ########################## AS5600 Settings ##########################

#define AS5600_I2C_ADDR AS5600_DEFAULT_ADDRESS // 0x36
// Global AS5600 device descriptor
extern as5600_dev_t as5600_dev;

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

// Global servo positions
static uint32_t current_yaw_duty = SERVO_CENTER_DUTY;
static uint32_t current_pitch_duty = SERVO_CENTER_DUTY;

// ########################## Power Management Settings ##########################

#define MOSFET_3V3_SENSOR_BUS_CUTOFF_GPIO GPIO_NUM_16 // 3.3V sensor bus control
#define MOSFET_5V_BUS_CUTOFF_GPIO GPIO_NUM_17         // 5V bus control (placeholder)
#define MOSFET_6V2_SERVO_BUS_CUTOFF_GPIO GPIO_NUM_18  // 6.2V servo bus control
#define MOSFET_12V_FAN_PWM_GPIO GPIO_NUM_19           // 12V fan PWM (placeholder)

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

typedef enum { SENSOR_NONE = 0, SENSOR_BME280, SENSOR_SHT4X } SensorType;

typedef struct {
    SensorType tag;
    float temperature;
    float humidity;
    float pressure;
} TaggedSensorData;


// ########################## FUNCTION DECLARATIONS ################################

// ADS1115 Functions
esp_err_t ads1115_read_channel(uint8_t device_id, ads111x_mux_t channel, int16_t *raw, float *voltage);

// Power Management Functions
esp_err_t power_on_3V3_sensor_bus(void);
esp_err_t power_off_3V3_sensor_bus(void);


#endif // MAIN_H