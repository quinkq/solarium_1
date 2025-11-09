/**
 * @file as5600_example.c
 * @brief AS5600 Magnetic Rotary Position Sensor - Usage Examples
 * @author Piotr P. <quinkq@gmail.com>
 * @date 2025
 *
 * This file demonstrates various use cases for the AS5600 driver.
 *
 */

#include "as5600.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "AS5600_EXAMPLE";

/*******************************************************************************
 * SUMMARY: Function Selection Guide
 ******************************************************************************/
void function_selection_guide(void)
{
    /*
     * QUICK REFERENCE: Which function should I use?
     *
     * ┌─────────────────────────────────────────────────────────────────┐
     * │ USE CASE                    │ RECOMMENDED FUNCTION              │
     * ├─────────────────────────────────────────────────────────────────┤
     * │ Wind speed / RPM            │ as5600_read_raw_counts()          │
     * │ Motor velocity              │ as5600_read_raw_counts()          │
     * │ Any speed measurement       │ as5600_read_raw_counts()          │
     * ├─────────────────────────────────────────────────────────────────┤
     * │ Robot arm angle (display)   │ as5600_read_angle_degrees()       │
     * │ Valve position (display)    │ as5600_read_angle_degrees()       │
     * │ Joystick/dial position      │ as5600_read_angle_degrees()       │
     * ├─────────────────────────────────────────────────────────────────┤
     * │ Control algorithms (PID)    │ as5600_read_angle_radians()       │
     * │ Physics calculations        │ as5600_read_angle_radians()       │
     * ├─────────────────────────────────────────────────────────────────┤
     * │ Relative position (counts)  │ as5600_read_relative_counts()     │
     * │ Custom unit conversion      │ as5600_read_relative_counts()     │
     * ├─────────────────────────────────────────────────────────────────┤
     * │ Multi-turn from home        │ as5600_read_accumulated_counts_   │
     * │ Robot joint absolute pos    │   relative()                      │
     * │ Valve turns from closed     │                                   │
     * ├─────────────────────────────────────────────────────────────────┤
     * │ Lifetime odometer           │ as5600_read_accumulated_counts()  │
     * │ Maintenance tracking        │                                   │
     * │ Total distance traveled     │                                   │
     * └─────────────────────────────────────────────────────────────────┘
     *
     * CALIBRATION:
     * - Call as5600_set_zero_offset() to set current position as "zero"
     * - Affects: angle_degrees, angle_radians, relative_counts,
     *           accumulated_counts_relative
     * - Does NOT affect: raw_counts, accumulated_counts (absolute)
     *
     * POWER MODES:
     * - Battery powered → LPM3 (saves ~5mA)
     * - Solar powered → LPM2 or LPM3
     * - AC powered / high precision → NORMAL
     *
     * DIAGNOSTICS:
     * - Always check as5600_read_status() during development/commissioning
     * - Verify magnet alignment before deployment
     */
}

/*******************************************************************************
 * EXAMPLE 1: Basic Initialization and Simple Reading
 ******************************************************************************/
void example_1_basic_usage(void)
{
    ESP_LOGI(TAG, "=== Example 1: Basic Usage ===");

    // Step 1: Initialize the device descriptor
    as5600_dev_t sensor = {0};
    esp_err_t ret;

    ret = as5600_init_desc(&sensor,
                           I2C_NUM_0,              // I2C port
                           AS5600_DEFAULT_ADDRESS, // 0x36
                           GPIO_NUM_21,            // SDA pin
                           GPIO_NUM_22);           // SCL pin

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize descriptor: %s", esp_err_to_name(ret));
        return;
    }

    // Step 2: Initialize the sensor (starts background task)
    ret = as5600_init(&sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor: %s", esp_err_to_name(ret));
        as5600_free_desc(&sensor);
        return;
    }

    // Step 3: Read angle in degrees (0-360°)
    float angle_deg;
    ret = as5600_read_angle_degrees(&sensor, &angle_deg);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Current angle: %.2f degrees", angle_deg);
    }

    // Step 4: Read angle in radians (0-2π)
    float angle_rad;
    ret = as5600_read_angle_radians(&sensor, &angle_rad);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Current angle: %.4f radians", angle_rad);
    }

    // Step 5: Read raw counts (0-4095)
    uint16_t raw_counts;
    ret = as5600_read_raw_counts(&sensor, &raw_counts);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Raw counts: %u / 4096", raw_counts);
    }

    // Step 6: Cleanup when done
    vTaskDelay(pdMS_TO_TICKS(5000)); // Use sensor for a while
    as5600_free_desc(&sensor);
    ESP_LOGI(TAG, "Sensor cleaned up");
}

/*******************************************************************************
 * EXAMPLE 2: Zero Offset Calibration (Robot Arm Joint)
 ******************************************************************************/
void example_2_robot_arm_calibration(void)
{
    ESP_LOGI(TAG, "=== Example 2: Robot Arm Joint with Calibration ===");

    as5600_dev_t joint_sensor = {0};

    // Initialize sensor
    as5600_init_desc(&joint_sensor, I2C_NUM_0, AS5600_DEFAULT_ADDRESS,
                     GPIO_NUM_21, GPIO_NUM_22);
    as5600_init(&joint_sensor);

    // Robot powers on, joint is at random position
    float current_angle;
    as5600_read_angle_degrees(&joint_sensor, &current_angle);
    ESP_LOGI(TAG, "Power-on position: %.2f°", current_angle);

    // Operator manually moves joint to HOME position
    ESP_LOGI(TAG, "Move joint to HOME position and press button...");
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for manual positioning

    // Calibrate current position as zero (home)
    as5600_set_zero_offset(&joint_sensor);
    ESP_LOGI(TAG, "HOME position calibrated!");

    // Now all readings are relative to HOME
    as5600_read_angle_degrees(&joint_sensor, &current_angle);
    ESP_LOGI(TAG, "Angle from HOME: %.2f°", current_angle);

    // Simulate joint movement
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Check position relative to home (after multiple rotations)
    float relative_accumulated;
    as5600_read_accumulated_counts_relative(&joint_sensor, &relative_accumulated);
    float rotations_from_home = relative_accumulated / 4096.0f;
    ESP_LOGI(TAG, "Position: %.2f rotations from HOME", rotations_from_home);

    // Example: If joint rotated 3.5 turns clockwise from home
    // relative_accumulated would be ~14,336 counts (3.5 * 4096)

    as5600_free_desc(&joint_sensor);
}

/*******************************************************************************
 * EXAMPLE 3: Wind Speed Measurement (Velocity Tracking)
 ******************************************************************************/
void example_3_wind_speed_anemometer(void)
{
    ESP_LOGI(TAG, "=== Example 3: Wind Speed Anemometer ===");

    as5600_dev_t anemometer = {0};

    // Initialize
    as5600_init_desc(&anemometer, I2C_NUM_0, AS5600_DEFAULT_ADDRESS,
                     GPIO_NUM_21, GPIO_NUM_22);
    as5600_init(&anemometer);

    // Set to low power mode (LPM3) - sensor polls at 100ms internally
    // This saves ~5mA (6.5mA → 1.5mA) for battery-powered systems
    as5600_set_power_mode(&anemometer, AS5600_POWER_MODE_LPM3);
    ESP_LOGI(TAG, "AS5600 set to LPM3 for power savings");

    // Measure wind speed over 5 seconds
    const uint32_t sample_duration_ms = 5000;
    const uint32_t sample_interval_ms = 200; // Read every 200ms

    uint16_t last_raw = 0;
    float total_rotations = 0.0f;
    bool first_sample = true;
    uint32_t sample_count = 0;

    TickType_t start_time = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(sample_duration_ms)) {
        uint16_t current_raw;

        if (as5600_read_raw_counts(&anemometer, &current_raw) == ESP_OK) {
            if (!first_sample) {
                // Calculate angle change (handling wrap-around)
                int16_t delta = current_raw - last_raw;

                // Handle wrap at 4096 counts (crossing 0°/360° boundary)
                if (delta > 2048) {
                    delta -= 4096; // Wrapped backward
                } else if (delta < -2048) {
                    delta += 4096; // Wrapped forward
                }

                total_rotations += (float)delta / 4096.0f;
                sample_count++;
            }

            last_raw = current_raw;
            first_sample = false;
        }

        vTaskDelay(pdMS_TO_TICKS(sample_interval_ms));
    }

    // Calculate RPM
    float duration_minutes = sample_duration_ms / 60000.0f;
    float rpm = fabs(total_rotations) / duration_minutes;

    ESP_LOGI(TAG, "Wind speed: %.2f RPM (%.1f rotations in %.1fs, %lu samples)",
             rpm, fabs(total_rotations), duration_minutes * 60.0f, sample_count);

    // Note: We use raw counts, not relative counts, because:
    // - We only care about velocity (delta), not absolute position
    // - Zero offset would cancel out in subtraction anyway
    // - Immune to calibration changes

    as5600_free_desc(&anemometer);
}

/*******************************************************************************
 * EXAMPLE 4: Odometer / Total Distance (Lifetime Rotation Tracking)
 ******************************************************************************/
void example_4_robot_wheel_odometer(void)
{
    ESP_LOGI(TAG, "=== Example 4: Robot Wheel Odometer ===");

    as5600_dev_t wheel_encoder = {0};

    // Initialize
    as5600_init_desc(&wheel_encoder, I2C_NUM_0, AS5600_DEFAULT_ADDRESS,
                     GPIO_NUM_21, GPIO_NUM_22);
    as5600_init(&wheel_encoder);

    // Robot starts, wheel has rotated 0 times
    float lifetime_counts;
    as5600_read_accumulated_counts(&wheel_encoder, &lifetime_counts);
    ESP_LOGI(TAG, "Initial odometer: %.1f rotations", lifetime_counts / 4096.0f);

    // Simulate robot driving (wheel rotates)
    ESP_LOGI(TAG, "Robot driving for 10 seconds...");
    vTaskDelay(pdMS_TO_TICKS(10000));

    // Check total distance traveled
    as5600_read_accumulated_counts(&wheel_encoder, &lifetime_counts);
    float total_rotations = lifetime_counts / 4096.0f;

    // Example: Wheel diameter = 100mm, circumference = 314mm
    float wheel_circumference_mm = 314.0f;
    float distance_mm = total_rotations * wheel_circumference_mm;

    ESP_LOGI(TAG, "Odometer: %.2f rotations = %.1f mm traveled",
             total_rotations, distance_mm);

    // Operator parks robot at charging station and sets this as "display zero"
    // for convenience (so display shows distance from charging station)
    as5600_set_zero_offset(&wheel_encoder);
    ESP_LOGI(TAG, "Display calibrated to charging station position");

    // Robot continues driving
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Check LIFETIME total (ignores zero offset)
    as5600_read_accumulated_counts(&wheel_encoder, &lifetime_counts);
    ESP_LOGI(TAG, "Lifetime odometer: %.2f rotations (for maintenance tracking)",
             lifetime_counts / 4096.0f);

    // Check distance from charging station (respects zero offset)
    float relative_counts;
    as5600_read_accumulated_counts_relative(&wheel_encoder, &relative_counts);
    ESP_LOGI(TAG, "Distance from charging station: %.2f rotations",
             relative_counts / 4096.0f);

    // Note: Lifetime accumulator is useful for:
    // - Maintenance scheduling (replace wheels after X million rotations)
    // - Wear tracking
    // - Total distance statistics

    as5600_free_desc(&wheel_encoder);
}

/*******************************************************************************
 * EXAMPLE 5: Valve Position with Multi-Turn Tracking
 ******************************************************************************/
void example_5_valve_actuator(void)
{
    ESP_LOGI(TAG, "=== Example 5: Multi-Turn Valve Actuator ===");

    as5600_dev_t valve_encoder = {0};

    // Initialize
    as5600_init_desc(&valve_encoder, I2C_NUM_0, AS5600_DEFAULT_ADDRESS,
                     GPIO_NUM_21, GPIO_NUM_22);
    as5600_init(&valve_encoder);

    // Step 1: Close valve fully (manual/motor operation)
    ESP_LOGI(TAG, "Closing valve to fully closed position...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Step 2: Set fully closed as zero reference
    as5600_set_zero_offset(&valve_encoder);
    ESP_LOGI(TAG, "Fully closed position set as zero");

    // Step 3: Open valve (requires 5.25 turns to fully open)
    ESP_LOGI(TAG, "Opening valve...");
    vTaskDelay(pdMS_TO_TICKS(5000)); // Simulate motor running

    // Step 4: Read absolute position from "fully closed" reference
    float position_counts;
    as5600_read_accumulated_counts_relative(&valve_encoder, &position_counts);
    float turns_from_closed = position_counts / 4096.0f;

    ESP_LOGI(TAG, "Valve position: %.2f turns from closed", turns_from_closed);

    // Calculate percentage open (if fully open = 5.25 turns)
    const float FULLY_OPEN_TURNS = 5.25f;
    float percent_open = (turns_from_closed / FULLY_OPEN_TURNS) * 100.0f;

    if (percent_open > 100.0f) {
        ESP_LOGW(TAG, "WARNING: Valve over-rotated! Position: %.1f%%", percent_open);
    } else {
        ESP_LOGI(TAG, "Valve is %.1f%% open", percent_open);
    }

    // Step 5: Read current single-rotation angle (within current turn)
    float current_angle;
    as5600_read_angle_degrees(&valve_encoder, &current_angle);
    ESP_LOGI(TAG, "Current angle within rotation: %.1f°", current_angle);

    as5600_free_desc(&valve_encoder);
}

/*******************************************************************************
 * EXAMPLE 6: Magnet Status Checking (Hardware Diagnostics)
 ******************************************************************************/
void example_6_magnet_diagnostic(void)
{
    ESP_LOGI(TAG, "=== Example 6: Magnet Status Diagnostics ===");

    as5600_dev_t sensor = {0};

    // Initialize
    as5600_init_desc(&sensor, I2C_NUM_0, AS5600_DEFAULT_ADDRESS,
                     GPIO_NUM_21, GPIO_NUM_22);
    as5600_init(&sensor);

    // Read hardware status
    as5600_status_t status;
    esp_err_t ret = as5600_read_status(&sensor, &status);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "=== AS5600 Hardware Status ===");
        ESP_LOGI(TAG, "Magnet detected: %s", status.magnet_detected ? "YES" : "NO");
        ESP_LOGI(TAG, "Magnet too weak: %s", status.magnet_too_weak ? "YES" : "NO");
        ESP_LOGI(TAG, "Magnet too strong: %s", status.magnet_too_strong ? "YES" : "NO");

        // Interpret status
        if (!status.magnet_detected) {
            ESP_LOGE(TAG, "ERROR: No magnet detected! Check:");
            ESP_LOGE(TAG, "  - Magnet installed?");
            ESP_LOGE(TAG, "  - Magnet alignment with sensor?");
            ESP_LOGE(TAG, "  - Distance from sensor surface?");
        } else if (status.magnet_too_weak) {
            ESP_LOGW(TAG, "WARNING: Magnetic field too weak!");
            ESP_LOGW(TAG, "  - Move magnet closer to sensor");
            ESP_LOGW(TAG, "  - Use stronger magnet (recommended: N35 or better)");
            ESP_LOGW(TAG, "  - Check for magnetic interference");
        } else if (status.magnet_too_strong) {
            ESP_LOGW(TAG, "WARNING: Magnetic field too strong!");
            ESP_LOGW(TAG, "  - Move magnet further from sensor");
            ESP_LOGW(TAG, "  - Use weaker magnet");
        } else {
            ESP_LOGI(TAG, "Magnet status: OPTIMAL");
        }
    }

    as5600_free_desc(&sensor);
}

/*******************************************************************************
 * EXAMPLE 7: Power Mode Comparison
 ******************************************************************************/
void example_7_power_modes(void)
{
    ESP_LOGI(TAG, "=== Example 7: Power Mode Selection ===");

    as5600_dev_t sensor = {0};
    as5600_init_desc(&sensor, I2C_NUM_0, AS5600_DEFAULT_ADDRESS,
                     GPIO_NUM_21, GPIO_NUM_22);
    as5600_init(&sensor);

    /*
     * Power Mode Selection Guide:
     *
     * AS5600_POWER_MODE_NORMAL (0):
     *   - Current: 6.5mA
     *   - Polling: Continuous
     *   - Use when: Highest accuracy needed, AC powered
     *   - Example: High-precision robotics, CNC machines
     *
     * AS5600_POWER_MODE_LPM1 (1):
     *   - Current: 3.4mA
     *   - Polling: 5ms intervals
     *   - Use when: Good balance, moderate battery life
     *   - Example: Portable devices with frequent updates
     *
     * AS5600_POWER_MODE_LPM2 (2):
     *   - Current: 1.8mA
     *   - Polling: 20ms intervals
     *   - Use when: Battery powered, 50Hz update rate acceptable
     *   - Example: Slow-moving applications, solar powered
     *
     * AS5600_POWER_MODE_LPM3 (3):
     *   - Current: 1.5mA (saves ~5mA vs NORMAL)
     *   - Polling: 100ms intervals
     *   - Use when: Maximum battery life, slow movements
     *   - Example: Wind speed (TEMPESTA), slow actuators
     */

    // Example: Solar-powered wind speed sensor (TEMPESTA use case)
    ESP_LOGI(TAG, "Setting to LPM3 for wind speed measurement...");
    as5600_set_power_mode(&sensor, AS5600_POWER_MODE_LPM3);
    ESP_LOGI(TAG, "Power savings: ~5mA (6.5mA → 1.5mA)");

    // Read at low power
    vTaskDelay(pdMS_TO_TICKS(1000));
    uint16_t counts;
    as5600_read_raw_counts(&sensor, &counts);
    ESP_LOGI(TAG, "Reading at LPM3: %u counts", counts);

    // Switch back to normal for high-speed operation
    ESP_LOGI(TAG, "Switching to NORMAL mode for high-speed tracking...");
    as5600_set_power_mode(&sensor, AS5600_POWER_MODE_NORMAL);

    as5600_free_desc(&sensor);
}

/*******************************************************************************
 * EXAMPLE 8: Continuous Monitoring Task
 ******************************************************************************/
void monitoring_task(void *pvParameters)
{
    as5600_dev_t *sensor = (as5600_dev_t *)pvParameters;

    ESP_LOGI(TAG, "=== Continuous Monitoring Started ===");

    while (1) {
        float angle;
        as5600_status_t status;

        // Read current angle
        if (as5600_read_angle_degrees(sensor, &angle) == ESP_OK) {
            ESP_LOGI(TAG, "Angle: %.2f°", angle);
        }

        // Periodically check magnet status (every 10 seconds)
        static uint32_t check_count = 0;
        if (++check_count >= 20) { // 20 * 500ms = 10s
            check_count = 0;

            if (as5600_read_status(sensor, &status) == ESP_OK) {
                if (!status.magnet_detected || status.magnet_too_weak ||
                    status.magnet_too_strong) {
                    ESP_LOGW(TAG, "Magnet issue detected!");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Update at 2Hz
    }
}

void example_8_continuous_monitoring(void)
{
    ESP_LOGI(TAG, "=== Example 8: Continuous Monitoring ===");

    static as5600_dev_t sensor = {0}; // Static for task access

    as5600_init_desc(&sensor, I2C_NUM_0, AS5600_DEFAULT_ADDRESS,
                     GPIO_NUM_21, GPIO_NUM_22);
    as5600_init(&sensor);

    // Create monitoring task
    xTaskCreate(monitoring_task, "as5600_monitor", 4096, &sensor, 5, NULL);

    ESP_LOGI(TAG, "Monitoring task created (runs forever)");

    // Note: Don't call as5600_free_desc() if task is running!
}

/*******************************************************************************
 * Main entry point (for demonstration)
 ******************************************************************************/
void app_main(void)
{
    ESP_LOGI(TAG, "\n\n");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   AS5600 Driver Usage Examples");
    ESP_LOGI(TAG, "========================================\n");

    // Uncomment the example you want to run:

    // example_1_basic_usage();
    // example_2_robot_arm_calibration();
    // example_3_wind_speed_anemometer();
    // example_4_robot_wheel_odometer();
    // example_5_valve_actuator();
    // example_6_magnet_diagnostic();
    // example_7_power_modes();
    // example_8_continuous_monitoring();

    ESP_LOGI(TAG, "\nAll examples completed!");
    ESP_LOGI(TAG, "See function_selection_guide() for quick reference");
}
