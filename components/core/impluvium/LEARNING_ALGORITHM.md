# IMPLUVIUM Learning Algorithm Documentation

## Overview

The **IMPLUVIUM** irrigation system implements an **adaptive learning algorithm** that continuously improves watering precision based on historical performance data. The system learns optimal water quantities, pump speeds, and timing for each irrigation zone independently, while integrating environmental data from the **TEMPESTA** weather station.

**Classification**: Classical online supervised learning (exponential weighted moving average)

## System Architecture

### Component Integration
- **IMPLUVIUM**: Core learning algorithm and watering control
- **TEMPESTA**: Environmental data (temperature for adaptive correction)
- **FLUCTUS**: Power management (learning paused during power save modes)
- **TELEMETRY**: Data aggregation and MQTT export

### Learning Data Structures

Each of the 5 zones maintains independent learning state:

```c
typedef struct {
    // Historical data (circular buffer)
    float pulses_used_history[15];               // Last 15 watering cycles
    float moisture_increase_percent_history[15]; // Corresponding moisture gains (%)
    bool anomaly_flags[15];                      // Marks invalid cycles (rain, errors)
    uint8_t history_entry_count;                 // Valid entries (0-15)
    uint8_t history_index;                       // Current write position

    // Learned parameters (updated via exponential moving averages)
    float calculated_ppmp_ratio;                 // Pulses per moisture percent
    uint32_t calculated_pump_duty_cycle;         // Optimal pump PWM duty
    float target_moisture_gain_rate;             // Optimal gain rate (%/sec)

    // Confidence tracking (prediction accuracy metrics)
    float confidence_level;                      // Success rate (0.0-1.0)
    uint32_t successful_predictions;             // Count of accurate predictions
    uint32_t total_predictions;                  // Total prediction attempts

    // Temperature integration
    float last_temperature_correction;           // Last calculated temp factor
} zone_learning_t;
```

## Complete Learning Workflow

### Phase 1: System Initialization

**When**: `impluvium_init()` at system boot

**Actions**:
1. **Load from LittleFS** (`/data/irrigation/learning.dat`):
   - 5 most recent valid cycles per zone (not full 15-cycle history)
   - Learned parameters (ppmp_ratio, pump_duty, gain_rate, confidence)
   - Only saved once daily at midnight to minimize flash wear

2. **Initialize defaults** (if no stored data):
   ```c
   calculated_ppmp_ratio = 8.0f;              // 8 pulses per 1% moisture
   calculated_pump_duty_cycle = PUMP_DEFAULT_DUTY;  // 512 (50%)
   target_moisture_gain_rate = 0.5f;          // 0.5%/sec target
   confidence_level = 0.0f;                   // No confidence yet
   ```

3. **Learning thresholds**:
   - `LEARNING_MIN_CYCLES = 3`: Minimum cycles before predictions start
   - `LEARNING_HISTORY_SIZE = 15`: RAM circular buffer size
   - `LEARNING_STORED_HISTORY = 5`: Flash storage limit (wear reduction)
   - `LEARNING_WEIGHT_RECENT = 0.7f`: Weight for recent cycles

### Phase 2: Pre-Watering Prediction (MEASURING State)

**Trigger**: Timer-based moisture check (15min normal, 60min power-save)

**Function**: `impluvium_calculate_zone_target_pulses()`

#### Step 2.1: Temperature Correction Calculation

```c
// Get current temperature from TEMPESTA
float current_temperature = tempesta_get_temperature();

// Calculate correction factor: ±1% per degree from 20°C baseline
float temp_correction = 1.0f + ((current_temperature - 20.0f) * 0.01f);
```

**Examples**:
- **25°C** → 1.05 (5% more water needed due to evaporation)
- **15°C** → 0.95 (5% less water needed due to reduced evaporation)
- **Sensor failure** → 1.0 (neutral, no correction applied)

#### Step 2.2: Learning Phase Decision

**Initial Phase** (< 3 cycles):
- Use fixed default: `DEFAULT_TARGET_PULSES = 80` (≈200mL)
- System is gathering baseline data

**Learned Phase** (≥ 3 cycles):
- Calculate weighted average from historical data
- Apply confidence-based blending (see Step 2.3)

#### Step 2.3: Weighted Historical Analysis

```c
// Calculate weighted average of pulses-per-percent from valid cycles
for (each valid cycle in history) {
    // Determine recency (circular buffer logic)
    relative_age = (history_index - h - 1 + 15) % 15;

    // Recent 3 cycles get 70% weight, older ones get 30%
    weight = (relative_age < 3) ? 0.7f : 0.3f;

    if (!anomaly_flags[h] && moisture_increase > 0) {
        ratio = pulses_used / moisture_increase_percent;
        weighted_sum += ratio * weight;
        total_weight += weight;
    }
}
learned_ratio = weighted_sum / total_weight;
```

**Requires**: Minimum 2 valid (non-anomalous) cycles

#### Step 2.4: Confidence-Based Prediction Blending

**NEW FEATURE** (not in original docs): 3-tier confidence system

```c
// Calculate both predictions
learned_prediction = moisture_deficit * learned_ratio * temp_correction;
default_prediction = moisture_deficit * DEFAULT_PPMP * temp_correction;

if (confidence >= 0.70f) {
    // High confidence: Use 100% learned prediction
    target_pulses = learned_prediction;

} else if (confidence >= 0.40f) {
    // Medium confidence: Blend based on confidence
    blend_factor = (confidence - 0.40f) / 0.30f;  // 0.0 to 1.0
    target_pulses = (learned_prediction * blend_factor) +
                    (default_prediction * (1.0f - blend_factor));

} else {
    // Low confidence: Use 80% default + 20% learned
    target_pulses = (default_prediction * 0.8f) + (learned_prediction * 0.2f);
}

// Safety bounds: clamp to [20, 300] pulses
```

**Why**: Prevents bad predictions during initial learning or after many anomalies

### Phase 3: Watering Execution (WATERING State)

**Function**: `impluvium_state_watering()`

**Process**:
1. **Power Management**: Request 12V bus for valves and pumps, enable level shifter (SN74AHCT125)
2. **STELLARIA Dimming**: Request 5% dimming for 12V bus load reduction (1s stabilization delay)
3. **Valve Control**: Open zone valve (1.5s actuation delay)
4. **Pump Ramp-Up**: Start pump at learned duty cycle (or default 50%), 5s ramp
5. **Flow Monitoring**: Count pulses from flow sensor, 3s flow establishment delay
6. **Real-Time Safety**:
   - Emergency cutoffs (pressure, flow, overcurrent)
   - Anomaly detection (see Section 5)
   - Max 60-second timeout enforcement

**Manual Watering Mode**:
- Triggered via HMI (5-300 seconds, 5s increments)
- **Bypasses MEASURING state**: Creates single-item queue directly (no moisture checks)
- Uses learned pump duty but **bypasses all learning updates**
- Only respects pressure safety limits
- Used for testing or forcing water regardless of moisture

### Phase 4: Post-Watering Learning Update (STOPPING State)

**Function**: `impluvium_state_stopping()` → `impluvium_process_zone_watering_data()`

#### Step 4.1: Shutdown and Data Collection

**Shutdown sequence**:
1. Pump ramp-down (3s for gentle hardware stress reduction)
2. Pressure equalization delay (1s)
3. Restore STELLARIA to previous intensity
4. Close valve, release level shifter

**Data collection**:
```c
total_pulses = flow_sensor_final - flow_sensor_start;
volume_ml = (total_pulses / PULSES_PER_LITER) * 1000;
final_moisture = read_moisture_sensor(zone);
moisture_increase = final_moisture - initial_moisture;
watering_duration = end_time - start_time;
```

**RTC RAM Accumulator** (NEW - not in original docs):
- Per-zone hourly/daily water usage tracked in RTC RAM
- Survives ESP32 resets (but not power loss)
- Provides statistics without reading LittleFS
- Auto-resets at midnight and top of hour

#### Step 4.2: Anomaly Detection

**Cycles marked as anomalous** (excluded from learning):

1. **Rain detection**: Current anomaly flag set during watering
2. **Temperature extremes**: < 5°C or > 45°C
3. **Sensor failures**: Temperature sensor unavailable
4. **System anomalies**: Pressure/flow alarms during cycle
5. **Manual watering**: Explicitly bypassed

**Impact**: Anomalous cycles stored in history but flagged, excluded from weighted calculations

#### Step 4.3: Circular Buffer Storage

```c
index = learning->history_index;
learning->pulses_used_history[index] = pulses_used;
learning->moisture_increase_percent_history[index] = moisture_increase_percent;
learning->anomaly_flags[index] = !learning_valid;

// Advance circular buffer
learning->history_index = (learning->history_index + 1) % 15;
learning->history_entry_count = min(history_entry_count + 1, 15);
```

**RAM**: Full 15-cycle history
**Flash**: Only 5 most recent valid cycles (saved daily at midnight)

#### Step 4.4: Real-Time Learning Updates (Valid Cycles Only)

**1. Pulses-Per-Moisture-Percent Ratio** (fast adaptation):
```c
measured_ratio = pulses_used / moisture_increase_percent;

if (measured_ratio > 0.1f && measured_ratio < 50.0f) {  // Sanity bounds
    // Exponential moving average: 10% new, 90% old
    learning->calculated_ppmp_ratio =
        (learning->calculated_ppmp_ratio * 0.9f) + (measured_ratio * 0.1f);
}
```

**2. Pump Duty Cycle** (moderate adaptation):
```c
// Only after ≥3 learning cycles
if (history_entry_count >= LEARNING_MIN_CYCLES) {
    // Exponential moving average: 10% new, 90% old
    learning->calculated_pump_duty_cycle =
        (calculated_pump_duty_cycle * 0.9f) + (current_pump_duty * 0.1f);
}
```

**3. Target Moisture Gain Rate** (slow, stable adaptation):
```c
actual_gain_rate = moisture_increase / (watering_duration_sec);

if (duration 5-60s && gain_rate 0.1-2.0 %/sec) {  // Reasonable bounds
    // Exponential moving average: 5% new, 95% old
    learning->target_moisture_gain_rate =
        (target_gain_rate * 0.95f) + (actual_gain_rate * 0.05f);
}
```

**Why different rates**:
- PPMP ratio: Direct measurement, can adapt quickly
- Pump duty: Mechanical system, moderate adaptation prevents oscillation
- Gain rate: Soil characteristic, slow adaptation for stability

#### Step 4.5: Confidence Tracking

```c
if (learning_valid) {
    learning->total_predictions++;

    // Calculate prediction accuracy
    expected_increase = pulses_used / learning->calculated_ppmp_ratio;
    accuracy = |moisture_increase - expected_increase| / expected_increase;

    // Within 20% tolerance = successful prediction
    if (accuracy <= 0.20f) {
        learning->successful_predictions++;
    }

    // Update confidence level (success rate)
    learning->confidence_level = successful_predictions / total_predictions;
}
```

**Drives**: Confidence-based blending in Phase 2 (Step 2.4)

### Phase 5: Persistent Storage (MAINTENANCE State)

**Trigger**: Daily at midnight via `solar_calc` callback

**Function**: `impluvium_save_learning_data_all_zones()`

**File**: `/data/irrigation/learning.dat` (~350 bytes)

**What's saved per zone**:
- Learned parameters (ppmp_ratio, pump_duty, gain_rate)
- Confidence metrics (level, successful_predictions, total_predictions)
- **Only 5 most recent valid cycles** (not full 15-cycle RAM history)
- CRC16-CCITT checksum for integrity

**Why only 5 cycles**:
- Minimize flash wear (daily writes × 5 zones × 7 years = 12,775 cycles)
- Sufficient for cold-start learning (3 cycles needed)
- Full 15-cycle history rebuilt during normal operation

**Daily reset also**:
- Clears daily water usage counters
- Rolls over RTC accumulator statistics

## Environmental Integration

### Temperature Adaptive Correction

**Formula**: `temp_correction = 1.0 + ((current_temp - 20.0°C) * 0.01)`

**Physical basis**:
- Higher temps → increased evaporation → more water needed
- Lower temps → reduced plant uptake → less water needed

**Integration points**:
1. **Pre-watering**: Applied to all predictions (learned and default)
2. **Post-watering**: Stored in `last_temperature_correction` for diagnostics
3. **Anomaly detection**: Extreme temps (< 5°C, > 45°C) invalidate learning

### Weather Data Integration

1. **Global safety limits**: 0°C to 50°C operational range
2. **Moisture check intervals**: Temperature-based timing (future feature)
3. **Learning validity**: Temperature used to flag anomalous cycles

## Learning Algorithm Parameters

### Constants and Thresholds
```c
#define LEARNING_HISTORY_SIZE 15           // RAM circular buffer size
#define LEARNING_STORED_HISTORY 5          // Flash storage limit
#define LEARNING_MIN_CYCLES 3              // Start predictions after 3 cycles
#define LEARNING_WEIGHT_RECENT 0.7f        // Recent cycles (vs 0.3f old)
#define DEFAULT_TARGET_PULSES 80           // ≈200mL starting point
#define DEFAULT_PULSES_PER_PERCENT 8.0f    // Default efficiency
#define MINIMUM_TARGET_PULSES 20           // Safety minimum
#define MAXIMUM_TARGET_PULSES 300          // Safety maximum
#define TARGET_MOISTURE_GAIN_RATE 0.5f     // Default (%/sec)
#define TEMP_CORRECTION_FACTOR 0.01f       // ±1% per °C
#define TEMPERATURE_BASELINE 20.0f         // Reference temperature
```

### Learning Rates (Exponential Moving Averages)
- **Pulses-per-percent ratio**: 10% new, 90% old (fast adaptation)
- **Pump duty cycle**: 10% new, 90% old (moderate adaptation)
- **Target gain rate**: 5% new, 95% old (slow, stable adaptation)

### Confidence Thresholds
- **High (≥70%)**: Use 100% learned prediction
- **Medium (40-69%)**: Blend learned + default based on confidence
- **Low (< 40%)**: Use 80% default + 20% learned

### Anomaly Detection Bounds
- **Efficiency**: 0.5 to 50.0 pulses per percent
- **Gain rate**: 0.1 to 2.0 %/sec
- **Duration**: 5 to 60 seconds (valid learning cycles)
- **Temperature**: 5°C to 45°C (extreme temps invalidate learning)

## Multi-Zone Learning Characteristics

### Per-Zone Independence

Each zone adapts to its unique conditions:

**Clay Soil Zones**:
- Lower gain rates (0.2-0.3 %/sec)
- Higher pulse requirements (slow absorption)
- Higher pump duty may be needed

**Sandy Soil Zones**:
- Higher gain rates (0.6-0.8 %/sec)
- Lower pulse requirements (fast drainage)
- Lower pump duty sufficient

**Mixed/Loam Zones**:
- Moderate values based on actual performance
- Typical gain rates (0.4-0.6 %/sec)

### Zone-Specific Learned Parameters

1. **Pulses per Percent**: Water efficiency for soil type
2. **Pump Duty Cycle**: Optimal pump speed for zone's hydraulic resistance
3. **Target Gain Rate**: Optimal watering speed for soil absorption
4. **Confidence Level**: Prediction accuracy history (builds over weeks)

## Error Handling and Recovery

### Data Validation

All learned values checked against sanity bounds:
- **Efficiency**: 0.5 to 50.0 pulses/%
- **Gain rate**: 0.1 to 2.0 %/sec
- **Duration**: 5 to 60 seconds
- **Temperature**: -50°C to 100°C sensor range

Out-of-bounds values → use previous valid value, no update

### Anomaly Recovery Strategies

**Isolated anomalies** (1-2 recent cycles):
- Stored with anomaly flag
- Ignored in weighted calculations
- System continues using older valid data

**Multiple consecutive anomalies** (> 50% of history):
- Confidence drops
- Prediction blending shifts toward defaults
- System remains safe and functional

**Sensor failures**:
- Learning paused until sensors recover
- Last known good values used
- Temperature correction defaults to 1.0 (neutral)

### Confidence-Based Fallbacks

- **Low confidence** (< 50%): Blended predictions reduce risk
- **Very low confidence** (< 20%): Mostly default predictions
- **No historical data**: Pure default until 3 cycles accumulated

## Performance Metrics and Monitoring

### Key Performance Indicators

1. **Prediction Accuracy**: % of predictions within 20% tolerance
2. **Confidence Level**: Per-zone success rate (0-100%)
3. **Water Efficiency**: Pulses per 1% moisture gain (lower = better)
4. **Adaptation Speed**: Cycles to reach stable predictions (typically 5-10)
5. **Anomaly Rate**: % of cycles flagged as invalid

### Logging Levels

**INFO level** (standard operation):
- Learning phase status (initial vs learned mode)
- Confidence-based prediction choices
- Real-time parameter updates (ppmp, pump duty, gain rate)
- Per-zone learning progress

**DEBUG level** (detailed diagnostics):
- Weighted calculation details for each historical cycle
- Temperature correction factors
- Circular buffer index management
- Individual cycle accuracy calculations

### Telemetry Integration

**Snapshot data** (pushed to TELEMETRY cache):
- Current learned parameters (ppmp, pump duty, gain rate)
- Confidence metrics (level, successful/total predictions)
- History entry count (0-15)
- Last temperature correction factor

**Real-time data** (500ms updates during watering):
- Current moisture gain rate
- Pump duty adjustments
- Flow/pressure anomaly flags

## System Integration

### Power Management (FLUCTUS)

**Normal operation**:
- 15-minute moisture check intervals
- Full learning enabled

**Power-save mode** (SOC < 40%):
- 60-minute moisture check intervals
- Learning continues but less frequent

**Shutdown** (SOC < 0%):
- All watering disabled
- Learning paused
- Resume when power restored

### Safety System Integration

**Emergency stops**:
- Current watering cycle marked as anomaly
- No learning data stored
- Safety always overrides learning

**Flow/pressure anomalies**:
- Automatic learning invalidation for current cycle
- Watering stopped immediately
- Emergency diagnostics may be triggered

**Sensor failures**:
- Learning paused for affected zones
- Confidence tracking suspended
- Default predictions used

### Manual Overrides

**Manual watering** (HMI-triggered):
- Uses learned pump duty for efficiency
- Bypasses all moisture checks
- **Does not update learning data**
- Duration: 5-300 seconds (user-specified)

**Learning reset** (per-zone or all):
- Clears history buffers
- Resets to default parameters
- Confidence set to 0.0
- Does not delete LittleFS file (overwritten at next save)

## State Machine Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                        IRRIGATION STATES                         │
└─────────────────────────────────────────────────────────────────┘

    ┌──────────────┐
    │   STANDBY    │  Wait for moisture check timer
    │              │  (15min normal / 60min power-save)
    └──────┬───────┘
           │ Timer fires
           ↓
    ┌──────────────┐
    │  MEASURING   │  1. Power on sensors (3V3+5V)
    │              │  2. Read all zone moisture levels
    │              │  3. Build watering queue (prioritize by deficit)
    │              │  4. Calculate predictions (learning algorithm)
    │              │  * Manual mode: skip 2-4, create single-item queue
    └──────┬───────┘
           │ Queue ready
           ↓
    ┌──────────────┐
    │  WATERING    │  1. Power on valves/pump (12V), enable level shifter
    │              │  2. Dim STELLARIA to 5% (1s delay)
    │   (loops for │  3. Open valve, ramp pump at learned duty (5s)
    │  each zone)  │  4. Monitor flow/pressure (500ms)
    │              │  5. Stop at target pulses or timeout (manual: time-based)
    └──────┬───────┘
           │ Zone complete
           ↓
    ┌──────────────┐
    │  STOPPING    │  1. Ramp down pump (3s), equalize pressure (1s)
    │              │  2. Restore STELLARIA, close valve, release level shifter
    │              │  3. Read final moisture (skip if manual)
    │              │  4. Update learning algorithm (skip if manual)
    │              │  5. Update RTC accumulator
    └──────┬───────┘
           │ More zones? → Loop to WATERING
           │ All done? ↓
    ┌──────────────┐
    │ MAINTENANCE  │  1. Save learning data (if midnight)
    │              │  2. Run emergency diagnostics (if needed)
    │              │  3. Release power buses
    └──────┬───────┘
           │ Complete
           ↓
    ┌──────────────┐
    │   STANDBY    │  Back to waiting...
    └──────────────┘

    Manual water → WATERING (bypass MEASURING, skip learning update)
    Emergency → DISABLED (user reset required)
```

## Workflow Summary

1. **Initialization**: Load 5 recent cycles from flash, set defaults
2. **Prediction**: Weighted average + temp correction + confidence blending
3. **Execution**: Use learned pump duty, monitor for anomalies
4. **Learning**: Update ppmp/pump/gain with exponential moving averages
5. **Storage**: Save to flash once daily at midnight (wear reduction)

All while maintaining safety, handling sensor failures gracefully, and preventing overfitting through confidence-based blending.
