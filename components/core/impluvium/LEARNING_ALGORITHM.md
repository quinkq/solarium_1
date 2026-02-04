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
    float ppmp_ratio_history[15];                // Pre-computed PPMP ratios (last 15 cycles)
    bool anomaly_flags[15];                      // Marks invalid cycles (rain, errors)
    uint8_t history_entry_count;                 // Valid entries (0-15)
    uint8_t history_index;                       // Current write position

    // Temperature integration
    float last_temperature_correction;           // Last calculated temp factor

    // Learned parameters
    float calculated_ppmp_ratio;                 // Weighted average PPMP ratio
    uint32_t calculated_pump_duty_cycle;         // Optimal pump PWM duty (0-1023)
    float soil_redistribution_factor;            // Moisture redistribution factor (1.0-3.0)
    float measured_moisture_gain_rate;           // Most recent gain rate (%/sec, telemetry)

    // Confidence tracking (prediction accuracy metrics)
    float confidence_level;                      // EMA-based confidence (0.0-1.0)
    uint32_t successful_predictions;             // Count of accurate predictions (legacy)
    uint32_t total_predictions;                  // Total prediction attempts (legacy)
} zone_learning_t;
```

## Complete Learning Workflow

### Phase 1: System Initialization

**When**: `impluvium_init()` at system boot

**Actions**:
1. **Load from LittleFS** (`/data/irrigation/learning.dat`):
   - 5 most recent valid cycles per zone (pre-computed PPMP ratios)
   - Learned parameters (ppmp_ratio, pump_duty, soil_redistribution_factor, confidence)
   - Only saved once daily at midnight to minimize flash wear

2. **Initialize defaults** (if no stored data):
   ```c
   calculated_ppmp_ratio = 15.0f;                     // 15 pulses per 1% moisture
   calculated_pump_duty_cycle = PUMP_DEFAULT_DUTY;    // 512 (50%)
   soil_redistribution_factor = 1.5f;                 // Conservative redistribution estimate
   confidence_level = 0.0f;                           // No confidence yet
   ```

3. **Learning thresholds**:
   - `LEARNING_MIN_CYCLES = 1`: Minimum cycles in history (enforced as 2 valid cycles via function logic)
   - `LEARNING_HISTORY_SIZE = 15`: RAM circular buffer size
   - `LEARNING_STORED_HISTORY = 5`: Flash storage limit (wear reduction)
   - `LEARNING_WEIGHT_RECENT = 0.7f`: Weight for recent cycles (last 3)

### Phase 2: Pre-Watering Prediction (MEASURING State)

**Trigger**: Timer-based moisture check (default 15min normal, 60min power-save)

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
// Calculate weighted average from pre-computed PPMP ratios stored in history
for (age = 0; age < history_entry_count; age++) {
    // Traverse circular buffer: age=0 is most recent
    index = (history_index - age - 1 + 15) % 15;

    ppmp_ratio = ppmp_ratio_history[index];

    if (!anomaly_flags[index] && ppmp_ratio > 0.0f) {
        // Recent 3 cycles get 70% weight, older ones get 30%
        weight = (age < 3) ? 0.7f : 0.3f;

        weighted_sum += ppmp_ratio * weight;
        total_weight += weight;
        valid_cycles++;
    }
}

if (valid_cycles < 2 || total_weight <= 0) {
    return 0.0f;  // Insufficient data, use defaults
}

learned_ratio = weighted_sum / total_weight;

// Clamp to hardware-safe bounds (wider than prediction bounds for tolerance)
if (learned_ratio < MINIMUM_PULSES_PER_MOISTURE_PERCENT) {    // 2.0f
    learned_ratio = MINIMUM_PULSES_PER_MOISTURE_PERCENT;
}
else if (learned_ratio > MAXIMUM_PULSES_PER_MOISTURE_PERCENT) { // 100.0f
    learned_ratio = MAXIMUM_PULSES_PER_MOISTURE_PERCENT;
}
```

**Requires**: Minimum 2 valid (non-anomalous) cycles

**Safety validation**: PPMP is clamped to [2.0, 100.0] pulses/% range to prevent hardware damage from sensor glitches or corrupted data. Values outside this range are clamped (not rejected) to allow gradual correction.

#### Step 2.4: Confidence-Based Prediction Blending

**2-tier linear interpolation system**

```c
// Calculate both predictions
learned_prediction = moisture_deficit * learned_ratio * temp_correction;
default_prediction = moisture_deficit * DEFAULT_PPMP * temp_correction;

if (confidence >= HIGH_CONFIDENCE_THRESHOLD) {  // 0.70f
    // High confidence: Use 100% learned prediction
    target_pulses = learned_prediction;
    ESP_LOGI("High confidence (%.0f%%), using 100%% learned", confidence * 100.0f);

} else {
    // Below threshold: Linear interpolation from 0% learned (at conf=0) to 100% learned (at conf=0.70)
    blend_factor = confidence / HIGH_CONFIDENCE_THRESHOLD;  // Maps [0.0, 0.70] → [0.0, 1.0]
    target_pulses = (learned_prediction * blend_factor) +
                    (default_prediction * (1.0f - blend_factor));
    ESP_LOGI("Confidence %.0f%%, blending %.0f%% learned + %.0f%% default",
             confidence * 100.0f, blend_factor * 100.0f, (1.0f - blend_factor) * 100.0f);
}

// Safety bounds: clamp to [20, 600] pulses (50mL to 1500mL)
target_pulses = CLAMP(target_pulses, MINIMUM_TARGET_PULSES, MAXIMUM_TARGET_PULSES);
```

**Why**: Simple linear interpolation provides smooth transition from default to learned behavior as confidence builds. Examples:
- **conf = 0.00**: 0% learned + 100% default (safe initial state)
- **conf = 0.35**: 50% learned + 50% default (transitioning)
- **conf = 0.70+**: 100% learned + 0% default (optimized)

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

**1. Pulses-Per-Moisture-Percent Ratio** (stored directly):
```c
// Calculate PPMP for this cycle
measured_ppmp_ratio = (moisture_increase > 0.0f)
                        ? (pulses_used / moisture_increase)
                        : 0.0f;

// Store in circular buffer history (no EMA here, averaging happens at prediction time)
learning->ppmp_ratio_history[index] = measured_ppmp_ratio;
```

**Note**: PPMP is stored as-is. The weighted average calculation happens during prediction (Phase 2), not during storage. This allows flexible weighting schemes (recent vs old cycles) without re-calculating historical data.

**2. Pump Duty Cycle** (ratio-based adjustment):
```c
// Calculate measured gain rate
measured_gain_rate = moisture_increase / (watering_duration_ms / 1000.0f);

// Store unclamped for telemetry
learning->measured_moisture_gain_rate = measured_gain_rate;

// Clamp for calculations
clamped_gain_rate = CLAMP(measured_gain_rate, 0.1f, 3.0f);

// Calculate adjustment ratio toward target (0.5 %/sec default)
gain_ratio = target_moisture_gain_rate / clamped_gain_rate;

// Limit adjustment to ±15% per cycle to prevent oscillation
gain_ratio = CLAMP(gain_ratio, 0.85f, 1.15f);

// Apply proportional adjustment
uint32_t new_duty = (uint32_t)(old_duty * gain_ratio);

// Clamp to hardware limits [434, 1023] (42-100%)
learning->calculated_pump_duty_cycle = CLAMP(new_duty, PUMP_MIN_DUTY, PUMP_MAX_DUTY);
```

**Why ratio-based instead of EMA**:
- Directly targets desired gain rate (0.5 %/sec default)
- If watering too fast → reduce pump duty
- If watering too slow → increase pump duty
- ±15% clamp prevents oscillation from noisy measurements
- More responsive than EMA to changing conditions (soil compaction, filter clogs)

#### Step 4.5: Confidence Tracking (Exponential Moving Average)

**New approach** (fixes "confidence trap" issue): Uses smooth gradient scoring with EMA instead of cumulative counters.

```c
if (learning_valid) {
    float prediction_quality = 0.0f;

    // Calculate prediction quality with smooth gradient scoring
    if (learning->calculated_ppmp_ratio > 0) {
        expected_increase = pulses_used / learning->calculated_ppmp_ratio;
        relative_error = |moisture_increase - expected_increase| / expected_increase;

        // Smooth gradient: 1.0 at 0% error, linearly decreasing to 0.0 at 30% error
        // Avoids harsh binary thresholds, rewards near-misses proportionally
        if (relative_error <= 0.30f) {
            prediction_quality = 1.0f - (relative_error / 0.30f);
        }
        // Errors > 30% contribute 0.0 quality
    }

    // Update confidence using Exponential Moving Average (20% weight to new result)
    // Higher weight allows faster adaptation to changing conditions (pump wear, soil changes)
    learning->confidence_level = (confidence_level * 0.80f) + (prediction_quality * 0.20f);

    // Confidence boost: Accelerate convergence when prediction is good but confidence low
    if (learning->history_entry_count >= LEARNING_MIN_CYCLES &&
        prediction_quality >= 0.50f &&
        learning->confidence_level < CONFIDENCE_BOOST_FLOOR) {  // 0.25f
        learning->confidence_level = CONFIDENCE_BOOST_FLOOR;
    }

    // Update legacy counters for telemetry/logging (not used for confidence calculation)
    learning->total_predictions++;
    if (prediction_quality >= 0.33f) {  // Equivalent to ≤20% error threshold
        learning->successful_predictions++;
    }
}
```

**Quality Score Examples**:
- 0% error → quality = 1.0 (perfect)
- 10% error → quality = 0.67 (good)
- 20% error → quality = 0.33 (marginal)
- 30%+ error → quality = 0.0 (failed)

**Why EMA vs Cumulative**:
- **Old problem**: After 1000 successes, even 20 consecutive failures barely affected confidence (89% → 82%)
- **New solution**: Confidence adapts within 2-5 cycles to hardware changes (pump degradation, filter clogs)
- **20% EMA weight**: Faster adaptation than 10%, balances responsiveness with stability
- **Smooth transitions**: Works seamlessly with 2-tier confidence-based blending (no discontinuities)

**Confidence Boost Floor**:
- When prediction quality ≥0.50 (≤20% error) but confidence <0.25, boost to 0.25 immediately
- Accelerates initial learning convergence after first good prediction
- Prevents extended periods in low-confidence state when algorithm is performing well

**Drives**: Confidence-based blending in Phase 2 (Step 2.4)

#### Step 4.6: Soil Redistribution Factor (RF) Learning

**Purpose**: Measures how moisture redistributes upward through soil after watering stops.

**Physical Basis**:
- Water applied at top redistributes downward and upward via capillary action
- Immediate moisture reading (at watering stop) underestimates final absorbed moisture
- Delayed reading (5 minutes later) captures redistributed moisture

**Update Process**:
```c
// Triggered 5 minutes after watering completes via verification timer
float immediate_increase = moisture_immediate - moisture_start;
float delayed_increase = moisture_delayed - moisture_start;

// Validate: delayed should be >= immediate, and immediate should be meaningful
if (delayed_increase > immediate_increase && immediate_increase > 0.5f) {
    float measured_factor = delayed_increase / immediate_increase;

    // Clamp to valid soil range [1.0, 3.0]
    measured_factor = CLAMP(measured_factor, 1.0f, 3.0f);

    // EMA update: 15% weight to new measurement (slow-changing soil property)
    learning->soil_redistribution_factor =
        (learning->soil_redistribution_factor * 0.85f) + (measured_factor * 0.15f);
}
```

**Soil Type Examples**:
- **Sandy soil** (RF ≈ 1.0-1.2): Fast drainage, minimal redistribution
- **Loam** (RF ≈ 1.3-1.7): Moderate redistribution
- **Clay** (RF ≈ 1.8-3.0): Slow drainage, significant upward redistribution

**Default**: 1.5 (conservative mid-range estimate)

**Why 15% EMA weight**: Soil characteristics change slowly (compaction over weeks/months), so slow adaptation prevents noise from affecting stable learned value.

#### Step 4.7: Verification Timer & Delayed Readings

**Trigger**: 5 minutes after watering workflow completes (all zones watered)

**Process**:
1. **Record at watering stop** (per zone):
   - `moisture_at_start_percent`: Pre-watering baseline
   - `moisture_immediate_percent`: Immediate post-watering reading
   - `zone_id`: Which zone was watered

2. **Set verification timer**: 5-minute one-shot timer

3. **Timer fires** → `impluvium_update_redistribution_from_delayed_readings()`:
   - Power on 3.3V + 5V buses (sensor power)
   - Read delayed moisture for each verification zone
   - Calculate RF for each zone (Step 4.6)
   - Release buses, clear verification state

**Batched Operation**: All watered zones processed in single 5-minute delay, minimizing sensor power cycles.

**Why 5 minutes**: Balance between:
- Too short: Redistribution incomplete
- Too long: Evaporation/drainage introduces error

#### Step 4.8: Dynamic Moisture Cutoff

**Purpose**: Stop watering before target moisture to account for redistribution.

**Calculation** (during prediction, Phase 2):
```c
float start_moisture = current_moisture_percent;
float target_moisture = zone->target_moisture_percent;
float rf = learning->soil_redistribution_factor;

dynamic_moisture_cutoff = start_moisture + (target_moisture - start_moisture) / rf;
```

**Example** (target = 60%, RF = 1.5):
- Start: 50%
- Deficit: 10%
- Cutoff: 50% + (10% / 1.5) = **56.7%**
- System stops watering at 56.7%, moisture redistributes to ~60%

**Why not just water to target?**:
- Over-watering: Would reach 60% immediately, redistribute to ~65%
- Runoff risk: Excess water drains away
- Under-cutoff approach: Reaches target after redistribution (no waste)

**Used during**: WATERING state monitoring - cutoff checked every 500ms alongside target pulses

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
// History Management
#define LEARNING_HISTORY_SIZE 15                    // RAM circular buffer size
#define LEARNING_STORED_HISTORY 5                   // Flash storage limit
#define LEARNING_MIN_CYCLES 1                       // Minimum (enforced as 2 via function logic)
#define LEARNING_WEIGHT_RECENT 0.7f                 // Recent 3 cycles (vs 0.3f for older)

// Default Values
#define DEFAULT_TARGET_PULSES 100                   // ≈250mL starting point
#define DEFAULT_PULSES_PER_PERCENT 15.0f            // Default PPMP efficiency
#define DEFAULT_SOIL_REDISTRIBUTION_FACTOR 1.5f     // Conservative RF estimate

// Safety Bounds (Pulse Targets)
#define MINIMUM_TARGET_PULSES 20                    // Safety minimum (~50mL)
#define MAXIMUM_TARGET_PULSES 600                   // Safety maximum (~1500mL)

// Safety Bounds (PPMP Clamping)
#define MINIMUM_PULSES_PER_MOISTURE_PERCENT 2.0f    // Hardware-safe minimum
#define MAXIMUM_PULSES_PER_MOISTURE_PERCENT 100.0f  // Hardware-safe maximum

// Pump Duty Limits
#define PUMP_MIN_DUTY 434                           // 42% (stall protection)
#define PUMP_DEFAULT_DUTY 512                       // 50% (starting point)
#define PUMP_MAX_DUTY 1023                          // 100% (full power)
#define PUMP_DUTY_ADJUSTMENT_MIN_RATIO 0.85f        // Max -15% per cycle
#define PUMP_DUTY_ADJUSTMENT_MAX_RATIO 1.15f        // Max +15% per cycle

// Soil Redistribution Factor
#define MIN_SOIL_REDISTRIBUTION_FACTOR 1.0f         // Sandy (instant absorption)
#define MAX_SOIL_REDISTRIBUTION_FACTOR 3.0f         // Clay (slow redistribution)
#define SOIL_REDISTRIBUTION_EMA_WEIGHT 0.15f        // 15% new, 85% old

// Gain Rate Constraints (for pump adjustment)
#define MIN_TARGET_GAIN_RATE_PER_SEC 0.1f           // Minimum valid gain rate
#define MAX_TARGET_GAIN_RATE_PER_SEC 3.0f           // Maximum valid gain rate
#define TARGET_MOISTURE_GAIN_RATE 0.5f              // Default target (%/sec)

// Temperature Correction
#define TEMP_CORRECTION_FACTOR 0.01f                // ±1% per °C
#define TEMPERATURE_BASELINE 20.0f                  // Reference temperature

// Confidence Parameters
#define HIGH_CONFIDENCE_THRESHOLD 0.70f             // 100% learned above this
#define CONFIDENCE_EMA_WEIGHT 0.20f                 // 20% new, 80% old
#define CONFIDENCE_BOOST_FLOOR 0.25f                // Accelerate convergence floor
```

### Learning Rates (Exponential Moving Averages)
- **PPMP ratio**: Stored directly (no EMA), weighted average at prediction time
- **Pump duty cycle**: Ratio-based adjustment (±15% max per cycle)
- **Soil redistribution factor**: 15% new, 85% old (slow-changing property)
- **Confidence**: 20% new, 80% old (moderately responsive)

### Confidence Thresholds
- **High (≥70%)**: Use 100% learned prediction
- **Below 70%**: Linear interpolation from 0% learned (conf=0) to 100% learned (conf=0.70)
- **Boost floor**: If prediction quality ≥0.50 and confidence <0.25, boost to 0.25

### Anomaly Detection Bounds
- **PPMP Ratio**: 2.0 to 100.0 pulses per percent (hardware-safe clamping bounds)
- **Gain rate**: 0.1 to 3.0 %/sec (for pump duty adjustment calculations)
- **Duration**: 3 to 60 seconds (valid learning cycles)
- **Temperature**: 5°C to 45°C (extreme temps invalidate learning)
- **Soil RF**: 1.0 to 3.0 (sandy to clay range)

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
- **PPMP Ratio**: 3.0 to 30.0 pulses/% (out-of-bounds → fall back to defaults)
- **Gain rate**: 0.1 to 2.0 %/sec
- **Duration**: 5 to 60 seconds
- **Temperature**: -50°C to 100°C sensor range (5-45°C for valid learning)

Out-of-bounds values → use previous valid value or defaults, no learning update

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
    │              │  2. Read final moisture (skip if manual)
    │              │  3. Store immediate moisture reading (for RF)
    │              │  4. Update learning algorithm (skip if manual)
    │              │  5. Update RTC accumulator
    └──────┬───────┘
           │ More zones? → Loop to WATERING
           │ All done? ↓ 6. Restore STELLARIA, close valve, release level shifter
    ┌──────────────┐
    │ MAINTENANCE  │  1. Set 5-min verification timer (RF learning)
    │              │  2. Save learning data (if midnight)
    │              │  3. Run emergency diagnostics (if needed)
    │              │  4. Release power buses
    └──────┬───────┘
           │ Complete
           ↓
    ┌──────────────┐
    │   STANDBY    │  Back to waiting...
    └──────────────┘
           ↑
           │ 5 min later
           │
    ┌──────────────┐
    │ VERIFICATION │  1. Power on sensors (3V3+5V)
    │   (Async)    │  2. Read delayed moisture (all watered zones)
    │              │  3. Update soil redistribution factor (RF)
    │              │  4. Release power buses
    └──────────────┘

    Manual water → WATERING (bypass MEASURING, skip learning update)
    Emergency → DISABLED (user reset required)
```

## Workflow Summary

1. **Initialization**: Load 5 recent PPMP ratios + learned parameters from flash, set defaults if not found
2. **Prediction**: Weighted average (70% recent, 30% older) + PPMP clamping [2.0, 100.0] + temp correction ±1%/°C + 2-tier confidence blending (linear 0→70%)
3. **Execution**: Use learned pump duty, monitor for anomalies, stop at dynamic moisture cutoff (accounts for soil redistribution)
4. **Learning**:
   - Store PPMP ratio directly (weighted averaging at prediction time)
   - Adjust pump duty via ratio-based approach (±15% max per cycle)
   - Update confidence with 20% EMA weight + boost floor at 0.25
5. **Verification** (5 min later): Read delayed moisture, update soil redistribution factor (15% EMA)
6. **Storage**: Save to flash once daily at midnight (wear reduction)

All while maintaining safety (PPMP clamping, pulse bounds [20, 600]), handling sensor failures gracefully, and adapting to hardware changes through responsive confidence tracking (20% EMA) and ratio-based pump adjustment.
