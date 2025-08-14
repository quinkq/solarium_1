# Irrigation Learning Algorithm & Workflow Documentation

## Overview

The **IMPLUVIUM** irrigation system implements a sophisticated machine learning algorithm that adapts watering behavior based on historical performance data. The system learns optimal water quantities, pump speeds, and timing for each irrigation zone individually, while integrating environmental data from the **TEMPESTA** weather station.

## System Architecture

### Components Integration
- **IMPLUVIUM** (Irrigation): Core learning algorithm and watering control
- **TEMPESTA** (Weather Station): Environmental data provider (temperature, humidity, pressure)
- **FLUCTUS** (Power Management): Power state awareness for learning decisions

### Learning Data Structures

```c
typedef struct {
    float pulses_used_history[15];               // Last 15 watering cycles (flow pulses used)
    float moisture_increase_percent_history[15]; // Corresponding moisture gains (%)
    bool anomaly_flags[15];                      // Anomalous cycle markers
    uint8_t history_entry_count;                 // Valid entries (0-15)
    uint8_t history_index;                       // Current write position (circular buffer)
    
    // Current learned parameters
    float calculated_ppmp_ratio;                 // Most recent learned pulses per moisture percent ratio
    uint32_t calculated_pump_duty_cycle;         // Optimal pump PWM duty cycle
    float target_moisture_gain_rate;             // Optimal moisture gain rate (%/sec)
    
    // Confidence tracking
    float confidence_level;                      // Prediction accuracy (0.0-1.0)
    uint32_t successful_predictions;             // Count of accurate predictions
    uint32_t total_predictions;                  // Total valid prediction attempts
    
    // Temperature integration
    float last_temperature_correction;          // Last calculated temp factor
} zone_learning_t;
```

## Complete Learning Workflow

### Phase 1: System Initialization

**Location**: `irrigation_system_init()` - Line 111

1. **Initialize Default Values**:
   ```c
   irrigation_zones[i].learning.current_pulses_per_percent = 8.0f;        // Default: 8 pulses per 1% moisture
   irrigation_zones[i].learning.learned_pump_duty_cycle = PUMP_DEFAULT_DUTY; // Default pump speed
   irrigation_zones[i].learning.target_moisture_gain_rate = 0.5f;         // Target 0.5%/sec gain rate
   ```

2. **Initialize Circular Buffers**: All history arrays are zero-initialized (static variables)
3. **Set Learning Thresholds**:
   - `LEARNING_MIN_CYCLES = 3`: Minimum cycles before predictions start
   - `LEARNING_HISTORY_SIZE = 15`: Maximum stored learning cycles
   - `LEARNING_WEIGHT_RECENT = 0.7f`: Weight for recent cycles vs. old ones

### Phase 2: Pre-Watering Prediction (MEASURING State)

**Location**: `irrigation_state_measuring()` → `irrigation_calc_zone_watering_predictions()` - Line 1185

#### Step 2.1: Zone Selection and Prioritization
1. **Moisture Assessment**: Read each zone's current moisture level
2. **Deficit Calculation**: `moisture_deficit = target_moisture - current_moisture`
3. **Queue Building**: Add zones needing water to priority queue
4. **Queue Sorting**: Sort by moisture deficit (highest priority first)

#### Step 2.2: Prediction Calculation for Each Zone

**Location**: `irrigation_calc_zone_target_pulses()` - Line 814

1. **Temperature Correction Calculation**:
   ```c
   // Get current temperature from TEMPESTA weather station
   float current_temperature = weather_get_temperature();
   
   // Calculate correction factor: 1% adjustment per degree from 20°C baseline
   float temp_correction = 1.0f + ((current_temperature - 20.0f) * 0.01f);
   ```
   - **Hot weather** (>20°C): Increases water needs (correction > 1.0)
   - **Cool weather** (<20°C): Decreases water needs (correction < 1.0)

2. **Learning Phase Decision**:
   ```c
   if (learning->history_entry_count >= 3) {
       // LEARNED MODE: Use historical data
   } else {
       // INITIAL MODE: Use defaults
       target_pulses = 80; // DEFAULT_TARGET_PULSES
   }
   ```

3. **Learned Mode Prediction** (≥3 historical cycles):

   **Weighted Historical Analysis**:
   ```c
   // Calculate weighted average of pulses-per-percent from history
   for (each valid historical cycle) {
       // Determine recency with proper circular buffer logic
       relative_age = (history_index - h - 1 + 15) % 15;
       weight = (relative_age < 3) ? 0.7f : 0.3f;  // Recent cycles get higher weight
       
       if (!anomaly_flags[h] && moisture_increase > 0) {
           ratio = pulses_used / moisture_increase_percent;
           weighted_sum += ratio * weight;
           total_weight += weight;
       }
   }
   learned_ratio = weighted_sum / total_weight;
   ```

   **Final Prediction Calculation**:
   ```c
   target_pulses = moisture_deficit * learned_ratio * temp_correction;
   
   // Safety bounds
   target_pulses = clamp(target_pulses, 20, 300); // MINIMUM_TARGET_PULSES to MAXIMUM_TARGET_PULSES
   ```

### Phase 3: Watering Execution (WATERING State)

**Location**: `irrigation_state_watering()` - Line 1191

1. **Power Management**: Request 12V bus for valves and pumps
2. **Valve Control**: Open zone valve
3. **Pump Ramp-Up**: Gradual speed increase using learned pump duty cycle
4. **Flow Monitoring**: Track actual water usage via flow sensor
5. **Real-Time Control**: 
   - **Adaptive Pump Control**: Adjust pump speed based on actual vs. target moisture gain rate
   - **Safety Monitoring**: Emergency cutoffs, anomaly detection
   - **Progress Tracking**: Monitor pulse count vs. predicted target

#### Real-Time Adaptive Control

**Location**: `irrigation_pump_adaptive_control()` - Line 678 (called from monitoring task)

```c
// Compare actual vs. target moisture gain rate
rate_error = target_gain_rate - current_gain_rate;

if (|rate_error| > tolerance) {
    if (rate_error > 0) {
        pump_duty += ADJUSTMENT_STEP;    // Need more water flow
    } else {
        pump_duty -= ADJUSTMENT_STEP/2;  // Too much flow, reduce gradually
    }
}
```

### Phase 4: Post-Watering Learning Update (STOPPING State)

**Location**: `irrigation_state_stopping()` → `irrigation_log_zone_watering_data()` - Line 1323

#### Step 4.1: Data Collection
1. **Final Measurements**:
   ```c
   total_pulses = flow_sensor_final - flow_sensor_start;
   volume_ml = (total_pulses / PULSES_PER_LITER) * 1000;
   final_moisture = read_moisture_sensor(zone);
   moisture_increase = final_moisture - initial_moisture;
   watering_duration = end_time - start_time;
   ```

#### Step 4.2: Anomaly Detection
**Location**: Line 1306-1315

Cycles are marked as anomalous if:
- **Rain Detection**: Excessive moisture spike during watering
- **Temperature Extremes**: < 5°C or > 45°C
- **Sensor Failures**: Invalid temperature readings
- **System Anomalies**: Pressure/flow anomalies detected during cycle

#### Step 4.3: Learning Data Storage
**Location**: `irrigation_log_zone_watering_data()` - Line 923

1. **Circular Buffer Update**:
   ```c
   index = learning->history_index;
   learning->pulse_amount_history[index] = pulses_used;
   learning->moisture_increase_percent_history[index] = moisture_increase_percent;
   learning->anomaly_flags[index] = !learning_valid;  // Store for future calculations
   
   // Advance circular buffer
   learning->history_index = (learning->history_index + 1) % 15;
   learning->history_entry_count = min(learning->history_entry_count + 1, 15);
   ```

#### Step 4.4: Real-Time Learning Updates

1. **Efficiency Learning** (Line 964-971):
   ```c
   // Calculate actual efficiency for this cycle
   efficiency = pulses_used / moisture_increase_percent;  // pulses per 1% moisture
   
   // Update learned efficiency with exponential moving average (10% new, 90% old)
   if (valid && efficiency is reasonable) {
       learning->current_pulses_per_percent = 
           (learning->current_pulses_per_percent * 0.9f) + (efficiency * 0.1f);
   }
   ```

2. **Target Gain Rate Learning** (Line 1325-1336):
   ```c
   // Learn optimal moisture gain rate for this zone's soil characteristics
   actual_gain_rate = moisture_increase_percent / (watering_duration_seconds);
   
   if (valid && gain_rate is reasonable) {
       // Slow adaptation (5% new, 95% old) for stability
       learning->target_moisture_gain_rate = 
           (learning->target_moisture_gain_rate * 0.95f) + (actual_gain_rate * 0.05f);
   }
   ```

#### Step 4.5: Confidence Tracking
**Location**: Line 974-990

```c
if (learning_valid) {
    learning->total_predictions++;
    
    // Check prediction accuracy (within 20% tolerance)
    expected_increase = pulses_used / learning->current_pulses_per_percent;
    accuracy = |moisture_increase - expected_increase| / expected_increase;
    
    if (accuracy <= 0.20f) {
        learning->successful_predictions++;
    }
    
    // Update confidence level
    learning->confidence_level = successful_predictions / total_predictions;
}
```

#### Step 4.6: Pump Duty Cycle Learning
**Location**: Line 995-1006

```c
// Only after sufficient learning cycles and for valid predictions
if (history_entry_count >= 3 && learning_valid) {
    // Update optimal pump speed with exponential moving average
    learning->learned_pump_duty_cycle = 
        (learned_pump_duty_cycle * 0.9f) + (current_pump_duty * 0.1f);
}
```

## Environmental Integration (TEMPESTA Weather Station)

### Temperature Influence on Watering

**Temperature Correction Formula**:
```c
temp_correction = 1.0 + ((current_temp - 20.0°C) * 0.01);
```

**Examples**:
- **25°C** → correction = 1.05 (5% more water needed)
- **15°C** → correction = 0.95 (5% less water needed)
- **30°C** → correction = 1.10 (10% more water needed)

### Weather Data Integration Points

1. **Pre-Check**: Global temperature safety limits (0°C to 50°C)
2. **Moisture Check Intervals**: Dynamic timing based on temperature
3. **Learning Predictions**: Temperature correction applied to all calculations
4. **Anomaly Detection**: Extreme temperatures invalidate learning cycles

## Learning Algorithm Parameters

### Constants and Thresholds
```c
#define LEARNING_HISTORY_SIZE 15           // Keep last 15 cycles
#define LEARNING_MIN_CYCLES 3              // Start predictions after 3 cycles  
#define LEARNING_WEIGHT_RECENT 0.7f        // Weight for recent cycles (vs 0.3f for old)
#define DEFAULT_TARGET_PULSES 80           // Default starting target (≈200mL)
#define MINIMUM_TARGET_PULSES 20           // Safety minimum
#define MAXIMUM_TARGET_PULSES 300          // Safety maximum
#define TARGET_MOISTURE_GAIN_RATE 0.5f     // Default target rate (%/sec)
#define TEMP_CORRECTION_FACTOR 0.01f       // 1% per degree C
#define TEMPERATURE_BASELINE 20.0f         // Reference temperature
```

### Learning Rates (Exponential Moving Averages)
- **Real-time efficiency**: 10% new, 90% old (fast adaptation)
- **Target gain rate**: 5% new, 95% old (slow, stable adaptation)
- **Pump duty cycle**: 10% new, 90% old (moderate adaptation)

## Multi-Zone Learning Characteristics

### Per-Zone Adaptation
Each zone learns independently:
- **Clay Soil Zones**: Lower gain rates (0.2-0.3 %/sec), higher pulse requirements
- **Sandy Soil Zones**: Higher gain rates (0.6-0.8 %/sec), lower pulse requirements  
- **Mixed Soil Zones**: Moderate values based on actual performance

### Zone-Specific Parameters Learned
1. **Pulses per Percent**: Water efficiency for soil type
2. **Pump Duty Cycle**: Optimal pump speed for zone's hydraulic characteristics
3. **Target Gain Rate**: Optimal watering speed for soil absorption characteristics
4. **Confidence Level**: Prediction accuracy history for each zone

## Error Handling and Recovery

### Learning Data Validation
- **Efficiency bounds**: 0.5 to 50.0 pulses per percent (reasonable range)
- **Gain rate bounds**: 0.1 to 2.0 %/sec (prevents extreme values)
- **Duration bounds**: 5 to 60 seconds (filters out incomplete cycles)

### Anomaly Recovery
- **Isolated anomalies**: Stored but ignored in calculations
- **Multiple anomalies**: System falls back to default predictions
- **Sensor failures**: Learning paused until sensors recover

### Confidence-Based Fallbacks
- **Low confidence** (<50%): Blend learned and default predictions
- **Very low confidence** (<20%): Revert to default values
- **No historical data**: Use system defaults until learning accumulates

## Performance Metrics and Monitoring

### Key Performance Indicators
1. **Prediction Accuracy**: Percentage of predictions within 20% tolerance
2. **Water Efficiency**: mL per 1% moisture gain improvement over time
3. **Adaptation Speed**: Time to reach stable predictions per zone
4. **Anomaly Rate**: Percentage of cycles flagged as anomalous

### Logging and Debugging
- **Real-time updates**: Efficiency and gain rate learning logged at INFO level
- **Prediction details**: Weighted calculations logged at DEBUG level  
- **Confidence tracking**: Success/failure ratios logged per zone
- **Temperature integration**: Correction factors logged at DEBUG level

## System Integration Notes

### Power Management Integration (FLUCTUS)
- **Critical power states**: Learning disabled during battery conservation
- **Power-aware scheduling**: Moisture check intervals extended during power saving

### Safety System Integration
- **Emergency stops**: Learning cycle marked as anomaly, no data stored
- **Flow/pressure anomalies**: Automatic learning invalidation
- **Sensor failures**: Learning paused, confidence tracking suspended

---

## Workflow Summary Diagram

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   INITIALIZATION│    │     MEASURING    │    │     WATERING    │
│                 │    │                  │    │                 │
│ • Set defaults  │───>│ • Read moisture  │───>│ • Use learned   │
│ • Clear buffers │    │ • Calculate      │    │   pump duty     │
│ • Learning=3    │    │   predictions    │    │ • Adaptive      │
│   cycles min    │    │ • Apply temp     │    │   control       │
└─────────────────┘    │   correction     │    │ • Monitor flow  │
                       └──────────────────┘    └─────────────────┘
                                                       │
┌─────────────────┐    ┌──────────────────┐            │
│    LEARNING     │    │     STOPPING     │            │
│                 │    │                  │<───────────┘
│ • Update        │<───│ • Measure final  │
│   efficiency    │    │   moisture       │
│ • Learn gain    │    │ • Detect         │
│   rate          │    │   anomalies      │
│ • Update        │    │ • Calculate      │
│   confidence    │    │   results        │
└─────────────────┘    └──────────────────┘
```

This learning algorithm enables the irrigation system to continuously improve its water delivery precision, adapting to seasonal changes, soil conditions, and plant growth patterns while maintaining safety and efficiency.