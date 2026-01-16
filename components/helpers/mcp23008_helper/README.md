# MCP23008_HELPER - I2C GPIO Expander Integration

**Author:** Piotr P. <quinkq@gmail.com>
**Part of Solarium** - ESP32-S3 smart irrigation and solar power management system
**Lines of Code:** ~890 (2 files)

## Overview

MCP23008_HELPER provides interrupt-driven integration for the MCP23008 I2C GPIO expander, replacing hardware PCNT functionality with software-based quadrature decoding and pulse counting. It abstracts MCP23008 hardware from consumer components (HMI, TEMPESTA, FLUCTUS) through a clean event-driven API.

**Core Capabilities:**
- Software quadrature encoder decoding with lookup table filtering
- Pulse counting for rainfall and tank intake tipbucket sensors
- Hall array power enable control (TEMPESTA wind direction)
- OLED display reset control with power-saving high-Z mode
- Event-driven button notifications via FreeRTOS task callbacks
- Automatic I2C error recovery and diagnostic reporting

**Hardware:**
- MCP23008 I2C GPIO expander @ 0x20 on Bus A (400kHz)
- INT pin connected to ESP32-S3 GPIO18 (level-triggered)
- 8 GPIO pins: 5 inputs (encoder, button, pulse counters), 2 outputs (hall enable, OLED reset), 1 reserved

## Architecture

### Module Structure

```
mcp23008_helper/
├── mcp23008_helper.c    (670 lines)  - Interrupt handling, quadrature decoding
└── mcp23008_helper.h    (218 lines)  - Public API
```

**Single-module design:** All functionality consolidated in one file (interrupt handler task, decoding logic, public API) with shared state protected by spinlock.

### Interrupt Architecture

```
Hardware Event (encoder rotation, button press, pulse)
          ↓
   MCP23008 pulls INT low
          ↓
ESP32 GPIO ISR (IRAM, level-triggered)
    - Disables interrupt
    - Notifies handler task via xTaskNotify
          ↓
mcp23008_handler_task (Med-5, task context)
    - Reads INTF register (which pins caused interrupt)
    - Reads INTCAP register (captured pin states, clears interrupt)
    - Dispatches to event handlers:
          ├─→ handle_encoder_change() (quadrature decoding)
          ├─→ handle_button_change() (edge detection + task notify)
          ├─→ handle_rainfall_pulse() (counter increment)
          └─→ handle_tank_pulse() (counter increment)
    - Re-enables ESP32 GPIO interrupt
          ↓
Consumer components read counters/states via public API
```

**Latency:** 106-225µs typical from INT trigger to INTCAP read complete.

**Level-Triggered Design:** ESP32 interrupt configured as `GPIO_INTR_LOW_LEVEL` instead of edge-triggered to prevent missed interrupts during encoder rotation bursts. ISR disables interrupt, task re-enables after clearing MCP23008 flags.

### Threading Model

**Task:** `mcp23008_handler` (Med-5, Core 1, 3072 bytes stack)
- Waits on task notification from ISR (blocked, no polling)
- Performs I2C reads (cannot be done in ISR context)
- Dispatches events to handlers
- Implements error recovery and diagnostics

**Thread Safety:**
- Spinlock (`portMUX_TYPE`) protects all shared counters (encoder, rainfall, tank, button)
- Critical sections kept minimal (<10µs) for atomic read/write
- No mutexes (interrupt context incompatible)

**Priority Rationale:**
- Med-5 matches other interrupt handlers (ads1115_retry_task, fluctus_monitor)
- Higher than HMI display task (Med-5 ensures responsive encoder feel)
- Time-sensitive: Encoder quadrature requires quick sampling to avoid missed transitions

### Initialization & Startup

**Default States:**
- Encoder count: 0
- Pulse counters: 0 (monotonic, explicit reset required)
- Button: Released (pull-up high)
- Hall enable: LOW (disabled, power saving)
- OLED reset: INPUT (high-Z, power saving)

**Initialization Order:**
- Must initialize **after** I2C bus setup (Bus A always powered)
- Must initialize **before** components that register button callbacks (HMI)
- Handler task created before ISR installed (prevents NULL task handle)

**Deployment Safety:**
- OLED reset starts in high-Z to prevent parasitic power draw
- Hall enable starts LOW to prevent unnecessary 14mA current
- Interrupts enabled only after task is ready to handle them

### Task Communication

**ISR → Handler Task:**
- `xTaskNotifyFromISR()` on INT pin transition
- Notification wakes handler from blocked state (no polling overhead)

**Handler Task → HMI:**
- `xTaskNotify()` on button press falling edge
- Registered via `mcp23008_helper_register_button_callback()`
- Event-driven wake (no polling required)

**Synchronization:**
- Spinlock for all counter/state access (safe from ISR and task contexts)
- No dependencies between handler logic paths (encoder/button/pulses independent)

## Hardware Configuration

### MCP23008 Pin Mapping

| Pin | Function | Direction | Pull-up | Interrupt | Logic |
|-----|----------|-----------|---------|-----------|-------|
| GP0 | Encoder A | Input | External 10kΩ | Any edge | Gray code |
| GP1 | Encoder B | Input | External 10kΩ | Any edge | Gray code |
| GP2 | Encoder Button | Input | Internal | Any edge | Active-low |
| GP3 | Rainfall Counter | Input | Internal | Any edge | Active-low |
| GP4 | Tank Intake Counter | Input | Internal | **Disabled** | Active-low (noise) |
| GP5 | Hall Array Enable | Output | - | - | HIGH=ON |
| GP6 | OLED Reset | Input/Output | - | - | Dynamic (high-Z when idle) |
| GP7 | Reserved | Input | Internal | - | - |

**Note:** GP4 (tank intake) interrupt temporarily disabled due to noise/floating signal causing interrupt storms.

### I2C Configuration

- **Address:** 0x20 (A2=A1=A0=0)
- **Bus:** I2C Bus A (shared with sensors)
- **Speed:** 400kHz
- **Interrupt Mode:** Open-drain active-low
- **INT Pin:** ESP32-S3 GPIO18 (level-triggered, internal pull-up)

### ESP32-S3 GPIO

| GPIO | Function | Mode | Interrupt Type |
|------|----------|------|----------------|
| GPIO18 | MCP23008 INT | Input, pull-up | `GPIO_INTR_LOW_LEVEL` |

## Key Features

### 1. Software Quadrature Decoding

Replaces ESP32 hardware PCNT with software table-based decoding due to GPIO shortage on ESP32-S3.

**Lookup Table Algorithm:**

```c
// Index: [last_AB_state << 2 | current_AB_state]
// Valid transitions change only 1 bit (Gray code)
static const int8_t quadrature_table[16] = {
    0,  -1,  1,  0,   // 00->00, 00->01, 00->10, 00->11
    1,   0,  0, -1,   // 01->00, 01->01, 01->10, 01->11
   -1,   0,  0,  1,   // 10->00, 10->01, 10->10, 10->11
    0,   1, -1,  0    // 11->00, 11->01, 11->10, 11->11
};
```

**Multi-Stage Filtering:**
1. **Duplicate rejection:** Ignore same-state interrupts (noise)
2. **Invalid transition filtering:** Lookup table returns 0 for impossible state jumps (both bits changed)
3. **Detent accumulation:** ±4 steps = 1 user count (standard encoder with 4 steps per detent)

**Example Rotation (clockwise):**
```
AB: 00→01 (+1) → 11 (+1) → 10 (+1) → 00 (+1) = +4 steps → encoder_count++
```

**Thread Safety:**
- `encoder_count` protected by spinlock (public API reads)
- `detent_accumulator` and `last_ab` only accessed from handler task (no protection needed)

### 2. Monotonic Pulse Counting

Simple increment-on-interrupt for rainfall and tank tipbucket sensors. Counters are **monotonically increasing** - consumers must explicitly reset them to prevent ambiguity between hardware resets and intentional clears.

**Rainfall Sensor:**
- 100mL/pulse tipbucket
- TEMPESTA reads hourly, accumulates to base-tracking total
- Weekly reset via `mcp23008_helper_reset_rainfall_pulses()` prevents overflow

**Tank Intake Sensor:**
- 50mL/pulse tipbucket
- TEMPESTA tracks tank volume refills
- Weekly reset via `mcp23008_helper_reset_tank_intake_pulses()` prevents overflow

**Overflow Protection:**
- `uint32_t` counters support 4.29 billion pulses
- At 100mL/pulse: 429,496,729 liters before overflow
- Weekly resets provide safety margin (typical weekly rainfall <1000 pulses)

### 3. Event-Driven Button Handling

Button press detection with edge filtering and task notification.

**Detection Logic:**
```c
bool is_pressed = !(gpio_state & (1 << MCP_PIN_ENCODER_BTN));  // Active-low
if (is_pressed && !was_pressed && button_notify_task != NULL) {
    xTaskNotify(button_notify_task, 1, eSetBits);  // Falling edge
}
```

**Benefits:**
- HMI task wakes only on button press (no polling overhead)
- Edge detection prevents repeated notifications during hold
- Registration API allows dynamic callback management

**Usage Pattern:**
```c
// HMI task during init
mcp23008_helper_register_button_callback(xTaskGetCurrentTaskHandle());

// HMI task loop
while (1) {
    xTaskNotifyWait(0, 0xFFFFFFFF, &notification, timeout);
    // Handle button press
}
```

### 4. Power-Aware Output Control

**Hall Array Enable (GP5):**
- Controls N-MOSFET gate for ADS1115-3 wind direction sensor
- Saves 14mA idle current
- 10ms enable duration per TEMPESTA read cycle
- Called by FLUCTUS via `fluctus_hall_array_enable(bool)`

**OLED Reset (GP6):**
- Dynamic pin mode switching for power optimization
- **High-Z mode (INPUT):** Default state when display off (saves 3-4mA parasitic current)
- **OUTPUT mode:** Driven HIGH when display active, LOW during reset pulse
- **Reset pulse sequence:** INPUT → OUTPUT LOW (10ms) → OUTPUT HIGH (10ms) → INPUT (high-Z)

**Power Saving Benefit:**
- OLED reset high-Z: ~3-4mA savings when display off
- Hall array gating: ~14mA savings except during 10ms reads

## Public API

### Initialization

```c
esp_err_t mcp23008_helper_init(void);
```
Initializes MCP23008 device, configures pins, installs interrupt handler, and starts handler task.

### Encoder Functions

```c
int32_t mcp23008_helper_get_encoder_count(void);
void mcp23008_helper_reset_encoder_count(void);
```
Thread-safe access to encoder count. Returns positive for clockwise, negative for counter-clockwise rotation.

### Pulse Counter Functions

```c
uint32_t mcp23008_helper_get_rainfall_pulses(void);
uint32_t mcp23008_helper_get_tank_intake_pulses(void);
void mcp23008_helper_reset_rainfall_pulses(void);
void mcp23008_helper_reset_tank_intake_pulses(void);
```
Monotonic counters with explicit reset functions. Used by TEMPESTA for hourly/weekly accumulation.

### Button Functions

```c
bool mcp23008_helper_get_button_state(void);
esp_err_t mcp23008_helper_register_button_callback(TaskHandle_t task_handle);
```
Button state query and event-driven callback registration. Pass NULL to unregister.

### Output Control

```c
esp_err_t mcp23008_helper_set_hall_enable(bool enable);
esp_err_t mcp23008_helper_set_oled_reset(bool high);
esp_err_t mcp23008_helper_oled_reset_pulse(void);
esp_err_t mcp23008_helper_oled_reset_release(void);
```
Power-gated outputs. `oled_reset_pulse()` is self-contained (OUTPUT LOW → HIGH → INPUT high-Z).

## Design Patterns

**Interrupt Deferral with Task Notifications**: The ESP32 GPIO ISR cannot perform I2C operations (clock stretching unsafe in interrupt context), so it immediately notifies a handler task via `xTaskNotifyFromISR()`. This splits latency-critical work (ISR wakeup ~5µs) from blocking I2C reads (~100-200µs) while maintaining responsive handling. Level-triggered interrupt with task-based re-enabling prevents missed events during rotation bursts.

**Monotonic Counters with Explicit Reset**: Pulse counters increment indefinitely until explicitly reset by consumers. This design avoids ambiguity between accidental hardware resets and intentional clears - TEMPESTA can detect unexpected resets by comparing counter deltas. Weekly resets prevent uint32_t overflow while maintaining multi-week uptime capability.

**Dynamic Pin Mode for Power Optimization**: OLED reset pin switches between INPUT (high-Z, power saving) and OUTPUT (active drive) based on display state. This eliminates 3-4mA parasitic current draw when display is off, critical for solar-powered operation. Self-contained reset pulse function handles mode transitions automatically.

**Lookup Table Quadrature Decoding**: Instead of state machine logic, a 16-entry table maps [previous_state, current_state] to delta (-1, 0, +1). Invalid transitions (noise causing both bits to change) produce zero delta, filtering electrical noise without complex conditionals. This approach handles all edge cases (including reverse rotation mid-detent) with zero branching.

**Event-Driven Button Callbacks**: Task notification registration allows HMI to wake only on button press instead of polling. Handler task detects falling edge and calls `xTaskNotify()` directly, eliminating the latency and power waste of periodic polling. Registration API supports dynamic callback changes without recompilation.

## Integration

**HMI (Display & Encoder):**
- Registers button callback during init for event-driven wake
- Polls encoder count at 1-4Hz for menu navigation
- Calls `reset_encoder_count()` after processing rotation
- Uses OLED reset pulse during display initialization

**TEMPESTA (Weather Sensors):**
- Reads rainfall pulses hourly for accumulation
- Reads tank pulses for volume tracking
- Resets counters weekly to prevent overflow
- Receives hall enable from FLUCTUS wrapper

**FLUCTUS (Power Management):**
- Calls `set_hall_enable()` before wind direction reads (10ms enable)
- Controls hall array power gating (saves 14mA)

## Performance

**Memory:**
- Static state: ~80 bytes (counters, flags, diagnostics)
- MCP23008 device descriptor: 24 bytes
- Task stack: 3072 bytes
- Total SRAM: ~3.2 KB

**Timing:**
- ISR latency: ~5µs (notification only)
- INTF read: ~50-100µs (I2C @ 400kHz)
- INTCAP read: ~50-100µs (I2C @ 400kHz)
- Total interrupt handling: 106-225µs (ISR trigger → INTCAP clear)
- Encoder read (spinlock): <1µs

**I2C Traffic:**
- Idle: 0 bytes/sec (interrupt-driven, no polling)
- Encoder rotation: ~2 reads × 1 byte = 2 bytes per detent
- Button press: 2 reads × 1 byte = 2 bytes per press
- Pulse event: 2 reads × 1 byte = 2 bytes per pulse
- Output control: 1-2 writes × 1 byte = 1-2 bytes per call

**Power Consumption:**
- MCP23008 active: ~1mA @ 3.3V
- Interrupt handling: Negligible (task wakes only on events)
- OLED reset high-Z savings: ~3-4mA
- Hall enable gating savings: ~14mA

## Configuration

Key defines in `mcp23008_helper.h`:

```c
// Hardware addresses
#define MCP23008_I2C_ADDR       0x20        // I2C address
#define MCP23008_INT_GPIO       GPIO_NUM_18 // ESP32 interrupt pin

// Task configuration
#define MCP23008_HANDLER_TASK_STACK_SIZE    3072
#define MCP23008_HANDLER_TASK_PRIORITY      5    // Med-5
#define MCP23008_HANDLER_TASK_CORE          1    // Core 1
```

## Known Limitations

- **GP4 (tank intake) interrupt disabled:** Floating signal causes interrupt storms. Hardware pull-down or external sensor required to re-enable.
- **No multi-client encoder support:** Only one consumer can read encoder delta (HMI). Multiple readers would need delta distribution logic.
- **Level-triggered interrupt overhead:** ESP32 interrupt fires continuously while INT is low. Task must read MCP23008 quickly to clear interrupt and prevent ISR spam.
- **No hardware filtering:** Encoder detent accumulation (±4 steps) assumes standard encoder. Custom detent counts require source code modification.
- **I2C bus contention:** Shares Bus A with sensors. Heavy I2C traffic from other components may increase interrupt handling latency.
- **No persistent counters:** Encoder and pulse counts reset to zero on system reboot. Persistent counting requires external storage (RTC RAM, NVS).

---

**Related Files:**
- Integration: `main/main.c`
- HMI: `components/core/hmi/hmi.c` (encoder reading, button callback)
- TEMPESTA: `components/core/tempesta/tempesta_sensors.c` (pulse counter reads)
- FLUCTUS: `components/core/fluctus/fluctus.c` (hall array enable wrapper)
