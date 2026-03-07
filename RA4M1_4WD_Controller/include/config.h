// =============================================================================
//  config.h — RA4M1 4WD Hoverboard Controller Configuration
//
//  All tuneable parameters in one place.
//  Adjust pin assignments and mixing parameters to match your build.
// =============================================================================
#ifndef CONFIG_H
#define CONFIG_H

// ========================== RC RECEIVER PINS (HOTRC DS650) ==================
// Connect the PWM signal wires from the RC receiver to these digital pins.
// IMPORTANT: The RA4M1 GPIO pins are 3.3 V only!
//   - If your receiver outputs 5 V signals, use a voltage divider or
//     level-shifter, OR power the receiver from 3.3 V (many work fine).
//
// NOTE: NOT all digital pins on the UNO R4 Minima support external interrupts!
//   Only these pins have IRQ capability:
//     D0(IRQ6), D1(IRQ5), D2(IRQ0), D3(IRQ1), D8(IRQ9),
//     D12(IRQ3), D13(IRQ4), A1(IRQ6), A2(IRQ7), A3(IRQ2), A4(IRQ1), A5(IRQ2)
//   Pins D4, D5, D6, D7, D9, D10, D11, A0 have NO interrupt support!
//   Pins sharing the same IRQ channel cannot be used simultaneously.

#define PIN_RC_CH1          2       // Steering  (left / right)    → D2  (IRQ0)
#define PIN_RC_CH2          3       // Throttle  (forward / back)  → D3  (IRQ1)
#define PIN_RC_CH3          8       // AUX / VR knob (speed limit) → D8  (IRQ9)
#define PIN_RC_CH4          A2      // AUX / switch  (mode select) → A2  (IRQ7)

// Number of RC channels to read (2 = steering+throttle, up to 4)
#define RC_NUM_CHANNELS     4

// ========================== RC PULSE CALIBRATION ============================
// Standard RC PWM: 1000–2000 µs, centre 1500 µs
// Adjust these if your transmitter end-points differ.
#define RC_PULSE_MIN        1000    // [µs] minimum pulse width
#define RC_PULSE_MID        1500    // [µs] centre / neutral pulse width
#define RC_PULSE_MAX        2000    // [µs] maximum pulse width
#define RC_PULSE_DEADBAND   40      // [µs] dead-band around centre (each side)

// Failsafe: if no valid pulse is received within this time → stop motors
#define RC_FAILSAFE_TIMEOUT 500     // [ms]

// ========================== HOVERBOARD UART =================================
// Protocol: START(0xABCD) + steer(int16) + speed(int16) + checksum(uint16) = 8 bytes
// Baud rate must match the hoverboard firmware (config.h → USART3 / USART2)
#define HOVER_BAUD          115200  // [bps] Baud rate for hoverboard communication
#define START_FRAME         0xABCD  // Start frame marker
#define UART_SEND_INTERVAL  20      // [ms] Command send interval (50 Hz)

// ---- Board 1 (FRONT axle) ----
// Uses hardware Serial1 on pins D0 (RX) and D1 (TX)
// Connect:  RA4M1 D1 (TX) → Hoverboard USART3 RX (right sensor cable, green wire)
//           RA4M1 D0 (RX) ← Hoverboard USART3 TX (right sensor cable, yellow wire)

// ---- Board 2 (REAR axle) ----
// Uses a second UART instance on D11 (TX) and D12 (RX).
// On the RA4M1 these map to P411/P410 (SCI0).
// Connect:  RA4M1 D11 (TX) → Hoverboard USART3 RX
//           RA4M1 D12 (RX) ← Hoverboard USART3 TX
#define PIN_BOARD2_TX       11
#define PIN_BOARD2_RX       12

// ========================== 4WD MIXING ======================================
//
// Each hoverboard receives <steer, speed>.  The firmware internally mixes:
//   cmdL = speed * speedCoeff − steer * steerCoeff
//   cmdR = speed * speedCoeff + steer * steerCoeff
//
// Steering ratio per axle (percent, 0–100):
//   100 = full steering,  0 = no steering (straight only)
//   Typical car-like: front=100, rear=0
//   4WS (four-wheel-steer): front=100, rear=50 (counter-phase at low speed)
#define FRONT_STEER_RATIO   100     // [%] Steering ratio applied to front board
#define REAR_STEER_RATIO    100     // [%] Steering ratio applied to rear board

// Invert steering direction per axle (true / false)
// Use this if the motor wiring direction is flipped on one board.
#define FRONT_STEER_INVERT  false
#define REAR_STEER_INVERT   false

// Invert speed (drive) direction per axle
#define FRONT_SPEED_INVERT  false
#define REAR_SPEED_INVERT   false

// Command output range sent to the hoverboard (must match hoverboard config.h INPUT range)
#define CMD_MIN             -1000
#define CMD_MAX              1000

// Input dead-band AFTER mapping to command range
#define CMD_DEADBAND         30     // values below ±30 are treated as 0

// Exponential curve for smoother control near centre (0 = linear, 100 = strong expo)
#define EXPO_STEERING        30     // [%]
#define EXPO_THROTTLE        20     // [%]

// Speed limit via CH3 button: each press cycles to the next step.
// Define the steps (in %) that CH3 cycles through.
#define SPEED_LIMIT_ENABLE   true   // set false to disable CH3 speed limiting
#define SPEED_LIMIT_NUM_STEPS  4    // number of speed-limit steps
// Speed-limit steps in percent (array values defined in main.cpp)
// Default: 25%, 50%, 75%, 100%
#define SPEED_LIMIT_DEFAULT_STEP 3   // index of initial step (0-based) → 100%

// ========================== DRIVE MODES (via CH4 button) ====================
// Each button press on CH4 cycles: NORMAL → SPORT → CRAWL → NORMAL → ...
#define MODE_NORMAL          0      // Standard 4WD
#define MODE_SPORT           1      // Increased responsiveness / higher limits
#define MODE_CRAWL           2      // Low speed, fine control
#define MODE_NUM_MODES       3      // total number of drive modes
#define MODE_DEFAULT         MODE_NORMAL // initial mode after power-on

// Speed limit per drive mode (percent of CMD_MAX)
#define SPORT_SPEED_LIMIT    100
#define NORMAL_SPEED_LIMIT   70
#define CRAWL_SPEED_LIMIT    35

// Button edge detection: pulse must cross this threshold (µs) to count as "pressed"
#define BUTTON_PULSE_THRESH  1700   // > 1700 µs = pressed, < 1300 µs = released
#define BUTTON_RELEASE_THRESH 1300
#define BUTTON_DEBOUNCE_MS   200    // [ms] ignore repeated edges within this time

// ========================== FEEDBACK ========================================
// Enable feedback reading from hoverboards (battery voltage, temperature, speed)
#define FEEDBACK_ENABLE      true
#define FEEDBACK_BOARD1      true   // read feedback from front board
#define FEEDBACK_BOARD2      true   // read feedback from rear board

// ========================== DEBUG ===========================================
#define DEBUG_ENABLE         true   // Send debug info over USB Serial
#define DEBUG_BAUD           115200 // USB Serial baud rate
#define DEBUG_INTERVAL       200    // [ms] Debug print interval

// ========================== SAFETY ==========================================
// Ramp rate: maximum command change per send interval (limits acceleration)
#define RAMP_RATE            30     // Max change per cycle (per 20 ms) ≈ 1500/s
#define RAMP_RATE_BRAKE      60     // Faster ramp when decelerating / braking

// LED indicator (pin 13 = built-in LED on UNO R4 Minima)
// NOTE: Do NOT use LED_BUILTIN here — it expands to PIN_LED, causing a circular macro.
#define PIN_LED              13

#endif // CONFIG_H
