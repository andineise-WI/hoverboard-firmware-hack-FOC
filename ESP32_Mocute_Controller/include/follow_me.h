/**
 * @file follow_me.h
 * @brief Follow Me mode using Bluetooth RSSI signal strength
 *
 * Uses RSSI (Received Signal Strength Indicator) to estimate distance
 * to the Bluetooth gamepad controller and derive speed commands.
 *
 * How it works:
 *   - RSSI is read periodically from the BT connection
 *   - The RSSI value is smoothed with an exponential moving average filter
 *   - RSSI is mapped to distance zones (CLOSE, FOLLOW, FAR, LOST)
 *   - Each zone results in a different speed behavior:
 *       CLOSE: stop or reverse slowly (too close to user)
 *       FOLLOW: drive forward proportionally to distance
 *       FAR: drive forward at maximum follow speed
 *       LOST: emergency stop (signal too weak)
 *
 * Direction (Dual-ESP32 Mode):
 *   - With two ESP32s (Master + Satellite) on opposite sideboards,
 *     direction detection is possible via RSSI difference.
 *   - Direction steering is computed by DirectionDetector (direction_detect.h)
 *     and fed into FollowMe via setDirectionSteer().
 *   - When direction data is available, the hoverboard auto-steers
 *     toward the user without manual joystick input.
 *
 * Limitations:
 *   - RSSI is noisy and affected by environment, body orientation, obstacles
 *   - Single ESP32: Only distance estimation, NO direction (manual steering)
 *   - Dual ESP32: Approximate direction via RSSI difference
 *   - Requires careful calibration for each environment
 *   - Works best in open areas with line-of-sight
 */

#ifndef FOLLOW_ME_H
#define FOLLOW_ME_H

#include <Arduino.h>

// ========================== RSSI Calibration ==========================
// RSSI values are typically negative dBm but BTstack/Bluepad32 reports
// them as uint8_t (0-255), where 255 = best signal.
// In practice: ~200-255 = very close, ~150-200 = medium, <150 = far
//
// IMPORTANT: These values MUST be calibrated for your specific setup!
// Use the serial monitor to observe RSSI values at different distances
// and adjust these thresholds accordingly.

// RSSI threshold zones (BTstack format: higher = closer, 0-255)
#define RSSI_ZONE_CLOSE     200   // Very close to user - stop/reverse
#define RSSI_ZONE_IDEAL     180   // Ideal following distance - hold speed
#define RSSI_ZONE_FOLLOW    155   // Following range - drive forward
#define RSSI_ZONE_FAR       130   // Getting too far - drive faster
#define RSSI_ZONE_LOST      100   // Signal too weak - emergency stop

// Speed settings for Follow Me mode
#define FOLLOW_SPEED_MAX     250  // Maximum speed in follow mode (conservative!)
#define FOLLOW_SPEED_MIN      50  // Minimum speed when barely in range
#define FOLLOW_REVERSE_SPEED -80  // Slow reverse when too close
#define FOLLOW_HOLD_SPEED      0  // Speed when at ideal distance

// RSSI smoothing filter (exponential moving average)
// Alpha = 0.0 to 1.0, lower = smoother but slower response
#define RSSI_FILTER_ALPHA    0.15f  // Filter coefficient

// RSSI read interval
#define RSSI_READ_INTERVAL_MS  200  // How often to read RSSI [ms]

// Timeout: if RSSI not updated for this long, consider signal lost
#define RSSI_TIMEOUT_MS       2000

// ========================== Follow Me State ==========================

enum FollowMeZone {
    ZONE_LOST = 0,    // Signal too weak or lost
    ZONE_FAR,         // Far from user, drive faster
    ZONE_FOLLOW,      // Normal following distance
    ZONE_IDEAL,       // Ideal distance, hold position
    ZONE_CLOSE,       // Too close, stop or reverse
};

class FollowMe {
public:
    FollowMe() {}

    /**
     * @brief Update RSSI reading and compute follow speed
     * @param rawRssi Current RSSI value (0-255, higher = closer)
     * @return true if RSSI was updated
     */
    bool updateRssi(uint8_t rawRssi) {
        if (rawRssi == 0) return false;

        _lastRssiTime = millis();
        _rawRssi = rawRssi;
        _hasRssi = true;

        // Apply exponential moving average filter
        if (!_filterInitialized) {
            _filteredRssi = (float)rawRssi;
            _filterInitialized = true;
        } else {
            _filteredRssi = RSSI_FILTER_ALPHA * (float)rawRssi +
                           (1.0f - RSSI_FILTER_ALPHA) * _filteredRssi;
        }

        // Determine zone
        _zone = classifyZone((uint8_t)_filteredRssi);

        return true;
    }

    /**
     * @brief Get the computed follow speed based on current RSSI
     * @return Speed value suitable for hoverboard [-1000, 1000]
     *         Returns 0 if signal is lost or Follow Me is disabled
     */
    int16_t getFollowSpeed() {
        if (!_enabled || !_hasRssi) return 0;

        // Check for timeout (RSSI not updated recently)
        if (millis() - _lastRssiTime > RSSI_TIMEOUT_MS) {
            _zone = ZONE_LOST;
            return 0;  // Safety: stop if no signal
        }

        uint8_t rssi = (uint8_t)_filteredRssi;

        switch (_zone) {
            case ZONE_LOST:
                return 0;  // Stop immediately

            case ZONE_FAR:
                // Drive forward at max follow speed
                return FOLLOW_SPEED_MAX;

            case ZONE_FOLLOW: {
                // Proportional speed: farther away = faster
                // Map RSSI from [RSSI_ZONE_FOLLOW, RSSI_ZONE_IDEAL] to [FOLLOW_SPEED_MAX, FOLLOW_SPEED_MIN]
                float ratio = (float)(RSSI_ZONE_IDEAL - rssi) /
                              (float)(RSSI_ZONE_IDEAL - RSSI_ZONE_FOLLOW);
                ratio = constrain(ratio, 0.0f, 1.0f);
                return (int16_t)(FOLLOW_SPEED_MIN + ratio * (FOLLOW_SPEED_MAX - FOLLOW_SPEED_MIN));
            }

            case ZONE_IDEAL:
                return FOLLOW_HOLD_SPEED;  // Hold position

            case ZONE_CLOSE:
                return FOLLOW_REVERSE_SPEED;  // Slowly reverse

            default:
                return 0;
        }
    }

    /**
     * @brief Toggle Follow Me mode on/off
     */
    void toggle() {
        _enabled = !_enabled;
        if (!_enabled) {
            // Reset state when disabled
            _filterInitialized = false;
            _hasRssi = false;
            _zone = ZONE_LOST;
        }
        Serial.printf("[FOLLOW] Follow Me mode: %s\n", _enabled ? "ENABLED" : "DISABLED");
    }

    void enable()  { if (!_enabled) toggle(); }
    void disable() { if (_enabled) toggle(); }

    bool isEnabled() const { return _enabled; }
    FollowMeZone getZone() const { return _zone; }
    uint8_t getRawRssi() const { return _rawRssi; }
    float getFilteredRssi() const { return _filteredRssi; }
    bool hasSignal() const { return _hasRssi && (millis() - _lastRssiTime < RSSI_TIMEOUT_MS); }

    // ========================== Direction Steering (Dual-ESP32) ==========================

    /**
     * @brief Set the auto-steering value from direction detection
     * @param steer Steering value from DirectionDetector [-DIR_STEER_MAX, DIR_STEER_MAX]
     *
     * This is called by the master main loop when Satellite RSSI is available.
     * If no direction data is available, set to 0.
     */
    void setDirectionSteer(int16_t steer) {
        _directionSteer = steer;
        _hasDirection = (steer != 0);
    }

    /**
     * @brief Get the auto-steering value for Follow Me mode
     * @return Steering value from direction detection, or 0 if not available.
     *         When direction data is available, this replaces manual joystick steering.
     */
    int16_t getDirectionSteer() const {
        if (!_enabled) return 0;
        return _directionSteer;
    }

    /**
     * @brief Check if direction data (dual-ESP32) is available
     */
    bool hasDirection() const { return _hasDirection; }

    /**
     * @brief Get human-readable zone name
     */
    const char* getZoneName() const {
        switch (_zone) {
            case ZONE_LOST:   return "LOST";
            case ZONE_FAR:    return "FAR";
            case ZONE_FOLLOW: return "FOLLOW";
            case ZONE_IDEAL:  return "IDEAL";
            case ZONE_CLOSE:  return "CLOSE";
            default:          return "???";
        }
    }

private:
    FollowMeZone classifyZone(uint8_t rssi) {
        if (rssi >= RSSI_ZONE_CLOSE)  return ZONE_CLOSE;
        if (rssi >= RSSI_ZONE_IDEAL)  return ZONE_IDEAL;
        if (rssi >= RSSI_ZONE_FOLLOW) return ZONE_FOLLOW;
        if (rssi >= RSSI_ZONE_FAR)    return ZONE_FAR;  // actually "getting far"
        if (rssi >= RSSI_ZONE_LOST)   return ZONE_FAR;
        return ZONE_LOST;
    }

    bool _enabled = false;
    bool _hasRssi = false;
    bool _filterInitialized = false;
    bool _hasDirection = false;

    uint8_t _rawRssi = 0;
    float _filteredRssi = 0;
    int16_t _directionSteer = 0;

    FollowMeZone _zone = ZONE_LOST;
    unsigned long _lastRssiTime = 0;
};

#endif // FOLLOW_ME_H
