/**
 * @file hoverboard_serial.h
 * @brief Hoverboard serial communication protocol
 *
 * Implements the serial protocol used by hoverboard-firmware-hack-FOC.
 * Protocol:  START_FRAME (0xABCD) | steer (int16) | speed (int16) | checksum (uint16)
 * Checksum:  XOR of start ^ steer ^ speed
 * Baud rate: 115200
 */

#ifndef HOVERBOARD_SERIAL_H
#define HOVERBOARD_SERIAL_H

#include <Arduino.h>

// ========================== Protocol Defines ==========================
#define HOVER_START_FRAME   0xABCD    // Start frame marker
#define HOVER_SERIAL_BAUD   115200    // Baud rate for hoverboard communication
#define HOVER_SEND_INTERVAL 50        // [ms] Command send interval (20 Hz)

// Input range for hoverboard commands
#define HOVER_INPUT_MIN    -1000
#define HOVER_INPUT_MAX     1000

// ========================== Command Structure ==========================
// Sent TO the hoverboard
typedef struct __attribute__((packed)) {
    uint16_t start;
    int16_t  steer;
    int16_t  speed;
    uint16_t checksum;
} SerialCommand;

// ========================== Feedback Structure ==========================
// Received FROM the hoverboard
typedef struct __attribute__((packed)) {
    uint16_t start;
    int16_t  cmd1;
    int16_t  cmd2;
    int16_t  speedR_meas;
    int16_t  speedL_meas;
    int16_t  batVoltage;
    int16_t  boardTemp;
    uint16_t cmdLed;
    uint16_t checksum;
} SerialFeedback;

// ========================== HoverboardSerial Class ==========================
class HoverboardSerial {
public:
    HoverboardSerial(HardwareSerial &serial, int rxPin, int txPin)
        : _serial(serial), _rxPin(rxPin), _txPin(txPin) {}

    void begin() {
        _serial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, _rxPin, _txPin);
    }

    // Enable/disable debug output of sent/received frames
    void setDebug(bool enable) { _debug = enable; }
    bool getDebug() const { return _debug; }

    /**
     * @brief Send steer and speed command to the hoverboard
     * @param steer Steering value [-1000, 1000]. Negative=left, Positive=right
     * @param speed Speed value [-1000, 1000]. Negative=backward, Positive=forward
     */
    void send(int16_t steer, int16_t speed) {
        // Clamp values
        steer = constrain(steer, HOVER_INPUT_MIN, HOVER_INPUT_MAX);
        speed = constrain(speed, HOVER_INPUT_MIN, HOVER_INPUT_MAX);

        SerialCommand cmd;
        cmd.start    = (uint16_t)HOVER_START_FRAME;
        cmd.steer    = steer;
        cmd.speed    = speed;
        cmd.checksum = (uint16_t)(cmd.start ^ cmd.steer ^ cmd.speed);

        _serial.write((uint8_t *)&cmd, sizeof(cmd));

        _lastCmd = cmd;
        _sendCount++;

        if (_debug) {
            // Print decoded fields + raw hex bytes
            Serial.printf("[HOVER-TX] #%lu steer=%4d speed=%4d chk=0x%04X | HEX:",
                          _sendCount, cmd.steer, cmd.speed, cmd.checksum);
            const uint8_t *p = (const uint8_t *)&cmd;
            for (size_t i = 0; i < sizeof(cmd); i++) {
                Serial.printf(" %02X", p[i]);
            }
            Serial.println();
        }
    }

    /**
     * @brief Try to receive feedback from the hoverboard
     * @param maxBytes Maximum bytes to process per call (0 = unlimited)
     *                 Limits processing time to prevent WDT timeout.
     * @return true if a valid feedback frame was received
     */
    bool receive(uint16_t maxBytes = 512) {
        bool newData = false;
        uint16_t bytesProcessed = 0;

        while (_serial.available() && (maxBytes == 0 || bytesProcessed < maxBytes)) {
            _incomingByte = _serial.read();
            bytesProcessed++;
            _bufStartFrame = ((uint16_t)(_incomingByte) << 8) | _incomingBytePrev;

            // Detect start frame
            if (_bufStartFrame == HOVER_START_FRAME) {
                _p    = (byte *)&_newFeedback;
                *_p++ = _incomingBytePrev;
                *_p++ = _incomingByte;
                _idx  = 2;
            } else if (_idx >= 2 && _idx < sizeof(SerialFeedback)) {
                *_p++ = _incomingByte;
                _idx++;
            }

            // Check if full frame received
            if (_idx == sizeof(SerialFeedback)) {
                uint16_t checksum = (uint16_t)(
                    _newFeedback.start ^ _newFeedback.cmd1 ^ _newFeedback.cmd2 ^
                    _newFeedback.speedR_meas ^ _newFeedback.speedL_meas ^
                    _newFeedback.batVoltage ^ _newFeedback.boardTemp ^ _newFeedback.cmdLed);

                if (_newFeedback.start == HOVER_START_FRAME && checksum == _newFeedback.checksum) {
                    memcpy(&feedback, &_newFeedback, sizeof(SerialFeedback));
                    newData = true;
                    _rxGoodCount++;
                    if (_debug) {
                        // Rate-limit debug output: max 2 prints/sec to prevent WDT
                        unsigned long now = millis();
                        if (now - _lastDebugPrintMs >= 500) {
                            _lastDebugPrintMs = now;
                            Serial.printf("[HOVER-RX] #%lu speedL=%d speedR=%d bat=%d temp=%d | HEX:",
                                          _rxGoodCount, feedback.speedL_meas, feedback.speedR_meas,
                                          feedback.batVoltage, feedback.boardTemp);
                            const uint8_t *p = (const uint8_t *)&feedback;
                            for (size_t i = 0; i < sizeof(SerialFeedback); i++) {
                                Serial.printf(" %02X", p[i]);
                            }
                            Serial.println();
                        }
                    }
                } else {
                    _rxBadCount++;
                    if (_debug) {
                        unsigned long now = millis();
                        if (now - _lastDebugPrintMs >= 500) {
                            _lastDebugPrintMs = now;
                            Serial.printf("[HOVER-RX] BAD FRAME #%lu (start=0x%04X, chk=0x%04X expected=0x%04X)\n",
                                          _rxBadCount, _newFeedback.start, _newFeedback.checksum, checksum);
                        }
                    }
                }
                _idx = 0;
            }

            _incomingBytePrev = _incomingByte;
        }

        return newData;
    }

    // Public feedback data (updated by receive())
    SerialFeedback feedback = {};

    // Statistics
    unsigned long getSendCount() const { return _sendCount; }
    unsigned long getRxGoodCount() const { return _rxGoodCount; }
    unsigned long getRxBadCount() const { return _rxBadCount; }
    SerialCommand getLastCommand() const { return _lastCmd; }

    /**
     * @brief Print a summary of UART communication status
     */
    void printStatus() {
        Serial.printf("[HOVER-DBG] TX pin=%d, RX pin=%d, baud=%d\n", _txPin, _rxPin, HOVER_SERIAL_BAUD);
        Serial.printf("[HOVER-DBG] Frames sent: %lu | RX good: %lu | RX bad: %lu\n",
                      _sendCount, _rxGoodCount, _rxBadCount);
        Serial.printf("[HOVER-DBG] Last TX: start=0x%04X steer=%d speed=%d chk=0x%04X\n",
                      _lastCmd.start, _lastCmd.steer, _lastCmd.speed, _lastCmd.checksum);
        Serial.printf("[HOVER-DBG] UART RX bytes available: %d\n", _serial.available());
    }

private:
    HardwareSerial &_serial;
    int _rxPin;
    int _txPin;
    bool _debug = false;

    // Receive state
    uint8_t  _idx = 0;
    uint16_t _bufStartFrame = 0;
    byte     _incomingByte = 0;
    byte     _incomingBytePrev = 0;
    byte    *_p = nullptr;
    SerialFeedback _newFeedback = {};

    // Debug / statistics
    SerialCommand _lastCmd = {};
    unsigned long _sendCount = 0;
    unsigned long _rxGoodCount = 0;
    unsigned long _rxBadCount = 0;
    unsigned long _lastDebugPrintMs = 0;
};

#endif // HOVERBOARD_SERIAL_H
