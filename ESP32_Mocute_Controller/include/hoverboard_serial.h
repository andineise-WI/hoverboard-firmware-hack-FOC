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
    }

    /**
     * @brief Try to receive feedback from the hoverboard
     * @return true if a valid feedback frame was received
     */
    bool receive() {
        bool newData = false;

        while (_serial.available()) {
            _incomingByte = _serial.read();
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
                }
                _idx = 0;
            }

            _incomingBytePrev = _incomingByte;
        }

        return newData;
    }

    // Public feedback data (updated by receive())
    SerialFeedback feedback = {};

private:
    HardwareSerial &_serial;
    int _rxPin;
    int _txPin;

    // Receive state
    uint8_t  _idx = 0;
    uint16_t _bufStartFrame = 0;
    byte     _incomingByte = 0;
    byte     _incomingBytePrev = 0;
    byte    *_p = nullptr;
    SerialFeedback _newFeedback = {};
};

#endif // HOVERBOARD_SERIAL_H
