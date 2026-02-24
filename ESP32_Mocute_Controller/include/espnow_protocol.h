/**
 * @file espnow_protocol.h
 * @brief ESP-NOW Kommunikationsprotokoll zwischen Master und Satellite ESP32
 *
 * Dieses gemeinsame Header definiert das Nachrichtenformat für die
 * ESP-NOW Kommunikation zwischen den beiden ESP32-Boards im
 * Dual-ESP32 Follow Me System.
 *
 * Architektur:
 *   Satellite ESP32 (Sideboard L) ──ESP-NOW──> Master ESP32 (Sideboard R)
 *                                                    │
 *                                                    ├── Bluepad32 (Gamepad BT)
 *                                                    └── UART (Hoverboard)
 *
 * Der Satellite misst RSSI des Gamepads und sendet den Wert per ESP-NOW
 * an den Master. Der Master vergleicht sein eigenes RSSI mit dem des
 * Satellite und berechnet daraus die Richtung des Nutzers.
 */

#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include <stdint.h>

// ========================== ESP-NOW Configuration ==========================

// ESP-NOW WiFi channel (must match on both devices)
#define ESPNOW_CHANNEL          1

// ESP-NOW PMK (Primary Master Key) for encryption (16 bytes)
// Change this to your own key for security!
#define ESPNOW_PMK              "HoverFollowMe!16"

// Message send interval from Satellite [ms]
#define ESPNOW_SEND_INTERVAL_MS 100

// Timeout: if no message from Satellite for this time, consider it offline
#define ESPNOW_TIMEOUT_MS       1000

// ========================== Message Types ==========================

enum EspNowMsgType : uint8_t {
    MSG_TYPE_RSSI_REPORT   = 0x01,   // Satellite -> Master: RSSI measurement
    MSG_TYPE_HEARTBEAT     = 0x02,   // Satellite -> Master: alive signal
    MSG_TYPE_CONFIG        = 0x03,   // Master -> Satellite: configuration
    MSG_TYPE_STATUS        = 0x04,   // Master -> Satellite: status update
};

// ========================== Message Structures ==========================

/**
 * @brief Header for all ESP-NOW messages
 */
struct __attribute__((packed)) EspNowHeader {
    uint8_t     magic[2];       // Magic bytes: 0xFE, 0xED
    uint8_t     msgType;        // EspNowMsgType
    uint8_t     seqNum;         // Sequence number (rolling 0-255)
    uint32_t    timestamp;      // millis() of sender
};

/**
 * @brief RSSI report from Satellite to Master
 * Satellite measures the BT signal strength of the gamepad and reports it
 */
struct __attribute__((packed)) EspNowRssiReport {
    EspNowHeader header;
    int8_t       rssi_dbm;      // RSSI in dBm (negative, e.g. -60)
    uint8_t      rssi_raw;      // Raw RSSI (0-255 BTstack format)
    uint8_t      scanMethod;    // 0=BT Classic inquiry, 1=BLE scan, 2=beacon
    uint8_t      confidence;    // 0-100: measurement confidence
    uint8_t      targetFound;   // 1=target device found, 0=not found
    uint8_t      reserved[3];   // Future use
};

/**
 * @brief Heartbeat from Satellite (sent when no RSSI available)
 */
struct __attribute__((packed)) EspNowHeartbeat {
    EspNowHeader header;
    uint8_t      status;        // 0=idle, 1=scanning, 2=found target
    uint8_t      batteryPct;    // Battery percentage (if available)
    uint8_t      reserved[2];
};

/**
 * @brief Configuration from Master to Satellite
 */
struct __attribute__((packed)) EspNowConfig {
    EspNowHeader header;
    uint8_t      targetMac[6];  // MAC address of gamepad to scan for
    uint8_t      scanMode;      // 0=BT Classic, 1=BLE, 2=both
    uint8_t      scanInterval;  // Scan interval in units of 100ms
};

/**
 * @brief Status update from Master to Satellite
 */
struct __attribute__((packed)) EspNowStatus {
    EspNowHeader header;
    uint8_t      followMeActive; // 1=Follow Me mode active
    int16_t      masterRssi;     // Master's own RSSI reading
    uint8_t      reserved[2];
};

// ========================== Magic Bytes ==========================

#define ESPNOW_MAGIC_0     0xFE
#define ESPNOW_MAGIC_1     0xED

// ========================== Helper Functions ==========================

/**
 * @brief Initialize an ESP-NOW message header
 */
static inline void espnow_init_header(EspNowHeader* hdr, EspNowMsgType type, uint8_t seq) {
    hdr->magic[0] = ESPNOW_MAGIC_0;
    hdr->magic[1] = ESPNOW_MAGIC_1;
    hdr->msgType = type;
    hdr->seqNum = seq;
    hdr->timestamp = millis();
}

/**
 * @brief Validate an ESP-NOW message header
 * @return true if magic bytes match
 */
static inline bool espnow_validate_header(const EspNowHeader* hdr) {
    return (hdr->magic[0] == ESPNOW_MAGIC_0 && hdr->magic[1] == ESPNOW_MAGIC_1);
}

// ========================== Side Identification ==========================

// Which side of the hoverboard each ESP32 is mounted on
enum HoverboardSide : uint8_t {
    SIDE_LEFT  = 0,   // Left sideboard
    SIDE_RIGHT = 1,   // Right sideboard
};

// Default assignment (can be changed via build flags):
// Master  = Right sideboard (USART3, 5V tolerant)
// Satellite = Left sideboard
#ifndef MASTER_SIDE
#define MASTER_SIDE     SIDE_RIGHT
#endif

#ifndef SATELLITE_SIDE
#define SATELLITE_SIDE  SIDE_LEFT
#endif

#endif // ESPNOW_PROTOCOL_H
