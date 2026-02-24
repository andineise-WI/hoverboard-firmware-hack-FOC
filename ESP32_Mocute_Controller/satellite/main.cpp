/**
 * @file main.cpp (Satellite)
 * @brief ESP32 Satellite – BT RSSI Scanner für Dual-ESP32 Follow Me
 *
 * Dieser ESP32 wird auf dem gegenüberliegenden Sideboard des Hoverboards
 * montiert (gegenüber dem Master ESP32). Er scannt nach dem Bluetooth-Signal
 * des Mocute 052 Gamepads und sendet die RSSI-Messung per ESP-NOW an den Master.
 *
 * Funktionsweise:
 *   1. Startet periodisch einen BT Classic Inquiry Scan
 *   2. Sucht nach der MAC-Adresse des Gamepads im Scan-Ergebnis
 *   3. Liest den RSSI-Wert aus dem Inquiry-Ergebnis
 *   4. Sendet den RSSI-Wert per ESP-NOW an den Master ESP32
 *
 * Hinweis: Der Satellite verbindet sich NICHT mit dem Gamepad!
 *          Er liest nur die Signalstärke aus dem BT Inquiry.
 *          Das Gamepad bleibt mit dem Master verbunden.
 *
 * Falls das Gamepad im Connected-Zustand nicht mehr per Inquiry sichtbar ist,
 * kann alternativ ein BLE Beacon-Tag verwendet werden, den der Nutzer
 * zusätzlich zum Gamepad trägt.
 *
 * Hardware:
 *   - ESP32 DevKit V1 (oder kompatibel)
 *   - Montiert auf dem linken Sideboard (gegenüber dem Master)
 *   - Keine UART-Verbindung zum Hoverboard nötig
 *   - Stromversorgung über USB oder 3.3V/5V vom Sideboard
 *
 * LED-Signalisierung:
 *   - LED aus:        Idle / kein Scan
 *   - LED blinkt:     Scanning
 *   - LED an:         Gamepad gefunden + RSSI wird gesendet
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_bt_api.h>
#include "espnow_protocol.h"

// ========================== Configuration ==========================

// Status LED
#define LED_PIN         2

// Master ESP32 MAC address - MUST be set to your Master's WiFi MAC!
// Find it by running: WiFi.macAddress() on the Master
// Format: {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}
uint8_t masterMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // CHANGE THIS!

// Target Gamepad MAC address - Set to your Mocute 052's BT MAC
// Find it in the Master's serial output when the gamepad connects
// Or use BT scan mode (set to all-zero to scan all devices)
uint8_t targetGamepadMac[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // CHANGE THIS!

// If true, report RSSI of ANY found BT device (useful for initial setup)
// Set to false once you know the gamepad MAC
#define SCAN_ALL_DEVICES    true

// BT Inquiry scan parameters
#define BT_INQUIRY_DURATION    3     // Inquiry duration in 1.28s units (3 = ~3.84s)
#define BT_INQUIRY_NUM_RESP    10    // Max number of responses to collect
#define SCAN_INTERVAL_MS       2000  // How often to start a new scan [ms]

// BLE Beacon fallback (if BT Classic inquiry doesn't find the gamepad)
#define USE_BLE_BEACON_FALLBACK  false
// BLE Beacon MAC or name (if using fallback)
// #define BLE_BEACON_MAC      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

// ========================== Global State ==========================

static uint8_t     seqNum = 0;
static bool        espnowReady = false;
static bool        targetFound = false;
static int8_t      lastRssiDbm = 0;
static uint8_t     lastRssiRaw = 0;
static uint8_t     scanConfidence = 0;
static bool        isScanning = false;
static unsigned long lastScanStart = 0;
static unsigned long lastSendTime = 0;
static unsigned long lastTargetSeen = 0;
static esp_now_peer_info_t peerInfo;

// ========================== ESP-NOW Callbacks ==========================

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        static unsigned long lastWarn = 0;
        if (millis() - lastWarn > 5000) {
            Serial.println("[ESPNOW] Send failed!");
            lastWarn = millis();
        }
    }
}

void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    // Handle messages from Master (e.g., target MAC configuration)
    if (len < (int)sizeof(EspNowHeader)) return;

    const EspNowHeader* hdr = (const EspNowHeader*)data;
    if (!espnow_validate_header(hdr)) return;

    if (hdr->msgType == MSG_TYPE_CONFIG && len >= (int)sizeof(EspNowConfig)) {
        const EspNowConfig* cfg = (const EspNowConfig*)data;
        memcpy(targetGamepadMac, cfg->targetMac, 6);
        Serial.printf("[ESPNOW] Target MAC updated: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      targetGamepadMac[0], targetGamepadMac[1], targetGamepadMac[2],
                      targetGamepadMac[3], targetGamepadMac[4], targetGamepadMac[5]);
    }
}

// ========================== BT Classic GAP Callback ==========================

/**
 * @brief Callback for BT Classic GAP events (inquiry results)
 */
void bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT: {
            // Device found during inquiry
            esp_bt_gap_cb_param_t::disc_res_param *disc = &param->disc_res;

            // Check if this is our target gamepad
            bool isTarget = false;

            if (SCAN_ALL_DEVICES) {
                isTarget = true;  // Accept any device during discovery mode
            } else {
                // Check MAC address match
                isTarget = (memcmp(disc->bda, targetGamepadMac, 6) == 0);
            }

            if (isTarget) {
                // Extract RSSI from inquiry result
                int8_t rssi = 0;
                bool hasRssi = false;

                for (int i = 0; i < disc->num_prop; i++) {
                    if (disc->prop[i].type == ESP_BT_GAP_DEV_PROP_RSSI) {
                        rssi = *(int8_t*)disc->prop[i].val;
                        hasRssi = true;
                    }
                }

                if (hasRssi) {
                    lastRssiDbm = rssi;
                    // Convert dBm to BTstack-like 0-255 scale
                    // Typical BT Classic RSSI: -100 dBm (far) to -30 dBm (close)
                    // Map: -100 → 0, -30 → 255
                    int mapped = (int)(((float)(rssi + 100) / 70.0f) * 255.0f);
                    lastRssiRaw = (uint8_t)constrain(mapped, 0, 255);
                    scanConfidence = 80;  // Good confidence from inquiry result
                    targetFound = true;
                    lastTargetSeen = millis();

                    Serial.printf("[BT-SCAN] Found device %02X:%02X:%02X:%02X:%02X:%02X "
                                  "RSSI=%d dBm (raw=%u)\n",
                                  disc->bda[0], disc->bda[1], disc->bda[2],
                                  disc->bda[3], disc->bda[4], disc->bda[5],
                                  rssi, lastRssiRaw);
                }
            }
            break;
        }

        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
            if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
                isScanning = false;
                Serial.println("[BT-SCAN] Inquiry complete");
            } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
                isScanning = true;
                Serial.println("[BT-SCAN] Inquiry started...");
            }
            break;
        }

        default:
            break;
    }
}

// ========================== ESP-NOW Setup ==========================

bool setupEspNow() {
    // Initialize WiFi in STA mode (required for ESP-NOW)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.printf("[ESPNOW] Satellite MAC: %s\n", WiFi.macAddress().c_str());

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNOW] Error initializing ESP-NOW!");
        return false;
    }

    // Register callbacks
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // Register Master as peer
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, masterMacAddress, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;  // Set true + add LMK for encryption

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESPNOW] Failed to add Master as peer!");
        return false;
    }

    Serial.printf("[ESPNOW] Master peer added: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  masterMacAddress[0], masterMacAddress[1], masterMacAddress[2],
                  masterMacAddress[3], masterMacAddress[4], masterMacAddress[5]);

    return true;
}

// ========================== BT Classic Setup ==========================

bool setupBtClassic() {
    // Release BLE memory (we only need Classic BT for inquiry)
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        Serial.println("[BT] Controller init failed!");
        return false;
    }

    if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
        Serial.println("[BT] Controller enable failed!");
        return false;
    }

    if (esp_bluedroid_init() != ESP_OK) {
        Serial.println("[BT] Bluedroid init failed!");
        return false;
    }

    if (esp_bluedroid_enable() != ESP_OK) {
        Serial.println("[BT] Bluedroid enable failed!");
        return false;
    }

    // Register GAP callback for inquiry results
    esp_bt_gap_register_callback(bt_gap_callback);

    // Set device name
    esp_bt_dev_set_device_name("HoverSat");

    // Set scan mode: non-connectable, non-discoverable (we're only scanning)
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

    Serial.println("[BT] Classic BT initialized for inquiry scanning");
    return true;
}

// ========================== Send RSSI via ESP-NOW ==========================

void sendRssiReport() {
    EspNowRssiReport msg;
    memset(&msg, 0, sizeof(msg));
    espnow_init_header(&msg.header, MSG_TYPE_RSSI_REPORT, seqNum++);

    msg.rssi_dbm = lastRssiDbm;
    msg.rssi_raw = lastRssiRaw;
    msg.scanMethod = 0;  // BT Classic inquiry
    msg.confidence = scanConfidence;
    msg.targetFound = targetFound ? 1 : 0;

    esp_err_t result = esp_now_send(masterMacAddress, (uint8_t*)&msg, sizeof(msg));
    if (result != ESP_OK) {
        Serial.printf("[ESPNOW] Send error: %d\n", result);
    }
}

void sendHeartbeat() {
    EspNowHeartbeat msg;
    memset(&msg, 0, sizeof(msg));
    espnow_init_header(&msg.header, MSG_TYPE_HEARTBEAT, seqNum++);

    msg.status = isScanning ? 1 : (targetFound ? 2 : 0);

    esp_now_send(masterMacAddress, (uint8_t*)&msg, sizeof(msg));
}

// ========================== Start BT Inquiry ==========================

void startInquiryScan() {
    if (isScanning) return;  // Already scanning

    // Start inquiry
    // inquiry_mode: ESP_BT_INQ_MODE_GENERAL_INQUIRY
    // duration: in 1.28s units
    // num_rsps: max responses (0 = unlimited)
    esp_err_t err = esp_bt_gap_start_discovery(
        ESP_BT_INQ_MODE_GENERAL_INQUIRY,
        BT_INQUIRY_DURATION,
        BT_INQUIRY_NUM_RESP
    );

    if (err != ESP_OK) {
        Serial.printf("[BT-SCAN] Start inquiry failed: %d\n", err);
    }
}

// ========================== SETUP ==========================

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("============================================");
    Serial.println("  ESP32 Satellite - BT RSSI Scanner");
    Serial.println("  Dual-ESP32 Follow Me Direction Detection");
    Serial.println("============================================");
    Serial.println();

    // LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Setup ESP-NOW first (uses WiFi)
    espnowReady = setupEspNow();
    if (!espnowReady) {
        Serial.println("[ERROR] ESP-NOW setup failed! Halting.");
        while (true) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(200);  // Fast blink = error
        }
    }

    // Setup BT Classic for inquiry scanning
    if (!setupBtClassic()) {
        Serial.println("[ERROR] BT Classic setup failed! Halting.");
        while (true) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(500);  // Medium blink = BT error
        }
    }

    // Print target info
    bool hasTarget = false;
    for (int i = 0; i < 6; i++) {
        if (targetGamepadMac[i] != 0) { hasTarget = true; break; }
    }

    if (hasTarget) {
        Serial.printf("[SETUP] Target gamepad MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      targetGamepadMac[0], targetGamepadMac[1], targetGamepadMac[2],
                      targetGamepadMac[3], targetGamepadMac[4], targetGamepadMac[5]);
    } else if (SCAN_ALL_DEVICES) {
        Serial.println("[SETUP] SCAN_ALL_DEVICES=true - reporting RSSI of all found BT devices");
        Serial.println("[SETUP] Set targetGamepadMac to your Mocute's MAC for production use!");
    } else {
        Serial.println("[SETUP] WARNING: No target MAC set and SCAN_ALL_DEVICES=false!");
        Serial.println("[SETUP] Set targetGamepadMac or enable SCAN_ALL_DEVICES!");
    }

    Serial.println();
    Serial.println("[SETUP] Starting BT inquiry scan loop...");
    Serial.println();
}

// ========================== LOOP ==========================

void loop() {
    unsigned long now = millis();

    // Start periodic BT inquiry scan
    if (now - lastScanStart >= SCAN_INTERVAL_MS && !isScanning) {
        lastScanStart = now;
        startInquiryScan();
    }

    // Send RSSI report or heartbeat at regular intervals
    if (now - lastSendTime >= ESPNOW_SEND_INTERVAL_MS) {
        lastSendTime = now;

        if (targetFound && (now - lastTargetSeen < 10000)) {
            sendRssiReport();

            // Decay confidence over time since last seen
            unsigned long age = now - lastTargetSeen;
            if (age > 5000) {
                scanConfidence = 20;
            } else if (age > 3000) {
                scanConfidence = 50;
            }
            // else keep original confidence from scan
        } else {
            sendHeartbeat();
            targetFound = false;
            scanConfidence = 0;
        }
    }

    // LED status
    if (targetFound && (now - lastTargetSeen < 5000)) {
        digitalWrite(LED_PIN, HIGH);  // Solid = target found
    } else if (isScanning) {
        // Blink while scanning
        digitalWrite(LED_PIN, (now / 200) % 2);
    } else {
        digitalWrite(LED_PIN, LOW);   // Off = idle
    }

    // Debug output every 2s
    static unsigned long lastDebug = 0;
    if (now - lastDebug > 2000) {
        lastDebug = now;
        if (targetFound) {
            Serial.printf("[SAT] RSSI=%d dBm (raw=%u) conf=%u%% scanning=%d\n",
                          lastRssiDbm, lastRssiRaw, scanConfidence, isScanning);
        } else {
            Serial.printf("[SAT] No target found. scanning=%d espnow=%d\n",
                          isScanning, espnowReady);
        }
    }

    delay(10);
}
