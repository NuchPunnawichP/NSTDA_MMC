// ═══════════════════════════════════════════════════════════════════════════════
//  ble_service.cpp — BLE GATT Service Implementation
//
//  BLEDevice.h lives HERE and ONLY HERE.
//
//  GATT layout:
//    Service (UUID_SERVICE)
//    ├── CAPS  (READ)           — 20B capabilities, set once at boot
//    ├── CMD   (WRITE_NR)       — 20B commands from host
//    ├── RSP   (NOTIFY + CCCD)  — 20B responses to host
//    ├── FAST  (NOTIFY + CCCD)  — 20B real-time telemetry
//    └── SLOW  (NOTIFY + CCCD)  — 20B diagnostics
//
//  CMD write callback:
//    1. Check len == 20 (else drop + counter)
//    2. Copy 20 bytes to local buffer
//    3. Enqueue raw bytes into telemetry_bridge for parsing
//    4. Return immediately (NO parsing, NO blocking)
// ═══════════════════════════════════════════════════════════════════════════════
#include "ble_service.h"
#include "comms_state.h"
#include "config.h"

// ── BLE headers — ONLY in this file ─────────────────────────────────────────
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ── Forward declarations for callbacks ──────────────────────────────────────
class ServerCallbacks;
class CmdCallbacks;

// ── Static GATT objects ─────────────────────────────────────────────────────
static BLEServer*         s_server   = nullptr;
static BLEService*        s_service  = nullptr;
static BLECharacteristic* s_caps_chr = nullptr;
static BLECharacteristic* s_cmd_chr  = nullptr;
static BLECharacteristic* s_rsp_chr  = nullptr;
static BLECharacteristic* s_fast_chr = nullptr;
static BLECharacteristic* s_slow_chr = nullptr;

static bool s_init = false;

// ═══════════════════════════════════════════════════════════════════════════════
// Raw CMD Queue — BLE callback copies raw 20 bytes here
//   Telemetry bridge dequeues from comms task context for parsing.
//   Using a FreeRTOS queue of raw 20-byte buffers.
// ═══════════════════════════════════════════════════════════════════════════════
#define RAW_CMD_QUEUE_DEPTH     8

typedef struct {
    uint8_t data[PROTO_PACKET_SIZE];
} RawCmdBuf;

static QueueHandle_t s_rawCmdQ = NULL;

// Public getter for telemetry_bridge to dequeue raw CMD packets
QueueHandle_t ble_get_raw_cmd_queue(void) {
    return s_rawCmdQ;
}

// ═══════════════════════════════════════════════════════════════════════════════
// BLE Server Callbacks (connect / disconnect)
// ═══════════════════════════════════════════════════════════════════════════════
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        comms_set_ble_connected(true);
        // Note: MTU negotiation happens via onMtuChanged (below)
    }

    void onDisconnect(BLEServer* pServer) override {
        comms_set_ble_connected(false);
    }
};

// ═══════════════════════════════════════════════════════════════════════════════
// CMD Characteristic Callback (WRITE_NR)
//   Iron Rule: copy → enqueue → return. Nothing else.
// ═══════════════════════════════════════════════════════════════════════════════
class CmdCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pChr) override {
        // Get received data
        size_t len = pChr->getLength();
        const uint8_t* data = pChr->getData();

        // ── Rule: accept ONLY 20 bytes ──
        if (len != PROTO_PACKET_SIZE || data == nullptr) {
            comms_inc_rx_drop();
            return;
        }

        // ── Copy into raw buffer and enqueue ──
        // Non-blocking: if queue full, drop and count
        RawCmdBuf buf;
        memcpy(buf.data, data, PROTO_PACKET_SIZE);

        if (s_rawCmdQ != NULL) {
            if (xQueueSend(s_rawCmdQ, &buf, 0) != pdTRUE) {
                comms_inc_rx_drop();
            }
        }
        // Return immediately — parsing happens in CommsTask context
    }
};

// Static callback instances
static ServerCallbacks s_serverCb;
static CmdCallbacks    s_cmdCb;

// ═══════════════════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════════════════

bool ble_service_init(const char* device_name) {
    // Create raw CMD queue
    s_rawCmdQ = xQueueCreate(RAW_CMD_QUEUE_DEPTH, sizeof(RawCmdBuf));
    if (!s_rawCmdQ) return false;

    // ── Init BLE stack ──
    BLEDevice::init(device_name);

    // Set MTU to allow 20-byte payload (MTU 23 is default, payload = MTU-3 = 20)
    // Don't request larger MTU — we want cross-platform 20B guarantee
    BLEDevice::setMTU(23);

    // ── Create server ──
    s_server = BLEDevice::createServer();
    s_server->setCallbacks(&s_serverCb);

    // ── Create service ──
    s_service = s_server->createService(UUID_SERVICE);

    // ── CAPS characteristic (READ only) ──
    s_caps_chr = s_service->createCharacteristic(
        UUID_CAPS,
        BLECharacteristic::PROPERTY_READ
    );

    // ── CMD characteristic (WRITE without response) ──
    s_cmd_chr = s_service->createCharacteristic(
        UUID_CMD,
        BLECharacteristic::PROPERTY_WRITE_NR
    );
    s_cmd_chr->setCallbacks(&s_cmdCb);

    // ── RSP characteristic (NOTIFY) + CCCD for iOS ──
    s_rsp_chr = s_service->createCharacteristic(
        UUID_RSP,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    s_rsp_chr->addDescriptor(new BLE2902());

    // ── FAST characteristic (NOTIFY) + CCCD for iOS ──
    s_fast_chr = s_service->createCharacteristic(
        UUID_FAST,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    s_fast_chr->addDescriptor(new BLE2902());

    // ── SLOW characteristic (NOTIFY) + CCCD for iOS ──
    s_slow_chr = s_service->createCharacteristic(
        UUID_SLOW,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    s_slow_chr->addDescriptor(new BLE2902());

    // ── Start service ──
    s_service->start();

    // ── Start advertising ──
    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(UUID_SERVICE);
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);   // connection interval hint
    adv->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();

    s_init = true;
    return true;
}

// ── Notify functions (CommsTask only) ───────────────────────────────────────

bool ble_notify_rsp(const RspPacket* pkt) {
    if (!s_init || !comms_get_ble_connected() || !pkt) return false;
    s_rsp_chr->setValue((uint8_t*)pkt, PROTO_PACKET_SIZE);
    s_rsp_chr->notify();
    return true;
}

bool ble_notify_fast(const FastPacket* pkt) {
    if (!s_init || !comms_get_ble_connected() || !pkt) return false;
    s_fast_chr->setValue((uint8_t*)pkt, PROTO_PACKET_SIZE);
    s_fast_chr->notify();
    return true;
}

bool ble_notify_slow(const SlowPacket* pkt) {
    if (!s_init || !comms_get_ble_connected() || !pkt) return false;
    s_slow_chr->setValue((uint8_t*)pkt, PROTO_PACKET_SIZE);
    s_slow_chr->notify();
    return true;
}

void ble_set_caps(const CapsPacket* caps) {
    if (!s_init || !caps) return;
    s_caps_chr->setValue((uint8_t*)caps, PROTO_PACKET_SIZE);
}

bool ble_is_connected(void) {
    return comms_get_ble_connected();
}

void ble_start_advertising(void) {
    if (!s_init) return;
    BLEDevice::startAdvertising();
}

void ble_stop_advertising(void) {
    if (!s_init) return;
    // ESP32 BLE: stop advertising by removing service UUID and calling stop
    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->stop();
}
