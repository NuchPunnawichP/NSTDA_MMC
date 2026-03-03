// ═══════════════════════════════════════════════════════════════════════════════
//  ble_service.h — BLE GATT Service (Public API)
//
//  Layer: Core Infrastructure
//  Dependencies: protocol.h, comms_state.h
//
//  ★ Iron Rules:
//    - BLEDevice.h / BLEServer.h / etc. are #included ONLY in ble_service.cpp
//    - This header exposes ONLY the minimal API needed by other modules
//    - BLE callbacks: copy → enqueue → return. No parsing. No blocking.
//    - All notify() calls go through this module, called by CommsTask only
//    - Every notify characteristic has CCCD (BLE2902) for iOS
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Initialization — call from setup(), after comms_state_init()
// ─────────────────────────────────────────────────────────────────────────────
// Sets up BLE stack, GATT service, characteristics, descriptors.
// Starts advertising. Returns true on success.
bool ble_service_init(const char* device_name);

// ─────────────────────────────────────────────────────────────────────────────
// Notify (send packet to connected client)
//   Called by CommsTask (Core0) ONLY — single notify authority.
//   All packets must be exactly 20 bytes.
//   Returns true if notify succeeded, false if no client or error.
// ─────────────────────────────────────────────────────────────────────────────
bool ble_notify_rsp(const RspPacket* pkt);
bool ble_notify_fast(const FastPacket* pkt);
bool ble_notify_slow(const SlowPacket* pkt);

// ─────────────────────────────────────────────────────────────────────────────
// CAPS — update and serve capabilities packet
//   Called once at boot after HAL init, to set correct sensor info.
// ─────────────────────────────────────────────────────────────────────────────
void ble_set_caps(const CapsPacket* caps);

// ─────────────────────────────────────────────────────────────────────────────
// Connection state (redundant with comms_state, but for convenience)
// ─────────────────────────────────────────────────────────────────────────────
bool ble_is_connected(void);

// ─────────────────────────────────────────────────────────────────────────────
// Start / Stop advertising
// ─────────────────────────────────────────────────────────────────────────────
void ble_start_advertising(void);
void ble_stop_advertising(void);

#ifdef __cplusplus
}
#endif

#endif // BLE_SERVICE_H
