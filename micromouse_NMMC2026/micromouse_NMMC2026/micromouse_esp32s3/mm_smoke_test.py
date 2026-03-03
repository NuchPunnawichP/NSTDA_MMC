#!/usr/bin/env python3
"""
mm_smoke_test.py — Micromouse BLE Protocol v2.0 Smoke Test

Prerequisites:
    pip install bleak

Usage:
    python mm_smoke_test.py                    # scan + auto-connect to "MM_*"
    python mm_smoke_test.py --addr AA:BB:CC:DD:EE:FF  # connect to specific device

What it tests:
    1. BLE scan + connect
    2. Read CAPS → verify proto_ver, packet sizes, CRC8
    3. Subscribe to RSP, FAST, SLOW notifications
    4. Send CMD_PING → expect RSP with uptime
    5. Send CMD_SET_FAST_HZ(50) → expect RSP OK
    6. Wait 3 seconds, count FAST + SLOW packets received
    7. Verify CRC8 on all received packets
    8. Report results

Pass criteria:
    - CAPS valid, proto_ver == 2
    - PING RSP received with status OK
    - At least 10 FAST packets in 3 seconds
    - 0 CRC failures
"""

import asyncio
import argparse
import struct
import sys
import time
from collections import Counter

# ═══════════════════════════════════════════════════════════════════════════════
# Protocol Constants (mirror of protocol.h)
# ═══════════════════════════════════════════════════════════════════════════════

PROTO_VER = 2
PKT_SIZE = 20

PKT_FAST = 0x01
PKT_SLOW = 0x02
PKT_CAPS = 0x10
PKT_CMD  = 0x20
PKT_RSP  = 0x30

CMD_PING          = 0x01
CMD_ARM           = 0x02
CMD_DISARM        = 0x03
CMD_SET_FAST_HZ   = 0x10
CMD_SET_SLOW_HZ   = 0x11
CMD_SET_STREAM_MASK = 0x12

RSP_OK = 0x00

# GATT UUIDs
UUID_SERVICE = "a0000000-5a19-40b6-926c-f22d00d00000"
UUID_CAPS    = "a0000001-5a19-40b6-926c-f22d00d00000"
UUID_CMD     = "a0000002-5a19-40b6-926c-f22d00d00000"
UUID_RSP     = "a0000003-5a19-40b6-926c-f22d00d00000"
UUID_FAST    = "a0000004-5a19-40b6-926c-f22d00d00000"
UUID_SLOW    = "a0000005-5a19-40b6-926c-f22d00d00000"


# ═══════════════════════════════════════════════════════════════════════════════
# CRC8/ATM — poly=0x07, init=0x00
# ═══════════════════════════════════════════════════════════════════════════════

def crc8_atm(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def verify_crc8(pkt: bytes) -> bool:
    """Verify CRC8 of a 20-byte packet (CRC at byte[19])"""
    if len(pkt) != PKT_SIZE:
        return False
    return crc8_atm(pkt[:19]) == pkt[19]


def build_cmd(opcode: int, req_id: int, payload: bytes = b'') -> bytes:
    """Build a 20-byte CMD packet with CRC8"""
    buf = bytearray(PKT_SIZE)
    buf[0] = PKT_CMD           # type
    buf[1] = 0                 # seq (host can track)
    buf[2] = opcode            # opcode
    struct.pack_into('<H', buf, 3, req_id)  # req_id LE
    # payload goes into bytes [5..18] (14 bytes max)
    n = min(len(payload), 14)
    buf[5:5+n] = payload[:n]
    buf[19] = crc8_atm(bytes(buf[:19]))
    return bytes(buf)


# ═══════════════════════════════════════════════════════════════════════════════
# CAPS parser
# ═══════════════════════════════════════════════════════════════════════════════

def parse_caps(data: bytes) -> dict:
    if len(data) != PKT_SIZE:
        return {'error': f'wrong size {len(data)}'}
    if not verify_crc8(data):
        return {'error': 'CRC fail'}
    return {
        'type':             data[0],
        'proto_ver':        data[1],
        'fw_ver':           struct.unpack_from('<H', data, 2)[0],
        'fast_size':        data[4],
        'slow_size':        data[5],
        'cmd_size':         data[6],
        'rsp_size':         data[7],
        'features':         struct.unpack_from('<H', data, 8)[0],
        'scale_id':         data[10],
        'num_wall_sensors': data[11],
        'wall_sensor_type': data[12],
        'num_labs':         data[13],
        'max_fast_hz':      data[14],
        'max_slow_hz':      data[15],
    }


def parse_fast(data: bytes) -> dict:
    if len(data) != PKT_SIZE or not verify_crc8(data):
        return None
    return {
        'type':      data[0],
        'seq':       data[1],
        't_ms':      struct.unpack_from('<H', data, 2)[0],
        'vCmdL_mV':  struct.unpack_from('<h', data, 4)[0],
        'vCmdR_mV':  struct.unpack_from('<h', data, 6)[0],
        'velL_mmps': struct.unpack_from('<h', data, 8)[0],
        'velR_mmps': struct.unpack_from('<h', data, 10)[0],
        'vBat_mV':   struct.unpack_from('<H', data, 12)[0],
        'gyroZ':     struct.unpack_from('<h', data, 14)[0],
        'encL':      struct.unpack_from('<b', data, 16)[0],
        'encR':      struct.unpack_from('<b', data, 17)[0],
        'state':     data[18],
    }


def parse_rsp(data: bytes) -> dict:
    if len(data) != PKT_SIZE or not verify_crc8(data):
        return None
    return {
        'type':    data[0],
        'seq':     data[1],
        'opcode':  data[2],
        'req_id':  struct.unpack_from('<H', data, 3)[0],
        'status':  data[5],
        'payload': data[6:19],
    }


# ═══════════════════════════════════════════════════════════════════════════════
# Main test
# ═══════════════════════════════════════════════════════════════════════════════

async def run_test(addr: str = None):
    from bleak import BleakClient, BleakScanner

    results = {
        'caps_ok': False,
        'ping_ok': False,
        'set_hz_ok': False,
        'fast_count': 0,
        'slow_count': 0,
        'rsp_count': 0,
        'crc_ok': 0,
        'crc_fail': 0,
    }

    # ── Notification data collectors ──
    fast_packets = []
    slow_packets = []
    rsp_packets = []

    def on_rsp(sender, data):
        data = bytes(data)
        if verify_crc8(data):
            results['crc_ok'] += 1
            rsp_packets.append(data)
        else:
            results['crc_fail'] += 1

    def on_fast(sender, data):
        data = bytes(data)
        if verify_crc8(data):
            results['crc_ok'] += 1
            fast_packets.append(data)
        else:
            results['crc_fail'] += 1

    def on_slow(sender, data):
        data = bytes(data)
        if verify_crc8(data):
            results['crc_ok'] += 1
            slow_packets.append(data)
        else:
            results['crc_fail'] += 1

    # ── Step 1: Scan / Connect ──
    if addr is None:
        print("[SCAN] Scanning for MM_* devices...")
        devices = await BleakScanner.discover(timeout=5.0)
        mm_devs = [d for d in devices if d.name and d.name.startswith("MM")]
        if not mm_devs:
            print("[FAIL] No Micromouse BLE devices found!")
            return False
        device = mm_devs[0]
        addr = device.address
        print(f"[SCAN] Found: {device.name} ({addr})")
    else:
        print(f"[CONN] Connecting to {addr}...")

    async with BleakClient(addr) as client:
        if not client.is_connected:
            print("[FAIL] Could not connect!")
            return False
        print(f"[CONN] Connected! MTU={client.mtu_size}")

        # ── Step 2: Read CAPS ──
        print("\n[TEST] Reading CAPS...")
        caps_data = await client.read_gatt_char(UUID_CAPS)
        caps = parse_caps(bytes(caps_data))
        if 'error' in caps:
            print(f"  [FAIL] CAPS error: {caps['error']}")
        else:
            print(f"  proto_ver:        {caps['proto_ver']}")
            print(f"  fw_ver:           {caps['fw_ver']:#06x}")
            print(f"  fast/slow/cmd/rsp: {caps['fast_size']}/{caps['slow_size']}/{caps['cmd_size']}/{caps['rsp_size']}")
            print(f"  features:         {caps['features']:#06x}")
            print(f"  scale_id:         {caps['scale_id']}")
            print(f"  wall_sensors:     {caps['num_wall_sensors']} (type={caps['wall_sensor_type']})")
            print(f"  num_labs:         {caps['num_labs']}")
            print(f"  max_fast_hz:      {caps['max_fast_hz']}")

            if caps['proto_ver'] == PROTO_VER and caps['fast_size'] == 20:
                results['caps_ok'] = True
                print("  [PASS] CAPS valid ✓")
            else:
                print("  [FAIL] CAPS mismatch!")

        # ── Step 3: Subscribe to notifications ──
        print("\n[TEST] Subscribing to RSP/FAST/SLOW...")
        await client.start_notify(UUID_RSP, on_rsp)
        await client.start_notify(UUID_FAST, on_fast)
        await client.start_notify(UUID_SLOW, on_slow)
        await asyncio.sleep(0.5)

        # ── Step 4: Send PING ──
        print("\n[TEST] Sending CMD_PING...")
        ping_cmd = build_cmd(CMD_PING, req_id=1)
        await client.write_gatt_char(UUID_CMD, ping_cmd, response=False)
        await asyncio.sleep(0.5)

        # Check RSP
        for pkt in rsp_packets:
            r = parse_rsp(pkt)
            if r and r['opcode'] == CMD_PING and r['req_id'] == 1:
                uptime = struct.unpack_from('<I', r['payload'], 0)[0]
                print(f"  RSP: status={r['status']:#04x}, uptime={uptime}ms")
                if r['status'] == RSP_OK:
                    results['ping_ok'] = True
                    print("  [PASS] PING OK ✓")
                break
        else:
            print("  [FAIL] No PING response received!")

        # ── Step 5: Set FAST_HZ ──
        print("\n[TEST] Sending CMD_SET_FAST_HZ(50)...")
        hz_payload = struct.pack('<H', 50)
        hz_cmd = build_cmd(CMD_SET_FAST_HZ, req_id=2, payload=hz_payload)
        rsp_packets.clear()
        await client.write_gatt_char(UUID_CMD, hz_cmd, response=False)
        await asyncio.sleep(0.5)

        for pkt in rsp_packets:
            r = parse_rsp(pkt)
            if r and r['opcode'] == CMD_SET_FAST_HZ and r['req_id'] == 2:
                if r['status'] == RSP_OK:
                    results['set_hz_ok'] = True
                    print("  [PASS] SET_FAST_HZ OK ✓")
                else:
                    print(f"  [FAIL] status={r['status']:#04x}")
                break
        else:
            print("  [FAIL] No response to SET_FAST_HZ!")

        # ── Step 6: Collect telemetry for 3 seconds ──
        print("\n[TEST] Collecting telemetry for 3 seconds...")
        fast_packets.clear()
        slow_packets.clear()
        await asyncio.sleep(3.0)

        results['fast_count'] = len(fast_packets)
        results['slow_count'] = len(slow_packets)
        results['rsp_count'] = len(rsp_packets)

        print(f"  FAST packets: {results['fast_count']}")
        print(f"  SLOW packets: {results['slow_count']}")
        print(f"  CRC OK/Fail:  {results['crc_ok']}/{results['crc_fail']}")

        # Show last FAST packet details
        if fast_packets:
            last = parse_fast(fast_packets[-1])
            if last:
                print(f"\n  Last FAST: t={last['t_ms']}ms vBat={last['vBat_mV']}mV "
                      f"vCmd=[{last['vCmdL_mV']},{last['vCmdR_mV']}] "
                      f"vel=[{last['velL_mmps']},{last['velR_mmps']}] "
                      f"gyroZ={last['gyroZ']} state={last['state']:#04x}")

        # ── Step 7: Stop notifications ──
        await client.stop_notify(UUID_RSP)
        await client.stop_notify(UUID_FAST)
        await client.stop_notify(UUID_SLOW)

    # ═════════════════════════════════════════════════════════════════════
    # Final Report
    # ═════════════════════════════════════════════════════════════════════
    print("\n" + "="*60)
    print("SMOKE TEST RESULTS")
    print("="*60)

    all_pass = True
    tests = [
        ("CAPS valid",        results['caps_ok']),
        ("PING response",     results['ping_ok']),
        ("SET_FAST_HZ",       results['set_hz_ok']),
        ("FAST packets ≥10",  results['fast_count'] >= 10),
        ("SLOW packets ≥1",   results['slow_count'] >= 1),
        ("CRC failures == 0", results['crc_fail'] == 0),
    ]

    for name, passed in tests:
        status = "PASS ✓" if passed else "FAIL ✗"
        if not passed:
            all_pass = False
        print(f"  {name:.<40s} {status}")

    print("="*60)
    if all_pass:
        print("ALL TESTS PASSED ✓ — Foundation is solid!")
    else:
        print("SOME TESTS FAILED — Check above for details")
    print("="*60)

    return all_pass


# ═══════════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Micromouse BLE Smoke Test")
    parser.add_argument("--addr", type=str, default=None,
                        help="BLE device address (e.g. AA:BB:CC:DD:EE:FF)")
    args = parser.parse_args()

    ok = asyncio.run(run_test(args.addr))
    sys.exit(0 if ok else 1)
