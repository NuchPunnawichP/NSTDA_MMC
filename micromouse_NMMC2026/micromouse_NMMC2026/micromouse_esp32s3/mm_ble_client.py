"""
mm_ble_client.py — BLE Client wrapper around Bleak
Handles connect/disconnect, notification dispatch, and async write.
"""
import asyncio
import threading
from typing import Callable, Optional
from bleak import BleakClient, BleakScanner
from mm_protocol import (
    UUID_SERVICE, UUID_CAPS, UUID_CMD, UUID_RSP, UUID_FAST, UUID_SLOW,
    PKT_SIZE, verify_crc, parse_caps
)


class MicromouseBLE:
    """
    BLE client for Micromouse robot.
    Runs Bleak event loop in a background thread.
    GUI calls send_cmd() which is thread-safe.
    """

    def __init__(self):
        self.client: Optional[BleakClient] = None
        self.connected = False
        self.caps = None

        # Callbacks set by GUI
        self.on_rsp:  Optional[Callable] = None   # (bytes)
        self.on_fast: Optional[Callable] = None   # (bytes)
        self.on_slow: Optional[Callable] = None   # (bytes)
        self.on_connect:    Optional[Callable] = None  # ()
        self.on_disconnect: Optional[Callable] = None  # ()

        # Async event loop in background thread
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def start(self):
        """Start the background asyncio event loop."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def _run_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def stop(self):
        """Stop background loop and disconnect."""
        if self._loop and self._running:
            asyncio.run_coroutine_threadsafe(self._async_disconnect(), self._loop)
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._running = False

    # ── Scanning ─────────────────────────────────────────────────────────

    def scan(self, timeout=5.0, callback=None):
        """Scan for MM_* devices. callback(list_of_devices) called when done."""
        if not self._loop:
            return
        fut = asyncio.run_coroutine_threadsafe(
            self._async_scan(timeout), self._loop
        )
        if callback:
            def on_done(f):
                try:
                    callback(f.result())
                except Exception:
                    callback([])
            fut.add_done_callback(on_done)

    async def _async_scan(self, timeout):
        devices = await BleakScanner.discover(timeout=timeout)
        return [d for d in devices if d.name and d.name.startswith("MM")]

    # ── Connect ──────────────────────────────────────────────────────────

    def connect(self, address: str, callback=None):
        """Connect to device. callback(success: bool) when done."""
        if not self._loop:
            return
        fut = asyncio.run_coroutine_threadsafe(
            self._async_connect(address), self._loop
        )
        if callback:
            def on_done(f):
                try:
                    callback(f.result())
                except Exception:
                    callback(False)
            fut.add_done_callback(on_done)

    async def _async_connect(self, address):
        try:
            def on_disconn(client):
                self.connected = False
                if self.on_disconnect:
                    self.on_disconnect()

            self.client = BleakClient(address, disconnected_callback=on_disconn)
            await self.client.connect()

            if not self.client.is_connected:
                return False

            # Read CAPS
            caps_data = await self.client.read_gatt_char(UUID_CAPS)
            self.caps = parse_caps(bytes(caps_data))

            # Subscribe to notifications
            await self.client.start_notify(UUID_RSP, self._on_rsp_raw)
            await self.client.start_notify(UUID_FAST, self._on_fast_raw)
            await self.client.start_notify(UUID_SLOW, self._on_slow_raw)

            self.connected = True
            if self.on_connect:
                self.on_connect()
            return True

        except Exception as e:
            print(f"[BLE] Connect error: {e}")
            self.connected = False
            return False

    # ── Disconnect ───────────────────────────────────────────────────────

    def disconnect(self, callback=None):
        if not self._loop:
            return
        fut = asyncio.run_coroutine_threadsafe(
            self._async_disconnect(), self._loop
        )
        if callback:
            fut.add_done_callback(lambda f: callback())

    async def _async_disconnect(self):
        if self.client and self.client.is_connected:
            try:
                await self.client.stop_notify(UUID_RSP)
                await self.client.stop_notify(UUID_FAST)
                await self.client.stop_notify(UUID_SLOW)
            except Exception:
                pass
            await self.client.disconnect()
        self.connected = False

    # ── Send CMD ─────────────────────────────────────────────────────────

    def send_cmd(self, data: bytes):
        """Thread-safe: send a 20-byte CMD packet."""
        if not self._loop or not self.connected:
            return
        asyncio.run_coroutine_threadsafe(
            self._async_write(data), self._loop
        )

    async def _async_write(self, data):
        if self.client and self.client.is_connected:
            try:
                await self.client.write_gatt_char(UUID_CMD, data, response=False)
            except Exception as e:
                print(f"[BLE] Write error: {e}")

    # ── Notification handlers ────────────────────────────────────────────

    def _on_rsp_raw(self, sender, data):
        data = bytes(data)
        if verify_crc(data) and self.on_rsp:
            self.on_rsp(data)

    def _on_fast_raw(self, sender, data):
        data = bytes(data)
        if verify_crc(data) and self.on_fast:
            self.on_fast(data)

    def _on_slow_raw(self, sender, data):
        data = bytes(data)
        if verify_crc(data) and self.on_slow:
            self.on_slow(data)
