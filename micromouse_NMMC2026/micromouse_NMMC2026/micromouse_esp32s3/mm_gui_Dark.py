#!/usr/bin/env python3
"""
mm_gui.py — Micromouse Debug GUI v2.0 (Modern Dark Theme)
Style matched to micromouse_test.py
Logic preserved from original mm_gui.py

Prerequisites:  pip install bleak customtkinter
Usage:          python mm_gui.py
"""
import customtkinter as ctk
import tkinter as tk # Still needed for variable tracing/IntVars
import struct, time, threading
from mm_protocol import *
from mm_ble_client import MicromouseBLE

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION & THEME
# ═══════════════════════════════════════════════════════════════════════════════
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("dark-blue")

# Palette from micromouse_test.py
COLOR_BG            = "#111827"        # Very Dark Blue/Gray
COLOR_CARD          = "#1F2937"        # Card Background
COLOR_INPUT         = "#374151"        # Input fields / Button normal
COLOR_TEXT          = "#E5E7EB"        # Light Gray
COLOR_TEXT_DIM      = "#9CA3AF"        # Dim Gray
COLOR_ACCENT        = "#34D399"        # Emerald Green/Cyan (Primary)
COLOR_ACCENT_HOVER  = "#059669"        
COLOR_RED           = "#EF4444"        # Danger / Stop
COLOR_RED_HOVER     = "#B91C1C"
COLOR_ORANGE        = "#F59E0B"        # Warning
COLOR_YELLOW        = "#FCD34D"
COLOR_BLUE          = "#3B82F6"        # Info

FONT_UI = ("Arial", 12)
FONT_BOLD = ("Arial", 12, "bold")
FONT_MONO = ("Consolas", 12)
FONT_HUD  = ("Consolas", 14, "bold")

# ═══════════════════════════════════════════════════════════════════════════════
# CUSTOM WIDGETS
# ═══════════════════════════════════════════════════════════════════════════════

class ModernSpinbox(ctk.CTkFrame):
    """Custom Spinbox to replace tk.Spinbox while keeping tk.IntVar logic"""
    def __init__(self, parent, variable, from_, to_, step, width=100):
        super().__init__(parent, fg_color="transparent")
        self.variable = variable
        self.step = step
        self.min_val = from_
        self.max_val = to_

        # Button -
        self.btn_sub = ctk.CTkButton(self, text="-", width=30, height=28,
                                     fg_color=COLOR_INPUT, text_color=COLOR_TEXT,
                                     command=self.decrement)
        self.btn_sub.pack(side="left", padx=(0, 5))

        # Value Display
        self.lbl_val = ctk.CTkEntry(self, width=width, height=28, justify="center",
                                    font=FONT_MONO)
        self.lbl_val.pack(side="left")
        self.lbl_val.insert(0, str(variable.get()))
        self.lbl_val.bind("<Return>", self.manual_entry)
        
        # Update entry when variable changes externally
        self.variable.trace_add("write", self.update_entry)

        # Button +
        self.btn_add = ctk.CTkButton(self, text="+", width=30, height=28,
                                     fg_color=COLOR_INPUT, text_color=COLOR_TEXT,
                                     command=self.increment)
        self.btn_add.pack(side="left", padx=(5, 0))

    def update_entry(self, *args):
        current = self.lbl_val.get()
        new_val = str(self.variable.get())
        if current != new_val:
            self.lbl_val.delete(0, "end")
            self.lbl_val.insert(0, new_val)

    def manual_entry(self, event=None):
        try:
            val = int(self.lbl_val.get())
            val = max(self.min_val, min(self.max_val, val))
            self.variable.set(val)
        except ValueError:
            self.update_entry() # Revert

    def increment(self):
        val = self.variable.get() + self.step
        if val <= self.max_val: self.variable.set(val)

    def decrement(self):
        val = self.variable.get() - self.step
        if val >= self.min_val: self.variable.set(val)

def make_card(parent, title):
    """Card container style"""
    frame = ctk.CTkFrame(parent, fg_color=COLOR_CARD, corner_radius=10)
    lbl = ctk.CTkLabel(frame, text=title, font=("Arial", 11, "bold"), text_color="gray")
    lbl.pack(pady=(10, 5), padx=15, anchor="w")
    content = ctk.CTkFrame(frame, fg_color="transparent")
    content.pack(fill="both", expand=True, padx=10, pady=(0, 10))
    return frame, content

# ═══════════════════════════════════════════════════════════════════════════════
# Lab 1 Tab — Hardware Check
# ═══════════════════════════════════════════════════════════════════════════════

class Lab1Tab:
    def __init__(self, parent, ble: MicromouseBLE):
        self.ble = ble
        self._pending = {}
        
        # Main layout: 2 Columns
        parent.grid_columnconfigure((0, 1), weight=1)
        parent.grid_rowconfigure(0, weight=1)

        left_col = ctk.CTkFrame(parent, fg_color="transparent")
        left_col.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        right_col = ctk.CTkFrame(parent, fg_color="transparent")
        right_col.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)

        # ════════════════════════════════════════════════════════════════
        # LEFT COLUMN — Controls
        # ════════════════════════════════════════════════════════════════

        # ── SAFETY CONTROL ──
        card_safety, content_safety = make_card(left_col, "SAFETY CONTROL")
        card_safety.pack(fill="x", pady=5)

        btn_row = ctk.CTkFrame(content_safety, fg_color="transparent")
        btn_row.pack(fill="x")
        
        ctk.CTkButton(btn_row, text="ARM", command=self.do_arm,
                      fg_color=COLOR_ACCENT, hover_color=COLOR_ACCENT_HOVER, 
                      text_color="black", width=80).pack(side="left", padx=2)
        ctk.CTkButton(btn_row, text="DISARM", command=self.do_disarm,
                      fg_color=COLOR_ORANGE, hover_color="#D97706",
                      text_color="black", width=80).pack(side="left", padx=2)
        ctk.CTkButton(btn_row, text="ABORT", command=self.do_abort,
                      fg_color=COLOR_RED, hover_color=COLOR_RED_HOVER,
                      text_color="white", width=80).pack(side="left", padx=2)

        self.state_lbl = ctk.CTkLabel(content_safety, text="[ IDLE ]", font=("Arial", 16, "bold"), text_color=COLOR_TEXT_DIM)
        self.state_lbl.pack(pady=(10, 0))

        # ── MOTOR TEST ──
        card_motor, content_motor = make_card(left_col, "MOTOR TEST [ARM required]")
        card_motor.pack(fill="x", pady=5)

        # Params
        param_row = ctk.CTkFrame(content_motor, fg_color="transparent")
        param_row.pack(fill="x", pady=5)
        
        ctk.CTkLabel(param_row, text="mV:", text_color=COLOR_ACCENT).pack(side="left")
        self.motor_mv = tk.IntVar(value=5000)
        ModernSpinbox(param_row, self.motor_mv, -8000, 8000, 500, width=70).pack(side="left", padx=5)
        
        ctk.CTkLabel(param_row, text="ms:", text_color=COLOR_ACCENT).pack(side="left", padx=(10,0))
        self.motor_dur = tk.IntVar(value=1000)
        ModernSpinbox(param_row, self.motor_dur, 100, 5000, 100, width=70).pack(side="left", padx=5)

        # Controls
        ctrl_row = ctk.CTkFrame(content_motor, fg_color="transparent")
        ctrl_row.pack(fill="x", pady=5)
        
        ctk.CTkButton(ctrl_row, text="◀ LEFT", width=70, command=lambda: self.do_motor(1)).pack(side="left", padx=2)
        ctk.CTkButton(ctrl_row, text="◀ BOTH ▶", width=70, fg_color=COLOR_ACCENT, text_color="black", hover_color=COLOR_ACCENT_HOVER, command=lambda: self.do_motor(0)).pack(side="left", padx=2)
        ctk.CTkButton(ctrl_row, text="RIGHT ▶", width=70, command=lambda: self.do_motor(2)).pack(side="left", padx=2)
        ctk.CTkButton(ctrl_row, text="■ STOP", width=60, fg_color=COLOR_RED, hover_color=COLOR_RED_HOVER, command=self.do_stop).pack(side="left", padx=2)

        # ── LED & BUZZER ──
        card_io, content_io = make_card(left_col, "LED & BUZZER")
        card_io.pack(fill="x", pady=5)

        led_row = ctk.CTkFrame(content_io, fg_color="transparent")
        led_row.pack(fill="x", pady=2)
        
        led_defs = [
            ("RED",   '#EF4444', 255, 0, 0),
            ("GRN",   '#22C55E', 0, 255, 0),
            ("BLUE",  '#3B82F6', 0, 0, 255),
            ("WHT",   '#F3F4F6', 255, 255, 255),
            ("OFF",   '#4B5563', 0, 0, 0),
        ]
        
        for txt, color, r, g, b in led_defs:
            fg = "white" if txt != "WHT" else "black"
            ctk.CTkButton(led_row, text=txt, width=50, height=24,
                          fg_color=color, hover_color=color, text_color=fg,
                          command=lambda r=r, g=g, b=b: self.do_led(r, g, b)).pack(side="left", padx=2)

        buz_row = ctk.CTkFrame(content_io, fg_color="transparent")
        buz_row.pack(fill="x", pady=(5,0))
        for freq, dur, lbl in [(1000, 200, "1kHz"), (2000, 100, "2kHz"), (500, 500, "500Hz")]:
            ctk.CTkButton(buz_row, text=f"♪ {lbl}", width=80, height=24,
                          fg_color="#8B5CF6", hover_color="#7C3AED",
                          command=lambda f=freq, d=dur: self.do_buzzer(f, d)).pack(side="left", padx=2)

        # ── SENSOR READOUT (Request) ──
        card_req, content_req = make_card(left_col, "MANUAL READ [No ARM needed]")
        card_req.pack(fill="x", pady=5)
        
        req_row = ctk.CTkFrame(content_req, fg_color="transparent")
        req_row.pack(fill="x")
        
        req_btns = [("⚡ BATT", self.do_read_battery), ("◈ WALLS", self.do_read_sensors),
                    ("◎ IMU", self.do_read_imu), ("⊞ INP", self.do_read_buttons)]
        for txt, cmd in req_btns:
            ctk.CTkButton(req_row, text=txt, width=65, command=cmd, fg_color=COLOR_INPUT).pack(side="left", padx=2)

        self.readout = ctk.CTkTextbox(content_req, height=100, font=FONT_MONO, text_color=COLOR_ACCENT)
        self.readout.pack(fill="x", pady=(10,0))
        self.readout.configure(state="disabled")

        # ════════════════════════════════════════════════════════════════
        # RIGHT COLUMN — Live Telemetry
        # ════════════════════════════════════════════════════════════════

        # ── POWER STATUS ──
        card_pwr, content_pwr = make_card(right_col, "POWER STATUS")
        card_pwr.pack(fill="x", pady=5)

        self.batt_lbl = ctk.CTkLabel(content_pwr, text="-.-- V", font=("Arial", 32, "bold"), text_color=COLOR_ACCENT)
        self.batt_lbl.pack()
        
        self.batt_bar = ctk.CTkProgressBar(content_pwr, height=10, progress_color=COLOR_ACCENT)
        self.batt_bar.pack(fill="x", pady=5)
        self.batt_bar.set(0)
        
        self.batt_detail = ctk.CTkLabel(content_pwr, text="Waiting for data...", text_color=COLOR_TEXT_DIM, font=("Arial", 10))
        self.batt_detail.pack()

        # ── FAST TELEMETRY ──
        card_fast, content_fast = make_card(right_col, "MOTION TELEMETRY")
        card_fast.pack(fill="both", expand=True, pady=5)

        self.fast_txt = ctk.CTkTextbox(content_fast, height=180, font=FONT_MONO, text_color=COLOR_TEXT, fg_color="#111827", corner_radius=5)
        self.fast_txt.pack(fill="both", expand=True)
        self.fast_txt.configure(state="disabled")

        # ── DIAGNOSTICS ──
        card_diag, content_diag = make_card(right_col, "SYSTEM DIAGNOSTICS")
        card_diag.pack(fill="x", pady=5)

        self.slow_txt = ctk.CTkTextbox(content_diag, height=80, font=FONT_MONO, text_color="#E879F9", fg_color="#111827")
        self.slow_txt.pack(fill="x")
        self.slow_txt.configure(state="disabled")

        # ── PACKET COUNTER ──
        self.pkt_lbl = ctk.CTkLabel(right_col, text="PKT  FAST:0  SLOW:0  RSP:0", font=FONT_MONO, text_color=COLOR_TEXT_DIM)
        self.pkt_lbl.pack(pady=5, anchor="e")

        self.fast_count = self.slow_count = self.rsp_count = 0

    # ── Actions (Logic Preserved) ────────────────────────────────────
    def do_arm(self):     self.ble.send_cmd(cmd_arm())
    def do_disarm(self):  self.ble.send_cmd(cmd_disarm())
    def do_abort(self):   self.ble.send_cmd(cmd_abort())
    def do_stop(self):    self.ble.send_cmd(cmd_stop())
    def do_motor(self, sel):
        self.ble.send_cmd(cmd_motor_spin(self.motor_mv.get(), self.motor_dur.get(),
                                          sel, req_id=200 + sel))
    def do_led(self, r, g, b): self.ble.send_cmd(cmd_led_set(r, g, b))
    def do_buzzer(self, f, d): self.ble.send_cmd(cmd_buzzer(f, d))

    def do_read_battery(self):
        self._pending[300] = self._show_battery
        self.ble.send_cmd(cmd_read_battery(req_id=300))
    def do_read_sensors(self):
        self._pending[301] = self._show_sensors
        self.ble.send_cmd(cmd_read_sensors(req_id=301))
    def do_read_imu(self):
        self._pending[302] = self._show_imu
        self.ble.send_cmd(cmd_read_imu(req_id=302))
    def do_read_buttons(self):
        self._pending[303] = self._show_buttons
        self.ble.send_cmd(cmd_read_buttons(req_id=303))

    # ── RSP Handlers (Logic Preserved) ───────────────────────────────
    def handle_rsp(self, data):
        self.rsp_count += 1
        rsp = parse_rsp(data)
        if not rsp: return
        cb = self._pending.pop(rsp['req_id'], None)
        if cb: cb(rsp)

    def _wr(self, txt):
        self.readout.configure(state="normal")
        self.readout.delete("1.0", "end")
        self.readout.insert("1.0", txt)
        self.readout.configure(state="disabled")

    def _show_battery(self, r):
        if r['status'] != RSP_OK:
            self._wr(f" ✖ ERROR: {RSP_NAMES.get(r['status'], '?')}"); return
        b = parse_rsp_battery(r['payload'])
        lvl_name = ['▪ CRITICAL', '▪ LOW', '▪ NOMINAL', '▪ FULL'][b['level']]
        self._wr(f" ⚡ Battery Report\n"
                 f"   Raw:   {b['vBat_raw_mV']} mV\n"
                 f"   Avg:   {b['vBat_avg_mV']} mV\n"
                 f"   Level: {lvl_name}")

    def _show_sensors(self, r):
        if r['status'] != RSP_OK:
            self._wr(f" ✖ ERROR: {RSP_NAMES.get(r['status'], '?')}"); return
        s = parse_rsp_sensors(r['payload'])
        bm = s['bitmap']
        wall = lambda b, i: '█' if (b >> i) & 1 else '░'
        self._wr(f" ◈ Wall Sensors\n"
                 f"   LEFT:    {s['left_mm']:4d} mm  {wall(bm,3)}\n"
                 f"   FRONT-L: {s['fl_mm']:4d} mm  {wall(bm,2)}\n"
                 f"   FRONT-R: {s['fr_mm']:4d} mm  {wall(bm,1)}\n"
                 f"   RIGHT:   {s['right_mm']:4d} mm  {wall(bm,0)}")

    def _show_imu(self, r):
        if r['status'] != RSP_OK:
            self._wr(f" ✖ ERROR: {RSP_NAMES.get(r['status'], '?')}"); return
        m = parse_rsp_imu(r['payload'])
        self._wr(f" ◎ IMU Raw Data\n"
                 f"   GYRO:  X={m['gyroX']:+6d}  Y={m['gyroY']:+6d}  Z={m['gyroZ']:+6d}\n"
                 f"   ACCEL: X={m['accelX']:+6d}  Y={m['accelY']:+6d}  Z={m['accelZ']:+6d}")

    def _show_buttons(self, r):
        if r['status'] != RSP_OK:
            self._wr(f" ✖ ERROR: {RSP_NAMES.get(r['status'], '?')}"); return
        b = parse_rsp_buttons(r['payload'])
        self._wr(f" ⊞ Input Status\n"
                 f"   START: {'▶ ACTIVE' if b['start'] else '○ idle'}\n"
                 f"   MODE:  {'▶ ACTIVE' if b['mode'] else '○ idle'}\n"
                 f"   DIP:   [{b['dip']:04b}] = {b['dip']}\n"
                 f"   ENC L: {b['encL']:+d}   ENC R: {b['encR']:+d}")

    # ── Live Telemetry (Logic Preserved, Updated UI) ────────────────
    def handle_fast(self, data):
        self.fast_count += 1
        f = parse_fast(data)
        if not f: return

        # Battery
        v = f['vBat_mV']
        volts = v / 1000.0
        self.batt_lbl.configure(text=f"{volts:.2f} V")
        self.batt_bar.set(min(1.0, v / 8400.0))

        # Colors
        if v < 6400:   self.batt_lbl.configure(text_color=COLOR_RED)
        elif v < 7000: self.batt_lbl.configure(text_color=COLOR_YELLOW)
        else:          self.batt_lbl.configure(text_color=COLOR_ACCENT)
        
        self.batt_detail.configure(text=f"{v} mV  |  {volts:.2f}V  |  2S LiPo")

        # State label
        st = f['state']
        st_text = state_str(st)
        if st & FLAG_ARMED and st & FLAG_RUNNING:
            self.state_lbl.configure(text=f"◆ {st_text} ◆", text_color="#E879F9") # Magenta
        elif st & FLAG_ARMED:
            self.state_lbl.configure(text=f"◆ {st_text}", text_color=COLOR_ACCENT)
        elif st & FLAG_LOW_BATT:
            self.state_lbl.configure(text=f"⚠ {st_text}", text_color=COLOR_RED)
        else:
            self.state_lbl.configure(text=f"[ {st_text} ]", text_color=COLOR_TEXT_DIM)

        # FAST text
        if self.fast_count % 5 == 0:
            gz = f['gyroZ_dps10'] / 10.0
            txt = (
                f"  ┌─ MOTOR CMD ────────────────────┐\n"
                f"  │  L: {f['vCmdL_mV']:+6d} mV   R: {f['vCmdR_mV']:+6d} mV │\n"
                f"  ├─ VELOCITY ─────────────────────┤\n"
                f"  │  L: {f['velL_mmps']:+6d} mm/s R: {f['velR_mmps']:+6d} mm/s │\n"
                f"  ├─ GYRO / ENCODER ───────────────┤\n"
                f"  │  GyroZ: {gz:+8.1f} °/s            │\n"
                f"  │  ΔEnc:  L={f['encL_delta']:+4d}  R={f['encR_delta']:+4d}       │\n"
                f"  └────────────── t={f['t_ms']:5d}ms ────┘"
            )
            self.fast_txt.configure(state="normal")
            self.fast_txt.delete("1.0", "end")
            self.fast_txt.insert("1.0", txt)
            self.fast_txt.configure(state="disabled")

    def handle_slow(self, data):
        self.slow_count += 1
        s = parse_slow(data)
        if not s: return
        txt = (
            f"  MISSED:{s['missed_ticks']:3d}  STEP_MAX:{s['step_max_us']:4d}µs"
            f"  WD:{s['wd_age_ms']:5d}ms\n"
            f"  RX_DROP:{s['rx_drop']}  CRC_FAIL:{s['crc_fail']}"
            f"  LAST:0x{s['last_cmd_op']:02X}→{RSP_NAMES.get(s['last_status'],'?')}\n"
            f"  VBAT_AVG:{s['vBat_avg_mV']}mV  FLAGS:{s['active_flags']:08b}"
        )
        self.slow_txt.configure(state="normal")
        self.slow_txt.delete("1.0", "end")
        self.slow_txt.insert("1.0", txt)
        self.slow_txt.configure(state="disabled")

    def update_counters(self):
        self.pkt_lbl.configure(
            text=f"PKT  FAST:{self.fast_count}  SLOW:{self.slow_count}  RSP:{self.rsp_count}")


# ═══════════════════════════════════════════════════════════════════════════════
# Main Application
# ═══════════════════════════════════════════════════════════════════════════════

class App:
    def __init__(self):
        self.root = ctk.CTk()
        self.root.title("MICROMOUSE // DEBUG TERMINAL v2.0")
        self.root.geometry("1024x720")
        
        self.ble = MicromouseBLE()
        self.ble.start()

        # ── TOP BAR — Connection HUD ──
        # Sidebar-like top bar to match the style better
        top_frame = ctk.CTkFrame(self.root, fg_color=COLOR_CARD, corner_radius=0)
        top_frame.pack(fill="x", pady=(0, 5))
        
        # Title
        ctk.CTkLabel(top_frame, text="MICROMOUSE", font=("Arial", 18, "bold"), text_color=COLOR_ACCENT).pack(side="left", padx=20, pady=15)
        ctk.CTkLabel(top_frame, text="// DEBUG TERMINAL", font=("Arial", 14), text_color=COLOR_TEXT_DIM).pack(side="left", pady=15)

        # Controls
        ctrl_frame = ctk.CTkFrame(top_frame, fg_color="transparent")
        ctrl_frame.pack(side="right", padx=20)

        self.status_lbl = ctk.CTkLabel(ctrl_frame, text="● OFFLINE", font=("Arial", 12, "bold"), text_color=COLOR_RED)
        self.status_lbl.pack(side="right", padx=10)

        self.disc_btn = ctk.CTkButton(ctrl_frame, text="DISCONNECT", command=self.do_disconnect,
                                      fg_color=COLOR_RED, hover_color=COLOR_RED_HOVER, width=100)
        self.disc_btn.pack(side="right", padx=5)
        self.disc_btn.configure(state="disabled")

        self.conn_btn = ctk.CTkButton(ctrl_frame, text="CONNECT", command=self.do_connect,
                                      fg_color=COLOR_ACCENT, text_color="black", hover_color=COLOR_ACCENT_HOVER, width=100)
        self.conn_btn.pack(side="right", padx=5)

        self.dev_var = ctk.StringVar(value="...")
        self.dev_combo = ctk.CTkOptionMenu(ctrl_frame, variable=self.dev_var, values=[], width=200, fg_color=COLOR_INPUT)
        self.dev_combo.pack(side="right", padx=5)

        self.scan_btn = ctk.CTkButton(ctrl_frame, text="🔍 SCAN", command=self.do_scan,
                                      fg_color=COLOR_INPUT, width=90)
        self.scan_btn.pack(side="right", padx=5)

        # ── TABVIEW (Replaces Notebook) ──
        self.tabview = ctk.CTkTabview(self.root, fg_color="transparent")
        self.tabview.pack(fill="both", expand=True, padx=10, pady=0)

        tab_names = ["LAB 1 HW", "LAB 2 ENC", "LAB 3 PROF", "LAB 4 PID", "LAB 5 SRCH", "LAB 6 RUN"]
        for name in tab_names:
            self.tabview.add(name)

        # Init Lab 1
        self.lab1 = Lab1Tab(self.tabview.tab("LAB 1 HW"), self.ble)

        # Placeholders for others
        for name in tab_names[1:]:
            f = self.tabview.tab(name)
            ctk.CTkLabel(f, text=f"[ {name} ]\n\n◇ COMING SOON ◇",
                         font=("Arial", 20, "bold"), text_color=COLOR_TEXT_DIM).pack(expand=True)

        # ── BLE callbacks (Logic Preserved) ──
        self.ble.on_rsp = lambda d: self.root.after(0, lambda: self.lab1.handle_rsp(d))
        self.ble.on_fast = lambda d: self.root.after(0, lambda: self.lab1.handle_fast(d))
        self.ble.on_slow = lambda d: self.root.after(0, lambda: self.lab1.handle_slow(d))
        self.ble.on_connect = lambda: self.root.after(0, self._ui_conn)
        self.ble.on_disconnect = lambda: self.root.after(0, self._ui_disc)

        self._devs = []
        self._ka_id = None
        self._tick()
        self.root.protocol("WM_DELETE_WINDOW", self._close)

    # ── UI State (Updated for CTK) ───────────────────────────────────

    def _ui_conn(self):
        self.status_lbl.configure(text="● ONLINE", text_color=COLOR_ACCENT)
        self.conn_btn.configure(state="disabled")
        self.disc_btn.configure(state="normal")
        self.scan_btn.configure(state="disabled")
        self.ble.send_cmd(cmd_set_fast_hz(50))
        self.ble.send_cmd(cmd_set_slow_hz(5))
        self._ka_start()

    def _ui_disc(self):
        self.status_lbl.configure(text="● OFFLINE", text_color=COLOR_RED)
        self.conn_btn.configure(state="normal")
        self.disc_btn.configure(state="disabled")
        self.scan_btn.configure(state="normal")
        self._ka_stop()

    # ── Keepalive PING (Logic Preserved) ─────────────────────────────

    def _ka_start(self):
        self._ka_stop()
        self._ka_ping()

    def _ka_ping(self):
        if self.ble.connected:
            self.ble.send_cmd(cmd_ping(req_id=9999))
            self._ka_id = self.root.after(3000, self._ka_ping)

    def _ka_stop(self):
        if self._ka_id:
            self.root.after_cancel(self._ka_id)
            self._ka_id = None

    # ── Actions (Logic Preserved) ────────────────────────────────────

    def do_scan(self):
        self.scan_btn.configure(state="disabled")
        self.status_lbl.configure(text="● SCANNING...", text_color=COLOR_ORANGE)
        self.ble.scan(timeout=5.0,
                      callback=lambda d: self.root.after(0, lambda: self._show_scan(d)))

    def _show_scan(self, devs):
        self._devs = devs
        # Update combo values
        dev_list = [f"{d.name} ({d.address})" for d in devs]
        if not dev_list: dev_list = ["No devices found"]
        
        self.dev_combo.configure(values=dev_list)
        if devs:
            self.dev_combo.set(dev_list[0])
            self.dev_var.set(dev_list[0])
            
        self.scan_btn.configure(state="normal")
        status_txt = f"● FOUND {len(devs)}"
        self.status_lbl.configure(text=status_txt, text_color=COLOR_BLUE)

    def do_connect(self):
        # Find selected device object
        val = self.dev_var.get()
        idx = -1
        for i, d in enumerate(self._devs):
            if val.startswith(d.name):
                idx = i
                break
        
        if idx < 0: return
        self.status_lbl.configure(text="● LINKING...", text_color=COLOR_YELLOW)
        self.conn_btn.configure(state="disabled")
        self.ble.connect(self._devs[idx].address)

    def do_disconnect(self):
        self._ka_stop()
        self.ble.disconnect()

    def _tick(self):
        self.lab1.update_counters()
        self.root.after(200, self._tick)

    def _close(self):
        self._ka_stop()
        self.ble.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    App().run()