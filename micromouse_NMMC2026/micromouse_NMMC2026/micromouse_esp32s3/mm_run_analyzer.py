#!/usr/bin/env python3
"""
mm_run_analyzer.py — Run Analysis Engine for Micromouse PID Tuning

Features:
  A. Run lifecycle (IDLE -> RUNNING -> FROZEN)
  B. Snapshot of gains/params per run
  C. Auto metrics (rise time, overshoot, settling time, SSE)
  D. Trapezoid profile model (GUI-side target generation)
  E. Export (CSV + JSON metadata) & Replay
  F. Run history with compare capability
"""

import math, json, csv, os, time
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Optional, Tuple

DT_S = 0.020
FLAG_ARMED   = 0x10
FLAG_RUNNING = 0x20

# =========================================================================
# Trapezoid Profile Model
# =========================================================================
@dataclass
class TrapProfile:
    distance_mm: float
    vel_max: float
    accel: float
    t_acc: float = 0
    t_cruise: float = 0
    t_dec: float = 0
    v_peak: float = 0
    is_triangle: bool = False

    def __post_init__(self):
        d = abs(self.distance_mm)
        v = abs(self.vel_max)
        a = abs(self.accel) if self.accel > 0 else 500
        sign = 1 if self.distance_mm >= 0 else -1
        self.t_acc = v / a if a > 0 else 0
        d_acc = 0.5 * a * self.t_acc ** 2
        if 2 * d_acc >= d and d > 0:
            self.is_triangle = True
            self.t_acc = math.sqrt(d / a) if a > 0 else 0
            self.v_peak = a * self.t_acc * sign
            self.t_cruise = 0
            self.t_dec = self.t_acc
        else:
            self.is_triangle = False
            self.v_peak = v * sign
            d_cruise = d - 2 * d_acc
            self.t_cruise = d_cruise / v if v > 0 else 0
            self.t_dec = self.t_acc

    @property
    def total_time(self):
        return self.t_acc + self.t_cruise + self.t_dec

    def sample_at(self, t):
        sign = 1 if self.distance_mm >= 0 else -1
        a = abs(self.accel) if self.accel > 0 else 500
        v_peak = abs(self.v_peak)
        if t < 0: return 0
        elif t < self.t_acc: return sign * a * t
        elif t < self.t_acc + self.t_cruise: return sign * v_peak
        elif t < self.total_time:
            dt = t - self.t_acc - self.t_cruise
            return sign * max(0, v_peak - a * dt)
        return 0

    def sample_array(self, dt=DT_S, pad_before=2, pad_after=5):
        n = int(self.total_time / dt) + pad_before + pad_after + 2
        return [self.sample_at((i - pad_before) * dt) for i in range(n)]

    @property
    def phases(self):
        t1 = self.t_acc
        t2 = t1 + self.t_cruise
        t3 = self.total_time
        return [(s, e, n) for s, e, n in [(0, t1, 'ACC'), (t1, t2, 'CRUISE'),
                (t2, t3, 'DECEL')] if e > s]


# =========================================================================
# Run Snapshot
# =========================================================================
@dataclass
class RunSnapshot:
    run_id: int = 0
    timestamp: float = 0
    mode: str = ''
    lab: int = 3
    distance_mm: int = 0
    vel_mmps: int = 0
    accel_mmps2: int = 0
    vel_kp: int = 0; vel_kd: int = 0; vel_max_mv: int = 0
    hdg_kp: int = 0; hdg_ki: int = 0; hdg_kd: int = 0; hdg_max_mv: int = 0
    trim_mv: int = 0

    def to_dict(self): return asdict(self)

    def gains_str(self):
        if self.lab == 3: return "FF only"
        return (f"V: Kp={self.vel_kp/100:.2f} Kd={self.vel_kd/100:.2f} max={self.vel_max_mv}mV\n"
                f"H: Kp={self.hdg_kp/100:.2f} Ki={self.hdg_ki/100:.2f} "
                f"Kd={self.hdg_kd/100:.2f} max={self.hdg_max_mv}mV  Trim={self.trim_mv:+d}mV")

    def params_str(self):
        return f"{self.mode.upper()} {abs(self.distance_mm)}mm {self.vel_mmps}mm/s acc={self.accel_mmps2}"

    def filename_stem(self):
        parts = [f"run{self.run_id:04d}", f"L{self.lab}", self.mode,
                 f"v{self.vel_mmps}", f"a{self.accel_mmps2}", f"d{abs(self.distance_mm)}"]
        if self.lab == 4:
            parts += [f"Kp{self.vel_kp}", f"Kd{self.vel_kd}"]
        return "_".join(parts)


# =========================================================================
# Run Metrics
# =========================================================================
@dataclass
class RunMetrics:
    rise_time_ms: float = 0
    overshoot_pct: float = 0
    settling_time_ms: float = 0
    sse_mmps: float = 0
    peak_vel_L: float = 0; peak_vel_R: float = 0
    lr_bias_mmps: float = 0; lr_std_mmps: float = 0
    yaw_drift_deg: float = 0; peak_gyro_dps: float = 0; gyro_bias_dps: float = 0
    tracking_err_mmps: float = 0; tracking_std_mmps: float = 0
    total_time_ms: float = 0; n_samples: int = 0
    i_motion_start: int = 0; i_cruise_start: int = 0
    i_cruise_end: int = 0; i_motion_end: int = 0
    target_vel: float = 0

    def summary_str(self):
        lines = (f"  T {self.n_samples} samples  {self.total_time_ms:.0f}ms  "
                f"target={self.target_vel:.0f}mm/s\n"
                f"  Rise: {self.rise_time_ms:.0f}ms  OS: {self.overshoot_pct:+.1f}%  "
                f"Settle: {self.settling_time_ms:.0f}ms\n"
                f"  SSE: {self.sse_mmps:+.1f}mm/s  Peak: L={self.peak_vel_L:.0f} R={self.peak_vel_R:.0f}\n"
                f"  L-R: bias={self.lr_bias_mmps:+.1f} σ={self.lr_std_mmps:.1f}mm/s\n"
                f"  Yaw: d={self.yaw_drift_deg:+.2f}°  peak={self.peak_gyro_dps:.1f}°/s")
        if self.tracking_err_mmps != 0 or self.tracking_std_mmps != 0:
            lines += f"\n  Track: err={self.tracking_err_mmps:+.1f} σ={self.tracking_std_mmps:.1f}mm/s (FW)"
        return lines

    def grade(self):
        sc = 100
        if self.overshoot_pct > 20: sc -= 30
        elif self.overshoot_pct > 10: sc -= 15
        elif self.overshoot_pct > 5: sc -= 5
        if self.settling_time_ms > 500: sc -= 25
        elif self.settling_time_ms > 300: sc -= 15
        elif self.settling_time_ms > 150: sc -= 5
        if abs(self.sse_mmps) > 10: sc -= 20
        elif abs(self.sse_mmps) > 5: sc -= 10
        if self.lr_std_mmps > 15: sc -= 15
        elif self.lr_std_mmps > 8: sc -= 5
        if abs(self.yaw_drift_deg) > 5: sc -= 15
        elif abs(self.yaw_drift_deg) > 2: sc -= 5
        sc = max(0, sc)
        g = 'A' if sc >= 90 else 'B' if sc >= 75 else 'C' if sc >= 60 else 'D' if sc >= 40 else 'F'
        return g, sc


def compute_metrics(samples, profile=None, target_vel=0):
    m = RunMetrics()
    n = len(samples)
    if n < 5: return m
    m.n_samples = n
    m.total_time_ms = n * DT_S * 1000
    vL = [s.get('velL_mmps', 0) for s in samples]
    vR = [s.get('velR_mmps', 0) for s in samples]
    vAvg = [(l + r) / 2 for l, r in zip(vL, vR)]
    gyro = [s.get('gyroZ_dps10', 0) / 10.0 for s in samples]

    # ── Firmware v_target (preferred) vs GUI trapezoid (fallback) ──
    fw_vt = [s.get('v_target_mmps', 0) for s in samples]
    has_fw_target = any(abs(v) > 1 for v in fw_vt)

    if target_vel == 0 and profile:
        target_vel = abs(profile.v_peak)
    if target_vel == 0:
        target_vel = max((abs(v) for v in vAvg), default=1)
    m.target_vel = target_vel
    sign = 1 if sum(vAvg) >= 0 else -1
    vS = [v * sign for v in vAvg]
    th_mov = target_vel * 0.05
    th_crz = target_vel * 0.80

    # ── Phase detection from firmware v_target ──
    if has_fw_target:
        fwS = [v * sign for v in fw_vt]
        # Derive phase: compare consecutive v_target values
        m.phases = []  # list of (start_idx, end_idx, 'ACC'|'CRUISE'|'DECEL')
        for i in range(n):
            if abs(fwS[i]) < 2:
                pass  # idle
            elif i > 0 and fwS[i] > fwS[i-1] + 0.5:
                if not m.phases or m.phases[-1][2] != 'ACC':
                    m.phases.append((i, i, 'ACC'))
                else:
                    m.phases[-1] = (m.phases[-1][0], i, 'ACC')
            elif i > 0 and fwS[i] < fwS[i-1] - 0.5:
                if not m.phases or m.phases[-1][2] != 'DECEL':
                    m.phases.append((i, i, 'DECEL'))
                else:
                    m.phases[-1] = (m.phases[-1][0], i, 'DECEL')
            elif abs(fwS[i]) > th_crz:
                if not m.phases or m.phases[-1][2] != 'CRUISE':
                    m.phases.append((i, i, 'CRUISE'))
                else:
                    m.phases[-1] = (m.phases[-1][0], i, 'CRUISE')

    m.i_motion_start = next((i for i, v in enumerate(vS) if v > th_mov), 0)
    m.i_cruise_start = next((i for i in range(m.i_motion_start, n) if vS[i] > th_crz), m.i_motion_start)
    m.i_cruise_end = m.i_cruise_start
    for i in range(n - 1, m.i_cruise_start, -1):
        if vS[i] > th_crz:
            m.i_cruise_end = i; break
    m.i_motion_end = next((i for i in range(n-1, 0, -1) if vS[i] > th_mov), n-1)
    m.peak_vel_L = max((abs(v) for v in vL), default=0)
    m.peak_vel_R = max((abs(v) for v in vR), default=0)
    peak_avg = max(vS) if vS else 0

    # ── Use firmware target for precise overshoot calc ──
    if has_fw_target:
        # Peak of firmware target during cruise
        fw_cruise = [abs(fw_vt[i]) for i in range(m.i_cruise_start, m.i_cruise_end+1)] if m.i_cruise_end > m.i_cruise_start else []
        if fw_cruise:
            target_vel = sum(fw_cruise) / len(fw_cruise)
            m.target_vel = target_vel

    v10, v90 = target_vel * 0.10, target_vel * 0.90
    t10 = next((i for i in range(m.i_motion_start, n) if vS[i] >= v10), -1)
    t90 = next((i for i in range(max(t10, 0), n) if vS[i] >= v90), -1)
    if t10 >= 0 and t90 >= 0:
        m.rise_time_ms = (t90 - t10) * DT_S * 1000
    if target_vel > 0:
        m.overshoot_pct = (peak_avg - target_vel) / target_vel * 100
    band = target_vel * 0.05
    for i in range(m.i_motion_start, min(m.i_cruise_end + 1, n)):
        if abs(vS[i] - target_vel) <= band:
            if all(abs(vS[j] - target_vel) <= band for j in range(i, min(i + 5, n))):
                m.settling_time_ms = (i - m.i_motion_start) * DT_S * 1000
                break
    cs, ce = m.i_cruise_start, m.i_cruise_end
    if ce > cs + 2:
        cv = vS[cs:ce+1]
        m.sse_mmps = (sum(cv)/len(cv) - target_vel) * sign

        # ── Tracking error (using firmware v_target if available) ──
        if has_fw_target:
            tracking = [abs(vAvg[i]) - abs(fw_vt[i]) for i in range(cs, ce+1)]
            m.tracking_err_mmps = sum(tracking) / len(tracking)
            m.tracking_std_mmps = (sum((e - m.tracking_err_mmps)**2 for e in tracking) / len(tracking))**0.5 if len(tracking) > 1 else 0

        lr = [vL[i] - vR[i] for i in range(cs, ce+1)]
        m.lr_bias_mmps = sum(lr)/len(lr)
        m.lr_std_mmps = (sum((d-m.lr_bias_mmps)**2 for d in lr)/len(lr))**0.5 if len(lr) > 1 else 0
        cg = gyro[cs:ce+1]
        m.gyro_bias_dps = sum(cg)/len(cg) if cg else 0
        m.yaw_drift_deg = sum(g * DT_S for g in cg)
    m.peak_gyro_dps = max((abs(g) for g in gyro), default=0)
    return m


# =========================================================================
# Run State Machine
# =========================================================================
class RunState:
    IDLE = 'IDLE'; RUNNING = 'RUNNING'; FROZEN = 'FROZEN'


class RunRecord:
    def __init__(self, snap):
        self.snap = snap
        self.samples = []
        self.metrics = None
        self.profile = None
        self.target_array = []

    def compute(self):
        if self.snap.distance_mm != 0 and self.snap.vel_mmps != 0:
            self.profile = TrapProfile(
                distance_mm=self.snap.distance_mm,
                vel_max=self.snap.vel_mmps,
                accel=self.snap.accel_mmps2 or 500)
            self.target_array = self.profile.sample_array(DT_S)
        self.metrics = compute_metrics(self.samples, self.profile, self.snap.vel_mmps)

    def export_csv(self, path):
        keys = ['t_ms', 'state', 'vCmdL_mV', 'vCmdR_mV', 'velL_mmps', 'velR_mmps',
                'v_target_mmps', 'gyroZ_dps10', 'encL_delta', 'encR_delta']
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
            w.writeheader()
            for r in self.samples: w.writerow(r)

    def export_json(self, path):
        meta = {'snapshot': self.snap.to_dict(),
                'metrics': asdict(self.metrics) if self.metrics else {},
                'n_samples': len(self.samples),
                'grade': self.metrics.grade()[0] if self.metrics else '?'}
        with open(path, 'w') as f: json.dump(meta, f, indent=2)

    def export_both(self, directory):
        os.makedirs(directory, exist_ok=True)
        stem = self.snap.filename_stem()
        cp = os.path.join(directory, f"{stem}.csv")
        jp = os.path.join(directory, f"{stem}.json")
        self.export_csv(cp); self.export_json(jp)
        return cp, jp


class RunManager:
    def __init__(self):
        self.state = RunState.IDLE
        self.current = None
        self.history = []
        self._run_counter = 0
        self._zero_count = 0
        self._min_samples = 10
        self.ZERO_THRESHOLD = 8

    def begin_run(self, snap):
        if self.state == RunState.FROZEN and self.current:
            self.history.append(self.current)
            if len(self.history) > 50:
                self.history = self.history[-50:]
        self._run_counter += 1
        snap.run_id = self._run_counter
        snap.timestamp = time.time()
        self.current = RunRecord(snap)
        self.state = RunState.RUNNING
        self._zero_count = 0
        return self.current

    def add_sample(self, f_data):
        if self.state == RunState.RUNNING and self.current:
            self.current.samples.append(f_data)

    def check_auto_freeze(self, f_data):
        if self.state != RunState.RUNNING or not self.current:
            return False
        if len(self.current.samples) < self._min_samples:
            return False
        cmd_zero = abs(f_data.get('vCmdL_mV', 999)) < 50 and abs(f_data.get('vCmdR_mV', 999)) < 50
        vel_zero = abs(f_data.get('velL_mmps', 999)) < 8 and abs(f_data.get('velR_mmps', 999)) < 8
        st = f_data.get('state', 0)
        state_stop = not (st & FLAG_RUNNING) and (st & FLAG_ARMED)
        if (cmd_zero and vel_zero) or state_stop:
            self._zero_count += 1
        else:
            self._zero_count = 0
        if self._zero_count >= self.ZERO_THRESHOLD:
            self.freeze()
            return True
        return False

    def freeze(self):
        if self.state != RunState.RUNNING or not self.current:
            return
        self.state = RunState.FROZEN
        s = self.current.samples
        if len(s) > 10:
            last = len(s) - 1
            for i in range(len(s)-1, -1, -1):
                if abs(s[i].get('vCmdL_mV', 0)) > 50 or abs(s[i].get('velL_mmps', 0)) > 5:
                    last = i; break
            self.current.samples = s[:min(last + 5, len(s))]
        self.current.compute()

    def clear(self):
        self.state = RunState.IDLE
        self.current = None
        self._zero_count = 0

    def get_metrics(self):
        if self.current and self.current.metrics:
            return self.current.metrics
        return None

    def get_snapshot(self):
        if self.current:
            return self.current.snap
        return None


# =========================================================================
# Replay
# =========================================================================
def load_run(csv_path, json_path=None):
    if not os.path.exists(csv_path): return None
    snap = RunSnapshot()
    if json_path and os.path.exists(json_path):
        with open(json_path) as f:
            sd = json.load(f).get('snapshot', {})
        for k, v in sd.items():
            if hasattr(snap, k): setattr(snap, k, v)
    rec = RunRecord(snap)
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            sample = {}
            for k, v in row.items():
                try: sample[k] = int(v)
                except (ValueError, TypeError): sample[k] = v
            rec.samples.append(sample)
    rec.compute()
    return rec
