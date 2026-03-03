#!/usr/bin/env python3
"""
mm_gui.py — Micromouse Debug GUI v4.0
Cyberpunk HUD — Labs 1–4. Bigger plots. Data viewer. CSV export.

Prerequisites:  pip install bleak
Usage:          python mm_gui.py
"""
import tkinter as tk
from tkinter import ttk, filedialog
import struct, time, threading, math, csv, os, json
from mm_protocol import *
from mm_ble_client import MicromouseBLE
from mm_run_analyzer import RunManager, RunSnapshot, RunState, RunMetrics, RunRecord, TrapProfile, load_run

# ═══════════════════════════════════════════════════════════════════════════════
# CYBERPUNK PALETTE
# ═══════════════════════════════════════════════════════════════════════════════
BG='#0a0a0f'; BG_CARD='#121218'; BG_INPUT='#1a1a24'; BG_HOVER='#1e1e2a'
BORDER='#2a2a3a'; NEON_GREEN='#00ff41'; NEON_CYAN='#00e5ff'
NEON_MAGENTA='#ff00ff'; NEON_YELLOW='#ffea00'; NEON_RED='#ff1744'
NEON_ORANGE='#ff9100'; DIM_GREEN='#00802a'; DIM_CYAN='#007a8a'
TEXT='#b0b8c0'; TEXT_DIM='#5a6068'; TEXT_BRIGHT='#e0e8f0'; WHITE='#ffffff'

def setup_theme(root):
    s=ttk.Style(); s.theme_use('clam')
    s.configure('.',background=BG,foreground=TEXT,borderwidth=0,focusthickness=0)
    s.configure('TFrame',background=BG); s.configure('TLabel',background=BG,foreground=TEXT)
    s.configure('TLabelframe',background=BG_CARD,foreground=NEON_GREEN,bordercolor=BORDER,relief='solid',borderwidth=1)
    s.configure('TLabelframe.Label',background=BG_CARD,foreground=NEON_GREEN,font=('Consolas',9,'bold'))
    s.configure('TNotebook',background=BG,borderwidth=0)
    s.configure('TNotebook.Tab',background=BG_CARD,foreground=TEXT_DIM,padding=[14,6],font=('Consolas',9,'bold'))
    s.map('TNotebook.Tab',background=[('selected',BG_HOVER)],foreground=[('selected',NEON_CYAN)])
    s.configure('TButton',background=BG_INPUT,foreground=NEON_CYAN,padding=[10,5],font=('Consolas',9,'bold'),borderwidth=1)
    s.map('TButton',background=[('active',BG_HOVER)],foreground=[('active',WHITE)])
    for name,bg,fg in [('G','#003d00',NEON_GREEN),('R','#3d0000',NEON_RED),
                        ('O','#3d2200',NEON_ORANGE),('M','#2a002a',NEON_MAGENTA),('C','#002a3d',NEON_CYAN)]:
        s.configure(f'{name}.TButton',background=bg,foreground=fg,font=('Consolas',9,'bold'))
        s.map(f'{name}.TButton',background=[('active',bg.replace('3d','55').replace('2a','44'))],foreground=[('active',WHITE)])
    s.configure('TCombobox',fieldbackground=BG_INPUT,foreground=NEON_GREEN,selectbackground=DIM_GREEN,font=('Consolas',9))
    s.map('TCombobox',fieldbackground=[('readonly',BG_INPUT)])
    s.configure('G.Horizontal.TProgressbar',troughcolor=BG_INPUT,background=NEON_GREEN,darkcolor=DIM_GREEN,lightcolor=NEON_GREEN)
    # Sub-notebook for data/plot tabs
    s.configure('Sub.TNotebook',background=BG_CARD,borderwidth=0)
    s.configure('Sub.TNotebook.Tab',background=BG_INPUT,foreground=TEXT_DIM,padding=[10,4],font=('Consolas',8,'bold'))
    s.map('Sub.TNotebook.Tab',background=[('selected',BG_HOVER)],foreground=[('selected',NEON_YELLOW)])
    root.configure(bg=BG)

def make_card(p,t): return ttk.LabelFrame(p,text=f" {t} ",padding=8)
def make_hud(p,t="---",sz=14,c=NEON_GREEN): return tk.Label(p,text=t,font=("Consolas",sz,"bold"),fg=c,bg=BG_CARD)
def make_ro(p,h=5,c=NEON_GREEN):
    t=tk.Text(p,height=h,font=("Consolas",9),bg=BG_INPUT,fg=c,relief=tk.FLAT,padx=8,pady=5,
              insertbackground=NEON_GREEN,selectbackground=DIM_GREEN,highlightbackground=BORDER,highlightthickness=1)
    t.config(state=tk.DISABLED); return t
def make_spin(p,v,f,t,s,w=7):
    return tk.Spinbox(p,from_=f,to=t,increment=s,textvariable=v,width=w,bg=BG_INPUT,fg=NEON_GREEN,
                      font=("Consolas",10,"bold"),insertbackground=NEON_GREEN,buttonbackground=BG_CARD,
                      selectbackground=DIM_GREEN,relief=tk.FLAT,highlightbackground=BORDER,highlightthickness=1)
def wr(w,t):
    w.config(state=tk.NORMAL); w.delete("1.0",tk.END); w.insert("1.0",t); w.config(state=tk.DISABLED)


# ═══════════════════════════════════════════════════════════════════════════════
# HUD Canvas Plot — resizable
# ═══════════════════════════════════════════════════════════════════════════════
class HudPlot:
    def __init__(self, parent, width=400, height=200, title="", y_min=-100, y_max=100,
                 lines=None, auto_scale=False):
        self.w=width; self.h=height; self.y_min=y_min; self.y_max=y_max
        self.auto_scale=auto_scale
        self.ml=55; self.mr=10; self.mt=18; self.mb=18
        self.pw=self.w-self.ml-self.mr; self.ph=self.h-self.mt-self.mb
        f=tk.Frame(parent,bg=BG_CARD); f.pack(fill=tk.X,pady=2)
        if title:
            tk.Label(f,text=title,font=("Consolas",8,"bold"),fg=NEON_CYAN,bg=BG_CARD).pack(anchor=tk.W)
        self.canvas=tk.Canvas(f,width=width,height=height,bg='#08080c',
                              highlightbackground=BORDER,highlightthickness=1)
        self.canvas.pack(fill=tk.X)
        self.lines=lines or [{'name':'y','color':NEON_GREEN}]
        self.data={l['name']:[] for l in self.lines}
        self.max_points=self.pw
        self._frozen=False
        self._visible={l['name']:True for l in self.lines}
        self._overlays=[]  # list of (y_value, color, label)
        self._regions=[]   # list of (x_start_frac, x_end_frac, color)
        self._overlay_series=[]  # list of (name, [values], color) for compare
        self._draw_grid()

    def _draw_grid(self):
        c=self.canvas; c.delete("grid")
        x0,y0=self.ml,self.mt; x1,y1=x0+self.pw,y0+self.ph
        for i in range(5):
            y=y0+i*self.ph//4
            c.create_line(x0,y,x1,y,fill='#1a1a2a',dash=(2,4),tags="grid")
            val=self.y_max-i*(self.y_max-self.y_min)/4
            c.create_text(x0-5,y,text=f"{val:.0f}",fill=TEXT_DIM,font=("Consolas",7),anchor=tk.E,tags="grid")
        if self.y_min<0<self.y_max:
            zy=y0+self.ph*self.y_max/(self.y_max-self.y_min)
            c.create_line(x0,zy,x1,zy,fill='#2a2a3a',tags="grid")
        c.create_rectangle(x0,y0,x1,y1,outline=BORDER,tags="grid")

    def _y2px(self,v):
        rng=max(self.y_max-self.y_min,1)
        f=(v-self.y_min)/rng; f=max(0,min(1,f))
        return self.mt+self.ph*(1-f)

    def add_point(self,**kw):
        if self._frozen: return  # ★ ignore new data when frozen
        for k,v in kw.items():
            if k in self.data:
                self.data[k].append(v)
                if len(self.data[k])>self.max_points:
                    self.data[k]=self.data[k][-self.max_points:]

    def redraw(self):
        if self._frozen: return  # ★ don't redraw when frozen
        if self.auto_scale: self._auto_range()
        self._do_redraw()

    def _do_redraw(self):
        """Internal redraw (used by both live and freeze)."""
        c=self.canvas; c.delete("plot"); c.delete("overlay")
        # Draw highlight regions first (behind lines)
        n_pts = max(len(self.data.get(self.lines[0]['name'],[])),1)
        for (fs, fe, clr) in self._regions:
            xs = self.ml + int(fs * self.pw)
            xe = self.ml + int(fe * self.pw)
            c.create_rectangle(xs, self.mt, xe, self.mt+self.ph,
                              fill=clr, outline='', tags="overlay")
        # Draw overlay series (reference runs — behind main data)
        for (name, vals, clr) in self._overlay_series:
            if len(vals) < 2: continue
            ns = len(vals)
            step = self.pw / (ns - 1) if ns > 1 else 1
            coords = []
            for i, v in enumerate(vals):
                coords.extend([self.ml + i * step, self._y2px(v)])
            c.create_line(*coords, fill=clr, width=1, dash=(3,3), tags="overlay")
        # Draw data lines
        for ld in self.lines:
            if not self._visible.get(ld['name'],True): continue  # skip hidden
            pts=self.data.get(ld['name'],[])
            if len(pts)<2: continue
            n=len(pts)
            # When frozen, spread data across full width
            if self._frozen and n > 1:
                step = self.pw / (n - 1) if n > 1 else 1
                coords=[]
                for i,v in enumerate(pts):
                    coords.extend([self.ml + i * step, self._y2px(v)])
            else:
                xs=self.ml+self.pw-n
                coords=[]
                for i,v in enumerate(pts): coords.extend([xs+i,self._y2px(v)])
            c.create_line(*coords,fill=ld['color'],width=1,tags="plot")
        # Draw overlay horizontal lines (target, etc.)
        for (yv, clr, lbl) in self._overlays:
            yp = self._y2px(yv)
            if self.mt <= yp <= self.mt + self.ph:
                c.create_line(self.ml, yp, self.ml+self.pw, yp,
                             fill=clr, width=1, dash=(4,3), tags="overlay")
                c.create_text(self.ml+self.pw-2, yp-6, text=lbl,
                             fill=clr, font=("Consolas",7), anchor=tk.E, tags="overlay")

    def _auto_range(self):
        all_v=[]
        for ld in self.lines: all_v.extend(self.data.get(ld['name'],[]))
        if not all_v: return
        mn,mx=min(all_v),max(all_v)
        mg=max(abs(mx-mn)*0.15,10)
        new_min,new_max=mn-mg,mx+mg
        if abs(new_min-self.y_min)>mg*0.3 or abs(new_max-self.y_max)>mg*0.3:
            self.y_min,self.y_max=new_min,new_max
            self.canvas.delete("grid"); self._draw_grid()

    def clear(self):
        for k in self.data: self.data[k]=[]
        self._frozen=False; self._overlays=[]; self._regions=[]; self._overlay_series=[]
        self._visible={l['name']:True for l in self.lines}
        self.canvas.delete("all"); self._draw_grid()

    def set_range(self,y_min,y_max):
        self.y_min=y_min; self.y_max=y_max
        self.canvas.delete("all"); self._draw_grid()

    # ── Freeze/Overlay API ──
    def freeze(self):
        """Stop accepting new data, lock scale, redraw spread across full width."""
        self._frozen=True
        if self.auto_scale: self._auto_range()
        self._do_redraw()

    def unfreeze(self):
        self._frozen=False; self._overlays=[]; self._regions=[]

    def set_overlay_lines(self, overlays):
        """Set horizontal overlay lines: list of (y_value, color, label)."""
        self._overlays = list(overlays)

    def set_highlight_regions(self, regions):
        """Set highlighted x-regions: list of (frac_start, frac_end, color)."""
        self._regions = list(regions)

    def load_data(self, data_dict):
        """Load all data at once (for replay). data_dict: {name: [values]}."""
        self._frozen = True
        for k, v in data_dict.items():
            if k in self.data:
                self.data[k] = list(v)
        if self.auto_scale: self._auto_range()
        self._do_redraw()

    def add_overlay_series(self, name, values, color):
        """Add a reference overlay series (for compare). Drawn dimmed behind main."""
        self._overlay_series.append((name, list(values), color))

    def clear_overlay_series(self):
        self._overlay_series = []

    def set_line_visible(self, name, visible):
        """Show/hide a named line. Redraws if frozen."""
        if name in self._visible:
            self._visible[name] = visible
            if self._frozen: self._do_redraw()
            elif any(self.data.get(k,[]) for k in self.data): self._do_redraw()


# ═══════════════════════════════════════════════════════════════════════════════
# DataViewer — scrollable table + CSV export
# ═══════════════════════════════════════════════════════════════════════════════
class DataViewer:
    """Scrollable data table with CSV export."""
    COLUMNS = ['#','t_ms','velL','velR','cmdL','cmdR','gyroZ','encL','encR']
    HEADER  = f"{'#':>5} {'t_ms':>7} {'velL':>6} {'velR':>6} {'cmdL':>6} {'cmdR':>6} {'gyro':>7} {'eL':>4} {'eR':>4}"

    def __init__(self, parent):
        f=tk.Frame(parent,bg=BG_CARD)
        f.pack(fill=tk.BOTH,expand=True)

        # Header + export
        hdr=tk.Frame(f,bg=BG_CARD); hdr.pack(fill=tk.X)
        tk.Label(hdr,text=self.HEADER,font=("Consolas",8),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT)
        ttk.Button(hdr,text="⬇ CSV",command=self.export_csv,style="G.TButton",width=7).pack(side=tk.RIGHT,padx=3)
        ttk.Button(hdr,text="📋 COPY",command=self.copy_all,width=7).pack(side=tk.RIGHT,padx=3)
        self.count_lbl=tk.Label(hdr,text="0 rows",font=("Consolas",8),fg=TEXT_DIM,bg=BG_CARD)
        self.count_lbl.pack(side=tk.RIGHT,padx=8)

        # Text widget with scrollbar
        tf=tk.Frame(f,bg=BG_CARD); tf.pack(fill=tk.BOTH,expand=True)
        self.text=tk.Text(tf,font=("Consolas",8),bg='#08080c',fg=NEON_GREEN,
                          relief=tk.FLAT,padx=4,pady=2,highlightbackground=BORDER,highlightthickness=1,
                          wrap=tk.NONE)
        sb=tk.Scrollbar(tf,command=self.text.yview,bg=BG_INPUT,troughcolor=BG_INPUT)
        self.text.config(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT,fill=tk.Y)
        self.text.pack(side=tk.LEFT,fill=tk.BOTH,expand=True)
        self.text.config(state=tk.DISABLED)

        self._rows = []   # list of dicts (raw FAST data)
        self._parent = parent

    def clear(self):
        self._rows = []
        self.text.config(state=tk.NORMAL)
        self.text.delete("1.0",tk.END)
        self.text.config(state=tk.DISABLED)
        self.count_lbl.config(text="0 rows")

    def add_sample(self, f_data):
        """Add a parsed FAST sample dict."""
        self._rows.append(f_data)
        n = len(self._rows)
        gz = f_data.get('gyroZ_dps10',0)/10.0
        line = (f"{n:5d} {f_data.get('t_ms',0):7d} "
                f"{f_data.get('velL_mmps',0):+6d} {f_data.get('velR_mmps',0):+6d} "
                f"{f_data.get('vCmdL_mV',0):+6d} {f_data.get('vCmdR_mV',0):+6d} "
                f"{gz:+7.1f} {f_data.get('encL_delta',0):+4d} {f_data.get('encR_delta',0):+4d}\n")
        self.text.config(state=tk.NORMAL)
        self.text.insert(tk.END, line)
        self.text.see(tk.END)
        self.text.config(state=tk.DISABLED)
        if n % 10 == 0:
            self.count_lbl.config(text=f"{n} rows")

    def finish(self):
        self.count_lbl.config(text=f"{len(self._rows)} rows")

    def export_csv(self):
        if not self._rows:
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV","*.csv"),("All","*.*")],
            initialfile=f"mm_data_{int(time.time())}.csv")
        if not path: return
        keys = ['t_ms','state','vCmdL_mV','vCmdR_mV','velL_mmps','velR_mmps',
                'v_target_mmps','gyroZ_dps10','encL_delta','encR_delta']
        with open(path,'w',newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
            w.writeheader()
            for r in self._rows:
                w.writerow(r)
        self.count_lbl.config(text=f"✓ Saved {len(self._rows)} rows",fg=NEON_GREEN)

    def copy_all(self):
        if not self._rows: return
        lines = [self.HEADER]
        for i,r in enumerate(self._rows):
            gz=r.get('gyroZ_dps10',0)/10.0
            lines.append(f"{i+1:5d} {r.get('t_ms',0):7d} "
                         f"{r.get('velL_mmps',0):+6d} {r.get('velR_mmps',0):+6d} "
                         f"{r.get('vCmdL_mV',0):+6d} {r.get('vCmdR_mV',0):+6d} "
                         f"{gz:+7.1f} {r.get('encL_delta',0):+4d} {r.get('encR_delta',0):+4d}")
        self._parent.winfo_toplevel().clipboard_clear()
        self._parent.winfo_toplevel().clipboard_append("\n".join(lines))
        self.count_lbl.config(text=f"✓ Copied {len(self._rows)} rows",fg=NEON_YELLOW)

    def get_data(self):
        return self._rows


# ═══════════════════════════════════════════════════════════════════════════════
# Lab 1 — HW Check (with LED brightness)
# ═══════════════════════════════════════════════════════════════════════════════
class Lab1Tab:
    def __init__(self,parent,ble):
        self.ble=ble; self.frame=ttk.Frame(parent,padding=5); self._pending={}
        left=ttk.Frame(self.frame);left.pack(side=tk.LEFT,fill=tk.BOTH,expand=True,padx=4)
        right=ttk.Frame(self.frame);right.pack(side=tk.LEFT,fill=tk.BOTH,expand=True,padx=4)

        # Safety
        af=make_card(left,"▸ SAFETY");af.pack(fill=tk.X,pady=3)
        br=ttk.Frame(af);br.pack(fill=tk.X)
        ttk.Button(br,text="◆ ARM",command=self.do_arm,style="G.TButton",width=11).pack(side=tk.LEFT,padx=2)
        ttk.Button(br,text="◇ DISARM",command=self.do_disarm,style="O.TButton",width=11).pack(side=tk.LEFT,padx=2)
        ttk.Button(br,text="✖ ABORT",command=self.do_abort,style="R.TButton",width=11).pack(side=tk.LEFT,padx=2)
        self.state_lbl=tk.Label(af,text="[ IDLE ]",font=("Consolas",13,"bold"),fg=TEXT_DIM,bg=BG_CARD)
        self.state_lbl.pack(pady=5)

        # Motor
        mf=make_card(left,"▸ MOTOR [ARM]");mf.pack(fill=tk.X,pady=3)
        r1=tk.Frame(mf,bg=BG_CARD);r1.pack(fill=tk.X,pady=2)
        tk.Label(r1,text="mV:",font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT)
        self.motor_mv=tk.IntVar(value=5000);make_spin(r1,self.motor_mv,-8000,8000,500).pack(side=tk.LEFT,padx=4)
        tk.Label(r1,text="ms:",font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT,padx=(10,0))
        self.motor_dur=tk.IntVar(value=1000);make_spin(r1,self.motor_dur,100,5000,100).pack(side=tk.LEFT,padx=4)
        r2=ttk.Frame(mf);r2.pack(pady=4)
        ttk.Button(r2,text="◁ L",command=lambda:self.do_motor(1),width=8).pack(side=tk.LEFT,padx=2)
        ttk.Button(r2,text="◁▷ BOTH",command=lambda:self.do_motor(0),style="G.TButton",width=9).pack(side=tk.LEFT,padx=2)
        ttk.Button(r2,text="R ▷",command=lambda:self.do_motor(2),width=8).pack(side=tk.LEFT,padx=2)
        ttk.Button(r2,text="■ STOP",command=self.do_stop,style="R.TButton",width=8).pack(side=tk.LEFT,padx=2)

        # LED / Buzzer / Brightness
        lf=make_card(left,"▸ LED & BUZZER [no ARM]");lf.pack(fill=tk.X,pady=3)
        lr=tk.Frame(lf,bg=BG_CARD);lr.pack(pady=2)
        for txt,fc,bc,r,g,b in [("● RED",'#ff0000','#330000',255,0,0),("● GRN",'#00ff00','#003300',0,255,0),
                ("● BLU",'#0066ff','#001133',0,0,255),("● WHT",'#ffffff','#333333',255,255,255),("○ OFF",TEXT_DIM,BG_INPUT,0,0,0)]:
            tk.Button(lr,text=txt,font=("Consolas",8,"bold"),fg=fc,bg=bc,activebackground=bc,
                      activeforeground=fc,relief=tk.FLAT,padx=6,pady=3,borderwidth=1,
                      highlightbackground=BORDER,highlightthickness=1,
                      command=lambda r=r,g=g,b=b:self.do_led(r,g,b)).pack(side=tk.LEFT,padx=2)
        bri_row=tk.Frame(lf,bg=BG_CARD);bri_row.pack(fill=tk.X,pady=3)
        tk.Label(bri_row,text="☀ Brightness:",font=("Consolas",8),fg=NEON_YELLOW,bg=BG_CARD).pack(side=tk.LEFT)
        self.bri_var=tk.IntVar(value=40)
        self.bri_scale=tk.Scale(bri_row,from_=0,to=255,orient=tk.HORIZONTAL,variable=self.bri_var,
                                length=200,bg=BG_CARD,fg=NEON_YELLOW,troughcolor=BG_INPUT,
                                highlightbackground=BG_CARD,activebackground=NEON_YELLOW,
                                font=("Consolas",8),command=self._on_brightness)
        self.bri_scale.pack(side=tk.LEFT,padx=5,fill=tk.X,expand=True)
        bz=ttk.Frame(lf);bz.pack(pady=2)
        for txt,f,d in [("♪ 1k",1000,200),("♪ 2k",2000,100),("♪ Low",500,500)]:
            ttk.Button(bz,text=txt,command=lambda f=f,d=d:self.do_buzzer(f,d),style="M.TButton").pack(side=tk.LEFT,padx=2)

        # Sensors
        sf=make_card(left,"▸ SENSORS [no ARM]");sf.pack(fill=tk.X,pady=3)
        sr=ttk.Frame(sf);sr.pack()
        ttk.Button(sr,text="⚡ BATT",command=self.do_read_battery).pack(side=tk.LEFT,padx=3)
        ttk.Button(sr,text="◈ WALLS",command=self.do_read_sensors).pack(side=tk.LEFT,padx=3)
        ttk.Button(sr,text="◎ IMU",command=self.do_read_imu).pack(side=tk.LEFT,padx=3)
        ttk.Button(sr,text="⊞ BTN",command=self.do_read_buttons).pack(side=tk.LEFT,padx=3)
        self.readout=make_ro(left,5,NEON_CYAN);self.readout.pack(fill=tk.X,pady=4)

        # Calibration
        cf2=make_card(left,"▸ CALIBRATE [no ARM, keep still!]");cf2.pack(fill=tk.X,pady=3)
        cr1=tk.Frame(cf2,bg=BG_CARD);cr1.pack(fill=tk.X,pady=2)
        ttk.Button(cr1,text="⊙ CAL GYRO",command=self.do_cal_gyro,style="C.TButton",width=14).pack(side=tk.LEFT,padx=2)
        tk.Label(cr1,text="samples:",font=("Consolas",8),fg=TEXT_DIM,bg=BG_CARD).pack(side=tk.LEFT,padx=(6,0))
        self.cal_gyro_n=tk.IntVar(value=500);make_spin(cr1,self.cal_gyro_n,100,2000,100).pack(side=tk.LEFT,padx=2)

        # Wall sensor offset calibration — per-sensor (GUI display only)
        tk.Label(cf2,text="── WALL SENSOR OFFSETS (display only) ──",font=("Consolas",8,"bold"),
                 fg=NEON_YELLOW,bg=BG_CARD).pack(anchor=tk.W,padx=4,pady=(6,2))
        cr_wb=tk.Frame(cf2,bg=BG_CARD);cr_wb.pack(fill=tk.X,pady=2)
        ttk.Button(cr_wb,text="📡 MEASURE WALLS",command=self.do_cal_walls,style="C.TButton",width=18).pack(side=tk.LEFT,padx=2)
        tk.Label(cr_wb,text="(กดซ้ำได้ avg 50 readings)",font=("Consolas",7),fg=TEXT_DIM,bg=BG_CARD).pack(side=tk.LEFT,padx=4)

        # Header row
        hdr=tk.Frame(cf2,bg=BG_CARD);hdr.pack(fill=tk.X,padx=4)
        for txt,w in [("Sensor",7),("Read",6),("Actual",6),("Offset",7)]:
            tk.Label(hdr,text=txt,font=("Consolas",7,"bold"),fg=TEXT_DIM,bg=BG_CARD,width=w,anchor=tk.W).pack(side=tk.LEFT)

        # Per-sensor rows — display only, no apply button
        self.wall_cal = {}  # key → {read_lbl, actual_var, offset_lbl}
        SNS_NAMES = [("L",0),("FL",1),("FR",2),("R",3)]
        for name,pos in SNS_NAMES:
            row=tk.Frame(cf2,bg=BG_CARD);row.pack(fill=tk.X,padx=4)
            tk.Label(row,text=f" {name:>2}:",font=("Consolas",9,"bold"),fg=NEON_CYAN,bg=BG_CARD,width=7,anchor=tk.W).pack(side=tk.LEFT)
            read_lbl=tk.Label(row,text="--",font=("Consolas",9),fg=TEXT_DIM,bg=BG_CARD,width=6,anchor=tk.E)
            read_lbl.pack(side=tk.LEFT)
            actual_var=tk.IntVar(value=0);make_spin(row,actual_var,0,300,1,w=5).pack(side=tk.LEFT,padx=2)
            offset_lbl=tk.Label(row,text="--",font=("Consolas",9,"bold"),fg=TEXT_DIM,bg=BG_CARD,width=7,anchor=tk.E)
            offset_lbl.pack(side=tk.LEFT)
            self.wall_cal[pos]={'read_lbl':read_lbl,'actual_var':actual_var,'offset_lbl':offset_lbl,'raw':0}
            # Auto-compute offset when actual spinbox changes
            actual_var.trace_add("write", lambda *a,p=pos: self._recompute_wall_offset(p))

        tk.Label(cf2,text="ใส่ระยะจริง (mm) แล้วดู offset คำนวณอัตโนมัติ → นำไปใส่ config.h",
                 font=("Consolas",7),fg=TEXT_DIM,bg=BG_CARD).pack(anchor=tk.W,padx=4,pady=(2,0))
        self.cal_status=tk.Label(cf2,text="",font=("Consolas",8),fg=NEON_YELLOW,bg=BG_CARD)
        self.cal_status.pack(anchor=tk.W,padx=4)

        # RIGHT: Live
        bf=make_card(right,"▸ POWER");bf.pack(fill=tk.X,pady=3)
        self.batt_lbl=make_hud(bf,"-.-- V",22);self.batt_lbl.pack()
        self.batt_bar=ttk.Progressbar(bf,length=280,maximum=8400,style='G.Horizontal.TProgressbar')
        self.batt_bar.pack(fill=tk.X,pady=3)
        ff=make_card(right,"▸ TELEMETRY");ff.pack(fill=tk.BOTH,expand=True,pady=3)
        self.fast_txt=make_ro(ff,8,NEON_GREEN);self.fast_txt.pack(fill=tk.BOTH,expand=True)
        df=make_card(right,"▸ DIAGNOSTICS");df.pack(fill=tk.X,pady=3)
        self.slow_txt=make_ro(df,3,NEON_MAGENTA);self.slow_txt.pack(fill=tk.X)
        self.pkt_lbl=tk.Label(right,text="PKT F:0 S:0 R:0",font=("Consolas",8),fg=TEXT_DIM,bg=BG)
        self.pkt_lbl.pack(pady=2)
        self.fast_count=self.slow_count=self.rsp_count=0

    def _on_brightness(self,val): self.ble.send_cmd(cmd_led_brightness(int(val)))
    def do_arm(self): self.ble.send_cmd(cmd_arm())
    def do_disarm(self): self.ble.send_cmd(cmd_disarm())
    def do_abort(self): self.ble.send_cmd(cmd_abort())
    def do_stop(self): self.ble.send_cmd(cmd_stop())
    def do_motor(self,s): self.ble.send_cmd(cmd_motor_spin(self.motor_mv.get(),self.motor_dur.get(),s,req_id=200+s))
    def do_led(self,r,g,b): self.ble.send_cmd(cmd_led_set(r,g,b))
    def do_buzzer(self,f,d): self.ble.send_cmd(cmd_buzzer(f,d))
    def do_read_battery(self): self._pending[300]=self._show_battery;self.ble.send_cmd(cmd_read_battery(req_id=300))
    def do_read_sensors(self): self._pending[301]=self._show_sensors;self.ble.send_cmd(cmd_read_sensors(req_id=301))
    def do_read_imu(self): self._pending[302]=self._show_imu;self.ble.send_cmd(cmd_read_imu(req_id=302))
    def do_read_buttons(self): self._pending[303]=self._show_buttons;self.ble.send_cmd(cmd_read_buttons(req_id=303))
    def do_cal_gyro(self):
        n=max(10,self.cal_gyro_n.get()//10)
        self.cal_status.config(text="⏳ Gyro calibration → LED จะเป็นสี CYAN...",fg=NEON_CYAN)
        wr(self.readout," ⊙ Sending gyro cal request...\n   หุ่นยนต์จะแสดง LED สี CYAN ระหว่าง calibrate\n   ห้ามขยับ! รอ ~1-2s...")
        self._cal_gyro_n = n
        self._pending[308]=self._show_cal_gyro;self.ble.send_cmd(cmd_cal_gyro(samples_div10=n,req_id=308))
    def do_cal_walls(self):
        self.cal_status.config(text="⏳ Measuring walls... กดซ้ำได้",fg=NEON_YELLOW)
        wr(self.readout," 📡 Measuring wall sensors...\n   Averaging 50 readings per sensor...")
        # ★ Always re-register callback — fixes "only works once" bug
        self._pending[309]=self._show_cal_walls;self.ble.send_cmd(cmd_cal_walls(req_id=309))
    def do_cal_query(self):
        self._pending[310]=self._show_cal_query;self.ble.send_cmd(cmd_cal_query(req_id=310))
    def _poll_gyro_cal(self):
        """Auto-poll gyro calibration status."""
        self._pending[308]=self._show_cal_gyro
        self.ble.send_cmd(cmd_cal_gyro(samples_div10=getattr(self,'_cal_gyro_n',50),req_id=308))
    def _poll_wall_cal(self):
        """Auto-poll wall calibration status."""
        self._pending[309]=self._show_cal_walls
        self.ble.send_cmd(cmd_cal_walls(req_id=309))
    def _recompute_wall_offset(self, pos):
        info = self.wall_cal[pos]
        raw = info['raw']
        names = {0:'L',1:'FL',2:'FR',3:'R'}
        try: actual = info['actual_var'].get()
        except: return
        if raw > 0 and actual > 0:
            offset = actual - raw
            info['offset_lbl'].config(text=f"{offset:+d}mm",
                fg=NEON_GREEN if abs(offset)<10 else NEON_YELLOW if abs(offset)<30 else NEON_RED)
        else:
            info['offset_lbl'].config(text="--",fg=TEXT_DIM)

    def handle_rsp(self,data):
        self.rsp_count+=1;rsp=parse_rsp(data)
        if not rsp:return
        cb=self._pending.pop(rsp['req_id'],None)
        if cb:cb(rsp)
    def _show_battery(self,r):
        if r['status']!=RSP_OK:wr(self.readout,f" ✖ {RSP_NAMES.get(r['status'],'?')}");return
        b=parse_rsp_battery(r['payload'])
        wr(self.readout,f" ⚡ Battery\n   Raw: {b['vBat_raw_mV']}mV  Avg: {b['vBat_avg_mV']}mV\n   Level: {['CRITICAL','LOW','OK','FULL'][b['level']]}")
    def _show_sensors(self,r):
        if r['status']!=RSP_OK:wr(self.readout,f" ✖ {RSP_NAMES.get(r['status'],'?')}");return
        s=parse_rsp_sensors(r['payload']);bm=s['bitmap'];w=lambda i:'█'if(bm>>i)&1 else'░'
        wr(self.readout,f" ◈ Walls\n   L:{s['left_mm']:4d}mm{w(3)} FL:{s['fl_mm']:4d}mm{w(2)}\n   FR:{s['fr_mm']:4d}mm{w(1)} R:{s['right_mm']:4d}mm{w(0)}")
    def _show_imu(self,r):
        if r['status']!=RSP_OK:wr(self.readout,f" ✖ {RSP_NAMES.get(r['status'],'?')}");return
        m=parse_rsp_imu(r['payload'])
        wr(self.readout,f" ◎ IMU\n   GYR: X={m['gyroX']:+6d} Y={m['gyroY']:+6d} Z={m['gyroZ']:+6d}\n   ACC: X={m['accelX']:+6d} Y={m['accelY']:+6d} Z={m['accelZ']:+6d}")
    def _show_buttons(self,r):
        if r['status']!=RSP_OK:wr(self.readout,f" ✖ {RSP_NAMES.get(r['status'],'?')}");return
        b=parse_rsp_buttons(r['payload'])
        wr(self.readout,f" ⊞ Input\n   START:{'▶'if b['start'] else'○'} MODE:{'▶'if b['mode'] else'○'}\n   DIP:[{b['dip']:04b}]={b['dip']}  EncL:{b['encL']:+d} EncR:{b['encR']:+d}")
    def _show_cal_gyro(self,r):
        if r['status']!=RSP_OK:
            self.cal_status.config(text=f"✖ {RSP_NAMES.get(r['status'],'?')}",fg=NEON_RED);return
        c=parse_rsp_cal_gyro(r['payload'])
        if c['status'] == 2:
            # Done — show results + clear feedback
            self.cal_status.config(text="✔ Gyro cal done! LED จะกลับปกติ",fg=NEON_GREEN)
            wr(self.readout,f" ⊙ GYRO CALIBRATION ✔ COMPLETE\n"
               f"   Offset Z: {c['gyro_z_off']:+.3f} °/s  ← heading\n"
               f"   Offset X: {c['gyro_x_off']:+.3f} °/s\n"
               f"   Offset Y: {c['gyro_y_off']:+.3f} °/s\n"
               f"   Calibration applied ✔  (LED cyan → off)")
        elif c['status'] == 0:
            self.cal_status.config(text="⏳ Gyro cal started → LED CYAN ← อย่าขยับ!",fg=NEON_CYAN)
            self.frame.after(1500, self._poll_gyro_cal)
        else:
            self.cal_status.config(text="⏳ Still calibrating... LED CYAN",fg=NEON_CYAN)
            self.frame.after(500, self._poll_gyro_cal)
    def _show_cal_walls(self,r):
        if r['status']!=RSP_OK:
            self.cal_status.config(text=f"✖ {RSP_NAMES.get(r['status'],'?')}",fg=NEON_RED);return
        c=parse_rsp_cal_walls(r['payload'])
        if c['status'] == 2:
            # Done — fill in per-sensor read columns
            self.cal_status.config(text="✔ Measured! ใส่ actual mm → ดู offset → ใส่ config.h",fg=NEON_GREEN)
            raw_keys = {0:'raw_L',1:'raw_FL',2:'raw_FR',3:'raw_R'}
            for pos in range(4):
                raw = c[raw_keys[pos]]
                info = self.wall_cal[pos]
                info['raw'] = raw
                info['read_lbl'].config(text=f"{raw:3d}" if raw>0 else "--",
                    fg=NEON_CYAN if raw>0 else TEXT_DIM)
                self._recompute_wall_offset(pos)
            wr(self.readout,f" ◈ WALL RAW AVERAGES (before offset)\n"
               f"   L:{c['raw_L']:3d}mm  FL:{c['raw_FL']:3d}mm  FR:{c['raw_FR']:3d}mm  R:{c['raw_R']:3d}mm\n"
               f"   FW offsets → L:{c['off_L']:+d} FL:{c['off_FL']:+d} FR:{c['off_FR']:+d} R:{c['off_R']:+d}\n"
               f"   ↳ ใส่ actual distance แล้วดู offset → นำไปใส่ config.h")
        elif c['status'] == 0:
            self.cal_status.config(text="⏳ Measuring walls... ~1.5s",fg=NEON_YELLOW)
            self.frame.after(2000, self._poll_wall_cal)
        else:
            self.cal_status.config(text="⏳ Still measuring...",fg=NEON_YELLOW)
            self.frame.after(500, self._poll_wall_cal)
    def handle_fast(self,data):
        self.fast_count+=1;f=parse_fast(data)
        if not f:return
        st=f['state']
        if st&FLAG_ARMED and st&FLAG_RUNNING:self.state_lbl.config(text=f"◆ {state_str(st)} ◆",fg=NEON_MAGENTA)
        elif st&FLAG_ARMED:self.state_lbl.config(text=f"◆ {state_str(st)}",fg=NEON_GREEN)
        elif st&FLAG_LOW_BATT:self.state_lbl.config(text=f"⚠ {state_str(st)}",fg=NEON_RED)
        else:self.state_lbl.config(text=f"[ {state_str(st)} ]",fg=TEXT_DIM)
        if self.fast_count%5==0:
            gz=f['gyroZ_dps10']/10.0
            wr(self.fast_txt,
                f"  ┌─ CMD ──────────────────────────┐\n"
                f"  │  L:{f['vCmdL_mV']:+6d}mV  R:{f['vCmdR_mV']:+6d}mV   │\n"
                f"  ├─ VEL ──────────────────────────┤\n"
                f"  │  L:{f['velL_mmps']:+6d}mm/s R:{f['velR_mmps']:+6d}mm/s │\n"
                f"  ├─ GYRO/ENC ─────────────────────┤\n"
                f"  │  Z:{gz:+8.1f}°/s                  │\n"
                f"  │  ΔE: L={f['encL_delta']:+4d} R={f['encR_delta']:+4d}       │\n"
                f"  └───────────── t={f['t_ms']:5d}ms ─────┘")
    def handle_slow(self,data):
        self.slow_count+=1;s=parse_slow(data)
        if not s:return
        wr(self.slow_txt,f"  MISS:{s['missed_ticks']:3d} MAX:{s['step_max_us']:4d}µs WD:{s['wd_age_ms']:5d}ms\n"
           f"  DROP:{s['rx_drop']} CRC:{s['crc_fail']} CMD:0x{s['last_cmd_op']:02X}→{RSP_NAMES.get(s['last_status'],'?')}\n"
           f"  VBAT:{s['vBat_avg_mV']}mV")
        # Battery display (from slow packet — fast packet now carries v_target)
        v=s['vBat_avg_mV']
        volts=v/1000.0
        self.batt_lbl.config(text=f"{volts:.2f} V",fg=NEON_RED if v<6400 else NEON_YELLOW if v<7000 else NEON_GREEN)
        self.batt_bar['value']=v
    def update_counters(self):
        self.pkt_lbl.config(text=f"PKT F:{self.fast_count} S:{self.slow_count} R:{self.rsp_count}")


# ═══════════════════════════════════════════════════════════════════════════════
# Lab 2a — Encoder Calibration (unchanged from v3)
# ═══════════════════════════════════════════════════════════════════════════════
class Lab2aTab:
    def __init__(self,parent,ble):
        self.ble=ble;self.frame=ttk.Frame(parent,padding=5);self._pending={};self._polling=False
        left=ttk.Frame(self.frame);left.pack(side=tk.LEFT,fill=tk.BOTH,expand=True,padx=4)
        right=ttk.Frame(self.frame);right.pack(side=tk.LEFT,fill=tk.BOTH,expand=True,padx=4)
        inf=make_card(left,"▸ PROCEDURE");inf.pack(fill=tk.X,pady=3)
        tk.Label(inf,text="1. RESET  2. Mark wheel  3. Spin 1 rev\n4. Read counts = CPR  5. Calculate mm/count",
                 font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD,justify=tk.LEFT).pack(anchor=tk.W)
        cf=make_card(left,"▸ CONTROL");cf.pack(fill=tk.X,pady=3)
        cr=ttk.Frame(cf);cr.pack(fill=tk.X,pady=3)
        ttk.Button(cr,text="⟲ RESET",command=self.do_reset,style="O.TButton",width=12).pack(side=tk.LEFT,padx=3)
        ttk.Button(cr,text="◉ READ",command=self.do_read,style="C.TButton",width=12).pack(side=tk.LEFT,padx=3)
        self.poll_btn=ttk.Button(cr,text="▶ LIVE",command=self.toggle_poll,style="G.TButton",width=12)
        self.poll_btn.pack(side=tk.LEFT,padx=3)
        ef=make_card(left,"▸ COUNTS");ef.pack(fill=tk.X,pady=3)
        self.enc_L=make_hud(ef,"L: ----",18,NEON_GREEN);self.enc_L.pack(anchor=tk.W,padx=10)
        self.enc_R=make_hud(ef,"R: ----",18,NEON_CYAN);self.enc_R.pack(anchor=tk.W,padx=10)
        calc=make_card(left,"▸ CALC");calc.pack(fill=tk.X,pady=3)
        cr2=tk.Frame(calc,bg=BG_CARD);cr2.pack(fill=tk.X,pady=2)
        tk.Label(cr2,text="⌀mm:",font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT)
        self.wheel_dia=tk.DoubleVar(value=30.0);make_spin(cr2,self.wheel_dia,10,100,0.5,5).pack(side=tk.LEFT,padx=3)
        tk.Label(cr2,text="CPR:",font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT,padx=(8,0))
        self.cpr_var=tk.IntVar(value=815);make_spin(cr2,self.cpr_var,1,10000,1,5).pack(side=tk.LEFT,padx=3)
        ttk.Button(cr2,text="CALC",command=self.do_calc,style="M.TButton").pack(side=tk.LEFT,padx=3)
        self.calc_res=make_hud(calc,"",11,NEON_YELLOW);self.calc_res.pack(pady=3)
        pf=make_card(right,"▸ LIVE PLOT");pf.pack(fill=tk.BOTH,expand=True,pady=3)
        self.plot=HudPlot(pf,420,250,"Counts",y_min=-1000,y_max=1000,
                          lines=[{'name':'L','color':NEON_GREEN},{'name':'R','color':NEON_CYAN}])
        nf=make_card(right,"▸ RESULTS");nf.pack(fill=tk.X,pady=3)
        self.notes=make_ro(nf,5,NEON_GREEN);self.notes.pack(fill=tk.X)
        wr(self.notes,"  Reset → spin 1 rev → read")

    def do_reset(self):
        self.ble.send_cmd(cmd_enc_reset());self.plot.clear()
        self.enc_L.config(text="L: 0");self.enc_R.config(text="R: 0")
    def do_read(self): self._pending[110]=self._show;self.ble.send_cmd(cmd_enc_monitor(req_id=110))
    def toggle_poll(self):
        self._polling=not self._polling
        self.poll_btn.config(text="■ STOP"if self._polling else"▶ LIVE",
                             style="R.TButton"if self._polling else"G.TButton")
    def do_calc(self):
        d=self.wheel_dia.get();c=self.cpr_var.get()
        if c<=0:return
        circ=math.pi*d;mpc=circ/c
        self.calc_res.config(text=f"circ={circ:.2f}mm  mm/cnt={mpc:.5f}")
        wr(self.notes,f"  ⌀={d:.1f}mm CPR={c}\n  circ={circ:.2f}mm\n  mm/count={mpc:.5f}\n\n  → config.h:\n    COUNTS_PER_REV {c}\n    MM_PER_COUNT   {mpc:.5f}")
    def _show(self,rsp):
        if rsp['status']!=RSP_OK:return
        e=parse_rsp_enc_monitor(rsp['payload'])
        self.enc_L.config(text=f"L: {e['countL']:+d}");self.enc_R.config(text=f"R: {e['countR']:+d}")
        self.plot.add_point(L=e['countL'],R=e['countR'])
        ap=self.plot.data['L']+self.plot.data['R']
        if ap:
            mn,mx=min(ap),max(ap);mg=max(abs(mx-mn)*0.2,50)
            self.plot.set_range(mn-mg,mx+mg)
        self.plot.redraw()
        best=max(abs(e['countL']),abs(e['countR']))
        if best>10:self.cpr_var.set(best)
    def handle_rsp(self,data):
        rsp=parse_rsp(data)
        if not rsp:return
        cb=self._pending.pop(rsp['req_id'],None)
        if cb:cb(rsp)
    def poll_tick(self):
        if self._polling:self._pending[110]=self._show;self.ble.send_cmd(cmd_enc_monitor(req_id=110))


# ═══════════════════════════════════════════════════════════════════════════════
# Lab 2b — Motor Characterization (unchanged from v3)
# ═══════════════════════════════════════════════════════════════════════════════
class Lab2bTab:
    def __init__(self,parent,ble):
        self.ble=ble;self.frame=ttk.Frame(parent,padding=5);self._pending={};self._recording=False;self._rec=[]
        left=ttk.Frame(self.frame);left.pack(side=tk.LEFT,fill=tk.BOTH,expand=True,padx=4)
        right=ttk.Frame(self.frame);right.pack(side=tk.LEFT,fill=tk.BOTH,expand=True,padx=4)
        sf=make_card(left,"▸ STEP RESPONSE");sf.pack(fill=tk.X,pady=3)
        r1=tk.Frame(sf,bg=BG_CARD);r1.pack(fill=tk.X,pady=2)
        tk.Label(r1,text="mV:",font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT)
        self.step_mv=tk.IntVar(value=5000);make_spin(r1,self.step_mv,0,8000,500).pack(side=tk.LEFT,padx=4)
        tk.Label(r1,text="ms:",font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT,padx=(8,0))
        self.step_dur=tk.IntVar(value=2000);make_spin(r1,self.step_dur,500,5000,500).pack(side=tk.LEFT,padx=4)
        r1b=ttk.Frame(sf);r1b.pack(pady=3)
        ttk.Button(r1b,text="◁ L",command=lambda:self.do_step(1),width=7).pack(side=tk.LEFT,padx=2)
        ttk.Button(r1b,text="◁▷",command=lambda:self.do_step(0),style="G.TButton",width=7).pack(side=tk.LEFT,padx=2)
        ttk.Button(r1b,text="R ▷",command=lambda:self.do_step(2),width=7).pack(side=tk.LEFT,padx=2)
        rf=make_card(left,"▸ RAMP");rf.pack(fill=tk.X,pady=3)
        rr=tk.Frame(rf,bg=BG_CARD);rr.pack(fill=tk.X,pady=2)
        for l,n,d,f,t in [("St:",'ramp_s',2000,0,8000),("End:",'ramp_e',7000,0,8000),
                           ("Stp:",'ramp_st',200,50,1000),("Dw:",'ramp_d',500,100,2000)]:
            tk.Label(rr,text=l,font=("Consolas",8),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT)
            v=tk.IntVar(value=d);setattr(self,n,v);make_spin(rr,v,f,t,100,4).pack(side=tk.LEFT,padx=1)
        rr2=ttk.Frame(rf);rr2.pack(pady=3)
        ttk.Button(rr2,text="◁ L",command=lambda:self.do_ramp(1),width=7).pack(side=tk.LEFT,padx=2)
        ttk.Button(rr2,text="◁▷",command=lambda:self.do_ramp(0),style="C.TButton",width=7).pack(side=tk.LEFT,padx=2)
        ttk.Button(rr2,text="R ▷",command=lambda:self.do_ramp(2),width=7).pack(side=tk.LEFT,padx=2)
        bf=make_card(left,"▸ FIND BIAS");bf.pack(fill=tk.X,pady=3)
        ttk.Button(bf,text="◆ AUTO FIND BIAS [ARM]",command=self.do_bias,style="M.TButton").pack(pady=3)
        ctrl=make_card(left,"▸ CONTROL");ctrl.pack(fill=tk.X,pady=3)
        cr=ttk.Frame(ctrl);cr.pack()
        self.rec_btn=ttk.Button(cr,text="● REC",command=self.toggle_rec,style="R.TButton",width=8)
        self.rec_btn.pack(side=tk.LEFT,padx=3)
        ttk.Button(cr,text="■ STOP",command=self.do_stop,style="R.TButton",width=8).pack(side=tk.LEFT,padx=3)
        ttk.Button(cr,text="CLEAR",command=self.do_clear,width=8).pack(side=tk.LEFT,padx=3)
        self.rec_lbl=tk.Label(ctrl,text="REC:OFF n=0",font=("Consolas",8),fg=TEXT_DIM,bg=BG_CARD)
        self.rec_lbl.pack(pady=2)
        self.plot_vel=HudPlot(right,430,170,"VELOCITY mm/s",-500,500,
                              [{'name':'vL','color':NEON_GREEN},{'name':'vR','color':NEON_CYAN}])
        self.plot_cmd=HudPlot(right,430,170,"MOTOR CMD mV",0,8000,
                              [{'name':'cL','color':NEON_GREEN},{'name':'cR','color':NEON_CYAN}])
        res=make_card(right,"▸ ANALYSIS");res.pack(fill=tk.X,pady=3)
        self.res_txt=make_ro(res,4,NEON_YELLOW);self.res_txt.pack(fill=tk.X)
        wr(self.res_txt,"  ARM → test → plots update")

    def do_step(self,s): self._start_rec();self.ble.send_cmd(cmd_step_response(self.step_mv.get(),self.step_dur.get(),s))
    def do_ramp(self,s): self._start_rec();self.ble.send_cmd(cmd_ramp_up(self.ramp_s.get(),self.ramp_e.get(),self.ramp_st.get(),self.ramp_d.get(),s))
    def do_bias(self): self._start_rec();self.ble.send_cmd(cmd_find_bias())
    def do_stop(self): self.ble.send_cmd(cmd_stop());self._stop_rec();self._analyze()
    def do_clear(self): self._rec=[];self.plot_vel.clear();self.plot_cmd.clear();wr(self.res_txt,"  Cleared.")
    def _start_rec(self): self._recording=True;self._rec=[];self.rec_btn.config(text="■ REC",style="O.TButton");self.rec_lbl.config(text="● REC n=0",fg=NEON_RED)
    def _stop_rec(self): self._recording=False;self.rec_btn.config(text="● REC",style="R.TButton");self.rec_lbl.config(text=f"REC:OFF n={len(self._rec)}",fg=TEXT_DIM)
    def toggle_rec(self):
        if self._recording:self._stop_rec();self._analyze()
        else:self._start_rec()
    def handle_fast(self,data):
        f=parse_fast(data)
        if not f:return
        self.plot_vel.add_point(vL=f['velL_mmps'],vR=f['velR_mmps'])
        self.plot_cmd.add_point(cL=f['vCmdL_mV'],cR=f['vCmdR_mV'])
        self.plot_vel.redraw();self.plot_cmd.redraw()
        if self._recording:
            self._rec.append(f);self.rec_lbl.config(text=f"● REC n={len(self._rec)}")
            if len(self._rec)>10:
                st=f['state']
                if not(st&FLAG_RUNNING)and(st&FLAG_ARMED):self._stop_rec();self._analyze()
    def _analyze(self):
        d=self._rec
        if len(d)<5:wr(self.res_txt,"  ✖ Not enough data");return
        bL=bR=None;VT=5
        for x in d:
            if bL is None and abs(x['velL_mmps'])>VT and x['vCmdL_mV']!=0:bL=abs(x['vCmdL_mV'])
            if bR is None and abs(x['velR_mmps'])>VT and x['vCmdR_mV']!=0:bR=abs(x['vCmdR_mV'])
        tail=d[int(len(d)*0.8):]
        ssL=sum(x['velL_mmps']for x in tail)/len(tail)if tail else 0
        ssR=sum(x['velR_mmps']for x in tail)/len(tail)if tail else 0
        lines=[f"  ◆ {len(d)} samples"]
        if bL:lines.append(f"  Bias L:~{bL}mV")
        if bR:lines.append(f"  Bias R:~{bR}mV")
        lines.append(f"  SS Vel: L={ssL:+.0f} R={ssR:+.0f}mm/s")
        wr(self.res_txt,"\n".join(lines))
    def handle_rsp(self,data):
        rsp=parse_rsp(data)
        if rsp and rsp['status']!=RSP_OK:wr(self.res_txt,f"  ✖ {RSP_NAMES.get(rsp['status'],'?')}")


# ═══════════════════════════════════════════════════════════════════════════════
# Lab 3 — Motion Profile (with data viewer)
# ═══════════════════════════════════════════════════════════════════════════════
class Lab3Tab:
    def __init__(self,parent,ble):
        self.ble=ble;self.frame=ttk.Frame(parent,padding=5)
        self.mgr=RunManager()
        left=ttk.Frame(self.frame);left.pack(side=tk.LEFT,fill=tk.BOTH,expand=True,padx=4)
        right=ttk.Frame(self.frame);right.pack(side=tk.LEFT,fill=tk.BOTH,expand=True,padx=4)

        sf=make_card(left,"▸ STRAIGHT [ARM]");sf.pack(fill=tk.X,pady=3)
        r1=tk.Frame(sf,bg=BG_CARD);r1.pack(fill=tk.X,pady=2)
        tk.Label(r1,text="mm:",font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT)
        self.fwd_dist=tk.IntVar(value=180);make_spin(r1,self.fwd_dist,10,2000,10).pack(side=tk.LEFT,padx=3)
        tk.Label(r1,text="vel:",font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT,padx=(8,0))
        self.fwd_vel=tk.IntVar(value=200);make_spin(r1,self.fwd_vel,50,800,50).pack(side=tk.LEFT,padx=3)
        tk.Label(r1,text="acc:",font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT,padx=(8,0))
        self.fwd_acc=tk.IntVar(value=500);make_spin(r1,self.fwd_acc,100,2000,100).pack(side=tk.LEFT,padx=3)
        r2=ttk.Frame(sf);r2.pack(pady=3)
        ttk.Button(r2,text="▲ FWD",command=self.do_fwd,style="G.TButton",width=8).pack(side=tk.LEFT,padx=2)
        ttk.Button(r2,text="▼ BACK",command=self.do_back,style="O.TButton",width=8).pack(side=tk.LEFT,padx=2)
        ttk.Button(r2,text="½",command=lambda:self._preset_fwd(90),width=4).pack(side=tk.LEFT,padx=1)
        ttk.Button(r2,text="1C",command=lambda:self._preset_fwd(180),style="G.TButton",width=4).pack(side=tk.LEFT,padx=1)
        ttk.Button(r2,text="2C",command=lambda:self._preset_fwd(360),width=4).pack(side=tk.LEFT,padx=1)

        tf=make_card(left,"▸ TURN [ARM]");tf.pack(fill=tk.X,pady=3)
        t1=tk.Frame(tf,bg=BG_CARD);t1.pack(fill=tk.X,pady=2)
        tk.Label(t1,text="deg:",font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT)
        self.turn_deg=tk.IntVar(value=90);make_spin(t1,self.turn_deg,10,360,10).pack(side=tk.LEFT,padx=3)
        tk.Label(t1,text="°/s:",font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT,padx=(8,0))
        self.turn_vel=tk.IntVar(value=200);make_spin(t1,self.turn_vel,50,500,50).pack(side=tk.LEFT,padx=3)
        t2=ttk.Frame(tf);t2.pack(pady=3)
        ttk.Button(t2,text="↻ CW",command=self.do_cw,style="C.TButton",width=8).pack(side=tk.LEFT,padx=2)
        ttk.Button(t2,text="↺ CCW",command=self.do_ccw,style="M.TButton",width=8).pack(side=tk.LEFT,padx=2)
        ttk.Button(t2,text="90°",command=lambda:self._preset_turn(90),width=5).pack(side=tk.LEFT,padx=1)
        ttk.Button(t2,text="180°",command=lambda:self._preset_turn(180),width=5).pack(side=tk.LEFT,padx=1)

        # ── Control + Run Status ──
        ctrl=make_card(left,"▸ CONTROL");ctrl.pack(fill=tk.X,pady=3)
        cr=ttk.Frame(ctrl);cr.pack()
        ttk.Button(cr,text="■ STOP",command=self.do_stop,style="R.TButton",width=8).pack(side=tk.LEFT,padx=3)
        ttk.Button(cr,text="CLEAR",command=self.do_clear,width=8).pack(side=tk.LEFT,padx=3)
        ttk.Button(cr,text="💾 SAVE",command=self._save_run,width=8).pack(side=tk.LEFT,padx=3)
        ttk.Button(cr,text="📂 LOAD",command=self._load_run,width=8).pack(side=tk.LEFT,padx=3)
        # Run status
        self.run_lbl=tk.Label(ctrl,text="[ IDLE ]",font=("Consolas",10,"bold"),
                              fg=TEXT_DIM,bg=BG_CARD)
        self.run_lbl.pack(pady=2)
        self.snap_lbl=tk.Label(ctrl,text="",font=("Consolas",7),fg=TEXT_DIM,bg=BG_CARD)
        self.snap_lbl.pack()

        # ── Analysis ──
        res=make_card(left,"▸ ANALYSIS");res.pack(fill=tk.X,pady=3)
        self.res_txt=make_ro(res,5,NEON_YELLOW);self.res_txt.pack(fill=tk.X)
        wr(self.res_txt,"  ARM → preset or custom → auto-freeze → metrics")

        # RIGHT: sub-notebook with plots and data tabs
        sub=ttk.Notebook(right,style='Sub.TNotebook')
        sub.pack(fill=tk.BOTH,expand=True)
        pf=ttk.Frame(sub);sub.add(pf,text="  📊 PLOTS  ")
        self.plot_vel=HudPlot(pf,440,200,"VELOCITY (mm/s)",-400,400,
                              [{'name':'vL','color':NEON_GREEN},{'name':'vR','color':NEON_CYAN},
                               {'name':'vT','color':NEON_YELLOW},{'name':'vA','color':'#80ff80'}],auto_scale=True)
        self.plot_vel._visible['vA']=False
        self.plot_cmd=HudPlot(pf,440,200,"MOTOR CMD (mV)",-8000,8000,
                              [{'name':'cL','color':NEON_GREEN},{'name':'cR','color':NEON_CYAN}],auto_scale=True)
        df=ttk.Frame(sub);sub.add(df,text="  📋 DATA  ")
        self.data_view=DataViewer(df)

    def _make_snap(self, mode, dist, vel, acc):
        return RunSnapshot(lab=3, mode=mode, distance_mm=dist,
                           vel_mmps=vel, accel_mmps2=acc)

    def _begin(self, snap):
        """Clear old plots, start new run."""
        self.plot_vel.clear();self.plot_cmd.clear();self.data_view.clear()
        self.mgr.begin_run(snap)
        self.run_lbl.config(text=f"● RUN #{self.mgr._run_counter} — RUNNING",fg=NEON_RED)
        self.snap_lbl.config(text=snap.params_str(),fg=NEON_CYAN)
        wr(self.res_txt,"  Recording...")

    def do_fwd(self):
        d,v,a=self.fwd_dist.get(),self.fwd_vel.get(),self.fwd_acc.get()
        self._begin(self._make_snap('fwd',d,v,a));self.ble.send_cmd(cmd_trap_fwd(d,v,a))
    def do_back(self):
        d,v,a=self.fwd_dist.get(),self.fwd_vel.get(),self.fwd_acc.get()
        self._begin(self._make_snap('back',-d,v,a));self.ble.send_cmd(cmd_trap_fwd(-d,v,a))
    def do_cw(self):
        d,v,a=self.turn_deg.get(),self.turn_vel.get(),self.fwd_acc.get()
        self._begin(self._make_snap('cw',d,v,a));self.ble.send_cmd(cmd_trap_turn(d,v,a))
    def do_ccw(self):
        d,v,a=self.turn_deg.get(),self.turn_vel.get(),self.fwd_acc.get()
        self._begin(self._make_snap('ccw',-d,v,a));self.ble.send_cmd(cmd_trap_turn(-d,v,a))
    def do_stop(self):
        self.ble.send_cmd(cmd_stop())
        if self.mgr.state==RunState.RUNNING:
            self.mgr.freeze()
            self._on_frozen()
    def _preset_fwd(self,mm):
        v,a=self.fwd_vel.get(),self.fwd_acc.get()
        self._begin(self._make_snap('fwd',mm,v,a));self.ble.send_cmd(cmd_trap_fwd(mm,v,a))
    def _preset_turn(self,deg):
        v,a=self.turn_vel.get(),self.fwd_acc.get()
        self._begin(self._make_snap('cw',deg,v,a));self.ble.send_cmd(cmd_trap_turn(deg,v,a))
    def do_clear(self):
        self.mgr.clear()
        self.plot_vel.clear();self.plot_cmd.clear();self.data_view.clear()
        self.run_lbl.config(text="[ IDLE ]",fg=TEXT_DIM)
        self.snap_lbl.config(text="")
        wr(self.res_txt,"  Cleared.")

    def handle_fast(self,data):
        f=parse_fast(data)
        if not f:return
        vA=(f['velL_mmps']+f['velR_mmps'])//2
        self.plot_vel.add_point(vL=f['velL_mmps'],vR=f['velR_mmps'],vT=f.get('v_target_mmps',0),vA=vA)
        self.plot_cmd.add_point(cL=f['vCmdL_mV'],cR=f['vCmdR_mV'])
        self.plot_vel.redraw();self.plot_cmd.redraw()
        if self.mgr.state==RunState.RUNNING:
            self.mgr.add_sample(f)
            self.data_view.add_sample(f)
            n=len(self.mgr.current.samples) if self.mgr.current else 0
            self.run_lbl.config(text=f"● RUN #{self.mgr._run_counter} n={n}")
            if self.mgr.check_auto_freeze(f):
                self._on_frozen()

    def _on_frozen(self):
        """Called when run transitions to FROZEN."""
        self.data_view.finish()
        m=self.mgr.get_metrics()
        sn=self.mgr.get_snapshot()
        if not m or not sn:return
        # Update run label
        g,sc=m.grade()
        self.run_lbl.config(text=f"❄ FROZEN #{self.mgr._run_counter}  [{g}:{sc}]",
                            fg=NEON_CYAN)
        # Freeze plots with target overlay
        tv=abs(sn.vel_mmps)
        if tv>0:
            self.plot_vel.set_overlay_lines([(tv,NEON_YELLOW,f'target {tv}'),
                                              (-tv,'#ffea0060',f'target {-tv}')])
            # Cruise band
            band=tv*0.05
            self.plot_vel.set_highlight_regions([])
        self.plot_vel.freeze()
        self.plot_cmd.freeze()
        # Show full metrics
        lines=[f"  {sn.params_str()}  {sn.gains_str()}"]
        lines.append(m.summary_str())
        lines.append(f"  ── Grade: {g} ({sc}/100) ──")
        wr(self.res_txt,"\n".join(lines))

    def _save_run(self):
        if not self.mgr.current or not self.mgr.current.samples:return
        d=filedialog.askdirectory(title="Save Run To...")
        if not d:return
        cp,jp=self.mgr.current.export_both(d)
        wr(self.res_txt,f"  ✓ Saved: {os.path.basename(cp)}")

    def _load_run(self):
        p=filedialog.askopenfilename(title="Load Run CSV",
            filetypes=[("CSV","*.csv"),("All","*.*")])
        if not p:return
        jp=p.replace('.csv','.json')
        rec=load_run(p,jp if os.path.exists(jp) else None)
        if not rec or not rec.samples:wr(self.res_txt,"  ✖ Empty file");return
        # Replay into plots
        self.plot_vel.clear();self.plot_cmd.clear()
        vL=[s.get('velL_mmps',0)for s in rec.samples]
        vR=[s.get('velR_mmps',0)for s in rec.samples]
        vT=[s.get('v_target_mmps',0)for s in rec.samples]
        vA=[(l+r)//2 for l,r in zip(vL,vR)]
        cL=[s.get('vCmdL_mV',0)for s in rec.samples]
        cR=[s.get('vCmdR_mV',0)for s in rec.samples]
        self.plot_vel.load_data({'vL':vL,'vR':vR,'vT':vT,'vA':vA})
        self.plot_cmd.load_data({'cL':cL,'cR':cR})
        tv=abs(rec.snap.vel_mmps)
        if tv>0:
            self.plot_vel.set_overlay_lines([(tv,NEON_YELLOW,f'target {tv}')])
        self.plot_vel.freeze();self.plot_cmd.freeze()
        m=rec.metrics
        if m:
            g,sc=m.grade()
            self.run_lbl.config(text=f"📂 REPLAY  [{g}:{sc}]",fg=NEON_MAGENTA)
            self.snap_lbl.config(text=rec.snap.params_str())
            wr(self.res_txt,f"  {rec.snap.params_str()}\n{m.summary_str()}\n  ── Grade: {g} ({sc}/100) ──")

    def handle_rsp(self,data):
        rsp=parse_rsp(data)
        if rsp and rsp['status']!=RSP_OK:wr(self.res_txt,f"  ✖ {RSP_NAMES.get(rsp['status'],'?')}")


# ═══════════════════════════════════════════════════════════════════════════════
# Lab 4 — PID Tuning (bigger plots + data viewer + CSV export)
# ═══════════════════════════════════════════════════════════════════════════════
class Lab4Tab:
    """Lab 4 — PID Tuning with Run Lifecycle, Auto-Metrics, History, Export/Replay."""

    def __init__(self,parent,ble):
        self.ble=ble; self.frame=ttk.Frame(parent,padding=5)
        self.mgr=RunManager()

        left=ttk.Frame(self.frame); left.pack(side=tk.LEFT,fill=tk.BOTH,expand=True,padx=4)
        right=ttk.Frame(self.frame); right.pack(side=tk.LEFT,fill=tk.BOTH,expand=True,padx=4)

        # ── PID GAINS ──
        gf=make_card(left,"▸ PID GAINS (live tune via ↑)"); gf.pack(fill=tk.X,pady=3)
        g1=tk.Frame(gf,bg=BG_CARD); g1.pack(fill=tk.X,pady=1)
        tk.Label(g1,text="Vel Kp:",font=("Consolas",8),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT)
        self.vel_kp=tk.IntVar(value=550); make_spin(g1,self.vel_kp,0,5000,50,5).pack(side=tk.LEFT,padx=2)
        tk.Label(g1,text="Kd:",font=("Consolas",8),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT,padx=(4,0))
        self.vel_kd=tk.IntVar(value=800); make_spin(g1,self.vel_kd,0,5000,50,5).pack(side=tk.LEFT,padx=2)
        tk.Label(g1,text="max:",font=("Consolas",8),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT,padx=(4,0))
        self.vel_max=tk.IntVar(value=2000); make_spin(g1,self.vel_max,100,5000,100,5).pack(side=tk.LEFT,padx=2)
        ttk.Button(g1,text="↑",command=self.upload_vel,style="G.TButton",width=2).pack(side=tk.LEFT,padx=2)
        g2=tk.Frame(gf,bg=BG_CARD); g2.pack(fill=tk.X,pady=1)
        tk.Label(g2,text="Hdg Kp:",font=("Consolas",8),fg=NEON_MAGENTA,bg=BG_CARD).pack(side=tk.LEFT)
        self.hdg_kp=tk.IntVar(value=500); make_spin(g2,self.hdg_kp,0,5000,50,5).pack(side=tk.LEFT,padx=2)
        tk.Label(g2,text="Ki:",font=("Consolas",8),fg=NEON_MAGENTA,bg=BG_CARD).pack(side=tk.LEFT,padx=(4,0))
        self.hdg_ki=tk.IntVar(value=10); make_spin(g2,self.hdg_ki,0,500,5,4).pack(side=tk.LEFT,padx=2)
        tk.Label(g2,text="Kd:",font=("Consolas",8),fg=NEON_MAGENTA,bg=BG_CARD).pack(side=tk.LEFT,padx=(4,0))
        self.hdg_kd=tk.IntVar(value=20); make_spin(g2,self.hdg_kd,0,5000,10,4).pack(side=tk.LEFT,padx=2)
        tk.Label(g2,text="max:",font=("Consolas",8),fg=NEON_MAGENTA,bg=BG_CARD).pack(side=tk.LEFT,padx=(4,0))
        self.hdg_max=tk.IntVar(value=1500); make_spin(g2,self.hdg_max,100,5000,100,5).pack(side=tk.LEFT,padx=2)
        ttk.Button(g2,text="↑",command=self.upload_hdg,style="M.TButton",width=2).pack(side=tk.LEFT,padx=2)
        g3=tk.Frame(gf,bg=BG_CARD); g3.pack(fill=tk.X,pady=1)
        tk.Label(g3,text="Trim mV:",font=("Consolas",8,"bold"),fg=NEON_YELLOW,bg=BG_CARD).pack(side=tk.LEFT)
        self.trim_mv=tk.IntVar(value=0); make_spin(g3,self.trim_mv,-500,500,10,5).pack(side=tk.LEFT,padx=2)
        tk.Label(g3,text="(+ = fix left drift)",font=("Consolas",7),fg=TEXT_DIM,bg=BG_CARD).pack(side=tk.LEFT,padx=4)
        tk.Label(gf,text="×100 format: 550=5.50  Upload ↑ sends live",
                 font=("Consolas",7),fg=TEXT_DIM,bg=BG_CARD).pack(anchor=tk.W)

        # ── STRAIGHT ──
        sf=make_card(left,"▸ PID STRAIGHT [ARM]"); sf.pack(fill=tk.X,pady=3)
        r1=tk.Frame(sf,bg=BG_CARD); r1.pack(fill=tk.X,pady=2)
        tk.Label(r1,text="mm:",font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT)
        self.fwd_dist=tk.IntVar(value=180); make_spin(r1,self.fwd_dist,10,2000,10).pack(side=tk.LEFT,padx=3)
        tk.Label(r1,text="vel:",font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT,padx=(6,0))
        self.fwd_vel=tk.IntVar(value=200); make_spin(r1,self.fwd_vel,50,800,50).pack(side=tk.LEFT,padx=3)
        tk.Label(r1,text="acc:",font=("Consolas",9),fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT,padx=(6,0))
        self.fwd_acc=tk.IntVar(value=500); make_spin(r1,self.fwd_acc,100,2000,100).pack(side=tk.LEFT,padx=3)
        r2=ttk.Frame(sf); r2.pack(pady=3)
        ttk.Button(r2,text="▲ FWD",command=self.do_fwd,style="G.TButton",width=7).pack(side=tk.LEFT,padx=2)
        ttk.Button(r2,text="▼ BACK",command=self.do_back,style="O.TButton",width=7).pack(side=tk.LEFT,padx=2)
        ttk.Button(r2,text="1C",command=lambda:self.do_cells(1),width=4).pack(side=tk.LEFT,padx=1)
        ttk.Button(r2,text="2C",command=lambda:self.do_cells(2),width=4).pack(side=tk.LEFT,padx=1)
        ttk.Button(r2,text="■ STOP",command=self.do_stop,style="R.TButton",width=7).pack(side=tk.LEFT,padx=2)

        # ── TURN ──
        tf=make_card(left,"▸ PID TURN [ARM]"); tf.pack(fill=tk.X,pady=3)
        t2=ttk.Frame(tf); t2.pack(pady=3)
        ttk.Button(t2,text="90° CW",command=lambda:self.do_turn(90,1),style="C.TButton",width=8).pack(side=tk.LEFT,padx=2)
        ttk.Button(t2,text="90° CCW",command=lambda:self.do_turn(90,-1),style="M.TButton",width=8).pack(side=tk.LEFT,padx=2)
        ttk.Button(t2,text="180° CW",command=lambda:self.do_turn(180,1),width=8).pack(side=tk.LEFT,padx=2)
        ttk.Button(t2,text="180° CCW",command=lambda:self.do_turn(180,-1),width=8).pack(side=tk.LEFT,padx=2)

        # ── RUN STATUS ──
        rsf=make_card(left,"▸ RUN STATUS"); rsf.pack(fill=tk.X,pady=3)
        self.run_lbl=tk.Label(rsf,text="[ IDLE ]",font=("Consolas",11,"bold"),
                              fg=TEXT_DIM,bg=BG_CARD)
        self.run_lbl.pack(anchor=tk.W,padx=8)
        rs_btns=tk.Frame(rsf,bg=BG_CARD); rs_btns.pack(fill=tk.X,pady=3)
        ttk.Button(rs_btns,text="🗑 CLEAR",command=self.do_clear,width=8).pack(side=tk.LEFT,padx=2)
        ttk.Button(rs_btns,text="💾 SAVE",command=self.do_save,style="G.TButton",width=8).pack(side=tk.LEFT,padx=2)
        ttk.Button(rs_btns,text="📂 LOAD",command=self.do_load,width=8).pack(side=tk.LEFT,padx=2)
        cmp_row=tk.Frame(rsf,bg=BG_CARD); cmp_row.pack(fill=tk.X,pady=1)
        ttk.Button(cmp_row,text="L3 1Cell",command=self.cmp_lab3,width=8).pack(side=tk.LEFT,padx=2)
        ttk.Button(cmp_row,text="L4 1Cell",command=self.cmp_lab4,style="G.TButton",width=8).pack(side=tk.LEFT,padx=2)
        ttk.Button(cmp_row,text="⚖ COMPARE",command=self.do_compare,style="C.TButton",width=10).pack(side=tk.LEFT,padx=2)

        # ── SNAPSHOT (gains@run) ──
        snf=make_card(left,"▸ SNAPSHOT (gains @ this run)"); snf.pack(fill=tk.X,pady=3)
        self.snap_txt=make_ro(snf,4,NEON_YELLOW); self.snap_txt.pack(fill=tk.X)
        wr(self.snap_txt,"  (จะแสดงค่า gains ที่ใช้หลังกด run)")

        # ═════════ RIGHT: sub-notebook (Plots / Metrics / Data / History) ═════════
        sub=ttk.Notebook(right,style='Sub.TNotebook')
        sub.pack(fill=tk.BOTH,expand=True)

        pf=ttk.Frame(sub); sub.add(pf,text="  📊 PLOTS  ")
        self.plot_vel=HudPlot(pf,440,200,"VELOCITY (mm/s)",-400,400,
                              [{'name':'vL','color':NEON_GREEN},{'name':'vR','color':NEON_CYAN},
                               {'name':'vT','color':NEON_YELLOW},{'name':'vA','color':'#80ff80'}],auto_scale=True)
        self.plot_vel._visible['vA']=False  # vAvg hidden by default
        self.plot_cmd=HudPlot(pf,440,180,"MOTOR CMD (mV)",-8000,8000,
                              [{'name':'cL','color':NEON_GREEN},{'name':'cR','color':NEON_CYAN}],auto_scale=True)
        self.plot_hdg=HudPlot(pf,440,130,"GYRO Z (°/s)",-300,300,
                              [{'name':'gz','color':NEON_MAGENTA}],auto_scale=True)

        mf=ttk.Frame(sub); sub.add(mf,text="  📐 METRICS  ")
        self.metrics_txt=make_ro(mf,16,NEON_CYAN); self.metrics_txt.pack(fill=tk.BOTH,expand=True)
        wr(self.metrics_txt,"  วิ่ง 1 ครั้ง → Freeze → Auto-compute metrics\n\n"
           "  ★ Rise time / Overshoot / Settling time\n"
           "  ★ SSE / L-R mismatch / Yaw drift\n\n"
           "  Target velocity คำนวณจาก trapezoid profile\n"
           "  เส้นสีเหลือง = target บนกราฟ velocity")

        df=ttk.Frame(sub); sub.add(df,text="  📋 DATA  ")
        self.data_view=DataViewer(df)

        hf=ttk.Frame(sub); sub.add(hf,text="  📜 HISTORY  ")
        hdr=tk.Frame(hf,bg=BG_CARD); hdr.pack(fill=tk.X)
        tk.Label(hdr,text="Run History (latest first)",font=("Consolas",9,"bold"),
                 fg=NEON_CYAN,bg=BG_CARD).pack(side=tk.LEFT,padx=4)
        self.hist_txt=make_ro(hf,18,NEON_GREEN); self.hist_txt.pack(fill=tk.BOTH,expand=True)

        # ── Settings tab: auto-save + plot toggles ──
        setf=ttk.Frame(sub); sub.add(setf,text="  ⚙ SETTINGS  ")
        # Auto-save
        asf=make_card(setf,"▸ AUTO-SAVE"); asf.pack(fill=tk.X,pady=3)
        ar=tk.Frame(asf,bg=BG_CARD); ar.pack(fill=tk.X)
        self._autosave=tk.BooleanVar(value=False)
        tk.Checkbutton(ar,text="Auto-save every run",variable=self._autosave,
                        font=("Consolas",9),fg=NEON_GREEN,bg=BG_CARD,selectcolor=BG_INPUT,
                        activebackground=BG_CARD,activeforeground=NEON_GREEN).pack(side=tk.LEFT)
        self._autosave_dir=tk.StringVar(value="")
        ttk.Button(ar,text="📁 DIR",command=self._pick_autosave_dir,width=6).pack(side=tk.LEFT,padx=4)
        self._autosave_lbl=tk.Label(asf,text="  (not set)",font=("Consolas",7),fg=TEXT_DIM,bg=BG_CARD)
        self._autosave_lbl.pack(anchor=tk.W)
        # Plot toggles
        ptf=make_card(setf,"▸ PLOT LINES"); ptf.pack(fill=tk.X,pady=3)
        self._tog={}
        for row_data in [
            [("velL",NEON_GREEN,"vL"),("velR",NEON_CYAN,"vR"),
             ("vTarget",NEON_YELLOW,"vT"),("vAvg","#80ff80","vA")],
            [("cmdL",NEON_GREEN,"cL"),("cmdR",NEON_CYAN,"cR")],
            [("gyroZ",NEON_MAGENTA,"gz")]
        ]:
            tr=tk.Frame(ptf,bg=BG_CARD); tr.pack(fill=tk.X,pady=1)
            for label,color,key in row_data:
                v=tk.BooleanVar(value=(key!='vA'))  # vAvg off by default
                self._tog[key]=v
                tk.Checkbutton(tr,text=label,variable=v,font=("Consolas",8),
                    fg=color,bg=BG_CARD,selectcolor=BG_INPUT,
                    activebackground=BG_CARD,activeforeground=color,
                    command=lambda k=key,var=v:self._toggle_line(k,var.get())).pack(side=tk.LEFT,padx=3)
        tk.Label(ptf,text="vAvg = (velL+velR)/2 — off by default",
                 font=("Consolas",7),fg=TEXT_DIM,bg=BG_CARD).pack(anchor=tk.W)

        # Target profile cache
        self._target_profile=[]

    # ═══════════════════════════════════════════════════════════════════════════
    # Settings helpers
    # ═══════════════════════════════════════════════════════════════════════════
    def _pick_autosave_dir(self):
        d=filedialog.askdirectory(title="Auto-save Directory")
        if d:
            self._autosave_dir.set(d)
            self._autosave_lbl.config(text=f"  📁 {d}",fg=NEON_GREEN)

    def _toggle_line(self,key,visible):
        """Toggle plot line visibility from checkbox."""
        plot_map={'vL':'vel','vR':'vel','vT':'vel','vA':'vel','cL':'cmd','cR':'cmd','gz':'hdg'}
        p=plot_map.get(key)
        if p=='vel': self.plot_vel.set_line_visible(key,visible)
        elif p=='cmd': self.plot_cmd.set_line_visible(key,visible)
        elif p=='hdg': self.plot_hdg.set_line_visible(key,visible)

    # ═══════════════════════════════════════════════════════════════════════════
    # Gain snapshots & upload
    # ═══════════════════════════════════════════════════════════════════════════
    def _make_snap(self, mode, dist, vel, acc):
        return RunSnapshot(lab=4, mode=mode, distance_mm=dist,
                           vel_mmps=vel, accel_mmps2=acc,
                           vel_kp=self.vel_kp.get(), vel_kd=self.vel_kd.get(),
                           vel_max_mv=self.vel_max.get(),
                           hdg_kp=self.hdg_kp.get(), hdg_ki=self.hdg_ki.get(),
                           hdg_kd=self.hdg_kd.get(), hdg_max_mv=self.hdg_max.get(),
                           trim_mv=self.trim_mv.get())

    def upload_vel(self): self.ble.send_cmd(cmd_set_pd_linear(self.vel_kp.get(),self.vel_kd.get(),self.vel_max.get()))
    def upload_hdg(self): self.ble.send_cmd(cmd_set_pd_steering(self.hdg_kp.get(),self.hdg_ki.get(),self.hdg_kd.get(),self.hdg_max.get(),self.trim_mv.get()))

    # ═══════════════════════════════════════════════════════════════════════════
    # Run lifecycle
    # ═══════════════════════════════════════════════════════════════════════════
    def _begin(self, snap):
        self.plot_vel.clear(); self.plot_cmd.clear(); self.plot_hdg.clear()
        self.data_view.clear()
        self.mgr.begin_run(snap)
        self.run_lbl.config(text=f"● RUN #{self.mgr._run_counter} — RUNNING",fg=NEON_MAGENTA)
        # Show gains snapshot
        wr(self.snap_txt, f"  {snap.params_str()}\n  {snap.gains_str()}")
        # Generate target profile for straight runs
        if snap.distance_mm != 0 and snap.vel_mmps > 0:
            p=TrapProfile(snap.distance_mm, snap.vel_mmps, snap.accel_mmps2 or 500)
            self._target_profile=p.sample_array()
            tv=abs(snap.vel_mmps)
            self.plot_vel.set_overlay_lines([(tv,NEON_YELLOW,f'target {tv}')])
        else:
            self._target_profile=[]
            self.plot_vel.set_overlay_lines([])

    def _on_frozen(self):
        self.data_view.finish()
        m=self.mgr.get_metrics()
        sn=self.mgr.get_snapshot()
        if not m or not sn: return
        g,sc=m.grade()
        self.run_lbl.config(text=f"❄ FROZEN #{self.mgr._run_counter}  [{g}:{sc}]",fg=NEON_CYAN)
        # Freeze plots
        n=m.n_samples
        if n>5 and m.i_cruise_start<m.i_cruise_end:
            self.plot_vel.set_highlight_regions([
                (m.i_cruise_start/n, m.i_cruise_end/n, '#0a1a0a')])
        # Trapezoid curve overlay — fallback when firmware v_target is all zeros
        samples=self.mgr.current.samples if self.mgr.current else []
        has_fw_vt = any(abs(s.get('v_target_mmps',0)) > 1 for s in samples)
        if not has_fw_vt and self._target_profile and n > 2:
            # Align: pad/trim target to match sample count, offset by motion start
            tp=self._target_profile
            aligned=[0.0]*n
            for i in range(min(len(tp), n - m.i_motion_start)):
                idx=m.i_motion_start + i
                if idx < n: aligned[idx]=tp[i]
            self.plot_vel.add_overlay_series('trapez', aligned, NEON_YELLOW)
        self.plot_vel.freeze();self.plot_cmd.freeze();self.plot_hdg.freeze()
        # Metrics tab
        lines=[f"  {sn.params_str()}",f"  {sn.gains_str()}","",m.summary_str(),
               f"\n  ── Grade: {g} ({sc}/100) ──"]
        if not has_fw_vt and self._target_profile:
            lines.append("  ⚠ v_target from GUI trapezoid (FW didn't send)")
        wr(self.metrics_txt,"\n".join(lines))
        # Auto-save
        if self._autosave.get() and self._autosave_dir.get() and self.mgr.current:
            try:
                cp,jp=self.mgr.current.export_both(self._autosave_dir.get())
                self._autosave_lbl.config(text=f"  ✓ {os.path.basename(cp)}",fg=NEON_GREEN)
            except Exception as e:
                self._autosave_lbl.config(text=f"  ✖ {e}",fg=NEON_RED)
        # History
        self._refresh_history()

    # ═══════════════════════════════════════════════════════════════════════════
    # Commands
    # ═══════════════════════════════════════════════════════════════════════════
    def do_fwd(self):
        d,v,a=self.fwd_dist.get(),self.fwd_vel.get(),self.fwd_acc.get()
        self._begin(self._make_snap('fwd',d,v,a))
        self.ble.send_cmd(cmd_pid_straight(d,v,a))
    def do_back(self):
        d,v,a=self.fwd_dist.get(),self.fwd_vel.get(),self.fwd_acc.get()
        self._begin(self._make_snap('back',-d,v,a))
        self.ble.send_cmd(cmd_pid_straight(-d,v,a))
    def do_cells(self,n):
        v=self.fwd_vel.get()
        self._begin(self._make_snap('cell',n*180,v,500))
        self.ble.send_cmd(cmd_pid_move_cells(n,v))
    def do_turn(self,deg,d):
        dr='cw'if d>0 else'ccw'
        self._begin(self._make_snap(f'{dr}{deg}',0,0,0))
        if deg==90: self.ble.send_cmd(cmd_pid_turn_90(d))
        else: self.ble.send_cmd(cmd_pid_turn_180(d))
    def do_stop(self):
        self.ble.send_cmd(cmd_stop())
        if self.mgr.state==RunState.RUNNING:
            self.mgr.freeze(); self._on_frozen()
    def cmp_lab3(self):
        v,a=self.fwd_vel.get(),self.fwd_acc.get()
        snap=RunSnapshot(lab=3,mode='cmp_ff',distance_mm=180,vel_mmps=v,accel_mmps2=a)
        self._begin(snap); self.ble.send_cmd(cmd_trap_fwd(180,v,a))
    def cmp_lab4(self):
        v,a=self.fwd_vel.get(),self.fwd_acc.get()
        self._begin(self._make_snap('cmp_pid',180,v,a))
        self.ble.send_cmd(cmd_pid_straight(180,v,a))

    # ═══════════════════════════════════════════════════════════════════════════
    # Clear / Save / Load
    # ═══════════════════════════════════════════════════════════════════════════
    def do_clear(self):
        self.mgr.clear()
        self.plot_vel.clear();self.plot_cmd.clear();self.plot_hdg.clear()
        self.data_view.clear();self._target_profile=[]
        self.run_lbl.config(text="[ IDLE ]",fg=TEXT_DIM)
        wr(self.snap_txt,"  (cleared)")
        wr(self.metrics_txt,"  (no data)")

    def do_save(self):
        if not self.mgr.current or not self.mgr.current.samples:
            wr(self.metrics_txt,"  ✖ No run to save"); return
        d=filedialog.askdirectory(title="Save Run To...")
        if not d: return
        cp,jp=self.mgr.current.export_both(d)
        wr(self.metrics_txt,f"  ✓ Saved: {os.path.basename(cp)}")

    def do_load(self):
        p=filedialog.askopenfilename(
            filetypes=[("CSV","*.csv"),("All","*.*")])
        if not p: return
        jp=p.replace('.csv','.json')
        rec=load_run(p, jp if os.path.exists(jp) else None)
        if not rec or not rec.samples:
            wr(self.metrics_txt,"  ✖ Empty file"); return
        self._replay_record(rec)

    def do_compare(self):
        """Load a reference run and overlay it (dimmed) on the current frozen plots."""
        p=filedialog.askopenfilename(title="Load Reference Run for Compare",
            filetypes=[("CSV","*.csv"),("All","*.*")])
        if not p: return
        jp=p.replace('.csv','.json')
        ref=load_run(p, jp if os.path.exists(jp) else None)
        if not ref or not ref.samples:
            wr(self.metrics_txt,"  ✖ Empty reference file"); return
        # Overlay reference velocity as dim lines
        rvL=[s.get('velL_mmps',0) for s in ref.samples]
        rvR=[s.get('velR_mmps',0) for s in ref.samples]
        rcL=[s.get('vCmdL_mV',0) for s in ref.samples]
        rcR=[s.get('vCmdR_mV',0) for s in ref.samples]
        self.plot_vel.add_overlay_series('refL', rvL, DIM_GREEN)
        self.plot_vel.add_overlay_series('refR', rvR, DIM_CYAN)
        self.plot_cmd.add_overlay_series('refL', rcL, DIM_GREEN)
        self.plot_cmd.add_overlay_series('refR', rcR, DIM_CYAN)
        self.plot_vel._do_redraw();self.plot_cmd._do_redraw()
        # Show comparison metrics
        rm=ref.metrics
        cm=self.mgr.get_metrics() if self.mgr.current else None
        lines=["  ── COMPARE ──"]
        if cm and rm:
            lines.append(f"  {'':12s} {'Current':>10s} {'Reference':>10s} {'Δ':>8s}")
            def cmp(n,cv,rv,fmt=".0f",unit=""): lines.append(f"  {n:12s} {cv:{fmt}}{unit:>3s} {rv:{fmt}}{unit:>3s} {cv-rv:+{fmt}}")
            cmp("Rise ms",cm.rise_time_ms,rm.rise_time_ms)
            cmp("Overshoot%",cm.overshoot_pct,rm.overshoot_pct,".1f","%")
            cmp("Settle ms",cm.settling_time_ms,rm.settling_time_ms)
            cmp("SSE mm/s",cm.sse_mmps,rm.sse_mmps,".1f")
            cmp("LR bias",cm.lr_bias_mmps,rm.lr_bias_mmps,".1f")
            cmp("Yaw drift",cm.yaw_drift_deg,rm.yaw_drift_deg,".2f","°")
            cg,cs=cm.grade(); rg,rs=rm.grade()
            lines.append(f"\n  Grade: {cg}({cs}) vs {rg}({rs})  {'✓ BETTER' if cs>rs else '✖ WORSE' if cs<rs else '= SAME'}")
        elif rm:
            lines.append(f"  Ref: {ref.snap.params_str()}")
            g,s=rm.grade(); lines.append(f"  Grade: {g}({s})")
        lines.append(f"  📂 {os.path.basename(p)}")
        wr(self.metrics_txt,"\n".join(lines))
    def _replay_record(self, rec):
        """Display a loaded RunRecord as frozen."""
        self.plot_vel.clear();self.plot_cmd.clear();self.plot_hdg.clear()
        vL=[s.get('velL_mmps',0) for s in rec.samples]
        vR=[s.get('velR_mmps',0) for s in rec.samples]
        vT=[s.get('v_target_mmps',0) for s in rec.samples]
        vA=[(l+r)//2 for l,r in zip(vL,vR)]
        cL=[s.get('vCmdL_mV',0) for s in rec.samples]
        cR=[s.get('vCmdR_mV',0) for s in rec.samples]
        gz=[s.get('gyroZ_dps10',0)/10.0 for s in rec.samples]
        tv=abs(rec.snap.vel_mmps)
        if tv: self.plot_vel.set_overlay_lines([(tv,NEON_YELLOW,f'target {tv}')])
        self.plot_vel.load_data({'vL':vL,'vR':vR,'vT':vT,'vA':vA})
        self.plot_cmd.load_data({'cL':cL,'cR':cR})
        self.plot_hdg.load_data({'gz':gz})
        m=rec.metrics
        if m:
            g,sc=m.grade()
            self.run_lbl.config(text=f"📂 REPLAY  [{g}:{sc}]",fg=NEON_ORANGE)
            wr(self.snap_txt,f"  {rec.snap.params_str()}\n  {rec.snap.gains_str()}")
            wr(self.metrics_txt,f"  {rec.snap.params_str()}\n{m.summary_str()}\n  ── Grade: {g} ({sc}/100) ──")
        self.data_view.clear()
        for s in rec.samples: self.data_view.add_sample(s)
        self.data_view.finish()

    def _refresh_history(self):
        runs=self.mgr.history[-15:]
        cur=self.mgr.current
        lines=[]
        for r in reversed(runs):
            m=r.metrics
            if not m: continue
            g,sc=m.grade()
            lines.append(f"  #{r.snap.run_id:3d} {r.snap.params_str():<30s} "
                        f"Rise={m.rise_time_ms:3.0f}ms OS={m.overshoot_pct:+.0f}% "
                        f"SSE={m.sse_mmps:+.1f} [{g}:{sc}]")
        if cur and cur.metrics:
            m=cur.metrics; g,sc=m.grade()
            lines.insert(0,f"  ★#{cur.snap.run_id:3d} {cur.snap.params_str():<30s} "
                        f"Rise={m.rise_time_ms:3.0f}ms OS={m.overshoot_pct:+.0f}% "
                        f"SSE={m.sse_mmps:+.1f} [{g}:{sc}]")
        wr(self.hist_txt,"\n".join(lines) if lines else "  (no runs yet)")

    # ═══════════════════════════════════════════════════════════════════════════
    # handle_fast — 50Hz telemetry
    # ═══════════════════════════════════════════════════════════════════════════
    def handle_fast(self, data):
        f=parse_fast(data)
        if not f: return
        vA=(f['velL_mmps']+f['velR_mmps'])//2
        self.plot_vel.add_point(vL=f['velL_mmps'],vR=f['velR_mmps'],vT=f.get('v_target_mmps',0),vA=vA)
        self.plot_cmd.add_point(cL=f['vCmdL_mV'],cR=f['vCmdR_mV'])
        self.plot_hdg.add_point(gz=f['gyroZ_dps10']/10.0)
        self.plot_vel.redraw();self.plot_cmd.redraw();self.plot_hdg.redraw()
        if self.mgr.state==RunState.RUNNING:
            self.mgr.add_sample(f)
            self.data_view.add_sample(f)
            n=len(self.mgr.current.samples) if self.mgr.current else 0
            self.run_lbl.config(text=f"● RUN #{self.mgr._run_counter} n={n}")
            if self.mgr.check_auto_freeze(f):
                self._on_frozen()

    def handle_rsp(self, data):
        rsp=parse_rsp(data)
        if rsp and rsp['status']!=RSP_OK:
            wr(self.metrics_txt,f"  ✖ {RSP_NAMES.get(rsp['status'],'?')}")


# ═══════════════════════════════════════════════════════════════════════════════
# Lab 5 — Maze Search (Flood Fill)
# ═══════════════════════════════════════════════════════════════════════════════
class MazeView:
    """16×16 maze grid — 3-state walls matching mazerunner-core reference.
    
    Wall states (derived from wall bits + visited flags):
      WALL    = wall bit set           → solid green line
      EXIT    = wall bit clear + seen  → no line (confirmed passage)
      UNKNOWN = wall bit clear + unseen → dotted dim line
      
    A wall is 'seen' if EITHER cell on that wall has been visited.
    This matches the reference's WallState: EXIT/WALL/UNKNOWN/VIRTUAL.
    """
    SZ = 16
    # Colors
    C_WALL    = '#00ff41'   # confirmed wall — bright green
    C_UNKN    = '#1a2a1a'   # unknown wall — very dim
    C_POST    = '#00aa30'   # corner posts
    C_BG_VIS  = '#0a1a0a'   # visited cell background
    C_BG_GOAL = '#1a0030'   # goal cell background
    C_BG      = '#08080c'   # unvisited cell background
    C_ROBOT   = NEON_MAGENTA
    C_FLOOD   = '#405040'   # flood value text

    def __init__(self, parent, cell_px=28):
        self.cp = cell_px
        # Canvas: 16 cells + 17 posts (post_width=3)
        self.pw = 3  # post pixel width
        total = self.SZ * cell_px + (self.SZ + 1) * self.pw
        self.total = total
        self.canvas = tk.Canvas(parent, width=total, height=total, bg=self.C_BG,
                                highlightbackground=BORDER, highlightthickness=1)
        self.canvas.pack(pady=4)
        # State
        self.walls = [[0]*self.SZ for _ in range(self.SZ)]    # walls[y][x] 4-bit
        self.flood = [[255]*self.SZ for _ in range(self.SZ)]
        self.visited = [[False]*self.SZ for _ in range(self.SZ)]
        self.robot_x = 0; self.robot_y = 0; self.robot_h = 0
        self.goal = (7,7,8,8)
        self._draw()

    def _cell_px(self, mx, my):
        """Convert maze coords (x,y) to canvas pixel (left, top) of cell interior."""
        cy = self.SZ - 1 - my  # flip Y
        px = self.pw + mx * (self.cp + self.pw)
        py = self.pw + cy * (self.cp + self.pw)
        return px, py

    def _post_px(self, px, py):
        """Convert post grid coords (0..16, 0..16) to canvas pixel of post top-left.
        Post (0,0) = bottom-left corner of maze = top-left of canvas after Y-flip."""
        cpy = self.SZ - py  # flip Y: post row 0 (bottom) → canvas row 16
        x = px * (self.cp + self.pw)
        y = cpy * (self.cp + self.pw)
        return x, y

    def _wall_state(self, x, y, direction):
        """Determine wall state: 'wall', 'exit', or 'unknown'.
        
        Reference logic from mazerunner-core:
          WALL = wall bit set (confirmed wall)
          EXIT = wall bit clear AND edge has been observed (either cell visited)
          UNKNOWN = wall bit clear AND edge has NOT been observed
        """
        w = self.walls[y][x]
        bit = 1 << direction  # N=0x01, E=0x02, S=0x04, W=0x08

        if w & bit:
            return 'wall'

        # Wall bit is clear — check if this edge has been observed
        this_visited = self.visited[y][x]

        # Find neighbor cell
        dx_t = [0, 1, 0, -1]  # N E S W
        dy_t = [1, 0, -1, 0]
        nx, ny = x + dx_t[direction], y + dy_t[direction]

        if 0 <= nx < self.SZ and 0 <= ny < self.SZ:
            neighbor_visited = self.visited[ny][nx]
        else:
            # Boundary — outer walls are always set in firmware,
            # so we shouldn't reach here for clear bits
            neighbor_visited = True

        if this_visited or neighbor_visited:
            return 'exit'
        return 'unknown'

    def _draw(self):
        c = self.canvas; c.delete("all")
        cp = self.cp; pw = self.pw
        gx0, gy0, gx1, gy1 = self.goal

        # ═══ Pass 1: Cell backgrounds + flood values ═══
        for y in range(self.SZ):
            for x in range(self.SZ):
                px, py = self._cell_px(x, y)
                fv = self.flood[y][x]
                is_goal = gx0 <= x <= gx1 and gy0 <= y <= gy1

                if is_goal:
                    bg = self.C_BG_GOAL
                elif self.visited[y][x]:
                    if fv < 255:
                        g = max(5, 40 - fv)
                        bg = f'#{g:02x}{g+10:02x}{g:02x}'
                    else:
                        bg = self.C_BG_VIS
                else:
                    bg = self.C_BG

                c.create_rectangle(px, py, px+cp, py+cp, fill=bg, outline='')

                # Flood value
                if fv < 255:
                    fc = NEON_YELLOW if is_goal else self.C_FLOOD
                    c.create_text(px+cp//2, py+cp//2, text=str(fv),
                                  fill=fc, font=("Consolas", 7))

        # ═══ Pass 2: Horizontal walls (North edge of each cell) ═══
        # For each cell, draw its North wall segment between two posts
        for y in range(self.SZ):
            for x in range(self.SZ):
                px, py = self._cell_px(x, y)
                # North wall of cell (x,y) = top edge of canvas cell
                state = self._wall_state(x, y, 0)  # DIR_N=0
                x1, y1 = px, py - pw       # just above cell
                x2, y2 = px + cp, py - pw
                if state == 'wall':
                    c.create_line(x1, y1+pw//2, x2, y2+pw//2,
                                  fill=self.C_WALL, width=pw)
                elif state == 'unknown':
                    c.create_line(x1, y1+pw//2, x2, y2+pw//2,
                                  fill=self.C_UNKN, width=1, dash=(2,3))

                # South wall of bottom row
                if y == 0:
                    state_s = self._wall_state(x, y, 2)  # DIR_S=2
                    sx1, sy1 = px, py + cp
                    sx2, sy2 = px + cp, py + cp
                    if state_s == 'wall':
                        c.create_line(sx1, sy1+pw//2, sx2, sy2+pw//2,
                                      fill=self.C_WALL, width=pw)
                    elif state_s == 'unknown':
                        c.create_line(sx1, sy1+pw//2, sx2, sy2+pw//2,
                                      fill=self.C_UNKN, width=1, dash=(2,3))

        # ═══ Pass 3: Vertical walls (West edge of each cell) ═══
        for y in range(self.SZ):
            for x in range(self.SZ):
                px, py = self._cell_px(x, y)
                # West wall
                state = self._wall_state(x, y, 3)  # DIR_W=3
                vx = px - pw
                if state == 'wall':
                    c.create_line(vx+pw//2, py, vx+pw//2, py+cp,
                                  fill=self.C_WALL, width=pw)
                elif state == 'unknown':
                    c.create_line(vx+pw//2, py, vx+pw//2, py+cp,
                                  fill=self.C_UNKN, width=1, dash=(2,3))

                # East wall of rightmost column
                if x == self.SZ - 1:
                    state_e = self._wall_state(x, y, 1)  # DIR_E=1
                    ex = px + cp
                    if state_e == 'wall':
                        c.create_line(ex+pw//2, py, ex+pw//2, py+cp,
                                      fill=self.C_WALL, width=pw)
                    elif state_e == 'unknown':
                        c.create_line(ex+pw//2, py, ex+pw//2, py+cp,
                                      fill=self.C_UNKN, width=1, dash=(2,3))

        # ═══ Pass 4: Corner posts (like 'o' in reference) ═══
        for py_post in range(self.SZ + 1):
            for px_post in range(self.SZ + 1):
                ppx, ppy = self._post_px(px_post, py_post)
                c.create_rectangle(ppx, ppy, ppx+pw, ppy+pw,
                                   fill=self.C_POST, outline='')

        # ═══ Pass 5: Robot marker ═══
        rpx_c, rpy_c = self._cell_px(self.robot_x, self.robot_y)
        rpx = rpx_c + cp // 2
        rpy = rpy_c + cp // 2
        r = cp // 3
        c.create_oval(rpx-r, rpy-r, rpx+r, rpy+r,
                       fill=self.C_ROBOT, outline=WHITE, width=1)
        # Direction arrow
        dx_arr = [0, 1, 0, -1]  # N E S W
        dy_arr = [-1, 0, 1, 0]
        h = self.robot_h
        ax = rpx + dx_arr[h] * r
        ay = rpy + dy_arr[h] * r
        c.create_line(rpx, rpy, ax, ay, fill=WHITE, width=2, arrow=tk.LAST)

    def update_from_telemetry(self, walls_2d, flood_2d, visited_2d, rx, ry, rh):
        self.walls = walls_2d
        self.flood = flood_2d
        self.visited = visited_2d
        self.robot_x = rx; self.robot_y = ry; self.robot_h = rh
        self._draw()

    def set_robot(self, x, y, h):
        self.robot_x = x; self.robot_y = y; self.robot_h = h
        self._draw()

    def clear(self):
        self.walls = [[0]*self.SZ for _ in range(self.SZ)]
        self.flood = [[255]*self.SZ for _ in range(self.SZ)]
        self.visited = [[False]*self.SZ for _ in range(self.SZ)]
        self.robot_x = 0; self.robot_y = 0; self.robot_h = 0
        self._draw()

    def demo_walls(self):
        """Load a small demo maze for testing GUI."""
        self.clear()
        # Outer walls are implicit (drawn via border)
        # Add some internal walls for demo
        demo = [
            (0,0,'N'), (1,0,'N'), (2,0,'E'), (3,1,'N'),
            (1,2,'E'), (2,2,'N'), (0,3,'E'), (3,3,'N'),
            (2,4,'E'), (4,2,'N'), (4,3,'E'),
        ]
        DIR = {'N':0x01,'E':0x02,'S':0x04,'W':0x08}
        for x,y,d in demo:
            if x < self.SZ and y < self.SZ:
                self.walls[y][x] |= DIR[d]
                self.visited[y][x] = True
        # Simple flood from goal
        from collections import deque
        q = deque()
        for gy in range(7,9):
            for gx in range(7,9):
                self.flood[gy][gx] = 0
                q.append((gx,gy))
        dx = [0,1,0,-1]; dy = [1,0,-1,0]; wb = [0x01,0x02,0x04,0x08]
        while q:
            cx,cy = q.popleft()
            cost = self.flood[cy][cx] + 1
            for i in range(4):
                if self.walls[cy][cx] & wb[i]: continue
                nx,ny = cx+dx[i], cy+dy[i]
                if 0<=nx<16 and 0<=ny<16 and cost < self.flood[ny][nx]:
                    self.flood[ny][nx] = cost
                    q.append((nx,ny))
        self._draw()


class Lab5Tab:
    def __init__(self, parent, ble):
        self.ble = ble; self.frame = ttk.Frame(parent, padding=5)
        left = ttk.Frame(self.frame); left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)
        right = ttk.Frame(self.frame); right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)

        # ── Controls ──
        cf = make_card(left, "▸ SEARCH CONTROL [ARM]"); cf.pack(fill=tk.X, pady=3)
        # Mode selector
        rm = tk.Frame(cf, bg=BG_CARD); rm.pack(fill=tk.X, pady=(2,0))
        tk.Label(rm, text="MODE:", font=("Consolas", 9, "bold"), fg=NEON_YELLOW, bg=BG_CARD).pack(side=tk.LEFT, padx=(0,4))
        self.search_mode = tk.IntVar(value=0)
        tk.Radiobutton(rm, text="Stop-and-Go", variable=self.search_mode, value=0,
                        font=("Consolas", 9), fg=NEON_GREEN, bg=BG_CARD, selectcolor=BG,
                        activebackground=BG_CARD, activeforeground=NEON_GREEN).pack(side=tk.LEFT, padx=2)
        tk.Radiobutton(rm, text="Continuous", variable=self.search_mode, value=1,
                        font=("Consolas", 9), fg=NEON_CYAN, bg=BG_CARD, selectcolor=BG,
                        activebackground=BG_CARD, activeforeground=NEON_CYAN).pack(side=tk.LEFT, padx=2)
        r1 = ttk.Frame(cf); r1.pack(pady=3)
        ttk.Button(r1, text="▶ SEARCH", command=self.do_search, style="G.TButton", width=10).pack(side=tk.LEFT, padx=2)
        ttk.Button(r1, text="▸ STEP", command=self.do_step, style="C.TButton", width=8).pack(side=tk.LEFT, padx=2)
        ttk.Button(r1, text="◁ RETURN", command=self.do_return, style="M.TButton", width=9).pack(side=tk.LEFT, padx=2)
        r2 = ttk.Frame(cf); r2.pack(pady=3)
        ttk.Button(r2, text="⟲ RESET", command=self.do_reset, style="O.TButton", width=8).pack(side=tk.LEFT, padx=2)
        ttk.Button(r2, text="■ STOP", command=self.do_stop, style="R.TButton", width=8).pack(side=tk.LEFT, padx=2)
        ttk.Button(r2, text="DEMO", command=self.do_demo, width=6).pack(side=tk.LEFT, padx=2)

        # ── Config: Goal & Start (set before SEARCH/STEP) ──
        gf = make_card(left, "▸ CONFIG (goal / start)"); gf.pack(fill=tk.X, pady=3)
        gr = tk.Frame(gf, bg=BG_CARD); gr.pack(fill=tk.X, pady=2)
        tk.Label(gr, text="Goal:", font=("Consolas", 8, "bold"), fg=NEON_YELLOW, bg=BG_CARD).pack(side=tk.LEFT)
        tk.Label(gr, text="X:", font=("Consolas", 8), fg=TEXT_DIM, bg=BG_CARD).pack(side=tk.LEFT, padx=(4,0))
        self.goal_x_min = tk.IntVar(value=7); make_spin(gr, self.goal_x_min, 0, 15, 1, w=3).pack(side=tk.LEFT)
        tk.Label(gr, text="-", font=("Consolas", 8), fg=TEXT_DIM, bg=BG_CARD).pack(side=tk.LEFT)
        self.goal_x_max = tk.IntVar(value=8); make_spin(gr, self.goal_x_max, 0, 15, 1, w=3).pack(side=tk.LEFT)
        tk.Label(gr, text="Y:", font=("Consolas", 8), fg=TEXT_DIM, bg=BG_CARD).pack(side=tk.LEFT, padx=(6,0))
        self.goal_y_min = tk.IntVar(value=7); make_spin(gr, self.goal_y_min, 0, 15, 1, w=3).pack(side=tk.LEFT)
        tk.Label(gr, text="-", font=("Consolas", 8), fg=TEXT_DIM, bg=BG_CARD).pack(side=tk.LEFT)
        self.goal_y_max = tk.IntVar(value=8); make_spin(gr, self.goal_y_max, 0, 15, 1, w=3).pack(side=tk.LEFT)
        sr = tk.Frame(gf, bg=BG_CARD); sr.pack(fill=tk.X, pady=2)
        tk.Label(sr, text="Start:", font=("Consolas", 8, "bold"), fg=NEON_CYAN, bg=BG_CARD).pack(side=tk.LEFT)
        tk.Label(sr, text="X:", font=("Consolas", 8), fg=TEXT_DIM, bg=BG_CARD).pack(side=tk.LEFT, padx=(2,0))
        self.start_x = tk.IntVar(value=0); make_spin(sr, self.start_x, 0, 15, 1, w=3).pack(side=tk.LEFT)
        tk.Label(sr, text="Y:", font=("Consolas", 8), fg=TEXT_DIM, bg=BG_CARD).pack(side=tk.LEFT, padx=(4,0))
        self.start_y = tk.IntVar(value=0); make_spin(sr, self.start_y, 0, 15, 1, w=3).pack(side=tk.LEFT)
        tk.Label(sr, text="Hdg:", font=("Consolas", 8), fg=TEXT_DIM, bg=BG_CARD).pack(side=tk.LEFT, padx=(4,0))
        self.start_hdg = tk.IntVar(value=0)
        hdg_frame = tk.Frame(sr, bg=BG_CARD); hdg_frame.pack(side=tk.LEFT, padx=2)
        for txt, val in [("N",0),("E",1),("S",2),("W",3)]:
            tk.Radiobutton(hdg_frame, text=txt, variable=self.start_hdg, value=val,
                            font=("Consolas", 8), fg=NEON_GREEN, bg=BG_CARD, selectcolor=BG,
                            activebackground=BG_CARD, activeforeground=NEON_GREEN,
                            indicatoron=0, padx=3, relief=tk.FLAT).pack(side=tk.LEFT)
        ab = tk.Frame(gf, bg=BG_CARD); ab.pack(fill=tk.X, pady=2)
        ttk.Button(ab, text="📤 APPLY CONFIG", command=self.do_apply_config, style="C.TButton", width=16).pack(side=tk.LEFT, padx=2)
        self.cfg_status = tk.Label(ab, text="default: goal(7-8,7-8) start(0,0,N)", font=("Consolas", 7), fg=TEXT_DIM, bg=BG_CARD)
        self.cfg_status.pack(side=tk.LEFT, padx=4)

        # ── Wall Follow (test mode) ──
        wff = make_card(left, "▸ WALL FOLLOW [ARM]"); wff.pack(fill=tk.X, pady=3)
        wfb = ttk.Frame(wff); wfb.pack(pady=3)
        ttk.Button(wfb, text="◁ LEFT WALL", command=self.do_wall_follow_l, style="C.TButton", width=12).pack(side=tk.LEFT, padx=2)
        ttk.Button(wfb, text="RIGHT WALL ▷", command=self.do_wall_follow_r, style="M.TButton", width=12).pack(side=tk.LEFT, padx=2)
        tk.Label(wff, text="ทดสอบเกาะกำแพง ซ้าย/ขวา — ARM ก่อนใช้งาน  STOP เมื่อจะหยุด",
                 font=("Consolas", 8), fg=TEXT_DIM, bg=BG_CARD).pack(anchor=tk.W, padx=4, pady=(0,3))

        # ── Status + Activity Log ──
        sf = make_card(left, "▸ STATUS"); sf.pack(fill=tk.X, pady=3)
        self.state_lbl = make_hud(sf, "[ IDLE ]", 14, TEXT_DIM); self.state_lbl.pack()
        self.pose_lbl = tk.Label(sf, text="Pos: (0,0) Hdg: N  Steps: 0  Visited: 0",
                                  font=("Consolas", 10), fg=NEON_CYAN, bg=BG_CARD)
        self.pose_lbl.pack(pady=2)
        # Activity Log — no wrap, 1 entry per line, scrollbar
        log_frame = tk.Frame(sf, bg=BG_INPUT); log_frame.pack(fill=tk.X)
        log_sb = tk.Scrollbar(log_frame, orient=tk.HORIZONTAL)
        self.activity_log = tk.Text(log_frame, height=10, font=("Consolas", 9),
                                     bg=BG_INPUT, fg=NEON_GREEN, relief=tk.FLAT,
                                     padx=4, pady=3, wrap=tk.NONE,
                                     insertbackground=NEON_GREEN, selectbackground=DIM_GREEN,
                                     highlightbackground=BORDER, highlightthickness=1,
                                     xscrollcommand=log_sb.set)
        log_sb.config(command=self.activity_log.xview)
        self.activity_log.pack(fill=tk.X, side=tk.TOP)
        log_sb.pack(fill=tk.X, side=tk.BOTTOM)
        self.activity_log.config(state=tk.DISABLED)
        # Tag styles for different message types
        self.activity_log.tag_config("sep", foreground=BORDER)
        self.activity_log.tag_config("goal", foreground=NEON_YELLOW)
        self.activity_log.tag_config("err", foreground=NEON_RED)
        self.activity_log.tag_config("done", foreground=NEON_GREEN)
        self._log_step = 0  # track step# for separator
        self._log_activity("Ready — STEP ทีละ cell / SEARCH วิ่งต่อเนื่อง")

        # ── Legend ──
        lf = make_card(left, "▸ LEGEND"); lf.pack(fill=tk.X, pady=3)
        tk.Label(lf, text="█ Green=wall  ●=robot  #=flood cost  ···=unknown\n"
                          "Dark=unvisited  Lit=visited  Purple=goal",
                 font=("Consolas", 8), fg=TEXT_DIM, bg=BG_CARD, justify=tk.LEFT).pack(anchor=tk.W)

        # ── Wall Sensor Debug ──
        wf = make_card(left, "▸ WALL SENSORS (debug)"); wf.pack(fill=tk.X, pady=3)
        wb = tk.Frame(wf, bg=BG_CARD); wb.pack(fill=tk.X, pady=2)
        ttk.Button(wb, text="📡 READ WALLS", command=self.do_read_walls, style="C.TButton", width=14).pack(side=tk.LEFT, padx=2)
        self.wall_auto = tk.BooleanVar(value=False)
        tk.Checkbutton(wb, text="Auto (1Hz)", variable=self.wall_auto, command=self._wall_auto_toggle,
                        font=("Consolas", 8), fg=TEXT_DIM, bg=BG_CARD, selectcolor=BG,
                        activebackground=BG_CARD, activeforeground=NEON_CYAN).pack(side=tk.LEFT, padx=4)
        # Wall display — top-down view of robot
        #        [FL] [FR]
        #   [L]    ●     [R]
        wd = tk.Frame(wf, bg=BG_CARD); wd.pack(pady=3)
        self.wall_lbl = {}
        # Row 0: front sensors
        r0 = tk.Frame(wd, bg=BG_CARD); r0.pack()
        tk.Label(r0, text="     ", font=("Consolas", 8), bg=BG_CARD).pack(side=tk.LEFT)
        self.wall_lbl['FL'] = tk.Label(r0, text="FL: ---mm", font=("Consolas", 9, "bold"), fg=TEXT_DIM, bg=BG_CARD, width=12)
        self.wall_lbl['FL'].pack(side=tk.LEFT, padx=2)
        self.wall_lbl['FR'] = tk.Label(r0, text="FR: ---mm", font=("Consolas", 9, "bold"), fg=TEXT_DIM, bg=BG_CARD, width=12)
        self.wall_lbl['FR'].pack(side=tk.LEFT, padx=2)
        # Row 1: side sensors + robot
        r1 = tk.Frame(wd, bg=BG_CARD); r1.pack()
        self.wall_lbl['L'] = tk.Label(r1, text="L: ---mm", font=("Consolas", 9, "bold"), fg=TEXT_DIM, bg=BG_CARD, width=10)
        self.wall_lbl['L'].pack(side=tk.LEFT, padx=2)
        tk.Label(r1, text="  ▲  ", font=("Consolas", 10, "bold"), fg=NEON_MAGENTA, bg=BG_CARD).pack(side=tk.LEFT, padx=6)
        self.wall_lbl['R'] = tk.Label(r1, text="R: ---mm", font=("Consolas", 9, "bold"), fg=TEXT_DIM, bg=BG_CARD, width=10)
        self.wall_lbl['R'].pack(side=tk.LEFT, padx=2)
        # Row 2: editable thresholds
        thr = tk.Frame(wf, bg=BG_CARD); thr.pack(fill=tk.X, padx=4, pady=(0,3))
        tk.Label(thr, text="Thresholds:", font=("Consolas", 8), fg=TEXT_DIM, bg=BG_CARD).pack(side=tk.LEFT)
        tk.Label(thr, text="Side ON:", font=("Consolas", 8), fg=NEON_YELLOW, bg=BG_CARD).pack(side=tk.LEFT, padx=(4,0))
        self.th_side_on = tk.IntVar(value=75); make_spin(thr, self.th_side_on, 10, 200, 5, w=4).pack(side=tk.LEFT)
        tk.Label(thr, text="OFF:", font=("Consolas", 8), fg=NEON_YELLOW, bg=BG_CARD).pack(side=tk.LEFT, padx=(3,0))
        self.th_side_off = tk.IntVar(value=90); make_spin(thr, self.th_side_off, 10, 200, 5, w=4).pack(side=tk.LEFT)
        tk.Label(thr, text="Front ON:", font=("Consolas", 8), fg=NEON_CYAN, bg=BG_CARD).pack(side=tk.LEFT, padx=(6,0))
        self.th_front_on = tk.IntVar(value=55); make_spin(thr, self.th_front_on, 10, 200, 5, w=4).pack(side=tk.LEFT)
        tk.Label(thr, text="OFF:", font=("Consolas", 8), fg=NEON_CYAN, bg=BG_CARD).pack(side=tk.LEFT, padx=(3,0))
        self.th_front_off = tk.IntVar(value=70); make_spin(thr, self.th_front_off, 10, 200, 5, w=4).pack(side=tk.LEFT)
        self.wall_info_lbl = tk.Label(wf, text="Heading: ?  Pos: (?,?)",
                                       font=("Consolas", 8), fg=TEXT_DIM, bg=BG_CARD)
        self.wall_info_lbl.pack(anchor=tk.W, padx=4, pady=(0,3))
        self._wall_auto_id = None

        # ── Plots (velocity during motion) ──
        pf = make_card(left, "▸ LIVE VELOCITY"); pf.pack(fill=tk.X, pady=3)
        self.plot_vel = HudPlot(pf, 420, 120, "", -400, 400,
                                [{'name':'vL','color':NEON_GREEN},{'name':'vR','color':NEON_CYAN}],
                                auto_scale=True)

        # ── RIGHT: Maze ──
        mf = make_card(right, "▸ MAZE MAP"); mf.pack(fill=tk.BOTH, expand=True, pady=3)
        self.maze_view = MazeView(mf, cell_px=28)

        # Internal state (simulate from telemetry)
        self._search_state = 0
        self._pose = {'x':0, 'y':0, 'h':0, 'steps':0}
        DIR_NAMES = ['N','E','S','W']
        self._dir_names = DIR_NAMES

    def do_search(self):
        mode = self.search_mode.get()
        cfg = self._get_config()
        self._sync_maze_goal(cfg)
        self._log_separator(f"SEARCH {'CONT' if mode else 'S&G'}")
        self._log_activity(f"goal({cfg['goal_x_min']}-{cfg['goal_x_max']},{cfg['goal_y_min']}-{cfg['goal_y_max']}) start({cfg['start_x']},{cfg['start_y']})")
        self.ble.send_cmd(cmd_search_run(mode=mode, cfg=cfg))
        self.frame.after(100, self._burst_fetch_maze)
        self.frame.after(500, self._burst_fetch_maze)
    def do_step(self):
        cfg = self._get_config()
        self._sync_maze_goal(cfg)
        self._log_step += 1
        self._log_separator(f"STEP #{self._log_step}")
        self.ble.send_cmd(cmd_search_step(cfg=cfg))
        self.frame.after(100, self._burst_fetch_maze)
        self.frame.after(800, self._burst_fetch_maze)
        self.frame.after(1500, self._burst_fetch_maze)
    def do_return(self):
        self._log_separator("RETURN")
        self.ble.send_cmd(cmd_search_return())
        self.frame.after(100, self._burst_fetch_maze)
    def do_reset(self):
        cfg = self._get_config()
        self._sync_maze_goal(cfg)
        self._log_separator("RESET")
        self._log_activity(f"goal({cfg['goal_x_min']}-{cfg['goal_x_max']},{cfg['goal_y_min']}-{cfg['goal_y_max']})")
        self._log_step = 0
        self.ble.send_cmd(cmd_search_reset(cfg=cfg))
        self.maze_view.clear()
        self._compute_local_flood()
        self.maze_view._draw()
        q0 = {'search_state':0,'mode':0,'x':0,'y':0,'heading':0,'steps':0,'visited_count':0}
        self._update_status(q0)

    def _sync_maze_goal(self, cfg):
        """Sync MazeView goal rectangle from config dict."""
        self.maze_view.goal = (cfg['goal_x_min'], cfg['goal_y_min'],
                               cfg['goal_x_max'], cfg['goal_y_max'])
    def do_stop(self):
        self._log_activity("■ STOP")
        self.ble.send_cmd(cmd_stop())
    def do_demo(self):
        self.maze_view.demo_walls()
        self._compute_local_flood()
        self.maze_view._draw()
    def do_read_walls(self): self.ble.send_cmd(cmd_search_read_walls())
    def do_wall_follow_l(self): self.ble.send_cmd(cmd_wall_follow_left())
    def do_wall_follow_r(self): self.ble.send_cmd(cmd_wall_follow_right())

    def _wall_auto_toggle(self):
        if self.wall_auto.get():
            self._wall_auto_tick()
        else:
            if self._wall_auto_id:
                self.frame.after_cancel(self._wall_auto_id)
                self._wall_auto_id = None

    def _wall_auto_tick(self):
        if self.wall_auto.get():
            self.do_read_walls()
            self._wall_auto_id = self.frame.after(1000, self._wall_auto_tick)

    def _update_wall_display(self, w):
        """Update wall sensor labels from parsed RSP data."""
        DIR_NAMES = ['N','E','S','W']
        # Read thresholds from spinbox variables (live-editable)
        s_on  = self.th_side_on.get()
        s_off = self.th_side_off.get()
        f_on  = self.th_front_on.get()
        f_off = self.th_front_off.get()
        # Sensor mapping: key → (mm_key, det_key, on_threshold, off_threshold)
        mapping = {
            'L':  ('left_mm',    'det_left',    s_on, s_off),
            'FL': ('front_l_mm', 'det_front_l', f_on, f_off),
            'FR': ('front_r_mm', 'det_front_r', f_on, f_off),
            'R':  ('right_mm',   'det_right',   s_on, s_off),
        }
        for key, (mm_key, det_key, on_th, off_th) in mapping.items():
            mm_val = w[mm_key]
            detected = w[det_key]
            # Color: red=wall, green=open, yellow=in hysteresis dead zone
            if mm_val < on_th:
                color = NEON_RED
            elif mm_val > off_th:
                color = NEON_GREEN
            else:
                color = NEON_YELLOW  # in dead zone
            icon = '█' if detected else '○'
            self.wall_lbl[key].config(text=f"{key}: {mm_val:3d}mm {icon}",
                                       fg=color)
        # Front combined result
        front_comb = w.get('det_front', w['det_front_l'] or w['det_front_r'])
        front_icon = '█ WALL' if front_comb else '○ OPEN'
        front_color = NEON_RED if front_comb else NEON_GREEN
        hdg_name = DIR_NAMES[w['heading']] if w['heading'] < 4 else '?'
        MODES = ['OR','AND','AVG','MIN','FL','FR']
        mode_str = MODES[0]  # default; actual mode from config.h compile-time
        self.wall_info_lbl.config(
            text=f"Front({mode_str}): {front_icon}  |  Hdg: {hdg_name}  Pos: ({w['x']},{w['y']})",
            fg=front_color
        )

    def _update_status(self, q):
        """Update status display from QUERY response dict."""
        state = q['search_state']
        x, y, h, steps = q['x'], q['y'], q['heading'], q['steps']
        # ★ Must match SearchState enum in lab5_search.h:
        STATES = ['IDLE','WARMUP','READING','DECIDING','TURNING','DRIVING',
                  'DRIVING▸▸','GOAL!','RETURNING','DONE','ERROR']
        sn = STATES[state] if state < len(STATES) else f'?{state}'
        mode_str = "CONT" if self.search_mode.get() == 1 else "S&G"
        colors = {0:TEXT_DIM, 1:NEON_CYAN, 5:NEON_MAGENTA, 6:NEON_CYAN,
                  7:NEON_GREEN, 8:NEON_CYAN, 9:NEON_GREEN, 10:NEON_RED}
        self.state_lbl.config(text=f"[ {sn} ] ({mode_str})", fg=colors.get(state, NEON_YELLOW))
        dn = self._dir_names[h] if h < 4 else '?'
        vc = q.get('visited_count', 0)
        self.pose_lbl.config(text=f"Pos: ({x},{y})  Hdg: {dn}  Steps: {steps}  Visited: {vc}")

    def _log_activity(self, msg, tag=None):
        """Append a timestamped line to the activity log."""
        import time
        ts = time.strftime("%H:%M:%S")
        self.activity_log.config(state=tk.NORMAL)
        content = self.activity_log.get("1.0", tk.END).strip()
        if content:
            self.activity_log.insert(tk.END, "\n")
        line = f"[{ts}] {msg}"
        if tag:
            self.activity_log.insert(tk.END, line, tag)
        else:
            self.activity_log.insert(tk.END, line)
        # Keep max 80 lines
        lines = int(self.activity_log.index('end-1c').split('.')[0])
        if lines > 80:
            self.activity_log.delete("1.0", f"{lines-80}.0")
        self.activity_log.see(tk.END)
        self.activity_log.config(state=tk.DISABLED)

    def _log_separator(self, label=""):
        """Add a visual separator line in the activity log."""
        self.activity_log.config(state=tk.NORMAL)
        content = self.activity_log.get("1.0", tk.END).strip()
        if content:
            self.activity_log.insert(tk.END, "\n")
        sep = f"──── {label} " if label else "────"
        sep = sep.ljust(60, "─")
        self.activity_log.insert(tk.END, sep, "sep")
        self.activity_log.see(tk.END)
        self.activity_log.config(state=tk.DISABLED)

    def _get_config(self):
        """Read current goal/start config from GUI spinboxes → dict for protocol."""
        return {
            'start_x': self.start_x.get(),
            'start_y': self.start_y.get(),
            'start_heading': self.start_hdg.get(),
            'goal_x_min': self.goal_x_min.get(),
            'goal_y_min': self.goal_y_min.get(),
            'goal_x_max': self.goal_x_max.get(),
            'goal_y_max': self.goal_y_max.get(),
        }

    def _log_state_transition(self, prev, q):
        """Log state transitions — concise, 1 line each."""
        DN = self._dir_names
        state = q['search_state']
        x, y, h = q['x'], q['y'], q['heading']
        hn = DN[h] if h < 4 else '?'
        steps = q.get('steps', 0)
        ec = q.get('error_code', 0)

        if state == 1:   # WARMUP
            self._log_activity(f"WARMUP  sensors settling ({x},{y})")
        elif state == 3: # DECIDING
            # Get flood cost at current cell
            fv = self.maze_view.flood[y][x] if y < 16 and x < 16 else '?'
            self._log_activity(f"DECIDE  ({x},{y}) {hn}  cost={fv}")
        elif state == 4: # TURNING
            self._log_activity(f"TURN    face {hn} at ({x},{y})")
        elif state == 5: # DRIVING
            self._log_activity(f"DRIVE   → {hn} from ({x},{y})")
        elif state == 6: # DRIVING_MULTI
            self._log_activity(f"DRIVE▸▸ → {hn} from ({x},{y})")
        elif state == 7: # GOAL
            self._log_activity(f"★ GOAL  ({x},{y}) steps={steps}", "goal")
        elif state == 9: # DONE
            self._log_activity(f"✔ DONE  back at ({x},{y})", "done")
        elif state == 10: # ERROR
            self._log_activity(f"✖ ERROR code={ec} at ({x},{y})", "err")
        elif state == 0 and prev != 0:  # → IDLE (step complete)
            self._log_activity(f"● IDLE  ({x},{y}) {hn} step#{steps}")

    def _compute_local_flood(self):
        """BFS flood fill in GUI — compute cost from each cell to goal."""
        from collections import deque
        mv = self.maze_view
        gx0, gy0, gx1, gy1 = mv.goal
        # Reset flood
        for y in range(16):
            for x in range(16):
                mv.flood[y][x] = 255
        q = deque()
        for gy in range(gy0, gy1+1):
            for gx in range(gx0, gx1+1):
                if 0 <= gx < 16 and 0 <= gy < 16:
                    mv.flood[gy][gx] = 0
                    q.append((gx, gy))
        # N=0x01, E=0x02, S=0x04, W=0x08
        dx = [0, 1, 0, -1]; dy = [1, 0, -1, 0]; wb = [0x01, 0x02, 0x04, 0x08]
        while q:
            cx, cy = q.popleft()
            cost = mv.flood[cy][cx] + 1
            for i in range(4):
                if mv.walls[cy][cx] & wb[i]: continue
                nx, ny = cx + dx[i], cy + dy[i]
                if 0 <= nx < 16 and 0 <= ny < 16 and cost < mv.flood[ny][nx]:
                    mv.flood[ny][nx] = cost
                    q.append((nx, ny))

    def do_apply_config(self):
        """Apply goal/start config — update GUI maze view + send to firmware."""
        cfg = self._get_config()
        # Update maze view goal highlight immediately
        self.maze_view.goal = (cfg['goal_x_min'], cfg['goal_y_min'],
                               cfg['goal_x_max'], cfg['goal_y_max'])
        self._compute_local_flood()
        self.maze_view._draw()
        # Also send standalone config command (in case FW has 0x59 handler)
        try:
            self.ble.send_cmd(cmd_search_set_config(
                start_x=cfg['start_x'], start_y=cfg['start_y'],
                start_heading=cfg['start_heading'],
                goal_x_min=cfg['goal_x_min'], goal_y_min=cfg['goal_y_min'],
                goal_x_max=cfg['goal_x_max'], goal_y_max=cfg['goal_y_max'],
                req_id=159))
        except Exception:
            pass
        self.cfg_status.config(
            text=f"✔ GUI: goal({cfg['goal_x_min']}-{cfg['goal_x_max']},{cfg['goal_y_min']}-{cfg['goal_y_max']}) "
                 f"start({cfg['start_x']},{cfg['start_y']},{'NESW'[cfg['start_heading']]})",
            fg=NEON_GREEN)
        self._log_activity(f"✔ Config set: goal({cfg['goal_x_min']}-{cfg['goal_x_max']},{cfg['goal_y_min']}-{cfg['goal_y_max']}) "
                           f"start({cfg['start_x']},{cfg['start_y']},{'NESW'[cfg['start_heading']]})")

    # ── Maze polling (★ ROOT FIX: GUI must poll firmware for maze data) ──────
    _maze_poll_id = None
    _maze_row_cursor = 0      # which row to request next (0-15)
    _maze_poll_active = False
    _maze_dirty = False        # ★ new: redraw pending

    def _burst_fetch_maze(self):
        """Immediately fetch QUERY + all 16 maze rows (after STEP/SEARCH/RETURN)."""
        try:
            self.ble.send_cmd(cmd_search_query(req_id=155))
            for row in range(16):
                self.ble.send_cmd(cmd_search_maze_row(row, req_id=1560 + row))
        except Exception:
            pass

    def start_maze_poll(self):
        """Start polling maze state at ~2Hz (QUERY) + 4 rows/poll."""
        if self._maze_poll_active:
            return
        self._maze_poll_active = True
        self._maze_poll_cycle()

    def stop_maze_poll(self):
        self._maze_poll_active = False
        if self._maze_poll_id:
            self.frame.after_cancel(self._maze_poll_id)
            self._maze_poll_id = None

    def _maze_poll_cycle(self):
        """Send QUERY + 4 MAZE_ROWs per cycle (full refresh every 2s)."""
        if not self._maze_poll_active:
            return
        try:
            # Always send QUERY (pose + state)
            self.ble.send_cmd(cmd_search_query(req_id=155))
            # Send 4 MAZE_ROWs per cycle (full 16 rows every 4 cycles = 2s)
            for i in range(4):
                row = self._maze_row_cursor
                self.ble.send_cmd(cmd_search_maze_row(row, req_id=1560 + row))
                self._maze_row_cursor = (row + 1) % 16
        except Exception:
            pass
        # Schedule next poll (500ms = 2Hz for pose, full maze update every 2s)
        self._maze_poll_id = self.frame.after(500, self._maze_poll_cycle)

    def poll_tick(self):
        """Called by App._tick every 200ms when Lab5 tab is active."""
        # Auto-start maze polling when on this tab
        if not self._maze_poll_active:
            self.start_maze_poll()

    def poll_stop(self):
        """Called when switching away from Lab5 tab."""
        self.stop_maze_poll()

    def handle_fast(self, data):
        f = parse_fast(data)
        if not f: return
        self.plot_vel.add_point(vL=f['velL_mmps'], vR=f['velR_mmps'])
        self.plot_vel.redraw()

    def handle_rsp(self, data):
        rsp = parse_rsp(data)
        if not rsp: return
        rid = rsp.get('req_id', 0)
        if rsp['status'] != RSP_OK:
            if 150 <= rid <= 199 or 1560 <= rid <= 1575:
                self._log_activity(f"✖ RSP error: {RSP_NAMES.get(rsp['status'], '?')}", "err")
            return
        pl = rsp.get('payload', b'')

        # ── SET_CONFIG ack (req_id=159) ──
        if rid == 159 and len(pl) >= 7:
            self.cfg_status.config(
                text=f"✔ FW config: start({pl[0]},{pl[1]},{'NESW'[pl[2]]}) goal({pl[3]}-{pl[5]},{pl[4]}-{pl[6]})",
                fg=NEON_GREEN)
            self._log_activity(f"✔ Config applied: start({pl[0]},{pl[1]}) goal({pl[3]}-{pl[5]},{pl[4]}-{pl[6]})")

        # ── Wall sensor read (req_id=154) ──
        elif rid == 154 and len(pl) >= 12:
            try:
                w = parse_rsp_walls(pl)
                self._update_wall_display(w)
            except Exception:
                pass

        # ── Search QUERY (req_id=155) ── ★ Updates pose + status ──
        elif rid == 155 and len(pl) >= 10:
            try:
                q = parse_rsp_search_query(pl)
                prev_state = self._search_state
                self._search_state = q['search_state']
                self._pose = {'x': q['x'], 'y': q['y'], 'h': q['heading'], 'steps': q['steps']}
                self._update_status(q)
                self.maze_view.set_robot(q['x'], q['y'], q['heading'])
                # ★ GUI is source of truth for goal — no sync from FW
                # ★ Log state transitions for debugging
                if prev_state != q['search_state']:
                    self._log_state_transition(prev_state, q)
            except Exception:
                pass

        # ── Maze row data (req_id=1560+row) ── ★ Updates walls on map ──
        elif 1560 <= rid <= 1575 and len(pl) >= 11:
            try:
                mr = parse_rsp_maze_row(pl)
                row = mr['row']
                for col in range(16):
                    self.maze_view.walls[row][col] = mr['walls'][col]
                    self.maze_view.visited[row][col] = bool(mr['visited'][col])
                self._maze_dirty = True
                # ★ Redraw + recompute flood on last row of burst
                if row == 15 or row == self._maze_row_cursor:
                    self._compute_local_flood()
                    self.maze_view._draw()
                    self._maze_dirty = False
            except Exception:
                pass

        # ── Fallback redraw ──
        if self._maze_dirty:
            self._compute_local_flood()
            self.maze_view._draw()
            self._maze_dirty = False
# ═══════════════════════════════════════════════════════════════════════════════

# ═══════════════════════════════════════════════════════════════════════════════
# Lab 6: Speed Run
# ═══════════════════════════════════════════════════════════════════════════════
class Lab6Tab:
    SPEED_LABELS = [
        "Level 0 — Cautious (200 mm/s)",
        "Level 1 — Moderate (350 mm/s)",
        "Level 2 — Fast (500 mm/s)",
        "Level 3 — Fastest (700 mm/s)",
    ]
    STATE_NAMES = ['IDLE','PLANNING','RUNNING','TURNING','DONE','ERROR']

    def __init__(self, parent, ble):
        self.ble = ble
        self.frame = ttk.Frame(parent, padding=10)
        top = tk.Frame(self.frame, bg=BG); top.pack(fill=tk.BOTH, expand=True)
        left = tk.Frame(top, bg=BG, width=500); left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0,5))

        # ── Info ──
        inf = make_card(left, "▸ LAB 6 — SPEED RUN"); inf.pack(fill=tk.X, pady=3)
        self.info_txt = make_ro(inf, 4, NEON_CYAN); self.info_txt.pack(fill=tk.X)
        wr(self.info_txt, "  วิ่งเส้นทางที่สั้นที่สุดผ่าน maze ที่รู้จักแล้ว\n"
           "  ★ ต้อง search (Lab 5) ให้เสร็จก่อน ถึงจะ speed run ได้\n"
           "  เลือก speed level → ARM → ▶ START\n"
           "  Level 0 ปลอดภัยสุด / Level 3 เร็วสุด (แข่งขัน)")

        # ── Speed Level ──
        sf = make_card(left, "▸ SPEED LEVEL"); sf.pack(fill=tk.X, pady=3)
        self.speed_var = tk.IntVar(value=0)
        for i, lbl in enumerate(self.SPEED_LABELS):
            tk.Radiobutton(sf, text=lbl, variable=self.speed_var, value=i,
                           font=("Consolas", 9), fg=NEON_GREEN, bg=BG_CARD,
                           selectcolor=BG, activebackground=BG_CARD,
                           activeforeground=NEON_CYAN).pack(anchor=tk.W, padx=8)

        # ── Controls ──
        cf = make_card(left, "▸ CONTROLS"); cf.pack(fill=tk.X, pady=3)
        cb = tk.Frame(cf, bg=BG_CARD); cb.pack(fill=tk.X, pady=4)
        ttk.Button(cb, text="▶ START", command=self.do_start, style="G.TButton", width=10).pack(side=tk.LEFT, padx=3)
        ttk.Button(cb, text="■ STOP", command=self.do_stop, style="R.TButton", width=10).pack(side=tk.LEFT, padx=3)

        # ── Status ──
        stf = make_card(left, "▸ STATUS"); stf.pack(fill=tk.X, pady=3)
        self.state_lbl = tk.Label(stf, text="[ IDLE ]", font=("Consolas", 12, "bold"),
                                   fg=TEXT_DIM, bg=BG_CARD)
        self.state_lbl.pack(anchor=tk.W, padx=8, pady=2)
        self.progress_lbl = tk.Label(stf, text="Path: --  Progress: --/--  Segments: --",
                                      font=("Consolas", 9), fg=TEXT_DIM, bg=BG_CARD)
        self.progress_lbl.pack(anchor=tk.W, padx=8, pady=2)
        # Progress bar
        self.progress_bar = tk.Canvas(stf, width=400, height=16, bg='#08080c',
                                       highlightthickness=0)
        self.progress_bar.pack(padx=8, pady=(2,6))

        self._poll_id = None
        self._running = False

    def do_start(self):
        self.ble.send_cmd(cmd_speedrun_start(speed_level=self.speed_var.get()))
        self._running = True
        self._start_poll()

    def do_stop(self):
        self.ble.send_cmd(cmd_speedrun_stop())
        self._running = False

    def _start_poll(self):
        if self._poll_id: return
        self._poll_cycle()

    def _stop_poll(self):
        if self._poll_id:
            self.frame.after_cancel(self._poll_id)
            self._poll_id = None

    def _poll_cycle(self):
        if self._running:
            self.ble.send_cmd(cmd_speedrun_query())
        self._poll_id = self.frame.after(500, self._poll_cycle)

    def _update_state(self, d):
        st = d['state']
        sn = self.STATE_NAMES[st] if st < len(self.STATE_NAMES) else f'?{st}'
        colors = {0: TEXT_DIM, 1: NEON_YELLOW, 2: NEON_MAGENTA, 3: NEON_CYAN, 4: NEON_GREEN, 5: NEON_RED}
        self.state_lbl.config(text=f"[ {sn} ]  Speed: Lv.{d['speed_level']}",
                               fg=colors.get(st, TEXT_DIM))
        pl = d['path_len']; pr = d['progress']; ns = d['num_segs']; cs = d['cur_seg']
        self.progress_lbl.config(
            text=f"Path: {pl} cells  Progress: {pr}/{pl}  Segment: {cs}/{ns}")
        # Update bar
        c = self.progress_bar; c.delete("all"); w = 400
        if pl > 0:
            frac = min(pr / pl, 1.0)
            fw = int(w * frac)
            c.create_rectangle(0, 0, fw, 16, fill=NEON_GREEN, outline='')
            c.create_text(w//2, 8, text=f"{int(frac*100)}%", fill=WHITE, font=("Consolas", 8, "bold"))
        # Stop polling when done
        if st >= 4:
            self._running = False

    def poll_tick(self):
        """Called by App when Lab6 tab is active."""
        if self._running and not self._poll_id:
            self._start_poll()

    def poll_stop(self):
        self._stop_poll()

    def handle_fast(self, data): pass

    def handle_rsp(self, data):
        rsp = parse_rsp(data)
        if not rsp: return
        rid = rsp.get('req_id', 0)
        if rsp['status'] != RSP_OK:
            if 160 <= rid <= 169:
                wr(self.info_txt, f"  ✖ {RSP_NAMES.get(rsp['status'], '?')}  — maze ยังไม่มีข้อมูล? ลอง Lab 5 ก่อน")
            return
        if rid == 162 and len(rsp.get('payload', b'')) >= 10:
            try:
                d = parse_rsp_speedrun(rsp['payload'])
                self._update_state(d)
            except Exception:
                pass


# ═══════════════════════════════════════════════════════════════════════════════
# Main App
# ═══════════════════════════════════════════════════════════════════════════════
class App:
    def __init__(self):
        self.root=tk.Tk();self.root.title("MICROMOUSE // DEBUG TERMINAL v4.0")
        self.root.geometry("1060x780");self.root.minsize(900,680)
        setup_theme(self.root)
        self.ble=MicromouseBLE();self.ble.start()

        top=tk.Frame(self.root,bg='#0e0e14',padx=10,pady=8,highlightbackground=BORDER,highlightthickness=1)
        top.pack(fill=tk.X)
        tk.Label(top,text="MICROMOUSE",font=("Consolas",12,"bold"),fg=NEON_GREEN,bg='#0e0e14').pack(side=tk.LEFT)
        tk.Label(top,text=" // DEBUG TERMINAL v4",font=("Consolas",10),fg=TEXT_DIM,bg='#0e0e14').pack(side=tk.LEFT)
        self.status_lbl=tk.Label(top,text="● OFFLINE",font=("Consolas",10,"bold"),fg=NEON_RED,bg='#0e0e14')
        self.status_lbl.pack(side=tk.RIGHT,padx=10)
        self.disc_btn=ttk.Button(top,text="DISCONNECT",command=self.do_disc,style="R.TButton")
        self.disc_btn.pack(side=tk.RIGHT,padx=3);self.disc_btn.state(['disabled'])
        self.conn_btn=ttk.Button(top,text="CONNECT",command=self.do_conn,style="G.TButton")
        self.conn_btn.pack(side=tk.RIGHT,padx=3)
        self.dev_var=tk.StringVar(value="...")
        self.dev_combo=ttk.Combobox(top,textvariable=self.dev_var,width=26,state='readonly')
        self.dev_combo.pack(side=tk.RIGHT,padx=5)
        self.scan_btn=ttk.Button(top,text="◉ SCAN",command=self.do_scan,style="M.TButton")
        self.scan_btn.pack(side=tk.RIGHT,padx=3)

        nb=ttk.Notebook(self.root);nb.pack(fill=tk.BOTH,expand=True,padx=5,pady=5)
        self.lab1=Lab1Tab(nb,self.ble);nb.add(self.lab1.frame,text="  LAB 1 // HW  ")
        self.lab2a=Lab2aTab(nb,self.ble);nb.add(self.lab2a.frame,text="  LAB 2a // ENC  ")
        self.lab2b=Lab2bTab(nb,self.ble);nb.add(self.lab2b.frame,text="  LAB 2b // MOT  ")
        self.lab3=Lab3Tab(nb,self.ble);nb.add(self.lab3.frame,text="  LAB 3 // MOVE  ")
        self.lab4=Lab4Tab(nb,self.ble);nb.add(self.lab4.frame,text="  LAB 4 // PID  ")
        self.lab5=Lab5Tab(nb,self.ble);nb.add(self.lab5.frame,text="  LAB 5 // MAZE  ")
        self.lab6=Lab6Tab(nb,self.ble);nb.add(self.lab6.frame,text="  LAB 6 // SPEED  ")

        self.ble.on_rsp=lambda d:self.root.after(0,lambda:self._rsp(d))
        self.ble.on_fast=lambda d:self.root.after(0,lambda:self._fast(d))
        self.ble.on_slow=lambda d:self.root.after(0,lambda:self.lab1.handle_slow(d))
        self.ble.on_connect=lambda:self.root.after(0,self._ui_conn)
        self.ble.on_disconnect=lambda:self.root.after(0,self._ui_disc)
        self._devs=[];self._ka_id=None;self._tab=0;self._prev_tab=0
        nb.bind("<<NotebookTabChanged>>",lambda e:setattr(self,'_tab',nb.index(nb.select())))
        self._tick();self.root.protocol("WM_DELETE_WINDOW",self._close)

    def _rsp(self,d): self.lab1.handle_rsp(d);self.lab2a.handle_rsp(d);self.lab2b.handle_rsp(d);self.lab3.handle_rsp(d);self.lab4.handle_rsp(d);self.lab5.handle_rsp(d);self.lab6.handle_rsp(d)
    def _fast(self,d):
        self.lab1.handle_fast(d)
        # Always route to tabs with active runs (RUNNING state) + active tab
        if self._tab==2 or getattr(self.lab2b,'_running',False):self.lab2b.handle_fast(d)
        if self._tab==3 or self.lab3.mgr.state==RunState.RUNNING:self.lab3.handle_fast(d)
        if self._tab==4 or self.lab4.mgr.state==RunState.RUNNING:self.lab4.handle_fast(d)
        if self._tab==5:self.lab5.handle_fast(d)
    def _ui_conn(self):
        self.status_lbl.config(text="● ONLINE",fg=NEON_GREEN)
        self.conn_btn.state(['disabled']);self.disc_btn.state(['!disabled']);self.scan_btn.state(['disabled'])
        self.ble.send_cmd(cmd_set_fast_hz(50));self.ble.send_cmd(cmd_set_slow_hz(5));self._ka_start()
    def _ui_disc(self):
        self.status_lbl.config(text="● OFFLINE",fg=NEON_RED)
        self.conn_btn.state(['!disabled']);self.disc_btn.state(['disabled']);self.scan_btn.state(['!disabled']);self._ka_stop()
    def _ka_start(self):self._ka_stop();self._ka_ping()
    def _ka_ping(self):
        if self.ble.connected:self.ble.send_cmd(cmd_ping(req_id=9999));self._ka_id=self.root.after(3000,self._ka_ping)
    def _ka_stop(self):
        if self._ka_id:self.root.after_cancel(self._ka_id);self._ka_id=None
    def do_scan(self):
        self.scan_btn.state(['disabled']);self.status_lbl.config(text="● SCANNING...",fg=NEON_MAGENTA)
        self.ble.scan(timeout=5.0,callback=lambda d:self.root.after(0,lambda:self._show_scan(d)))
    def _show_scan(self,devs):
        self._devs=devs;self.dev_combo['values']=[f"{d.name} ({d.address})"for d in devs]
        if devs:self.dev_combo.current(0)
        self.scan_btn.state(['!disabled']);self.status_lbl.config(text=f"● FOUND {len(devs)}",fg=NEON_CYAN)
    def do_conn(self):
        i=self.dev_combo.current()
        if i<0 or i>=len(self._devs):return
        self.status_lbl.config(text="● LINKING...",fg=NEON_YELLOW);self.conn_btn.state(['disabled'])
        self.ble.connect(self._devs[i].address)
    def do_disc(self):self._ka_stop();self.ble.disconnect()
    def _tick(self):
        self.lab1.update_counters()
        if self._tab==1: self.lab2a.poll_tick()
        if self._tab==5: self.lab5.poll_tick()
        elif hasattr(self,'_prev_tab') and self._prev_tab==5: self.lab5.poll_stop()
        if self._tab==6: self.lab6.poll_tick()
        elif hasattr(self,'_prev_tab') and self._prev_tab==6: self.lab6.poll_stop()
        self._prev_tab=self._tab
        self.root.after(200,self._tick)
    def _close(self):self._ka_stop();self.ble.stop();self.root.destroy()
    def run(self):self.root.mainloop()

if __name__=="__main__":App().run()
