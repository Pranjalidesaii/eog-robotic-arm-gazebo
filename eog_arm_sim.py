#!/usr/bin/env python3
"""EOG-controlled 4-DOF robotic arm: signal simulation + processing pipeline + Gazebo bridge.

    synthetic 2-channel EOG  ->  notch / smoothing / adaptive drift tracking
        ->  intent validation (dwell, amplitude, refractory)  ->  5 commands
        ->  arm controller (IK)  ->  ROS 2 /eog_arm/<joint>/cmd_pos  ->  Gazebo

The "user" is a closed-loop script that performs a pick-and-place using only eye
movements: it looks left/right/up/down and does one deliberate blink to grasp/release.
Natural blinks, involuntary glances, 50 Hz hum and baseline drift are injected on top.

Modes:  --mode robust  (default)  adaptive drift correction + false-command rejection
        --mode naive              fixed threshold, no validation (for comparison)
"""
import argparse
import collections
import math
import os
import sys
import threading
import time

import numpy as np
from scipy import signal

import arm_config as C

FS = 250                    # EOG sampling rate (Hz)
DT = 1.0 / FS
TICK = 0.02                 # control tick (s) = 5 EOG samples
SPT = int(round(TICK * FS))
WINDOW = 0.2                # decision window (s), as in the methodology slide
DWELL = 0.35                # gaze must persist this long to count as a command (robust)
BLINK_RATIO = 6.0           # deliberate blink: vertical peak > 6 x threshold
THR_FLOOR = 70.0            # uV, minimum detection threshold
CMD_ROW = {"left": 0, "right": 1, "up": 2, "down": 3, "blink": 4}


# ======================================================================================
# 1. Synthetic EOG source
# ======================================================================================
class EOGSynth:
    """Two-channel EOG (horizontal, vertical) in microvolts."""
    GAZE = {"rest": (0, 0), "left": (-260, 0), "right": (260, 0), "up": (0, 210), "down": (0, -180)}

    def __init__(self, rng, natural_events=True):
        self.rng, self.t, self.natural = rng, 0.0, natural_events
        self.gaze, self.level, self.target = "rest", np.zeros(2), np.zeros(2)
        self.pulses = []
        self.hum_ph = rng.uniform(0, 6.28)
        # baseline drift: slow ramp + slow oscillation + offset (what breaks fixed thresholds)
        self.offset = np.array([40.0, -25.0])
        self.ramp = np.array([3.2, -2.0])
        self.sin_amp, self.sin_f = np.array([60.0, 35.0]), np.array([0.031, 0.047])
        self.sin_ph = rng.uniform(0, 6.28, 2)
        self.next_blink = 2.5 + rng.uniform(0, 2)
        self.next_glance = 8.0 + rng.uniform(0, 3)
        self.onset_t = None

    def set_gaze(self, g):
        if g != self.gaze:
            self.gaze = g
            self.target = np.array(self.GAZE[g], float)
            if g != "rest":
                self.onset_t = self.t

    def blink(self, amp=750.0, dur=0.16):
        self.pulses.append(("blink", 1, self.t, dur, amp))

    def glance(self, ch, amp, dur):
        self.pulses.append(("glance", ch, self.t, dur, amp))

    def sample(self):
        r = self.rng
        if self.natural:
            if self.t >= self.next_blink:                      # involuntary blink (weak)
                self.blink(amp=r.uniform(120, 165), dur=0.14)
                self.next_blink = self.t + r.uniform(3.0, 6.0)
            if self.t >= self.next_glance:                     # involuntary glance (brief)
                self.glance(0, r.choice([-1, 1]) * r.uniform(110, 150), r.uniform(0.12, 0.2))
                self.next_glance = self.t + r.uniform(9.0, 13.0)
        self.level += (self.target - self.level) * (DT / 0.02)   # fast saccade rise
        x = self.level.copy()
        keep = []
        for p in self.pulses:
            kind, ch, t0, dur, amp = p
            u = (self.t - t0) / dur
            if kind == "blink":
                if u <= 1:
                    x[ch] += amp * math.sin(math.pi * max(u, 0))
                elif u <= 1.5:
                    x[ch] -= 0.25 * amp * math.sin(math.pi * (u - 1) / 0.5)
                if u < 1.5:
                    keep.append(p)
            else:
                if 0 <= u <= 1:
                    x[ch] += amp * min(1.0, 4 * min(u, 1 - u))
                if u < 1:
                    keep.append(p)
        self.pulses = keep
        drift = self.offset + self.ramp * self.t + \
            self.sin_amp * np.sin(2 * math.pi * self.sin_f * self.t + self.sin_ph)
        hum = 25.0 * math.sin(2 * math.pi * 50.0 * self.t + self.hum_ph) * np.array([1.0, 0.7])
        self.t += DT
        return x + drift + r.normal(0, 6.0, 2) + hum


# ======================================================================================
# 2. Processing pipeline + intent validation
# ======================================================================================
class Chan:
    """Per-channel conditioning: 50 Hz notch -> moving average (64 ms)."""
    def __init__(self):
        b, a = signal.iirnotch(50.0, 30.0, FS)
        self.sos = signal.tf2sos(b, a)
        self.zi = None
        self.ma = collections.deque(maxlen=16)

    def block(self, x):
        x = np.asarray(x, float)
        if self.zi is None:
            self.zi = signal.sosfilt_zi(self.sos) * x[0]
        y, self.zi = signal.sosfilt(self.sos, x, zi=self.zi)
        out = []
        for v in y:
            self.ma.append(v)
            out.append(sum(self.ma) / len(self.ma))
        return out


class Pipeline:
    def __init__(self, mode="robust"):
        self.mode = mode
        self.ch = [Chan(), Chan()]
        self.n = 0
        self.t = 0.0
        self.cal = [[], []]
        self.base = [0.0, 0.0]
        self.thr = [THR_FLOOR, THR_FLOOR]
        self.cand = None
        self.refractory_until = 0.0
        self.cmd = None                 # command held for the current 200 ms window
        self.next_window = WINDOW
        self.events = []                # (t, kind, text)
        self.calibrated = False
        self.confidence = 0.0

    def log(self, kind, text):
        self.events.append((self.t, kind, text))

    # ---- one block of SPT raw samples; returns per-sample trace rows ------------------
    def process(self, raw_block):
        sm = [self.ch[i].block(raw_block[:, i]) for i in (0, 1)]
        rows = []
        for k in range(len(raw_block)):
            self.t += DT
            self.n += 1
            x = (sm[0][k], sm[1][k])
            if not self.calibrated:
                for i in (0, 1):
                    self.cal[i].append(x[i])
                if self.n >= FS:                       # 1 s of rest = initial calibration
                    for i in (0, 1):
                        a = np.array(self.cal[i][FS // 4:])
                        self.base[i] = float(a.mean())
                        if self.mode == "robust":
                            self.thr[i] = max(THR_FLOOR, 10.0 * float(a.std()))
                    self.calibrated = True
                    self.log("info", f"calibrated: thr H/V = {self.thr[0]:.0f}/{self.thr[1]:.0f} uV")
                rows.append((self.t, raw_block[k, 0], raw_block[k, 1], x[0], x[1], 0.0, 0.0))
                continue
            dev = [x[0] - self.base[0], x[1] - self.base[1]]
            if self.mode == "robust":                  # adaptive drift tracking, per channel,
                for i in (0, 1):                       # frozen only while that channel is in an excursion
                    if abs(dev[i]) < 0.95 * self.thr[i]:
                        self.base[i] += (x[i] - self.base[i]) * (DT / 1.2)
            self._detect(dev)
            if self.t >= self.next_window:
                self.next_window += WINDOW
                self.cmd = self.cand["announced"] if self.cand and self.cand["announced"] else None
            rows.append((self.t, raw_block[k, 0], raw_block[k, 1], self.base[0], self.base[1], dev[0], dev[1]))
        return rows

    # ---- candidate event logic ------------------------------------------------------------
    def _detect(self, dev):
        robust = self.mode == "robust"
        n = [dev[0] / self.thr[0], dev[1] / self.thr[1]]
        mag = max(abs(n[0]), abs(n[1]))
        c = self.cand
        if c is None:
            if self.t < self.refractory_until or mag <= 1.0:
                return
            c = self.cand = dict(t0=self.t, peak=[0.0, 0.0], ema=[0.0, 0.0], below=0,
                                 announced=None, cls=None, cls_t0=self.t)
        for i in (0, 1):
            if abs(dev[i]) > abs(c["peak"][i]):
                c["peak"][i] = dev[i]
            c["ema"][i] += (n[i] - c["ema"][i]) * (DT / 0.15)
        c["below"] = c["below"] + 1 if mag < 0.7 else 0
        dur = self.t - c["t0"]
        if c["below"] >= 10:
            self._finish(c, dur - 0.04)
            self.cand = None
            return
        if robust and dur > 8.0:                       # stuck far from baseline => drift, re-baseline
            self.base = [self.base[0] + dev[0], self.base[1] + dev[1]]
            self.log("info", "long excursion treated as drift: baseline reset")
            self.cand = None
            return
        # --- continuous classification: dominant channel + sign of the smoothed deviation ---
        i = 0 if abs(c["ema"][0]) >= abs(c["ema"][1]) else 1
        name = None
        if abs(c["ema"][i]) > 1.2:
            name = ("left" if c["ema"][0] < 0 else "right") if i == 0 else ("up" if c["ema"][1] > 0 else "down")
        if name != c["cls"]:                           # direction changed -> restart the dwell timer
            if c["announced"] and name != c["announced"]:
                self.log("info", f"{c['announced'].upper()} released")
                c["announced"] = None
            c["cls"], c["cls_t0"] = name, self.t
            if name is not None:
                c["peak"] = list(dev)
        blinkish = abs(c["peak"][1]) >= BLINK_RATIO * self.thr[1]
        held = self.t - c["cls_t0"]
        if (name is not None and c["announced"] is None
                and (held >= DWELL or not robust) and not (robust and blinkish)):
            c["announced"] = name
            c["ever"] = True
            self.confidence = min(1.0, abs(c["ema"][i]) / 2.5)
            self.log("cmd", f"{name.upper():5s} detected (conf {self.confidence:.2f})")

    def _finish(self, c, dur):
        robust = self.mode == "robust"
        pv = c["peak"][1]
        if robust:
            blink = abs(pv) >= BLINK_RATIO * self.thr[1] and pv > 0 and dur < 0.9
        else:
            blink = c["peak"][1] > self.thr[1] and dur < 0.45 and abs(c["peak"][0]) < self.thr[0]
        if blink:
            self.confidence = min(1.0, abs(pv) / (BLINK_RATIO * self.thr[1] * 1.3))
            self.log("blink", f"BLINK/SELECT (conf {self.confidence:.2f})")
            self.refractory_until = self.t + 0.35
            self.pending_blink = True
        elif c["announced"]:
            self.log("info", f"{c['announced'].upper()} released after {dur:.2f}s")
        elif robust and not c.get("ever"):
            pk = max(abs(c["peak"][0]), abs(c["peak"][1]))
            self.log("reject", f"rejected: {dur * 1000:.0f} ms, peak {pk:.0f} uV (involuntary)")

    pending_blink = False


# ======================================================================================
# 3. Arm controller (commands -> joint set-points)
# ======================================================================================
class Arm:
    PAN_RATE, REACH_RATE, GRIP_RATE = 0.35, 0.28, 0.06

    def __init__(self):
        self.pan, self.s, self.grip, self.grip_target = 0.0, 0.0, 0.0, 0.0
        self.actual = {}          # measured joint positions from Gazebo (empty without ROS)

    def settled(self, tol=0.025):
        """True when the physical arm has caught up with the commanded arm joints."""
        if not self.actual:
            return True
        return all(abs(self.actual.get(j, v) - v) < tol
                   for j, v in self.joints().items() if not j.startswith("finger"))

    def step(self, dt, cmd, blink):
        if cmd == "left":
            self.pan = min(1.6, self.pan + self.PAN_RATE * dt)
        elif cmd == "right":
            self.pan = max(-1.6, self.pan - self.PAN_RATE * dt)
        elif cmd == "down":
            self.s = min(1.0, self.s + self.REACH_RATE * dt)
        elif cmd == "up":
            self.s = max(0.0, self.s - self.REACH_RATE * dt)
        if blink:
            self.grip_target = C.FINGER_STROKE if self.grip_target == 0.0 else 0.0
        d = self.grip_target - self.grip
        self.grip += max(-self.GRIP_RATE * dt, min(self.GRIP_RATE * dt, d))

    def joints(self):
        q1, q2, q3 = C.ik_down(*C.path_rz(self.s))
        return {"shoulder_pan": self.pan, "shoulder_lift": q1, "elbow": q2, "wrist": q3,
                "finger_left": self.grip, "finger_right": self.grip}

    @property
    def gripper_closed(self):
        return self.grip_target > 0


# ======================================================================================
# 4. The simulated "user": closed-loop eye-movement script for pick & place
# ======================================================================================
class User:
    def __init__(self, synth, arm):
        s, a = synth, arm
        lp, ls = a.PAN_RATE * 0.2, a.REACH_RATE * 0.2          # release slightly early (latency lead)
        P, Q = C.PICK_ANGLE, C.PLACE_ANGLE
        self.steps = [
            ("Rest - system calibrates, natural blinks injected", "rest", 3.5, None),
            ("Look LEFT  -> turn towards the cube", "left", 9, lambda: a.pan >= P - lp),
            ("Settle", "rest", 0.9, a.settled),
            ("Look DOWN  -> reach the cube", "down", 9, lambda: a.s >= 1 - ls),
            ("Settle", "rest", 0.9, a.settled),
            ("Deliberate BLINK -> close gripper", "blink", 1.8, None),
            ("Look UP    -> lift the cube", "up", 9, lambda: a.s <= ls),
            ("Settle", "rest", 0.9, a.settled),
            ("Look RIGHT -> swing to the green goal pad", "right", 12, lambda: a.pan <= Q + lp),
            ("Settle", "rest", 0.9, a.settled),
            ("Look DOWN  -> lower onto the pad", "down", 9, lambda: a.s >= 1 - ls),
            ("Settle", "rest", 0.9, a.settled),
            ("Deliberate BLINK -> open gripper", "blink", 1.8, None),
            ("Look UP    -> retract", "up", 9, lambda: a.s <= ls),
            ("Settle", "rest", 0.9, a.settled),
            ("Look LEFT  -> return to home", "left", 12, lambda: a.pan >= -lp),
            ("Rest - done", "rest", 4.0, None),
        ]
        self.i, self.t_start, self.synth, self.done = -1, 0.0, s, False
        self._enter(0.0)

    @property
    def label(self):
        return self.steps[min(self.i, len(self.steps) - 1)][0]

    def _enter(self, t):
        self.i += 1
        self.t_start = t
        if self.i >= len(self.steps):
            self.done = True
            self.synth.set_gaze("rest")
            return
        _, kind, _, _ = self.steps[self.i]
        if kind == "blink":
            self.synth.set_gaze("rest")
            self.synth.blink()
        else:
            self.synth.set_gaze(kind)

    def update(self, t):
        if self.done:
            return
        _, kind, dur, until = self.steps[self.i]
        el = t - self.t_start
        if kind == "rest":                 # wait out the time AND (if given) the physical arm settling
            if el >= dur and (until is None or until() or el > dur + 8):
                self._enter(t)
        elif el >= dur or (until is not None and until() and el > 0.3):
            self._enter(t)


# ======================================================================================
# 5. Simulation thread (EOG -> pipeline -> arm -> ROS)
# ======================================================================================
class Sim:
    def __init__(self, args):
        self.args = args
        rng = np.random.default_rng(args.seed)
        self.synth = EOGSynth(rng, natural_events=not args.clean)
        self.pipe = Pipeline(args.mode)
        self.arm = Arm()
        self.user = User(self.synth, self.arm)
        self.t = 0.0
        self.stop = False
        self.trace = collections.deque(maxlen=FS * 12 // 2)       # every 2nd sample
        self.cmd_hist = collections.deque(maxlen=int(12 / TICK))
        self.truth_hist = collections.deque(maxlen=int(12 / TICK))
        self.log_lines = collections.deque(maxlen=14)
        self.compute_ms = 0.0
        self.cube_xy = None
        self.cube_z = None
        self.node = None
        self.pubs = {}
        self.last_cmd = None
        self.latencies = []
        self.finished = False
        self._n_events = 0

    # ---- ROS plumbing ----------------------------------------------------------------------
    def init_ros(self):
        import rclpy
        from std_msgs.msg import Float64
        rclpy.init()
        self.node = rclpy.create_node("eog_arm_sim")
        self.Float64 = Float64
        for j in C.JOINT_ORDER:
            self.pubs[j] = self.node.create_publisher(Float64, f"/eog_arm/{j}/cmd_pos", 10)

        from sensor_msgs.msg import JointState

        def on_js(msg):
            self.arm.actual = dict(zip(msg.name, msg.position))
        self.node.create_subscription(
            JointState, "/world/eog_world/model/eog_arm/joint_state", on_js, 10)
        threading.Thread(target=lambda: rclpy.spin(self.node), daemon=True).start()

    def poll_cube(self):
        """Reads the cube pose straight from Gazebo (the ROS bridge drops entity names)."""
        import re
        import subprocess
        while not self.stop:
            try:
                out = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", "/world/eog_world/pose/info"],
                                     capture_output=True, text=True, timeout=5).stdout
                m = re.search(r'name: "target_cube".*?position \{(.*?)\}', out, re.S)
                if m:
                    g = lambda k: float(re.search(k + r": ([-\d.e]+)", m.group(1)).group(1))                         if re.search(k + r": ([-\d.e]+)", m.group(1)) else 0.0
                    self.cube_xy, self.cube_z = (g("x"), g("y")), g("z")
            except Exception:
                pass
            time.sleep(1.0)

    def publish(self):
        if not self.node:
            return
        for j, v in self.arm.joints().items():
            m = self.Float64()
            m.data = float(v)
            self.pubs[j].publish(m)

    # ---- main loop -----------------------------------------------------------------------------
    def say(self, kind, text):
        line = f"[{self.t:6.2f}s] {text}"
        self.log_lines.append((kind, line))
        print(line, flush=True)

    def run(self):
        nxt = time.monotonic()
        self.say("info", f"EOG pipeline started - mode: {self.args.mode}")
        while not self.stop:
            raw = np.array([self.synth.sample() for _ in range(SPT)])
            t0 = time.perf_counter()
            rows = self.pipe.process(raw)
            self.compute_ms = 0.9 * self.compute_ms + 0.1 * (time.perf_counter() - t0) * 1000
            self.t = self.pipe.t
            self.trace.extend(rows[::2])

            # new pipeline events -> log + latency bookkeeping
            while self._n_events < len(self.pipe.events):
                te, kind, text = self.pipe.events[self._n_events]
                self._n_events += 1
                self.say(kind, text)
            blink = self.pipe.pending_blink
            self.pipe.pending_blink = False
            cmd = self.pipe.cmd if self.pipe.calibrated else None
            if cmd and cmd != self.last_cmd and self.synth.onset_t is not None:
                self.latencies.append(self.t - self.synth.onset_t)
            self.last_cmd = cmd
            if blink:
                self.cmd_hist.append((self.t, "blink"))

            self.arm.step(TICK, cmd, blink)
            self.cmd_hist.append((self.t, cmd))
            self.truth_hist.append((self.t, self.synth.gaze))
            if self.pipe.calibrated:
                prev = self.user.i
                self.user.update(self.synth.t)
                if self.user.i != prev and not self.user.done and "Settle" not in self.user.label:
                    self.say("step", f"STEP: {self.user.label}")
            self.publish()
            if self.user.done and not self.finished and self.synth.t > 5:
                self.finished = True
                self.report()
            nxt += TICK / self.args.speed
            time.sleep(max(0.0, nxt - time.monotonic()))
            if self.args.duration and self.synth.t > self.args.duration:
                break

    def report(self):
        goal = (C.TASK_RADIUS * math.cos(C.PLACE_ANGLE), C.TASK_RADIUS * math.sin(C.PLACE_ANGLE))
        msg = "scenario complete"
        time.sleep(1.5)
        if self.cube_xy:
            err = math.hypot(self.cube_xy[0] - goal[0], self.cube_xy[1] - goal[1])
            ok = err < 0.045 and self.cube_z < 0.05
            msg += f" - cube is {err * 100:.1f} cm from goal centre: {'SUCCESS' if ok else 'MISSED'}"
        rej = sum(1 for e in self.pipe.events if e[1] == "reject")
        msg += f" | rejected involuntary events: {rej}"
        if self.latencies:
            msg += f" | mean command latency {np.mean(self.latencies) * 1000:.0f} ms"
        self.say("done", msg)


# ======================================================================================
# 6. Live dashboard (matplotlib)
# ======================================================================================
def dashboard(sim):
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.gridspec import GridSpec

    BG, PANEL, FG, GRID, SUB = "#0e1621", "#16202e", "#d9e2ee", "#2a3a4f", "#9fb3c8"
    plt.rcParams.update({"text.color": FG, "axes.labelcolor": FG, "xtick.color": FG, "ytick.color": FG,
                         "axes.edgecolor": GRID, "font.size": 8})
    fig = plt.figure(figsize=(7.6, 10), facecolor=BG)
    fig.canvas.manager.set_window_title("EOG signal -> command -> robotic arm")
    try:                                    # dock on the right-hand side of the screen
        win = fig.canvas.manager.window
        sw, sh = int(win.winfo_screenwidth() * 0.8), int(win.winfo_screenheight() * 0.8)   # WSLg scales by 1.25
        w, h = int(sw * 0.40), int(sh * 0.90)
        win.geometry(f"{w}x{h}+{sw - w}+0")
    except Exception:
        pass
    gs = GridSpec(7, 1, figure=fig, height_ratios=[1, 1, 1, 1, 1.25, 1.15, 1.7],
                  left=0.09, right=0.98, top=0.945, bottom=0.03, hspace=0.62)
    fig.text(0.04, 0.972, "EOG -> command -> robotic arm", fontsize=14, fontweight="bold", color=FG)
    fig.text(0.97, 0.975, "live signal processing", fontsize=9, color=SUB, ha="right")
    axes = {}
    spec = [("h_raw", "Horizontal EOG: raw (with drift) + tracked baseline  [uV]"),
            ("h_dev", "Horizontal: drift-corrected + threshold  [uV]"),
            ("v_raw", "Vertical EOG: raw (with drift) + tracked baseline  [uV]"),
            ("v_dev", "Vertical: drift-corrected + threshold  [uV]")]
    for r, (k, title) in enumerate(spec):
        ax = fig.add_subplot(gs[r])
        ax.set_title(title, loc="left", fontsize=8, color=SUB, pad=2)
        axes[k] = ax
    ax_cmd = fig.add_subplot(gs[4])
    ax_cmd.set_title("Detected command (thick)  vs  true intent (thin)   x = rejected", loc="left",
                     fontsize=8, color=SUB, pad=2)
    ax_st = fig.add_subplot(gs[5])
    ax_lg = fig.add_subplot(gs[6])
    for ax in list(axes.values()) + [ax_cmd, ax_st, ax_lg]:
        ax.set_facecolor(PANEL)
    for ax in list(axes.values()) + [ax_cmd]:
        ax.grid(color=GRID, lw=0.5)
    for ax in (ax_st, ax_lg):
        ax.set_xticks([]); ax.set_yticks([])
    ln = {}
    ln["h_raw"], = axes["h_raw"].plot([], [], color="#6b7f99", lw=0.8)
    ln["h_base"], = axes["h_raw"].plot([], [], color="#ffb454", lw=1.6)
    ln["v_raw"], = axes["v_raw"].plot([], [], color="#6b7f99", lw=0.8)
    ln["v_base"], = axes["v_raw"].plot([], [], color="#ffb454", lw=1.6)
    ln["h_dev"], = axes["h_dev"].plot([], [], color="#4cc9f0", lw=1.2)
    ln["v_dev"], = axes["v_dev"].plot([], [], color="#80ed99", lw=1.2)
    for k in ("h_dev", "v_dev"):
        axes[k].set_ylim(-450, 450)
        for s_ in (1, -1):
            axes[k].axhline(s_ * THR_FLOOR, color="#ff5d73", ls=":", lw=1)
    ax_cmd.set_yticks(range(5))
    ax_cmd.set_yticklabels(["LEFT", "RIGHT", "UP", "DOWN", "BLINK"])
    ax_cmd.set_ylim(-0.7, 4.7)
    ln["truth"], = ax_cmd.plot([], [], color="#d9e2ee", lw=1.5, alpha=0.85)
    ln["cmd"], = ax_cmd.plot([], [], color="#ffd166", lw=6, solid_capstyle="butt", alpha=0.9)
    ln["rej"], = ax_cmd.plot([], [], "x", color="#ff5d73", ms=8, mew=2)
    ax_st.set_title("Detected command / status", loc="left", fontsize=8, color=SUB, pad=2)
    ax_lg.set_title("Event log", loc="left", fontsize=8, color=SUB, pad=2)
    big = ax_st.text(0.03, 0.50, "", va="center", fontsize=26, fontweight="bold", transform=ax_st.transAxes)
    st = ax_st.text(0.50, 0.93, "", va="top", fontsize=8.5, family="monospace", transform=ax_st.transAxes)
    lg = [ax_lg.text(0.02, 0.95 - 0.118 * i, "", va="top", fontsize=7.6, family="monospace",
                     transform=ax_lg.transAxes) for i in range(8)]
    colors = {"cmd": "#ffd166", "blink": "#ff9ff3", "reject": "#ff5d73", "step": "#4cc9f0",
              "info": "#8fa3b8", "done": "#80ed99"}

    def update(_):
        rows = list(sim.trace)
        if len(rows) < 5:
            return
        a = np.array(rows)
        t = a[:, 0]
        now = t[-1]
        x0 = now - 10
        for k in ("h", "v"):
            col = 1 if k == "h" else 2
            ln[f"{k}_raw"].set_data(t, a[:, col])
            ln[f"{k}_base"].set_data(t, a[:, 3 if k == "h" else 4])
            ln[f"{k}_dev"].set_data(t, a[:, 5 if k == "h" else 6])
            vis = a[t > x0]
            axes[f"{k}_raw"].set_ylim(vis[:, col].min() - 60, vis[:, col].max() + 60)
        for ax in list(axes.values()) + [ax_cmd]:
            ax.set_xlim(x0, now + 0.2)
        hist = list(sim.cmd_hist)
        ln["cmd"].set_data([h[0] for h in hist], [CMD_ROW[h[1]] if h[1] else np.nan for h in hist])
        th_ = list(sim.truth_hist)
        ln["truth"].set_data([h[0] for h in th_], [CMD_ROW.get(h[1], np.nan) for h in th_])
        rej = [e[0] for e in list(sim.pipe.events) if e[1] == "reject" and e[0] > x0]
        ln["rej"].set_data(rej, [2] * len(rej))
        cmd = sim.pipe.cmd if sim.pipe.calibrated else None
        big.set_text(f"> {cmd.upper()}" if cmd else "- idle -")
        big.set_color("#ffd166" if cmd else "#5f7389")
        a_, p_ = sim.arm, sim.pipe
        n_rej = sum(1 for e in list(p_.events) if e[1] == "reject")
        st.set_text("\n".join([
            f"mode     {sim.args.mode.upper()}",
            f"pan      {a_.pan:+.2f} rad",
            f"reach    {a_.s * 100:3.0f} %",
            f"gripper  {'CLOSED' if a_.gripper_closed else 'open'}",
            f"thr H/V  {p_.thr[0]:.0f}/{p_.thr[1]:.0f} uV",
            f"rejected {n_rej} involuntary",
            f"compute  {sim.compute_ms:.2f} ms/20ms"]))
        lines = list(sim.log_lines)[-8:]
        for i, tx in enumerate(lg):
            if i < len(lines):
                tx.set_text(lines[i][1][:70]); tx.set_color(colors.get(lines[i][0], FG))
            else:
                tx.set_text("")
    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    fig._anim = anim
    plt.show()


# ======================================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["robust", "naive"], default="robust")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--speed", type=float, default=1.0, help="wall-clock speed-up of the scenario")
    ap.add_argument("--duration", type=float, default=0, help="stop after N simulated seconds")
    ap.add_argument("--clean", action="store_true", help="no natural blinks / glances")
    ap.add_argument("--no-dashboard", action="store_true")
    ap.add_argument("--no-ros", action="store_true", help="signal-processing only, don't talk to Gazebo")
    args = ap.parse_args()

    sim = Sim(args)
    if not args.no_ros:
        sim.init_ros()
        threading.Thread(target=sim.poll_cube, daemon=True).start()
        time.sleep(1.0)
    if args.no_dashboard:
        try:
            sim.run()
        except KeyboardInterrupt:
            pass
        sys.stdout.flush()
        os._exit(0)             # skip noisy rclpy spin-thread teardown
    th = threading.Thread(target=sim.run, daemon=True)
    th.start()
    try:
        dashboard(sim)
    finally:
        sim.stop = True


if __name__ == "__main__":
    main()
