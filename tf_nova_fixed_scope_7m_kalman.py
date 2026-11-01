#!/usr/bin/env python3
"""Lightweight fixed-direction TF-Nova range scope for Raspberry Pi.

No servo, GPIO, web server, or heavy plotting library is used.
LiDAR: /dev/ttyAMA0 at 115200 baud. Press Esc or close the window to quit.
"""
import math
import sys
import time
import threading
from collections import deque

try:
    import serial
    import tkinter as tk
except ImportError as exc:
    print(f"Missing package: {exc}\nInstall with: sudo apt install python3-serial python3-tk", file=sys.stderr)
    raise SystemExit(1)

PORT = "/dev/ttyAMA0"
BAUD = 115200
MAX_RANGE_CM = 700
ZOOM_RANGE_CM = 100
HISTORY_SECONDS = 12
SAMPLE_SECONDS = 0.10


class KalmanFilter1D:
    """Scalar Kalman filter for a fixed LiDAR's range measurement (cm)."""
    def __init__(self, process_variance=0.5, measurement_variance=9.0):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = None
        self.error_covariance = 1.0
        self.gain = 0.0

    def update(self, measurement):
        if self.estimate is None:
            self.estimate = float(measurement)
            self.error_covariance = 1.0
            self.gain = 1.0
            return self.estimate
        # Prediction: distance is approximately constant between rapid samples.
        predicted_estimate = self.estimate
        predicted_covariance = self.error_covariance + self.process_variance
        self.gain = predicted_covariance / (predicted_covariance + self.measurement_variance)
        self.estimate = predicted_estimate + self.gain * (measurement - predicted_estimate)
        self.error_covariance = (1.0 - self.gain) * predicted_covariance
        return self.estimate


state = {
    "distance_cm": None, "estimated_distance_cm": None, "kalman_gain": 0.0,
    "peak": 0, "confidence": 0, "last_seen": 0.0,
    "valid": 0, "bad_checksum": 0, "error": None,
}
lock = threading.Lock()
stop_event = threading.Event()


def lidar_reader():
    buf = bytearray()
    kalman = KalmanFilter1D()
    try:
        with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
            ser.reset_input_buffer()
            while not stop_event.is_set():
                chunk = ser.read(max(1, ser.in_waiting))
                if chunk:
                    buf.extend(chunk)
                while len(buf) >= 9:
                    start = buf.find(b"\x59\x59")
                    if start < 0:
                        keep = 1 if buf[-1] == 0x59 else 0
                        buf[:] = buf[-1:] if keep else b""
                        break
                    if start:
                        del buf[:start]
                    if len(buf) < 9:
                        break
                    frame = bytes(buf[:9])
                    if (sum(frame[:8]) & 0xFF) != frame[8]:
                        with lock:
                            state["bad_checksum"] += 1
                        del buf[0]
                        continue
                    del buf[:9]
                    distance_cm = frame[2] | (frame[3] << 8)
                    estimated_cm = kalman.update(distance_cm)
                    with lock:
                        state["distance_cm"] = distance_cm
                        state["estimated_distance_cm"] = estimated_cm
                        state["kalman_gain"] = kalman.gain
                        state["peak"] = frame[4] | (frame[5] << 8)
                        state["confidence"] = frame[7]
                        state["last_seen"] = time.monotonic()
                        state["valid"] += 1
                if len(buf) > 4096:
                    del buf[:-16]
    except Exception as exc:
        with lock:
            state["error"] = str(exc)


class FixedScope:
    def __init__(self, root):
        self.root = root
        root.title("TF-Nova | Fixed Range Scope")
        root.configure(bg="#07110f")
        root.minsize(720, 520)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<Escape>", lambda _event: self.close())
        self.w, self.h = 900, 650
        self.canvas = tk.Canvas(root, width=self.w, height=self.h, bg="#07110f", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=8, pady=8)
        self.status = tk.Label(root, text="Connecting to TF-Nova…", bg="#07110f", fg="#8ca99e",
                               font=("DejaVu Sans Mono", 10), anchor="w")
        self.status.pack(fill="x", padx=16, pady=(0, 10))
        self.history = deque(maxlen=180)
        self.last_sample = 0.0
        self.last_seen_count = -1
        self.tick()

    def draw(self):
        c = self.canvas
        c.delete("all")
        w = max(self.w, c.winfo_width())
        h = max(self.h, c.winfo_height())
        c.create_text(22, 18, text="TF-NOVA  /  FIXED RANGE SCOPE", fill="#55edb4",
                      font=("DejaVu Sans Mono", 14, "bold"), anchor="nw")
        c.create_text(w - 22, 22, text="FIXED DIRECTION  •  0–7 m", fill="#64877b",
                      font=("DejaVu Sans Mono", 9), anchor="ne")

        with lock:
            d = state["distance_cm"]
            estimated = state["estimated_distance_cm"]
            kalman_gain = state["kalman_gain"]
            peak = state["peak"]
            confidence = state["confidence"]
            last_seen = state["last_seen"]
            valid = state["valid"]
            bad = state["bad_checksum"]
            error = state["error"]
        now = time.monotonic()
        fresh = d is not None and now - last_seen < 1.0
        if fresh and d > 0 and now - self.last_sample >= SAMPLE_SECONDS:
            self.history.append((now, min(d, MAX_RANGE_CM), min(estimated if estimated is not None else d, MAX_RANGE_CM)))
            self.last_sample = now

        # Main 0–7 m semicircular scope, left side.
        cx, cy = w * 0.28, h * 0.62
        radius = min(w * 0.255, h * 0.36)
        c.create_text(cx, 63, text="MAIN RANGE  /  0–7 m", fill="#8ca99e",
                      font=("DejaVu Sans Mono", 10, "bold"), anchor="center")
        for meters in range(1, 8):
            r = radius * (meters * 100) / MAX_RANGE_CM
            c.create_arc(cx-r, cy-r, cx+r, cy+r, start=0, extent=180,
                         style="arc", outline="#1b4d3e" if meters < 7 else "#2d8063", width=1)
            c.create_text(cx + 5, cy-r+3, text=f"{meters}m", fill="#618a7a",
                          font=("DejaVu Sans Mono", 8), anchor="nw")
        c.create_line(cx-radius, cy, cx+radius, cy, fill="#1b4d3e", width=1)
        c.create_line(cx, cy, cx, cy-radius, fill="#2d8063", width=2)
        c.create_oval(cx-5, cy-5, cx+5, cy+5, fill="#55edb4", outline="")
        if fresh and 0 < d <= MAX_RANGE_CM:
            r = radius * d / MAX_RANGE_CM
            yy = cy - r
            c.create_line(cx-14, yy, cx+14, yy, fill="#55edb4", width=2)
            c.create_oval(cx-6, yy-6, cx+6, yy+6, fill="#eafff6", outline="#55edb4", width=2)
        elif fresh and d > MAX_RANGE_CM:
            c.create_text(cx, cy-radius-18, text=f"OUT OF RANGE: {d/100:.2f} m",
                          fill="#e7c66a", font=("DejaVu Sans Mono", 9), anchor="center")
        if fresh and estimated is not None and 0 < estimated <= MAX_RANGE_CM:
            er = radius * estimated / MAX_RANGE_CM
            ey = cy - er
            c.create_oval(cx-4, ey-4, cx+4, ey+4, fill="#e7c66a", outline="#07110f", width=1)

        # Large numeric readout, upper right.
        px, py = w * 0.56, 66
        c.create_text(px, py, text="MEASURED DISTANCE", fill="#8ca99e",
                      font=("DejaVu Sans Mono", 10), anchor="nw")
        if fresh:
            c.create_text(px, py+20, text=f"{d/100:.2f} m", fill="#eafff6",
                          font=("DejaVu Sans Mono", 30, "bold"), anchor="nw")
            c.create_text(px, py+62, text=f"RAW: {d} cm   /   {d*10} mm", fill="#8ca99e",
                          font=("DejaVu Sans Mono", 10), anchor="nw")
            if estimated is not None:
                c.create_text(px, py+82, text=f"KALMAN ESTIMATE: {estimated/100:.2f} m  ({estimated:.1f} cm)",
                              fill="#e7c66a", font=("DejaVu Sans Mono", 10, "bold"), anchor="nw")
            if d < 50:
                zone, color = "VERY CLOSE", "#ff7777"
            elif d < 100:
                zone, color = "UNDER 1 METRE", "#e7c66a"
            else:
                zone, color = "RANGE > 1 m", "#55edb4"
            c.create_text(px, py+106, text=zone, fill=color,
                          font=("DejaVu Sans Mono", 10, "bold"), anchor="nw")
        else:
            c.create_text(px, py+22, text="— — —", fill="#64877b",
                          font=("DejaVu Sans Mono", 26, "bold"), anchor="nw")
            c.create_text(px, py+64, text="Waiting for valid measurement", fill="#64877b",
                          font=("DejaVu Sans Mono", 9), anchor="nw")

        # Magnified 0–100 cm inset, with the requested 25 cm graduations.
        zx, zy = w * 0.75, h * 0.64
        zrad = min(w * 0.17, h * 0.245)
        c.create_text(zx, h*0.285, text="CLOSE-UP  /  0–100 cm", fill="#8ca99e",
                      font=("DejaVu Sans Mono", 10, "bold"), anchor="center")
        for cm in (25, 50, 75, 100):
            rr = zrad * cm / ZOOM_RANGE_CM
            c.create_arc(zx-rr, zy-rr, zx+rr, zy+rr, start=0, extent=180,
                         style="arc", outline="#2d8063" if cm == 100 else "#1b4d3e", width=2 if cm == 100 else 1)
            c.create_text(zx+5, zy-rr+3, text=f"{cm} cm" if cm < 100 else "1 m",
                          fill="#72a28f", font=("DejaVu Sans Mono", 8), anchor="nw")
        c.create_line(zx-zrad, zy, zx+zrad, zy, fill="#1b4d3e", width=1)
        c.create_line(zx, zy, zx, zy-zrad, fill="#2d8063", width=2)
        c.create_oval(zx-5, zy-5, zx+5, zy+5, fill="#55edb4", outline="")
        if fresh and 0 < d <= ZOOM_RANGE_CM:
            rr = zrad * d / ZOOM_RANGE_CM
            yy = zy - rr
            c.create_line(zx-13, yy, zx+13, yy, fill="#e7c66a", width=2)
            c.create_oval(zx-6, yy-6, zx+6, yy+6, fill="#fff3cc", outline="#e7c66a", width=2)
        elif fresh and d > ZOOM_RANGE_CM:
            c.create_text(zx, zy-zrad-18, text="Beyond 1 m zoom", fill="#64877b",
                          font=("DejaVu Sans Mono", 8), anchor="center")
        if fresh and estimated is not None and 0 < estimated <= ZOOM_RANGE_CM:
            er = zrad * estimated / ZOOM_RANGE_CM
            ey = zy - er
            c.create_oval(zx-4, ey-4, zx+4, ey+4, fill="#e7c66a", outline="#07110f", width=1)

        # Lightweight diagnostic line below the zoom scope.
        diag_y = min(h-155, zy+12)
        c.create_text(w*0.56, diag_y, text=f"PEAK {peak}   CONF {confidence}   VALID {valid}",
                      fill="#8ca99e", font=("DejaVu Sans Mono", 8), anchor="nw")
        c.create_text(w*0.56, diag_y+17, text=f"KALMAN GAIN {kalman_gain:.3f}   PROCESS Q 0.5   MEASUREMENT R 9.0",
                      fill="#e7c66a", font=("DejaVu Sans Mono", 7), anchor="nw")
        c.create_text(w*0.56, diag_y+34, text=f"BAD CHECKSUM {bad}",
                      fill="#8ca99e", font=("DejaVu Sans Mono", 8), anchor="nw")

        # Rolling history now uses a 0–7 m scale.
        gx1, gy1, gx2, gy2 = 24, h-115, w-24, h-27
        c.create_text(gx1, gy1-17, text=f"DISTANCE HISTORY  /  LAST {HISTORY_SECONDS} SECONDS  (0–7 m)  RAW=GREEN  KALMAN=AMBER",
                      fill="#8ca99e", font=("DejaVu Sans Mono", 9), anchor="nw")
        c.create_rectangle(gx1, gy1, gx2, gy2, outline="#1b4d3e", width=1)
        for m in (1, 3, 5, 7):
            yy = gy2 - (gy2-gy1) * (m*100) / MAX_RANGE_CM
            c.create_line(gx1, yy, gx2, yy, fill="#12382e", width=1)
            c.create_text(gx2-4, yy-3, text=f"{m}m", fill="#52796c",
                          font=("DejaVu Sans Mono", 7), anchor="se")
        points = []
        oldest = now - HISTORY_SECONDS
        visible = [(t, raw, est) for t, raw, est in self.history if t >= oldest]
        raw_points, est_points = [], []
        for t, raw, est in visible:
            x = gx1 + (t-oldest) / HISTORY_SECONDS * (gx2-gx1)
            y_raw = gy2 - (raw / MAX_RANGE_CM) * (gy2-gy1)
            y_est = gy2 - (est / MAX_RANGE_CM) * (gy2-gy1)
            raw_points.extend((x, y_raw))
            est_points.extend((x, y_est))
        if len(raw_points) >= 4:
            c.create_line(*raw_points, fill="#55edb4", width=1, smooth=False)
            c.create_line(*est_points, fill="#e7c66a", width=2, smooth=False)
        elif len(raw_points) == 2:
            x, y_raw = raw_points
            _, y_est = est_points
            c.create_oval(x-2, y_raw-2, x+2, y_raw+2, fill="#55edb4", outline="")
            c.create_oval(x-2, y_est-2, x+2, y_est+2, fill="#e7c66a", outline="")

        if error:
            status = f"UART ERROR: {error}"
        elif not fresh:
            status = f"No fresh data | port {PORT} | valid frames {valid}"
        else:
            status = f"LIVE  |  {PORT} @ {BAUD}  |  max 7 m  |  Esc to quit"
        self.status.config(text=status, fg="#ff7777" if error else "#8ca99e")

    def tick(self):
        self.draw()
        if not stop_event.is_set():
            self.root.after(80, self.tick)

    def close(self):
        stop_event.set()
        self.root.destroy()


def main():
    threading.Thread(target=lidar_reader, daemon=True).start()
    root = tk.Tk()
    try:
        FixedScope(root)
        root.mainloop()
    except Exception as exc:
        stop_event.set()
        print(f"Could not start scope: {exc}", file=sys.stderr)
        raise SystemExit(2)
    finally:
        stop_event.set()


if __name__ == "__main__":
    main()
