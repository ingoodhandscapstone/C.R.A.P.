import csv
import math
import queue
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk

import paho.mqtt.client as mqtt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.ticker import MultipleLocator


MQTT_HOST = "localhost"
MQTT_PORT = 1883
CMD_TOPIC = "system/command"
CAL_TOPIC = "system/calibration_status"
PLOT_TICK_SECONDS = 0.10
MAX_PLOT_POINTS = 2500
LIVE_WINDOW_SECONDS = 30.0
Y_FOLLOW_SECONDS = 3.0
UI_POLL_MS = 50

CMD = {
    "CALIBRATE": 7,
    "START": 5,
    "STOP": 6,
    "CALIBRATION_COMPLETED": 9,
}

SESSIONS = [
    {"key": "WRIST", "label": "Wrist Orientation", "cmd": 1, "topics": ["orientation/WRIST_X", "orientation/WRIST_Y"]},
    {"key": "POINTER", "label": "Pointer Finger", "cmd": 3, "topics": ["joint/POINTER_MCP", "joint/POINTER_PIP", "joint/POINTER_DIP", "abduction/POINTER"]},
    {"key": "SPO2", "label": "Blood Oxygen", "cmd": 4, "topics": ["spo2/WRIST"]},
    {"key": "MIDDLE", "label": "Middle Finger", "cmd": 10, "topics": ["joint/MIDDLE_MCP", "joint/MIDDLE_PIP", "joint/MIDDLE_DIP", "abduction/MIDDLE"]},
    {"key": "RING", "label": "Ring Finger", "cmd": 11, "topics": ["joint/RING_MCP", "joint/RING_PIP", "joint/RING_DIP", "abduction/RING"]},
    {"key": "PINKY", "label": "Pinky Finger", "cmd": 12, "topics": ["joint/PINKY_MCP", "joint/PINKY_PIP", "joint/PINKY_DIP", "abduction/PINKY"]},
    {"key": "THUMB", "label": "Thumb Finger", "cmd": 13, "topics": ["joint/THUMB_MCP", "joint/THUMB_PIP", "abduction/THUMB"]},
    {"key": "POINTER_MIDDLE", "label": "Pointer + Middle", "cmd": 14, "topics": ["joint/POINTER_MCP", "joint/POINTER_PIP", "joint/POINTER_DIP", "joint/MIDDLE_MCP", "joint/MIDDLE_PIP", "joint/MIDDLE_DIP", "abduction/POINTER", "abduction/MIDDLE"]},
    {"key": "POINTER_WRIST", "label": "Pointer + Wrist", "cmd": 15, "topics": ["joint/POINTER_MCP", "joint/POINTER_PIP", "joint/POINTER_DIP", "abduction/POINTER", "orientation/WRIST_X", "orientation/WRIST_Y"]},
    {"key": "GRIPPER_POINTER", "label": "Gripper Pointer", "cmd": 16, "topics": ["force/POINTER"]},
    {"key": "GRIPPER_MIDDLE", "label": "Gripper Middle", "cmd": 17, "topics": ["force/MIDDLE"]},
    {"key": "GRIPPER_RING", "label": "Gripper Ring", "cmd": 18, "topics": ["force/RING"]},
    {"key": "GRIPPER_PINKY", "label": "Gripper Pinky", "cmd": 19, "topics": ["force/PINKY"]},
    {"key": "GRIPPER_THUMB", "label": "Gripper Thumb", "cmd": 20, "topics": ["force/THUMB"]},
    {"key": "GRIPPER_POINTER_MIDDLE", "label": "Gripper Pointer + Middle", "cmd": 21, "topics": ["force/POINTER", "force/MIDDLE"]},
    {"key": "GRIPPER_ALL", "label": "Gripper All", "cmd": 22, "topics": ["force/POINTER", "force/MIDDLE", "force/RING", "force/PINKY", "force/THUMB"]},
]

ALL_TOPICS = sorted({CAL_TOPIC, *[t for s in SESSIONS for t in s["topics"]]})


class SessionUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("In Good Hands - Session Console")
        self.geometry("1260x820")
        self.minsize(1080, 720)
        self.configure(bg="#edf2f8")

        self.selected = None
        self.waiting_for_calibration = False
        self.running = False
        self.start_time = None
        self.series = {}
        self.last_values = {}
        self.last_plot_elapsed = {}
        self.records = []
        self.lines = {}
        self.axes = []
        self.canvas = None
        self.figure = None
        self.graph_scroll_canvas = None
        self.graph_scroll_window = None
        self.inbox = queue.Queue()

        self._build_ui()
        self._setup_mqtt()
        self.after(UI_POLL_MS, self._poll_inbox)
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _build_ui(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#edf2f8")
        style.configure("Card.TFrame", background="#ffffff")
        style.configure("TLabel", background="#edf2f8", foreground="#1a3656", font=("Segoe UI", 11))
        style.configure("Card.TLabel", background="#ffffff", foreground="#1a3656", font=("Segoe UI", 11))
        style.configure("Header.TLabel", background="#ffffff", foreground="#0f2742", font=("Segoe UI", 24, "bold"))
        style.configure("Subheader.TLabel", background="#ffffff", foreground="#3d5d7a", font=("Segoe UI", 11))
        style.configure("Section.TLabel", background="#ffffff", foreground="#0f2742", font=("Segoe UI", 12, "bold"))
        style.configure("Prompt.TLabel", background="#ffffff", foreground="#294a66", font=("Segoe UI", 11))
        style.configure("Status.TLabel", background="#ffffff", foreground="#1f4a72", font=("Segoe UI", 10, "bold"))

        style.configure("Session.TButton", font=("Segoe UI", 10, "bold"), padding=(10, 8))
        style.map("Session.TButton", background=[("active", "#e3f0ff")])
        style.configure("Accent.TButton", font=("Segoe UI", 10, "bold"), padding=(12, 9))
        style.map("Accent.TButton", background=[("active", "#d9ebff")])
        style.configure("Primary.TButton", font=("Segoe UI", 10, "bold"), padding=(12, 9))
        style.map("Primary.TButton", background=[("active", "#d7f4ea")])
        style.configure("Danger.TButton", font=("Segoe UI", 10, "bold"), padding=(12, 9))
        style.map("Danger.TButton", background=[("active", "#ffe0de")])

        top = ttk.Frame(self, style="Card.TFrame", padding=(18, 14))
        top.pack(fill="x", padx=16, pady=(16, 8))
        ttk.Label(top, text="In Good Hands Session Console", style="Header.TLabel").pack(anchor="w")
        ttk.Label(top, text="MQTT command, calibration, live graphing, and export flow", style="Subheader.TLabel").pack(anchor="w", pady=(2, 10))

        self.status = tk.StringVar(value="Connecting to MQTT...")
        ttk.Label(top, textvariable=self.status, style="Status.TLabel").pack(anchor="w")

        self.session_frame = ttk.Frame(self, style="Card.TFrame", padding=(16, 14))
        self.session_frame.pack(fill="x", padx=16, pady=8)
        ttk.Label(self.session_frame, text="Choose Session Type", style="Section.TLabel").pack(anchor="w", pady=(0, 8))

        grid = ttk.Frame(self.session_frame, style="Card.TFrame")
        grid.pack(fill="x")
        for i, s in enumerate(SESSIONS):
            ttk.Button(grid, text=s["label"], style="Session.TButton", command=lambda sess=s: self._select_session(sess)).grid(row=i // 4, column=i % 4, sticky="ew", padx=4, pady=4)
        for i in range(4):
            grid.grid_columnconfigure(i, weight=1)

        self.action_frame = ttk.Frame(self, style="Card.TFrame", padding=(16, 14))
        self.action_frame.pack(fill="x", padx=16, pady=(4, 8))
        self.prompt = tk.StringVar(value="Select a session to begin.")
        ttk.Label(self.action_frame, textvariable=self.prompt, style="Prompt.TLabel").pack(anchor="w", pady=(0, 8))

        btns = ttk.Frame(self.action_frame, style="Card.TFrame")
        btns.pack(anchor="w")
        self.cal_btn = ttk.Button(btns, text="I Am In Position - Calibrate", style="Accent.TButton", command=self._start_calibration)
        self.start_btn = ttk.Button(btns, text="Start Session", style="Primary.TButton", command=self._start_session)
        self.stop_btn = ttk.Button(btns, text="Stop Session", style="Danger.TButton", command=self._stop_session)
        self.cal_btn.pack_forget()
        self.start_btn.pack_forget()
        self.stop_btn.pack_forget()

        graph_card = ttk.Frame(self, style="Card.TFrame", padding=(0, 0))
        graph_card.pack(fill="both", expand=True, padx=16, pady=(0, 16))

        self.graph_scroll_canvas = tk.Canvas(graph_card, bg="#ffffff", highlightthickness=0, borderwidth=0)
        graph_scrollbar = ttk.Scrollbar(graph_card, orient="vertical", command=self.graph_scroll_canvas.yview)
        self.graph_scroll_canvas.configure(yscrollcommand=graph_scrollbar.set)

        graph_scrollbar.pack(side="right", fill="y")
        self.graph_scroll_canvas.pack(side="left", fill="both", expand=True)

        self.graph_frame = ttk.Frame(self.graph_scroll_canvas, style="Card.TFrame", padding=(12, 12))
        self.graph_scroll_window = self.graph_scroll_canvas.create_window((0, 0), window=self.graph_frame, anchor="nw")
        self.graph_frame.bind("<Configure>", self._on_graph_frame_configure)
        self.graph_scroll_canvas.bind("<Configure>", self._on_graph_canvas_configure)
        self.bind_all("<MouseWheel>", self._on_mouse_wheel)

    def _setup_mqtt(self):
        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_message = self._on_message
        self.mqtt.connect_async(MQTT_HOST, MQTT_PORT, keepalive=30)
        self.mqtt.loop_start()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            for topic in ALL_TOPICS:
                client.subscribe(topic, qos=1)
            self.after(0, lambda: self.status.set(f"Connected to MQTT at {MQTT_HOST}:{MQTT_PORT}"))
        else:
            self.after(0, lambda: self.status.set(f"MQTT connection failed: {reason_code}"))

    def _on_message(self, client, userdata, msg):
        self.inbox.put((msg.topic, msg.payload.decode("utf-8").strip(), datetime.now()))

    def _send_command(self, value):
        self.mqtt.publish(CMD_TOPIC, str(value), qos=1)

    def _select_session(self, session):
        self.selected = session
        self.running = False
        self.waiting_for_calibration = False
        self._send_command(session["cmd"])
        self.prompt.set("Place your hand flat on the table, then press calibrate.")
        self.cal_btn.pack(side="left", padx=(0, 8))
        self.start_btn.pack_forget()
        self.stop_btn.pack_forget()
        self.status.set(f"Configured: {session['label']} (command {session['cmd']})")
        self._clear_plot()

    def _start_calibration(self):
        if not self.selected:
            return
        self._send_command(CMD["CALIBRATE"])
        self.waiting_for_calibration = True
        self.prompt.set("Calibrating... keep hand flat and still. Waiting for completion message.")
        self.cal_btn.pack_forget()
        self.start_btn.pack_forget()
        self.stop_btn.pack_forget()

    def _start_session(self):
        if not self.selected:
            return
        self.running = True
        self.waiting_for_calibration = False
        self.start_time = datetime.now()
        self.series = {topic: {"x": [], "y": []} for topic in self.selected["topics"]}
        self.last_values = {topic: 0.0 for topic in self.selected["topics"]}
        self.last_plot_elapsed = {topic: -PLOT_TICK_SECONDS for topic in self.selected["topics"]}
        self.records = []
        self._build_plot(self.selected["topics"])
        self._send_command(CMD["START"])
        self.prompt.set("Session running.")
        self.cal_btn.pack_forget()
        self.start_btn.pack_forget()
        self.stop_btn.pack(side="left")
        if self._append_heartbeat_point():
            self._refresh_plot()

    def _stop_session(self):
        if not self.selected:
            return
        self._send_command(CMD["STOP"])
        self.running = False
        saved = self._save_csv()
        self.status.set(f"Session stopped. Saved CSV: {saved}")
        self.prompt.set("Select a session to begin.")
        self.cal_btn.pack_forget()
        self.start_btn.pack_forget()
        self.stop_btn.pack_forget()
        self.selected = None
        self.waiting_for_calibration = False
        self._clear_plot()

    def _poll_inbox(self):
        updated = False
        while not self.inbox.empty():
            topic, payload, ts = self.inbox.get()
            if topic == CAL_TOPIC:
                if self.waiting_for_calibration and payload == str(CMD["CALIBRATION_COMPLETED"]):
                    self.waiting_for_calibration = False
                    self.prompt.set("Calibration complete. Keep hand in place until you press Start Session.")
                    self.start_btn.pack(side="left")
                    self.status.set("Calibration completed.")
                continue
            if not self.running or not self.selected or topic not in self.series:
                continue
            try:
                value = float(payload)
            except ValueError:
                continue
            self.last_values[topic] = value
            elapsed = (ts - self.start_time).total_seconds()
            self.records.append([self.selected["key"], self.selected["label"], ts.isoformat(), f"{elapsed:.6f}", topic, f"{value:.6f}"])
            updated = True
        heartbeat = self._append_heartbeat_point()
        if updated or heartbeat:
            self._refresh_plot()
        self.after(UI_POLL_MS, self._poll_inbox)

    def _build_plot(self, topics):
        self._clear_plot()
        height_per_plot = 4.2
        fig_height = max(9.0, len(topics) * height_per_plot)
        self.figure = Figure(figsize=(11.4, fig_height), dpi=100, facecolor="#ffffff")
        self.axes = self.figure.subplots(len(topics), 1, sharex=True)
        if len(topics) == 1:
            self.axes = [self.axes]
        self.lines = {}
        palette = ["#2b7fff", "#1f9d73", "#ef7d22", "#cf2e2e", "#009fb7", "#5b6cff", "#2f855a", "#8f5f2a"]
        for ax, topic in zip(self.axes, topics):
            color = palette[len(self.lines) % len(palette)]
            line, = ax.plot([], [], color=color, linewidth=2.2)
            ax.set_facecolor("#ffffff")
            ax.grid(True, which="major", color="#c8d8e8", linewidth=0.95)
            ax.grid(True, which="minor", color="#e9eff6", linewidth=0.6)
            ax.set_ylabel(topic.split("/")[-1], color="#1a3656")
            ax.tick_params(axis="y", colors="#17324d", labelsize=10, width=1.0, length=4)
            ax.tick_params(axis="x", colors="#17324d", labelsize=10, width=1.0, length=4)
            ax.spines["left"].set_color("#bfd1e3")
            ax.spines["bottom"].set_color("#bfd1e3")
            ax.spines["right"].set_color("#bfd1e3")
            ax.spines["top"].set_color("#bfd1e3")
            self.lines[topic] = line
        self.axes[-1].set_xlabel("Time (s)", color="#1a3656")
        self.figure.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.figure, self.graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.graph_scroll_canvas.update_idletasks()
        self.graph_scroll_canvas.configure(scrollregion=self.graph_scroll_canvas.bbox("all"))

    def _refresh_plot(self):
        for topic, line in self.lines.items():
            line.set_data(self.series[topic]["x"], self.series[topic]["y"])
            ax = line.axes
            x_vals = self.series[topic]["x"]
            y_vals = self.series[topic]["y"]
            if not x_vals:
                continue

            x_max = x_vals[-1]
            window_start = math.floor(x_max / LIVE_WINDOW_SECONDS) * LIVE_WINDOW_SECONDS
            window_end = window_start + LIVE_WINDOW_SECONDS
            ax.set_xlim(window_start, window_end)
            ax.xaxis.set_major_locator(MultipleLocator(5.0))
            ax.xaxis.set_minor_locator(MultipleLocator(1.0))

            y_follow_start = max(window_start, x_max - Y_FOLLOW_SECONDS)
            visible_y = [y for x, y in zip(x_vals, y_vals) if y_follow_start <= x <= x_max]
            if not visible_y:
                visible_y = [y for x, y in zip(x_vals, y_vals) if window_start <= x <= window_end]
            if not visible_y:
                visible_y = [y_vals[-1]]

            y_min = min(visible_y)
            y_max = max(visible_y)
            y_low = math.floor(y_min)
            y_high = math.ceil(y_max)
            if y_low == y_high:
                y_low -= 2
                y_high += 2
            else:
                y_low -= 1
                y_high += 1

            ax.set_ylim(y_low, y_high)
            ax.yaxis.set_major_locator(MultipleLocator(5.0))
            ax.yaxis.set_minor_locator(MultipleLocator(1.0))

        if self.canvas:
            self.canvas.draw()

    def _clear_plot(self):
        if self.canvas:
            self.canvas.get_tk_widget().destroy()
        self.canvas = None
        self.figure = None
        self.lines = {}
        self.axes = []

    def _append_heartbeat_point(self):
        if not self.running or not self.start_time or not self.series:
            return False
        elapsed = (datetime.now() - self.start_time).total_seconds()
        changed = False
        for topic, data in self.series.items():
            last_elapsed = self.last_plot_elapsed.get(topic, -PLOT_TICK_SECONDS)
            if elapsed - last_elapsed < PLOT_TICK_SECONDS:
                continue
            data["x"].append(elapsed)
            data["y"].append(self.last_values.get(topic, 0.0))
            self.last_plot_elapsed[topic] = elapsed
            if len(data["x"]) > MAX_PLOT_POINTS:
                overflow = len(data["x"]) - MAX_PLOT_POINTS
                del data["x"][:overflow]
                del data["y"][:overflow]
            changed = True
        return changed

    def _on_graph_frame_configure(self, event):
        if self.graph_scroll_canvas:
            self.graph_scroll_canvas.configure(scrollregion=self.graph_scroll_canvas.bbox("all"))

    def _on_graph_canvas_configure(self, event):
        if self.graph_scroll_canvas and self.graph_scroll_window is not None:
            self.graph_scroll_canvas.itemconfigure(self.graph_scroll_window, width=event.width)

    def _on_mouse_wheel(self, event):
        if self.graph_scroll_canvas is None:
            return
        self.graph_scroll_canvas.yview_scroll(int(-event.delta / 120), "units")

    def _save_csv(self):
        out_dir = Path("session_exports")
        out_dir.mkdir(exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        name = self.selected["key"] if self.selected else "session"
        path = out_dir / f"{name}_{stamp}.csv"
        with path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["session_key", "session_label", "timestamp_iso", "elapsed_seconds", "topic", "value"])
            w.writerows(self.records)
        return str(path.resolve())

    def _close(self):
        try:
            self.mqtt.loop_stop()
            self.mqtt.disconnect()
        finally:
            self.destroy()


if __name__ == "__main__":
    SessionUI().mainloop()
