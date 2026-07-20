# -*- coding: gbk -*-
"""AGV底盘串口调试与二维位姿显示工具。"""

from __future__ import annotations

import csv
import math
import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


PROTOCOL_VERSION = 1
DEFAULT_BAUDRATE = 115200
SERIAL_POLL_MS = 40
SIMULATION_PERIOD_MS = 50
FORWARD_TEST_MIN_DISTANCE_MM = 300
FORWARD_TEST_MAX_DISTANCE_MM = 3000
FORWARD_TEST_MIN_SPEED_MM_S = 50
FORWARD_TEST_MAX_SPEED_MM_S = 100


@dataclass
class PoseTelemetry:
    timestamp_ms: int = 0
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_deg: float = 0.0
    linear_m_s: float = 0.0
    angular_deg_s: float = 0.0
    left_rpm: float = 0.0
    right_rpm: float = 0.0
    state: str = "IDLE"
    fault: int = 0
    left_target_rpm: float = 0.0
    right_target_rpm: float = 0.0
    left_pwm: int = 0
    right_pwm: int = 0
    left_count: int = 0
    right_count: int = 0
    left_invalid: int = 0
    right_invalid: int = 0


@dataclass
class MotionGoal:
    x_m: float
    y_m: float
    yaw_deg: float
    speed_m_s: float
    sequence: int


def wrap_angle_deg(angle_deg: float) -> float:
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class SerialWorker:
    """在后台线程读取串口，主线程只处理消息队列。"""

    def __init__(self, receive_queue: queue.Queue[tuple[str, object]]) -> None:
        self.receive_queue = receive_queue
        self.port = None
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self.port is not None and self.port.is_open

    def connect(self, port_name: str, baudrate: int) -> None:
        if serial is None:
            raise RuntimeError("未安装pyserial，请先执行 pip install -r tools/requirements.txt")
        self.disconnect()
        self.port = serial.Serial(
            port=port_name,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            write_timeout=0.5,
        )
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=0.8)
        self.thread = None
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
        self.port = None

    def send_line(self, line: str) -> None:
        if not self.connected:
            raise RuntimeError("串口尚未连接")
        payload = (line.rstrip("\r\n") + "\r\n").encode("ascii")
        with self.write_lock:
            self.port.write(payload)
            self.port.flush()

    def _read_loop(self) -> None:
        while not self.stop_event.is_set() and self.connected:
            try:
                raw_line = self.port.readline()
                if raw_line:
                    line = raw_line.decode("ascii", errors="replace").strip("\r\n\x00")
                    if line:
                        self.receive_queue.put(("line", line))
            except Exception as exc:
                self.receive_queue.put(("error", str(exc)))
                break


class CoordinateCanvas(tk.Canvas):
    """显示世界坐标、目标点、车辆姿态和历史轨迹。"""

    def __init__(self, master: tk.Misc, goal_callback) -> None:
        super().__init__(master, background="#f7f9fb", highlightthickness=0)
        self.goal_callback = goal_callback
        self.scale_px_per_m = 90.0
        self.view_center_x = 0.0
        self.view_center_y = 0.0
        self.pose = PoseTelemetry()
        self.goal: MotionGoal | None = None
        self.path: list[tuple[float, float]] = []
        self.pan_start: tuple[int, int] | None = None
        self.pan_world_start: tuple[float, float] | None = None

        self.bind("<Configure>", lambda _event: self.redraw())
        self.bind("<Button-1>", self._on_left_click)
        self.bind("<ButtonPress-3>", self._on_pan_start)
        self.bind("<B3-Motion>", self._on_pan_move)
        self.bind("<MouseWheel>", self._on_mouse_wheel)

    def set_pose(self, pose: PoseTelemetry, append_path: bool = True) -> None:
        self.pose = pose
        if append_path:
            if not self.path:
                self.path.append((pose.x_m, pose.y_m))
            else:
                last_x, last_y = self.path[-1]
                if math.hypot(pose.x_m - last_x, pose.y_m - last_y) >= 0.01:
                    self.path.append((pose.x_m, pose.y_m))
                    if len(self.path) > 20000:
                        self.path = self.path[-10000:]
        self.redraw()

    def set_goal(self, goal: MotionGoal | None) -> None:
        self.goal = goal
        self.redraw()

    def clear_path(self) -> None:
        self.path.clear()
        self.redraw()

    def center_on_vehicle(self) -> None:
        self.view_center_x = self.pose.x_m
        self.view_center_y = self.pose.y_m
        self.redraw()

    def world_to_screen(self, x_m: float, y_m: float) -> tuple[float, float]:
        center_px_x = self.winfo_width() / 2.0
        center_px_y = self.winfo_height() / 2.0
        screen_x = center_px_x + (x_m - self.view_center_x) * self.scale_px_per_m
        screen_y = center_px_y - (y_m - self.view_center_y) * self.scale_px_per_m
        return screen_x, screen_y

    def screen_to_world(self, screen_x: float, screen_y: float) -> tuple[float, float]:
        center_px_x = self.winfo_width() / 2.0
        center_px_y = self.winfo_height() / 2.0
        x_m = self.view_center_x + (screen_x - center_px_x) / self.scale_px_per_m
        y_m = self.view_center_y - (screen_y - center_px_y) / self.scale_px_per_m
        return x_m, y_m

    def redraw(self) -> None:
        self.delete("all")
        width = max(self.winfo_width(), 1)
        height = max(self.winfo_height(), 1)
        self._draw_grid(width, height)
        self._draw_target_line()
        self._draw_path()
        self._draw_goal()
        self._draw_vehicle()
        self.create_text(
            14,
            height - 14,
            anchor="sw",
            fill="#536170",
            font=("Microsoft YaHei UI", 9),
            text=f"缩放: {self.scale_px_per_m:.0f} px/m    右键拖动画布    滚轮缩放",
        )

    def _choose_grid_step(self) -> float:
        target_pixels = 75.0
        raw_step = target_pixels / self.scale_px_per_m
        exponent = math.floor(math.log10(max(raw_step, 1e-6)))
        base = 10.0**exponent
        for factor in (1.0, 2.0, 5.0, 10.0):
            step = base * factor
            if step * self.scale_px_per_m >= target_pixels:
                return step
        return base * 10.0

    def _draw_grid(self, width: int, height: int) -> None:
        step_m = self._choose_grid_step()
        left_world, top_world = self.screen_to_world(0, 0)
        right_world, bottom_world = self.screen_to_world(width, height)
        min_x, max_x = sorted((left_world, right_world))
        min_y, max_y = sorted((top_world, bottom_world))

        first_x = math.floor(min_x / step_m) * step_m
        first_y = math.floor(min_y / step_m) * step_m

        x_m = first_x
        while x_m <= max_x + step_m:
            screen_x, _ = self.world_to_screen(x_m, 0.0)
            color = "#aeb8c4" if abs(x_m) < step_m * 0.1 else "#dfe4ea"
            line_width = 2 if abs(x_m) < step_m * 0.1 else 1
            self.create_line(screen_x, 0, screen_x, height, fill=color, width=line_width)
            self.create_text(
                screen_x + 4,
                8,
                anchor="nw",
                fill="#7a8794",
                font=("Consolas", 8),
                text=f"{x_m:g}",
            )
            x_m += step_m

        y_m = first_y
        while y_m <= max_y + step_m:
            _, screen_y = self.world_to_screen(0.0, y_m)
            color = "#aeb8c4" if abs(y_m) < step_m * 0.1 else "#dfe4ea"
            line_width = 2 if abs(y_m) < step_m * 0.1 else 1
            self.create_line(0, screen_y, width, screen_y, fill=color, width=line_width)
            self.create_text(
                6,
                screen_y - 3,
                anchor="sw",
                fill="#7a8794",
                font=("Consolas", 8),
                text=f"{y_m:g}",
            )
            y_m += step_m

        origin_x, origin_y = self.world_to_screen(0.0, 0.0)
        self.create_text(origin_x + 20, origin_y - 8, text="X", fill="#334155", font=("Arial", 9, "bold"))
        self.create_text(origin_x + 8, origin_y - 20, text="Y", fill="#334155", font=("Arial", 9, "bold"))

    def _draw_target_line(self) -> None:
        if self.goal is None:
            return
        start_x, start_y = self.world_to_screen(self.pose.x_m, self.pose.y_m)
        goal_x, goal_y = self.world_to_screen(self.goal.x_m, self.goal.y_m)
        self.create_line(
            start_x,
            start_y,
            goal_x,
            goal_y,
            fill="#15803d",
            width=2,
            dash=(7, 5),
            arrow=tk.LAST,
        )

    def _draw_path(self) -> None:
        if len(self.path) < 2:
            return
        points: list[float] = []
        for x_m, y_m in self.path:
            screen_x, screen_y = self.world_to_screen(x_m, y_m)
            points.extend((screen_x, screen_y))
        self.create_line(*points, fill="#2563eb", width=2, smooth=False)

    def _draw_goal(self) -> None:
        if self.goal is None:
            return
        screen_x, screen_y = self.world_to_screen(self.goal.x_m, self.goal.y_m)
        radius = 9
        self.create_oval(
            screen_x - radius,
            screen_y - radius,
            screen_x + radius,
            screen_y + radius,
            outline="#15803d",
            width=3,
            fill="#dcfce7",
        )
        heading_rad = math.radians(self.goal.yaw_deg)
        end_x = screen_x + math.cos(heading_rad) * 28
        end_y = screen_y - math.sin(heading_rad) * 28
        self.create_line(screen_x, screen_y, end_x, end_y, fill="#15803d", width=3, arrow=tk.LAST)

    def _draw_vehicle(self) -> None:
        center_x, center_y = self.world_to_screen(self.pose.x_m, self.pose.y_m)
        yaw_rad = math.radians(self.pose.yaw_deg)
        half_length = 25.0
        half_width = 17.0
        local_points = [
            (half_length, 0.0),
            (half_length * 0.35, half_width),
            (-half_length, half_width),
            (-half_length, -half_width),
            (half_length * 0.35, -half_width),
        ]
        points: list[float] = []
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        for local_x, local_y in local_points:
            world_screen_x = center_x + local_x * cos_yaw - local_y * sin_yaw
            world_screen_y = center_y - (local_x * sin_yaw + local_y * cos_yaw)
            points.extend((world_screen_x, world_screen_y))
        fill = "#fee2e2" if self.pose.fault else "#dbeafe"
        outline = "#b91c1c" if self.pose.fault else "#1d4ed8"
        self.create_polygon(*points, fill=fill, outline=outline, width=3)
        self.create_oval(center_x - 3, center_y - 3, center_x + 3, center_y + 3, fill=outline, outline="")

    def _on_left_click(self, event: tk.Event) -> None:
        x_m, y_m = self.screen_to_world(event.x, event.y)
        self.goal_callback(x_m, y_m)

    def _on_pan_start(self, event: tk.Event) -> None:
        self.pan_start = (event.x, event.y)
        self.pan_world_start = (self.view_center_x, self.view_center_y)

    def _on_pan_move(self, event: tk.Event) -> None:
        if self.pan_start is None or self.pan_world_start is None:
            return
        delta_x = event.x - self.pan_start[0]
        delta_y = event.y - self.pan_start[1]
        self.view_center_x = self.pan_world_start[0] - delta_x / self.scale_px_per_m
        self.view_center_y = self.pan_world_start[1] + delta_y / self.scale_px_per_m
        self.redraw()

    def _on_mouse_wheel(self, event: tk.Event) -> None:
        factor = 1.12 if event.delta > 0 else 1.0 / 1.12
        self.scale_px_per_m = clamp(self.scale_px_per_m * factor, 25.0, 320.0)
        self.redraw()


class AgvDebugApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("AGV底盘调试工具")
        self.root.geometry("1280x780")
        self.root.minsize(1040, 680)
        self.root.configure(background="#eef2f5")

        self.receive_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.serial_worker = SerialWorker(self.receive_queue)
        self.pose = PoseTelemetry()
        self.goal: MotionGoal | None = None
        self.sequence = 0
        self.log_rows: list[list[object]] = []
        self.last_simulation_time = time.monotonic()

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUDRATE))
        self.simulation_var = tk.BooleanVar(value=False)
        self.connection_text_var = tk.StringVar(value="未连接")
        self.target_x_var = tk.StringVar(value="0.500")
        self.target_y_var = tk.StringVar(value="0.000")
        self.target_yaw_var = tk.StringVar(value="0.00")
        self.target_speed_var = tk.StringVar(value="0.05")
        self.telemetry_vars = {
            "position": tk.StringVar(value="0.000, 0.000 m"),
            "yaw": tk.StringVar(value="0.00 deg"),
            "velocity": tk.StringVar(value="0.000 m/s"),
            "angular": tk.StringVar(value="0.00 deg/s"),
            "wheel": tk.StringVar(value="0.00 / 0.00 RPM"),
            "wheel_target": tk.StringVar(value="0.00 / 0.00 RPM"),
            "pwm": tk.StringVar(value="0 / 0"),
            "encoder_invalid": tk.StringVar(value="0 / 0"),
            "state": tk.StringVar(value="IDLE"),
            "fault": tk.StringVar(value="0x0000"),
            "sequence": tk.StringVar(value="-")
        }

        self._configure_style()
        self._build_ui()
        self.refresh_ports()
        self._set_connection_status("未连接", "#b45309")

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(SERIAL_POLL_MS, self._poll_receive_queue)
        self.root.after(SIMULATION_PERIOD_MS, self._simulation_step)

    def _configure_style(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#eef2f5")
        style.configure("Panel.TFrame", background="#ffffff")
        style.configure("TLabel", background="#eef2f5", foreground="#17212b", font=("Microsoft YaHei UI", 9))
        style.configure("Panel.TLabel", background="#ffffff", foreground="#17212b", font=("Microsoft YaHei UI", 9))
        style.configure("Title.TLabel", background="#eef2f5", foreground="#111827", font=("Microsoft YaHei UI", 15, "bold"))
        style.configure("Section.TLabel", background="#ffffff", foreground="#111827", font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("Value.TLabel", background="#ffffff", foreground="#0f172a", font=("Consolas", 11, "bold"))
        style.configure("TButton", font=("Microsoft YaHei UI", 9), padding=(10, 6))
        style.configure("Primary.TButton", background="#166534", foreground="#ffffff")
        style.map("Primary.TButton", background=[("active", "#15803d")])
        style.configure("Danger.TButton", background="#b91c1c", foreground="#ffffff")
        style.map("Danger.TButton", background=[("active", "#991b1b")])
        style.configure("TCheckbutton", background="#eef2f5", font=("Microsoft YaHei UI", 9))

    def _build_ui(self) -> None:
        top_bar = ttk.Frame(self.root, padding=(14, 10))
        top_bar.pack(fill=tk.X)
        ttk.Label(top_bar, text="AGV底盘调试工具", style="Title.TLabel").pack(side=tk.LEFT)

        connection_group = ttk.Frame(top_bar)
        connection_group.pack(side=tk.RIGHT)
        self.status_canvas = tk.Canvas(connection_group, width=14, height=14, background="#eef2f5", highlightthickness=0)
        self.status_canvas.pack(side=tk.LEFT, padx=(0, 4))
        ttk.Label(connection_group, textvariable=self.connection_text_var).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Checkbutton(
            connection_group,
            text="模拟模式",
            variable=self.simulation_var,
            command=self._on_simulation_toggle,
        ).pack(side=tk.LEFT, padx=(0, 10))
        self.port_combo = ttk.Combobox(connection_group, textvariable=self.port_var, width=10, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=3)
        ttk.Button(connection_group, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT, padx=3)
        ttk.Combobox(
            connection_group,
            textvariable=self.baud_var,
            values=("115200", "230400", "460800", "921600"),
            width=9,
            state="readonly",
        ).pack(side=tk.LEFT, padx=3)
        self.connect_button = ttk.Button(connection_group, text="连接", command=self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=(3, 0))

        content = ttk.Frame(self.root, padding=(14, 0, 14, 14))
        content.pack(fill=tk.BOTH, expand=True)

        left_panel = ttk.Frame(content, style="Panel.TFrame")
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        map_toolbar = ttk.Frame(left_panel, style="Panel.TFrame", padding=(10, 8))
        map_toolbar.pack(fill=tk.X)
        ttk.Label(map_toolbar, text="本地任务坐标系", style="Section.TLabel").pack(side=tk.LEFT)
        ttk.Button(map_toolbar, text="车辆居中", command=lambda: self.map_canvas.center_on_vehicle()).pack(side=tk.RIGHT, padx=3)
        ttk.Button(map_toolbar, text="清除轨迹", command=lambda: self.map_canvas.clear_path()).pack(side=tk.RIGHT, padx=3)
        self.map_canvas = CoordinateCanvas(left_panel, self._on_map_goal_selected)
        self.map_canvas.pack(fill=tk.BOTH, expand=True)

        right_panel = ttk.Frame(content, style="Panel.TFrame", width=340, padding=14)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(12, 0))
        right_panel.pack_propagate(False)
        self._build_telemetry_panel(right_panel)
        self._build_goal_panel(right_panel)
        self._build_log_panel(right_panel)

    def _build_telemetry_panel(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="实时状态", style="Section.TLabel").pack(anchor=tk.W)
        rows = (
            ("位置 X / Y", "position"),
            ("航向", "yaw"),
            ("线速度", "velocity"),
            ("角速度", "angular"),
            ("左 / 右轮速", "wheel"),
            ("左 / 右目标", "wheel_target"),
            ("左 / 右 PWM", "pwm"),
            ("编码器异常", "encoder_invalid"),
            ("控制状态", "state"),
            ("故障码", "fault"),
            ("任务序号", "sequence"),
        )
        for label_text, key in rows:
            row = ttk.Frame(parent, style="Panel.TFrame")
            row.pack(fill=tk.X, pady=3)
            ttk.Label(row, text=label_text, style="Panel.TLabel").pack(side=tk.LEFT)
            ttk.Label(row, textvariable=self.telemetry_vars[key], style="Value.TLabel").pack(side=tk.RIGHT)
        ttk.Separator(parent).pack(fill=tk.X, pady=12)

    def _build_goal_panel(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="目标任务", style="Section.TLabel").pack(anchor=tk.W)
        fields = (
            ("目标 X (m)", self.target_x_var),
            ("目标 Y (m)", self.target_y_var),
            ("目标航向 (deg)", self.target_yaw_var),
            ("最大速度 (m/s)", self.target_speed_var),
        )
        for label_text, variable in fields:
            row = ttk.Frame(parent, style="Panel.TFrame")
            row.pack(fill=tk.X, pady=3)
            ttk.Label(row, text=label_text, style="Panel.TLabel").pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=variable, width=12, justify=tk.RIGHT).pack(side=tk.RIGHT)

        ttk.Label(
            parent,
            text="在左侧坐标图单击可设置目标位置。",
            style="Panel.TLabel",
            foreground="#64748b",
        ).pack(anchor=tk.W, pady=(6, 8))

        button_row = ttk.Frame(parent, style="Panel.TFrame")
        button_row.pack(fill=tk.X)
        ttk.Button(button_row, text="发送目标", style="Primary.TButton", command=self.send_goal).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        ttk.Button(button_row, text="平滑停止", command=self.send_stop).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0))

        safety_row = ttk.Frame(parent, style="Panel.TFrame")
        safety_row.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(safety_row, text="急停", style="Danger.TButton", command=self.send_emergency_stop).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        ttk.Button(safety_row, text="里程清零", command=self.reset_odometry).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0))
        ttk.Separator(parent).pack(fill=tk.X, pady=12)

    def _build_log_panel(self, parent: ttk.Frame) -> None:
        header = ttk.Frame(parent, style="Panel.TFrame")
        header.pack(fill=tk.X)
        ttk.Label(header, text="通信日志", style="Section.TLabel").pack(side=tk.LEFT)
        ttk.Button(header, text="保存", command=self.save_log).pack(side=tk.RIGHT)
        self.log_text = tk.Text(
            parent,
            height=11,
            wrap=tk.NONE,
            font=("Consolas", 8),
            background="#f8fafc",
            foreground="#334155",
            relief=tk.FLAT,
            borderwidth=0,
        )
        self.log_text.pack(fill=tk.BOTH, expand=True, pady=(7, 0))
        self.log_text.configure(state=tk.DISABLED)

    def refresh_ports(self) -> None:
        if list_ports is None:
            ports = []
        else:
            ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            preferred = "COM20" if "COM20" in ports else ports[0]
            self.port_var.set(preferred)

    def toggle_connection(self) -> None:
        if self.serial_worker.connected:
            self.serial_worker.disconnect()
            self.connect_button.configure(text="连接")
            self._set_connection_status("未连接", "#b45309")
            return
        if self.simulation_var.get():
            messagebox.showinfo("提示", "请先关闭模拟模式，再连接真实串口。")
            return
        port_name = self.port_var.get().strip()
        if not port_name:
            messagebox.showerror("连接失败", "没有选择串口。")
            return
        try:
            baudrate = int(self.baud_var.get())
            self.serial_worker.connect(port_name, baudrate)
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))
            return
        self.connect_button.configure(text="断开")
        self._set_connection_status(f"{port_name} 已连接", "#166534")
        self._append_log("SYS", f"连接 {port_name} @ {baudrate}")

    def _on_simulation_toggle(self) -> None:
        if self.simulation_var.get():
            self.serial_worker.disconnect()
            self.connect_button.configure(text="连接")
            self._set_connection_status("模拟模式", "#166534")
            self._append_log("SYS", "进入模拟模式")
        else:
            self._set_connection_status("未连接", "#b45309")
            self._append_log("SYS", "退出模拟模式")

    def _set_connection_status(self, text: str, color: str) -> None:
        self.connection_text_var.set(text)
        self.status_canvas.delete("all")
        self.status_canvas.create_oval(3, 3, 11, 11, fill=color, outline=color)

    def _on_map_goal_selected(self, x_m: float, y_m: float) -> None:
        self.target_x_var.set(f"{x_m:.3f}")
        self.target_y_var.set(f"{y_m:.3f}")
        try:
            yaw_deg = float(self.target_yaw_var.get())
            speed_m_s = float(self.target_speed_var.get())
        except ValueError:
            yaw_deg = self.pose.yaw_deg
            speed_m_s = 0.10
        preview = MotionGoal(x_m, y_m, yaw_deg, speed_m_s, self.sequence + 1)
        self.map_canvas.set_goal(preview)

    def _next_sequence(self) -> int:
        self.sequence = (self.sequence + 1) & 0xFFFF
        if self.sequence == 0:
            self.sequence = 1
        return self.sequence

    def send_goal(self) -> None:
        try:
            x_m = float(self.target_x_var.get())
            y_m = float(self.target_y_var.get())
            yaw_deg = float(self.target_yaw_var.get())
            speed_m_s = float(self.target_speed_var.get())
        except ValueError:
            messagebox.showerror("参数错误", "目标位置、航向和速度必须是数字。")
            return
        if speed_m_s <= 0.0 or speed_m_s > 1.0:
            messagebox.showerror("参数错误", "调试速度必须在0~1m/s范围内。")
            return

        if self.simulation_var.get():
            current_yaw_rad = math.radians(self.pose.yaw_deg)
            delta_x = x_m - self.pose.x_m
            delta_y = y_m - self.pose.y_m
            relative_x_m = math.cos(current_yaw_rad) * delta_x + math.sin(current_yaw_rad) * delta_y
            relative_y_m = -math.sin(current_yaw_rad) * delta_x + math.cos(current_yaw_rad) * delta_y
            relative_yaw_deg = wrap_angle_deg(yaw_deg - self.pose.yaw_deg)
        else:
            # 当前实车固件在接收命令时锁定航向，不能把抖动的实时yaw重复换算进任务。
            relative_x_m = x_m
            relative_y_m = 0.0
            relative_yaw_deg = 0.0

        x_mm = round(relative_x_m * 1000)
        y_mm = round(relative_y_m * 1000)
        yaw_cdeg = round(relative_yaw_deg * 100)
        speed_mm_s = round(speed_m_s * 1000)
        if not self.simulation_var.get() and (
            x_mm < FORWARD_TEST_MIN_DISTANCE_MM
            or x_mm > FORWARD_TEST_MAX_DISTANCE_MM
            or y_mm != 0
            or yaw_cdeg != 0
            or speed_mm_s < FORWARD_TEST_MIN_SPEED_MM_S
            or speed_mm_s > FORWARD_TEST_MAX_SPEED_MM_S
        ):
            detail = (
                f"本次任务换算结果：X={x_mm}mm，Y={y_mm}mm，"
                f"Yaw={yaw_cdeg / 100:.2f}°，速度={speed_mm_s}mm/s。\n\n"
                "当前固件只接受：X=300~3000mm、Y=0、Yaw=0、"
                "速度=50~100mm/s。"
            )
            self._append_log("ERR", "任务未发送，超出当前直线测试范围")
            messagebox.showwarning("任务未发送", detail)
            return

        sequence = self._next_sequence()
        new_goal = MotionGoal(relative_x_m, relative_y_m,
                              relative_yaw_deg, speed_m_s, sequence)
        self.telemetry_vars["sequence"].set(str(sequence))
        command = (
            f"CMD,GOTO,{PROTOCOL_VERSION},{sequence},"
            f"{x_mm},{y_mm},{yaw_cdeg},{speed_mm_s}"
        )
        if self.simulation_var.get():
            self._reset_mission_frame(new_goal)
            self.pose.state = "RUNNING"
            self._append_log("TX", command)
            self._append_log("ACK", f"{sequence},GOTO,ACCEPTED,0")
        else:
            try:
                self.serial_worker.send_line(command)
                self._reset_mission_frame(new_goal)
                self._append_log("SYS", "前向任务使用发送时刻车头方向，指令Y和Yaw固定为0")
                self._append_log("TX", command)
            except Exception as exc:
                messagebox.showerror("发送失败", str(exc))

    def _reset_mission_frame(self, goal: MotionGoal) -> None:
        """每次发送目标后，以当前车辆位置和朝向建立新的任务坐标系。"""
        timestamp_ms = self.pose.timestamp_ms
        self.pose = PoseTelemetry(timestamp_ms=timestamp_ms, state="WAIT_ACK")
        self.goal = goal
        self.target_x_var.set(f"{goal.x_m:.3f}")
        self.target_y_var.set(f"{goal.y_m:.3f}")
        self.target_yaw_var.set(f"{goal.yaw_deg:.2f}")
        self.map_canvas.path.clear()
        self.map_canvas.view_center_x = 0.0
        self.map_canvas.view_center_y = 0.0
        self.map_canvas.set_pose(self.pose, append_path=False)
        self.map_canvas.set_goal(goal)
        self._update_telemetry_display()
        self._append_log("SYS", "新任务坐标系已归零，车辆位姿=(0,0,0)")

    def send_stop(self) -> None:
        sequence = self._next_sequence()
        command = f"CMD,STOP,{PROTOCOL_VERSION},{sequence}"
        if self.simulation_var.get():
            self.goal = None
            self.pose.linear_m_s = 0.0
            self.pose.angular_deg_s = 0.0
            self.pose.state = "HOLD"
            self.map_canvas.set_goal(None)
            self._update_telemetry_display()
            self._append_log("TX", command)
        else:
            self._send_real_command(command)

    def send_emergency_stop(self) -> None:
        sequence = self._next_sequence()
        command = f"CMD,ESTOP,{PROTOCOL_VERSION},{sequence}"
        if self.simulation_var.get():
            self.goal = None
            self.pose.linear_m_s = 0.0
            self.pose.angular_deg_s = 0.0
            self.pose.state = "ESTOP"
            self.pose.fault |= 0x0001
            self.map_canvas.set_goal(None)
            self.map_canvas.set_pose(self.pose, append_path=False)
            self._update_telemetry_display()
            self._append_log("TX", command)
        else:
            self._send_real_command(command)

    def reset_odometry(self) -> None:
        sequence = self._next_sequence()
        command = f"CMD,RESET_ODOM,{PROTOCOL_VERSION},{sequence}"
        if self.simulation_var.get():
            self.pose = PoseTelemetry()
            self.goal = None
            self.map_canvas.path.clear()
            self.map_canvas.set_goal(None)
            self.map_canvas.set_pose(self.pose, append_path=False)
            self._update_telemetry_display()
            self._append_log("TX", command)
        else:
            self._send_real_command(command)

    def _send_real_command(self, command: str) -> None:
        try:
            self.serial_worker.send_line(command)
            self._append_log("TX", command)
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))

    def _poll_receive_queue(self) -> None:
        while True:
            try:
                message_type, payload = self.receive_queue.get_nowait()
            except queue.Empty:
                break
            if message_type == "line":
                self._handle_serial_line(str(payload))
            elif message_type == "error":
                self._append_log("ERR", str(payload))
                self.serial_worker.disconnect()
                self.connect_button.configure(text="连接")
                self._set_connection_status("串口异常", "#b91c1c")
        self.root.after(SERIAL_POLL_MS, self._poll_receive_queue)

    def _handle_serial_line(self, line: str) -> None:
        self._append_log("RX", line)
        parts = [part.strip() for part in line.split(",")]
        if not parts:
            return
        if parts[0] == "POSE":
            self._parse_pose(parts)
        elif parts[0] == "ACK":
            self._parse_ack(parts)
        elif parts[0].lstrip("-").isdigit():
            self._parse_legacy_telemetry(parts)

    def _parse_pose(self, parts: list[str]) -> None:
        if len(parts) < 11:
            self._append_log("ERR", "POSE字段数量不足")
            return
        try:
            self.pose = PoseTelemetry(
                timestamp_ms=int(parts[1]),
                x_m=int(parts[2]) / 1000.0,
                y_m=int(parts[3]) / 1000.0,
                yaw_deg=int(parts[4]) / 100.0,
                linear_m_s=int(parts[5]) / 1000.0,
                angular_deg_s=int(parts[6]) / 100.0,
                left_rpm=int(parts[7]) / 100.0,
                right_rpm=int(parts[8]) / 100.0,
                state=parts[9],
                fault=int(parts[10], 0),
                left_target_rpm=int(parts[11]) / 100.0 if len(parts) >= 19 else 0.0,
                right_target_rpm=int(parts[12]) / 100.0 if len(parts) >= 19 else 0.0,
                left_pwm=int(parts[13]) if len(parts) >= 19 else 0,
                right_pwm=int(parts[14]) if len(parts) >= 19 else 0,
                left_count=int(parts[15]) if len(parts) >= 19 else 0,
                right_count=int(parts[16]) if len(parts) >= 19 else 0,
                left_invalid=int(parts[17]) if len(parts) >= 19 else 0,
                right_invalid=int(parts[18]) if len(parts) >= 19 else 0,
            )
        except (ValueError, IndexError) as exc:
            self._append_log("ERR", f"POSE解析失败: {exc}")
            return
        self._record_pose()

    def _parse_ack(self, parts: list[str]) -> None:
        if len(parts) >= 5:
            self.telemetry_vars["sequence"].set(parts[1])
            if parts[3] == "RECEIVED":
                self.pose.state = "RX_TEST"
                self._update_telemetry_display()
            elif parts[3] == "ACCEPTED":
                accepted_states = {
                    "GOTO": "RUNNING",
                    "STOP": "HOLD",
                    "ESTOP": "ESTOP",
                    "RESET_ODOM": "IDLE",
                }
                self.pose.state = accepted_states.get(parts[2], "ACCEPTED")
                self._update_telemetry_display()
            elif parts[3] in ("COMPLETED", "CANCELED", "FAULT"):
                self.pose.state = parts[3]
                if parts[3] == "FAULT":
                    try:
                        self.pose.fault = int(parts[4], 0)
                    except ValueError:
                        pass
                self.goal = None
                self.map_canvas.set_goal(None)
                self._update_telemetry_display()
            elif parts[3] == "REJECTED":
                self.pose.state = "REJECTED"
                self.goal = None
                self.map_canvas.set_goal(None)
                self._update_telemetry_display()

    def _parse_legacy_telemetry(self, parts: list[str]) -> None:
        """兼容当前测试固件，只更新航向、轮速和状态，不伪造x、y。"""
        try:
            if len(parts) >= 14:
                self.pose.state = f"LEGACY_{int(parts[0])}"
                self.pose.yaw_deg = float(parts[2])
                self.pose.angular_deg_s = float(parts[4])
                if len(parts) == 14 and "." in parts[5]:
                    self.pose.left_rpm = float(parts[9])
                    self.pose.right_rpm = float(parts[12])
                self._update_telemetry_display()
                self.map_canvas.set_pose(self.pose, append_path=False)
        except ValueError:
            return

    def _simulation_step(self) -> None:
        now = time.monotonic()
        dt_s = clamp(now - self.last_simulation_time, 0.001, 0.2)
        self.last_simulation_time = now
        if self.simulation_var.get() and self.goal is not None and self.pose.fault == 0:
            dx = self.goal.x_m - self.pose.x_m
            dy = self.goal.y_m - self.pose.y_m
            distance = math.hypot(dx, dy)
            if distance <= 0.025:
                final_error = wrap_angle_deg(self.goal.yaw_deg - self.pose.yaw_deg)
                if abs(final_error) <= 1.0:
                    self.pose.x_m = self.goal.x_m
                    self.pose.y_m = self.goal.y_m
                    self.pose.yaw_deg = self.goal.yaw_deg
                    self.pose.linear_m_s = 0.0
                    self.pose.angular_deg_s = 0.0
                    self.pose.left_rpm = 0.0
                    self.pose.right_rpm = 0.0
                    self.pose.state = "COMPLETED"
                    self._append_log("ACK", f"{self.goal.sequence},GOTO,COMPLETED,0")
                    self.goal = None
                    self.map_canvas.set_goal(None)
                else:
                    angular_rate = clamp(final_error * 2.0, -45.0, 45.0)
                    self._integrate_simulation(0.0, angular_rate, dt_s)
            else:
                desired_yaw = math.degrees(math.atan2(dy, dx))
                heading_error = wrap_angle_deg(desired_yaw - self.pose.yaw_deg)
                angular_rate = clamp(heading_error * 2.2, -55.0, 55.0)
                heading_scale = max(0.0, math.cos(math.radians(clamp(heading_error, -90.0, 90.0))))
                linear_speed = min(self.goal.speed_m_s, distance * 0.8) * heading_scale
                self._integrate_simulation(linear_speed, angular_rate, dt_s)
            self.pose.timestamp_ms = int(time.monotonic() * 1000)
            self._record_pose()
        self.root.after(SIMULATION_PERIOD_MS, self._simulation_step)

    def _integrate_simulation(self, linear_m_s: float, angular_deg_s: float, dt_s: float) -> None:
        yaw_mid_rad = math.radians(self.pose.yaw_deg + angular_deg_s * dt_s * 0.5)
        self.pose.x_m += linear_m_s * math.cos(yaw_mid_rad) * dt_s
        self.pose.y_m += linear_m_s * math.sin(yaw_mid_rad) * dt_s
        self.pose.yaw_deg = wrap_angle_deg(self.pose.yaw_deg + angular_deg_s * dt_s)
        self.pose.linear_m_s = linear_m_s
        self.pose.angular_deg_s = angular_deg_s
        wheel_circumference_m = math.pi * 0.085
        omega_rad_s = math.radians(angular_deg_s)
        left_m_s = linear_m_s - omega_rad_s * 0.25
        right_m_s = linear_m_s + omega_rad_s * 0.25
        self.pose.left_rpm = left_m_s * 60.0 / wheel_circumference_m
        self.pose.right_rpm = right_m_s * 60.0 / wheel_circumference_m
        self.pose.state = "RUNNING"

    def _record_pose(self) -> None:
        self._update_telemetry_display()
        self.map_canvas.set_pose(self.pose)
        self.log_rows.append(
            [
                self.pose.timestamp_ms,
                self.pose.x_m,
                self.pose.y_m,
                self.pose.yaw_deg,
                self.pose.linear_m_s,
                self.pose.angular_deg_s,
                self.pose.left_rpm,
                self.pose.right_rpm,
                self.pose.state,
                self.pose.fault,
                self.pose.left_target_rpm,
                self.pose.right_target_rpm,
                self.pose.left_pwm,
                self.pose.right_pwm,
                self.pose.left_count,
                self.pose.right_count,
                self.pose.left_invalid,
                self.pose.right_invalid,
            ]
        )
        if len(self.log_rows) > 100000:
            self.log_rows = self.log_rows[-50000:]

    def _update_telemetry_display(self) -> None:
        self.telemetry_vars["position"].set(f"{self.pose.x_m:.3f}, {self.pose.y_m:.3f} m")
        self.telemetry_vars["yaw"].set(f"{self.pose.yaw_deg:.2f} deg")
        self.telemetry_vars["velocity"].set(f"{self.pose.linear_m_s:.3f} m/s")
        self.telemetry_vars["angular"].set(f"{self.pose.angular_deg_s:.2f} deg/s")
        self.telemetry_vars["wheel"].set(f"{self.pose.left_rpm:.2f} / {self.pose.right_rpm:.2f} RPM")
        self.telemetry_vars["wheel_target"].set(
            f"{self.pose.left_target_rpm:.2f} / {self.pose.right_target_rpm:.2f} RPM"
        )
        self.telemetry_vars["pwm"].set(f"{self.pose.left_pwm} / {self.pose.right_pwm}")
        self.telemetry_vars["encoder_invalid"].set(
            f"{self.pose.left_invalid} / {self.pose.right_invalid}"
        )
        self.telemetry_vars["state"].set(self.pose.state)
        self.telemetry_vars["fault"].set(f"0x{self.pose.fault:04X}")

    def _append_log(self, direction: str, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"{timestamp} {direction:<4} {message}\n")
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > 500:
            self.log_text.delete("1.0", "101.0")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def save_log(self) -> None:
        if not self.log_rows:
            messagebox.showinfo("保存日志", "当前没有位姿数据。")
            return
        default_name = time.strftime("agv_pose_%Y%m%d_%H%M%S.csv")
        file_path = filedialog.asksaveasfilename(
            title="保存位姿日志",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=(("CSV文件", "*.csv"), ("所有文件", "*.*")),
        )
        if not file_path:
            return
        with Path(file_path).open("w", newline="", encoding="utf-8-sig") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "timestamp_ms",
                    "x_m",
                    "y_m",
                    "yaw_deg",
                    "linear_m_s",
                    "angular_deg_s",
                    "left_rpm",
                    "right_rpm",
                    "state",
                    "fault",
                    "left_target_rpm",
                    "right_target_rpm",
                    "left_pwm",
                    "right_pwm",
                    "left_count",
                    "right_count",
                    "left_invalid",
                    "right_invalid",
                ]
            )
            writer.writerows(self.log_rows)
        self._append_log("SYS", f"日志已保存: {file_path}")

    def on_close(self) -> None:
        self.serial_worker.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    AgvDebugApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
