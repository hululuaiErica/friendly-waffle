#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""UART 串口调试工具（tkinter + pyserial）。

功能:
    * 串口参数配置：端口 / 波特率 / 数据位 / 校验 / 停止位 / 流控 / 编码
    * 文本与 HEX 两种方式收发，支持追加 CRLF、定时发送、回显、时间戳
    * 三栏界面：左侧串口设置，中间收发，右侧常驻「发送记录 / 快捷指令」（分隔条位置可记忆）
    * 支持切换配置文件，每个配置文件各自拥有一套快捷指令与发送记录
    * 记录每一条发出的指令：
        - 界面「发送记录」表格（最多 500 条，持久化到 config.json，重启不丢失）
        - 自动写入 logs/uart_YYYYMMDD.log（时间 + HEX + 文本）
        - 支持重新发送 / 载入发送区 / 复制 HEX / 导出 CSV
    * 快捷指令管理，双击即发送

运行方式:
    python uart_tool.py

依赖:
    pip install pyserial
"""

from __future__ import annotations

import codecs
import csv
import json
import os
import queue
import re
import subprocess
import sys
import threading
import uuid
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, font as tkfont, messagebox, ttk

import serial
from serial.tools import list_ports

APP_TITLE = "UART 串口调试工具"


def _app_dir() -> Path:
    """配置与日志所在目录。

    源码运行时紧跟脚本；PyInstaller 打包后指向 exe 所在目录，
    避免 onefile 模式下写进临时解包目录（退出即被删除）。
    """
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent


APP_DIR = _app_dir()
CONFIG_PATH = APP_DIR / "config.json"
LOG_DIR = APP_DIR / "logs"

HISTORY_LIMIT = 500          # 界面中保留的发送记录条数
RX_LINE_LIMIT = 5000         # 接收区最多显示的行数
RX_LINE_TRIM = 2000          # 超出上限时一次性删除的行数
POLL_INTERVAL_MS = 40        # 串口事件轮询间隔
CONFIG_SAVE_DELAY_MS = 800   # 配置延迟保存时间

BAUD_RATES = (
    "1200", "2400", "4800", "9600", "14400", "19200", "38400", "57600",
    "115200", "128000", "230400", "256000", "460800", "921600",
    "1000000", "2000000",
)
BYTESIZE_MAP = {
    "5": serial.FIVEBITS,
    "6": serial.SIXBITS,
    "7": serial.SEVENBITS,
    "8": serial.EIGHTBITS,
}
PARITY_MAP = {
    "无 (None)": serial.PARITY_NONE,
    "偶校验 (Even)": serial.PARITY_EVEN,
    "奇校验 (Odd)": serial.PARITY_ODD,
    "标记 (Mark)": serial.PARITY_MARK,
    "空格 (Space)": serial.PARITY_SPACE,
}
PARITY_LETTER = {
    "无 (None)": "N",
    "偶校验 (Even)": "E",
    "奇校验 (Odd)": "O",
    "标记 (Mark)": "M",
    "空格 (Space)": "S",
}
STOPBITS_MAP = {
    "1": serial.STOPBITS_ONE,
    "1.5": serial.STOPBITS_ONE_POINT_FIVE,
    "2": serial.STOPBITS_TWO,
}
FLOW_MAP = {
    "无": (False, False),
    "RTS/CTS": (True, False),
    "XON/XOFF": (False, True),
}
ENCODINGS = ("UTF-8", "GBK", "Latin-1")
DATA_FONT = ("Consolas", 10)
RX_BG = "#101418"
RX_FG = "#E6E6E6"
TX_FG = "#63B3ED"
SYS_FG = "#F6AD55"


# --------------------------------------------------------------------------- #
# 通用工具函数
# --------------------------------------------------------------------------- #
def now_stamp() -> str:
    """完整时间戳，精确到毫秒，例如 2026-10-23 12:00:00.123。"""
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def now_clock() -> str:
    """简短时间戳，精确到毫秒，例如 12:00:00.123。"""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def to_hex(data: bytes) -> str:
    """字节序列 -> 空格分隔的大写 HEX 字符串。"""
    return " ".join(f"{byte:02X}" for byte in data)


_HEX_NOISE = re.compile(r"0[xX]|[\s,;:_\-]")


def parse_hex(text: str) -> bytes:
    """把用户输入的 HEX 文本转换为字节序列。

    支持 "01 03 00 0A"、"0103000A"、"0x01,0x03" 等写法。
    """
    cleaned = _HEX_NOISE.sub("", text)
    if not cleaned:
        return b""
    if len(cleaned) % 2 != 0:
        raise ValueError(f"HEX 字符个数必须为偶数（当前 {len(cleaned)} 个）")
    try:
        return bytes.fromhex(cleaned)
    except ValueError as exc:
        raise ValueError(f"包含非法的 HEX 字符：{exc}") from exc


_CONTROL_CHARS = re.compile(r"[\x00-\x08\x0b\x0c\x0e-\x1f\x7f]")


def preview_text(data: bytes, encoding: str = "UTF-8") -> str:
    """把字节序列转成单行可读文本，用于记录与日志。"""
    try:
        text = data.decode(encoding, errors="replace")
    except LookupError:
        text = data.decode("latin-1", errors="replace")
    text = (
        text.replace("\\", "\\\\")
        .replace("\r", "\\r")
        .replace("\n", "\\n")
        .replace("\t", "\\t")
    )
    # 其余控制字符统一转义，保证日志每条只占一行
    return _CONTROL_CHARS.sub(lambda match: f"\\x{ord(match.group()):02x}", text)



def load_json(path: Path, default):
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return default


def save_json(path: Path, payload) -> bool:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
        return True
    except OSError as exc:
        print(f"[uart_tool] 保存配置失败: {exc}")
        return False


# --------------------------------------------------------------------------- #
# 串口通信层（后台线程读取，通过队列把事件交给界面线程）
# --------------------------------------------------------------------------- #
class SerialLink:
    """对 pyserial 的简单封装。

    读取工作放在后台线程中，收到的数据以事件形式放入 ``events`` 队列：
        ("rx", bytes, decoded_text)
        ("error", message)
    界面线程定时轮询队列，避免跨线程操作 tkinter 控件。
    """

    def __init__(self) -> None:
        self.events: "queue.Queue[tuple]" = queue.Queue()
        self._ser: serial.Serial | None = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._decoder = None
        self.encoding = "UTF-8"

    # -- 状态 ------------------------------------------------------------- #
    @property
    def is_open(self) -> bool:
        ser = self._ser
        return ser is not None and ser.is_open

    @property
    def port(self) -> str:
        ser = self._ser
        return ser.port if ser is not None else ""

    # -- 打开 / 关闭 ------------------------------------------------------- #
    def open(
        self,
        port: str,
        baudrate: int,
        bytesize: int,
        parity: str,
        stopbits: float,
        rtscts: bool = False,
        xonxoff: bool = False,
        encoding: str = "UTF-8",
    ) -> serial.Serial:
        self.close()
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=0.05,
            write_timeout=2.0,
            rtscts=rtscts,
            xonxoff=xonxoff,
        )
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except (serial.SerialException, OSError):
            pass

        self.encoding = encoding
        self._decoder = codecs.getincrementaldecoder(encoding)(errors="replace")
        self._ser = ser
        self._stop.clear()
        self._reader = threading.Thread(
            target=self._read_loop, args=(ser,), name="serial-reader", daemon=True
        )
        self._reader.start()
        return ser

    def close(self) -> None:
        self._stop.set()
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass
        thread, self._reader = self._reader, None
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=1.5)
        self._decoder = None

    # -- 读写 ------------------------------------------------------------- #
    def write(self, data: bytes) -> int:
        ser = self._ser
        if ser is None or not ser.is_open:
            raise serial.SerialException("串口未打开")
        if not data:
            return 0
        with self._write_lock:
            written = ser.write(data)
            try:
                ser.flush()
            except (serial.SerialException, OSError):
                pass
        return written

    def set_encoding(self, encoding: str) -> None:
        self.encoding = encoding
        if self.is_open:
            self._decoder = codecs.getincrementaldecoder(encoding)(errors="replace")

    def set_lines(self, dtr: bool, rts: bool) -> None:
        ser = self._ser
        if ser is None or not ser.is_open:
            return
        ser.dtr = bool(dtr)
        ser.rts = bool(rts)

    # -- 后台读取 --------------------------------------------------------- #
    def _read_loop(self, ser: serial.Serial) -> None:
        while not self._stop.is_set():
            try:
                waiting = ser.in_waiting
                data = ser.read(waiting if waiting else 1)
            except (serial.SerialException, OSError) as exc:
                if not self._stop.is_set():
                    self.events.put(("error", f"串口读取异常：{exc}"))
                return
            if not data:
                continue
            try:
                text = self._decoder.decode(data) if self._decoder else ""
            except Exception:  # noqa: BLE001 - 解码失败不应中断接收
                text = ""
            self.events.put(("rx", data, text))


# --------------------------------------------------------------------------- #
# 主界面
# --------------------------------------------------------------------------- #
class UartTool:
    """串口调试工具主界面。"""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("1280x820")
        self.root.minsize(1100, 680)

        cfg = load_json(CONFIG_PATH, {})
        cfg = cfg if isinstance(cfg, dict) else {}
        # config.json 里记录着上次选中的配置文件（active_config），存在就沿用
        self.config_path = CONFIG_PATH
        active = str(cfg.get("active_config") or "").strip()
        if active:
            candidate = Path(active)
            if not candidate.is_absolute():
                candidate = APP_DIR / candidate
            loaded = load_json(candidate, None)
            if isinstance(loaded, dict):
                self.config_path = candidate
                cfg = loaded
        self.cfg = cfg
        self.link = SerialLink()
        self.history: list[dict] = [
            self._normalize_history_entry(item)
            for item in self.cfg.get("history", [])
            if isinstance(item, dict)
        ][:HISTORY_LIMIT]
        self.quick_commands: list[dict] = [
            self._normalize_quick_entry(item)
            for item in self.cfg.get("quick_commands", [])
            if isinstance(item, dict)
        ]

        self.log_handle = None
        self.log_path: Path | None = None
        self.rx_count = 0
        self.tx_count = 0
        self._rx_at_line_start = True
        self._repeat_job = None
        self._recall_index = -1
        self._save_job = None
        self._closing = False
        self._port_map: dict[str, str] = {}
        self._port_details: dict[str, str] = {}
        self._settings_widgets: list[tuple[tk.Widget, str]] = []

        self._init_vars()
        self._build_ui()
        self._apply_saved_state()
        self._refresh_ports()
        self._refresh_history_view()
        self._refresh_quick_view()
        self._update_counters()
        self._set_status("未连接")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(POLL_INTERVAL_MS, self._poll_events)

    # ------------------------------------------------------------------ #
    # 初始化
    # ------------------------------------------------------------------ #
    @staticmethod
    def _normalize_history_entry(entry: dict) -> dict:
        try:
            size = int(entry.get("size", 0) or 0)
        except (TypeError, ValueError):
            size = 0
        return {
            "id": str(entry.get("id") or uuid.uuid4().hex),
            "time": str(entry.get("time", "")),
            "mode": "HEX" if str(entry.get("mode", "")).upper() == "HEX" else "TEXT",
            "size": size,
            "hex": str(entry.get("hex", "")),
            "text": str(entry.get("text", "")),
            "encoding": str(entry.get("encoding", "UTF-8")),
        }

    @staticmethod
    def _normalize_quick_entry(entry: dict) -> dict:
        return {
            "id": str(entry.get("id") or uuid.uuid4().hex),
            "name": str(entry.get("name", "未命名")),
            "mode": "HEX" if str(entry.get("mode", "")).upper() == "HEX" else "TEXT",
            "content": str(entry.get("content", "")),
        }

    def _init_vars(self) -> None:
        cfg = self.cfg
        self.port_var = tk.StringVar()
        self.port_detail_var = tk.StringVar(
            value="未发现串口，请连接设备后点击“刷新”（F5）"
        )
        self.baud_var = tk.StringVar(value=str(cfg.get("baudrate", "115200")))
        self.bytesize_var = tk.StringVar(value=str(cfg.get("bytesize", "8")))
        self.parity_var = tk.StringVar(value=str(cfg.get("parity", "无 (None)")))
        self.stopbits_var = tk.StringVar(value=str(cfg.get("stopbits", "1")))
        self.flow_var = tk.StringVar(value=str(cfg.get("flow", "无")))
        self.encoding_var = tk.StringVar(value=str(cfg.get("encoding", "UTF-8")))

        self.dtr_var = tk.BooleanVar(value=bool(cfg.get("dtr", True)))
        self.rts_var = tk.BooleanVar(value=bool(cfg.get("rts", True)))
        self.hex_send_var = tk.BooleanVar(value=bool(cfg.get("hex_send", False)))
        self.append_crlf_var = tk.BooleanVar(value=bool(cfg.get("append_crlf", False)))
        self.repeat_var = tk.BooleanVar(value=False)
        self.repeat_ms_var = tk.StringVar(value=str(cfg.get("repeat_ms", "1000")))

        self.hex_view_var = tk.BooleanVar(value=bool(cfg.get("hex_view", False)))
        self.show_ts_var = tk.BooleanVar(value=bool(cfg.get("show_timestamp", True)))
        self.echo_tx_var = tk.BooleanVar(value=bool(cfg.get("echo_tx", True)))
        self.autoscroll_var = tk.BooleanVar(value=bool(cfg.get("autoscroll", True)))
        self.log_file_var = tk.BooleanVar(value=bool(cfg.get("log_to_file", True)))

        self.status_var = tk.StringVar(value="未连接")
        self.counter_var = tk.StringVar(value="RX 0 字节 | TX 0 字节")
        self.config_hint_var = tk.StringVar(value="")
        self.log_hint_var = tk.StringVar(
            value=f"发送记录最多保留 {HISTORY_LIMIT} 条，并自动写入 logs 目录"
        )

    def _apply_saved_state(self) -> None:
        saved_port = str(self.cfg.get("port", "") or "")
        if saved_port:
            self.port_var.set(saved_port)
        baud = self.baud_var.get()
        if baud not in BAUD_RATES:
            self.baud_combo.configure(values=(baud,) + BAUD_RATES)
        else:
            self.baud_combo.configure(values=BAUD_RATES)

    # ------------------------------------------------------------------ #
    # 界面搭建
    # ------------------------------------------------------------------ #
    def _build_ui(self) -> None:
        style = ttk.Style(self.root)
        for theme in ("vista", "clam"):
            if theme in style.theme_names():
                style.theme_use(theme)
                break
        style.configure("Hint.TLabel", foreground="#666666")

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        self._build_workspace()
        self._build_status_bar()
        self._update_config_hint()
        self.root.after(120, self._apply_workspace_ratio)

    def _build_status_bar(self) -> None:
        bar = ttk.Frame(self.root, padding=(10, 4))
        bar.grid(row=1, column=0, sticky="ew", padx=10, pady=(2, 8))
        ttk.Label(bar, textvariable=self.status_var).pack(side="left")
        ttk.Label(bar, textvariable=self.counter_var).pack(side="right")

    def _build_connection_bar(self, frame: ttk.LabelFrame) -> None:
        """串口设置：纵向排布在左侧栏，窄宽度下也能完整显示。"""
        frame.columnconfigure(0, weight=1)

        ttk.Label(frame, text="串口").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(
            frame, textvariable=self.port_var, state="readonly", width=24
        )
        self.port_combo.grid(row=1, column=0, sticky="ew", pady=(2, 0))
        self.port_combo.bind("<<ComboboxSelected>>", self._on_port_selected)

        buttons = ttk.Frame(frame)
        buttons.grid(row=2, column=0, sticky="ew", pady=(6, 0))
        self.open_btn = ttk.Button(
            buttons, text="打开串口", command=self.toggle_connection
        )
        self.open_btn.pack(side="left", fill="x", expand=True)
        self.refresh_btn = ttk.Button(
            buttons, text="刷新", width=6, command=self._refresh_ports
        )
        self.refresh_btn.pack(side="left", padx=(6, 0))

        # 下拉框放不下的完整信息（描述 / 厂商 / VID:PID / LOCATION）显示在这里
        self.port_detail_label = ttk.Label(
            frame,
            textvariable=self.port_detail_var,
            style="Hint.TLabel",
            anchor="w",
            justify="left",
            wraplength=240,
        )
        self.port_detail_label.grid(row=3, column=0, sticky="ew", pady=(6, 0))
        self.port_detail_label.bind("<Configure>", self._auto_wrap)

        ttk.Separator(frame, orient="horizontal").grid(
            row=4, column=0, sticky="ew", pady=(10, 8)
        )

        params = ttk.Frame(frame)
        params.grid(row=5, column=0, sticky="ew")
        params.columnconfigure(1, weight=1)
        params.columnconfigure(3, weight=1)

        ttk.Label(params, text="波特率").grid(row=0, column=0, sticky="w", pady=2)
        self.baud_combo = ttk.Combobox(
            params, textvariable=self.baud_var, values=BAUD_RATES, width=10
        )
        self.baud_combo.grid(
            row=0, column=1, columnspan=3, sticky="ew", padx=(6, 0), pady=2
        )

        ttk.Label(params, text="数据位").grid(row=1, column=0, sticky="w", pady=2)
        self.bytesize_combo = ttk.Combobox(
            params,
            textvariable=self.bytesize_var,
            values=tuple(BYTESIZE_MAP),
            state="readonly",
            width=4,
        )
        self.bytesize_combo.grid(row=1, column=1, sticky="w", padx=(6, 10), pady=2)
        ttk.Label(params, text="停止位").grid(row=1, column=2, sticky="w", pady=2)
        self.stopbits_combo = ttk.Combobox(
            params,
            textvariable=self.stopbits_var,
            values=tuple(STOPBITS_MAP),
            state="readonly",
            width=4,
        )
        self.stopbits_combo.grid(row=1, column=3, sticky="ew", padx=(6, 0), pady=2)

        ttk.Label(params, text="校验").grid(row=2, column=0, sticky="w", pady=2)
        self.parity_combo = ttk.Combobox(
            params,
            textvariable=self.parity_var,
            values=tuple(PARITY_MAP),
            state="readonly",
            width=14,
        )
        self.parity_combo.grid(
            row=2, column=1, columnspan=3, sticky="ew", padx=(6, 0), pady=2
        )

        ttk.Label(params, text="流控").grid(row=3, column=0, sticky="w", pady=2)
        self.flow_combo = ttk.Combobox(
            params,
            textvariable=self.flow_var,
            values=tuple(FLOW_MAP),
            state="readonly",
            width=10,
        )
        self.flow_combo.grid(
            row=3, column=1, columnspan=3, sticky="ew", padx=(6, 0), pady=2
        )

        ttk.Label(params, text="编码").grid(row=4, column=0, sticky="w", pady=2)
        self.encoding_combo = ttk.Combobox(
            params,
            textvariable=self.encoding_var,
            values=ENCODINGS,
            state="readonly",
            width=10,
        )
        self.encoding_combo.grid(
            row=4, column=1, columnspan=3, sticky="ew", padx=(6, 0), pady=2
        )
        self.encoding_combo.bind("<<ComboboxSelected>>", self._on_encoding_changed)

        lines = ttk.Frame(frame)
        lines.grid(row=6, column=0, sticky="ew", pady=(8, 0))
        self.dtr_check = ttk.Checkbutton(
            lines, text="DTR", variable=self.dtr_var, command=self._apply_line_states
        )
        self.dtr_check.pack(side="left")
        self.rts_check = ttk.Checkbutton(
            lines, text="RTS", variable=self.rts_var, command=self._apply_line_states
        )
        self.rts_check.pack(side="left", padx=(12, 0))

        ttk.Checkbutton(
            frame,
            text="记录收发到文件",
            variable=self.log_file_var,
            command=self._schedule_config_save,
        ).grid(row=7, column=0, sticky="w", pady=(6, 0))

        file_row = ttk.Frame(frame)
        file_row.grid(row=8, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(file_row, text="打开日志目录", command=self.open_log_dir).pack(
            side="left"
        )
        self.config_btn = ttk.Button(
            file_row, text="选择配置", command=self.select_config
        )
        self.config_btn.pack(side="left", padx=(6, 0))

        self.config_hint_label = ttk.Label(
            frame,
            textvariable=self.config_hint_var,
            style="Hint.TLabel",
            anchor="w",
            justify="left",
            wraplength=240,
        )
        self.config_hint_label.grid(row=9, column=0, sticky="ew", pady=(6, 0))
        self.config_hint_label.bind("<Configure>", self._auto_wrap)

        hint = ttk.Label(
            frame,
            text="提示：文本模式按选定编码发送；HEX 支持 01 03 00 0A / 0x01,0x03 写法",
            style="Hint.TLabel",
            anchor="w",
            justify="left",
            wraplength=240,
        )
        hint.grid(row=10, column=0, sticky="ew", pady=(8, 0))
        hint.bind("<Configure>", self._auto_wrap)

        self._settings_widgets = [
            (self.port_combo, "readonly"),
            (self.refresh_btn, "normal"),
            (self.baud_combo, "normal"),
            (self.bytesize_combo, "readonly"),
            (self.parity_combo, "readonly"),
            (self.stopbits_combo, "readonly"),
            (self.flow_combo, "readonly"),
            (self.dtr_check, "normal"),
            (self.rts_check, "normal"),
        ]

    def _build_workspace(self) -> None:
        """三栏布局：左 = 串口设置，中 = 数据收发，右 = 发送记录 / 快捷指令。"""
        self.workspace = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.workspace.grid(row=0, column=0, sticky="nsew", padx=10, pady=(8, 4))

        # ---------------- 左栏：串口设置 ----------------
        self.settings_frame = ttk.LabelFrame(
            self.workspace, text="串口设置", padding=(10, 6)
        )
        self._build_connection_bar(self.settings_frame)
        self.workspace.add(self.settings_frame, weight=1)

        # ---------------- 中栏：数据收发 ----------------
        self.data_frame = ttk.Frame(self.workspace)
        self.data_frame.columnconfigure(0, weight=1)
        self.data_frame.rowconfigure(0, weight=1)
        self._build_data_tab()
        self.workspace.add(self.data_frame, weight=3)

        # ---------------- 右栏：发送记录 / 快捷指令 ----------------
        self.side_notebook = ttk.Notebook(self.workspace)
        self.tab_history = ttk.Frame(self.side_notebook)
        self.tab_quick = ttk.Frame(self.side_notebook)
        self.side_notebook.add(self.tab_history, text="发送记录")
        self.side_notebook.add(self.tab_quick, text="快捷指令")
        self._build_history_tab()
        self._build_quick_tab()
        self.workspace.add(self.side_notebook, weight=2)

    def _apply_workspace_ratio(self) -> None:
        """恢复两条分隔条的位置；没有记录时按 左 26% / 中 50% / 右 24% 分配。"""
        try:
            total = self.workspace.winfo_width()
            if total <= 600:
                return
            min_settings, min_middle, min_right = 250, 340, 260
            upper = max(min_settings, total - min_middle - min_right)
            pos0 = min(max(int(total * 0.26), min_settings), upper)
            pos1 = min(
                max(pos0 + min_middle, pos0 + int((total - pos0) * 0.66)),
                total - min_right,
            )
            saved = self.cfg.get("sash_pos")
            if isinstance(saved, (list, tuple)) and len(saved) == 2:
                try:
                    left, middle = int(saved[0]), int(saved[1])
                except (TypeError, ValueError):
                    left = middle = 0
                if left > 0:
                    pos0 = min(max(left, min_settings), upper)
                if middle > pos0:
                    pos1 = min(max(middle, pos0 + 200), total - min_right)
            self.workspace.sashpos(0, pos0)
            self.workspace.sashpos(1, pos1)
        except tk.TclError:
            pass

    def _workspace_sash(self) -> list[int]:
        try:
            return [int(self.workspace.sashpos(0)), int(self.workspace.sashpos(1))]
        except (tk.TclError, AttributeError):
            return []

    def _build_data_tab(self) -> None:
        tab = self.data_frame
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)

        paned = ttk.PanedWindow(tab, orient=tk.VERTICAL)
        paned.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)

        # ---------------- 接收区 ----------------
        rx_frame = ttk.LabelFrame(paned, text="接收区", padding=(6, 4))
        rx_frame.columnconfigure(0, weight=1)
        rx_frame.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(rx_frame)
        toolbar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 4))
        ttk.Button(toolbar, text="清空", width=8, command=self.clear_rx).pack(
            side="left"
        )
        ttk.Button(toolbar, text="保存接收内容", command=self.save_rx).pack(
            side="left", padx=(6, 0)
        )
        ttk.Checkbutton(
            toolbar,
            text="HEX 显示",
            variable=self.hex_view_var,
            command=self._schedule_config_save,
        ).pack(side="left", padx=(18, 0))
        ttk.Checkbutton(
            toolbar,
            text="时间戳",
            variable=self.show_ts_var,
            command=self._schedule_config_save,
        ).pack(side="left", padx=(8, 0))
        ttk.Checkbutton(
            toolbar,
            text="显示发送",
            variable=self.echo_tx_var,
            command=self._schedule_config_save,
        ).pack(side="left", padx=(8, 0))
        ttk.Checkbutton(
            toolbar,
            text="自动滚动",
            variable=self.autoscroll_var,
            command=self._schedule_config_save,
        ).pack(side="left", padx=(8, 0))
        ttk.Button(toolbar, text="重置统计", command=self.reset_counters).pack(
            side="right"
        )

        self.rx_text = tk.Text(
            rx_frame,
            height=15,
            wrap="char",
            font=DATA_FONT,
            background=RX_BG,
            foreground=RX_FG,
            insertbackground=RX_FG,
            relief="flat",
            padx=6,
            pady=4,
            state="disabled",
        )
        self.rx_text.grid(row=1, column=0, sticky="nsew")
        rx_scroll = ttk.Scrollbar(
            rx_frame, orient="vertical", command=self.rx_text.yview
        )
        self.rx_text.configure(yscrollcommand=rx_scroll.set)
        rx_scroll.grid(row=1, column=1, sticky="ns")
        self.rx_text.tag_configure("TX", foreground=TX_FG)
        self.rx_text.tag_configure("RX", foreground=RX_FG)
        self.rx_text.tag_configure("SYS", foreground=SYS_FG)

        # ---------------- 发送区 ----------------
        tx_frame = ttk.LabelFrame(paned, text="发送区", padding=(6, 4))
        tx_frame.columnconfigure(0, weight=1)
        tx_frame.rowconfigure(1, weight=1)

        options = ttk.Frame(tx_frame)
        options.grid(row=0, column=0, columnspan=3, sticky="ew")
        ttk.Checkbutton(
            options,
            text="HEX 发送",
            variable=self.hex_send_var,
            command=self._schedule_config_save,
        ).pack(side="left")
        ttk.Checkbutton(
            options,
            text="追加 \\r\\n",
            variable=self.append_crlf_var,
            command=self._schedule_config_save,
        ).pack(side="left", padx=(12, 0))
        self.repeat_check = ttk.Checkbutton(
            options,
            text="定时发送",
            variable=self.repeat_var,
            command=self._toggle_repeat,
        )
        self.repeat_check.pack(side="left", padx=(18, 0))
        ttk.Entry(options, textvariable=self.repeat_ms_var, width=8).pack(
            side="left", padx=(6, 2)
        )
        ttk.Label(options, text="ms（≥10）").pack(side="left")
        ttk.Label(
            options,
            text="Ctrl+Enter 发送 | Ctrl+↑ 上一条",
            style="Hint.TLabel",
        ).pack(side="right")

        self.send_text = tk.Text(
            tx_frame,
            height=4,
            wrap="word",
            font=DATA_FONT,
            undo=True,
            padx=6,
            pady=4,
        )
        self.send_text.grid(row=1, column=0, sticky="nsew", pady=(4, 0))
        send_scroll = ttk.Scrollbar(
            tx_frame, orient="vertical", command=self.send_text.yview
        )
        self.send_text.configure(yscrollcommand=send_scroll.set)
        send_scroll.grid(row=1, column=1, sticky="ns", pady=(4, 0))

        buttons = ttk.Frame(tx_frame)
        buttons.grid(row=1, column=2, sticky="ns", padx=(8, 0), pady=(4, 0))
        ttk.Button(buttons, text="发送", width=11, command=self.send_current).pack(
            fill="x"
        )
        ttk.Button(buttons, text="清空输入", command=self.clear_send).pack(
            fill="x", pady=(6, 0)
        )
        ttk.Button(buttons, text="上一条", command=self.recall_previous).pack(
            fill="x", pady=(6, 0)
        )

        self.send_text.bind("<Control-Return>", self._on_send_shortcut)
        self.send_text.bind("<Control-Up>", self._on_recall_shortcut)
        self.root.bind("<F5>", lambda _event: self._refresh_ports())

        paned.add(rx_frame, weight=3)
        paned.add(tx_frame, weight=1)

    def _build_history_tab(self) -> None:
        tab = self.tab_history
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(1, weight=1)

        bar = ttk.Frame(tab)
        bar.grid(row=0, column=0, columnspan=2, sticky="ew", padx=6, pady=(8, 4))

        row1 = ttk.Frame(bar)
        row1.pack(fill="x")
        ttk.Button(row1, text="重新发送", command=self.resend_selected_history).pack(
            side="left"
        )
        ttk.Button(row1, text="载入发送区", command=self.load_selected_history).pack(
            side="left", padx=(6, 0)
        )
        ttk.Button(row1, text="复制 HEX", command=self.copy_history_hex).pack(
            side="left", padx=(6, 0)
        )

        row2 = ttk.Frame(bar)
        row2.pack(fill="x", pady=(4, 0))
        ttk.Button(row2, text="导出 CSV", command=self.export_history).pack(
            side="left"
        )
        ttk.Button(row2, text="清空记录", command=self.clear_history).pack(
            side="left", padx=(6, 0)
        )

        columns = ("time", "mode", "size", "text", "hex")
        self.history_tree = ttk.Treeview(
            tab, columns=columns, show="headings", selectmode="browse", height=12
        )
        headings = {
            "time": ("时间", 150, False),
            "mode": ("格式", 56, False),
            "size": ("字节", 52, False),
            "text": ("内容（文本）", 200, True),
            "hex": ("内容（HEX）", 200, True),
        }
        for name, (title, width, stretch) in headings.items():
            self.history_tree.heading(name, text=title)
            self.history_tree.column(
                name, width=width, minwidth=46, anchor="w", stretch=stretch
            )
        self.history_tree.grid(row=1, column=0, sticky="nsew", padx=(6, 0))
        history_scroll = ttk.Scrollbar(
            tab, orient="vertical", command=self.history_tree.yview
        )
        self.history_tree.configure(yscrollcommand=history_scroll.set)
        history_scroll.grid(row=1, column=1, sticky="ns", padx=(0, 6))
        history_xscroll = ttk.Scrollbar(
            tab, orient="horizontal", command=self.history_tree.xview
        )
        self.history_tree.configure(xscrollcommand=history_xscroll.set)
        history_xscroll.grid(row=2, column=0, sticky="ew", padx=(6, 0))
        self.history_tree.bind("<Double-1>", self._on_history_double_click)

        footer = ttk.Frame(tab)
        footer.grid(row=3, column=0, columnspan=2, sticky="ew", padx=6, pady=(4, 8))
        log_hint = ttk.Label(footer, textvariable=self.log_hint_var, style="Hint.TLabel")
        log_hint.pack(fill="x")
        log_hint.bind("<Configure>", self._auto_wrap)
        hint = ttk.Label(
            footer,
            text="双击记录可载入发送区；每次发送都会写入 logs 目录的日志文件。",
            style="Hint.TLabel",
            wraplength=240,
            justify="left",
        )
        hint.pack(fill="x", pady=(2, 0))
        hint.bind("<Configure>", self._auto_wrap)

    def _build_quick_tab(self) -> None:
        tab = self.tab_quick
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(1, weight=1)

        bar = ttk.Frame(tab)
        bar.grid(row=0, column=0, columnspan=2, sticky="ew", padx=6, pady=(8, 4))

        row1 = ttk.Frame(bar)
        row1.pack(fill="x")
        ttk.Button(row1, text="新增", command=self.add_quick_command).pack(side="left")
        ttk.Button(row1, text="编辑", command=self.edit_quick_command).pack(
            side="left", padx=(6, 0)
        )
        ttk.Button(row1, text="删除", command=self.delete_quick_command).pack(
            side="left", padx=(6, 0)
        )

        row2 = ttk.Frame(bar)
        row2.pack(fill="x", pady=(4, 0))
        ttk.Button(row2, text="发送选中指令", command=self.send_quick_selected).pack(
            side="left"
        )

        # 列表只展示名称与内容；HEX / 文本格式在新增、编辑对话框中设置
        self.quick_tree = ttk.Treeview(
            tab,
            columns=("name", "content"),
            show="headings",
            selectmode="browse",
            height=12,
        )
        self.quick_tree.heading("name", text="名称")
        self.quick_tree.heading("content", text="内容")
        self.quick_tree.column(
            "name", width=120, minwidth=70, anchor="w", stretch=False
        )
        self.quick_tree.column("content", width=200, minwidth=90, anchor="w")
        self.quick_tree.grid(row=1, column=0, sticky="nsew", padx=(6, 0))
        scroll = ttk.Scrollbar(tab, orient="vertical", command=self.quick_tree.yview)
        self.quick_tree.configure(yscrollcommand=scroll.set)
        scroll.grid(row=1, column=1, sticky="ns", padx=(0, 6))
        quick_xscroll = ttk.Scrollbar(
            tab, orient="horizontal", command=self.quick_tree.xview
        )
        self.quick_tree.configure(xscrollcommand=quick_xscroll.set)
        quick_xscroll.grid(row=2, column=0, sticky="ew", padx=(6, 0))
        self.quick_tree.bind("<Double-1>", self._on_quick_double_click)

        footer = ttk.Frame(tab)
        footer.grid(row=3, column=0, columnspan=2, sticky="ew", padx=6, pady=(4, 8))
        hint = ttk.Label(
            footer,
            text=(
                "双击即可下发；格式（文本 / HEX）在新增、编辑时选择；发送遵循“追加 \\r\\n”选项。"
            ),
            style="Hint.TLabel",
            wraplength=240,
            justify="left",
        )
        hint.pack(fill="x")
        hint.bind("<Configure>", self._auto_wrap)

    # ------------------------------------------------------------------ #
    # 串口管理
    # ------------------------------------------------------------------ #
    def _refresh_ports(self) -> None:
        labels: list[str] = []
        self._port_map.clear()
        self._port_details.clear()
        try:
            ports = sorted(list_ports.comports(), key=self._port_sort_key)
        except Exception:  # noqa: BLE001 - 枚举失败时退化为空列表
            ports = []
        for port in ports:
            description = self._clean_description(port.description or "")
            label = f"{port.device} · {description}" if description else port.device
            if label in self._port_map:  # 同名设备（少见）补序号，保证条目唯一
                suffix = 2
                while f"{label} #{suffix}" in self._port_map:
                    suffix += 1
                label = f"{label} #{suffix}"
            labels.append(label)
            self._port_map[label] = port.device
            self._port_details[label] = self._describe_port(port)

        self.port_combo.configure(values=labels)
        self._fit_port_popdown(labels)

        current = self.port_var.get().strip()
        matched = self._match_port_label(current, labels)
        if matched:
            if matched != current:
                self.port_var.set(matched)
            self._update_port_detail()
            return

        if labels:
            self.port_var.set(labels[0])
            if not self.link.is_open:
                self._set_status("就绪，点击“打开串口”开始通信")
        else:
            self.port_var.set("")
            if not self.link.is_open:
                self._set_status("未发现串口，请连接设备后点击“刷新”（F5）")
        self._update_port_detail()

    @staticmethod
    def _port_sort_key(port: list_ports.ListPortInfo) -> tuple[str, int, str]:
        """让 COM2 排在 COM10 前面：按前缀 + 数字 + 原文三级排序。"""
        match = re.search(r"(\d+)$", port.device)
        if not match:
            return (port.device.upper(), 0, port.device)
        return (port.device[: match.start()].upper(), int(match.group(1)), port.device)

    @staticmethod
    def _clean_description(description: str) -> str:
        """Windows 的描述常以 "(COM3)" 结尾，去掉后避免与前面的设备名重复。"""
        text = description.strip()
        if not text or text.lower() == "n/a":
            return ""
        return re.sub(r"\s*\(\s*COM\d+\s*\)\s*$", "", text, flags=re.IGNORECASE).strip()

    @classmethod
    def _describe_port(cls, port: list_ports.ListPortInfo) -> str:
        """下拉框放不下的完整信息，放到“当前选择”那一行展示。"""
        parts = [port.device]
        description = (port.description or "").strip()
        if description and description.lower() != "n/a":
            parts.append(description)
        manufacturer = str(getattr(port, "manufacturer", "") or "").strip()
        if manufacturer:
            parts.append(manufacturer)
        hwid = str(getattr(port, "hwid", "") or "").strip()
        if hwid and hwid.lower() != "n/a":
            parts.append(cls._short_hwid(hwid))
        return "  |  ".join(parts)

    @staticmethod
    def _short_hwid(hwid: str) -> str:
        """hwid 很长，只保留能区分设备的 VID:PID 与 LOCATION。"""
        parts = re.findall(r"VID:PID=\S+|LOCATION=\S+", hwid, flags=re.IGNORECASE)
        return " ".join(parts) if parts else hwid

    def _fit_port_popdown(self, labels: list[str]) -> None:
        """把下拉弹窗撑到最长条目的宽度，避免设备名在列表里被裁掉。"""
        if not labels:
            return
        try:
            popdown = self.port_combo.tk.call(
                "ttk::combobox::PopdownWindow", str(self.port_combo)
            )
            listbox = f"{popdown}.f.l"
            style_font = ttk.Style(self.root).lookup("TCombobox", "font")
            font = (
                tkfont.Font(font=style_font)
                if style_font
                else tkfont.nametofont("TkDefaultFont")
            )
            wanted = max(font.measure(text) for text in labels) + 30
            wanted = min(wanted, self.root.winfo_screenwidth() - 80)
            current = int(float(self.port_combo.tk.call(listbox, "cget", "-width")))
            if wanted > current:
                self.port_combo.tk.call(listbox, "configure", "-width", wanted)
        except (tk.TclError, ValueError):
            pass

    def _match_port_label(self, current: str, labels: list[str]) -> str:
        """先按完整标签匹配，再退化为按设备名（COMx）找回上次的选择。"""
        if not current:
            return ""
        if current in labels:
            return current
        device = self._port_map.get(current, current.split(" · ")[0].strip())
        for label in labels:
            if self._port_map.get(label) == device:
                return label
        return ""

    def _update_port_detail(self) -> None:
        label = self.port_var.get().strip()
        detail = self._port_details.get(label, "")
        if detail:
            self.port_detail_var.set(f"当前选择：{detail}")
        elif self._port_details:
            self.port_detail_var.set(f"当前选择：{label or '未选择'}")
        else:
            self.port_detail_var.set("未发现串口，请连接设备后点击“刷新”（F5）")

    def _on_port_selected(self, _event=None) -> None:
        self._update_port_detail()
        self._schedule_config_save()

    def _auto_wrap(self, event) -> None:
        """提示类标签按当前宽度换行，左侧栏变窄时也不会被截断。"""
        widget = event.widget
        wrap = max(120, int(event.width) - 4)
        try:
            current = int(float(widget.cget("wraplength") or 0))
        except (tk.TclError, ValueError):
            current = 0
        if current == wrap:
            return
        try:
            widget.configure(wraplength=wrap)
        except tk.TclError:
            pass

    def _selected_port_device(self) -> str:
        label = self.port_var.get().strip()
        if not label:
            return ""
        return self._port_map.get(label, label.split(" · ")[0].strip())

    def toggle_connection(self) -> None:
        if self.link.is_open:
            self.close_port()
        else:
            self.open_port()

    def open_port(self) -> None:
        port = self._selected_port_device()
        if not port:
            messagebox.showwarning("提示", "请先选择串口。", parent=self.root)
            return
        try:
            baudrate = int(self.baud_var.get())
            if baudrate <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("参数错误", "波特率必须是正整数。", parent=self.root)
            return

        rtscts, xonxoff = FLOW_MAP.get(self.flow_var.get(), (False, False))
        encoding = self.encoding_var.get()
        try:
            self.link.open(
                port=port,
                baudrate=baudrate,
                bytesize=BYTESIZE_MAP.get(self.bytesize_var.get(), serial.EIGHTBITS),
                parity=PARITY_MAP.get(self.parity_var.get(), serial.PARITY_NONE),
                stopbits=STOPBITS_MAP.get(self.stopbits_var.get(), serial.STOPBITS_ONE),
                rtscts=rtscts,
                xonxoff=xonxoff,
                encoding=encoding,
            )
        except (serial.SerialException, OSError, ValueError) as exc:
            messagebox.showerror(
                "打开串口失败", f"{port} 打开失败：\n{exc}", parent=self.root
            )
            self._set_status("未连接")
            return

        self._apply_line_states()
        self.rx_count = 0
        self.tx_count = 0
        self._update_counters()
        self.open_btn.configure(text="关闭串口")
        self._enable_settings(False)
        frame_text = (
            f"{self.bytesize_var.get()}"
            f"{PARITY_LETTER.get(self.parity_var.get(), 'N')}"
            f"{self.stopbits_var.get()}"
        )
        self._set_status(
            f"已连接 {port} | {baudrate} {frame_text} | "
            f"流控 {self.flow_var.get()} | 编码 {encoding}"
        )
        self._append_line(f"串口已打开：{port} {baudrate} {frame_text}", "SYS", newline=True)
        self._schedule_config_save()

    def close_port(self) -> None:
        self._stop_repeat()
        was_open = self.link.is_open
        port = self.link.port
        self.link.close()
        self.open_btn.configure(text="打开串口")
        self._enable_settings(True)
        self._set_status("已断开连接" if was_open else "未连接")
        if was_open:
            self._append_line(f"串口已关闭：{port or '-'}", "SYS", newline=True)

    def _enable_settings(self, enabled: bool) -> None:
        for widget, normal_state in self._settings_widgets:
            try:
                widget.configure(state=normal_state if enabled else "disabled")
            except tk.TclError:
                pass

    def _apply_line_states(self) -> None:
        if not self.link.is_open:
            return
        try:
            self.link.set_lines(self.dtr_var.get(), self.rts_var.get())
        except (serial.SerialException, OSError) as exc:
            self._set_status(f"设置 DTR/RTS 失败：{exc}")

    def _on_encoding_changed(self, _event=None) -> None:
        try:
            self.link.set_encoding(self.encoding_var.get())
        except LookupError:
            self.encoding_var.set("UTF-8")
            self.link.set_encoding("UTF-8")
        self._schedule_config_save()

    def open_log_dir(self) -> None:
        try:
            LOG_DIR.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            messagebox.showerror("打开日志目录失败", str(exc), parent=self.root)
            return
        try:
            os.startfile(str(LOG_DIR))  # type: ignore[attr-defined]  # Windows
        except AttributeError:
            try:
                subprocess.Popen(["xdg-open", str(LOG_DIR)])
            except OSError as exc:
                messagebox.showerror("打开日志目录失败", str(exc), parent=self.root)
        except OSError as exc:
            messagebox.showerror("打开日志目录失败", str(exc), parent=self.root)

    def _set_status(self, message: str) -> None:
        self.status_var.set(message)

    def _update_counters(self) -> None:
        self.counter_var.set(f"RX {self.rx_count:,} 字节 | TX {self.tx_count:,} 字节")

    def reset_counters(self) -> None:
        self.rx_count = 0
        self.tx_count = 0
        self._update_counters()

    # ------------------------------------------------------------------ #
    # 串口事件轮询
    # ------------------------------------------------------------------ #
    def _poll_events(self) -> None:
        try:
            while True:
                event = self.link.events.get_nowait()
                kind = event[0]
                if kind == "rx":
                    self._handle_rx(event[1], event[2])
                elif kind == "error":
                    self._handle_serial_error(event[1])
        except queue.Empty:
            pass
        if not self._closing:
            self.root.after(POLL_INTERVAL_MS, self._poll_events)

    def _handle_rx(self, data: bytes, text: str) -> None:
        if not data:
            return
        self.rx_count += len(data)
        self._update_counters()
        self._write_log("RX", data)
        if self.hex_view_var.get():
            self._append_line(to_hex(data), "RX", newline=True)
        else:
            self._append_line(text, "RX")

    def _handle_serial_error(self, message: str) -> None:
        if self._closing:
            return
        self._stop_repeat()
        was_open = self.link.is_open
        self.link.close()
        self.open_btn.configure(text="打开串口")
        self._enable_settings(True)
        self._set_status(f"串口异常：{message}")
        self._append_line(message, "SYS", newline=True)
        if was_open:
            messagebox.showwarning(
                "串口异常",
                f"{message}\n\n连接已断开，请检查设备后重新打开串口。",
                parent=self.root,
            )

    # ------------------------------------------------------------------ #
    # 接收区显示
    # ------------------------------------------------------------------ #
    _TAG_LABEL = {"RX": "RX ←", "TX": "TX →", "SYS": "--"}

    def _append_line(self, text: str, tag: str = "RX", newline: bool = False) -> None:
        """向接收区追加一行内容，可按需添加时间戳与前缀。"""
        if text is None:
            return
        if newline:
            text += "\n"
        if text == "":
            return

        head = ""
        if self.show_ts_var.get():
            head += f"[{now_clock()}] "
        head += f"{self._TAG_LABEL.get(tag, tag)} "

        parts = text.split("\n")
        pieces: list[str] = []
        at_start = self._rx_at_line_start
        if not at_start and tag != "RX":
            # 收到半行数据时，TX/SYS 回显另起一行，避免和接收内容黏在一起
            pieces.append("\n")
            at_start = True
        for index, part in enumerate(parts):
            is_last = index == len(parts) - 1
            if is_last and part == "":
                break
            if at_start:
                pieces.append(head)
                at_start = False
            pieces.append(part)
            if not is_last:
                pieces.append("\n")
                at_start = True
        if not pieces:
            return
        self._write_rx_text("".join(pieces), tag)
        self._rx_at_line_start = at_start

    def _write_rx_text(self, content: str, tag: str = "RX") -> None:
        widget = self.rx_text
        widget.configure(state="normal")
        widget.insert("end", content, (tag,))
        line_count = int(widget.index("end-1c").split(".")[0])
        if line_count > RX_LINE_LIMIT:
            widget.delete("1.0", f"{RX_LINE_TRIM}.0")
        widget.configure(state="disabled")
        if self.autoscroll_var.get():
            widget.see("end")

    def clear_rx(self) -> None:
        self.rx_text.configure(state="normal")
        self.rx_text.delete("1.0", "end")
        self.rx_text.configure(state="disabled")
        self._rx_at_line_start = True

    def save_rx(self) -> None:
        content = self.rx_text.get("1.0", "end-1c")
        if not content.strip():
            messagebox.showinfo("提示", "接收区还没有内容。", parent=self.root)
            return
        path = filedialog.asksaveasfilename(
            parent=self.root,
            title="保存接收内容",
            defaultextension=".txt",
            filetypes=(("文本文件", "*.txt"), ("所有文件", "*.*")),
            initialfile=f"uart_rx_{datetime.now():%Y%m%d_%H%M%S}.txt",
        )
        if not path:
            return
        try:
            Path(path).write_text(content, encoding="utf-8")
        except OSError as exc:
            messagebox.showerror("保存失败", str(exc), parent=self.root)
            return
        self._set_status(f"接收内容已保存：{path}")

    # ------------------------------------------------------------------ #
    # 发送
    # ------------------------------------------------------------------ #
    def _on_send_shortcut(self, _event=None):
        self.send_current()
        return "break"

    def _on_recall_shortcut(self, _event=None):
        self.recall_previous()
        return "break"

    def clear_send(self) -> None:
        self.send_text.delete("1.0", "end")
        self.send_text.focus_set()

    def send_current(self, silent: bool = False) -> bool:
        raw = self.send_text.get("1.0", "end-1c")
        if not raw.strip():
            if not silent:
                self._set_status("发送区为空，请输入要发送的内容")
                self.send_text.focus_set()
            return False
        return self._send_raw(raw, silent=silent)

    def _send_raw(self, raw: str, silent: bool = False) -> bool:
        hex_mode = self.hex_send_var.get()
        try:
            if hex_mode:
                data = parse_hex(raw)
            else:
                data = raw.encode(self.encoding_var.get(), errors="replace")
        except (ValueError, LookupError) as exc:
            self._stop_repeat()
            if not silent:
                messagebox.showerror(
                    "发送失败", f"数据格式有误：\n{exc}", parent=self.root
                )
            return False
        if self.append_crlf_var.get():
            data += b"\r\n"
        if not data:
            if not silent:
                self._set_status("待发送数据为空")
            return False
        self._recall_index = -1
        return self._transmit(data, hex_mode, silent=silent)

    def _transmit(self, data: bytes, hex_mode: bool, silent: bool = False) -> bool:
        if not self.link.is_open:
            self._stop_repeat()
            if not silent:
                messagebox.showwarning(
                    "未连接", "请先打开串口再发送数据。", parent=self.root
                )
            return False
        try:
            self.link.write(data)
        except (serial.SerialException, OSError) as exc:
            self._handle_serial_error(f"串口写入失败：{exc}")
            return False

        self.tx_count += len(data)
        self._update_counters()
        self._write_log("TX", data)
        self._record_history(data, hex_mode)
        self._echo_tx(data)
        return True

    def _echo_tx(self, data: bytes) -> None:
        if not self.echo_tx_var.get():
            return
        if self.hex_view_var.get():
            self._append_line(to_hex(data), "TX", newline=True)
            return
        text = data.decode(self.encoding_var.get(), errors="replace")
        self._append_line(text, "TX", newline=not text.endswith("\n"))

    # ------------------------------------------------------------------ #
    # 定时发送 / 历史指令回填
    # ------------------------------------------------------------------ #
    def _toggle_repeat(self) -> None:
        if not self.repeat_var.get():
            self._cancel_repeat()
            return
        try:
            interval = int(self.repeat_ms_var.get())
            if interval < 10:
                raise ValueError
        except ValueError:
            self.repeat_var.set(False)
            messagebox.showerror(
                "定时发送", "定时间隔必须是 ≥ 10 的整数（毫秒）。", parent=self.root
            )
            return
        if not self.link.is_open:
            self.repeat_var.set(False)
            messagebox.showwarning("未连接", "请先打开串口。", parent=self.root)
            return
        if not self.send_text.get("1.0", "end-1c").strip():
            self.repeat_var.set(False)
            messagebox.showwarning(
                "提示", "发送区为空，请先输入要定时发送的内容。", parent=self.root
            )
            return
        self._cancel_repeat(keep_flag=True)
        self.repeat_check.configure(text="停止定时")
        self._set_status(f"定时发送中，间隔 {interval} ms")
        self._repeat_job = self.root.after(interval, self._repeat_tick)

    def _repeat_tick(self) -> None:
        self._repeat_job = None
        if not self.repeat_var.get():
            return
        self.send_current(silent=True)
        try:
            interval = max(int(self.repeat_ms_var.get()), 10)
        except ValueError:
            interval = 1000
        if self.repeat_var.get():
            self._repeat_job = self.root.after(interval, self._repeat_tick)

    def _cancel_repeat(self, keep_flag: bool = False) -> None:
        if self._repeat_job is not None:
            try:
                self.root.after_cancel(self._repeat_job)
            except tk.TclError:
                pass
            self._repeat_job = None
        if not keep_flag:
            self.repeat_var.set(False)
        try:
            self.repeat_check.configure(text="定时发送")
        except tk.TclError:
            pass

    def _stop_repeat(self) -> None:
        self._cancel_repeat()

    def recall_previous(self) -> None:
        if not self.history:
            self._set_status("暂无发送记录")
            return
        self._recall_index = min(self._recall_index + 1, len(self.history) - 1)
        entry = self.history[self._recall_index]
        self._load_entry_to_send(entry)
        self._set_status(
            f"已载入第 {self._recall_index + 1} 条历史指令（共 {len(self.history)} 条）"
        )

    def _load_entry_to_send(self, entry: dict) -> None:
        hex_mode = entry.get("mode") == "HEX"
        self.hex_send_var.set(hex_mode)
        content = entry.get("hex", "") if hex_mode else entry.get("text", "")
        self.send_text.delete("1.0", "end")
        self.send_text.insert("1.0", content)
        self.send_text.focus_set()
        self.send_text.mark_set("insert", "end")

    # ------------------------------------------------------------------ #
    # 日志（记录收发数据）
    # ------------------------------------------------------------------ #
    def _ensure_log_handle(self):
        today = datetime.now().strftime("%Y%m%d")
        target = LOG_DIR / f"uart_{today}.log"
        if self.log_handle is not None and self.log_path == target:
            return self.log_handle
        self._close_log()
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.log_handle = open(target, "a", encoding="utf-8")
        self.log_path = target
        self.log_hint_var.set(f"当前日志：{target.name}")
        return self.log_handle

    def _close_log(self) -> None:
        if self.log_handle is not None:
            try:
                self.log_handle.close()
            except OSError:
                pass
        self.log_handle = None
        self.log_path = None

    def _write_log(self, direction: str, data: bytes) -> None:
        """把收发数据写入当天日志，格式：时间 方向 HEX | 文本。"""
        if not self.log_file_var.get():
            return
        try:
            handle = self._ensure_log_handle()
            handle.write(
                f"{now_stamp()} {direction} {to_hex(data)} | "
                f"{preview_text(data, self.encoding_var.get())}\n"
            )
            handle.flush()
        except (OSError, ValueError) as exc:
            self.log_file_var.set(False)
            self._set_status(f"日志写入失败（已停止记录）：{exc}")

    # ------------------------------------------------------------------ #
    # 发送记录（内存 + 界面表格 + 持久化）
    # ------------------------------------------------------------------ #
    @staticmethod
    def _history_preview(entry: dict, limit: int = 120) -> str:
        text = str(entry.get("text", ""))
        preview = (
            text.replace("\\", "\\\\")
            .replace("\r", "\\r")
            .replace("\n", "\\n")
            .replace("\t", "\\t")
        )
        preview = _CONTROL_CHARS.sub(
            lambda match: f"\\x{ord(match.group()):02x}", preview
        )
        if len(preview) > limit:
            preview = preview[:limit] + "…"
        return preview

    def _record_history(self, data: bytes, hex_mode: bool) -> None:
        encoding = self.encoding_var.get()
        try:
            text = data.decode(encoding, errors="replace")
        except LookupError:
            text = data.decode("latin-1", errors="replace")
        entry = {
            "id": uuid.uuid4().hex,
            "time": now_stamp(),
            "mode": "HEX" if hex_mode else "TEXT",
            "size": len(data),
            "hex": to_hex(data),
            "text": text,
            "encoding": encoding,
        }
        self.history.insert(0, entry)
        del self.history[HISTORY_LIMIT:]
        self._insert_history_row(0, entry)
        self._trim_history_rows()
        self._schedule_config_save()

    def _insert_history_row(self, position: int, entry: dict) -> None:
        try:
            self.history_tree.insert(
                "",
                position,
                iid=entry["id"],
                values=(
                    entry["time"],
                    "HEX" if entry["mode"] == "HEX" else "文本",
                    entry["size"],
                    self._history_preview(entry),
                    entry["hex"],
                ),
            )
        except tk.TclError:
            pass

    def _trim_history_rows(self) -> None:
        for iid in self.history_tree.get_children()[HISTORY_LIMIT:]:
            self.history_tree.delete(iid)

    def _refresh_history_view(self) -> None:
        for iid in self.history_tree.get_children():
            self.history_tree.delete(iid)
        for index, entry in enumerate(self.history):
            self._insert_history_row(index, entry)

    def _selected_history(self, quiet: bool = False) -> dict | None:
        selection = self.history_tree.selection()
        if not selection:
            if not quiet:
                messagebox.showinfo(
                    "提示", "请先在列表中选中一条记录。", parent=self.root
                )
            return None
        iid = selection[0]
        for entry in self.history:
            if entry["id"] == iid:
                return entry
        return None

    def _on_history_double_click(self, event) -> None:
        if self.history_tree.identify_row(event.y):
            self.load_selected_history()

    def resend_selected_history(self) -> None:
        entry = self._selected_history()
        if not entry:
            return
        try:
            data = parse_hex(entry["hex"])
        except ValueError as exc:
            messagebox.showerror("重新发送失败", str(exc), parent=self.root)
            return
        if not data:
            messagebox.showinfo("提示", "该记录没有可发送的数据。", parent=self.root)
            return
        self._transmit(data, entry["mode"] == "HEX")

    def load_selected_history(self) -> None:
        entry = self._selected_history()
        if not entry:
            return
        self._load_entry_to_send(entry)
        self._set_status("已把历史指令载入左侧发送区，可修改后发送")

    def copy_history_hex(self) -> None:
        entry = self._selected_history()
        if not entry:
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(entry["hex"])
        self._set_status("HEX 内容已复制到剪贴板")

    def export_history(self) -> None:
        if not self.history:
            messagebox.showinfo("提示", "暂无发送记录可导出。", parent=self.root)
            return
        path = filedialog.asksaveasfilename(
            parent=self.root,
            title="导出发送记录",
            defaultextension=".csv",
            filetypes=(("CSV 文件", "*.csv"), ("所有文件", "*.*")),
            initialfile=f"uart_tx_history_{datetime.now():%Y%m%d_%H%M%S}.csv",
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8-sig", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(
                    ["时间", "格式", "字节数", "内容（文本）", "内容（HEX）", "编码"]
                )
                for entry in reversed(self.history):
                    writer.writerow(
                        [
                            entry["time"],
                            "HEX" if entry["mode"] == "HEX" else "文本",
                            entry["size"],
                            entry["text"],
                            entry["hex"],
                            entry.get("encoding", ""),
                        ]
                    )
        except OSError as exc:
            messagebox.showerror("导出失败", str(exc), parent=self.root)
            return
        self._set_status(f"发送记录已导出：{path}")

    def clear_history(self) -> None:
        if not self.history:
            return
        if not messagebox.askyesno(
            "清空记录",
            "确定清空发送记录列表吗？\n（logs 目录中的日志文件不会被删除）",
            parent=self.root,
        ):
            return
        self.history.clear()
        self._refresh_history_view()
        self._recall_index = -1
        self._schedule_config_save()
        self._set_status("发送记录已清空")

    # ------------------------------------------------------------------ #
    # 快捷指令
    # ------------------------------------------------------------------ #
    def _refresh_quick_view(self) -> None:
        for iid in self.quick_tree.get_children():
            self.quick_tree.delete(iid)
        for entry in self.quick_commands:
            content = (
                entry["content"].replace("\r", "\\r").replace("\n", "\\n")
            )
            self.quick_tree.insert(
                "",
                "end",
                iid=entry["id"],
                values=(entry["name"], content),
            )

    def _selected_quick(self, quiet: bool = False) -> dict | None:
        selection = self.quick_tree.selection()
        if not selection:
            if not quiet:
                messagebox.showinfo(
                    "提示", "请先选中一条快捷指令。", parent=self.root
                )
            return None
        iid = selection[0]
        for entry in self.quick_commands:
            if entry["id"] == iid:
                return entry
        return None

    def _on_quick_double_click(self, event) -> None:
        if self.quick_tree.identify_row(event.y):
            self.send_quick_selected()

    def add_quick_command(self) -> None:
        result = self._command_dialog("新增快捷指令")
        if not result:
            return
        result["id"] = uuid.uuid4().hex
        self.quick_commands.append(result)
        self._refresh_quick_view()
        self._schedule_config_save()
        self._set_status(f"已添加快捷指令：{result['name']}")

    def edit_quick_command(self) -> None:
        entry = self._selected_quick()
        if not entry:
            return
        result = self._command_dialog("编辑快捷指令", entry)
        if not result:
            return
        entry.update(result)
        self._refresh_quick_view()
        self._schedule_config_save()
        self._set_status(f"已更新快捷指令：{entry['name']}")

    def delete_quick_command(self) -> None:
        entry = self._selected_quick()
        if not entry:
            return
        if not messagebox.askyesno(
            "删除指令",
            f"确定删除快捷指令“{entry['name']}”吗？",
            parent=self.root,
        ):
            return
        self.quick_commands = [
            item for item in self.quick_commands if item["id"] != entry["id"]
        ]
        self._refresh_quick_view()
        self._schedule_config_save()
        self._set_status("快捷指令已删除")

    def send_quick_selected(self) -> None:
        entry = self._selected_quick()
        if not entry:
            return
        try:
            if entry["mode"] == "HEX":
                data = parse_hex(entry["content"])
            else:
                data = entry["content"].encode(
                    self.encoding_var.get(), errors="replace"
                )
        except (ValueError, LookupError) as exc:
            messagebox.showerror(
                "发送失败", f"快捷指令内容有误：\n{exc}", parent=self.root
            )
            return
        if self.append_crlf_var.get():
            data += b"\r\n"
        if not data:
            messagebox.showinfo("提示", "该快捷指令内容为空。", parent=self.root)
            return
        self._transmit(data, entry["mode"] == "HEX")

    def _auto_quick_name(self, body: str, is_hex: bool) -> str:
        """按内容生成缺省名称：文本取前 24 字，HEX 归一化后加前缀。"""
        text = " ".join(body.split())
        if is_hex:
            try:
                text = f"HEX {to_hex(parse_hex(body))}"
            except ValueError:
                pass
        if not text:
            return f"指令 {len(self.quick_commands) + 1}"
        return text if len(text) <= 24 else text[:24] + "…"

    def _command_dialog(self, title: str, initial: dict | None = None) -> dict | None:
        initial = initial or {}
        dialog = tk.Toplevel(self.root)
        dialog.title(title)
        dialog.transient(self.root)
        dialog.resizable(True, True)
        dialog.geometry("560x340")
        dialog.minsize(480, 300)
        dialog.grab_set()

        result: dict = {}

        frame = ttk.Frame(dialog, padding=12)
        frame.pack(fill="both", expand=True)
        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(2, weight=1)

        ttk.Label(frame, text="名称（留空自动命名）").grid(row=0, column=0, sticky="w")
        name_var = tk.StringVar(value=str(initial.get("name", "")))
        name_entry = ttk.Entry(frame, textvariable=name_var)
        name_entry.grid(row=0, column=1, sticky="ew", pady=(0, 8), padx=(8, 0))

        hex_var = tk.BooleanVar(value=initial.get("mode") == "HEX")
        ttk.Checkbutton(
            frame, text="HEX 格式（例如 01 03 00 00 00 0A）", variable=hex_var
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 6))

        ttk.Label(frame, text="内容").grid(row=2, column=0, sticky="nw")
        content_text = tk.Text(frame, height=8, font=DATA_FONT, wrap="word")
        content_text.grid(row=2, column=1, sticky="nsew", padx=(8, 0))
        content_text.insert("1.0", str(initial.get("content", "")))

        def on_ok() -> None:
            body = content_text.get("1.0", "end-1c")
            if not body.strip():
                messagebox.showwarning("提示", "请输入指令内容。", parent=dialog)
                content_text.focus_set()
                return
            if hex_var.get():
                try:
                    parse_hex(body)
                except ValueError as exc:
                    messagebox.showerror("格式错误", str(exc), parent=dialog)
                    return
            # 名称留空时按内容自动生成，新增指令不必先起名字
            name = name_var.get().strip() or self._auto_quick_name(
                body, hex_var.get()
            )
            result.update(
                {
                    "name": name,
                    "mode": "HEX" if hex_var.get() else "TEXT",
                    "content": body,
                }
            )
            dialog.destroy()

        buttons = ttk.Frame(frame)
        buttons.grid(row=3, column=0, columnspan=2, sticky="e", pady=(12, 0))
        ttk.Button(buttons, text="确定", width=10, command=on_ok).pack(
            side="left", padx=(0, 8)
        )
        ttk.Button(buttons, text="取消", width=10, command=dialog.destroy).pack(
            side="left"
        )

        dialog.bind("<Escape>", lambda _event: dialog.destroy())
        name_entry.focus_set()
        self.root.wait_window(dialog)
        return result or None

    # ------------------------------------------------------------------ #
    # 配置持久化
    # ------------------------------------------------------------------ #
    def _collect_config(self) -> dict:
        return {
            "port": self.port_var.get(),
            "baudrate": self.baud_var.get(),
            "bytesize": self.bytesize_var.get(),
            "parity": self.parity_var.get(),
            "stopbits": self.stopbits_var.get(),
            "flow": self.flow_var.get(),
            "encoding": self.encoding_var.get(),
            "dtr": self.dtr_var.get(),
            "rts": self.rts_var.get(),
            "hex_send": self.hex_send_var.get(),
            "append_crlf": self.append_crlf_var.get(),
            "repeat_ms": self.repeat_ms_var.get(),
            "hex_view": self.hex_view_var.get(),
            "show_timestamp": self.show_ts_var.get(),
            "echo_tx": self.echo_tx_var.get(),
            "autoscroll": self.autoscroll_var.get(),
            "log_to_file": self.log_file_var.get(),
            "sash_pos": self._workspace_sash(),
            "history": self.history[:HISTORY_LIMIT],
            "quick_commands": self.quick_commands,
        }

    def _schedule_config_save(self) -> None:
        if self._closing:
            return
        if self._save_job is not None:
            try:
                self.root.after_cancel(self._save_job)
            except tk.TclError:
                pass
        self._save_job = self.root.after(CONFIG_SAVE_DELAY_MS, self._save_config)

    def _save_config(self) -> None:
        self._save_job = None
        if not save_json(self.config_path, self._collect_config()):
            self._set_status(f"配置保存失败：{self.config_path}")
            return
        self._sync_config_pointer()

    def _sync_config_pointer(self) -> None:
        """把当前选中的配置文件写进默认 config.json，下次启动自动沿用。"""
        if self.config_path.resolve() == CONFIG_PATH.resolve():
            return
        base = load_json(CONFIG_PATH, {})
        base = base if isinstance(base, dict) else {}
        base["active_config"] = str(self.config_path)
        save_json(CONFIG_PATH, base)

    def _update_config_hint(self) -> None:
        name = self.config_path.name
        try:
            if self.config_path.parent.resolve() != APP_DIR.resolve():
                name = f"{self.config_path.parent.name}/{name}"
        except OSError:
            pass
        self.config_hint_var.set(f"配置文件：{name}")

    def _flush_config_save(self) -> None:
        """立即把挂起的延迟保存落盘（切换配置时使用）。"""
        if self._save_job is not None:
            try:
                self.root.after_cancel(self._save_job)
            except tk.TclError:
                pass
            self._save_job = None
        if not self._closing:
            self._save_config()

    # ------------------------------------------------------------------ #
    # 配置文件切换
    # ------------------------------------------------------------------ #
    def select_config(self) -> None:
        """选择另一个配置文件：不同配置拥有独立的快捷指令与发送记录。"""
        path = filedialog.askopenfilename(
            title="选择配置文件",
            initialdir=str(self.config_path.parent),
            filetypes=[("配置文件", "*.json"), ("所有文件", "*.*")],
            parent=self.root,
        )
        if not path:
            return
        target = Path(path)
        try:
            same = target.resolve() == self.config_path.resolve()
        except OSError:
            same = False
        if same:
            self._set_status(f"当前已在用该配置：{target.name}")
            return
        cfg = load_json(target, None)
        if not isinstance(cfg, dict):
            messagebox.showerror(
                "配置文件无效",
                f"无法读取配置文件：\n{target}\n\n请确认它是本工具生成的 JSON 配置。",
                parent=self.root,
            )
            return
        self._flush_config_save()
        self.cfg = cfg
        self.config_path = target
        self._apply_config_values(cfg)
        self._update_config_hint()
        self._schedule_config_save()
        self._set_status(
            f"已切换配置文件：{target.name}（快捷指令 {len(self.quick_commands)} 条）"
        )
        if self.link.is_open:
            self._set_status(
                f"已切换配置文件：{target.name}；串口参数将在下次连接时生效"
            )

    def _apply_config_values(self, cfg: dict) -> None:
        """把配置文件中的参数套用到界面（切换配置时使用）。"""
        self.port_var.set(str(cfg.get("port", "") or ""))
        self.baud_var.set(str(cfg.get("baudrate", "115200")))
        self.bytesize_var.set(str(cfg.get("bytesize", "8")))
        self.parity_var.set(str(cfg.get("parity", "无 (None)")))
        self.stopbits_var.set(str(cfg.get("stopbits", "1")))
        self.flow_var.set(str(cfg.get("flow", "无")))
        self.encoding_var.set(str(cfg.get("encoding", "UTF-8")))
        self.dtr_var.set(bool(cfg.get("dtr", True)))
        self.rts_var.set(bool(cfg.get("rts", True)))
        self.hex_send_var.set(bool(cfg.get("hex_send", False)))
        self.append_crlf_var.set(bool(cfg.get("append_crlf", False)))
        self.repeat_ms_var.set(str(cfg.get("repeat_ms", "1000")))
        self.hex_view_var.set(bool(cfg.get("hex_view", False)))
        self.show_ts_var.set(bool(cfg.get("show_timestamp", True)))
        self.echo_tx_var.set(bool(cfg.get("echo_tx", True)))
        self.autoscroll_var.set(bool(cfg.get("autoscroll", True)))
        self.log_file_var.set(bool(cfg.get("log_to_file", True)))

        self._stop_repeat()
        self.repeat_var.set(False)

        self.history = [
            self._normalize_history_entry(item)
            for item in cfg.get("history", [])
            if isinstance(item, dict)
        ][:HISTORY_LIMIT]
        self.quick_commands = [
            self._normalize_quick_entry(item)
            for item in cfg.get("quick_commands", [])
            if isinstance(item, dict)
        ]
        self._recall_index = -1
        try:
            self.link.set_encoding(self.encoding_var.get())
        except LookupError:
            self.encoding_var.set("UTF-8")
            self.link.set_encoding("UTF-8")
        self._refresh_ports()
        self._apply_saved_state()
        self._refresh_history_view()
        self._refresh_quick_view()

    # ------------------------------------------------------------------ #
    # 退出
    # ------------------------------------------------------------------ #
    def on_close(self) -> None:
        if self._closing:
            return
        self._closing = True
        if self._save_job is not None:
            try:
                self.root.after_cancel(self._save_job)
            except tk.TclError:
                pass
            self._save_job = None
        self._cancel_repeat()
        self.link.close()
        self._close_log()
        save_json(self.config_path, self._collect_config())
        self._sync_config_pointer()
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def main() -> None:
    if os.name == "nt":
        try:
            import ctypes

            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:  # noqa: BLE001 - 高 DPI 设置失败不影响运行
            pass
    root = tk.Tk()
    UartTool(root)
    root.mainloop()


if __name__ == "__main__":
    main()
