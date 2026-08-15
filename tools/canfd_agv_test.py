#!/usr/bin/env python3
"""AGV电机板CAN FD协议抓包与安全自测工具。

适配当前电脑上的WeActStudio USB2CANFDV1（gs_usb/candleLight FD）。
支持状态监视、安全控制、无电机固件自测，以及指定距离的闭环GOTO命令。
"""

from __future__ import annotations

import argparse
import secrets
import struct
import sys
import time
from dataclasses import dataclass

import libusb_package
import usb.core
import usb.util


USB_VID = 0x1D50
USB_PID = 0x606F
USB_ENDPOINT_OUT = 0x02
USB_ENDPOINT_IN = 0x81
USB_FRAME_SIZE = 76

GS_USB_BREQ_HOST_FORMAT = 0
GS_USB_BREQ_BITTIMING = 1
GS_USB_BREQ_MODE = 2
GS_USB_BREQ_DATA_BITTIMING = 10
GS_CAN_MODE_RESET = 0
GS_CAN_MODE_START = 1
GS_CAN_MODE_LISTEN_ONLY = 1 << 0
GS_CAN_MODE_FD = 1 << 8
GS_CAN_FLAG_FD = 1 << 1
GS_CAN_FLAG_BRS = 1 << 2
GS_HOST_FRAME_ECHO_ID_RX = 0xFFFFFFFF

CANFD_VERSION = 1
CANFD_ID_ESTOP = 0x080
CANFD_ID_CONTROL = 0x091
CANFD_ID_HEARTBEAT = 0x101
CANFD_ID_GOTO_LOCAL = 0x121
CANFD_ID_ACK = 0x181
CANFD_ID_DRIVE = 0x201
CANFD_ID_ODOM = 0x211
CANFD_ID_MOTOR = 0x221
CANFD_ID_SAFETY = 0x231

CONTROL_STOP = 1
CONTROL_CLEAR_FAULT = 2
CONTROL_RESET_ODOM = 3

ACK_RECEIVED = 0
ACK_ACCEPTED = 1
ACK_COMPLETED = 2
ACK_CANCELED = 3
ACK_DUPLICATE = 4
ACK_REJECTED = 5
ACK_FAULT = 6
ERROR_CRC = 7

FAULT_ESTOP = 0x0001
FAULT_IMU = 0x0002
FAULT_MOTION = 0x0004
FAULT_BUMPER = 0x0008
FAULT_COMM_TIMEOUT = 0x0010
FAULT_CAN_LINK = 0x0020

DLC_TO_SIZE = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)
SIZE_TO_DLC = {size: dlc for dlc, size in enumerate(DLC_TO_SIZE)}


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def append_crc(payload: bytearray) -> bytes:
    struct.pack_into("<H", payload, len(payload) - 2, crc16_ccitt(payload[:-2]))
    return bytes(payload)


def make_heartbeat(sequence: int, session: int) -> bytes:
    payload = bytearray(16)
    struct.pack_into("<BBHII", payload, 0, CANFD_VERSION, 0, sequence, session,
                     int(time.monotonic() * 1000) & 0xFFFFFFFF)
    return append_crc(payload)


def make_control(opcode: int, sequence: int, session: int) -> bytes:
    payload = bytearray(16)
    struct.pack_into("<BBHI", payload, 0, CANFD_VERSION, opcode, sequence, session)
    return append_crc(payload)


def make_estop(sequence: int, session: int, reason: int = 1) -> bytes:
    payload = bytearray(16)
    struct.pack_into("<BBHII", payload, 0, CANFD_VERSION, reason, sequence, session,
                     int(time.monotonic() * 1000) & 0xFFFFFFFF)
    return append_crc(payload)


def make_goto(sequence: int, session: int, x_mm: int = 300,
              speed_mm_s: int = 50) -> bytes:
    payload = bytearray(32)
    struct.pack_into("<BBHIiiiH", payload, 0, CANFD_VERSION, 0,
                     sequence, session, x_mm, 0, 0, speed_mm_s)
    return append_crc(payload)


@dataclass
class CanFdFrame:
    can_id: int
    data: bytes
    flags: int


class GsUsbFd:
    def __init__(self) -> None:
        backend = libusb_package.get_libusb1_backend()
        self.device = usb.core.find(idVendor=USB_VID, idProduct=USB_PID, backend=backend)
        if self.device is None:
            raise RuntimeError("未找到USB-CAN FD设备 1D50:606F")
        self.echo_id = 0
        self.claimed = False
        self.pending_rx: list[CanFdFrame] = []

    def _control(self, request: int, data: bytes) -> None:
        written = self.device.ctrl_transfer(0x41, request, 0, 0, data, timeout=1000)
        if written != len(data):
            raise RuntimeError(f"USB控制请求{request}长度异常: {written}/{len(data)}")

    def open(self) -> None:
        try:
            self.device.set_configuration()
        except usb.core.USBError:
            pass
        usb.util.claim_interface(self.device, 0)
        self.claimed = True

        self._control(GS_USB_BREQ_HOST_FORMAT, struct.pack("<I", 0x0000BEEF))
        self._control(GS_USB_BREQ_MODE, struct.pack("<II", GS_CAN_MODE_RESET, 0))

        # 60MHz CAN时钟：500kbps，24TQ，采样点75%，SJW 250ns。
        self._control(GS_USB_BREQ_BITTIMING,
                      struct.pack("<IIIII", 1, 16, 6, 3, 5))
        # 4Mbps，15TQ，采样点73.3%，SJW 66.7ns。
        self._control(GS_USB_BREQ_DATA_BITTIMING,
                      struct.pack("<IIIII", 1, 9, 4, 4, 1))
        self._control(GS_USB_BREQ_MODE,
                      struct.pack("<II", GS_CAN_MODE_START, GS_CAN_MODE_FD))

    def close(self) -> None:
        try:
            self._control(GS_USB_BREQ_MODE, struct.pack("<II", GS_CAN_MODE_RESET, 0))
        except Exception:
            pass
        if self.claimed:
            try:
                usb.util.release_interface(self.device, 0)
            except Exception:
                pass
            self.claimed = False

    def set_listen_only(self, enabled: bool) -> None:
        self._control(GS_USB_BREQ_MODE,
                      struct.pack("<II", GS_CAN_MODE_RESET, 0))
        mode_flags = GS_CAN_MODE_FD
        if enabled:
            mode_flags |= GS_CAN_MODE_LISTEN_ONLY
        self._control(GS_USB_BREQ_MODE,
                      struct.pack("<II", GS_CAN_MODE_START, mode_flags))

    def __enter__(self) -> "GsUsbFd":
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def send(self, can_id: int, data: bytes) -> None:
        if len(data) not in SIZE_TO_DLC:
            raise ValueError(f"非法CAN FD长度: {len(data)}")
        header = struct.pack("<IIBBBB", self.echo_id, can_id,
                             SIZE_TO_DLC[len(data)], 0,
                             GS_CAN_FLAG_FD | GS_CAN_FLAG_BRS, 0)
        packet = header + data.ljust(64, b"\x00")
        try:
            written = self.device.write(USB_ENDPOINT_OUT, packet, timeout=1000)
        except usb.core.USBTimeoutError:
            # candleLight异常退出后偶发保留OUT halt；清端点后只重试一次。
            self.device.clear_halt(USB_ENDPOINT_OUT)
            written = self.device.write(USB_ENDPOINT_OUT, packet, timeout=1000)
        if written != USB_FRAME_SIZE:
            raise RuntimeError(f"USB发送长度异常: {written}/{USB_FRAME_SIZE}")
        self.echo_id = (self.echo_id + 1) % 10

    def _read_usb_packet(self, timeout_ms: int) -> tuple[int, CanFdFrame | None]:
        packet = bytes(self.device.read(USB_ENDPOINT_IN, USB_FRAME_SIZE,
                                        timeout=timeout_ms))
        if len(packet) < 12:
            return -1, None
        echo_id, can_id, dlc, channel, flags, _ = struct.unpack_from("<IIBBBB", packet)
        if (echo_id != GS_HOST_FRAME_ECHO_ID_RX or channel != 0 or
                dlc >= len(DLC_TO_SIZE)):
            return echo_id, None
        size = DLC_TO_SIZE[dlc]
        return echo_id, CanFdFrame(can_id & 0x1FFFFFFF,
                                   packet[12:12 + size], flags)

    def receive(self, timeout_ms: int = 100) -> CanFdFrame | None:
        if self.pending_rx:
            return self.pending_rx.pop(0)
        try:
            _, frame = self._read_usb_packet(timeout_ms)
        except usb.core.USBTimeoutError:
            return None
        return frame


def valid_protocol_crc(frame: CanFdFrame) -> bool:
    return len(frame.data) >= 2 and crc16_ccitt(frame.data[:-2]) == struct.unpack_from("<H", frame.data, len(frame.data) - 2)[0]


def decode_frame(frame: CanFdFrame) -> str:
    prefix = f"ID=0x{frame.can_id:03X} LEN={len(frame.data)} CRC={'OK' if valid_protocol_crc(frame) else 'BAD'}"
    if frame.can_id == CANFD_ID_ACK and len(frame.data) == 16:
        version, status, sequence, device_id = struct.unpack_from("<BBHI", frame.data)
        command, error, fault = struct.unpack_from("<BBH", frame.data, 8)
        state, node = struct.unpack_from("<BB", frame.data, 12)
        return f"{prefix} ACK ver={version} seq={sequence} status={status} cmd={command} error={error} fault=0x{fault:04X} state={state} node={node} dev=0x{device_id:08X}"
    if frame.can_id == CANFD_ID_SAFETY and len(frame.data) == 24:
        version, state, sequence, uptime = struct.unpack_from("<BBHI", frame.data)
        raw, active, latched, sources = struct.unpack_from("<BBBB", frame.data, 8)
        fault, active_sequence, link_age = struct.unpack_from("<HHH", frame.data, 12)
        bus_state, drops = struct.unpack_from("<BB", frame.data, 18)
        return f"{prefix} SAFETY ver={version} seq={sequence} uptime={uptime} state={state} raw=0x{raw:02X} active=0x{active:02X} latched=0x{latched:02X} sources=0x{sources:02X} fault=0x{fault:04X} active_seq={active_sequence} link_age={link_age} bus=0x{bus_state:02X} drops={drops}"
    if frame.can_id == CANFD_ID_DRIVE and len(frame.data) == 24:
        _, state, sequence, uptime = struct.unpack_from("<BBHI", frame.data)
        active_sequence, fault = struct.unpack_from("<HH", frame.data, 8)
        target_left, target_right, measured_left, measured_right = \
            struct.unpack_from("<hhhh", frame.data, 12)
        source, bumper = struct.unpack_from("<BB", frame.data, 20)
        return (f"{prefix} DRIVE seq={sequence} uptime={uptime} state={state} "
                f"active_seq={active_sequence} fault=0x{fault:04X} "
                f"target_rpm={target_left / 100:.2f}/{target_right / 100:.2f} "
                f"measured_rpm={measured_left / 100:.2f}/{measured_right / 100:.2f} "
                f"source={source} bumper=0x{bumper:02X}")
    if frame.can_id == CANFD_ID_ODOM and len(frame.data) == 32:
        _, state, sequence, uptime = struct.unpack_from("<BBHI", frame.data)
        x_mm, y_mm, yaw_cdeg = struct.unpack_from("<iii", frame.data, 8)
        linear_mm_s, yaw_rate_cdeg_s = struct.unpack_from("<hh", frame.data, 20)
        quality = frame.data[24]
        return (f"{prefix} ODOM seq={sequence} uptime={uptime} state={state} "
                f"x={x_mm}mm y={y_mm}mm yaw={yaw_cdeg / 100:.2f}deg "
                f"linear={linear_mm_s}mm/s yaw_rate={yaw_rate_cdeg_s / 100:.2f}deg/s "
                f"quality={quality}")
    return f"{prefix} DATA={frame.data.hex()}"


def wait_for(bus: GsUsbFd, predicate, timeout_s: float, show: bool = True) -> CanFdFrame | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = bus.receive(100)
        if frame is None:
            continue
        if show:
            print(decode_frame(frame))
        if predicate(frame):
            return frame
    return None


def is_ack(frame: CanFdFrame, sequence: int, status: int | None = None,
           error: int | None = None) -> bool:
    if frame.can_id != CANFD_ID_ACK or len(frame.data) != 16 or not valid_protocol_crc(frame):
        return False
    frame_status = frame.data[1]
    frame_sequence = struct.unpack_from("<H", frame.data, 2)[0]
    frame_error = frame.data[9]
    return frame_sequence == sequence and (status is None or frame_status == status) and (error is None or frame_error == error)


def safety_fault(frame: CanFdFrame) -> int | None:
    if frame.can_id != CANFD_ID_SAFETY or len(frame.data) != 24 or not valid_protocol_crc(frame):
        return None
    return struct.unpack_from("<H", frame.data, 12)[0]


def ack_status(frame: CanFdFrame, sequence: int) -> int | None:
    if not is_ack(frame, sequence):
        return None
    return frame.data[1]


def run_goto(bus: GsUsbFd, distance_mm: int, speed_mm_s: int,
             timeout_s: float, clear_faults: bool) -> int:
    if not 300 <= distance_mm <= 3000:
        print("ERROR,GOTO_RANGE,distance_mm必须为300..3000", file=sys.stderr)
        return 2
    if not 50 <= speed_mm_s <= 100:
        print("ERROR,GOTO_RANGE,speed_mm_s必须为50..100", file=sys.stderr)
        return 2

    session = secrets.randbits(32) or 1
    heartbeat_sequence = 1
    next_heartbeat = 0.0
    last_report: dict[int, float] = {}
    last_safety_key = None

    def show_live_frame(frame: CanFdFrame, now: float) -> None:
        nonlocal last_safety_key

        if frame.can_id == CANFD_ID_ACK:
            print(decode_frame(frame), flush=True)
            return
        if frame.can_id == CANFD_ID_SAFETY and len(frame.data) == 24:
            state = frame.data[1]
            raw, active, latched, sources = struct.unpack_from(
                "<BBBB", frame.data, 8)
            fault = struct.unpack_from("<H", frame.data, 12)[0]
            key = (state, raw, active, latched, sources, fault)
            if key != last_safety_key or now - last_report.get(frame.can_id, 0.0) >= 2.0:
                print(decode_frame(frame), flush=True)
                last_safety_key = key
                last_report[frame.can_id] = now
            return
        period = 0.5 if frame.can_id == CANFD_ID_ODOM else 1.0
        if (frame.can_id in (CANFD_ID_ODOM, CANFD_ID_DRIVE) and
                now - last_report.get(frame.can_id, 0.0) >= period):
            print(decode_frame(frame), flush=True)
            last_report[frame.can_id] = now

    def pump_until(predicate, wait_s: float) -> CanFdFrame | None:
        nonlocal heartbeat_sequence, next_heartbeat

        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                bus.send(CANFD_ID_HEARTBEAT,
                         make_heartbeat(heartbeat_sequence, session))
                heartbeat_sequence = (heartbeat_sequence + 1) & 0xFFFF or 1
                next_heartbeat = now + 0.1
            frame = bus.receive(20)
            if frame is None:
                continue
            show_live_frame(frame, now)
            if predicate(frame):
                return frame
        return None

    def send_stop(sequence: int) -> None:
        print(f"GOTO,SAFETY_STOP,seq={sequence}", flush=True)
        bus.send(CANFD_ID_CONTROL,
                 make_control(CONTROL_STOP, sequence, session))
        _ = pump_until(
            lambda item: ack_status(item, sequence) == ACK_ACCEPTED,
            1.0)

    print(f"GOTO,CONNECT,session=0x{session:08X}", flush=True)
    safety = pump_until(
        lambda item: item.can_id == CANFD_ID_SAFETY and
        len(item.data) == 24 and valid_protocol_crc(item),
        2.0)
    if safety is None:
        print("ERROR,GOTO_NO_SAFETY_STATUS", file=sys.stderr)
        return 1

    fault = safety_fault(safety) or 0
    active_bumper = safety.data[9]
    latched_bumper = safety.data[10]
    if active_bumper != 0:
        print(f"ERROR,GOTO_BUMPER_ACTIVE,mask=0x{active_bumper:02X}",
              file=sys.stderr)
        return 1
    if latched_bumper != 0 and not clear_faults:
        print(f"ERROR,GOTO_BUMPER_LATCHED,mask=0x{latched_bumper:02X},"
              "需要确认现场后增加--clear-faults", file=sys.stderr)
        return 1

    if fault != 0:
        can_auto_clear = fault == FAULT_CAN_LINK and latched_bumper == 0
        if not can_auto_clear and not clear_faults:
            print(f"ERROR,GOTO_FAULT_LOCKED,fault=0x{fault:04X},"
                  "需要排除故障或显式增加--clear-faults", file=sys.stderr)
            return 1

        clear_sequence = 100
        print(f"GOTO,CLEAR_FAULT_REQUEST,fault=0x{fault:04X},"
              f"seq={clear_sequence}", flush=True)
        bus.send(CANFD_ID_CONTROL,
                 make_control(CONTROL_CLEAR_FAULT, clear_sequence, session))
        clear_ack = pump_until(
            lambda item: ack_status(item, clear_sequence) in
            (ACK_ACCEPTED, ACK_REJECTED, ACK_FAULT),
            2.0)
        if clear_ack is None or clear_ack.data[1] != ACK_ACCEPTED:
            print("ERROR,GOTO_CLEAR_FAULT_REJECTED", file=sys.stderr)
            return 1
        cleared = pump_until(lambda item: safety_fault(item) == 0, 2.0)
        if cleared is None:
            print("ERROR,GOTO_FAULT_NOT_CLEARED", file=sys.stderr)
            return 1

    goto_sequence = 101
    if timeout_s <= 0.0:
        timeout_s = max(30.0, distance_mm / speed_mm_s + 20.0)
    print(f"GOTO,SEND,distance_mm={distance_mm},speed_mm_s={speed_mm_s},"
          f"seq={goto_sequence},timeout_s={timeout_s:.1f}", flush=True)
    bus.send(CANFD_ID_GOTO_LOCAL,
             make_goto(goto_sequence, session, distance_mm, speed_mm_s))

    accepted = pump_until(
        lambda item: ack_status(item, goto_sequence) in
        (ACK_ACCEPTED, ACK_REJECTED, ACK_FAULT),
        2.0)
    if accepted is None or accepted.data[1] != ACK_ACCEPTED:
        print("ERROR,GOTO_NOT_ACCEPTED", file=sys.stderr)
        send_stop(102)
        return 1
    print("GOTO,ACCEPTED", flush=True)

    try:
        terminal = pump_until(
            lambda item: (
                ack_status(item, goto_sequence) in
                (ACK_COMPLETED, ACK_CANCELED, ACK_FAULT) or
                ((safety_fault(item) or 0) != 0)),
            timeout_s)
    except KeyboardInterrupt:
        send_stop(102)
        print("GOTO,INTERRUPTED,STOP_SENT", file=sys.stderr)
        return 130

    if terminal is None:
        send_stop(102)
        print("ERROR,GOTO_HOST_TIMEOUT,STOP_SENT", file=sys.stderr)
        return 1
    if ack_status(terminal, goto_sequence) == ACK_COMPLETED:
        print("GOTO,COMPLETED", flush=True)
        return 0

    send_stop(102)
    if terminal.can_id == CANFD_ID_SAFETY:
        print(f"ERROR,GOTO_FAULT,fault=0x{(safety_fault(terminal) or 0):04X}",
              file=sys.stderr)
    else:
        print(f"ERROR,GOTO_TERMINATED,status={terminal.data[1]}",
              file=sys.stderr)
    return 1


def run_selftest(bus: GsUsbFd) -> int:
    session = secrets.randbits(32) or 1
    passed = 0
    failed = 0

    def result(name: str, ok: bool) -> None:
        nonlocal passed, failed
        print(f"TEST,{name},{'PASS' if ok else 'FAIL'}")
        passed += int(ok)
        failed += int(not ok)

    for sequence in range(1, 13):
        bus.send(CANFD_ID_HEARTBEAT, make_heartbeat(sequence, session))
        # gs_usb的发送echo与板端上报共用IN端点，心跳突发期间也要持续排空。
        _ = bus.receive(20)
        time.sleep(0.08)

    frame = wait_for(bus, lambda item: item.can_id == CANFD_ID_SAFETY and valid_protocol_crc(item), 1.0)
    result("SAFETY_STATUS_RX", frame is not None)
    result("SAFETY_FD_BRS", frame is not None and
           (frame.flags & (GS_CAN_FLAG_FD | GS_CAN_FLAG_BRS)) ==
           (GS_CAN_FLAG_FD | GS_CAN_FLAG_BRS))

    bus.send(CANFD_ID_CONTROL, make_control(CONTROL_CLEAR_FAULT, 20, session))
    frame = wait_for(bus, lambda item: is_ack(item, 20, ACK_ACCEPTED), 1.0)
    result("INITIAL_CLEAR_FAULT", frame is not None)

    bus.send(CANFD_ID_CONTROL, make_control(CONTROL_STOP, 21, session))
    frame = wait_for(bus, lambda item: is_ack(item, 21, ACK_ACCEPTED), 1.0)
    result("STOP_ACK", frame is not None)

    corrupted = bytearray(make_control(CONTROL_STOP, 22, session))
    corrupted[8] ^= 0x5A
    bus.send(CANFD_ID_CONTROL, bytes(corrupted))
    frame = wait_for(bus, lambda item: is_ack(item, 22, ACK_REJECTED, ERROR_CRC), 1.0)
    result("CRC_REJECT", frame is not None)

    bus.send(CANFD_ID_ESTOP, make_estop(23, session))
    frame = wait_for(bus, lambda item: is_ack(item, 23, ACK_ACCEPTED), 1.0)
    result("ESTOP_ACK", frame is not None)
    frame = wait_for(bus, lambda item: (safety_fault(item) or 0) & 0x0001, 1.0)
    result("ESTOP_EVENT", frame is not None)

    time.sleep(0.12)
    bus.send(CANFD_ID_HEARTBEAT, make_heartbeat(24, session))
    bus.send(CANFD_ID_CONTROL, make_control(CONTROL_CLEAR_FAULT, 25, session))
    frame = wait_for(bus, lambda item: is_ack(item, 25, ACK_ACCEPTED), 1.0)
    result("CLEAR_FAULT_ACK", frame is not None)
    frame = wait_for(bus, lambda item: safety_fault(item) == 0, 1.0)
    result("FAULT_CLEARED_EVENT", frame is not None)

    bus.send(CANFD_ID_GOTO_LOCAL, make_goto(30, session))
    frame = wait_for(bus, lambda item: is_ack(item, 30, ACK_ACCEPTED), 1.0)
    result("SIMULATED_MOTION_ACK", frame is not None)
    time.sleep(0.35)
    frame = wait_for(bus, lambda item: (safety_fault(item) or 0) & 0x0010, 1.0)
    result("HEARTBEAT_TIMEOUT_EVENT", frame is not None)

    bus.send(CANFD_ID_HEARTBEAT, make_heartbeat(31, session))
    time.sleep(0.12)
    bus.send(CANFD_ID_CONTROL, make_control(CONTROL_CLEAR_FAULT, 31, session))
    frame = wait_for(bus, lambda item: is_ack(item, 31, ACK_ACCEPTED), 1.0)
    result("TIMEOUT_CLEAR_ACK", frame is not None)
    frame = wait_for(bus, lambda item: safety_fault(item) == 0, 1.0)
    result("TIMEOUT_CLEARED_EVENT", frame is not None)

    print("TEST,BUS_OFF_INJECTION,START")
    bus.set_listen_only(True)
    injection_deadline = time.monotonic() + 2.0
    while time.monotonic() < injection_deadline:
        _ = bus.receive(50)
    bus.close()
    time.sleep(0.2)
    bus.open()
    frame = wait_for(bus, lambda item: (safety_fault(item) or 0) & 0x0020,
                     4.0, show=False)
    if frame is not None:
        print(decode_frame(frame))
    result("CAN_LINK_FAULT_EVENT", frame is not None)

    bus.send(CANFD_ID_HEARTBEAT, make_heartbeat(40, session))
    time.sleep(0.12)
    bus.send(CANFD_ID_CONTROL, make_control(CONTROL_CLEAR_FAULT, 41, session))
    frame = wait_for(bus, lambda item: is_ack(item, 41, ACK_ACCEPTED), 1.0)
    result("CAN_LINK_CLEAR_ACK", frame is not None)
    frame = wait_for(bus, lambda item: safety_fault(item) == 0, 1.0)
    result("CAN_LINK_CLEARED_EVENT", frame is not None)

    print(f"SUMMARY,PASS={passed},FAIL={failed}")
    return 0 if failed == 0 else 1


def run_monitor(bus: GsUsbFd, seconds: float, send_heartbeat: bool) -> int:
    session = secrets.randbits(32) or 1
    sequence = 1
    deadline = time.monotonic() + seconds
    next_heartbeat = 0.0
    last_safety_key = None
    last_report = 0.0
    recoveries = 0
    while time.monotonic() < deadline:
        now = time.monotonic()
        try:
            if send_heartbeat and now >= next_heartbeat:
                bus.send(CANFD_ID_HEARTBEAT, make_heartbeat(sequence, session))
                sequence = (sequence + 1) & 0xFFFF or 1
                next_heartbeat = now + 0.1
            frame = bus.receive(50)
        except usb.core.USBError as exc:
            recoveries += 1
            print(f"WARN,USB_RECOVERY,{recoveries},{exc}", flush=True)
            if recoveries > 3:
                return 2
            bus.close()
            time.sleep(0.2)
            bus.open()
            next_heartbeat = 0.0
            continue
        if frame is not None:
            if frame.can_id == CANFD_ID_SAFETY and len(frame.data) == 24:
                state = frame.data[1]
                raw, active, latched, sources = struct.unpack_from("<BBBB", frame.data, 8)
                fault = struct.unpack_from("<H", frame.data, 12)[0]
                safety_key = (state, raw, active, latched, sources, fault)
                if safety_key == last_safety_key and now - last_report < 5.0:
                    continue
                last_safety_key = safety_key
                last_report = now
            print(decode_frame(frame), flush=True)
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=(
        "goto", "monitor", "stop", "estop", "clear", "selftest"))
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--heartbeat", action="store_true")
    parser.add_argument("--distance-mm", type=int, default=500,
                        help="goto前进距离，范围300..3000mm，默认500")
    parser.add_argument("--speed-mm-s", type=int, default=50,
                        help="goto速度，范围50..100mm/s，默认50")
    parser.add_argument("--timeout-s", type=float, default=0.0,
                        help="goto主机等待超时；0表示按距离自动计算")
    parser.add_argument("--clear-faults", action="store_true",
                        help="确认现场安全后，允许goto尝试清除非CAN链路故障")
    parser.add_argument("--confirm-no-motor-test-firmware", action="store_true",
                        help="仅在板端烧录无电机测试固件时允许selftest")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session = secrets.randbits(32) or 1
    try:
        with GsUsbFd() as bus:
            if args.action == "selftest":
                if not args.confirm_no_motor_test_firmware:
                    print("ERROR,SELFTEST_REQUIRES_NO_MOTOR_TEST_FIRMWARE_CONFIRMATION",
                          file=sys.stderr)
                    return 2
                return run_selftest(bus)
            if args.action == "goto":
                return run_goto(bus, args.distance_mm, args.speed_mm_s,
                                args.timeout_s, args.clear_faults)
            if args.action == "monitor":
                return run_monitor(bus, args.seconds, args.heartbeat)

            bus.send(CANFD_ID_HEARTBEAT, make_heartbeat(1, session))
            time.sleep(0.05)
            if args.action == "stop":
                bus.send(CANFD_ID_CONTROL, make_control(CONTROL_STOP, 2, session))
            elif args.action == "estop":
                bus.send(CANFD_ID_ESTOP, make_estop(2, session))
            else:
                time.sleep(0.1)
                bus.send(CANFD_ID_CONTROL, make_control(CONTROL_CLEAR_FAULT, 2, session))
            return run_monitor(bus, 2.0, False)
    except Exception as exc:
        print(f"ERROR,{type(exc).__name__},{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
