"""
serial_bridge.py - Thread-safe serial transport for the ESP32 DS2502 bridge.
"""
from __future__ import annotations
import re
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

try:
    import serial
    from serial.tools import list_ports
    _HAVE_PYSERIAL = True
except Exception:
    serial = None
    list_ports = None
    _HAVE_PYSERIAL = False


class SerialBridgeError(Exception):
    pass


@dataclass
class PortInfo:
    device: str
    description: str
    def label(self) -> str:
        if self.description and self.description != "n/a":
            return f"{self.device} - {self.description}"
        return self.device


class SerialBridge:
    DEFAULT_BAUD = 115200

    def __init__(self) -> None:
        self._ser = None
        self._lock = threading.RLock()
        self.port: Optional[str] = None
        self.baud: int = self.DEFAULT_BAUD

    @staticmethod
    def available() -> bool:
        return _HAVE_PYSERIAL

    @staticmethod
    def list_ports() -> List[PortInfo]:
        if not _HAVE_PYSERIAL:
            return []
        ports: List[PortInfo] = []
        for p in list_ports.comports():
            ports.append(PortInfo(device=p.device, description=p.description or "n/a"))
        ports.sort(key=lambda x: [int(t) if t.isdigit() else t.lower()
                                  for t in re.split(r"(\d+)", x.device)])
        return ports

    @property
    def is_open(self) -> bool:
        return self._ser is not None and getattr(self._ser, "is_open", False)

    def connect(self, port: str, baud: int = DEFAULT_BAUD, timeout: float = 2.0) -> None:
        if not _HAVE_PYSERIAL:
            raise SerialBridgeError("pyserial is not installed. Run: pip install pyserial")
        with self._lock:
            self.disconnect()
            try:
                self._ser = serial.Serial(
                    port=port, baudrate=baud,
                    bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE, timeout=timeout, write_timeout=timeout)
            except Exception as exc:
                self._ser = None
                raise SerialBridgeError(f"Could not open {port}: {exc}") from exc
            self.port = port
            self.baud = baud
            time.sleep(1.8)
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()

    def disconnect(self) -> None:
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
            self._ser = None

    def _write_line(self, line: str) -> None:
        assert self._ser is not None
        self._ser.write((line.strip() + "\n").encode("ascii", errors="ignore"))
        self._ser.flush()

    def _read_line(self, timeout: float) -> str:
        assert self._ser is not None
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self._ser.read(1)
            if not chunk:
                continue
            if chunk in (b"\n", b"\r"):
                if buf:
                    return buf.decode("ascii", errors="replace").strip()
                continue
            buf.extend(chunk)
        if buf:
            return buf.decode("ascii", errors="replace").strip()
        raise SerialBridgeError("Timed out waiting for a response")

    def command(self, line: str, timeout: float = 5.0) -> str:
        with self._lock:
            if not self.is_open:
                raise SerialBridgeError("Not connected")
            self._ser.reset_input_buffer()
            self._write_line(line)
            return self._read_line(timeout)

    def command_multi(self, line: str, terminator: str = "END",
                      timeout: float = 8.0) -> List[str]:
        with self._lock:
            if not self.is_open:
                raise SerialBridgeError("Not connected")
            self._ser.reset_input_buffer()
            self._write_line(line)
            lines: List[str] = []
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    rl = self._read_line(timeout=max(0.2, deadline - time.monotonic()))
                except SerialBridgeError:
                    break
                if rl == terminator:
                    break
                lines.append(rl)
            return lines
