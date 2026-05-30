"""
ds2502.py - High-level DS2502 device model layered on SerialBridge.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Optional
from serial_bridge import SerialBridge, SerialBridgeError

DATA_SIZE = 128
PAGE_SIZE = 32
PAGE_COUNT = DATA_SIZE // PAGE_SIZE
STATUS_SIZE = 8
FAMILY_CODE = 0x09
BLANK_BYTE = 0xFF

STATUS_FIELDS = [
    ("0x00", "Write Protect - Pages 0..3 / Status"),
    ("0x01", "Page 0 Redirection Byte"),
    ("0x02", "Page 1 Redirection Byte"),
    ("0x03", "Page 2 Redirection Byte"),
    ("0x04", "Page 3 Redirection Byte"),
    ("0x05", "Factory / Reserved"),
    ("0x06", "Factory / Reserved"),
    ("0x07", "Factory / Reserved"),
]


class DS2502Error(Exception):
    pass


@dataclass
class DeviceInfo:
    rom: bytes = b""
    crc_ok: bool = False

    @property
    def present(self) -> bool:
        return len(self.rom) == 8

    @property
    def family(self) -> Optional[int]:
        return self.rom[0] if self.present else None

    @property
    def serial(self) -> str:
        if not self.present:
            return ""
        return "".join(f"{b:02X}" for b in self.rom[1:7])

    def rom_hex(self) -> str:
        return "".join(f"{b:02X}" for b in self.rom)

    def is_ds2502(self) -> bool:
        return self.present and self.rom[0] == FAMILY_CODE


@dataclass
class WriteResult:
    address: int
    requested: bytes
    readback: bytes
    crc_ok: bool
    pf_ok: bool = True      # Program Flag: True = all bytes programmed successfully
    pf_flags: str = ""      # Per-byte PF string (e.g. "0000" = all good)

    @property
    def verified(self) -> bool:
        return self.readback == self.requested and self.crc_ok and self.pf_ok

    def mismatches(self) -> List[int]:
        return [i for i in range(len(self.requested))
                if i < len(self.readback) and self.readback[i] != self.requested[i]]

    def pf_failures(self) -> List[int]:
        """Indices where Program Flag indicated failure (pulse not accepted)."""
        return [i for i, ch in enumerate(self.pf_flags) if ch != '0']


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        for _ in range(8):
            mix = (crc ^ b) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            b >>= 1
    return crc & 0xFF


class DS2502:
    def __init__(self, bridge: SerialBridge) -> None:
        self.bridge = bridge

    @staticmethod
    def _expect_ok(resp: str) -> str:
        if resp is None:
            raise DS2502Error("No response from bridge")
        if resp.startswith("ERR"):
            raise DS2502Error(resp[3:].strip() or "device error")
        if not resp.startswith("OK"):
            raise DS2502Error(f"Unexpected response: {resp!r}")
        return resp

    @staticmethod
    def _parse_kv(resp: str) -> Dict[str, str]:
        out: Dict[str, str] = {}
        for tok in resp.split():
            if "=" in tok:
                k, v = tok.split("=", 1)
                out[k] = v
        return out

    @staticmethod
    def _hex_to_bytes(s: str) -> bytes:
        s = s.strip()
        if len(s) % 2:
            raise DS2502Error(f"Odd-length hex payload: {s!r}")
        return bytes.fromhex(s) if s else b""

    def ping(self) -> bool:
        try:
            return self.bridge.command("PING", timeout=2.0).startswith("OK")
        except SerialBridgeError:
            return False

    def info(self) -> Dict[str, str]:
        resp = self._expect_ok(self.bridge.command("INFO"))
        return self._parse_kv(resp)

    def set_use_rom(self, use: bool) -> None:
        self._expect_ok(self.bridge.command(f"USEROM {1 if use else 0}"))

    def set_vpp(self, on: bool) -> None:
        self._expect_ok(self.bridge.command(f"VPP {1 if on else 0}"))

    def reset(self) -> bool:
        resp = self._expect_ok(self.bridge.command("RESET"))
        return resp.split()[-1] == "1"

    def read_rom(self) -> DeviceInfo:
        resp = self._expect_ok(self.bridge.command("READROM"))
        parts = resp.split()
        rom_hex = parts[2] if len(parts) > 2 else ""
        kv = self._parse_kv(resp)
        return DeviceInfo(rom=self._hex_to_bytes(rom_hex), crc_ok=(kv.get("crc") == "1"))

    def search(self) -> List[bytes]:
        lines = self.bridge.command_multi("SEARCH")
        roms: List[bytes] = []
        for ln in lines:
            if ln.startswith("DEV "):
                roms.append(self._hex_to_bytes(ln[4:].strip()))
            elif ln.startswith("ERR"):
                raise DS2502Error(ln[3:].strip())
        return roms

    def _read_block(self, cmd: str, addr: int, length: int):
        resp = self._expect_ok(self.bridge.command(f"{cmd} {addr:X} {length}", timeout=6.0))
        kv = self._parse_kv(resp)
        tokens = resp.split()
        try:
            di = tokens.index("DATA")
            payload = tokens[di + 1]
        except (ValueError, IndexError):
            raise DS2502Error(f"Malformed read response: {resp!r}")
        data = self._hex_to_bytes(payload)
        # Combined CRC flag (cmd CRC + page CRC both OK)
        crc_ok = (kv.get("crc") == "1")
        # Detailed: cmdcrc and pagecrc available in fw 2.0+
        cmd_crc_ok = (kv.get("cmdcrc", "1") == "1")
        page_crc_ok = (kv.get("pagecrc", "1") == "1")
        return data, crc_ok, cmd_crc_ok, page_crc_ok

    def read_memory(self, addr: int = 0, length: int = DATA_SIZE):
        """Read data EEPROM. Returns (data, crc_ok, cmd_crc_ok, page_crc_ok)."""
        return self._read_block("RDMEM", addr, length)

    def read_status(self, addr: int = 0, length: int = STATUS_SIZE):
        """Read status memory. Returns (data, crc_ok, cmd_crc_ok, page_crc_ok)."""
        return self._read_block("RDSTAT", addr, length)

    def _write_block(self, cmd: str, addr: int, data: bytes) -> WriteResult:
        if not data:
            raise DS2502Error("Nothing to write")
        hexstr = data.hex().upper()
        resp = self._expect_ok(self.bridge.command(f"{cmd} {addr:X} {hexstr}", timeout=15.0))
        kv = self._parse_kv(resp)
        readback = self._hex_to_bytes(kv.get("rb", ""))
        pf_ok = (kv.get("pfok", "1") == "1")
        pf_flags = kv.get("pf", "")
        return WriteResult(
            address=addr, requested=data, readback=readback,
            crc_ok=(kv.get("crcok") == "1"),
            pf_ok=pf_ok,
            pf_flags=pf_flags,
        )

    def write_memory(self, addr: int, data: bytes) -> WriteResult:
        return self._write_block("WRMEM", addr, data)

    def write_status(self, addr: int, data: bytes) -> WriteResult:
        return self._write_block("WRSTAT", addr, data)

    @staticmethod
    def can_program(current: bytes, desired: bytes) -> List[int]:
        return [i for i, (c, d) in enumerate(zip(current, desired)) if (d & c) != d]
