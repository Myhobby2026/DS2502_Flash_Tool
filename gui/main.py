#!/usr/bin/env python3
"""
DS2502 Flash Tool - Advanced Tkinter GUI for Windows.
Uses ESP32-S as USB-serial bridge to read/write DS2502 1-Wire EPROM.
"""
from __future__ import annotations
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from ds2502 import (DATA_SIZE, PAGE_COUNT, PAGE_SIZE, STATUS_FIELDS,
                    STATUS_SIZE, DeviceInfo, DS2502, DS2502Error)
from hex_editor import HexEditor
from serial_bridge import SerialBridge, SerialBridgeError

APP_TITLE = "DS2502 Flash Tool"
APP_VERSION = "1.0.0"


def _group_runs(indices):
    runs = []
    if not indices:
        return runs
    indices = sorted(indices)
    start = prev = indices[0]
    for i in indices[1:]:
        if i == prev + 1:
            prev = i
        else:
            runs.append((start, prev - start + 1))
            start = prev = i
    runs.append((start, prev - start + 1))
    return runs


class App(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title(f"{APP_TITLE} v{APP_VERSION}")
        self.geometry("1000x720")
        self.minsize(900, 640)
        self.bridge = SerialBridge()
        self.dev = DS2502(self.bridge)
        self.device_info = DeviceInfo()
        self.last_eeprom = bytes([0xFF] * DATA_SIZE)
        self.last_status = bytes([0xFF] * STATUS_SIZE)
        self._busy = False
        self._task_queue: "queue.Queue" = queue.Queue()
        self._op_buttons: list = []
        self._build_style()
        self._build_ui()
        self._refresh_ports()
        self._set_connected(False)
        self.after(50, self._poll_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        if not SerialBridge.available():
            self.log("WARNING: pyserial not installed. Run: pip install pyserial", "warn")

    def _build_style(self) -> None:
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Accent.TButton", font=("TkDefaultFont", 9, "bold"))
        style.configure("Danger.TButton", foreground="#b00020", font=("TkDefaultFont", 9, "bold"))
        style.configure("Status.TLabel", font=("TkDefaultFont", 9))
        style.configure("Info.TLabel", font=("TkFixedFont", 9))

    def _build_ui(self) -> None:
        self._build_connection_bar()
        self._build_device_panel()
        self._build_toolbar()
        self._build_notebook()
        self._build_statusbar()

    def _build_connection_bar(self) -> None:
        bar = ttk.LabelFrame(self, text="Connection")
        bar.pack(fill="x", padx=8, pady=(8, 4))
        ttk.Label(bar, text="Port:").grid(row=0, column=0, padx=(8, 2), pady=6)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(bar, textvariable=self.port_var, width=34, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=2, pady=6)
        ttk.Button(bar, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=4)
        ttk.Label(bar, text="Baud:").grid(row=0, column=3, padx=(12, 2))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(bar, textvariable=self.baud_var, width=8, state="readonly",
                     values=("9600", "57600", "115200", "230400", "460800")).grid(row=0, column=4, padx=2)
        self.connect_btn = ttk.Button(bar, text="Connect", style="Accent.TButton",
                                      command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=(12, 8))
        self.conn_indicator = tk.Canvas(bar, width=16, height=16, highlightthickness=0)
        self.conn_indicator.grid(row=0, column=6, padx=(0, 8))
        self._draw_indicator(False)

    def _draw_indicator(self, on: bool) -> None:
        self.conn_indicator.delete("all")
        color = "#2ecc71" if on else "#e74c3c"
        self.conn_indicator.create_oval(2, 2, 14, 14, fill=color, outline="")

    def _build_device_panel(self) -> None:
        panel = ttk.LabelFrame(self, text="Device")
        panel.pack(fill="x", padx=8, pady=4)
        self.rom_var = tk.StringVar(value="--")
        self.family_var = tk.StringVar(value="--")
        self.serial_var = tk.StringVar(value="--")
        self.crc_var = tk.StringVar(value="--")

        def field(parent, label, var, col):
            ttk.Label(parent, text=label).grid(row=0, column=col*2, sticky="e", padx=(10, 2), pady=6)
            ttk.Label(parent, textvariable=var, style="Info.TLabel").grid(
                row=0, column=col*2+1, sticky="w", padx=(0, 10))

        field(panel, "ROM ID:", self.rom_var, 0)
        field(panel, "Family:", self.family_var, 1)
        field(panel, "Serial:", self.serial_var, 2)
        field(panel, "ROM CRC:", self.crc_var, 3)

        opts = ttk.Frame(panel)
        opts.grid(row=1, column=0, columnspan=8, sticky="w", padx=8, pady=(0, 6))
        self.use_rom_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(opts, text="Match ROM (multi-drop bus)",
                        variable=self.use_rom_var, command=self._on_use_rom).pack(side="left", padx=(0, 16))
        self.vpp_arm_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(opts, text="Arm 12V Vpp (enable programming)",
                        variable=self.vpp_arm_var, command=self._on_vpp_arm).pack(side="left", padx=(0, 16))
        ttk.Button(opts, text="Read ROM / Detect", command=self._detect).pack(side="left", padx=4)
        ttk.Button(opts, text="Scan Bus", command=self._scan_bus).pack(side="left", padx=4)

    def _build_toolbar(self) -> None:
        tb = ttk.Frame(self)
        tb.pack(fill="x", padx=8, pady=4)

        def add(text, cmd, style=None):
            b = ttk.Button(tb, text=text, command=cmd, style=style or "TButton")
            b.pack(side="left", padx=3)
            self._op_buttons.append(b)
            return b

        add("Read EEPROM", self._read_eeprom)
        add("Write EEPROM", self._write_eeprom, "Accent.TButton")
        add("Verify", self._verify_eeprom)
        ttk.Separator(tb, orient="vertical").pack(side="left", fill="y", padx=6)
        add("Read Status", self._read_status)
        add("Write Status", self._write_status, "Accent.TButton")
        ttk.Separator(tb, orient="vertical").pack(side="left", fill="y", padx=6)
        add("Load .bin", self._load_file)
        add("Save .bin", self._save_file)
        add("Fill 0xFF", self._fill_blank)

    def _build_notebook(self) -> None:
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=8, pady=4)

        eeprom_tab = ttk.Frame(nb)
        nb.add(eeprom_tab, text=f"EEPROM Data ({DATA_SIZE} bytes)")
        ttk.Label(eeprom_tab, text=(
            f"{PAGE_COUNT} pages x {PAGE_SIZE} bytes. Edited bytes highlighted. "
            "Programming clears bits only (1->0).")).pack(anchor="w", padx=6, pady=(6, 0))
        self.eeprom_editor = HexEditor(eeprom_tab, num_bytes=DATA_SIZE, on_change=self._on_eeprom_change)
        self.eeprom_editor.pack(fill="both", expand=True, padx=4, pady=6)

        status_tab = ttk.Frame(nb)
        nb.add(status_tab, text="Status Register")
        top = ttk.Frame(status_tab)
        top.pack(fill="x", padx=6, pady=6)
        self.status_editor = HexEditor(top, num_bytes=STATUS_SIZE, bytes_per_row=8,
                                       on_change=self._on_status_change)
        self.status_editor.pack(anchor="w")
        legend = ttk.LabelFrame(status_tab, text="Status memory map")
        legend.pack(fill="both", expand=True, padx=6, pady=6)
        for addr, desc in STATUS_FIELDS:
            ttk.Label(legend, text=f"{addr}   {desc}", style="Info.TLabel").pack(anchor="w", padx=10, pady=1)

        console_tab = ttk.Frame(nb)
        nb.add(console_tab, text="Console")
        self.console = tk.Text(console_tab, height=12, bg="#101010", fg="#d4d4d4",
                               insertbackground="#d4d4d4", wrap="word", font=("TkFixedFont", 9))
        self.console.pack(fill="both", expand=True, padx=4, pady=4)
        self.console.tag_config("info", foreground="#9cdcfe")
        self.console.tag_config("ok", foreground="#6a9955")
        self.console.tag_config("warn", foreground="#dcdcaa")
        self.console.tag_config("err", foreground="#f44747")
        self.console.tag_config("tx", foreground="#c586c0")
        self.console.configure(state="disabled")

    def _build_statusbar(self) -> None:
        bar = ttk.Frame(self)
        bar.pack(fill="x", side="bottom")
        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(bar, textvariable=self.status_var, style="Status.TLabel", anchor="w").pack(side="left", padx=8)
        self.progress = ttk.Progressbar(bar, mode="indeterminate", length=160)
        self.progress.pack(side="right", padx=8, pady=3)

    # --- Logging ---
    def log(self, msg: str, tag: str = "info") -> None:
        ts = time.strftime("%H:%M:%S")
        self.console.configure(state="normal")
        self.console.insert("end", f"[{ts}] {msg}\n", tag)
        self.console.see("end")
        self.console.configure(state="disabled")

    def set_status(self, msg: str) -> None:
        self.status_var.set(msg)

    # --- Async machinery ---
    def _poll_queue(self) -> None:
        try:
            while True:
                self._task_queue.get_nowait()()
        except queue.Empty:
            pass
        self.after(50, self._poll_queue)

    def _begin_busy(self, msg: str) -> None:
        self._busy = True
        self.set_status(msg)
        self.progress.start(12)
        for b in self._op_buttons:
            b.configure(state="disabled")

    def _end_busy(self) -> None:
        self._busy = False
        self.progress.stop()
        if self.bridge.is_open:
            for b in self._op_buttons:
                b.configure(state="normal")

    def _run_async(self, work, done=None, busy_msg="Working...") -> None:
        if self._busy:
            self.log("Busy - please wait.", "warn")
            return
        self._begin_busy(busy_msg)

        def runner():
            result, error = None, None
            try:
                result = work()
            except Exception as exc:
                error = exc
            finally:
                self._task_queue.put(lambda: (self._end_busy(), done and done(result, error)))
        threading.Thread(target=runner, daemon=True).start()

    # --- Connection ---
    def _refresh_ports(self) -> None:
        ports = SerialBridge.list_ports()
        labels = [p.label() for p in ports]
        self._port_map = {p.label(): p.device for p in ports}
        self.port_combo["values"] = labels
        if labels and not self.port_var.get():
            self.port_combo.current(0)
        self.log(f"Found {len(labels)} serial port(s).")

    def _selected_port(self):
        label = self.port_var.get()
        return self._port_map.get(label, label) if label else None

    def _toggle_connection(self) -> None:
        if self.bridge.is_open:
            self.bridge.disconnect()
            self._set_connected(False)
            self.log("Disconnected.", "warn")
            return
        port = self._selected_port()
        if not port:
            messagebox.showwarning(APP_TITLE, "Select a serial port first.")
            return
        baud = int(self.baud_var.get())
        self.log(f"Connecting to {port} @ {baud}...", "tx")

        def work():
            self.bridge.connect(port, baud)
            ok = self.dev.ping()
            info = self.dev.info() if ok else {}
            return ok, info

        def done(result, error):
            if error:
                self.log(f"Connection failed: {error}", "err")
                self._set_connected(False)
                return
            ok, info = result
            if not ok:
                self.log("Connected but bridge did not answer PING.", "warn")
            else:
                self.log(f"Bridge online: {info}", "ok")
            self._set_connected(True)
            self._detect()

        self._run_async(work, done, "Connecting...")

    def _set_connected(self, on: bool) -> None:
        self._draw_indicator(on)
        self.connect_btn.configure(text="Disconnect" if on else "Connect")
        state = "normal" if on else "disabled"
        for b in self._op_buttons:
            b.configure(state=state)
        self.set_status(f"Connected to {self.bridge.port}" if on else "Disconnected")

    # --- Device ops ---
    def _on_use_rom(self) -> None:
        if not self.bridge.is_open:
            return
        try:
            self.dev.set_use_rom(self.use_rom_var.get())
            self.log(f"Addressing: {'Match ROM' if self.use_rom_var.get() else 'Skip ROM'}", "info")
        except (DS2502Error, SerialBridgeError) as exc:
            self.log(f"USEROM failed: {exc}", "err")

    def _on_vpp_arm(self) -> None:
        if not self.bridge.is_open:
            self.vpp_arm_var.set(False)
            return
        if self.vpp_arm_var.get():
            ok = messagebox.askyesno(APP_TITLE,
                "Arming 12V Vpp enables PROGRAMMING.\n\n"
                "Ensure 12V Vpp circuit is wired correctly.\nArm now?")
            if not ok:
                self.vpp_arm_var.set(False)
                return
        self.log("Programming " + ("ARMED" if self.vpp_arm_var.get() else "disarmed"),
                 "warn" if self.vpp_arm_var.get() else "info")

    def _detect(self) -> None:
        if not self._require_connection():
            return

        def work():
            present = self.dev.reset()
            info = self.dev.read_rom() if present else DeviceInfo()
            return present, info

        def done(result, error):
            if error:
                self.log(f"Detect failed: {error}", "err")
                return
            present, info = result
            if not present:
                self.log("No device present.", "warn")
                self._update_device_panel(DeviceInfo())
                return
            self.device_info = info
            self._update_device_panel(info)
            kind = "DS2502" if info.is_ds2502() else "non-DS2502"
            self.log(f"Detected {kind} ROM={info.rom_hex()} CRC={'OK' if info.crc_ok else 'FAIL'}", "ok")

        self._run_async(work, done, "Detecting...")

    def _scan_bus(self) -> None:
        if not self._require_connection():
            return

        def work():
            return self.dev.search()

        def done(result, error):
            if error:
                self.log(f"Scan failed: {error}", "err")
                return
            if not result:
                self.log("No devices found.", "warn")
                return
            self.log(f"Found {len(result)} device(s):", "ok")
            for r in result:
                self.log("   " + r.hex().upper(), "info")

        self._run_async(work, done, "Scanning...")

    def _update_device_panel(self, info: DeviceInfo) -> None:
        if not info.present:
            self.rom_var.set("--"); self.family_var.set("--")
            self.serial_var.set("--"); self.crc_var.set("--")
            return
        self.rom_var.set(info.rom_hex())
        self.family_var.set(f"0x{info.family:02X}" + (" (DS2502)" if info.is_ds2502() else ""))
        self.serial_var.set(info.serial)
        self.crc_var.set("OK" if info.crc_ok else "FAIL")

    # --- Read/Write ---
    def _read_eeprom(self) -> None:
        if not self._require_connection():
            return

        def work():
            return self.dev.read_memory(0, DATA_SIZE)

        def done(result, error):
            if error:
                self.log(f"Read EEPROM failed: {error}", "err")
                return
            data, crc_ok, cmd_crc_ok, page_crc_ok = result
            self.last_eeprom = data
            self.eeprom_editor.set_data(data, set_baseline=True)
            crc_detail = ""
            if not cmd_crc_ok:
                crc_detail = " [Command CRC FAIL]"
            elif not page_crc_ok:
                crc_detail = " [Page CRC FAIL]"
            self.log(f"Read {len(data)} bytes. CRC {'OK' if crc_ok else 'FAIL'}{crc_detail}.",
                     "ok" if crc_ok else "warn")

        self._run_async(work, done, "Reading EEPROM...")

    def _write_eeprom(self) -> None:
        if not self._require_connection() or not self._require_armed():
            return
        desired = self.eeprom_editor.get_data()
        changed = [i for i in range(DATA_SIZE) if desired[i] != self.last_eeprom[i]]
        if not changed:
            self.log("No changes to write.", "info")
            return
        impossible = [i for i in DS2502.can_program(self.last_eeprom, desired) if i in set(changed)]
        if impossible:
            if not messagebox.askyesno(APP_TITLE,
                f"{len(impossible)} byte(s) need 0->1 (impossible on EPROM). Continue?"):
                return
        runs = _group_runs(changed)
        self.log(f"Programming {len(changed)} byte(s)...", "tx")

        def work():
            results = []
            for start, length in runs:
                results.append(self.dev.write_memory(start, desired[start:start+length]))
            verify, _, _, _ = self.dev.read_memory(0, DATA_SIZE)
            return results, verify

        def done(result, error):
            if error:
                self.log(f"Write failed: {error}", "err")
                return
            results, verify = result
            self.last_eeprom = verify
            self.eeprom_editor.set_data(verify, set_baseline=True)
            self._report_write(results, "EEPROM")

        self._run_async(work, done, "Programming EEPROM...")

    def _verify_eeprom(self) -> None:
        if not self._require_connection():
            return
        desired = self.eeprom_editor.get_data()

        def work():
            return self.dev.read_memory(0, DATA_SIZE)

        def done(result, error):
            if error:
                self.log(f"Verify failed: {error}", "err")
                return
            data, _, _, _ = result
            diffs = [i for i in range(DATA_SIZE) if data[i] != desired[i]]
            if not diffs:
                self.log("Verify PASSED.", "ok")
            else:
                self.log(f"Verify FAILED - {len(diffs)} byte(s) differ.", "err")

        self._run_async(work, done, "Verifying...")

    def _read_status(self) -> None:
        if not self._require_connection():
            return

        def work():
            return self.dev.read_status(0, STATUS_SIZE)

        def done(result, error):
            if error:
                self.log(f"Read status failed: {error}", "err")
                return
            data, crc_ok, cmd_crc_ok, page_crc_ok = result
            self.last_status = data
            self.status_editor.set_data(data, set_baseline=True)
            crc_detail = ""
            if not cmd_crc_ok:
                crc_detail = " [Command CRC FAIL]"
            elif not page_crc_ok:
                crc_detail = " [Page CRC FAIL]"
            self.log(f"Status: {data.hex().upper()} CRC {'OK' if crc_ok else 'FAIL'}{crc_detail}",
                     "ok" if crc_ok else "warn")

        self._run_async(work, done, "Reading status...")

    def _write_status(self) -> None:
        if not self._require_connection() or not self._require_armed():
            return
        desired = self.status_editor.get_data()
        changed = [i for i in range(STATUS_SIZE) if desired[i] != self.last_status[i]]
        if not changed:
            self.log("No status changes.", "info")
            return
        if not messagebox.askyesno(APP_TITLE,
            "Writing status can permanently protect/redirect pages. Proceed?"):
            return
        runs = _group_runs(changed)

        def work():
            results = []
            for start, length in runs:
                results.append(self.dev.write_status(start, desired[start:start+length]))
            verify, _, _, _ = self.dev.read_status(0, STATUS_SIZE)
            return results, verify

        def done(result, error):
            if error:
                self.log(f"Write status failed: {error}", "err")
                return
            results, verify = result
            self.last_status = verify
            self.status_editor.set_data(verify, set_baseline=True)
            self._report_write(results, "Status")

        self._run_async(work, done, "Programming status...")

    def _report_write(self, results, what: str) -> None:
        total = sum(len(r.requested) for r in results)
        mism = sum(len(r.mismatches()) for r in results)
        pf_fails = sum(len(r.pf_failures()) for r in results)
        all_pf_ok = all(r.pf_ok for r in results)

        if mism == 0 and all_pf_ok:
            self.log(f"{what} write OK - {total} byte(s) programmed & verified. "
                     "All Program Flags OK.", "ok")
        else:
            if pf_fails > 0:
                self.log(f"{what}: {pf_fails} Program Flag FAILURE(s) - "
                         "12V pulse may not have been applied correctly!", "err")
                for r in results:
                    for i in r.pf_failures():
                        self.log(f"  PF FAIL @ 0x{r.address+i:02X}", "err")
            if mism > 0:
                self.log(f"{what}: {mism} mismatch(es) in {total} byte(s).", "warn")
                for r in results:
                    for i in r.mismatches():
                        self.log(f"  0x{r.address+i:02X}: want 0x{r.requested[i]:02X} "
                                 f"got 0x{r.readback[i]:02X}", "err")

    # --- File I/O ---
    def _load_file(self) -> None:
        path = filedialog.askopenfilename(title="Load EEPROM image",
            filetypes=[("Binary", "*.bin"), ("All", "*.*")])
        if not path:
            return
        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return
        if len(data) > DATA_SIZE:
            data = data[:DATA_SIZE]
        elif len(data) < DATA_SIZE:
            data += bytes([0xFF] * (DATA_SIZE - len(data)))
        self.eeprom_editor.set_data(data, set_baseline=False)
        self.log(f"Loaded {os.path.basename(path)} into editor.", "info")

    def _save_file(self) -> None:
        path = filedialog.asksaveasfilename(title="Save EEPROM image",
            defaultextension=".bin", filetypes=[("Binary", "*.bin"), ("All", "*.*")])
        if not path:
            return
        with open(path, "wb") as f:
            f.write(self.eeprom_editor.get_data())
        self.log(f"Saved to {os.path.basename(path)}.", "ok")

    def _fill_blank(self) -> None:
        self.eeprom_editor.set_data(bytes([0xFF] * DATA_SIZE), set_baseline=False)
        self.log("Filled with 0xFF.", "info")

    def _on_eeprom_change(self) -> None:
        n = len(self.eeprom_editor.changed_indices())
        if n:
            self.set_status(f"{n} EEPROM byte(s) modified")

    def _on_status_change(self) -> None:
        n = len(self.status_editor.changed_indices())
        if n:
            self.set_status(f"{n} status byte(s) modified")

    # --- Guards ---
    def _require_connection(self) -> bool:
        if not self.bridge.is_open:
            messagebox.showwarning(APP_TITLE, "Connect first.")
            return False
        return True

    def _require_armed(self) -> bool:
        if not self.vpp_arm_var.get():
            messagebox.showwarning(APP_TITLE, "Arm 12V Vpp before writing.")
            return False
        return True

    def _on_close(self) -> None:
        try:
            if self.bridge.is_open:
                self.dev.set_vpp(False)
            self.bridge.disconnect()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
