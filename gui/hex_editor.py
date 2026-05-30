"""
hex_editor.py - Editable hex-editor widget for Tkinter.
"""
from __future__ import annotations
import tkinter as tk
from tkinter import font as tkfont
from typing import Callable, List, Optional

COLOR_BG = "#1e1e1e"
COLOR_FG = "#d4d4d4"
COLOR_OFFSET = "#569cd6"
COLOR_ASCII = "#9cdcfe"
COLOR_CHANGED_BG = "#5a3a00"
COLOR_CHANGED_FG = "#ffd479"
COLOR_SELECT = "#264f78"


def _pick_mono(root: tk.Misc, size: int = 10) -> tkfont.Font:
    families = set(tkfont.families(root))
    for name in ("Consolas", "DejaVu Sans Mono", "Courier New", "Courier", "TkFixedFont"):
        if name in families or name == "TkFixedFont":
            return tkfont.Font(root=root, family=name, size=size)
    return tkfont.Font(root=root, size=size)


class HexEditor(tk.Frame):
    def __init__(self, master, num_bytes: int, bytes_per_row: int = 16,
                 read_only: bool = False,
                 on_change: Optional[Callable[[], None]] = None, **kw):
        super().__init__(master, bg=COLOR_BG, **kw)
        self.num_bytes = num_bytes
        self.bpr = bytes_per_row
        self.read_only = read_only
        self.on_change = on_change
        self._mono = _pick_mono(self)
        self._mono_bold = _pick_mono(self)
        self._mono_bold.configure(weight="bold")
        self._vars: List[tk.StringVar] = []
        self._entries: List[tk.Entry] = []
        self._ascii_labels: List[tk.Label] = []
        self._baseline = bytearray([0xFF] * num_bytes)
        self._suspend = False
        vcmd = (self.register(self._validate), "%P")
        self._build(vcmd)
        self.set_data(bytes([0xFF] * num_bytes))

    def _build(self, vcmd) -> None:
        rows = (self.num_bytes + self.bpr - 1) // self.bpr
        tk.Label(self, text="OFFSET", bg=COLOR_BG, fg=COLOR_OFFSET,
                 font=self._mono_bold).grid(row=0, column=0, padx=(6, 10), pady=(4, 2))
        for c in range(self.bpr):
            tk.Label(self, text=f"{c:02X}", bg=COLOR_BG, fg=COLOR_OFFSET,
                     font=self._mono_bold).grid(row=0, column=1 + c, padx=1, pady=(4, 2))
        tk.Label(self, text="ASCII", bg=COLOR_BG, fg=COLOR_OFFSET,
                 font=self._mono_bold).grid(row=0, column=1 + self.bpr, padx=(12, 6), pady=(4, 2))
        idx = 0
        for r in range(rows):
            tk.Label(self, text=f"{r * self.bpr:04X}", bg=COLOR_BG, fg=COLOR_OFFSET,
                     font=self._mono).grid(row=1 + r, column=0, padx=(6, 10))
            ascii_row = tk.Label(self, text="", bg=COLOR_BG, fg=COLOR_ASCII,
                                 font=self._mono, anchor="w")
            ascii_row.grid(row=1 + r, column=1 + self.bpr, padx=(12, 6), sticky="w")
            self._ascii_labels.append(ascii_row)
            for c in range(self.bpr):
                if idx >= self.num_bytes:
                    break
                var = tk.StringVar(value="FF")
                ent = tk.Entry(self, textvariable=var, width=2, justify="center",
                               font=self._mono, bg="#2d2d2d", fg=COLOR_FG,
                               insertbackground=COLOR_FG, relief="flat",
                               highlightthickness=1, highlightbackground="#3c3c3c",
                               highlightcolor=COLOR_SELECT, validate="key",
                               validatecommand=vcmd, disabledbackground="#2d2d2d",
                               disabledforeground="#808080")
                ent.grid(row=1 + r, column=1 + c, padx=1, pady=1)
                if self.read_only:
                    ent.configure(state="disabled")
                ent.bind("<FocusOut>", lambda e, i=idx: self._normalize(i))
                ent.bind("<KeyRelease>", lambda e, i=idx: self._on_edit(i))
                self._vars.append(var)
                self._entries.append(ent)
                idx += 1

    def _validate(self, proposed: str) -> bool:
        if len(proposed) > 2:
            return False
        return all(ch in "0123456789abcdefABCDEF" for ch in proposed)

    def _normalize(self, i: int) -> None:
        v = self._vars[i].get()
        if v == "":
            v = "00"
        self._vars[i].set(f"{int(v, 16):02X}")
        self._refresh_row(i // self.bpr)
        self._refresh_cell_color(i)

    def _on_edit(self, i: int) -> None:
        if self._suspend:
            return
        self._refresh_row(i // self.bpr)
        self._refresh_cell_color(i)
        if self.on_change:
            self.on_change()

    def set_data(self, data: bytes, set_baseline: bool = True) -> None:
        self._suspend = True
        n = min(len(data), self.num_bytes)
        for i in range(self.num_bytes):
            b = data[i] if i < n else 0xFF
            self._vars[i].set(f"{b:02X}")
        if set_baseline:
            self._baseline = bytearray(self.get_data())
        self._suspend = False
        self._refresh_all()
        if self.on_change:
            self.on_change()

    def get_data(self) -> bytes:
        out = bytearray(self.num_bytes)
        for i in range(self.num_bytes):
            v = self._vars[i].get().strip()
            out[i] = int(v, 16) if v else 0
        return bytes(out)

    def get_byte(self, i: int) -> int:
        v = self._vars[i].get().strip()
        return int(v, 16) if v else 0

    def mark_baseline(self) -> None:
        self._baseline = bytearray(self.get_data())
        self._refresh_all()

    def changed_indices(self) -> List[int]:
        cur = self.get_data()
        return [i for i in range(self.num_bytes) if cur[i] != self._baseline[i]]

    def set_read_only(self, ro: bool) -> None:
        self.read_only = ro
        state = "disabled" if ro else "normal"
        for ent in self._entries:
            ent.configure(state=state)

    def _refresh_cell_color(self, i: int) -> None:
        try:
            changed = self.get_byte(i) != self._baseline[i]
        except ValueError:
            changed = True
        ent = self._entries[i]
        if changed:
            ent.configure(bg=COLOR_CHANGED_BG, fg=COLOR_CHANGED_FG,
                          disabledbackground=COLOR_CHANGED_BG)
        else:
            ent.configure(bg="#2d2d2d", fg=COLOR_FG, disabledbackground="#2d2d2d")

    def _refresh_row(self, r: int) -> None:
        start = r * self.bpr
        chars = []
        for c in range(self.bpr):
            i = start + c
            if i >= self.num_bytes:
                break
            try:
                b = self.get_byte(i)
            except ValueError:
                b = 0
            chars.append(chr(b) if 32 <= b < 127 else ".")
        self._ascii_labels[r].configure(text="".join(chars))

    def _refresh_all(self) -> None:
        rows = (self.num_bytes + self.bpr - 1) // self.bpr
        for r in range(rows):
            self._refresh_row(r)
        for i in range(self.num_bytes):
            self._refresh_cell_color(i)
