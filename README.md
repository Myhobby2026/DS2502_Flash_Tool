# DS2502 Flash Tool

Advanced Windows application for **reading and programming the Dallas/Maxim DS2502** (1 Kbit 1-Wire Add-Only Memory) using an **ESP32-S** dev board as a USB-to-1-Wire bridge.

## Features

- Read/Write **128-byte data EEPROM** (4 pages x 32 bytes)
- Read/Write **8-byte status register** (page redirection + write protect)
- Live hex editor with change highlighting and ASCII pane
- Per-byte read-back verification after 12V programming pulse
- CRC-8 (ROM) and CRC-16 (memory) validation
- ROM detection, multi-drop bus scan (Search ROM)
- Load/Save `.bin` files
- Safety interlocks for EPROM programming
- Threaded UI - never freezes during serial operations

## Project Structure

```
DS2502_Flash_Tool/
+-- firmware/
|   +-- ds2502_bridge/
|       +-- ds2502_bridge.ino     ESP32 USB-to-1Wire bridge firmware
+-- gui/
|   +-- main.py                   Tkinter application (entry point)
|   +-- ds2502.py                 DS2502 device model + CRC
|   +-- serial_bridge.py          Thread-safe pyserial transport
|   +-- hex_editor.py             Hex editor widget
|   +-- requirements.txt          Python dependencies
+-- README.md
```

## Hardware Wiring

### Read-Only (no 12V needed)

```
ESP32-S                         DS2502
+---------+                     +------+
| GPIO4   |----o--------------->| DATA |
|         |    |                |      |
| 3V3     |----+--[4.7k]-------+      |
| GND     |--------------------------->| GND  |
+---------+                     +------+
```

### Programming (requires 12V Vpp circuit)

```
                         +12V
                          |
                      [Q1] P-MOSFET high-side switch
           GPIO5 --[drv]--|
                          |
  GPIO4 --[1k]--+--------+---------> DS2502 DATA
                 |        |
                 |      [4.7k] to 3V3
                 |
               [D1] Schottky clamp to 3V3 (protects GPIO4)
```

**Key points:**
- GPIO4 connects through a 1k series resistor + Schottky clamp
- Q1 (P-MOSFET) connects +12V to the data line only during the 500us programming pulse
- GPIO5 controls the MOSFET gate
- ESP32 GPIO is NEVER exposed to 12V directly

### Pin Defaults (edit in .ino)

| Signal | GPIO | Purpose |
|--------|------|---------|
| OW_PIN | 4 | 1-Wire data line |
| VPP_PIN | 5 | 12V Vpp switch control |
| LED_PIN | 2 | Activity LED |

## Setup

### 1. Flash ESP32 Firmware

1. Install ESP32 board package in Arduino IDE
2. Open `firmware/ds2502_bridge/ds2502_bridge.ino`
3. Select your ESP32-S board and COM port
4. Upload
5. Open Serial Monitor @ 115200 - you should see `OK READY DS2502-Bridge fw=1.0.0`

No external Arduino libraries required.

### 2. Run the GUI (Windows)

```bat
cd gui
pip install -r requirements.txt
python main.py
```

Tkinter ships with the standard Python installer on Windows.

## Usage

1. **Connect** - Select COM port, click Connect
2. **Read EEPROM** - Pulls all 128 bytes into hex editor
3. **Edit** - Modify bytes (changes highlighted) or Load .bin
4. **Arm 12V Vpp** - Tick checkbox (confirmation dialog)
5. **Write EEPROM** - Programs only changed bytes, verifies each one
6. **Verify** - Re-reads device and compares to editor
7. **Status Register** - Read/Write the 8 status bytes

## Serial Protocol

ASCII, newline-terminated, 115200 8N1:

| Command | Response |
|---------|----------|
| `PING` | `OK PONG` |
| `INFO` | `OK INFO fw=1.0.0 ow=4 vpp=5 useRom=0` |
| `RESET` | `OK PRESENCE 1` |
| `READROM` | `OK ROM <16hex> crc=1 family=09` |
| `SEARCH` | `OK SEARCH n=<k>` + `DEV <rom>` lines + `END` |
| `RDMEM <addr> <len>` | `OK DATA <hex> crc=1` |
| `RDSTAT <addr> <len>` | `OK DATA <hex> crc=1` |
| `WRMEM <addr> <hex>` | `OK WROTE n=<k> crcok=1 rb=<hex>` |
| `WRSTAT <addr> <hex>` | `OK WROTE n=<k> crcok=1 rb=<hex>` |
| `VPP <0\|1>` | `OK VPP <0\|1>` |
| `USEROM <0\|1>` | `OK USEROM <0\|1>` |

## Important: DS2502 is EPROM

- Blank = 0xFF, bits can only go **1 -> 0**
- Writing is a logical AND with existing contents
- Status writes can **permanently** protect/redirect pages
- The tool warns about impossible 0->1 bit changes
