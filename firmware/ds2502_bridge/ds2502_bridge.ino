/*
 * ============================================================================
 *  DS2502 1-Wire EEPROM Flash Bridge  -  ESP32-S firmware
 * ============================================================================
 *
 *  This firmware turns an ESP32 / ESP32-S2 / ESP32-S3 board into a USB-serial
 *  to 1-Wire bridge for the Maxim/Dallas DS2502 (1 Kbit Add-Only Memory).
 *
 *  The companion Python/Tkinter application (gui/main.py) connects to this
 *  board over the USB CDC serial port and sends ASCII line commands. This
 *  firmware performs the low-level 1-Wire timing, EPROM read-back verification
 *  and (optionally) controls a 12 V Vpp programming switch.
 *
 *  ------------------------------------------------------------------------
 *  IMPORTANT - The DS2502 is an EPROM (One-Time-Programmable per bit).
 *  ------------------------------------------------------------------------
 *   * READING memory / status only needs the 1-Wire data line + GND + a
 *     4.7k pull-up to 3.3 V. No 12 V is required.
 *   * WRITING (programming) requires a 12 V programming pulse (Vpp) applied
 *     to the data line. You MUST add the Vpp switching circuit described in
 *     README.md and connect its control input to VPP_PIN.
 *   * EPROM cells can only be changed from 1 -> 0. A blank cell reads 0xFF.
 *     Writing therefore performs a logical AND with the existing contents.
 *     The firmware reads each byte back after the pulse and reports the
 *     ACTUAL programmed value so the GUI can verify.
 *
 *  ------------------------------------------------------------------------
 *  Wiring (default pins, change below if needed)
 *  ------------------------------------------------------------------------
 *    OW_PIN   (GPIO4)  -> DS2502 DATA (with 4.7k pull-up to 3V3)
 *    VPP_PIN  (GPIO5)  -> control input of the 12 V Vpp switch (active HIGH)
 *    LED_PIN  (GPIO2)  -> activity LED (optional)
 *    GND               -> DS2502 GND and Vpp circuit GND (common ground)
 *
 *  ------------------------------------------------------------------------
 *  Serial protocol  (115200 baud, 8N1, '\n' terminated lines)
 *  ------------------------------------------------------------------------
 *    PC -> ESP32                      ESP32 -> PC
 *    -----------------------------    --------------------------------------
 *    PING                             OK PONG
 *    INFO                             OK INFO fw=... ow=4 vpp=5
 *    RESET                            OK PRESENCE 1   | OK PRESENCE 0
 *    READROM                          OK ROM <16 hex> crc=1 | ERR ...
 *    SEARCH                           OK SEARCH n=<count> then DEV <rom> lines, END
 *    RDMEM <hexAddr> <len>            OK DATA <hex...> crc=<1|0>
 *    RDSTAT <hexAddr> <len>           OK DATA <hex...> crc=<1|0>
 *    WRMEM <hexAddr> <hexBytes>       OK WROTE n=<count> rb=<hex> crc=<hex>
 *    WRSTAT <hexAddr> <hexBytes>      OK WROTE n=<count> rb=<hex> crc=<hex>
 *    VPP <0|1>                        OK VPP <0|1>
 *    USEROM <0|1>                     OK USEROM <0|1>  (1=Match ROM, 0=Skip ROM)
 *
 *  Errors are always reported as:    ERR <reason>
 *
 *  The DS2502 family code is 0x09.
 * ============================================================================
 */

#include <Arduino.h>

// ----------------------------------------------------------------------------
//  Pin configuration
// ----------------------------------------------------------------------------
#define OW_PIN    4      // 1-Wire data line
#define VPP_PIN   5      // 12V Vpp switch control (active HIGH)
#define LED_PIN   2      // activity LED

#define FW_VERSION "2.0.0"
#define DS2502_FAMILY 0x09

// DS2502 memory function command bytes
#define CMD_READ_MEMORY   0xF0
#define CMD_READ_STATUS   0xAA
#define CMD_WRITE_MEMORY  0x0F
#define CMD_WRITE_STATUS  0x55

// ROM function command bytes
#define ROM_READ          0x33
#define ROM_MATCH         0x55
#define ROM_SKIP          0xCC
#define ROM_SEARCH        0xF0

// 1-Wire timing (microseconds) - standard speed
#define T_RESET_LOW   500
#define T_PRESENCE_W  70
#define T_RESET_REC   410
#define T_W1_LOW      6
#define T_W1_REC      64
#define T_W0_LOW      60
#define T_W0_REC      10
#define T_RD_LOW      6
#define T_RD_SAMPLE   9
#define T_RD_REC      55

// Programming pulse width (tPP) for the 12V Vpp pulse, microseconds.
// Datasheet tPP (min) is 480us; 500us gives a small margin.
#define T_PROG_PULSE  500

// ----------------------------------------------------------------------------
//  Globals
// ----------------------------------------------------------------------------
portMUX_TYPE owMux = portMUX_INITIALIZER_UNLOCKED;

uint8_t  g_rom[8];          // last ROM read / selected device
bool     g_romValid = false;
bool     g_useRom   = false; // false = Skip ROM, true = Match ROM (g_rom)

// ----------------------------------------------------------------------------
//  Low level 1-Wire bus control (open-drain emulation)
// ----------------------------------------------------------------------------
static inline void owDriveLow() { pinMode(OW_PIN, OUTPUT); digitalWrite(OW_PIN, LOW); }
static inline void owRelease()  { pinMode(OW_PIN, INPUT); }       // external 4.7k pulls high
static inline int  owLevel()    { return digitalRead(OW_PIN); }

// Bus reset + presence detect. Returns true if a device pulled the line low.
bool owReset() {
  owRelease();
  delayMicroseconds(5);
  owDriveLow();
  delayMicroseconds(T_RESET_LOW);
  bool present;
  portENTER_CRITICAL(&owMux);
  owRelease();
  delayMicroseconds(T_PRESENCE_W);
  present = (owLevel() == 0);
  portEXIT_CRITICAL(&owMux);
  delayMicroseconds(T_RESET_REC);
  return present;
}

void owWriteBit(int bit) {
  portENTER_CRITICAL(&owMux);
  if (bit) {
    owDriveLow();
    delayMicroseconds(T_W1_LOW);
    owRelease();
    delayMicroseconds(T_W1_REC);
  } else {
    owDriveLow();
    delayMicroseconds(T_W0_LOW);
    owRelease();
    delayMicroseconds(T_W0_REC);
  }
  portEXIT_CRITICAL(&owMux);
}

int owReadBit() {
  int bit;
  portENTER_CRITICAL(&owMux);
  owDriveLow();
  delayMicroseconds(T_RD_LOW);
  owRelease();
  delayMicroseconds(T_RD_SAMPLE);
  bit = owLevel();
  portEXIT_CRITICAL(&owMux);
  delayMicroseconds(T_RD_REC);
  return bit;
}

void owWriteByte(uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    owWriteBit(b & 0x01);
    b >>= 1;
  }
}

uint8_t owReadByte() {
  uint8_t b = 0;
  for (uint8_t i = 0; i < 8; i++) {
    b >>= 1;
    if (owReadBit()) b |= 0x80;
  }
  return b;
}

// ----------------------------------------------------------------------------
//  Vpp (12V programming voltage) control + programming pulse
// ----------------------------------------------------------------------------
static inline void vppOn()  { digitalWrite(VPP_PIN, HIGH); }
static inline void vppOff() { digitalWrite(VPP_PIN, LOW);  }

// Apply a single EPROM programming pulse.
// The 1-Wire pin is released (high-Z) so the external hardware can raise the
// data line to 12V for tPP. The MCU GPIO MUST be protected from 12V by the
// external Vpp circuit (see README.md).
void programPulse() {
  owRelease();
  delayMicroseconds(5);
  vppOn();
  delayMicroseconds(T_PROG_PULSE);
  vppOff();
  delayMicroseconds(10);   // recovery before read-back
}

// ----------------------------------------------------------------------------
//  CRC helpers
// ----------------------------------------------------------------------------
// Maxim/Dallas CRC-8 (poly X^8 + X^5 + X^4 + 1, reflected = 0x8C), init 0.
uint8_t crc8Update(uint8_t crc, uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t mix = (crc ^ data) & 0x01;
    crc >>= 1;
    if (mix) crc ^= 0x8C;
    data >>= 1;
  }
  return crc;
}

uint8_t crc8Buf(const uint8_t* p, int len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) crc = crc8Update(crc, p[i]);
  return crc;
}

// 1-Wire CRC-16 (poly X^16 + X^15 + X^2 + 1, reflected = 0xA001), init 0.
uint16_t crc16Update(uint16_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
    else              crc >>= 1;
  }
  return crc;
}

// ----------------------------------------------------------------------------
//  ROM operations
// ----------------------------------------------------------------------------
// Issue Skip ROM or Match ROM depending on g_useRom.
void owRomCommand() {
  if (g_useRom && g_romValid) {
    owWriteByte(ROM_MATCH);
    for (int i = 0; i < 8; i++) owWriteByte(g_rom[i]);
  } else {
    owWriteByte(ROM_SKIP);
  }
}

// Read the 64-bit ROM (only valid with a single device on the bus).
// Returns: 0 ok, 1 no presence, 2 crc fail.
int readRom() {
  if (!owReset()) return 1;
  owWriteByte(ROM_READ);
  for (int i = 0; i < 8; i++) g_rom[i] = owReadByte();
  g_romValid = true;
  uint8_t crc = crc8Buf(g_rom, 7);
  return (crc == g_rom[7]) ? 0 : 2;
}

// ----------------------------------------------------------------------------
//  1-Wire Search ROM algorithm (enumerate all devices on the bus)
// ----------------------------------------------------------------------------
uint8_t searchRom[8];
int     lastDiscrepancy;
bool    lastDeviceFlag;

void searchReset() {
  lastDiscrepancy = 0;
  lastDeviceFlag  = false;
  memset(searchRom, 0, 8);
}

// Returns true when a device was found (rom filled), false when done.
bool searchNext() {
  if (lastDeviceFlag) return false;
  if (!owReset()) { searchReset(); return false; }

  int idBit, cmpBit;
  int searchDir;
  int lastZero = 0;
  int romByte = 0;
  uint8_t romMask = 1;

  owWriteByte(ROM_SEARCH);

  for (int idBitNum = 1; idBitNum <= 64; idBitNum++) {
    idBit  = owReadBit();
    cmpBit = owReadBit();

    if (idBit == 1 && cmpBit == 1) {        // no devices responded
      searchReset();
      return false;
    }

    if (idBit != cmpBit) {
      searchDir = idBit;                    // all devices have same bit
    } else {
      if (idBitNum < lastDiscrepancy) {
        searchDir = ((searchRom[romByte] & romMask) > 0);
      } else {
        searchDir = (idBitNum == lastDiscrepancy);
      }
      if (searchDir == 0) lastZero = idBitNum;
    }

    if (searchDir == 1) searchRom[romByte] |= romMask;
    else                searchRom[romByte] &= ~romMask;

    owWriteBit(searchDir);

    romMask <<= 1;
    if (romMask == 0) { romByte++; romMask = 1; }
  }

  lastDiscrepancy = lastZero;
  if (lastDiscrepancy == 0) lastDeviceFlag = true;
  return true;
}

// ----------------------------------------------------------------------------
//  Memory / Status read - with page-boundary CRC-16 verification
// ----------------------------------------------------------------------------
// DS2502 datasheet: After the command CRC-16 (cmd+TA1+TA2), the device sends
// data bytes. At each 32-byte page boundary, the device transmits a CRC-16
// computed over all data bytes from the start of the current page (or from the
// starting address within that page for the first partial page).
//
// This implementation:
//  1) Verifies the command CRC-16 (cmd + TA1 + TA2)
//  2) Reads data bytes and verifies the page-end CRC-16 at each boundary
//
// Parameters:
//   cmd     - CMD_READ_MEMORY or CMD_READ_STATUS
//   addr    - starting address
//   buf     - output buffer (caller must allocate >= len bytes)
//   len     - number of data bytes to read
//   cmdCrcOk  - set to true if the command CRC matches
//   pageCrcOk - set to true if ALL page-boundary CRCs match
//
// Returns: 0 ok, 1 no presence.

#define DS2502_PAGE_SIZE  32   // bytes per page for CRC boundary

int readBlock(uint8_t cmd, uint16_t addr, uint8_t* buf, int len,
              bool* cmdCrcOk, bool* pageCrcOk) {
  if (!owReset()) return 1;
  owRomCommand();

  uint8_t ta1 = addr & 0xFF;
  uint8_t ta2 = (addr >> 8) & 0xFF;
  owWriteByte(cmd);
  owWriteByte(ta1);
  owWriteByte(ta2);

  // --- Command CRC-16 verification (over cmd + TA1 + TA2) ---
  uint8_t crcLo = owReadByte();
  uint8_t crcHi = owReadByte();
  uint16_t gotCmd = (uint16_t)crcLo | ((uint16_t)crcHi << 8);

  uint16_t crc = 0;
  crc = crc16Update(crc, cmd);
  crc = crc16Update(crc, ta1);
  crc = crc16Update(crc, ta2);
  uint16_t expectedCmd = (~crc) & 0xFFFF;
  if (cmdCrcOk) *cmdCrcOk = (gotCmd == expectedCmd);

  // --- Read data with page-boundary CRC-16 verification ---
  // The DS2502 outputs a CRC-16 after the last byte of each 32-byte page.
  // The CRC covers all data bytes read within that page segment.
  bool allPageCrcOk = true;
  uint16_t pageCrc = 0;
  int bytesInPage = 0;
  int currentAddr = (int)addr;

  // How many bytes until the first page boundary?
  int firstBoundary = DS2502_PAGE_SIZE - (currentAddr % DS2502_PAGE_SIZE);

  for (int i = 0; i < len; i++) {
    buf[i] = owReadByte();
    pageCrc = crc16Update(pageCrc, buf[i]);
    bytesInPage++;
    currentAddr++;

    // Check if we hit a page boundary
    bool atBoundary = (currentAddr % DS2502_PAGE_SIZE == 0);
    bool isLastByte = (i == len - 1);

    if (atBoundary) {
      // DS2502 sends CRC-16 at page boundary
      uint8_t pLo = owReadByte();
      uint8_t pHi = owReadByte();
      uint16_t gotPage = (uint16_t)pLo | ((uint16_t)pHi << 8);
      uint16_t expectedPage = (~pageCrc) & 0xFFFF;
      if (gotPage != expectedPage) allPageCrcOk = false;

      // Reset for next page
      pageCrc = 0;
      bytesInPage = 0;
    }
  }

  if (pageCrcOk) *pageCrcOk = allPageCrcOk;
  return 0;
}

// Overloaded version for backward compatibility (single crcOk flag)
int readBlock(uint8_t cmd, uint16_t addr, uint8_t* buf, int len, bool* crcOk) {
  bool cmdOk = false, pageOk = true;
  int ret = readBlock(cmd, addr, buf, len, &cmdOk, &pageOk);
  if (crcOk) *crcOk = (cmdOk && pageOk);
  return ret;
}

// ----------------------------------------------------------------------------
//  Memory / Status write - with Program Flag detection & continuous page write
// ----------------------------------------------------------------------------
// DS2502 Datasheet programming sequence (per byte):
//   Master: cmd + TA1 + TA2 + data_byte
//   Slave:  CRC-16 (of cmd+TA1+TA2+data)
//   Master: programming pulse (12V, >= 480us)
//   Slave:  Program Flag (PF) - device pulls line LOW briefly after pulse
//            PF=0 (low) means programming succeeded
//            PF=1 (high) means programming failed (pulse too short / no Vpp)
//   Slave:  read-back byte (actual value in memory after programming)
//
// For CONTINUOUS write within same page:
//   After reading back the first byte, subsequent bytes can be written
//   WITHOUT re-issuing reset + ROM command + write command + full address.
//   The DS2502 auto-increments the address within the current page.
//   Master just sends: next_data_byte
//   Slave responds: CRC-16 (of new address + data)
//   Then: pulse -> PF -> readback -> repeat until page boundary.
//
// Parameters:
//   cmd        - CMD_WRITE_MEMORY or CMD_WRITE_STATUS
//   addr       - starting byte address
//   data       - pointer to bytes to program
//   count      - number of bytes to program
//   readbacks  - output: actual read-back values (caller allocates >= count)
//   pfFlags    - output: program flag per byte (0=success, 1=fail)
//   crcFlags   - output: CRC ok per byte (true/false)
//
// Returns: 0 ok, 1 no presence, 2 at least one CRC mismatch.

int writeBlock(uint8_t cmd, uint16_t startAddr, const uint8_t* data, int count,
               uint8_t* readbacks, uint8_t* pfFlags, bool* crcFlags) {
  if (!owReset()) return 1;
  owRomCommand();

  uint8_t ta1 = startAddr & 0xFF;
  uint8_t ta2 = (startAddr >> 8) & 0xFF;
  owWriteByte(cmd);
  owWriteByte(ta1);
  owWriteByte(ta2);

  bool anyCrcFail = false;
  uint16_t currentAddr = startAddr;
  bool newTransaction = true;  // true when we just issued cmd+TA1+TA2

  for (int i = 0; i < count; i++) {
    // Send data byte
    owWriteByte(data[i]);

    // Read CRC-16 from device
    uint8_t crcLo = owReadByte();
    uint8_t crcHi = owReadByte();
    uint16_t gotCrc = (uint16_t)crcLo | ((uint16_t)crcHi << 8);

    // Compute expected CRC-16
    uint16_t crc = 0;
    if (newTransaction) {
      // First byte after cmd+TA: CRC covers cmd + TA1 + TA2 + data
      crc = crc16Update(crc, cmd);
      crc = crc16Update(crc, ta1);
      crc = crc16Update(crc, ta2);
      crc = crc16Update(crc, data[i]);
      newTransaction = false;
    } else {
      // Subsequent bytes in continuous mode:
      // CRC covers the auto-incremented address (TA1+TA2) + data
      uint8_t newTa1 = currentAddr & 0xFF;
      uint8_t newTa2 = (currentAddr >> 8) & 0xFF;
      crc = crc16Update(crc, newTa1);
      crc = crc16Update(crc, newTa2);
      crc = crc16Update(crc, data[i]);
    }
    uint16_t expectedCrc = (~crc) & 0xFFFF;
    bool crcOk = (gotCrc == expectedCrc);
    if (crcFlags) crcFlags[i] = crcOk;
    if (!crcOk) anyCrcFail = true;

    // Apply 12V programming pulse
    programPulse();

    // Read Program Flag (PF) - DS2502 pulls line LOW if programming succeeded
    // PF bit: 0 = success (device pulled low), 1 = fail (line stayed high)
    uint8_t pf = owReadBit() ? 1 : 0;
    if (pfFlags) pfFlags[i] = pf;

    // Read back the programmed byte
    uint8_t rb = owReadByte();
    if (readbacks) readbacks[i] = rb;

    // Advance address (DS2502 auto-increments within the page)
    currentAddr++;

    // Check if we crossed a page boundary - must re-issue full command
    if ((currentAddr % DS2502_PAGE_SIZE == 0) && (i < count - 1)) {
      if (!owReset()) return 1;
      owRomCommand();
      ta1 = currentAddr & 0xFF;
      ta2 = (currentAddr >> 8) & 0xFF;
      owWriteByte(cmd);
      owWriteByte(ta1);
      owWriteByte(ta2);
      newTransaction = true;  // next byte uses cmd+TA+data CRC format
    }
  }

  return anyCrcFail ? 2 : 0;
}

// Legacy single-byte write (backward compat for the serial handler)
int writeByte(uint8_t cmd, uint16_t addr, uint8_t value,
              uint8_t* readbackOut, uint16_t* crcOut, bool* crcOk) {
  uint8_t rb = 0xFF;
  uint8_t pf = 1;
  bool cFlag = false;

  int ret = writeBlock(cmd, addr, &value, 1, &rb, &pf, &cFlag);
  if (readbackOut) *readbackOut = rb;
  if (crcOk) *crcOk = cFlag;
  if (crcOut) *crcOut = 0;  // deprecated field
  return ret;
}

// ----------------------------------------------------------------------------
//  Serial command parser
// ----------------------------------------------------------------------------
String inLine;

int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Parse a hex string (e.g. "1A2B") into bytes. Returns number of bytes parsed,
// or -1 on malformed input.
int parseHex(const String& s, uint8_t* out, int maxLen) {
  int n = s.length();
  if (n % 2 != 0) return -1;
  int count = n / 2;
  if (count > maxLen) return -1;
  for (int i = 0; i < count; i++) {
    char hi = s.charAt(i * 2);
    char lo = s.charAt(i * 2 + 1);
    int vh = hexVal(hi), vl = hexVal(lo);
    if (vh < 0 || vl < 0) return -1;
    out[i] = (uint8_t)((vh << 4) | vl);
  }
  return count;
}

void printHex(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

void blink() {
  digitalWrite(LED_PIN, HIGH);
  delay(2);
  digitalWrite(LED_PIN, LOW);
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  blink();

  int sp = line.indexOf(' ');
  String cmd = (sp < 0) ? line : line.substring(0, sp);
  String args = (sp < 0) ? "" : line.substring(sp + 1);
  cmd.toUpperCase();

  if (cmd == "PING") {
    Serial.println("OK PONG");
    return;
  }

  if (cmd == "INFO") {
    Serial.print("OK INFO fw=");
    Serial.print(FW_VERSION);
    Serial.print(" ow=");  Serial.print(OW_PIN);
    Serial.print(" vpp="); Serial.print(VPP_PIN);
    Serial.print(" useRom="); Serial.print(g_useRom ? 1 : 0);
    Serial.println();
    return;
  }

  if (cmd == "RESET") {
    bool p = owReset();
    Serial.print("OK PRESENCE ");
    Serial.println(p ? 1 : 0);
    return;
  }

  if (cmd == "USEROM") {
    g_useRom = (args.toInt() != 0);
    Serial.print("OK USEROM ");
    Serial.println(g_useRom ? 1 : 0);
    return;
  }

  if (cmd == "VPP") {
    int v = args.toInt();
    if (v) vppOn(); else vppOff();
    Serial.print("OK VPP ");
    Serial.println(v ? 1 : 0);
    return;
  }

  if (cmd == "READROM") {
    int r = readRom();
    if (r == 1) { Serial.println("ERR no_presence"); return; }
    Serial.print("OK ROM ");
    for (int i = 0; i < 8; i++) printHex(g_rom[i]);
    Serial.print(" crc=");
    Serial.print(r == 0 ? 1 : 0);
    Serial.print(" family=");
    printHex(g_rom[0]);
    Serial.println();
    return;
  }

  if (cmd == "SEARCH") {
    searchReset();
    uint8_t found[16][8];
    int count = 0;
    while (count < 16 && searchNext()) {
      memcpy(found[count], searchRom, 8);
      count++;
    }
    Serial.print("OK SEARCH n=");
    Serial.println(count);
    for (int i = 0; i < count; i++) {
      Serial.print("DEV ");
      for (int j = 0; j < 8; j++) printHex(found[i][j]);
      Serial.println();
    }
    Serial.println("END");
    return;
  }

  if (cmd == "RDMEM" || cmd == "RDSTAT") {
    int sp2 = args.indexOf(' ');
    if (sp2 < 0) { Serial.println("ERR bad_args"); return; }
    long addr = strtol(args.substring(0, sp2).c_str(), NULL, 16);
    int len   = args.substring(sp2 + 1).toInt();
    if (len <= 0 || len > 256) { Serial.println("ERR bad_len"); return; }
    uint8_t buf[256];
    bool cmdCrcOk = false;
    bool pageCrcOk = true;
    uint8_t c = (cmd == "RDMEM") ? CMD_READ_MEMORY : CMD_READ_STATUS;
    int r = readBlock(c, (uint16_t)addr, buf, len, &cmdCrcOk, &pageCrcOk);
    if (r == 1) { Serial.println("ERR no_presence"); return; }
    Serial.print("OK DATA ");
    for (int i = 0; i < len; i++) printHex(buf[i]);
    Serial.print(" crc=");
    Serial.print((cmdCrcOk && pageCrcOk) ? 1 : 0);
    Serial.print(" cmdcrc=");
    Serial.print(cmdCrcOk ? 1 : 0);
    Serial.print(" pagecrc=");
    Serial.print(pageCrcOk ? 1 : 0);
    Serial.println();
    return;
  }

  if (cmd == "WRMEM" || cmd == "WRSTAT") {
    int sp2 = args.indexOf(' ');
    if (sp2 < 0) { Serial.println("ERR bad_args"); return; }
    long addr = strtol(args.substring(0, sp2).c_str(), NULL, 16);
    String hex = args.substring(sp2 + 1);
    hex.trim();
    uint8_t data[256];
    int n = parseHex(hex, data, 256);
    if (n < 0) { Serial.println("ERR bad_hex"); return; }
    uint8_t c = (cmd == "WRMEM") ? CMD_WRITE_MEMORY : CMD_WRITE_STATUS;

    uint8_t  readbacks[256];
    uint8_t  pfFlags[256];
    bool     crcFlags[256];

    int r = writeBlock(c, (uint16_t)addr, data, n, readbacks, pfFlags, crcFlags);
    if (r == 1) { Serial.println("ERR no_presence"); return; }

    // Compute summary flags
    bool allCrcOk = true;
    bool allPfOk  = true;
    for (int i = 0; i < n; i++) {
      if (!crcFlags[i]) allCrcOk = false;
      if (pfFlags[i] != 0) allPfOk = false;
    }

    Serial.print("OK WROTE n=");
    Serial.print(n);
    Serial.print(" crcok=");
    Serial.print(allCrcOk ? 1 : 0);
    Serial.print(" pfok=");
    Serial.print(allPfOk ? 1 : 0);
    Serial.print(" rb=");
    for (int i = 0; i < n; i++) printHex(readbacks[i]);
    Serial.print(" pf=");
    for (int i = 0; i < n; i++) Serial.print(pfFlags[i]);
    Serial.println();
    return;
  }

  Serial.print("ERR unknown_cmd ");
  Serial.println(cmd);
}

// ----------------------------------------------------------------------------
//  Arduino entry points
// ----------------------------------------------------------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(VPP_PIN, OUTPUT);
  digitalWrite(VPP_PIN, LOW);   // Vpp OFF by default - SAFETY

  owRelease();                  // data line high-Z (external pull-up)

  Serial.begin(115200);
  delay(200);
  Serial.println("OK READY DS2502-Bridge fw=" FW_VERSION);
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (inLine.length() > 0) {
        handleCommand(inLine);
        inLine = "";
      }
    } else {
      inLine += ch;
      if (inLine.length() > 700) inLine = "";  // overflow guard
    }
  }
}
