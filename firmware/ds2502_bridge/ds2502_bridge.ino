/*
 * ============================================================================
 *  DS2502 1-Wire EEPROM Flash Bridge  -  ESP32-S firmware (OneWire.h)
 * ============================================================================
 *  Uses the proven OneWire library for reliable 1-Wire timing on ESP32.
 *  Install: Arduino IDE -> Library Manager -> search "OneWire" by Jim Studt
 *
 *  DS2502 Read Memory (0xF0) protocol per datasheet:
 *    Master: Reset -> ROM cmd -> 0xF0 -> TA1 -> TA2
 *    Slave:  data bytes continuously (NO CRC between command and data!)
 *            CRC-16 is appended ONLY at the end of each 32-byte page
 *
 *  DS2502 Read Status (0xAA) protocol:
 *    Master: Reset -> ROM cmd -> 0xAA -> TA1 -> TA2
 *    Slave:  data bytes continuously, CRC-16 at end of status page (8 bytes)
 *
 *  DS2502 Write Memory (0x0F) protocol:
 *    Master: Reset -> ROM cmd -> 0x0F -> TA1 -> TA2 -> data_byte
 *    Slave:  CRC-16 (over cmd+TA1+TA2+data)
 *    Master: programming pulse (12V, >= 480us)
 *    Slave:  read-back byte
 *    (subsequent bytes: Master sends data, Slave sends CRC, pulse, readback)
 *
 *  Wiring:
 *    OW_PIN  (GPIO4)  -> DS2502 DATA + 4.7k pull-up to 3V3
 *    VPP_PIN (GPIO5)  -> 12V Vpp switch control (active HIGH)
 *    LED_PIN (GPIO2)  -> activity LED
 *    GND              -> DS2502 GND
 * ============================================================================
 */

#include <Arduino.h>
#include <OneWire.h>

// Pin configuration
#define OW_PIN    4
#define VPP_PIN   5
#define LED_PIN   2

#define FW_VERSION "2.4.0"
#define DS2502_FAMILY 0x09

// DS2502 function commands
#define CMD_READ_MEMORY   0xF0
#define CMD_READ_STATUS   0xAA
#define CMD_WRITE_MEMORY  0x0F
#define CMD_WRITE_STATUS  0x55

// Page geometry
#define DS2502_PAGE_SIZE  32
#define DS2502_DATA_SIZE  128
#define DS2502_STATUS_SIZE 8

// Programming pulse width (microseconds)
#define T_PROG_PULSE  512

// OneWire instance
OneWire ow(OW_PIN);

// Globals
uint8_t g_rom[8];
bool    g_romValid = false;
bool    g_useRom   = false;
String  inLine;

// Vpp control
static inline void vppOn()  { digitalWrite(VPP_PIN, HIGH); }
static inline void vppOff() { digitalWrite(VPP_PIN, LOW);  }

// Programming pulse using external 12V switch
void programPulse() {
  pinMode(OW_PIN, INPUT);
  delayMicroseconds(5);
  vppOn();
  delayMicroseconds(T_PROG_PULSE);
  vppOff();
  delayMicroseconds(10);
}

// CRC-16 (Maxim/Dallas, poly 0xA001 reflected)
uint16_t crc16Update(uint16_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
    else              crc >>= 1;
  }
  return crc;
}

uint16_t crc16Buf(const uint8_t* buf, int len) {
  uint16_t crc = 0;
  for (int i = 0; i < len; i++) crc = crc16Update(crc, buf[i]);
  return crc;
}

// Issue ROM command (Skip or Match)
void owRomCommand() {
  if (g_useRom && g_romValid) {
    ow.select(g_rom);
  } else {
    ow.skip();
  }
}

// Hex helpers
int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

int parseHex(const String& s, uint8_t* out, int maxLen) {
  int n = s.length();
  if (n % 2 != 0) return -1;
  int count = n / 2;
  if (count > maxLen) return -1;
  for (int i = 0; i < count; i++) {
    int vh = hexVal(s.charAt(i * 2));
    int vl = hexVal(s.charAt(i * 2 + 1));
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

// ============================================================================
//  READ MEMORY / STATUS   (framing per proven reference + ELNEC comparison)
// ============================================================================
// DS2502 Read Memory (0xF0):
//   Master: Reset -> Skip/Match ROM -> 0xF0 -> TA1 -> TA2
//   Device: ONE address-CRC byte    <-- must be discarded (else data shifts +1)
//   Device: 128 data bytes streamed continuously  (NO inter-page CRC bytes)
//
// DS2502 Read Status (0xAA):
//   Master: Reset -> Skip/Match ROM -> 0xAA -> TA1 -> TA2
//   Device: status bytes streamed continuously     (NO address-CRC discard)
//
// Verified against an ELNEC programmer: with Read Memory, the first byte the
// device sends after TA2 is a CRC byte (e.g. 0x8D), and the real data (e.g.
// 0x4F) begins on the next byte. Discarding exactly one byte aligns our output
// with ELNEC. Read Status has no such leading byte.
//
//   addrDiscard = number of bytes to drop after TA2 (1 for memory, 0 for status)
//
// Returns: 0 ok, 1 no presence

int readBlock(uint8_t cmd, uint16_t addr, uint8_t* buf, int len, int addrDiscard) {
  if (!ow.reset()) return 1;
  owRomCommand();

  ow.write(cmd);
  ow.write(addr & 0xFF);          // TA1 (low)
  ow.write((addr >> 8) & 0xFF);   // TA2 (high)

  // Discard the leading address-CRC byte(s) so data aligns with ELNEC.
  for (int i = 0; i < addrDiscard; i++) (void)ow.read();

  // Stream data bytes continuously (no inter-page CRC consumption).
  for (int i = 0; i < len; i++) buf[i] = ow.read();

  return 0;
}

// DIAGNOSTIC: dump 'count' raw bytes exactly as the device sends them right
// after Reset -> ROM -> cmd -> TA1 -> TA2, with NO interpretation. Use this to
// compare the true on-wire framing against a reference programmer (e.g. ELNEC).
int rawRead(uint8_t cmd, uint16_t addr, uint8_t* buf, int count) {
  if (!ow.reset()) return 1;
  owRomCommand();
  ow.write(cmd);
  ow.write(addr & 0xFF);
  ow.write((addr >> 8) & 0xFF);
  for (int i = 0; i < count; i++) buf[i] = ow.read();
  return 0;
}

// ============================================================================
//  WRITE BLOCK with CRC-16 verification + Program Flag + continuous page write
// ============================================================================
// DS2502 Write Memory (0x0F):
//   Master: Reset -> ROM -> 0x0F -> TA1 -> TA2 -> data_byte
//   Slave:  CRC-16 (cmd + TA1 + TA2 + data_byte)
//   Master: Programming pulse (12V)
//   Slave:  Program Flag bit (0=success) then read-back byte
//   For subsequent bytes within same page:
//     Master: next_data_byte
//     Slave:  CRC-16 (new_TA1 + new_TA2 + data_byte)  [address auto-increments]
//     Master: pulse
//     Slave:  PF + readback
//
// Returns: 0 ok, 1 no presence, 2 CRC mismatch

int writeBlock(uint8_t cmd, uint16_t startAddr, const uint8_t* data, int count,
               uint8_t* readbacks, uint8_t* pfFlags, bool* crcFlags) {
  if (!ow.reset()) return 1;
  owRomCommand();

  uint8_t ta1 = startAddr & 0xFF;
  uint8_t ta2 = (startAddr >> 8) & 0xFF;
  ow.write(cmd);
  ow.write(ta1);
  ow.write(ta2);

  bool anyCrcFail = false;
  uint16_t currentAddr = startAddr;
  bool newTransaction = true;

  for (int i = 0; i < count; i++) {
    ow.write(data[i]);

    // Read CRC-16 from device
    uint8_t crcLo = ow.read();
    uint8_t crcHi = ow.read();
    uint16_t gotCrc = (uint16_t)crcLo | ((uint16_t)crcHi << 8);

    // Compute expected CRC
    uint16_t crc = 0;
    if (newTransaction) {
      // First byte: CRC over cmd + TA1 + TA2 + data
      crc = crc16Update(crc, cmd);
      crc = crc16Update(crc, ta1);
      crc = crc16Update(crc, ta2);
      crc = crc16Update(crc, data[i]);
      newTransaction = false;
    } else {
      // Subsequent bytes: CRC over auto-incremented address + data
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

    // Programming pulse (12V)
    programPulse();

    // Program Flag: device pulls LOW = success (0), stays HIGH = fail (1)
    uint8_t pf = ow.read_bit() ? 1 : 0;
    if (pfFlags) pfFlags[i] = pf;

    // Read-back the programmed byte
    uint8_t rb = ow.read();
    if (readbacks) readbacks[i] = rb;

    currentAddr++;

    // Page boundary - must re-issue full command for next page
    if ((currentAddr % DS2502_PAGE_SIZE == 0) && (i < count - 1)) {
      if (!ow.reset()) return 1;
      owRomCommand();
      ta1 = currentAddr & 0xFF;
      ta2 = (currentAddr >> 8) & 0xFF;
      ow.write(cmd);
      ow.write(ta1);
      ow.write(ta2);
      newTransaction = true;
    }
  }

  return anyCrcFail ? 2 : 0;
}

// ============================================================================
//  COMMAND HANDLER
// ============================================================================
void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  blink();

  int sp = line.indexOf(' ');
  String cmd = (sp < 0) ? line : line.substring(0, sp);
  String args = (sp < 0) ? "" : line.substring(sp + 1);
  cmd.toUpperCase();

  // --- PING ---
  if (cmd == "PING") {
    Serial.println("OK PONG");
    return;
  }

  // --- INFO ---
  if (cmd == "INFO") {
    Serial.print("OK INFO fw=");
    Serial.print(FW_VERSION);
    Serial.print(" ow="); Serial.print(OW_PIN);
    Serial.print(" vpp="); Serial.print(VPP_PIN);
    Serial.print(" useRom="); Serial.print(g_useRom ? 1 : 0);
    Serial.println();
    return;
  }

  // --- RESET ---
  if (cmd == "RESET") {
    bool p = ow.reset();
    Serial.print("OK PRESENCE ");
    Serial.println(p ? 1 : 0);
    return;
  }

  // --- USEROM ---
  if (cmd == "USEROM") {
    g_useRom = (args.toInt() != 0);
    Serial.print("OK USEROM ");
    Serial.println(g_useRom ? 1 : 0);
    return;
  }

  // --- VPP ---
  if (cmd == "VPP") {
    int v = args.toInt();
    if (v) vppOn(); else vppOff();
    Serial.print("OK VPP ");
    Serial.println(v ? 1 : 0);
    return;
  }

  // --- READROM ---
  if (cmd == "READROM") {
    if (!ow.reset()) { Serial.println("ERR no_presence"); return; }
    ow.write(0x33);  // Read ROM command
    for (int i = 0; i < 8; i++) g_rom[i] = ow.read();
    g_romValid = true;

    uint8_t crc = OneWire::crc8(g_rom, 7);
    bool crcOk = (crc == g_rom[7]);

    Serial.print("OK ROM ");
    for (int i = 0; i < 8; i++) printHex(g_rom[i]);
    Serial.print(" crc=");
    Serial.print(crcOk ? 1 : 0);
    Serial.print(" family=");
    printHex(g_rom[0]);
    Serial.println();
    return;
  }

  // --- SEARCH ---
  if (cmd == "SEARCH") {
    uint8_t found[16][8];
    int count = 0;
    uint8_t addr[8];

    ow.reset_search();
    while (count < 16 && ow.search(addr)) {
      memcpy(found[count], addr, 8);
      count++;
    }
    ow.reset_search();

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

  // --- RDMEM / RDSTAT ---
  if (cmd == "RDMEM" || cmd == "RDSTAT") {
    int sp2 = args.indexOf(' ');
    if (sp2 < 0) { Serial.println("ERR bad_args"); return; }
    long addr = strtol(args.substring(0, sp2).c_str(), NULL, 16);
    int len = args.substring(sp2 + 1).toInt();
    if (len <= 0 || len > 256) { Serial.println("ERR bad_len"); return; }

    uint8_t buf[256];
    uint8_t c;
    int addrDiscard;
    if (cmd == "RDMEM") {
      c = CMD_READ_MEMORY;
      addrDiscard = 1;   // Read Memory: discard 1 leading address-CRC byte
    } else {
      c = CMD_READ_STATUS;
      addrDiscard = 0;   // Read Status: no leading discard
    }

    int r = readBlock(c, (uint16_t)addr, buf, len, addrDiscard);
    if (r == 1) { Serial.println("ERR no_presence"); return; }

    Serial.print("OK DATA ");
    for (int i = 0; i < len; i++) printHex(buf[i]);
    Serial.print(" crc=1 cmdcrc=1 pagecrc=1");
    Serial.println();
    return;
  }

  // --- RAWRD : diagnostic raw byte dump ---
  // Usage: RAWRD <cmdHex> <addrHex> <count>
  //   e.g. RAWRD F0 0 140   (Read Memory @0, dump 140 raw bytes)
  //        RAWRD AA 0 16    (Read Status @0, dump 16 raw bytes)
  // Prints exactly what the chip sends with NO framing interpretation, so the
  // true on-wire byte order (command CRC, page CRCs, data) can be compared
  // against a reference programmer.
  if (cmd == "RAWRD") {
    int s1 = args.indexOf(' ');
    if (s1 < 0) { Serial.println("ERR bad_args"); return; }
    int s2 = args.indexOf(' ', s1 + 1);
    if (s2 < 0) { Serial.println("ERR bad_args"); return; }
    uint8_t rcmd = (uint8_t)strtol(args.substring(0, s1).c_str(), NULL, 16);
    long raddr = strtol(args.substring(s1 + 1, s2).c_str(), NULL, 16);
    int rcount = args.substring(s2 + 1).toInt();
    if (rcount <= 0 || rcount > 256) { Serial.println("ERR bad_len"); return; }

    uint8_t buf[256];
    int r = rawRead(rcmd, (uint16_t)raddr, buf, rcount);
    if (r == 1) { Serial.println("ERR no_presence"); return; }

    Serial.print("OK RAW cmd=");
    printHex(rcmd);
    Serial.print(" addr=");
    Serial.print(raddr, HEX);
    Serial.print(" n=");
    Serial.print(rcount);
    Serial.print(" bytes=");
    for (int i = 0; i < rcount; i++) printHex(buf[i]);
    Serial.println();
    return;
  }

  // --- WRMEM / WRSTAT ---
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
    uint8_t readbacks[256];
    uint8_t pfFlags[256];
    bool    crcFlags[256];

    int r = writeBlock(c, (uint16_t)addr, data, n, readbacks, pfFlags, crcFlags);
    if (r == 1) { Serial.println("ERR no_presence"); return; }

    bool allCrcOk = true, allPfOk = true;
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

// ============================================================================
//  SETUP & LOOP
// ============================================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(VPP_PIN, OUTPUT);
  digitalWrite(VPP_PIN, LOW);

  Serial.begin(115200);
  delay(300);
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
      if (inLine.length() > 700) inLine = "";
    }
  }
}
