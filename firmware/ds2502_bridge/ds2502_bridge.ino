/*
 * ============================================================================
 *  DS2502 1-Wire EEPROM Flash Bridge  -  ESP32-S firmware (OneWire.h)
 * ============================================================================
 *  Uses the proven OneWire library for reliable 1-Wire timing on ESP32.
 *  Install: Arduino IDE -> Library Manager -> search "OneWire" by Jim Studt
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

#define FW_VERSION "2.1.0"
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
  // Release the data line (OneWire lib does this internally but be explicit)
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
//  READ BLOCK with page-boundary CRC-16
// ============================================================================
// Returns: 0 ok, 1 no presence
int readBlock(uint8_t cmd, uint16_t addr, uint8_t* buf, int len,
              bool* cmdCrcOk, bool* pageCrcOk) {
  if (!ow.reset()) return 1;
  owRomCommand();

  uint8_t ta1 = addr & 0xFF;
  uint8_t ta2 = (addr >> 8) & 0xFF;
  ow.write(cmd);
  ow.write(ta1);
  ow.write(ta2);

  // Command CRC-16 (over cmd + TA1 + TA2)
  uint8_t crcLo = ow.read();
  uint8_t crcHi = ow.read();
  uint16_t gotCmd = (uint16_t)crcLo | ((uint16_t)crcHi << 8);

  uint16_t crc = 0;
  crc = crc16Update(crc, cmd);
  crc = crc16Update(crc, ta1);
  crc = crc16Update(crc, ta2);
  uint16_t expectedCmd = (~crc) & 0xFFFF;
  if (cmdCrcOk) *cmdCrcOk = (gotCmd == expectedCmd);

  // Read data with page-boundary CRC
  bool allPageCrcOk = true;
  uint16_t pageCrc = 0;
  int currentAddr = (int)addr;

  for (int i = 0; i < len; i++) {
    buf[i] = ow.read();
    pageCrc = crc16Update(pageCrc, buf[i]);
    currentAddr++;

    if (currentAddr % DS2502_PAGE_SIZE == 0) {
      uint8_t pLo = ow.read();
      uint8_t pHi = ow.read();
      uint16_t gotPage = (uint16_t)pLo | ((uint16_t)pHi << 8);
      uint16_t expectedPage = (~pageCrc) & 0xFFFF;
      if (gotPage != expectedPage) allPageCrcOk = false;
      pageCrc = 0;
    }
  }

  if (pageCrcOk) *pageCrcOk = allPageCrcOk;
  return 0;
}

// ============================================================================
//  WRITE BLOCK with Program Flag detection & continuous page write
// ============================================================================
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

    uint8_t crcLo = ow.read();
    uint8_t crcHi = ow.read();
    uint16_t gotCrc = (uint16_t)crcLo | ((uint16_t)crcHi << 8);

    // Compute expected CRC
    uint16_t crc = 0;
    if (newTransaction) {
      crc = crc16Update(crc, cmd);
      crc = crc16Update(crc, ta1);
      crc = crc16Update(crc, ta2);
      crc = crc16Update(crc, data[i]);
      newTransaction = false;
    } else {
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

    // Programming pulse
    programPulse();

    // Program Flag (PF): device pulls LOW = success (read 0), HIGH = fail (read 1)
    uint8_t pf = ow.read_bit() ? 1 : 0;
    if (pfFlags) pfFlags[i] = pf;

    // Read-back
    uint8_t rb = ow.read();
    if (readbacks) readbacks[i] = rb;

    currentAddr++;

    // Page boundary - re-issue full command for next page
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
    ow.write(0x33);  // Read ROM
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
    bool cmdCrcOk = false, pageCrcOk = true;
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
