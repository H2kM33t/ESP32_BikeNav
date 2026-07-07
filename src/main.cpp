//// NEW CODE ////

/*
  BikeNav OLED Display Firmware
  ESP32 + 1.3" OLED (SH1106, 128x64) via u8g2 + NimBLE

  Layout:
  ┌─────────────────────────────┐
  │ [ turn ]         total: 4.2km│  <- right side, small, top
  │ [ icon ]          next: 120m │  <- right side, small
  │ (medium)     Turn right onto │  <- right side, small, wraps
  │  55 kmh         Oak Street   │
  └─────────────────────────────┘
  Left column : turn icon (medium) on top, speed at the bottom.
  Right column: total distance left (small, top), distance to next
                turn (small, below it), then the turn instruction
                text as given by Google Maps (small, wraps, below that).

  Packet format (matches navigation.h / Android BleNavClient.kt):
    byte 0   : TurnDir (0-10)
    byte 1-2 : distance to next turn, uint16 big-endian, metres
    byte 3-4 : total journey distance, uint16 big-endian, metres
    byte 5-6 : remaining journey distance, uint16 big-endian, metres
    byte 7   : speed in km/h (0-255)
    byte 8+  : "streetName|towardName" ASCII  (we just show it as one string)

  TurnDir codes (must match BleUuids / NavAccessibilityService):
    0 = straight/continue     6 = sharp left
    1 = left                  7 = sharp right
    2 = right                 8 = roundabout
    3 = slight left           9 = (reserved / merge, add as needed)
    4 = slight right         10 = (reserved / fork, add as needed)
    5 = u-turn
*/

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <NimBLEDevice.h>
#include <math.h>

// ---------------- I2C pins ----------------
// Most ESP32 dev boards default to SDA=21, SCL=22, but plenty of boards
// (incl. some "denky_d4" / D1-mini32 style boards) wire it differently.
// Set these to whatever your board actually uses. If you're not sure,
// the I2C scanner in setup() below will tell you if anything responds
// at all on these pins.
#define OLED_SDA 21
#define OLED_SCL 22

// ---------------- Display ----------------
// 1.3" OLED modules are almost always SH1106 128x64 (not SSD1306) even
// though they're often sold as "0.96 upgrade". If your text looks shifted
// 2px to the right or clipped, you have the wrong controller — swap to
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C and re-test.
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// ---------------- BLE ----------------
// Must match BikeNavApp/BleUuids.kt exactly.
static const char *SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char *NAV_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
static const char *DEVICE_NAME = "BikeNav";

NimBLEServer *pServer = nullptr;
NimBLECharacteristic *pNavChar = nullptr;
bool deviceConnected = false;

// ---------------- Nav state ----------------
struct NavState
{
  uint8_t turn = 0;
  uint16_t distToTurn = 0; // metres
  uint16_t totalDist = 0;  // metres
  uint16_t remainDist = 0; // metres
  uint8_t speedKmh = 0;
  String instruction = ""; // "streetName|towardName" or plain text
  bool valid = false;
  unsigned long lastUpdateMs = 0;
};

NavState nav;

// Consider data stale if nothing arrives for this long (matches the
// PACKET_TIMEOUT_MS mentioned in the Android heartbeat comments).
static const unsigned long PACKET_TIMEOUT_MS = 20000;

// ---------------- Turn icon drawing ----------------
// Medium-size icon drawn with simple vector shapes so it doesn't depend on
// a font/glyph set. Drawn inside a square box (x,y = top-left, size = box).
void drawTurnIcon(int x, int y, int size, uint8_t turn)
{
  int cx = x + size / 2;
  int cy = y + size / 2;
  int arm = size / 2 - 2;

  u8g2.setDrawColor(1);

  switch (turn)
  {
  case 1: // turn-left: clean 90-degree corner (up, then left)
    u8g2.drawLine(cx, cy + arm, cx, cy - 2);
    u8g2.drawLine(cx, cy - 2, cx - arm, cy - 2);
    u8g2.drawLine(cx - arm, cy - 2, cx - arm + 7, cy - 2 - 7);
    u8g2.drawLine(cx - arm, cy - 2, cx - arm + 7, cy - 2 + 7);
    break;
  case 2: // turn-right: mirror
    u8g2.drawLine(cx, cy + arm, cx, cy - 2);
    u8g2.drawLine(cx, cy - 2, cx + arm, cy - 2);
    u8g2.drawLine(cx + arm, cy - 2, cx + arm - 7, cy - 2 - 7);
    u8g2.drawLine(cx + arm, cy - 2, cx + arm - 7, cy - 2 + 7);
    break;
  case 3: // slight-left: mostly vertical, small kink near the top (<90 deg bend)
    u8g2.drawLine(cx, cy + arm, cx, cy - arm / 3);
    u8g2.drawLine(cx, cy - arm / 3, cx - arm, cy - arm);
    u8g2.drawLine(cx - arm, cy - arm, cx - arm + 7, cy - arm + 2);
    u8g2.drawLine(cx - arm, cy - arm, cx - arm + 2, cy - arm + 7);
    break;
  case 4: // slight-right: mirror
    u8g2.drawLine(cx, cy + arm, cx, cy - arm / 3);
    u8g2.drawLine(cx, cy - arm / 3, cx + arm, cy - arm);
    u8g2.drawLine(cx + arm, cy - arm, cx + arm - 7, cy - arm + 2);
    u8g2.drawLine(cx + arm, cy - arm, cx + arm - 2, cy - arm + 7);
    break;
  case 5: // sharp-left: bends well PAST 90 degrees — exits below the
          // pivot height, reading as a much tighter hook than a plain turn
    u8g2.drawLine(cx, cy + arm, cx, cy - 4);
    u8g2.drawLine(cx, cy - 4, cx - arm, cy + arm / 3);
    u8g2.drawLine(cx - arm, cy + arm / 3, cx - arm + 8, cy + arm / 3 - 3);
    u8g2.drawLine(cx - arm, cy + arm / 3, cx - arm + 4, cy + arm / 3 + 7);
    break;
  case 6: // sharp-right: mirror
    u8g2.drawLine(cx, cy + arm, cx, cy - 4);
    u8g2.drawLine(cx, cy - 4, cx + arm, cy + arm / 3);
    u8g2.drawLine(cx + arm, cy + arm / 3, cx + arm - 8, cy + arm / 3 - 3);
    u8g2.drawLine(cx + arm, cy + arm / 3, cx + arm - 4, cy + arm / 3 + 7);
    break;
  case 7: // uturn-left: hook curving back to the left
    u8g2.drawCircle(cx, cy - 2, arm - 2, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
    u8g2.drawLine(cx - (arm - 2), cy - 2, cx - (arm - 2), cy + arm - 4);
    u8g2.drawLine(cx - (arm - 2), cy + arm - 4, cx - (arm - 2) - 5, cy + arm - 9);
    u8g2.drawLine(cx - (arm - 2), cy + arm - 4, cx - (arm - 2) + 5, cy + arm - 9);
    break;
  case 8: // uturn-right: mirror
    u8g2.drawCircle(cx, cy - 2, arm - 2, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
    u8g2.drawLine(cx + (arm - 2), cy - 2, cx + (arm - 2), cy + arm - 4);
    u8g2.drawLine(cx + (arm - 2), cy + arm - 4, cx + (arm - 2) - 5, cy + arm - 9);
    u8g2.drawLine(cx + (arm - 2), cy + arm - 4, cx + (arm - 2) + 5, cy + arm - 9);
    break;
  case 9: // roundabout-left: entry from bottom, exit top-LEFT (counterclockwise)
  {
    int r = arm - 4;
    u8g2.drawCircle(cx, cy, r);
    u8g2.drawLine(cx + 2, cy + arm, cx + 2, cy + r);
    int exitStartX = cx - (r * 7) / 10;
    int exitStartY = cy - (r * 7) / 10;
    int exitEndX = cx - arm;
    int exitEndY = cy - arm;
    u8g2.drawLine(exitStartX, exitStartY, exitEndX, exitEndY);
    u8g2.drawLine(exitEndX, exitEndY, exitEndX + 6, exitEndY);
    u8g2.drawLine(exitEndX, exitEndY, exitEndX, exitEndY + 6);
    break;
  }
  case 10: // roundabout-right: entry bottom, exit top-RIGHT (clockwise)
  {
    int r = arm - 4;
    u8g2.drawCircle(cx, cy, r);
    u8g2.drawLine(cx - 2, cy + arm, cx - 2, cy + r);
    int exitStartX = cx + (r * 7) / 10;
    int exitStartY = cy - (r * 7) / 10;
    int exitEndX = cx + arm;
    int exitEndY = cy - arm;
    u8g2.drawLine(exitStartX, exitStartY, exitEndX, exitEndY);
    u8g2.drawLine(exitEndX, exitEndY, exitEndX - 6, exitEndY);
    u8g2.drawLine(exitEndX, exitEndY, exitEndX, exitEndY + 6);
    break;
  }
  case 11: // merge: two lines converging into one, continuing up
    u8g2.drawLine(cx - arm, cy + arm, cx, cy);
    u8g2.drawLine(cx + arm - 6, cy + arm, cx, cy);
    u8g2.drawLine(cx, cy, cx, cy - arm);
    u8g2.drawLine(cx, cy - arm, cx - 5, cy - arm + 6);
    u8g2.drawLine(cx, cy - arm, cx + 5, cy - arm + 6);
    break;
  case 12: // fork-left: Y splitting, left branch is the taken (arrowed) one
    u8g2.drawLine(cx, cy + arm, cx, cy);
    u8g2.drawLine(cx, cy, cx - arm, cy - arm);
    u8g2.drawLine(cx - arm, cy - arm, cx - arm + 7, cy - arm + 2);
    u8g2.drawLine(cx - arm, cy - arm, cx - arm + 2, cy - arm + 7);
    u8g2.drawLine(cx, cy, cx + arm - 6, cy - arm + 6); // thin untaken branch
    break;
  case 13: // fork-right: mirror
    u8g2.drawLine(cx, cy + arm, cx, cy);
    u8g2.drawLine(cx, cy, cx + arm, cy - arm);
    u8g2.drawLine(cx + arm, cy - arm, cx + arm - 7, cy - arm + 2);
    u8g2.drawLine(cx + arm, cy - arm, cx + arm - 2, cy - arm + 7);
    u8g2.drawLine(cx, cy, cx - arm + 6, cy - arm + 6); // thin untaken branch
    break;
  case 14: // ramp-left: slight-left with a curved ramp line alongside
    u8g2.drawLine(cx + 3, cy + arm, cx + 3, cy);
    u8g2.drawCircle(cx - arm + 6, cy - 3, arm - 4, U8G2_DRAW_UPPER_RIGHT);
    u8g2.drawLine(cx - arm + 6, cy - arm + 1, cx - arm + 13, cy - arm + 3);
    u8g2.drawLine(cx - arm + 6, cy - arm + 1, cx - arm + 8, cy - arm + 8);
    break;
  case 15: // ramp-right: mirror
    u8g2.drawLine(cx - 3, cy + arm, cx - 3, cy);
    u8g2.drawCircle(cx + arm - 6, cy - 3, arm - 4, U8G2_DRAW_UPPER_LEFT);
    u8g2.drawLine(cx + arm - 6, cy - arm + 1, cx + arm - 13, cy - arm + 3);
    u8g2.drawLine(cx + arm - 6, cy - arm + 1, cx + arm - 8, cy - arm + 8);
    break;
  case 0: // straight
  default:
    u8g2.drawLine(cx, cy + arm, cx, cy - arm);
    u8g2.drawLine(cx, cy - arm, cx - 6, cy - arm + 6);
    u8g2.drawLine(cx, cy - arm, cx + 6, cy - arm + 6);
    break;
  }
}

// ---------------- Helpers ----------------
// Switches to "<n>.<n>km" above 1000m (matches the layout diagram at the
// top of this file); below that, plain metres. The BLE payload already
// carries plain metre integers (converted from whatever unit Maps was
// displaying, including ft/mi), so this is the only place units get
// reformatted for display.
String formatDistance(uint16_t metres)
{
  if (metres >= 1000)
  {
    float km = metres / 1000.0f;
    char buf[8];
    snprintf(buf, sizeof(buf), "%.1fkm", km);
    return String(buf);
  }
  return String(metres) + "m";
}

// Word-wrap a string into up to maxLines lines that fit width pixels in the
// currently selected font, writing them starting at (x, yFirstBaseline)
// with lineHeight spacing.
void drawWrapped(const String &text, int x, int yFirstBaseline, int width,
                 int lineHeight, int maxLines)
{
  int start = 0;
  int lines = 0;
  int len = text.length();
  while (start < len && lines < maxLines)
  {
    int end = start;
    int lastSpace = -1;
    while (end < len)
    {
      if (text[end] == ' ')
        lastSpace = end;
      String candidate = text.substring(start, end + 1);
      if (u8g2.getStrWidth(candidate.c_str()) > width)
      {
        break;
      }
      end++;
    }
    int cut;
    if (end >= len)
    {
      cut = len;
    }
    else if (lastSpace > start)
    {
      cut = lastSpace;
    }
    else
    {
      cut = end; // hard break, no space found
    }
    String line = text.substring(start, cut);
    line.trim();
    u8g2.drawStr(x, yFirstBaseline + lines * lineHeight, line.c_str());
    lines++;
    start = cut;
    while (start < len && text[start] == ' ')
      start++;
  }
}

// ---------------- Animation helpers ----------------
// Everything here is driven off millis() rather than a stored frame
// counter, so there's no extra state to keep in sync - the animation
// phase is just "what time is it", which also means it can't drift or
// get stuck on a skipped frame.

// A dot that grows and shrinks smoothly (a "breathing" pulse), used as a
// generic "something is happening, please wait" indicator. periodMs is
// one full grow+shrink cycle.
void drawBreathingDot(int cx, int cy, int minR, int maxR, unsigned long periodMs)
{
  float phase = fmod(millis(), (float)periodMs) / (float)periodMs; // 0..1
  float t = (1.0f - cosf(phase * 2.0f * PI)) / 2.0f;                // 0..1, eased
  int r = minR + (int)((maxR - minR) * t);
  u8g2.drawDisc(cx, cy, r);
}

// "..." with 0-3 dots cycling, the classic minimal "loading" indicator.
// Returns the string so callers can measure/center it before drawing.
String loadingDots(unsigned long periodMs, int maxDots)
{
  int count = (int)((millis() / (periodMs / (maxDots + 1))) % (maxDots + 1));
  String s;
  for (int i = 0; i < count; i++)
    s += '.';
  return s;
}

void drawCentered(const String &text, int y)
{
  int w = u8g2.getStrWidth(text.c_str());
  int x = (128 - w) / 2;
  if (x < 0)
    x = 0;
  u8g2.drawStr(x, y, text.c_str());
}

// ---------------- Screens ----------------

// Shown while advertising but no phone has connected yet.
void renderWaitingForConnection()
{
  u8g2.setFont(u8g2_font_helvB10_tr);
  drawCentered("BikeNav", 20);

  drawBreathingDot(64, 38, 3, 7, 1400);

  u8g2.setFont(u8g2_font_helvR08_tr);
  String line = "Waiting for phone" + loadingDots(1200, 3);
  drawCentered(line, 58);
}

// Shown once the phone is connected but no navigation packet has arrived
// (or navigation session has ended) yet.
void renderConnectedWaitingForNav()
{
  u8g2.setFont(u8g2_font_helvB10_tr);
  drawCentered("Connected", 20);

  drawBreathingDot(64, 38, 3, 7, 1000);

  u8g2.setFont(u8g2_font_helvR08_tr);
  String line = "Waiting for route" + loadingDots(1200, 3);
  drawCentered(line, 58);
}

// The normal in-navigation screen - turn icon, speed, distances,
// instruction text - plus a small live-data pulse.
void renderNavigationActive()
{
  bool stale = millis() - nav.lastUpdateMs > PACKET_TIMEOUT_MS;

  // ---- Left column: turn icon (medium) + speed below ----
  const int leftColW = 46;
  const int iconSize = 32;
  const int iconX = 4;
  const int iconY = 2;
  drawTurnIcon(iconX, iconY, iconSize, nav.turn);

  u8g2.setFont(u8g2_font_helvB12_tr);
  char speedBuf[8];
  snprintf(speedBuf, sizeof(speedBuf), "%dkm/h", nav.speedKmh);
  // Center the speed text under the icon.
  int speedW = u8g2.getStrWidth(speedBuf);
  int speedX = iconX + (iconSize - speedW) / 2;
  if (speedX < 0)
    speedX = 0;
  u8g2.drawStr(speedX, 60, speedBuf);

  // Vertical divider between the two columns.
  u8g2.drawVLine(leftColW, 0, 64);

  // ---- Right column: total left / next-turn distance / instruction ----
  const int rightX = leftColW + 5;
  const int rightW = 128 - rightX - 2;

  // Total distance remaining — small, top.
  u8g2.setFont(u8g2_font_helvR08_tr);
  String totalLine = "Total: " + formatDistance(nav.remainDist);
  u8g2.drawStr(rightX, 9, totalLine.c_str());

  // Small live-data pulse in the corner: a steady 1Hz blink while packets
  // are still arriving, held solid-off when stale. Deliberately tiny and
  // out of the way - a status indicator, not a decoration.
  if (!stale && (millis() / 500) % 2 == 0)
  {
    u8g2.drawDisc(124, 4, 2);
  }

  // Distance to next turn — medium size, this is the number that matters
  // most moment-to-moment so it gets the biggest type in the right column.
  u8g2.setFont(u8g2_font_helvB14_tr);
  String nextLine = formatDistance(nav.distToTurn);
  u8g2.drawStr(rightX, 27, nextLine.c_str());

  u8g2.drawHLine(rightX, 31, rightW);

  // Instruction text (e.g. "Turn right onto Oak Street"), wrapped, back to
  // small type so 2-3 lines fit under the medium distance readout.
  u8g2.setFont(u8g2_font_helvR08_tr);
  String instr = nav.instruction;
  instr.replace("|", " ");
  drawWrapped(instr, rightX, 42, rightW, 11, 2);


  if (stale)
  {
    u8g2.drawStr(rightX, 63, "(stale)");
  }
}

// ---------------- Rendering dispatcher ----------------
// Picks which screen to draw based on connection/nav state. Called
// frequently (see loop()) so the animated screens stay smooth; each
// screen function only draws into the buffer; sendBuffer() happens once
// here so there's a single clean handoff to the display per frame.
void renderScreen()
{
  u8g2.clearBuffer();

  if (!deviceConnected)
  {
    renderWaitingForConnection();
  }
  else if (!nav.valid)
  {
    renderConnectedWaitingForNav();
  }
  else
  {
    renderNavigationActive();
  }

  u8g2.sendBuffer();
}


// ---------------- BLE payload parsing ----------------
void onNavPacket(const uint8_t *data, size_t len)
{
  Serial.printf("[NAV] Packet received, %u bytes: ", (unsigned)len);
  for (size_t i = 0; i < len; i++)
  {
    Serial.printf("%02X ", data[i]);
  }
  Serial.println();

  if (len < 8)
  {
    Serial.println("[NAV] Packet too short (<8 bytes), ignoring.");
    return;
  }

  nav.turn = data[0];
  nav.distToTurn = (uint16_t(data[1]) << 8) | data[2];
  nav.totalDist = (uint16_t(data[3]) << 8) | data[4];
  nav.remainDist = (uint16_t(data[5]) << 8) | data[6];
  nav.speedKmh = data[7];

  if (len > 8)
  {
    nav.instruction = String((const char *)(data + 8), len - 8);
  }
  else
  {
    nav.instruction = "";
  }

  nav.valid = true;
  nav.lastUpdateMs = millis();

  Serial.printf("[NAV] turn=%d distToTurn=%um total=%um remain=%um speed=%ukmh instr='%s'\n",
                nav.turn, nav.distToTurn, nav.totalDist, nav.remainDist,
                nav.speedKmh, nav.instruction.c_str());

  // Deliberately NOT calling renderScreen() here. NimBLE callbacks run on
  // their own FreeRTOS task, separate from the Arduino loop() task; with
  // loop() now repainting every 120ms for the animations, having two
  // tasks both touch u8g2/Wire (I2C) at once is exactly what was causing
  // the intermittent glitching. loop()'s repaint picks up this data
  // within 120ms on its own - imperceptible, and confines all display/I2C
  // access to a single task.
}

// ---------------- NimBLE server callbacks ----------------
class NavCharCallbacks : public NimBLECharacteristicCallbacks
{
  // Newer NimBLE-Arduino versions pass a NimBLEConnInfo& as a second
  // argument; the single-argument signature no longer matches the base
  // class's virtual, which is why 'override' was failing to compile.
  void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override
  {
    std::string value = pChar->getValue();
    onNavPacket((const uint8_t *)value.data(), value.length());
  }
};

class ServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override
  {
    deviceConnected = true;
    Serial.println("[BLE] Phone connected.");
    // Wipe leftover state from a previous session so the screen goes back
    // to "Waiting for navigation..." until a genuinely fresh packet
    // arrives, rather than continuing to show whatever was last drawn.
    // No renderScreen() call here either, for the same reason as
    // onNavPacket() above - this runs on NimBLE's task, not loop()'s;
    // the fast periodic repaint reflects nav.valid=false within 120ms.
    nav.valid = false;
  }
  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override
  {
    deviceConnected = false;
    Serial.printf("[BLE] Phone disconnected (reason=%d). Restarting advertising.\n", reason);
    NimBLEDevice::startAdvertising();
  }
};

void setupBle()
{
  NimBLEDevice::init(DEVICE_NAME);

  // 0-only lets any phone pair/bond without a passkey prompt; fine for a
  // hobby project, tighten later if you care about someone else's phone
  // being able to see/connect to it.
  NimBLEDevice::setSecurityAuth(false, false, false);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService *pService = pServer->createService(SERVICE_UUID);
  pNavChar = pService->createCharacteristic(
      NAV_CHAR_UUID,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pNavChar->setCallbacks(new NavCharCallbacks());

  // NOTE: pService->start() is deprecated/no-op in newer NimBLE-Arduino —
  // services are started automatically when the server starts advertising.

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);

  // IMPORTANT: with a 128-bit service UUID advertised, there usually isn't
  // room left in the 31-byte primary advertising packet to also fit the
  // device name — Maps/your phone's scanner then sees an unnamed device
  // and it doesn't show up as "BikeNav" in the list (this is very likely
  // why it wasn't visible). Forcing the name into the scan-response packet
  // fixes that.
  pAdvertising->setName(DEVICE_NAME);
  pAdvertising->enableScanResponse(true);

  bool advOk = pAdvertising->start();
  Serial.printf("[BLE] advertising start() returned: %s\n", advOk ? "true" : "false");
}

// ---------------- I2C scanner (debug) ----------------
// Confirms whether the OLED is actually wired/powered correctly and what
// address it responds on (0x3C and 0x3D are the two common SH1106/SSD1306
// addresses). If this prints nothing, it's a wiring/power problem, not a
// u8g2/software problem.
void scanI2C()
{
  Serial.println("[I2C] Scanning...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0)
    {
      Serial.printf("[I2C] Device found at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0)
  {
    Serial.println("[I2C] No devices found! Check wiring (SDA/SCL), power, and pin numbers (OLED_SDA/OLED_SCL).");
  }
  else
  {
    Serial.printf("[I2C] Scan complete, %d device(s) found.\n", found);
  }
}

// ---------------- Arduino lifecycle ----------------
void setup()
{
  Serial.begin(115200);
  delay(1000); // give the serial monitor time to attach
  Serial.println();
  Serial.println("=== BikeNav Display booting ===");

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  scanI2C();

  Serial.println("[OLED] Calling u8g2.begin()...");
  bool ok = u8g2.begin();
  Serial.printf("[OLED] u8g2.begin() returned: %s\n", ok ? "true" : "false");

  // Animated boot splash - blocks briefly here, which is fine since BLE
  // isn't set up yet and nothing else needs the CPU. Kept short (~1.2s)
  // so it reads as a boot flourish, not a delay.
  unsigned long bootStart = millis();
  while (millis() - bootStart < 1200)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvB10_tr);
    drawCentered("BikeNav", 30);
    u8g2.setFont(u8g2_font_helvR08_tr);
    drawCentered("booting" + loadingDots(600, 3), 50);
    u8g2.sendBuffer();
    delay(80);
  }
  Serial.println("[OLED] Boot animation complete.");

  Serial.println("[BLE] Setting up NimBLE server...");
  setupBle();
  Serial.printf("[BLE] Advertising as '%s'\n", DEVICE_NAME);
  Serial.println("=== Setup complete ===");
}

void loop()
{
  // Repaint frequently enough for the waiting/connected animations to
  // read as smooth motion rather than a slideshow, and so the "(stale)"
  // indicator and live-data pulse stay current even without a fresh BLE
  // packet. 120ms (~8fps) is comfortably inside what a 400kHz I2C link
  // can push a 128x64 monochrome frame in, with headroom to spare.
  static unsigned long lastRepaint = 0;
  if (millis() - lastRepaint > 120)
  {
    lastRepaint = millis();
    renderScreen();
  }

  // Periodic heartbeat on the serial monitor so you can confirm the
  // firmware is alive and see connection/nav state at a glance, even
  // when no BLE packets are arriving.
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 3000)
  {
    lastLog = millis();
    Serial.printf("[STATUS] uptime=%lus connected=%s navValid=%s lastPacket=%lums ago\n",
                  millis() / 1000,
                  deviceConnected ? "yes" : "no",
                  nav.valid ? "yes" : "no",
                  nav.valid ? (millis() - nav.lastUpdateMs) : 0);
  }
}
