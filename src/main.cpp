//// NEW CODE ////

/*
  BikeNav TFT Display Firmware
  ESP32-C3 SuperMini + 1.8" SPI TFT (ST7735, 128x160) via Adafruit_GFX/ST7735 + NimBLE

  The display is mounted/used in LANDSCAPE (rotation 1), giving a 160
  (wide) x 128 (tall) logical canvas — the same aspect the layout below
  was designed around. If your panel is wired the other way round from
  what you expect, change TFT_ROTATION below (0-3) rather than rewiring.

  Layout (160x128 landscape):
  ┌──────────────────────────────────────┐
  │ [        ]      Total: 4.2km      (o)│  <- right side, small, top
  │ [  icon  ]                            │
  │ [ (big)  ]           120m             │  <- right side, big
  │            ─────────────────────────  │
  │  55 km/h    Turn right onto Oak St    │  <- right side, small, wraps
  └──────────────────────────────────────┘
  Left column : turn icon (big) on top, speed at the bottom.
  Right column: total distance left (small, top), distance to next
                turn (big), then the turn instruction text as given by
                Google Maps (small, wraps, below that).

  Packet format v2 (matches NavDataState.kt / BleNavClient.kt):
    byte 0   : TurnDir (0-15)
    byte 1-2 : distance to next turn, uint16 big-endian, metres
    byte 3-4 : total journey distance, uint16 big-endian, metres
    byte 5-6 : remaining journey distance, uint16 big-endian, metres
    byte 7   : speed in km/h (0-255)
    byte 8   : roundabout exit angle, 0-255 mapped to 0-360deg (fallback
               only - used for the single frame before a bitmap arrives)
    byte 9   : iconFlag - 1 if a 32x32 1bpp bitmap follows, else 0
    [byte 10..137 : 128-byte packed XBM bitmap, only present if iconFlag==1]
    remaining bytes: "streetName|towardName" ASCII (shown as one string)

  Roundabout icons are drawn from the actual bitmap Google Maps drew in
  its own notification (forwarded by the phone, see IconBitmapConverter.kt
  on the Android side) rather than computed from an angle estimate — same
  approach as maisonsmd's esp32-google-maps project. This sidesteps the
  angle-estimation misfires that came from a coarse icon hash/heuristic;
  Google has already solved "what does a 3rd-exit roundabout look like".
  The angle byte above is kept only as a one-frame fallback.

  TurnDir codes (must match BleUuids / NavAccessibilityService):
    0 = straight/continue     6 = sharp left
    1 = left                  7 = sharp right
    2 = right                 8 = roundabout
    3 = slight left           9 = (reserved / merge, add as needed)
    4 = slight right         10 = (reserved / fork, add as needed)
    5 = u-turn
*/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <NimBLEDevice.h>
#include <math.h>
#include <string.h> // memcpy, for copying the roundabout icon bitmap out of BLE packets

// ---------------- SPI TFT pins ----------------
// Target board: ESP32-C3 SuperMini. The C3 only has one hardware SPI
// (FSPI); its default pins are SCK=4, MISO=5, MOSI=6, SS=7. We call
// SPI.begin() explicitly in setup() below with those pins rather than
// relying on board defaults, so this keeps working even if a different
// C3 board variant ships different defaults. GPIO8/9 are strapping pins
// used at boot (9 = boot mode select) so they're avoided here even though
// they're broken out on the SuperMini silkscreen.
#define TFT_SCK 4
#define TFT_MISO 5
#define TFT_MOSI 6
#define TFT_CS 7
#define TFT_DC 2
#define TFT_RST 3

// Panel is used in landscape, giving a 160(w) x 128(h) logical canvas.
// Rotation 1 = landscape with the ribbon cable on the left; if your text
// comes up upside-down or mirrored, try rotation 3 instead (the other
// landscape orientation) rather than rewiring anything.
#define TFT_ROTATION 1
#define SCREEN_W 160
#define SCREEN_H 128

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);

// Offscreen 16-bit colour framebuffer. Everything below draws into this
// canvas exactly like the old u8g2 clearBuffer()/sendBuffer() model: draw
// a whole frame into RAM, then push it to the panel in one shot at the end
// of renderScreen(). That keeps the single-clean-handoff-per-frame
// behaviour (and therefore the "only loop() touches the display" threading
// rule below) unchanged from the OLED version.
GFXcanvas16 canvas(SCREEN_W, SCREEN_H);

#define COL_BG ST77XX_BLACK
#define COL_FG ST77XX_WHITE
#define COL_ACCENT ST77XX_CYAN

// ---------------- BLE ----------------
// Must match BikeNavApp/BleUuids.kt exactly.
static const char *SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char *NAV_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
// New: dedicated characteristic for the phone to push its clock over,
// separate from the nav data characteristic above so the existing nav
// packet format doesn't need a version bump.
//   6e400003 is already used by BleUuids.MEDIA_CHAR_UUID on the app side
//   (reserved for media/call control), so this uses 6e400004 instead —
//   using 03 here would have silently collided with that.
// Write 2 or 3 bytes: byte 0 = hour (0-23, always 24h on the wire),
// byte 1 = minute (0-59), optional byte 2 = 1 for 12-hour display format,
// 0 (or omitted) for 24-hour. Send once on connect and then periodically
// (e.g. once a minute) is enough - the ESP32 free-runs the clock off
// millis() between updates (see tickClock()).
static const char *TIME_CHAR_UUID = "6e400004-b5a3-f393-e0a9-e50e24dcca9e";
static const char *DEVICE_NAME = "BikeNav";

NimBLEServer *pServer = nullptr;
NimBLECharacteristic *pNavChar = nullptr;
NimBLECharacteristic *pTimeChar = nullptr;
bool deviceConnected = false;

// ---------------- Nav state ----------------
struct NavState
{
  uint8_t turn = 0;
  uint16_t distToTurn = 0; // metres
  uint16_t totalDist = 0;  // metres
  uint16_t remainDist = 0; // metres
  uint8_t speedKmh = 0;
  uint16_t roundaboutAngleDeg = 90; // exit angle fallback, only used until a bitmap arrives
  bool hasIcon = false;             // true if iconBitmap below holds a fresh roundabout icon
  uint8_t iconBitmap[128];          // packed 32x32 1bpp XBM bitmap, the actual icon Maps drew
  String instruction = ""; // "streetName|towardName" or plain text
  bool valid = false;
  unsigned long lastUpdateMs = 0;
};

NavState nav;

// ---------------- Clock (synced from the phone over BLE) ----------------
// The ESP32 has no RTC battery backup and this firmware doesn't join WiFi
// (BLE-only), so it has no time source of its own — the phone (which
// always knows the correct time) pushes its clock over a dedicated BLE
// characteristic (see TimeCharCallbacks / TIME_CHAR_UUID) once right after
// connecting and then every ~60s for as long as it stays connected. That
// keeps this a genuinely real-time clock, not just something set once —
// each sync snaps clockHour/clockMinute to the phone's actual time; the
// millis()-based tick below only fills in the seconds between syncs (or
// keeps roughly-correct time if the BLE link briefly drops). It only shows
// stale/wrong time if the phone has never connected at all since boot, in
// which case it free-runs from 00:00 — the Serial "T HH MM" command below
// is purely a manual fallback for that case (e.g. testing without a phone
// paired yet).
uint8_t clockHour = 0;
uint8_t clockMinute = 0;
unsigned long clockLastTickMs = 0;
bool use12HourFormat = false; // set from the phone's app setting (see TimeCharCallbacks)

void tickClock()
{
  unsigned long now = millis();
  if (now - clockLastTickMs < 60000)
    return;
  clockLastTickMs = now;
  clockMinute++;
  if (clockMinute >= 60)
  {
    clockMinute = 0;
    clockHour = (clockHour + 1) % 24;
  }
}

// Reads a line like "T 14 05" off Serial to set the clock above. Cheap
// and dependency-free; fine for a "set it once after power-up" workflow.
void pollSerialClockSet()
{
  static String line;
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (line.startsWith("T ") || line.startsWith("t "))
      {
        int h = line.substring(2).toInt();
        int spaceIdx = line.indexOf(' ', 2);
        int m = spaceIdx >= 0 ? line.substring(spaceIdx + 1).toInt() : 0;
        if (h >= 0 && h < 24 && m >= 0 && m < 60)
        {
          clockHour = (uint8_t)h;
          clockMinute = (uint8_t)m;
          clockLastTickMs = millis();
          Serial.printf("[CLOCK] Set to %02d:%02d\n", clockHour, clockMinute);
        }
        else
        {
          Serial.println("[CLOCK] Bad time, expected: T HH MM");
        }
      }
      line = "";
    }
    else
    {
      line += c;
    }
  }
}

String currentTimeString()
{
  char buf[9]; // "12:34 PM" + nul
  if (use12HourFormat)
  {
    uint8_t h12 = clockHour % 12;
    if (h12 == 0)
      h12 = 12;
    snprintf(buf, sizeof(buf), "%d:%02d%s", h12, clockMinute, clockHour < 12 ? "AM" : "PM");
  }
  else
  {
    snprintf(buf, sizeof(buf), "%02d:%02d", clockHour, clockMinute);
  }
  return String(buf);
}

// ---------------- Battery ----------------
// No battery-voltage sensing wired up yet, so this only reserves the icon's
// screen space and draws its outline for now, same as the request that
// prompted it — battPercent is plumbed through ready for whenever a
// voltage divider / fuel-gauge IC gets added (fillFrac below is what you'd
// wire that up to).
int battPercent = -1; // -1 = unknown/not wired up yet, draws outline only

void drawBatteryIcon(int x, int y, int w, int h, uint16_t color)
{
  int nubW = 2, nubH = h / 2;
  canvas.drawRect(x, y, w - nubW, h, color);
  canvas.fillRect(x + w - nubW, y + (h - nubH) / 2, nubW, nubH, color);

  if (battPercent >= 0)
  {
    int innerW = w - nubW - 4;
    int fillW = (innerW * battPercent) / 100;
    canvas.fillRect(x + 2, y + 2, fillW, h - 4, color);
  }
}

// Consider data stale if nothing arrives for this long (matches the
// PACKET_TIMEOUT_MS mentioned in the Android heartbeat comments).
static const unsigned long PACKET_TIMEOUT_MS = 20000;

// ---------------- Text helpers ----------------
// Adafruit_GFX's default font, at textSize N, is a fixed 6px-wide
// (5px glyph + 1px gutter) x 8px-tall cell per character, scaled by N.
// These two helpers stand in for u8g2's getStrWidth()/drawStr(), including
// treating y as the text BASELINE (bottom of the glyphs) rather than GFX's
// native top-left cursor, so the rest of the layout code below reads the
// same way it did with u8g2.
int textWidthPx(const String &s, uint8_t size)
{
  return s.length() * 6 * size;
}

void drawStr(int x, int y, const String &s, uint8_t size, uint16_t color = COL_FG)
{
  canvas.setTextSize(size);
  canvas.setTextColor(color);
  canvas.setCursor(x, y - 8 * size);
  canvas.print(s);
}

void drawCentered(const String &text, int y, uint8_t size, uint16_t color = COL_FG)
{
  int w = textWidthPx(text, size);
  int x = (SCREEN_W - w) / 2;
  if (x < 0)
    x = 0;
  drawStr(x, y, text, size, color);
}

// ---------------- Thick-stroke helpers ----------------
// A single 1px line/circle that looked fine against the old 128x64
// monochrome OLED all but disappears on the bigger, higher-resolution
// color TFT — there's just more empty space around each pixel-wide
// stroke, and no natural anti-aliasing glow the way OLED pixels have. GFX
// has no built-in "thick line" primitive, so these draw a small cluster of
// parallel/offset 1px strokes to fake stroke width without pulling in a
// full vector-graphics library.
#define ICON_STROKE 2 // stroke width in px for the turn-icon vector art

void tLine(int x0, int y0, int x1, int y1, uint16_t color)
{
  canvas.drawLine(x0, y0, x1, y1, color);
  for (int o = 1; o < ICON_STROKE; o++)
  {
    // Offsetting in both x and y covers diagonal strokes reasonably well
    // without needing to compute a true perpendicular offset per segment.
    canvas.drawLine(x0 + o, y0, x1 + o, y1, color);
    canvas.drawLine(x0, y0 + o, x1, y1 + o, color);
  }
}

void tCircle(int cx, int cy, int r, uint16_t color)
{
  for (int o = 0; o < ICON_STROKE; o++)
  {
    canvas.drawCircle(cx, cy, r - o, color);
  }
}

void tCircleHelper(int cx, int cy, int r, uint8_t cornername, uint16_t color)
{
  for (int o = 0; o < ICON_STROKE; o++)
  {
    canvas.drawCircleHelper(cx, cy, r - o, cornername, color);
  }
}

// Draws a packed 1bpp XBM bitmap. NOT the same as Adafruit_GFX's own
// drawBitmap(): XBM packs bits LSB-first within each byte (bit0 = leftmost
// pixel of that byte's 8), while Adafruit_GFX's drawBitmap expects
// MSB-first — feeding XBM data straight into drawBitmap() decodes every
// byte backwards. u8g2's drawXBMP (used on the old OLED build) handled
// XBM's native bit order correctly; this is the equivalent for the canvas.
void drawXbm(int x, int y, const uint8_t *bitmap, int w, int h, uint16_t color, int scale = 1)
{
  int rowBytes = (w + 7) / 8;
  for (int row = 0; row < h; row++)
  {
    for (int col = 0; col < w; col++)
    {
      uint8_t b = bitmap[row * rowBytes + (col / 8)];
      if ((b >> (col % 8)) & 0x1)
      {
        if (scale == 1)
        {
          canvas.drawPixel(x + col, y + row, color);
        }
        else
        {
          canvas.fillRect(x + col * scale, y + row * scale, scale, scale, color);
        }
      }
    }
  }
}

// ---------------- Turn icon drawing ----------------
// Medium/large icon drawn with simple vector shapes so it doesn't depend
// on a font/glyph set. Drawn inside a square box (x,y = top-left, size =
// box).
void drawTurnIcon(int x, int y, int size, uint8_t turn)
{
  int cx = x + size / 2;
  int cy = y + size / 2;
  int arm = size / 2 - 2;
  const uint16_t iconColor = COL_ACCENT; // brighter than plain white against the black bg

  // Preferred path for every turn type: draw the actual icon bitmap Google
  // Maps rendered, forwarded by the phone (IconBitmapConverter.kt / packet
  // format v2). No more classifying into a fixed set of vector shapes -
  // whatever Maps drew is what shows up here, same approach as maisonsmd's
  // esp32-google-maps project. Falls through to the vector icons below
  // only when no bitmap has arrived yet (e.g. very first frame of a fresh
  // instruction, before the phone has captured+sent the icon).
  if (nav.hasIcon)
  {
    // The bitmap is a packed 32x32 1bpp XBM (same as before); GFX's
    // drawBitmap draws 1-bits as the given colour and leaves 0-bits alone,
    // matching u8g2.drawXBMP's behaviour on our black background.
    // The forwarded bitmap is always a fixed 32x32 1bpp XBM (see
    // IconBitmapConverter.kt / packet format v2), independent of however
    // big the on-screen icon box is. Draw it at 2x scale (64x64) so it
    // reads at roughly the same size as the enlarged vector icons below,
    // centered in the box — rather than telling the draw call the box
    // size directly (which reads the packed bytes with the wrong row
    // stride and produces garbage).
    const int bmW = 32, bmH = 32, bmScale = 2;
    int bx = x + (size - bmW * bmScale) / 2;
    int by = y + (size - bmH * bmScale) / 2;
    drawXbm(bx, by, nav.iconBitmap, bmW, bmH, iconColor, bmScale);
    return;
  }

  switch (turn)
  {
  case 1: // turn-left: clean 90-degree corner (up, then left)
    tLine(cx, cy + arm, cx, cy - 2, iconColor);
    tLine(cx, cy - 2, cx - arm, cy - 2, iconColor);
    tLine(cx - arm, cy - 2, cx - arm + 7, cy - 2 - 7, iconColor);
    tLine(cx - arm, cy - 2, cx - arm + 7, cy - 2 + 7, iconColor);
    break;
  case 2: // turn-right: mirror
    tLine(cx, cy + arm, cx, cy - 2, iconColor);
    tLine(cx, cy - 2, cx + arm, cy - 2, iconColor);
    tLine(cx + arm, cy - 2, cx + arm - 7, cy - 2 - 7, iconColor);
    tLine(cx + arm, cy - 2, cx + arm - 7, cy - 2 + 7, iconColor);
    break;
  case 3: // slight-left: mostly vertical, small kink near the top (<90 deg bend)
    tLine(cx, cy + arm, cx, cy - arm / 3, iconColor);
    tLine(cx, cy - arm / 3, cx - arm, cy - arm, iconColor);
    tLine(cx - arm, cy - arm, cx - arm + 7, cy - arm + 2, iconColor);
    tLine(cx - arm, cy - arm, cx - arm + 2, cy - arm + 7, iconColor);
    break;
  case 4: // slight-right: mirror
    tLine(cx, cy + arm, cx, cy - arm / 3, iconColor);
    tLine(cx, cy - arm / 3, cx + arm, cy - arm, iconColor);
    tLine(cx + arm, cy - arm, cx + arm - 7, cy - arm + 2, iconColor);
    tLine(cx + arm, cy - arm, cx + arm - 2, cy - arm + 7, iconColor);
    break;
  case 5: // sharp-left: bends well PAST 90 degrees — exits below the
          // pivot height, reading as a much tighter hook than a plain turn
    tLine(cx, cy + arm, cx, cy - 4, iconColor);
    tLine(cx, cy - 4, cx - arm, cy + arm / 3, iconColor);
    tLine(cx - arm, cy + arm / 3, cx - arm + 8, cy + arm / 3 - 3, iconColor);
    tLine(cx - arm, cy + arm / 3, cx - arm + 4, cy + arm / 3 + 7, iconColor);
    break;
  case 6: // sharp-right: mirror
    tLine(cx, cy + arm, cx, cy - 4, iconColor);
    tLine(cx, cy - 4, cx + arm, cy + arm / 3, iconColor);
    tLine(cx + arm, cy + arm / 3, cx + arm - 8, cy + arm / 3 - 3, iconColor);
    tLine(cx + arm, cy + arm / 3, cx + arm - 4, cy + arm / 3 + 7, iconColor);
    break;
  case 7: // uturn-left: hook curving back to the left
    tCircleHelper(cx, cy - 2, arm - 2, 1 | 2, iconColor); // upper-right | upper-left
    tLine(cx - (arm - 2), cy - 2, cx - (arm - 2), cy + arm - 4, iconColor);
    tLine(cx - (arm - 2), cy + arm - 4, cx - (arm - 2) - 5, cy + arm - 9, iconColor);
    tLine(cx - (arm - 2), cy + arm - 4, cx - (arm - 2) + 5, cy + arm - 9, iconColor);
    break;
  case 8: // uturn-right: mirror
    tCircleHelper(cx, cy - 2, arm - 2, 1 | 2, iconColor);
    tLine(cx + (arm - 2), cy - 2, cx + (arm - 2), cy + arm - 4, iconColor);
    tLine(cx + (arm - 2), cy + arm - 4, cx + (arm - 2) - 5, cy + arm - 9, iconColor);
    tLine(cx + (arm - 2), cy + arm - 4, cx + (arm - 2) + 5, cy + arm - 9, iconColor);
    break;
  case 9:  // roundabout (generic — kept as a code for backward
  case 10: // compatibility)
  {
    // The nav.hasIcon bitmap path is already handled at the top of this
    // function for every turn code. Everything below only runs when no
    // bitmap has arrived yet - the old angle-based vector arrow, kept as a
    // one-frame fallback using whatever angle we last had, so the screen
    // shows *something* sensible instead of a blank circle.
    int r = arm - 4;
    tCircle(cx, cy, r, iconColor);

    // Entry is always drawn from the bottom (south) — that's "you,
    // arriving at the roundabout". This is fixed regardless of exit.
    tLine(cx, cy + arm, cx, cy + r, iconColor);

    // Exit is drawn at the actual angle, measured clockwise from north
    // (straight up = continue straight across = 0deg/360deg). This is
    // what makes 45/90/135/180/225/270/315-degree exits all look
    // distinct instead of collapsing into just two icon variants.
    float rad = radians((float)nav.roundaboutAngleDeg);
    int exitStartX = cx + (int)(r * 0.7f * sin(rad));
    int exitStartY = cy - (int)(r * 0.7f * cos(rad));
    int exitEndX = cx + (int)(arm * sin(rad));
    int exitEndY = cy - (int)(arm * cos(rad));
    tLine(exitStartX, exitStartY, exitEndX, exitEndY, iconColor);

    // Arrowhead: two short ticks angled back from the exit line's own
    // direction, so they stay correctly oriented at any exit angle
    // instead of only looking right for a fixed left/right case.
    float back1 = rad + radians(150.0f);
    float back2 = rad - radians(150.0f);
    int p1x = exitEndX + (int)(6 * sin(back1));
    int p1y = exitEndY - (int)(6 * cos(back1));
    int p2x = exitEndX + (int)(6 * sin(back2));
    int p2y = exitEndY - (int)(6 * cos(back2));
    tLine(exitEndX, exitEndY, p1x, p1y, iconColor);
    tLine(exitEndX, exitEndY, p2x, p2y, iconColor);
    break;
  }
  case 11: // merge: two lines converging into one, continuing up
    tLine(cx - arm, cy + arm, cx, cy, iconColor);
    tLine(cx + arm - 6, cy + arm, cx, cy, iconColor);
    tLine(cx, cy, cx, cy - arm, iconColor);
    tLine(cx, cy - arm, cx - 5, cy - arm + 6, iconColor);
    tLine(cx, cy - arm, cx + 5, cy - arm + 6, iconColor);
    break;
  case 12: // fork-left: Y splitting, left branch is the taken (arrowed) one
    tLine(cx, cy + arm, cx, cy, iconColor);
    tLine(cx, cy, cx - arm, cy - arm, iconColor);
    tLine(cx - arm, cy - arm, cx - arm + 7, cy - arm + 2, iconColor);
    tLine(cx - arm, cy - arm, cx - arm + 2, cy - arm + 7, iconColor);
    tLine(cx, cy, cx + arm - 6, cy - arm + 6, iconColor); // thin untaken branch
    break;
  case 13: // fork-right: mirror
    tLine(cx, cy + arm, cx, cy, iconColor);
    tLine(cx, cy, cx + arm, cy - arm, iconColor);
    tLine(cx + arm, cy - arm, cx + arm - 7, cy - arm + 2, iconColor);
    tLine(cx + arm, cy - arm, cx + arm - 2, cy - arm + 7, iconColor);
    tLine(cx, cy, cx - arm + 6, cy - arm + 6, iconColor); // thin untaken branch
    break;
  case 14: // ramp-left: slight-left with a curved ramp line alongside
    tLine(cx + 3, cy + arm, cx + 3, cy, iconColor);
    tCircleHelper(cx - arm + 6, cy - 3, arm - 4, 1, iconColor); // upper-right
    tLine(cx - arm + 6, cy - arm + 1, cx - arm + 13, cy - arm + 3, iconColor);
    tLine(cx - arm + 6, cy - arm + 1, cx - arm + 8, cy - arm + 8, iconColor);
    break;
  case 15: // ramp-right: mirror
    tLine(cx - 3, cy + arm, cx - 3, cy, iconColor);
    tCircleHelper(cx + arm - 6, cy - 3, arm - 4, 2, iconColor); // upper-left
    tLine(cx + arm - 6, cy - arm + 1, cx + arm - 13, cy - arm + 3, iconColor);
    tLine(cx + arm - 6, cy - arm + 1, cx + arm - 8, cy - arm + 8, iconColor);
    break;
  case 16: // arrived: simple checkmark
    tLine(cx - arm + 2, cy, cx - 3, cy + arm - 4, iconColor);
    tLine(cx - 3, cy + arm - 4, cx + arm - 2, cy - arm + 4, iconColor);
    break;
  case 0: // straight
  default:
    tLine(cx, cy + arm, cx, cy - arm, iconColor);
    tLine(cx, cy - arm, cx - 6, cy - arm + 6, iconColor);
    tLine(cx, cy - arm, cx + 6, cy - arm + 6, iconColor);
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

// Word-wrap a string into up to maxLines lines that fit width pixels at
// the given text size, writing them starting at (x, yFirstBaseline) with
// lineHeight spacing.
void drawWrapped(const String &text, int x, int yFirstBaseline, int width,
                  int lineHeight, int maxLines, uint8_t size)
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
      if (textWidthPx(candidate, size) > width)
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
    drawStr(x, yFirstBaseline + lines * lineHeight, line, size);
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
  canvas.fillCircle(cx, cy, r, COL_ACCENT);
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

// ---------------- Screens ----------------
// Coordinates below target the 160x128 landscape canvas (see SCREEN_W/H
// above); this is a straight re-layout of the original 128x64 OLED screens
// onto the larger, higher-resolution TFT canvas rather than a pixel scale,
// so text sizes and spacing are chosen to look right at 160x128 rather
// than stretched from the old panel's proportions.

// Reserved band across the very top of every screen: time + battery
// (top-right), and optionally a left-aligned label (e.g. "Total: 4.2km")
// for screens that have something to say there. Pass "" for leftLabel to
// leave that side blank.
const int STATUS_BAR_H = 16;
const int BATT_W = 20, BATT_H = 10;

void drawStatusBar(const String &leftLabel)
{
  if (leftLabel.length() > 0)
  {
    drawStr(4, 12, leftLabel, 1);
  }

  int battX = SCREEN_W - 4 - BATT_W;
  int battY = 3;
  drawBatteryIcon(battX, battY, BATT_W, BATT_H, COL_FG);

  String t = currentTimeString();
  int tW = textWidthPx(t, 1);
  drawStr(battX - 6 - tW, 12, t, 1);
}

// Route-progress bar along the very bottom of every screen: outline plus
// a filled portion for how much of the total route distance is done.
// fraction is clamped to [0,1] here so callers don't each have to guard
// against totalDist==0 or a stale/negative remaining distance.
const int PROGRESS_BAR_H = 6;

void drawProgressBar(float fraction)
{
  if (fraction < 0)
    fraction = 0;
  if (fraction > 1)
    fraction = 1;

  int x = 4, w = SCREEN_W - 8;
  int y = SCREEN_H - PROGRESS_BAR_H - 4;
  canvas.drawRect(x, y, w, PROGRESS_BAR_H, COL_FG);
  int fillW = (int)((w - 2) * fraction);
  if (fillW > 0)
  {
    canvas.fillRect(x + 1, y + 1, fillW, PROGRESS_BAR_H - 2, COL_ACCENT);
  }
}

// Shown while advertising but no phone has connected yet.
void renderWaitingForConnection()
{
  drawStatusBar("");

  drawCentered("BikeNav", 50, 2);

  drawBreathingDot(80, 76, 4, 9, 1400);

  String line = "Waiting for phone" + loadingDots(1200, 3);
  drawCentered(line, 110, 1);
}

// Shown once the phone is connected but no navigation packet has arrived
// (or navigation session has ended) yet.
void renderConnectedWaitingForNav()
{
  drawStatusBar("");

  drawCentered("Connected", 50, 2);

  drawBreathingDot(80, 76, 4, 9, 1000);

  String line = "Move to start nav" + loadingDots(1200, 3);
  drawCentered(line, 110, 1);
}

// Shown when nav.turn == 16 (ARRIVED) - deliberately just a checkmark and
// one word, not the full turn/distance layout, since there's no more
// turn-by-turn info left to show at this point. Route is done, so the
// progress bar is drawn full.
void renderArrived()
{
  drawStatusBar("");

  drawTurnIcon(48, STATUS_BAR_H + 2, 62, 16);

  drawCentered("Arrived", 108, 2);

  drawProgressBar(1.0f);
}

// The normal in-navigation screen - turn icon, speed, distances,
// instruction text - plus the status bar and route-progress bar.
void renderNavigationActive()
{
  bool stale = millis() - nav.lastUpdateMs > PACKET_TIMEOUT_MS;

  // Total distance remaining goes in the status bar's left slot now,
  // alongside time/battery on the right, instead of its own line — frees
  // up the whole content area below for icon + instruction.
  // "Total" now genuinely means the whole route's distance (captured once
  // per navigation session on the app side), not remaining-to-go — see
  // NavDataState.kt's sessionTotalDist. The progress bar at the bottom is
  // what shows how much of that total has been covered so far.
  drawStatusBar("Total: " + formatDistance(nav.totalDist));

  const int contentTop = STATUS_BAR_H;
  const int contentBottom = SCREEN_H - PROGRESS_BAR_H - 8;

  // ---- Left column: turn icon (big) + speed below ----
  const int leftColW = 72;
  const int iconSize = 58;
  const int iconX = 7;
  const int iconY = contentTop;
  drawTurnIcon(iconX, iconY, iconSize, nav.turn);

  char speedBuf[8];
  snprintf(speedBuf, sizeof(speedBuf), "%dkm/h", nav.speedKmh);
  // Center the speed text under the icon.
  int speedW = textWidthPx(speedBuf, 2);
  int speedX = iconX + (iconSize - speedW) / 2;
  if (speedX < 0)
    speedX = 0;
  drawStr(speedX, contentBottom, speedBuf, 2);

  // Vertical divider between the two columns, spanning just the content
  // area (not through the status bar or progress bar).
  canvas.drawFastVLine(leftColW, contentTop, contentBottom - contentTop, COL_FG);

  // ---- Right column: next-turn distance / instruction ----
  const int rightX = leftColW + 6;
  const int rightW = SCREEN_W - rightX - 4;

  // Distance to next turn — big, this is the number that matters most
  // moment-to-moment so it gets the biggest type in the right column.
  String nextLine = formatDistance(nav.distToTurn);
  drawStr(rightX, contentTop + 30, nextLine, 3, COL_ACCENT);

  canvas.drawFastHLine(rightX, contentTop + 36, rightW, COL_FG);

  // Instruction text (e.g. "Turn right onto Oak Street"), wrapped, small
  // type so 2-3 lines fit under the big distance readout.
  String instr = nav.instruction;
  instr.replace("|", " ");
  drawWrapped(instr, rightX, contentTop + 52, rightW, 14, 3, 1);

  if (stale)
  {
    drawStr(rightX, contentBottom, "(stale)", 1);
  }

  // Route progress: how much of the total journey distance is behind us.
  // Guards totalDist==0 (no route loaded yet / packet not carrying it)
  // by just drawing an empty bar via drawProgressBar's own clamping.
  float progress = 0.0f;
  if (nav.totalDist > 0)
  {
    progress = (float)(nav.totalDist - nav.remainDist) / (float)nav.totalDist;
  }
  drawProgressBar(progress);
}

// ---------------- Rendering dispatcher ----------------
// Picks which screen to draw based on connection/nav state. Called
// frequently (see loop()) so the animated screens stay smooth; each
// screen function only draws into the offscreen canvas; the canvas is
// blitted to the panel once here so there's a single clean handoff to the
// display per frame (same discipline as the old u8g2 clearBuffer/
// sendBuffer pairing).
void renderScreen()
{
  canvas.fillScreen(COL_BG);

  if (!deviceConnected)
  {
    renderWaitingForConnection();
  }
  else if (!nav.valid)
  {
    renderConnectedWaitingForNav();
  }
  else if (nav.turn == 16) // ARRIVED
  {
    renderArrived();
  }
  else
  {
    renderNavigationActive();
  }

  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), SCREEN_W, SCREEN_H);
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

  // Byte 8 (when present) is the roundabout exit angle, 0-255 mapped to
  // 0-360 degrees, clockwise from straight-ahead. Only meaningful when
  // turn is a roundabout code, but always present/parsed if the packet
  // is long enough, for a consistent format. Packets from older app
  // builds that don't send this byte still work — angle just stays at
  // its default (90 deg / generic right-ish exit) and instruction text
  // is read starting at byte 8 instead of 9, same as before.
  size_t textStart;
  nav.hasIcon = false; // reset each packet; only set true below if a fresh bitmap is present
  if (len > 8)
  {
    nav.roundaboutAngleDeg = (uint16_t)((data[8] * 360UL) / 255UL);
    textStart = 9;

    // Byte 9 (when present): iconFlag. 1 means a 128-byte packed 32x32
    // 1bpp bitmap immediately follows — the actual roundabout icon Maps
    // drew, forwarded from the phone (see IconBitmapConverter.kt). We only
    // trust it if the packet is actually long enough to hold the full 128
    // bytes; a truncated/malformed packet just falls back to the angle
    // (and ultimately the vector arrow) instead of reading garbage memory.
    if (len > 9)
    {
      uint8_t iconFlag = data[9];
      textStart = 10;
      if (iconFlag == 1 && len >= 10 + sizeof(nav.iconBitmap))
      {
        memcpy(nav.iconBitmap, data + 10, sizeof(nav.iconBitmap));
        nav.hasIcon = true;
        textStart = 10 + sizeof(nav.iconBitmap);
      }
    }
  }
  else
  {
    textStart = 8;
  }

  if (len > textStart)
  {
    nav.instruction = String((const char *)(data + textStart), len - textStart);
  }
  else
  {
    nav.instruction = "";
  }

  // Only leave the "Move to start nav" idle screen once there's actually
  // something meaningful to show. The phone can send an early packet
  // before GPS/route data has settled (distToTurn=0, empty instruction) -
  // treating that as "valid" flashed a blank/broken-looking nav screen for
  // a moment instead of staying on the idle screen until real data arrives.
  bool hasMeaningfulData = nav.distToTurn > 0 || nav.instruction.length() > 0;
  if (hasMeaningfulData)
  {
    nav.valid = true;
  }
  nav.lastUpdateMs = millis();

  Serial.printf("[NAV] turn=%d distToTurn=%um total=%um remain=%um speed=%ukmh roundaboutAngle=%udeg instr='%s'\n",
                nav.turn, nav.distToTurn, nav.totalDist, nav.remainDist,
                nav.speedKmh, nav.roundaboutAngleDeg, nav.instruction.c_str());

  // Deliberately NOT calling renderScreen() here. NimBLE callbacks run on
  // their own FreeRTOS task, separate from the Arduino loop() task; with
  // loop() now repainting every 120ms for the animations, having two
  // tasks both touch the SPI bus/canvas at once would be exactly the kind
  // of cross-task glitching the old I2C version had to work around.
  // loop()'s repaint picks up this data within 120ms on its own -
  // imperceptible, and confines all display/SPI access to a single task.
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

class TimeCharCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override
  {
    std::string value = pChar->getValue();
    if (value.length() < 2)
    {
      Serial.println("[CLOCK] Time packet too short (<2 bytes), ignoring.");
      return;
    }
    uint8_t h = (uint8_t)value[0];
    uint8_t m = (uint8_t)value[1];
    if (h < 24 && m < 60)
    {
      clockHour = h;
      clockMinute = m;
      clockLastTickMs = millis(); // resync the millis()-based tick to this fresh reading
      if (value.length() >= 3)
      {
        use12HourFormat = (uint8_t)value[2] != 0;
      }
      Serial.printf("[CLOCK] Synced from phone: %02d:%02d (%s)\n", clockHour, clockMinute,
                    use12HourFormat ? "12h" : "24h");
    }
    else
    {
      Serial.printf("[CLOCK] Bad time from phone: %u:%u, ignoring.\n", h, m);
    }
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

  pTimeChar = pService->createCharacteristic(
      TIME_CHAR_UUID,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pTimeChar->setCallbacks(new TimeCharCallbacks());

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

// ---------------- Arduino lifecycle ----------------
void setup()
{
  Serial.begin(115200);
  delay(1000); // give the serial monitor time to attach
  Serial.println();
  Serial.println("=== BikeNav Display booting ===");

  // ESP32-C3 has only one hardware SPI bus and no fixed "VSPI/HSPI"
  // default pins guaranteed across all board variants, so wire it up
  // explicitly to the SuperMini pins defined above before touching the TFT.
  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);

  Serial.println("[TFT] Initializing ST7735...");
  // INITR_BLACKTAB is the right init sequence for the vast majority of
  // 1.8" 128x160 SPI TFT modules sold as "ST7735 1.8 TFT" (the ones with a
  // black tab sticker on the ribbon). If colours look off/inverted or the
  // image is offset by a few pixels, your particular panel may need
  // INITR_GREENTAB, INITR_REDTAB, or INITR_144GREENTAB instead — swap this
  // one line and re-flash.
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(COL_BG);
  Serial.println("[TFT] Init complete.");

  // Animated boot splash - blocks briefly here, which is fine since BLE
  // isn't set up yet and nothing else needs the CPU. Kept short (~1.2s)
  // so it reads as a boot flourish, not a delay.
  unsigned long bootStart = millis();
  while (millis() - bootStart < 1200)
  {
    canvas.fillScreen(COL_BG);
    drawCentered("BikeNav", 60, 2);
    drawCentered("booting" + loadingDots(600, 3), 84, 1);
    tft.drawRGBBitmap(0, 0, canvas.getBuffer(), SCREEN_W, SCREEN_H);
    delay(80);
  }
  Serial.println("[TFT] Boot animation complete.");

  Serial.println("[BLE] Setting up NimBLE server...");
  setupBle();
  Serial.printf("[BLE] Advertising as '%s'\n", DEVICE_NAME);
  Serial.println("[CLOCK] Waiting for phone to sync time over BLE (falls back to Serial 'T HH MM' if never connected).");
  Serial.println("=== Setup complete ===");
}

void loop()
{
  tickClock();
  pollSerialClockSet();

  // Repaint frequently enough for the waiting/connected animations to
  // read as smooth motion rather than a slideshow, and so the "(stale)"
  // indicator and live-data pulse stay current even without a fresh BLE
  // packet. 120ms (~8fps) is comfortably inside what hardware SPI can
  // push a 160x128 16-bit-colour frame in (roughly 40KB/frame), with
  // headroom to spare.
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
