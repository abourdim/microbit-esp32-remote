/*
 * Micro:bit Remote Builder — ESP32 firmware
 *
 * Speaks the same BLE protocol as a BBC micro:bit running the rxy
 * MakeCode template, so the unmodified web app at
 *   https://abourdim.github.io/rxy/
 * can connect to an ESP32 and drive widgets exactly as it would a micro:bit.
 *
 * Target board: ESP32-C3 Super Mini (any ESP32 family chip with BLE works).
 * Library:      NimBLE-Arduino (Library Manager → search "NimBLE-Arduino").
 *               Tested against NimBLE-Arduino 2.x.
 *
 * HOW TO USE
 * ----------
 *  1. Open the rxy web app, switch to the "Build" tab, design your remote.
 *  2. Click "📄 Code". In the generated MakeCode, find the line
 *        const CFG = "..."
 *     Copy the long base64 string.
 *  3. Paste it into LAYOUT_CFG_BASE64 below, replacing the default.
 *  4. Fill in your widget logic in handleWidget().
 *  5. Upload to the ESP32, then in the rxy "Play" tab click "📡 Connect"
 *     and pick "BBC micro:bit ESP32" from the chooser.
 *
 * PROTOCOL (reverse-audited from rxy script.js)
 * ---------------------------------------------
 *  Service:    6e400001-b5a3-f393-e0a9-e50e24dcca9e
 *  Notify:     6e400002-...   (device → app, the micro:bit "TX" characteristic)
 *  Write:      6e400003-...   (app → device, the micro:bit "RX" characteristic)
 *  Encoding:   ASCII lines, '\n' terminated. CRLF tolerated.
 *  App → ESP:  "SET <id> <val>"   or   "GETCFG"
 *  ESP → App:  "UPD <id> <val>"
 *              "CFGBEGIN" / "CFG <chunk>"... / "CFGEND"
 *
 *  Note: the role of characteristics 0002 / 0003 is the OPPOSITE of the
 *  Nordic UART Service convention used by Adafruit/Bluefruit. We follow the
 *  micro:bit's convention here because the web app expects it.
 */

#include <Arduino.h>
#include <NimBLEDevice.h>

// =====================================================================
//  CONFIGURATION  —  edit this section for your project
// =====================================================================

// Advertised BLE name. MUST start with "BBC micro:bit" or the rxy app
// will not show this device in the connection chooser. The name is short
// on purpose: BLE advertisement packets are only 31 bytes, and combined
// with the 128-bit UART service UUID the budget is tight. Anything longer
// gets silently dropped by some BLE stacks. The rxy filter is a
// namePrefix match, so this exact string qualifies.
static const char* BLE_DEVICE_NAME = "BBC micro:bit";

// Paste the base64 layout string from the rxy "📄 Code" button here.
// The default below is a minimal layout with a single "Test" button.
static const char* LAYOUT_CFG_BASE64 =
  "eyJ0aXRsZSI6Ik15IFJlbW90ZSIsIndpZGdldHMiOlt7ImlkIjoiYnRuX3Rlc3"
  "QiLCJ0IjoiYnV0dG9uIiwieCI6NTAsInkiOjUwLCJ3IjoxMDAsImgiOjEwMCwi"
  "bGFiZWwiOiJUZXN0IiwibW9kZWwiOiJuZW8ifV19";

// On the ESP32-C3 Super Mini the on-board blue LED is wired to GPIO 8
// and is ACTIVE LOW (write LOW to turn it on). Adjust for your board.
static const int LED_PIN          = 8;
static const int LED_ON           = LOW;
static const int LED_OFF          = HIGH;

// =====================================================================
//  BLE UUIDs  —  DO NOT CHANGE (these are the micro:bit's UART service)
// =====================================================================
#define UART_SERVICE_UUID   "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define UART_TX_CHAR_UUID   "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // notify
#define UART_RX_CHAR_UUID   "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // write

// =====================================================================
//  State
// =====================================================================
static NimBLECharacteristic* gTxChar    = nullptr;
static volatile bool         gConnected = false;
static String                gRxBuffer;

// Forward declarations
static void handleLine(const String& line);
static void handleWidget(const String& id, const String& val);
static void sendLine(const String& line);
static void sendCfg();

/**
 * Send "UPD <id> <val>" to the app. Use this to update output widgets
 * like LEDs, labels, gauges, graphs, and battery indicators.
 */
static inline void sendValue(const String& id, const String& val) {
  sendLine("UPD " + id + " " + val);
}

// =====================================================================
//  BLE callbacks
// =====================================================================
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* /*server*/, NimBLEConnInfo& /*info*/) override {
    gConnected = true;
    gRxBuffer  = "";
    Serial.println("[BLE] Client connected");
  }
  void onDisconnect(NimBLEServer* /*server*/, NimBLEConnInfo& /*info*/, int reason) override {
    gConnected = false;
    Serial.printf("[BLE] Client disconnected (reason 0x%02x)\n", reason);
    NimBLEDevice::startAdvertising();
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& /*info*/) override {
    std::string v = chr->getValue();
    for (size_t i = 0; i < v.size(); ++i) {
      const char c = v[i];
      if (c == '\r') continue;
      if (c == '\n') {
        if (gRxBuffer.length() > 0) {
          handleLine(gRxBuffer);
          gRxBuffer = "";
        }
      } else {
        gRxBuffer += c;
        if (gRxBuffer.length() > 256) gRxBuffer = "";  // overflow guard
      }
    }
  }
};

// =====================================================================
//  Protocol
// =====================================================================
static void sendLine(const String& line) {
  if (!gConnected || gTxChar == nullptr) return;
  String out = line + "\n";
  gTxChar->setValue((const uint8_t*)out.c_str(), out.length());
  gTxChar->notify();
}

static void sendCfg() {
  sendLine("CFGBEGIN");
  const char* p   = LAYOUT_CFG_BASE64;
  const size_t n  = strlen(p);
  const size_t CHUNK = 18;  // matches the official rxy MakeCode template
  for (size_t i = 0; i < n; i += CHUNK) {
    String line = "CFG ";
    for (size_t j = 0; j < CHUNK && (i + j) < n; ++j) line += p[i + j];
    sendLine(line);
    delay(8);  // pace notifications so the BLE stack does not drop any
  }
  sendLine("CFGEND");
  Serial.println("[BLE] Sent CFG");
}

static void handleLine(const String& line) {
  Serial.print("[RX] ");
  Serial.println(line);

  if (line == "GETCFG") { sendCfg(); return; }

  if (line.startsWith("SET ")) {
    int sp = line.indexOf(' ', 4);
    if (sp < 0) return;
    String id  = line.substring(4, sp);
    String val = line.substring(sp + 1);
    handleWidget(id, val);
  }
}

// =====================================================================
//  WIDGET HANDLERS  —  edit this for your project
// =====================================================================
//
// Widget value formats (from the rxy README + audit):
//
//   Button    "btn_..."     val: "0" | "1"
//   Slider    "slider_..."  val: integer in the widget's min..max range
//   Toggle    "toggle_..."  val: "0" | "1"
//   Joystick  "joy_..."     val: "<angle 0-360> <distance 0-100>"
//                                  (angle: 0°=right, 90°=down, 180°=left, 270°=up)
//   D-Pad     "dpad_..."    val: "<up|down|left|right> <0|1>"
//   XY Pad    "xypad_..."   val: "<x 0-100> <y 0-100>"
//   Timer     "timer_..."   val: seconds elapsed (sent ~every 5 s)
//
// To update an output widget (LED, label, gauge, graph, battery)
// call sendValue() from anywhere:
//
//   sendValue("led_status",  "1");
//   sendValue("gauge_temp",  "23");
//   sendValue("label_score", "Score: 42");
//   sendValue("graph_data",  "12,7,18");
//   sendValue("battery_lvl", "75");

static void handleWidget(const String& id, const String& val) {

  // ------- Example: built-in LED follows the "Test" button -------------
  if (id == "btn_test") {
    digitalWrite(LED_PIN, val == "1" ? LED_ON : LED_OFF);
    return;
  }

  // ------- Buttons -----------------------------------------------------
  // if (id == "btn_fire" && val == "1") { /* fire! */ }

  // ------- Sliders -----------------------------------------------------
  // if (id == "slider_speed") {
  //   int s = val.toInt();          // 0..100 by default
  //   analogWrite(MOTOR_PIN, s * 255 / 100);
  //   return;
  // }

  // ------- Toggles -----------------------------------------------------
  // if (id == "toggle_turbo") {
  //   bool on = (val == "1");
  //   ...
  //   return;
  // }

  // ------- Joystick: "angle distance" ----------------------------------
  // if (id == "joy_move") {
  //   int sp = val.indexOf(' ');
  //   int angle = val.substring(0, sp).toInt();
  //   int dist  = val.substring(sp + 1).toInt();
  //   ...
  //   return;
  // }

  // ------- D-Pad: "direction state" ------------------------------------
  // if (id == "dpad_nav") {
  //   int sp = val.indexOf(' ');
  //   String dir = val.substring(0, sp);          // "up" | "down" | "left" | "right"
  //   bool  down = val.substring(sp + 1) == "1";
  //   ...
  //   return;
  // }

  // ------- XY Pad: "x y" (0..100 each) ---------------------------------
  // if (id == "xypad_aim") {
  //   int sp = val.indexOf(' ');
  //   int x = val.substring(0, sp).toInt();
  //   int y = val.substring(sp + 1).toInt();
  //   ...
  //   return;
  // }

  // ------- Timer: seconds elapsed --------------------------------------
  // if (id == "timer_game") {
  //   int secs = val.toInt();
  //   ...
  //   return;
  // }
}

// =====================================================================
//  Setup / Loop
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println("=== Micro:bit Remote — ESP32 firmware ===");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);   // max TX power for range
  Serial.printf("[BLE] Local MAC: %s\n",
                NimBLEDevice::getAddress().toString().c_str());

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* svc = server->createService(UART_SERVICE_UUID);

  gTxChar = svc->createCharacteristic(
              UART_TX_CHAR_UUID,
              NIMBLE_PROPERTY::NOTIFY);

  NimBLECharacteristic* rxChar = svc->createCharacteristic(
              UART_RX_CHAR_UUID,
              NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rxChar->setCallbacks(new RxCallbacks());

  svc->start();

  // ---------------------------------------------------------------
  // Advertising:
  //   - Primary packet:   flags + 128-bit service UUID
  //   - Scan response:    the name (kept here to guarantee it survives)
  // The rxy web app filters scans by namePrefix "BBC micro:bit", which
  // matches whether the name is in the primary packet or the scan response.
  // ---------------------------------------------------------------
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(UART_SERVICE_UUID);
  adv->setName(BLE_DEVICE_NAME);
  adv->enableScanResponse(true);
  NimBLEDevice::startAdvertising();

  Serial.printf("[BLE] Advertising as '%s'\n", BLE_DEVICE_NAME);
  Serial.println("[BLE] Open https://abourdim.github.io/rxy/ and click Connect.");
}

void loop() {
  // -----------------------------------------------------------------
  // Periodic update example. Uncomment + adapt for your project.
  // The rxy web app rate-limits incoming messages to one per 200 ms,
  // so do not flood it. A 200–500 ms cadence is comfortable.
  // -----------------------------------------------------------------
  // static uint32_t last = 0;
  // if (gConnected && millis() - last > 500) {
  //   last = millis();
  //   sendValue("gauge_temp", String((int)temperatureRead()));
  // }
}
