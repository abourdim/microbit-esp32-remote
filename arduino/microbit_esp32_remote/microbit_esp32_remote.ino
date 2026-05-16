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
#include <esp_system.h>
#include <esp_chip_info.h>

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

// Debug button — when pressed, dumps the current LAYOUT_CFG_BASE64 to
// Serial in the same "CFGBEGIN / CFG <18-char> / CFGEND" framing that
// is sent over BLE. Useful to confirm visually that the layout you
// pasted is what is actually running, without needing a BLE connect.
// GPIO0 with INPUT_PULLUP — pressed = LOW.
static const int BUTTON_PIN       = 0;
static const int BUTTON_ACTIVE    = LOW;

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
  void onConnect(NimBLEServer* /*server*/, NimBLEConnInfo& info) override {
    gConnected = true;
    gRxBuffer  = "";
    Serial.printf("[BLE] Client connected  peer=%s\n",
                  info.getAddress().toString().c_str());
  }
  void onDisconnect(NimBLEServer* /*server*/, NimBLEConnInfo& /*info*/, int reason) override {
    gConnected = false;
    Serial.printf("[BLE] Client disconnected (reason 0x%02x) — re-advertising\n", reason);
    NimBLEDevice::startAdvertising();
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo& /*info*/) override {
    Serial.printf("[BLE] MTU negotiated: %u\n", mtu);
  }
};

// Human-readable name for an ESP reset reason code.
static const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT (external pin)";
    case ESP_RST_SW:        return "SW (esp_restart)";
    case ESP_RST_PANIC:     return "PANIC (exception/crash)";
    case ESP_RST_INT_WDT:   return "INT_WDT (interrupt watchdog)";
    case ESP_RST_TASK_WDT:  return "TASK_WDT (task watchdog)";
    case ESP_RST_WDT:       return "WDT (other watchdog)";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP wakeup";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    case ESP_RST_USB:       return "USB host reset";
    case ESP_RST_JTAG:      return "JTAG";
    default:                return "UNKNOWN";
  }
}

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

// Print the current LAYOUT_CFG_BASE64 to Serial, framed the same way
// it is sent over BLE. Call from the button handler.
static void printCfg() {
  const size_t n = strlen(LAYOUT_CFG_BASE64);
  const size_t CHUNK = 18;
  Serial.println();
  Serial.println("---------------- CFG DUMP (button) ----------------");
  Serial.printf("[CFG] device='%s' total_bytes=%lu chunks=%lu\n",
                BLE_DEVICE_NAME,
                (unsigned long)n,
                (unsigned long)((n + CHUNK - 1) / CHUNK));
  Serial.println("CFGBEGIN");
  for (size_t i = 0; i < n; i += CHUNK) {
    Serial.print("CFG ");
    for (size_t j = 0; j < CHUNK && (i + j) < n; ++j) {
      Serial.print(LAYOUT_CFG_BASE64[i + j]);
    }
    Serial.println();
  }
  Serial.println("CFGEND");
  Serial.println("---------------------------------------------------");
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
  delay(200);                                 // give USB-CDC time to enumerate
  Serial.println();
  Serial.println("=================================================");
  Serial.println("=== Micro:bit Remote — ESP32 firmware (boot) ===");
  Serial.println("=================================================");

  // ----- Boot diagnostics -----------------------------------------------
  esp_reset_reason_t rr = esp_reset_reason();
  Serial.printf("[BOOT] reset_reason : %d (%s)\n", (int)rr, resetReasonStr(rr));

  esp_chip_info_t chip;
  esp_chip_info(&chip);
  Serial.printf("[BOOT] chip_model   : %d  cores=%u  rev=%u  features=0x%08lx\n",
                (int)chip.model, chip.cores, chip.revision,
                (unsigned long)chip.features);
  Serial.printf("[BOOT] sdk_version  : %s\n", esp_get_idf_version());
  Serial.printf("[BOOT] arduino_core : %d.%d.%d\n",
                ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR,
                ESP_ARDUINO_VERSION_PATCH);
  Serial.printf("[BOOT] cpu_mhz      : %lu\n",
                (unsigned long)getCpuFrequencyMhz());
  Serial.printf("[BOOT] free_heap    : %lu bytes\n",
                (unsigned long)ESP.getFreeHeap());
  Serial.printf("[BOOT] min_heap     : %lu bytes\n",
                (unsigned long)ESP.getMinFreeHeap());
  Serial.printf("[BOOT] flash_size   : %lu bytes\n",
                (unsigned long)ESP.getFlashChipSize());
  Serial.printf("[BOOT] sketch_size  : %lu / %lu bytes\n",
                (unsigned long)ESP.getSketchSize(),
                (unsigned long)ESP.getFreeSketchSpace());
  Serial.flush();

  // ----- Step 1: GPIO ---------------------------------------------------
  Serial.println("[SETUP] step 1/4 — GPIO init");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.printf("[GPIO] LED_PIN=%d  BUTTON_PIN=%d (INPUT_PULLUP, press to dump CFG)\n",
                LED_PIN, BUTTON_PIN);

  // ----- Step 2: NimBLE init -------------------------------------------
  Serial.println("[SETUP] step 2/4 — NimBLEDevice::init");
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  Serial.printf("[BLE] device_name   : '%s'\n", BLE_DEVICE_NAME);
  Serial.printf("[BLE] local_mac     : %s\n",
                NimBLEDevice::getAddress().toString().c_str());
  Serial.printf("[BLE] heap_after_init: %lu bytes\n",
                (unsigned long)ESP.getFreeHeap());

  // ----- Step 3: GATT service + characteristics ------------------------
  Serial.println("[SETUP] step 3/4 — GATT service setup");
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
  Serial.printf("[BLE] service_uuid  : %s\n", UART_SERVICE_UUID);
  Serial.printf("[BLE] tx_char_uuid  : %s (notify)\n", UART_TX_CHAR_UUID);
  Serial.printf("[BLE] rx_char_uuid  : %s (write)\n",  UART_RX_CHAR_UUID);

  // ----- Step 4: Advertising -------------------------------------------
  // Primary packet:  flags + 128-bit service UUID.
  // Scan response:   the name (kept here to guarantee it survives).
  // rxy filters by namePrefix "BBC micro:bit", matched in either packet.
  Serial.println("[SETUP] step 4/4 — start advertising");
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(UART_SERVICE_UUID);
  adv->setName(BLE_DEVICE_NAME);
  adv->enableScanResponse(true);
  NimBLEDevice::startAdvertising();

  Serial.printf("[BLE] advertising as: '%s'\n", BLE_DEVICE_NAME);
  Serial.printf("[BOOT] setup() OK   — free_heap=%lu bytes\n",
                (unsigned long)ESP.getFreeHeap());
  Serial.println("[BLE] Open https://abourdim.github.io/rxy/ and click Connect.");
  Serial.println("=================================================");
  Serial.flush();
}

void loop() {
  const uint32_t now = millis();

  // -----------------------------------------------------------------
  // Periodic heartbeat — confirms the firmware is alive and shows
  // connection state + heap headroom. Cheap, prints once every 5 s.
  // -----------------------------------------------------------------
  static uint32_t lastBeat = 0;
  if (now - lastBeat >= 5000) {
    lastBeat = now;
    Serial.printf("[HB] uptime=%lus  connected=%d  heap=%lu  min_heap=%lu\n",
                  (unsigned long)(now / 1000),
                  gConnected ? 1 : 0,
                  (unsigned long)ESP.getFreeHeap(),
                  (unsigned long)ESP.getMinFreeHeap());
  }

  // -----------------------------------------------------------------
  // BUTTON_PIN poll — on a clean press (active-low, debounced 30 ms),
  // dump the LAYOUT_CFG_BASE64 to Serial. Edge-triggered: only fires
  // once per press, not while held.
  // -----------------------------------------------------------------
  static int      btnLast       = HIGH;
  static uint32_t btnLastChange = 0;
  const int btnNow = digitalRead(BUTTON_PIN);
  if (btnNow != btnLast && (now - btnLastChange) > 30) {
    btnLastChange = now;
    if (btnNow == BUTTON_ACTIVE) {
      Serial.println("[BTN] press detected — dumping CFG");
      printCfg();
    }
    btnLast = btnNow;
  }

  // -----------------------------------------------------------------
  // Periodic update example. Uncomment + adapt for your project.
  // The rxy web app rate-limits incoming messages to one per 200 ms,
  // so do not flood it. A 200–500 ms cadence is comfortable.
  // -----------------------------------------------------------------
  // static uint32_t lastUpd = 0;
  // if (gConnected && now - lastUpd > 500) {
  //   lastUpd = now;
  //   sendValue("gauge_temp", String((int)temperatureRead()));
  // }
}
