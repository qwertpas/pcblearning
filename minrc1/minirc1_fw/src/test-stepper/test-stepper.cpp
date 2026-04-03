#include <Arduino.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <string>

// Stepper outputs (LEDC channels 0–3)
constexpr uint8_t pins[] = {43, 44, 1, 2};
constexpr uint8_t pwmBits = 8;
constexpr uint16_t pwmMax = (1u << pwmBits) - 1;
constexpr uint8_t phases[][4] = {
  {1, 0, 1, 0},
  {0, 1, 1, 0},
  {0, 1, 0, 1},
  {1, 0, 0, 1},
};
constexpr uint8_t phaseCount = sizeof(phases) / sizeof(phases[0]);

// One full hue cycle every this many motor steps
constexpr uint16_t kStepsPerHueCycle = 20;

// WS2812 (from blink.cpp)
constexpr uint8_t kWs2812Pin = 48;
constexpr uint16_t kPixelCount = 1;

// Nordic UART service (BLE; works with laptop via bleak)
constexpr char kBleDeviceName[] = "MinRC1-Stepper";
constexpr char kUartServiceUuid[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kUartRxUuid[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kUartTxUuid[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

// NVS (survives reboot / reflash of app partition)
constexpr char kPrefsNamespace[] = "stepper";
constexpr char kPrefsDuty[] = "duty";
constexpr char kPrefsFreqKhz[] = "fkhz";
constexpr char kPrefsPeriodMs[] = "pms";

Adafruit_NeoPixel pixels(kPixelCount, kWs2812Pin, NEO_GRB + NEO_KHZ800);

uint32_t stepCount = 0;
uint8_t phase = 0;
float pwmDutyPercent = 80.0f;
float pwmFreqKhz = 20.0f;
float stepPeriodMs = 10.0f;
uint32_t lastStepUs = 0;
String command;

BLEServer* bleServer = nullptr;
BLECharacteristic* bleTxCharacteristic = nullptr;
bool bleConnected = false;
bool bleOldConnected = false;

portMUX_TYPE bleCmdMux = portMUX_INITIALIZER_UNLOCKED;
String bleIncoming;

uint32_t pwmFreqHz() {
  return static_cast<uint32_t>(pwmFreqKhz * 1000.0f + 0.5f);
}

uint32_t stepPeriodUs() {
  return static_cast<uint32_t>(stepPeriodMs * 1000.0f + 0.5f);
}

uint32_t pwmDutyValue() {
  return static_cast<uint32_t>(pwmDutyPercent * pwmMax / 100.0f + 0.5f);
}

void bleNotify(const String& s) {
  if (bleTxCharacteristic != nullptr && bleConnected) {
    bleTxCharacteristic->setValue(s.c_str());
    bleTxCharacteristic->notify();
  }
}

void replyln(const String& line) {
  Serial.println(line);
  bleNotify(line + "\n");
}

void replyBad(const char* msg) {
  replyln(msg);
}

void setPhase(uint8_t nextPhase) {
  const uint32_t duty = pwmDutyValue();
  for (uint8_t i = 0; i < 4; ++i) {
    ledcWrite(i, phases[nextPhase][i] ? duty : 0);
  }
}

void updateRgbForStep() {
  const uint32_t s = (stepCount - 1u) % kStepsPerHueCycle;
  const uint16_t hueNow = static_cast<uint16_t>((s * 65536u) / kStepsPerHueCycle);
  const uint32_t color = pixels.gamma32(pixels.ColorHSV(hueNow, 255, 255));
  pixels.setPixelColor(0, color);
  pixels.show();
}

void loadStepperPrefs() {
  Preferences prefs;
  if (!prefs.begin(kPrefsNamespace, true)) {
    return;
  }
  const float d = prefs.getFloat(kPrefsDuty, pwmDutyPercent);
  const float f = prefs.getFloat(kPrefsFreqKhz, pwmFreqKhz);
  const float p = prefs.getFloat(kPrefsPeriodMs, stepPeriodMs);
  prefs.end();

  if (d >= 0.0f && d <= 100.0f) {
    pwmDutyPercent = d;
  }
  if (f > 0.0f) {
    pwmFreqKhz = f;
  }
  if (p > 0.0f) {
    stepPeriodMs = p;
  }
}

void saveStepperPrefs() {
  Preferences prefs;
  if (!prefs.begin(kPrefsNamespace, false)) {
    return;
  }
  prefs.putFloat(kPrefsDuty, pwmDutyPercent);
  prefs.putFloat(kPrefsFreqKhz, pwmFreqKhz);
  prefs.putFloat(kPrefsPeriodMs, stepPeriodMs);
  prefs.end();
}

void setupPwm() {
  const uint32_t freqHz = pwmFreqHz();

  for (uint8_t i = 0; i < 4; ++i) {
    ledcSetup(i, freqHz, pwmBits);
    ledcAttachPin(pins[i], i);
  }
}

void printState() {
  String line = "duty_pct=" + String(pwmDutyPercent, 2) + " freq_khz=" + String(pwmFreqKhz, 3) +
                " period_ms=" + String(stepPeriodMs, 3);
  Serial.println(line);
  bleNotify(line + "\n");
}

void applyCommand() {
  if (command.length() < 2) {
    command = "";
    return;
  }

  const char type = command[0];
  const String valueText = command.substring(1);

  if (type == 'd') {
    const float value = valueText.toFloat();
    if (value < 0.0f || value > 100.0f) {
      replyBad("bad duty");
    } else {
      pwmDutyPercent = value;
      setPhase(phase);
      saveStepperPrefs();
      replyln("duty_pct=" + String(pwmDutyPercent, 2));
    }
  } else if (type == 'f') {
    const float value = valueText.toFloat();
    if (value <= 0.0f) {
      replyBad("bad freq");
    } else {
      pwmFreqKhz = value;
      setupPwm();
      setPhase(phase);
      saveStepperPrefs();
      replyln("freq_khz=" + String(pwmFreqKhz, 3));
    }
  } else if (type == 'p') {
    const float value = valueText.toFloat();
    if (value <= 0.0f) {
      replyBad("bad period");
    } else {
      stepPeriodMs = value;
      saveStepperPrefs();
      replyln("period_ms=" + String(stepPeriodMs, 3));
    }
  } else {
    replyBad("bad cmd");
  }

  command = "";
}

void drainBleIncoming() {
  String chunk;
  portENTER_CRITICAL(&bleCmdMux);
  if (bleIncoming.length() > 0) {
    chunk = bleIncoming;
    bleIncoming = "";
  }
  portEXIT_CRITICAL(&bleCmdMux);

  for (unsigned i = 0; i < chunk.length(); ++i) {
    const char c = chunk[i];
    if (c == '\n' || c == '\r') {
      applyCommand();
    } else {
      command += c;
    }
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\n' || c == '\r') {
      applyCommand();
      continue;
    }

    command += c;
  }
}

class BleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* /* p */) override { bleConnected = true; }

  void onDisconnect(BLEServer* p) override {
    bleConnected = false;
    p->getAdvertising()->start();
  }
};

class BleRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    const std::string rxValue = characteristic->getValue();
    if (rxValue.empty()) {
      return;
    }
    portENTER_CRITICAL(&bleCmdMux);
    for (char c : rxValue) {
      bleIncoming += c;
    }
    portEXIT_CRITICAL(&bleCmdMux);
  }
};

void setupBle() {
  BLEDevice::init(kBleDeviceName);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new BleServerCallbacks());

  BLEService* service = bleServer->createService(kUartServiceUuid);

  BLECharacteristic* rxChar = service->createCharacteristic(
    kUartRxUuid,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new BleRxCallbacks());

  bleTxCharacteristic = service->createCharacteristic(
    kUartTxUuid,
    BLECharacteristic::PROPERTY_NOTIFY);
  bleTxCharacteristic->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(kUartServiceUuid);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE Nordic UART advertising as \"" + String(kBleDeviceName) + "\"");
}

void setup() {
  Serial.begin(115200);
  
  delay(100);

  pixels.begin();
  pixels.setBrightness(64);
  pixels.setPixelColor(0, pixels.gamma32(pixels.ColorHSV(0, 255, 255)));
  pixels.show();

  loadStepperPrefs();
  setupPwm();
  setPhase(phase);
  printState();

  setupBle();
}

void loop() {
  readSerialCommands();
  drainBleIncoming();

  if (!bleConnected && bleOldConnected) {
    delay(500);
    bleServer->startAdvertising();
    Serial.println("BLE start advertising");
    bleOldConnected = bleConnected;
  }
  if (bleConnected && !bleOldConnected) {
    bleOldConnected = bleConnected;
  }

  const uint32_t nowUs = micros();
  if (nowUs - lastStepUs < stepPeriodUs()) {
    return;
  }

  lastStepUs = nowUs;
  setPhase(phase);
  phase = (phase + 1) % phaseCount;

  ++stepCount;
  updateRgbForStep();
}
