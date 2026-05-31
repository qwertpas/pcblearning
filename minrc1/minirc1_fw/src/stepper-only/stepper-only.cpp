#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp32-hal-matrix.h>

#include <cmath>
#include <string>

struct Coil {
  uint8_t pinPos;
  uint8_t pinNeg;
  uint8_t channel;
};

struct Motor {
  Coil coilA;
  Coil coilB;
};

struct MotorState {
  uint8_t stepIndex;
  uint32_t lastStepUs;
  float stepHz;
};

constexpr Motor kMotors[2] = {
  {{44, 1, 0}, {2, 3, 1}},
  {{9, 10, 2}, {11, 12, 3}},
};

constexpr uint8_t kPwmBits = 8;
constexpr uint16_t kPwmMax = (1u << kPwmBits) - 1;
constexpr int8_t kPhases[][2] = {
  {1, 1},
  {-1, 1},
  {-1, -1},
  {1, -1},
};
constexpr uint8_t kPhaseCount = sizeof(kPhases) / sizeof(kPhases[0]);

constexpr uint16_t kStepsPerHueCycle = 20;
constexpr uint8_t kWs2812Pin = 48;
constexpr uint16_t kPixelCount = 1;
constexpr uint8_t kVbusSensePin = 8;
constexpr uint8_t kVbusSamples = 8;
constexpr float kVbusDividerScale = 2.0f;
constexpr uint8_t kBuzzerPinPos = 7;
constexpr uint8_t kBuzzerPinNeg = 6;
constexpr uint8_t kBuzzerChannel = 4;
constexpr uint32_t kDefaultBuzzerFreqHz = 4000;
constexpr uint32_t kBuzzerOnMs = 120;

constexpr char kBleDeviceName[] = "MinRC1-Stepper";
constexpr char kUartServiceUuid[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kUartRxUuid[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kUartTxUuid[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

constexpr char kPrefsNamespace[] = "stepper";
constexpr char kPrefsDuty[] = "duty";
constexpr char kPrefsFreqKhz[] = "fkhz";
constexpr char kPrefsPeriodMs[] = "pms";
constexpr char kPrefsRun[] = "run";
constexpr char kPrefsOrder[] = "order";

Adafruit_NeoPixel pixels(kPixelCount, kWs2812Pin, NEO_GRB + NEO_KHZ800);

uint32_t stepCount = 0;
float pwmDutyPercent = 80.0f;
float pwmFreqKhz = 20.0f;
float stepPeriodMs = 10.0f;
String command;
char phaseOrder[5] = "0123";
MotorState motorStates[2] = {
  {0, 0, 0.0f},
  {0, 0, 0.0f},
};

BLEServer* bleServer = nullptr;
BLECharacteristic* bleTxCharacteristic = nullptr;
bool bleConnected = false;
bool bleOldConnected = false;
bool buzzerOn = false;
uint32_t buzzerOffMs = 0;
uint32_t buzzerFreqHz = kDefaultBuzzerFreqHz;

portMUX_TYPE bleCmdMux = portMUX_INITIALIZER_UNLOCKED;
String bleIncoming;

uint32_t pwmFreqHz() {
  return static_cast<uint32_t>(pwmFreqKhz * 1000.0f + 0.5f);
}

uint32_t stepPeriodUs() {
  return static_cast<uint32_t>(stepPeriodMs * 1000.0f + 0.5f);
}

uint32_t pwmDutyValue() {
  return static_cast<uint32_t>(pwmDutyPercent * kPwmMax / 100.0f + 0.5f);
}

float defaultStepHz() {
  if (stepPeriodMs <= 0.0f) {
    return 0.0f;
  }
  return 1000.0f / stepPeriodMs;
}

bool anyMotorRunning() {
  return motorStates[0].stepHz != 0.0f || motorStates[1].stepHz != 0.0f;
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

void driveCoil(const Coil& coil, int8_t state) {
  ledcDetachPin(coil.pinPos);
  ledcDetachPin(coil.pinNeg);
  digitalWrite(coil.pinPos, LOW);
  digitalWrite(coil.pinNeg, LOW);

  if (state == 0 || pwmDutyPercent <= 0.0f) {
    return;
  }

  if (pwmDutyPercent >= 100.0f) {
    digitalWrite(state > 0 ? coil.pinPos : coil.pinNeg, HIGH);
    return;
  }

  const uint8_t activePin = state > 0 ? coil.pinPos : coil.pinNeg;
  ledcAttachPin(activePin, coil.channel);
  ledcWrite(coil.channel, pwmDutyValue());
}

void setMotorPhase(uint8_t motorIndex, uint8_t nextPhase) {
  const Motor& motor = kMotors[motorIndex];
  driveCoil(motor.coilA, kPhases[nextPhase][0]);
  driveCoil(motor.coilB, kPhases[nextPhase][1]);
}

uint8_t phaseIndexForStep(uint8_t step) {
  return static_cast<uint8_t>(phaseOrder[step % kPhaseCount] - '0');
}

bool parsePhaseOrder(const String& text, char out[5]) {
  if (text.length() != kPhaseCount) {
    return false;
  }

  bool seen[kPhaseCount] = {false, false, false, false};
  for (uint8_t i = 0; i < kPhaseCount; ++i) {
    const char c = text[i];
    if (c < '0' || c >= ('0' + kPhaseCount)) {
      return false;
    }
    const uint8_t value = static_cast<uint8_t>(c - '0');
    if (seen[value]) {
      return false;
    }
    seen[value] = true;
    out[i] = c;
  }
  out[kPhaseCount] = '\0';
  return true;
}

void stopMotor(uint8_t motorIndex) {
  const Motor& motor = kMotors[motorIndex];
  driveCoil(motor.coilA, 0);
  driveCoil(motor.coilB, 0);
}

void stopMotors() {
  for (uint8_t i = 0; i < 2; ++i) {
    motorStates[i].stepHz = 0.0f;
    stopMotor(i);
  }
}

void setupMotorPinsLow() {
  for (const Motor& motor : kMotors) {
    for (const Coil& coil : {motor.coilA, motor.coilB}) {
      pinMode(coil.pinPos, OUTPUT);
      pinMode(coil.pinNeg, OUTPUT);
      digitalWrite(coil.pinPos, LOW);
      digitalWrite(coil.pinNeg, LOW);
    }
  }
}

void updateRgbForStep() {
  const uint32_t s = (stepCount - 1u) % kStepsPerHueCycle;
  const uint16_t hueNow = static_cast<uint16_t>((s * 65536u) / kStepsPerHueCycle);
  pixels.setPixelColor(0, pixels.gamma32(pixels.ColorHSV(hueNow, 255, 255)));
  pixels.show();
}

void setupBuzzer() {
  ledcSetup(kBuzzerChannel, buzzerFreqHz, kPwmBits);
  ledcAttachPin(kBuzzerPinPos, kBuzzerChannel);
  pinMode(kBuzzerPinNeg, OUTPUT);
  ledcWrite(kBuzzerChannel, 0);
  digitalWrite(kBuzzerPinNeg, LOW);
}

void stopBuzzer() {
  buzzerOn = false;
  ledcWrite(kBuzzerChannel, 0);
  pinMatrixOutDetach(kBuzzerPinNeg, false, false);
  digitalWrite(kBuzzerPinNeg, LOW);
}

void beepBuzzer(uint32_t durationMs = kBuzzerOnMs) {
  ledcSetup(kBuzzerChannel, buzzerFreqHz, kPwmBits);
  buzzerOn = true;
  buzzerOffMs = millis() + durationMs;
  pinMatrixOutAttach(kBuzzerPinNeg, LEDC_LS_SIG_OUT0_IDX + kBuzzerChannel, true, false);
  ledcWrite(kBuzzerChannel, kPwmMax / 2);
}

void setBuzzerFreq(uint32_t nextFreqHz) {
  buzzerFreqHz = nextFreqHz;
  ledcSetup(kBuzzerChannel, buzzerFreqHz, kPwmBits);
  if (buzzerOn) {
    ledcWrite(kBuzzerChannel, kPwmMax / 2);
  }
}

void updateBuzzer() {
  if (!buzzerOn) {
    return;
  }
  if (static_cast<int32_t>(millis() - buzzerOffMs) >= 0) {
    stopBuzzer();
  }
}

float readVbusVolts() {
  uint32_t sumMv = 0;
  for (uint8_t i = 0; i < kVbusSamples; ++i) {
    sumMv += static_cast<uint32_t>(analogReadMilliVolts(kVbusSensePin));
  }
  const float pinVolts = static_cast<float>(sumMv) / static_cast<float>(kVbusSamples) / 1000.0f;
  return pinVolts * kVbusDividerScale;
}

void refreshMotorOutputs() {
  for (uint8_t i = 0; i < 2; ++i) {
    if (motorStates[i].stepHz == 0.0f) {
      stopMotor(i);
      continue;
    }
    setMotorPhase(i, phaseIndexForStep(motorStates[i].stepIndex));
  }
}

void printVbus() {
  replyln("vbus_v=" + String(readVbusVolts(), 3));
}

void loadPrefs() {
  Preferences prefs;
  if (!prefs.begin(kPrefsNamespace, true)) {
    return;
  }

  const float duty = prefs.getFloat(kPrefsDuty, pwmDutyPercent);
  const float freq = prefs.getFloat(kPrefsFreqKhz, pwmFreqKhz);
  const float period = prefs.getFloat(kPrefsPeriodMs, stepPeriodMs);
  const String order = prefs.isKey(kPrefsOrder) ? prefs.getString(kPrefsOrder, phaseOrder) : String(phaseOrder);
  prefs.end();

  if (duty >= 0.0f && duty <= 100.0f) {
    pwmDutyPercent = duty;
  }
  if (freq > 0.0f) {
    pwmFreqKhz = freq;
  }
  if (period > 0.0f) {
    stepPeriodMs = period;
  }
  char loadedOrder[5];
  if (parsePhaseOrder(order, loadedOrder)) {
    memcpy(phaseOrder, loadedOrder, sizeof(phaseOrder));
  }
}

void savePrefs() {
  Preferences prefs;
  if (!prefs.begin(kPrefsNamespace, false)) {
    return;
  }
  prefs.putFloat(kPrefsDuty, pwmDutyPercent);
  prefs.putFloat(kPrefsFreqKhz, pwmFreqKhz);
  prefs.putFloat(kPrefsPeriodMs, stepPeriodMs);
  prefs.putBool(kPrefsRun, anyMotorRunning());
  prefs.putString(kPrefsOrder, phaseOrder);
  prefs.end();
}

void setupPwm() {
  for (const Motor& motor : kMotors) {
    for (const Coil& coil : {motor.coilA, motor.coilB}) {
      pinMode(coil.pinPos, OUTPUT);
      pinMode(coil.pinNeg, OUTPUT);
      digitalWrite(coil.pinPos, LOW);
      digitalWrite(coil.pinNeg, LOW);
      ledcSetup(coil.channel, pwmFreqHz(), kPwmBits);
      ledcWrite(coil.channel, 0);
    }
  }
}

void printState() {
  const String line = "duty_pct=" + String(pwmDutyPercent, 2) +
                      " freq_khz=" + String(pwmFreqKhz, 3) +
                      " period_ms=" + String(stepPeriodMs, 3) +
                      " run=" + String(anyMotorRunning() ? 1 : 0) +
                      " left_hz=" + String(motorStates[0].stepHz, 3) +
                      " right_hz=" + String(motorStates[1].stepHz, 3) +
                      " beep_hz=" + String(buzzerFreqHz) +
                      " vbus_v=" + String(readVbusVolts(), 3) +
                      " order=" + String(phaseOrder);
  replyln(line);
}

bool parseDriveValue(const String& text, float& out) {
  if (text.length() == 0) {
    return false;
  }
  char* end = nullptr;
  const float value = strtof(text.c_str(), &end);
  if (end == nullptr || *end != '\0' || !std::isfinite(value)) {
    return false;
  }
  out = value;
  return true;
}

bool parseDrivePair(const String& text, float& leftHz, float& rightHz) {
  const int comma = text.indexOf(',');
  if (comma <= 0 || comma >= static_cast<int>(text.length()) - 1) {
    return false;
  }
  return parseDriveValue(text.substring(0, comma), leftHz) &&
         parseDriveValue(text.substring(comma + 1), rightHz);
}

void setDrive(float leftHz, float rightHz) {
  const float nextHz[2] = {leftHz, rightHz};
  const uint32_t nowUs = micros();

  for (uint8_t i = 0; i < 2; ++i) {
    const bool wasStopped = motorStates[i].stepHz == 0.0f;
    motorStates[i].stepHz = nextHz[i];
    if (motorStates[i].stepHz == 0.0f) {
      stopMotor(i);
      continue;
    }
    if (wasStopped) {
      motorStates[i].lastStepUs = nowUs;
      setMotorPhase(i, phaseIndexForStep(motorStates[i].stepIndex));
    }
  }
}

void applyCommand() {
  if (command.length() == 0) {
    command = "";
    return;
  }

  const char type = command[0];
  const String valueText = command.substring(1);

  if (type == 'b' && command.length() == 1) {
    printVbus();
    command = "";
    return;
  }

  if (type == 'q' && command.length() == 1) {
    printState();
    command = "";
    return;
  }

  if (type == 'x' && command.length() == 1) {
    beepBuzzer();
    command = "";
    return;
  }

  if (command.length() < 2) {
    command = "";
    return;
  }

  const float value = valueText.toFloat();

  if (type == 'd') {
    if (value < 0.0f || value > 100.0f) {
      replyln("bad duty");
    } else {
      pwmDutyPercent = value;
      refreshMotorOutputs();
      savePrefs();
      printState();
    }
  } else if (type == 'f') {
    if (value <= 0.0f) {
      replyln("bad freq");
    } else {
      pwmFreqKhz = value;
      setupPwm();
      refreshMotorOutputs();
      savePrefs();
      printState();
    }
  } else if (type == 'p') {
    if (value <= 0.0f) {
      replyln("bad period");
    } else {
      stepPeriodMs = value;
      savePrefs();
      printState();
    }
  } else if (type == 'r') {
    if (value < 0.5f) {
      stepCount = 0;
      setDrive(0.0f, 0.0f);
    } else {
      const float stepHz = defaultStepHz();
      setDrive(stepHz, stepHz);
    }
    savePrefs();
    printState();
  } else if (type == 'v') {
    float leftHz = 0.0f;
    float rightHz = 0.0f;
    if (!parseDrivePair(valueText, leftHz, rightHz)) {
      replyln("bad drive");
    } else {
      setDrive(leftHz, rightHz);
      printState();
    }
  } else if (type == 'o') {
    char nextOrder[5];
    if (!parsePhaseOrder(valueText, nextOrder)) {
      replyln("bad order");
    } else {
      memcpy(phaseOrder, nextOrder, sizeof(phaseOrder));
      refreshMotorOutputs();
      savePrefs();
      printState();
    }
  } else if (type == 'z') {
    const uint32_t nextFreqHz = static_cast<uint32_t>(value + 0.5f);
    if (nextFreqHz < 100 || nextFreqHz > 20000) {
      replyln("bad beep");
    } else {
      setBuzzerFreq(nextFreqHz);
      printState();
    }
  } else {
    replyln("bad cmd");
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
    } else {
      command += c;
    }
  }
}

class BleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* /* server */) override { bleConnected = true; }

  void onDisconnect(BLEServer* server) override {
    bleConnected = false;
    server->getAdvertising()->start();
  }
};

class BleRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    const std::string value = characteristic->getValue();
    if (value.empty()) {
      return;
    }
    portENTER_CRITICAL(&bleCmdMux);
    for (char c : value) {
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
}

void setup() {
  setupMotorPinsLow();
  analogSetPinAttenuation(kVbusSensePin, ADC_11db);
  Serial.begin(115200);
  delay(100);

  pixels.begin();
  pixels.setBrightness(64);
  pixels.setPixelColor(0, pixels.gamma32(pixels.ColorHSV(0, 255, 255)));
  pixels.show();

  loadPrefs();
  setupPwm();
  setupBuzzer();
  stopBuzzer();
  stopMotors();
  printState();
  setupBle();
  replyln("BLE ready; commands: v<left_hz,right_hz> d<duty_pct> b(vbus) q(state) x(beep) z<beep_hz> o0123/o0321");
}

void loop() {
  readSerialCommands();
  drainBleIncoming();
  updateBuzzer();

  if (!bleConnected && bleOldConnected) {
    delay(500);
    bleServer->startAdvertising();
    bleOldConnected = bleConnected;
  }
  if (bleConnected && !bleOldConnected) {
    bleOldConnected = bleConnected;
  }

  const uint32_t nowUs = micros();
  for (uint8_t i = 0; i < 2; ++i) {
    const float stepHz = std::fabs(motorStates[i].stepHz);
    if (stepHz <= 0.0f) {
      continue;
    }

    const uint32_t periodUs = static_cast<uint32_t>(1000000.0f / stepHz);
    if (periodUs == 0 || nowUs - motorStates[i].lastStepUs < periodUs) {
      continue;
    }

    motorStates[i].lastStepUs = nowUs;
    setMotorPhase(i, phaseIndexForStep(motorStates[i].stepIndex));
    if (motorStates[i].stepHz > 0.0f) {
      motorStates[i].stepIndex = (motorStates[i].stepIndex + 1u) % kPhaseCount;
    } else {
      motorStates[i].stepIndex = (motorStates[i].stepIndex + kPhaseCount - 1u) % kPhaseCount;
    }
    ++stepCount;
    updateRgbForStep();
  }
}
