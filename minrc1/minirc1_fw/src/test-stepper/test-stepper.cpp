#include <Arduino.h>

// constexpr uint8_t pins[] = {13, 12, 11, 10};
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

uint32_t stepCount = 0;
uint8_t phase = 0;
float pwmDutyPercent = 50.0f;
float pwmFreqKhz = 20.0f;
float stepPeriodMs = 5.0f;
uint32_t lastStepUs = 0;
String command;

uint32_t pwmFreqHz() {
  return static_cast<uint32_t>(pwmFreqKhz * 1000.0f + 0.5f);
}

uint32_t stepPeriodUs() {
  return static_cast<uint32_t>(stepPeriodMs * 1000.0f + 0.5f);
}

uint32_t pwmDutyValue() {
  return static_cast<uint32_t>(pwmDutyPercent * pwmMax / 100.0f + 0.5f);
}

void setPhase(uint8_t nextPhase) {
  const uint32_t duty = pwmDutyValue();
  for (uint8_t i = 0; i < 4; ++i) {
    ledcWrite(i, phases[nextPhase][i] ? duty : 0);
  }
}

void setupPwm() {
  const uint32_t freqHz = pwmFreqHz();

  for (uint8_t i = 0; i < 4; ++i) {
    ledcSetup(i, freqHz, pwmBits);
    ledcAttachPin(pins[i], i);
  }
}

void printState() {
  Serial.print("duty_pct=");
  Serial.print(pwmDutyPercent, 2);
  Serial.print(" freq_khz=");
  Serial.print(pwmFreqKhz, 3);
  Serial.print(" period_ms=");
  Serial.println(stepPeriodMs, 3);
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
      Serial.println("bad duty");
    } else {
      pwmDutyPercent = value;
      setPhase(phase);
      Serial.print("duty_pct=");
      Serial.println(pwmDutyPercent, 2);
    }
  } else if (type == 'f') {
    const float value = valueText.toFloat();
    if (value <= 0.0f) {
      Serial.println("bad freq");
    } else {
      pwmFreqKhz = value;
      setupPwm();
      setPhase(phase);
      Serial.print("freq_khz=");
      Serial.println(pwmFreqKhz, 3);
    }
  } else if (type == 'p') {
    const float value = valueText.toFloat();
    if (value <= 0.0f) {
      Serial.println("bad period");
    } else {
      stepPeriodMs = value;
      Serial.print("period_ms=");
      Serial.println(stepPeriodMs, 3);
    }
  } else {
    Serial.println("bad cmd");
  }

  command = "";
}

void readCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\n' || c == '\r') {
      applyCommand();
      continue;
    }

    command += c;
  }
}

void setup() {
  Serial.begin(115200);
  setupPwm();
  setPhase(phase);
  printState();
}

void loop() {
  readCommands();

  const uint32_t nowUs = micros();
  if (nowUs - lastStepUs < stepPeriodUs()) {
    return;
  }

  lastStepUs = nowUs;
  setPhase(phase);
  phase = (phase + 1) % phaseCount;

//   Serial.print("steps:");
//   Serial.println(stepCount);
  ++stepCount;
}
