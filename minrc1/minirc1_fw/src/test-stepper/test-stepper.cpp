#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <driver/i2s.h>
#include <esp32-hal-matrix.h>

constexpr uint8_t kMicPins[3] = {43, 13, 4};
constexpr uint8_t kMicClkPin = 5;
constexpr uint8_t kBuzzerPinPos = 7;
constexpr uint8_t kBuzzerPinNeg = 6;
constexpr uint8_t kWs2812Pin = 48;

constexpr uint8_t kPwmBits = 8;
constexpr uint16_t kPwmMax = (1u << kPwmBits) - 1;
constexpr uint8_t kBuzzerChannel = 0;
constexpr uint32_t kBuzzerFreqHz = 4000;
constexpr uint32_t kBuzzerPeriodMs = 3000;
constexpr uint32_t kBuzzerOnMs = 180;
constexpr uint8_t kMotorBaseChannel = 2;
constexpr uint32_t kMotorPwmHz = 20000;
constexpr float kDefaultStepHz = 10.0f;
constexpr uint8_t kDefaultDutyPercent = 100;
constexpr uint32_t kSerialBaud = 460800;
constexpr uint32_t kMicSampleRate = 16000;
constexpr size_t kPacketSamples = 128;
constexpr size_t kFlushSamples = 128;
constexpr uint8_t kFlushReads = 2;
constexpr uint8_t kPacketsPerMic = 16;
constexpr uint32_t kPixelCount = 1;
constexpr char kSync[4] = {'M', 'D', 'A', 'Q'};
constexpr int8_t kStepTable[4][2] = {
  {1, 1},
  {-1, 1},
  {-1, -1},
  {1, -1},
};

struct __attribute__((packed)) PacketHeader {
  char magic[4];
  uint8_t mic;
  uint8_t reserved;
  uint16_t count;
  uint16_t rateHz;
};

struct Coil {
  uint8_t pinPos;
  uint8_t pinNeg;
  uint8_t channel;
};

struct Motor {
  Coil coilA;
  Coil coilB;
};

constexpr Motor kMotors[2] = {
  {{44, 3, 2}, {1, 2, 3}},
  {{9, 12, 4}, {10, 11, 5}},
};

Adafruit_NeoPixel pixels(kPixelCount, kWs2812Pin, NEO_GRB + NEO_KHZ800);

uint8_t motorPhase = 0;
uint8_t micIndex = 0;
uint8_t packetsOnMic = 0;
uint32_t lastStepUs = 0;
uint32_t lastBeepMs = 0;
bool buzzerOn = false;
bool buzzerEnabled = true;
float stepHz = kDefaultStepHz;
uint8_t dutyPercent = kDefaultDutyPercent;
char commandBuffer[32];
size_t commandLen = 0;

uint8_t dutyValue() {
  return static_cast<uint8_t>((static_cast<uint32_t>(dutyPercent) * kPwmMax) / 100u);
}

void driveCoil(const Coil& coil, int8_t state) {
  ledcDetachPin(coil.pinPos);
  ledcDetachPin(coil.pinNeg);
  digitalWrite(coil.pinPos, LOW);
  digitalWrite(coil.pinNeg, LOW);

  if (state == 0 || dutyPercent == 0) {
    return;
  }

  if (dutyPercent >= 100) {
    digitalWrite(state > 0 ? coil.pinPos : coil.pinNeg, HIGH);
    return;
  }

  const uint8_t activePin = state > 0 ? coil.pinPos : coil.pinNeg;
  ledcAttachPin(activePin, coil.channel);
  ledcWrite(coil.channel, dutyValue());
}

void applyMotors() {
  const bool enabled = stepHz > 0.0f && dutyPercent > 0;
  const int8_t coilA = enabled ? kStepTable[motorPhase][0] : 0;
  const int8_t coilB = enabled ? kStepTable[motorPhase][1] : 0;

  for (const Motor& motor : kMotors) {
    driveCoil(motor.coilA, coilA);
    driveCoil(motor.coilB, coilB);
  }
}

void setupMotors() {
  for (const Motor& motor : kMotors) {
    for (const Coil& coil : {motor.coilA, motor.coilB}) {
      pinMode(coil.pinPos, OUTPUT);
      pinMode(coil.pinNeg, OUTPUT);
      digitalWrite(coil.pinPos, LOW);
      digitalWrite(coil.pinNeg, LOW);
      ledcSetup(coil.channel, kMotorPwmHz, kPwmBits);
      ledcWrite(coil.channel, dutyValue());
    }
  }
  applyMotors();
}

void setupBuzzer() {
  ledcSetup(kBuzzerChannel, kBuzzerFreqHz, kPwmBits);
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

void updateBuzzer() {
  const uint32_t nowMs = millis();

  if (!buzzerEnabled) {
    stopBuzzer();
    return;
  }

  if (!buzzerOn && nowMs - lastBeepMs >= kBuzzerPeriodMs) {
    lastBeepMs = nowMs;
    buzzerOn = true;
    pinMatrixOutAttach(kBuzzerPinNeg, LEDC_LS_SIG_OUT0_IDX + kBuzzerChannel, true, false);
    ledcWrite(kBuzzerChannel, kPwmMax / 2);
  }

  if (buzzerOn && nowMs - lastBeepMs >= kBuzzerOnMs) {
    stopBuzzer();
  }
}

void stepMotors() {
  if (stepHz <= 0.0f || dutyPercent == 0) {
    return;
  }

  const uint32_t periodUs = static_cast<uint32_t>(1000000.0f / stepHz);
  if (periodUs == 0) {
    return;
  }

  const uint32_t nowUs = micros();
  if (nowUs - lastStepUs < periodUs) {
    return;
  }

  lastStepUs = nowUs;
  motorPhase = (motorPhase + 1) & 0x03;
  applyMotors();

  const uint16_t hue = static_cast<uint16_t>((motorPhase * 65536u) / 4u);
  pixels.setPixelColor(0, pixels.gamma32(pixels.ColorHSV(hue, 255, 255)));
  pixels.show();
}

bool readWords(int16_t* dst, size_t count) {
  size_t totalBytes = 0;
  const size_t targetBytes = count * sizeof(int16_t);
  while (totalBytes < targetBytes) {
    size_t bytesRead = 0;
    const esp_err_t err = i2s_read(
      I2S_NUM_0,
      reinterpret_cast<uint8_t*>(dst) + totalBytes,
      targetBytes - totalBytes,
      &bytesRead,
      pdMS_TO_TICKS(50));
    if (err != ESP_OK || bytesRead == 0) {
      return false;
    }
    totalBytes += bytesRead;
  }
  return true;
}

bool selectMicPin(uint8_t pin) {
  i2s_stop(I2S_NUM_0);

  i2s_pin_config_t pinConfig = {};
  pinConfig.mck_io_num = I2S_PIN_NO_CHANGE;
  pinConfig.bck_io_num = I2S_PIN_NO_CHANGE;
  pinConfig.ws_io_num = kMicClkPin;
  pinConfig.data_out_num = I2S_PIN_NO_CHANGE;
  pinConfig.data_in_num = pin;

  if (i2s_set_pin(I2S_NUM_0, &pinConfig) != ESP_OK) {
    return false;
  }

  i2s_start(I2S_NUM_0);
  delay(4);

  int16_t flush[kFlushSamples];
  for (uint8_t i = 0; i < kFlushReads; ++i) {
    if (!readWords(flush, kFlushSamples)) {
      return false;
    }
  }

  return true;
}

bool setupMics() {
  i2s_config_t config = {};
  config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
  config.sample_rate = kMicSampleRate;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = 4;
  config.dma_buf_len = 256;
  config.use_apll = false;
  config.tx_desc_auto_clear = false;
  config.fixed_mclk = 0;

  if (i2s_driver_install(I2S_NUM_0, &config, 0, nullptr) != ESP_OK) {
    return false;
  }
  if (i2s_set_clk(I2S_NUM_0, kMicSampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO) != ESP_OK) {
    return false;
  }
  if (i2s_set_pdm_rx_down_sample(I2S_NUM_0, I2S_PDM_DSR_16S) != ESP_OK) {
    return false;
  }
  return selectMicPin(kMicPins[micIndex]);
}

bool switchMic(uint8_t nextMicIndex) {
  micIndex = nextMicIndex;
  packetsOnMic = 0;
  return selectMicPin(kMicPins[micIndex]);
}

void sendPacket(const int16_t* samples, uint8_t mic) {
  const PacketHeader header = {
    {kSync[0], kSync[1], kSync[2], kSync[3]},
    mic,
    0,
    static_cast<uint16_t>(kPacketSamples),
    static_cast<uint16_t>(kMicSampleRate),
  };
  Serial.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  Serial.write(reinterpret_cast<const uint8_t*>(samples), kPacketSamples * sizeof(int16_t));
}

void applyCommand(const char* line) {
  if (line[0] == '\0') {
    return;
  }

  char* end = nullptr;
  const float value = strtof(line + 1, &end);
  if (end == line + 1 && line[0] != 'b' && line[0] != 'B') {
    return;
  }

  switch (line[0]) {
    case 's':
    case 'S':
      if (value < 0.0f || value > 200.0f) {
        return;
      }
      stepHz = value;
      lastStepUs = micros();
      applyMotors();
      return;
    case 'd':
    case 'D':
      if (value < 0.0f || value > 100.0f) {
        return;
      }
      dutyPercent = static_cast<uint8_t>(value + 0.5f);
      applyMotors();
      return;
    case 'b':
    case 'B':
      buzzerEnabled = line[1] != '0';
      if (!buzzerEnabled) {
        stopBuzzer();
      }
      return;
    default:
      return;
  }
}

void readCommands() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      commandBuffer[commandLen] = '\0';
      applyCommand(commandBuffer);
      commandLen = 0;
      continue;
    }
    if (commandLen + 1 < sizeof(commandBuffer)) {
      commandBuffer[commandLen++] = ch;
    }
  }
}

void setup() {
  Serial.begin(kSerialBaud);
  const uint32_t waitStart = millis();
  while (!Serial && (millis() - waitStart) < 3000) {
    delay(10);
  }

  pixels.begin();
  pixels.setBrightness(32);
  pixels.setPixelColor(0, pixels.Color(0, 8, 16));
  pixels.show();

  setupMotors();
  setupBuzzer();

  if (!setupMics()) {
    pixels.setPixelColor(0, pixels.Color(32, 0, 0));
    pixels.show();
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  static int16_t samples[kPacketSamples];

  readCommands();
  stepMotors();
  updateBuzzer();

  if (!readWords(samples, kPacketSamples)) {
    return;
  }

  sendPacket(samples, micIndex);

  ++packetsOnMic;
  if (packetsOnMic >= kPacketsPerMic) {
    switchMic((micIndex + 1) % 3);
  }
}
