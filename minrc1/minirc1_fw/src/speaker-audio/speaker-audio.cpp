#include <Arduino.h>
#include <LittleFS.h>
#include "esp_heap_caps.h"

constexpr int AMP_ENABLE_PIN = 5;
constexpr int AMP_INPUT_NODE_PIN = 6;
constexpr int AUDIO_PIN = 7;

constexpr int SAMPLE_RATE = 22050;
constexpr int PWM_FREQ = 78125;
constexpr int PWM_BITS = 8;
constexpr int PWM_CHANNEL = 0;
constexpr int TIMER_DIVIDER = 8;
constexpr int TIMER_TICKS = 80000000 / TIMER_DIVIDER / SAMPLE_RATE;
constexpr int ACTUAL_SAMPLE_RATE = 80000000 / TIMER_DIVIDER / TIMER_TICKS;

hw_timer_t *audioTimer = nullptr;
portMUX_TYPE audioMux = portMUX_INITIALIZER_UNLOCKED;

uint8_t *audioData = nullptr;
volatile uint32_t activeLength = 0;
volatile uint32_t activeIndex = 0;
volatile bool playbackDone = false;
volatile uint32_t sampleCount = 0;
volatile uint32_t underruns = 0;
volatile uint8_t lastDuty = 128;
volatile uint32_t loopCount = 0;

uint32_t fileBytes = 0;
uint32_t fileHash = 2166136261u;
int16_t minSample = 32767;
int16_t maxSample = -32768;
int64_t sumAbs = 0;
uint32_t statSamples = 0;

void IRAM_ATTR onAudioTimer() {
  uint8_t duty = 128;

  portENTER_CRITICAL_ISR(&audioMux);
  if (!playbackDone && activeIndex < activeLength) {
    duty = audioData[activeIndex++];
    sampleCount++;
    if (activeIndex >= activeLength) {
      playbackDone = true;
    }
  } else if (!playbackDone) {
    underruns++;
  }
  lastDuty = duty;
  portEXIT_CRITICAL_ISR(&audioMux);

  ledcWrite(PWM_CHANNEL, duty);
}

void printPins() {
  Serial.printf("pins: amp_en_gpio%d=%d audio_gpio%d_pwm=%d input_node_gpio%d_mode=input\n",
                AMP_ENABLE_PIN, digitalRead(AMP_ENABLE_PIN), AUDIO_PIN, lastDuty,
                AMP_INPUT_NODE_PIN);
}

bool inspectAudioFile() {
  File file = LittleFS.open("/audio.s16", "r");
  if (!file) {
    Serial.println("audio: missing /audio.s16");
    return false;
  }

  fileBytes = file.size();
  if (fileBytes < 2 || (fileBytes % 2) != 0) {
    Serial.printf("audio: invalid byte count=%lu\n", fileBytes);
    file.close();
    return false;
  }

  audioData = static_cast<uint8_t *>(heap_caps_malloc(fileBytes / 2, MALLOC_CAP_8BIT));
  if (!audioData) {
    Serial.printf("audio: malloc failed bytes=%lu\n", fileBytes / 2);
    file.close();
    return false;
  }

  uint8_t bytes[256];
  uint32_t outIndex = 0;
  while (true) {
    size_t got = file.read(bytes, sizeof(bytes));
    if (!got) {
      break;
    }
    for (size_t i = 0; i < got; i++) {
      fileHash ^= bytes[i];
      fileHash *= 16777619u;
    }
    for (size_t i = 0; i + 1 < got; i += 2) {
      int16_t sample = int16_t(uint16_t(bytes[i]) | (uint16_t(bytes[i + 1]) << 8));
      minSample = min(minSample, sample);
      maxSample = max(maxSample, sample);
      sumAbs += abs(sample);
      statSamples++;
      audioData[outIndex++] = uint8_t((int(sample) + 32768) >> 8);
    }
  }
  file.close();

  Serial.printf("audio: bytes=%lu samples=%lu seconds=%.2f fnv1a=0x%08lx\n",
                fileBytes, statSamples, double(statSamples) / SAMPLE_RATE, fileHash);
  Serial.printf("audio: min=%d max=%d avg_abs=%ld\n",
                minSample, maxSample, long(statSamples ? sumAbs / statSamples : 0));
  return fileBytes >= 2 && (fileBytes % 2) == 0;
}

void startAudio() {
  pinMode(AMP_INPUT_NODE_PIN, INPUT);
  pinMode(AMP_ENABLE_PIN, OUTPUT);
  digitalWrite(AMP_ENABLE_PIN, LOW);

  double pwmResult = ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_BITS);
  ledcAttachPin(AUDIO_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 128);

  portENTER_CRITICAL(&audioMux);
  activeLength = statSamples;
  activeIndex = 0;
  playbackDone = statSamples == 0;
  sampleCount = 0;
  underruns = 0;
  loopCount = 0;
  portEXIT_CRITICAL(&audioMux);

  audioTimer = timerBegin(0, TIMER_DIVIDER, true);
  timerAttachInterrupt(audioTimer, &onAudioTimer, true);
  timerAlarmWrite(audioTimer, TIMER_TICKS, true);
  timerAlarmEnable(audioTimer);

  Serial.printf("playback: pwm_request=%dHz pwm_actual=%.1fHz/%dbit timer_ticks=%d actual_sample_rate=%dHz memory_samples=%lu\n",
                PWM_FREQ, pwmResult, PWM_BITS, TIMER_TICKS, ACTUAL_SAMPLE_RATE,
                statSamples);
  printPins();
}

void serviceAudio() {
  static uint32_t lastReport = 0;
  static uint32_t startMs = millis();

  if (millis() - lastReport >= 500) {
    lastReport = millis();
    Serial.printf("playback: ms=%lu samples=%lu underruns=%lu last_pwm=%u heap=%u\n",
                  millis() - startMs, sampleCount, underruns, lastDuty,
                  heap_caps_get_free_size(MALLOC_CAP_8BIT));
  }

  if (playbackDone) {
    delay(80);
    ledcWrite(PWM_CHANNEL, 128);
    loopCount++;
    Serial.printf("playback: loop=%lu samples=%lu expected=%lu underruns=%lu elapsed_ms=%lu\n",
                  loopCount, sampleCount, statSamples, underruns, millis() - startMs);
    printPins();
    delay(350);
    portENTER_CRITICAL(&audioMux);
    activeIndex = 0;
    playbackDone = false;
    sampleCount = 0;
    underruns = 0;
    portEXIT_CRITICAL(&audioMux);
    startMs = millis();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("speaker-audio: boot");
  Serial.printf("build: %s %s\n", __DATE__, __TIME__);
  Serial.printf("heap: %u\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));

  if (!LittleFS.begin(false)) {
    Serial.println("fs: LittleFS mount failed");
    return;
  }
  Serial.println("fs: LittleFS mounted");

  if (!inspectAudioFile()) {
    Serial.println("audio: invalid file, stopping");
    return;
  }

  startAudio();
}

void loop() {
  serviceAudio();
}
