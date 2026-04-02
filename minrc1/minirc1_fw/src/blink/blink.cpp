#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

constexpr uint8_t kWs2812Pin = 48;
constexpr uint16_t kPixelCount = 1;
constexpr uint16_t kFrameDelayMs = 20;
constexpr uint32_t kPrintIntervalMs = 1000;

Adafruit_NeoPixel pixels(
  kPixelCount,
  kWs2812Pin,
  NEO_GRB + NEO_KHZ800
);

uint16_t hue = 0;
uint32_t lastPrintMs = 0;

void setup() {
  // With ARDUINO_USB_CDC_ON_BOOT enabled, Serial is USB CDC (not UART0).
  Serial.begin(115200);
  const uint32_t waitStart = millis();
  while (!Serial && (millis() - waitStart) < 3000) {
    delay(10);
  }

  pixels.begin();
  pixels.setBrightness(64);
  pixels.show();

  Serial.println("WS2812 rainbow start on GPIO48 (USB CDC Serial)");
}

void loop() {
  const uint32_t color = pixels.gamma32(pixels.ColorHSV(static_cast<uint32_t>(hue) * 256u));
  pixels.setPixelColor(0, color);
  pixels.show();

  const uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= kPrintIntervalMs) {
    lastPrintMs = nowMs;
    Serial.print("hue=");
    Serial.print(hue);
    Serial.print(" color=0x");
    Serial.println(color, HEX);
  }

  hue += 8;
  delay(kFrameDelayMs);
}