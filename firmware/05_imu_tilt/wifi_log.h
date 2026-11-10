/*
 * wifi_log.h : 무선 기록 (ESP32 전용)
 *
 * ESP32가 직접 Wi-Fi를 만든다. 맥북을 이 Wi-Fi에 연결하고 터미널에 아래 명령을 치면
 * 시리얼 모니터와 같은 내용이 들어오고, 파일로도 저장된다.
 *     nc 192.168.4.1 2323 | tee 파일이름.csv
 * 그 터미널에 글자를 치고 Enter를 누르면 시리얼 명령(c, q, z, d 등)도 보낼 수 있다.
 * 자세한 방법은 docs/esp32.md.
 *
 * 스케치에서는 Serial 대신 Log를 쓴다. USB 시리얼과 무선으로 똑같이 나간다.
 * ESP32가 아닌 보드(Mega, Uno)에서는 Log가 그냥 Serial이다.
 *
 * 02, 04, 05 폴더에 같은 파일이 들어 있다. 고칠 때는 세 개를 똑같이 고친다.
 */
#pragma once
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>

const char *const WIFI_NAME = "SpaceToTouch";
const char *const WIFI_PASSWORD = "touch1234";  // 8자 이상. 원하는 것으로 바꿔도 된다
const uint16_t LOG_PORT = 2323;

class WifiLog : public Stream {
 public:
  void begin(unsigned long baud) {
    Serial.begin(baud);
    WiFi.softAP(WIFI_NAME, WIFI_PASSWORD);
    server.begin();
    server.setNoDelay(true);
    Serial.print("# Wi-Fi 이름: ");
    Serial.print(WIFI_NAME);
    Serial.print("  주소: ");
    Serial.print(WiFi.softAPIP());
    Serial.print(' ');
    Serial.println(LOG_PORT);
  }

  // USB로는 바로 보내고, 무선으로는 한 줄씩 모아서 보낸다 (한 글자씩 보내면 느리다)
  size_t write(uint8_t c) override {
    Serial.write(c);
    line[len++] = c;
    if (c == '\n' || len == sizeof(line)) sendLine();
    return 1;
  }

  size_t write(const uint8_t *buf, size_t n) override {
    for (size_t i = 0; i < n; i++) write(buf[i]);
    return n;
  }

  // 명령은 USB와 무선 양쪽에서 받는다. 새로 연결한 맥북도 여기서 받아들인다
  int available() override {
    checkClient();
    int n = Serial.available();
    if (n > 0) return n;
    return client ? client.available() : 0;
  }

  int read() override {
    if (Serial.available()) return Serial.read();
    return client ? client.read() : -1;
  }

  int peek() override {
    if (Serial.available()) return Serial.peek();
    return client ? client.peek() : -1;
  }

 private:
  WiFiServer server{LOG_PORT};
  WiFiClient client;
  uint8_t line[128];
  size_t len = 0;

  void checkClient() {
    if (client && client.connected()) return;
    WiFiClient c = server.accept();
    if (c) {
      client = c;
      client.setNoDelay(true);
    }
  }

  void sendLine() {
    if (client && client.connected()) client.write(line, len);
    len = 0;
  }
};

WifiLog Log;

#else
#define Log Serial
#endif
