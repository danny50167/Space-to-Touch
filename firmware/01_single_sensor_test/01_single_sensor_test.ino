/*
 * 01_single_sensor_test
 * VL53L1X 센서 1개로 거리 모드를 비교하고 정확도를 측정한다.
 *
 * 배선: 센서를 아두이노 I2C에 직접 연결 (멀티플렉서 없이)
 *   Mega: SDA=20, SCL=21 / Uno: SDA=A4, SCL=A5
 *
 * 시리얼 명령 (115200 baud, 줄 끝: 새 줄)
 *   s / m / l : 거리 모드 Short / Medium / Long
 *   p         : 실시간 출력 켜기/끄기
 *   a         : 50회 측정 → 평균, 표준편차, 유효 비율 출력 (정확도 실험용)
 */

#include <Wire.h>
#include <VL53L1X.h>

const uint32_t TIMING_BUDGET_US = 50000;  // 측정 1회에 쓰는 시간 (길수록 정확, 느림)
const uint8_t SAMPLES = 50;

VL53L1X sensor;
bool streaming = true;

void applyMode(VL53L1X::DistanceMode mode, const __FlashStringHelper *name) {
  sensor.stopContinuous();
  sensor.setDistanceMode(mode);
  sensor.setMeasurementTimingBudget(TIMING_BUDGET_US);
  sensor.startContinuous(TIMING_BUDGET_US / 1000);
  Serial.print(F("# 거리 모드: "));
  Serial.println(name);
}

void measureStats() {
  uint16_t values[SAMPLES];
  uint8_t valid = 0;

  for (uint8_t i = 0; i < SAMPLES; i++) {
    uint16_t mm = sensor.read();
    if (sensor.timeoutOccurred()) continue;
    if (sensor.ranging_data.range_status != VL53L1X::RangeValid) continue;
    values[valid++] = mm;
  }

  if (valid == 0) {
    Serial.println(F("# 유효한 측정값 없음"));
    return;
  }

  float mean = 0;
  for (uint8_t i = 0; i < valid; i++) mean += values[i];
  mean /= valid;

  float var = 0;
  for (uint8_t i = 0; i < valid; i++) var += (values[i] - mean) * (values[i] - mean);
  float sd = sqrt(var / valid);

  // 표에 바로 붙여넣을 수 있는 형식: 평균,표준편차,유효개수/전체
  Serial.print(F("STATS,"));
  Serial.print(mean, 1);
  Serial.print(',');
  Serial.print(sd, 1);
  Serial.print(',');
  Serial.print(valid);
  Serial.print('/');
  Serial.println(SAMPLES);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println(F("# 센서를 찾을 수 없음. 배선을 확인하세요."));
    while (true) {}
  }
  applyMode(VL53L1X::Long, F("Long"));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') applyMode(VL53L1X::Short, F("Short"));
    else if (c == 'm') applyMode(VL53L1X::Medium, F("Medium"));
    else if (c == 'l') applyMode(VL53L1X::Long, F("Long"));
    else if (c == 'p') streaming = !streaming;
    else if (c == 'a') measureStats();
  }

  if (streaming && sensor.dataReady()) {
    uint16_t mm = sensor.read(false);
    Serial.print(mm);
    Serial.print(F(" mm  "));
    Serial.println(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
  }
}
