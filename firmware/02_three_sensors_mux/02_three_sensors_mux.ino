/*
 * 02_three_sensors_mux
 * TCA9548A 멀티플렉서로 VL53L1X 3개를 읽고, 측정 방식별 갱신 속도를 비교한다.
 *
 * 센서 번호 = 멀티플렉서 채널 번호 (가로 한 줄, 머리 높이)
 *   왼쪽  가운데  오른쪽
 *    0      1      2
 *
 * 출력 (115200 baud)
 *   CSV 한 줄: 시간(ms),d0,d1,d2   (-1 = 감지 없음 / 무효)
 *   2초마다:  # 센서별 갱신 속도(Hz)
 *   → 시리얼 플로터로 보면 3개 값을 한 번에 비교할 수 있다.
 *
 * 시리얼 명령
 *   c : 연속 측정 (3개가 동시에 측정 → 빠름, 적외선 간섭 가능성 있음)
 *   q : 순차 측정 (한 번에 1개만 측정 → 느림, 간섭 없음)
 */

#include <Wire.h>
#include <VL53L1X.h>

const uint8_t NUM_SENSORS = 3;
const uint8_t MUX_ADDR = 0x70;
const uint32_t TIMING_BUDGET_US = 50000;
const uint16_t PRINT_INTERVAL_MS = 100;
const uint16_t RATE_INTERVAL_MS = 2000;

VL53L1X sensors[NUM_SENSORS];
bool sensorOk[NUM_SENSORS];
int16_t distance[NUM_SENSORS];
uint16_t updates[NUM_SENSORS];
bool continuousMode = true;

uint32_t lastPrint = 0;
uint32_t lastRate = 0;

bool selectChannel(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  return Wire.endTransmission() == 0;
}

void storeResult(uint8_t i, uint16_t mm) {
  bool valid = !sensors[i].timeoutOccurred() &&
               sensors[i].ranging_data.range_status == VL53L1X::RangeValid;
  distance[i] = valid ? (int16_t)mm : -1;
  updates[i]++;
}

void setContinuous(bool on) {
  continuousMode = on;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (!sensorOk[i]) continue;
    selectChannel(i);
    if (on) sensors[i].startContinuous(TIMING_BUDGET_US / 1000);
    else sensors[i].stopContinuous();
  }
  Serial.println(on ? F("# 연속 측정 모드") : F("# 순차 측정 모드"));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!selectChannel(0)) {
    Serial.println(F("# 멀티플렉서(0x70)를 찾을 수 없음. 배선을 확인하세요."));
    while (true) {}
  }

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    selectChannel(i);
    sensors[i].setTimeout(500);
    sensorOk[i] = sensors[i].init();
    distance[i] = -1;
    if (sensorOk[i]) {
      sensors[i].setDistanceMode(VL53L1X::Long);
      sensors[i].setMeasurementTimingBudget(TIMING_BUDGET_US);
    } else {
      Serial.print(F("# 센서 "));
      Serial.print(i);
      Serial.println(F(" 초기화 실패"));
    }
  }

  setContinuous(true);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' && !continuousMode) setContinuous(true);
    else if (c == 'q' && continuousMode) setContinuous(false);
  }

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (!sensorOk[i]) continue;
    selectChannel(i);
    if (continuousMode) {
      // 측정이 끝난 센서만 읽는다 (기다리지 않음)
      if (sensors[i].dataReady()) storeResult(i, sensors[i].read(false));
    } else {
      // 이 센서 하나만 측정하고 끝날 때까지 기다린다
      storeResult(i, sensors[i].readSingle(true));
    }
  }

  uint32_t now = millis();

  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;
    Serial.print(now);
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      Serial.print(',');
      Serial.print(distance[i]);
    }
    Serial.println();
  }

  if (now - lastRate >= RATE_INTERVAL_MS) {
    float seconds = (now - lastRate) / 1000.0;
    lastRate = now;
    Serial.print(F("# Hz:"));
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      Serial.print(' ');
      Serial.print(updates[i] / seconds, 1);
      updates[i] = 0;
    }
    Serial.println();
  }
}
