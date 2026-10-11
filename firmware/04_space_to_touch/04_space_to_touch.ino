/*
 * 04_space_to_touch
 * 통합 펌웨어: 센서 6개 → 필터 → 거리 단계 → 진동 패턴
 *
 * 번호 규칙 (센서 = 멀티플렉서 채널 = 모터)
 *           왼쪽  가운데  오른쪽
 *   위 줄     0     1      2
 *   아래 줄   3     4      5
 *
 * 시리얼 명령 (115200 baud)
 *   d : 디버그 출력 켜기/끄기 (거리,단계)
 *
 * 조정할 값은 아래 "설정" 부분에 모여 있다. 바꿀 때마다 이유를 일지에 남길 것.
 */

#include <Wire.h>
#include <VL53L1X.h>

// ───────── 설정 ─────────
const uint8_t NUM_ZONES = 6;
const uint8_t MUX_ADDR = 0x70;
const uint32_t TIMING_BUDGET_US = 50000;

// 단계 경계 (mm). LEVEL_MM[k] 보다 가까우면 k+1 단계 이상
const uint16_t LEVEL_MM[3] = {1500, 1000, 500};
const uint16_t HYSTERESIS_MM = 50;

// 단계별 진동 패턴: 켜짐 시간 / 주기 (ms). 3단계는 계속 진동
const uint16_t PULSE_ON_MS[4] = {0, 80, 80, 1};
const uint16_t PULSE_PERIOD_MS[4] = {1, 700, 300, 1};

const uint8_t MAX_DUTY = 150;          // 3V 모터를 5V로 구동 → 약 2.9V
const uint16_t STALE_MS = 500;         // 이 시간 동안 새 값이 없으면 센서 고장으로 판단
const uint16_t ERROR_PERIOD_MS = 2000; // 고장 알림 주기

#if defined(ARDUINO_AVR_MEGA2560)
const uint8_t MOTOR_PINS[NUM_ZONES] = {2, 3, 5, 6, 7, 8};
#else
const uint8_t MOTOR_PINS[NUM_ZONES] = {3, 5, 6, 9, 10, 11};
#endif
// ────────────────────────

const uint16_t NO_TARGET = 0xFFFF;  // 감지 없음 = 무한히 멀다고 취급

VL53L1X sensors[NUM_ZONES];
uint16_t history[NUM_ZONES][3];     // 중앙값 필터용 최근 3개 값
uint8_t historyIdx[NUM_ZONES];
uint16_t filtered[NUM_ZONES];
uint8_t level[NUM_ZONES];
uint32_t patternStart[NUM_ZONES];
uint32_t lastUpdate[NUM_ZONES];
bool debug = false;

bool selectChannel(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  return Wire.endTransmission() == 0;
}

uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { b = c; }
  return a > b ? a : b;
}

uint8_t rawLevel(uint16_t mm) {
  uint8_t lv = 0;
  for (uint8_t k = 0; k < 3; k++) {
    if (mm < LEVEL_MM[k]) lv = k + 1;
  }
  return lv;
}

// 가까워질 때는 즉시 단계를 올리고, 멀어질 때는 HYSTERESIS_MM 만큼 더 멀어져야 내린다
uint8_t nextLevel(uint16_t mm, uint8_t prev) {
  uint8_t raw = rawLevel(mm);
  if (raw >= prev) return raw;
  uint16_t shifted = (mm == NO_TARGET || mm <= HYSTERESIS_MM) ? mm : mm - HYSTERESIS_MM;
  uint8_t held = rawLevel(shifted);
  return held < prev ? held : prev;
}

void allMotors(uint8_t duty) {
  for (uint8_t i = 0; i < NUM_ZONES; i++) analogWrite(MOTOR_PINS[i], duty);
}

// 시작할 때 모터를 0번부터 차례로 울려서 사용자가 위치를 익히고, 배선도 확인한다
void startupSequence() {
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    analogWrite(MOTOR_PINS[i], MAX_DUTY);
    delay(200);
    analogWrite(MOTOR_PINS[i], 0);
    delay(100);
  }
}

void readSensors(uint32_t now) {
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    selectChannel(i);
    if (!sensors[i].dataReady()) continue;

    uint16_t mm = sensors[i].read(false);
    if (sensors[i].ranging_data.range_status != VL53L1X::RangeValid) mm = NO_TARGET;

    history[i][historyIdx[i]] = mm;
    historyIdx[i] = (historyIdx[i] + 1) % 3;
    filtered[i] = median3(history[i][0], history[i][1], history[i][2]);
    lastUpdate[i] = now;

    uint8_t lv = nextLevel(filtered[i], level[i]);
    if (lv != level[i]) {
      level[i] = lv;
      patternStart[i] = now;  // 단계가 바뀌면 새 패턴을 바로 시작
    }
  }
}

bool anySensorStale(uint32_t now) {
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    if (now - lastUpdate[i] > STALE_MS) return true;
  }
  return false;
}

// 고장 알림: 전체 모터를 짧게 두 번 ("드르륵 드르륵")
void driveErrorPattern(uint32_t now) {
  uint16_t phase = now % ERROR_PERIOD_MS;
  bool on = phase < 100 || (phase >= 200 && phase < 300);
  allMotors(on ? MAX_DUTY : 0);
}

void driveMotors(uint32_t now) {
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    uint8_t lv = level[i];
    bool on;
    if (lv == 0) on = false;
    else if (lv == 3) on = true;
    else on = (now - patternStart[i]) % PULSE_PERIOD_MS[lv] < PULSE_ON_MS[lv];
    analogWrite(MOTOR_PINS[i], on ? MAX_DUTY : 0);
  }
}

void printDebug(uint32_t now) {
  static uint32_t lastPrint = 0;
  if (!debug || now - lastPrint < 200) return;
  lastPrint = now;
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    if (i) Serial.print(',');
    if (filtered[i] == NO_TARGET) Serial.print('-');
    else Serial.print(filtered[i]);
    Serial.print(':');
    Serial.print(level[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    analogWrite(MOTOR_PINS[i], 0);
  }

  bool ok = selectChannel(0);
  for (uint8_t i = 0; ok && i < NUM_ZONES; i++) {
    selectChannel(i);
    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.print(F("# 센서 "));
      Serial.print(i);
      Serial.println(F(" 초기화 실패"));
      ok = false;
      break;
    }
    sensors[i].setDistanceMode(VL53L1X::Long);
    sensors[i].setMeasurementTimingBudget(TIMING_BUDGET_US);
    sensors[i].startContinuous(TIMING_BUDGET_US / 1000);

    for (uint8_t k = 0; k < 3; k++) history[i][k] = NO_TARGET;
    filtered[i] = NO_TARGET;
  }

  if (!ok) {
    // 초기화 실패도 진동으로 알린다 (사용자는 화면을 볼 수 없다)
    Serial.println(F("# 초기화 실패: 배선을 확인하세요."));
    while (true) driveErrorPattern(millis());
  }

  startupSequence();
  uint32_t now = millis();
  for (uint8_t i = 0; i < NUM_ZONES; i++) lastUpdate[i] = now;
}

void loop() {
  if (Serial.available() && Serial.read() == 'd') debug = !debug;

  uint32_t now = millis();
  readSensors(now);

  if (anySensorStale(now)) driveErrorPattern(now);
  else driveMotors(now);

  printDebug(now);
}
