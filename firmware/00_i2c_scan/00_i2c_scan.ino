/*
 * 00_i2c_scan
 * 부품 확인용: I2C 버스에 무엇이 연결되어 있는지 찾는다.
 *   - 0x29에 응답한 장치는 모델 ID를 읽어 진짜 VL53L1X인지 확인한다 (VL53L0X와 헷갈리기 쉽다).
 *   - 멀티플렉서(0x70)가 있으면 채널 0~7을 하나씩 열어 각 채널의 장치도 찾는다.
 *
 * 배선: 01(센서 1개 직접 연결) 또는 02(멀티플렉서) 배선 그대로
 * 시리얼 (115200 baud): 아무 키나 보내면 다시 검사한다.
 *   가진 센서 6개를 채널 0~5에 꽂으면 한 번에 검사할 수 있다. 장치에는 그중 3개(채널 0~2)를 쓰고 나머지는 예비로 둔다.
 */

#include <Wire.h>

const uint8_t MUX_ADDR = 0x70;
const uint8_t TOF_ADDR = 0x29;
const uint16_t VL53L1X_MODEL_ID = 0xEACC;  // VL53L1X의 레지스터 0x010F에 들어 있는 값
const uint8_t NUM_SENSORS = 6;             // 점검용: 채널 0~5를 검사한다 (장치는 0~2만 쓴다)

bool present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// VL53L1X는 레지스터 주소가 16비트다. 읽기 실패 시 -1
int32_t readModelId() {
  Wire.beginTransmission(TOF_ADDR);
  Wire.write(0x01);
  Wire.write(0x0F);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom(TOF_ADDR, (uint8_t)2) != 2) return -1;
  uint16_t id = (uint16_t)Wire.read() << 8;
  id |= Wire.read();
  return id;
}

// 0x29 장치의 모델 ID를 출력하고, VL53L1X이면 true
bool checkTof() {
  int32_t id = readModelId();
  if (id == VL53L1X_MODEL_ID) {
    Serial.println(F("    → VL53L1X 확인"));
    return true;
  }
  if (id < 0) {
    Serial.println(F("    → 모델 ID를 읽지 못함"));
  } else {
    Serial.print(F("    → 모델 ID 0x"));
    Serial.print((uint16_t)id, HEX);
    Serial.println(F(": VL53L1X가 아님 (VL53L0X 등 다른 센서인지 판매 페이지 확인)"));
  }
  return false;
}

// 버스를 훑어 찾은 장치를 출력한다. skip 주소(멀티플렉서 자신)는 건너뛴다.
// 반환값: 찾은 장치 수. tofOk에는 VL53L1X 확인 여부를 담는다.
uint8_t scanBus(int16_t skip, bool &tofOk) {
  uint8_t found = 0;
  tofOk = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (addr == skip || !present(addr)) continue;
    found++;
    Serial.print(F("  0x"));
    if (addr < 16) Serial.print('0');
    Serial.print(addr, HEX);
    if (addr == MUX_ADDR) Serial.print(F("  TCA9548A 멀티플렉서"));
    if (addr == TOF_ADDR) Serial.print(F("  ToF 센서 주소"));
    if (addr == 0x68) Serial.print(F("  MPU-6050 (머리 각도 센서, 멀티플렉서 채널마다 보이는 것이 정상)"));
    Serial.println();
    if (addr == TOF_ADDR) tofOk = checkTof();
  }
  if (found == 0) Serial.println(F("  장치 없음"));
  return found;
}

// ch < 0 이면 모든 채널을 닫는다
void selectChannel(int8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(ch < 0 ? 0 : (1 << ch));
  Wire.endTransmission();
}

void runScan() {
  bool tofOk;
  Serial.println();
  Serial.println(F("=== I2C 검사 ==="));

  bool mux = present(MUX_ADDR);
  if (mux) selectChannel(-1);  // 이전에 열려 있던 채널을 닫고 본 버스만 본다

  Serial.println(F("[본 버스]"));
  if (scanBus(-1, tofOk) == 0) {
    Serial.println(F("# 아무것도 없음: 전원(VIN, GND), SDA/SCL 위치, 납땜 상태를 확인하세요"));
    return;
  }
  if (!mux) return;

  bool ok[NUM_SENSORS];
  for (uint8_t ch = 0; ch < 8; ch++) {
    Serial.print(F("[채널 "));
    Serial.print(ch);
    Serial.println(']');
    selectChannel(ch);
    scanBus(MUX_ADDR, tofOk);
    if (ch < NUM_SENSORS) ok[ch] = tofOk;
  }
  selectChannel(-1);

  // 점검 기록표(docs/parts.md)에 옮겨 적기 좋은 한 줄 요약
  Serial.print(F("# 채널별 VL53L1X:"));
  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
    Serial.print(' ');
    Serial.print(ch);
    Serial.print(ok[ch] ? F("=O") : F("=X"));
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  runScan();
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    runScan();
  }
}
