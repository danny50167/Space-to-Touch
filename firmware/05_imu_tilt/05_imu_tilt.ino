/*
 * 05_imu_tilt
 * MPU-6050으로 고개를 숙인 각도(pitch)를 잰다. 세 가지 방법을 나란히 출력해 비교한다.
 *   가속도  : 중력 방향으로 계산한다. 가만히 있으면 정확하지만, 걸을 때는 발걸음 충격 때문에 튄다.
 *   자이로  : 회전 속도(°/s)를 시간에 대해 더한다(적분). 부드럽지만 시간이 지나면 조금씩 어긋난다(드리프트).
 *   상보 필터: 둘을 섞는다. 각도 = α·(이전 각도 + 회전 속도·Δt) + (1 − α)·가속도 각도
 *
 * 배선 (docs/wiring.md 4장)
 *   VCC → 5V (GY-521 모듈은 레귤레이터 내장), GND → GND
 *   SDA, SCL → 아두이노 SDA, SCL에 바로 연결 (멀티플렉서를 거치지 않는다)
 *   AD0 → GND (주소 0x68)
 * 장착: 센서 바에 단단히 고정한다. 모듈 기판의 X축 화살표가 앞(센서가 보는 쪽)을 향하게 한다.
 *
 * 출력 (115200 baud, 50 Hz)
 *   CSV: 시간(ms),가속도각,자이로각,상보필터각,예상바닥거리(mm)
 *   각도는 아래로 숙이면 +. 예상 바닥 거리는 시야 아래 끝이 바닥에 닿는 거리 (-1 = 바닥에 안 닿음)
 *
 * 시리얼 명령
 *   z     : 지금 자세를 0°로 정한다 (정면을 보고 누른다)
 *   + / - : 상보 필터 α를 0.01씩 조절한다
 *
 * 처음 할 일: 고개를 숙였을 때 가속도각과 자이로각이 모두 +로 커지는지 확인한다.
 *   가속도각이 반대로 움직이면 ACC_SIGN을, 자이로각이 반대면 GYRO_SIGN을 -1로 바꾼다.
 */

#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;
const int8_t ACC_SIGN = 1;
const int8_t GYRO_SIGN = 1;
const uint16_t SAMPLE_MS = 20;              // 50 Hz
const uint16_t BIAS_SAMPLES = 200;          // 시작할 때 자이로 영점을 잡는 표본 수 (약 4초)

// 예상 바닥 거리 계산용. 통합 펌웨어(04)로 옮길 값이다
const float SENSOR_HEIGHT_MM = 1740;        // H: 바닥에서 센서까지 높이
const float MOUNT_TILT_DEG = 12;            // θ: 센서 줄을 아래로 기울인 각도 (출발점)
const float HALF_FOV_DEG = 13.5;            // 시야 반각

const float ACC_LSB_PER_G = 16384.0;        // ±2 g 범위
const float GYRO_LSB_PER_DPS = 131.0;       // ±250 °/s 범위

float alpha = 0.98;                         // 출발점. 실험으로 정한다
float gyroBias = 0;                         // 자이로 영점 (°/s)
float zeroOffset = 0;                       // 'z'로 정한 0° 기준
float accPitch = 0, gyroPitch = 0, compPitch = 0;
uint32_t lastMs = 0;

int16_t ax, ay, az, gy;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t read16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

// 가속도 3축과 자이로 Y축을 읽는다 (레지스터 0x3B부터 14바이트)
bool readRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  ax = read16();
  ay = read16();
  az = read16();
  read16();  // 온도
  read16();  // 자이로 X
  gy = read16();
  read16();  // 자이로 Z
  return true;
}

// 중력 방향으로 구한 각도. 앞(X)으로 숙이면 X축에 걸리는 중력 성분이 sin(각도)만큼 생긴다
float accelPitchDeg() {
  float x = ax / ACC_LSB_PER_G, y = ay / ACC_LSB_PER_G, z = az / ACC_LSB_PER_G;
  return ACC_SIGN * atan2(-x, sqrt(y * y + z * z)) * 180.0 / PI;
}

float gyroRateDps() {
  return GYRO_SIGN * (gy / GYRO_LSB_PER_DPS) - gyroBias;
}

void calibrateGyro() {
  Serial.println(F("# 자이로 영점 잡는 중. 센서를 가만히 두세요."));
  float sum = 0;
  uint16_t n = 0;
  while (n < BIAS_SAMPLES) {
    if (readRaw()) {
      sum += GYRO_SIGN * (gy / GYRO_LSB_PER_DPS);
      n++;
    }
    delay(SAMPLE_MS);
  }
  gyroBias = sum / n;
  Serial.print(F("# 자이로 영점: "));
  Serial.print(gyroBias, 3);
  Serial.println(F(" °/s"));
}

// 시야 아래 끝이 바닥에 닿는 거리. 아래를 보지 않으면 -1
float floorDistanceMm(float pitch) {
  float beta = (MOUNT_TILT_DEG + pitch + HALF_FOV_DEG) * PI / 180.0;
  if (beta <= 0.01) return -1;
  return SENSOR_HEIGHT_MM / sin(beta);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("# MPU-6050(0x68)을 찾을 수 없음. 배선과 AD0를 확인하세요."));
    while (true) {}
  }
  writeReg(0x6B, 0x00);  // 절전 모드 해제
  writeReg(0x1A, 0x03);  // 내부 저역 통과 필터 약 44 Hz
  writeReg(0x1B, 0x00);  // 자이로 ±250 °/s
  writeReg(0x1C, 0x00);  // 가속도 ±2 g
  delay(100);

  calibrateGyro();
  while (!readRaw()) {}
  accPitch = gyroPitch = compPitch = accelPitchDeg();
  zeroOffset = compPitch;
  lastMs = millis();
  Serial.println(F("# z: 지금 자세를 0°로, +/-: α 조절"));
  Serial.println(F("ms,acc,gyro,comp,floor_mm"));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') {
      zeroOffset = compPitch;
      gyroPitch = compPitch;  // 자이로 각도의 드리프트를 이 순간부터 다시 본다
      Serial.println(F("# 0° 기준을 다시 잡았다"));
    } else if (c == '+' || c == '-') {
      alpha = constrain(alpha + (c == '+' ? 0.01 : -0.01), 0.0, 0.999);
      Serial.print(F("# α = "));
      Serial.println(alpha, 3);
    }
  }

  uint32_t now = millis();
  if (now - lastMs < SAMPLE_MS) return;
  float dt = (now - lastMs) / 1000.0;
  lastMs = now;
  if (!readRaw()) return;

  float rate = gyroRateDps();
  accPitch = accelPitchDeg();
  gyroPitch += rate * dt;
  compPitch = alpha * (compPitch + rate * dt) + (1 - alpha) * accPitch;

  float pitch = compPitch - zeroOffset;
  Serial.print(now);
  Serial.print(',');
  Serial.print(accPitch - zeroOffset, 2);
  Serial.print(',');
  Serial.print(gyroPitch - zeroOffset, 2);
  Serial.print(',');
  Serial.print(pitch, 2);
  Serial.print(',');
  Serial.println(floorDistanceMm(pitch), 0);
}
