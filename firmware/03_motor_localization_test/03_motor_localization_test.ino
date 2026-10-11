/*
 * 03_motor_localization_test
 * 진동 위치 인식 실험: 무작위 모터를 울리고, 참가자가 몇 번 모터였는지 답한다.
 * 착용 부위(이마 / 팔뚝 / 복부)마다 실험해서 정확도를 비교한다.
 *
 * 모터 번호 (참가자가 답하는 번호는 1~6)
 *           왼쪽  가운데  오른쪽
 *   위 줄     1     2      3
 *   아래 줄   4     5      6
 *
 * 시리얼 명령 (115200 baud, 줄 끝: 새 줄)
 *   대기 상태:
 *     1~6 : 해당 모터를 1초 진동 (배선 확인용)
 *     + / - : 진동 세기(듀티) 조절
 *     t   : 실험 시작 (모터마다 TRIALS_PER_MOTOR번, 무작위 순서)
 *   실험 중:
 *     1~6 : 참가자의 답 입력 (실험자가 대신 입력)
 *     r   : 같은 자극 다시 주기
 *
 * 결과: 정확도, 혼동 행렬, 시행별 CSV → 엑셀에 붙여넣어 분석
 *
 * 주의: 코인 모터는 소리가 나므로 참가자가 소리로 위치를 맞힐 수 있다.
 *       참가자는 안대와 함께 이어폰으로 백색소음을 듣게 한다 (실험 설계의 통제 변인).
 */

#if defined(ARDUINO_AVR_MEGA2560)
const uint8_t MOTOR_PINS[6] = {2, 3, 5, 6, 7, 8};
#else
const uint8_t MOTOR_PINS[6] = {3, 5, 6, 9, 10, 11};
#endif

const uint8_t NUM_MOTORS = 6;
const uint8_t TRIALS_PER_MOTOR = 5;
const uint8_t NUM_TRIALS = NUM_MOTORS * TRIALS_PER_MOTOR;
const uint16_t STIMULUS_MS = 400;

uint8_t duty = 150;  // 0~255. 모듈의 모터가 3V 정격이면 5V 전원에서 약 150 이하 권장

uint8_t order[NUM_TRIALS];
uint8_t answers[NUM_TRIALS];

void vibrate(uint8_t motor, uint16_t ms) {
  analogWrite(MOTOR_PINS[motor], duty);
  delay(ms);
  analogWrite(MOTOR_PINS[motor], 0);
}

// 1~6 입력을 기다린다. 'r'이 오면 자극을 다시 준다.
uint8_t waitAnswer(uint8_t motor) {
  while (true) {
    if (!Serial.available()) continue;
    char c = Serial.read();
    if (c >= '1' && c <= '6') return c - '1';
    if (c == 'r') vibrate(motor, STIMULUS_MS);
  }
}

void shuffleOrder() {
  for (uint8_t i = 0; i < NUM_TRIALS; i++) order[i] = i % NUM_MOTORS;
  for (uint8_t i = NUM_TRIALS - 1; i > 0; i--) {
    uint8_t j = random(i + 1);
    uint8_t tmp = order[i];
    order[i] = order[j];
    order[j] = tmp;
  }
}

void printResults() {
  uint8_t confusion[NUM_MOTORS][NUM_MOTORS] = {};
  uint8_t correct = 0;
  for (uint8_t t = 0; t < NUM_TRIALS; t++) {
    confusion[order[t]][answers[t]]++;
    if (order[t] == answers[t]) correct++;
  }

  Serial.print(F("# 정확도: "));
  Serial.print(correct);
  Serial.print('/');
  Serial.print(NUM_TRIALS);
  Serial.print(F(" = "));
  Serial.print(100.0 * correct / NUM_TRIALS, 1);
  Serial.print(F("%   (듀티 "));
  Serial.print(duty);
  Serial.println(')');

  Serial.println(F("# 혼동 행렬 (행: 실제 모터, 열: 참가자 답)"));
  Serial.println(F("#     1  2  3  4  5  6"));
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    Serial.print(F("#  "));
    Serial.print(i + 1);
    for (uint8_t j = 0; j < NUM_MOTORS; j++) {
      Serial.print(confusion[i][j] < 10 ? F("  ") : F(" "));
      Serial.print(confusion[i][j]);
    }
    Serial.println();
  }

  Serial.println(F("trial,actual,answer"));
  for (uint8_t t = 0; t < NUM_TRIALS; t++) {
    Serial.print(t + 1);
    Serial.print(',');
    Serial.print(order[t] + 1);
    Serial.print(',');
    Serial.println(answers[t] + 1);
  }
}

void runTest() {
  shuffleOrder();
  Serial.println(F("# 실험 시작. 참가자의 답(1~6)을 입력하세요."));

  for (uint8_t t = 0; t < NUM_TRIALS; t++) {
    delay(1000);
    Serial.print(F("# 시행 "));
    Serial.print(t + 1);
    Serial.print('/');
    Serial.println(NUM_TRIALS);
    vibrate(order[t], STIMULUS_MS);
    answers[t] = waitAnswer(order[t]);
  }

  printResults();
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    analogWrite(MOTOR_PINS[i], 0);
  }
  randomSeed(analogRead(A0));  // 연결하지 않은 핀의 잡음으로 매번 다른 순서
  Serial.println(F("# 1~6: 모터 확인, +/-: 세기, t: 실험 시작"));
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();

  if (c >= '1' && c <= '6') {
    vibrate(c - '1', 1000);
  } else if (c == '+' || c == '-') {
    duty = (c == '+') ? min(255, duty + 15) : max(0, duty - 15);
    Serial.print(F("# 듀티: "));
    Serial.println(duty);
  } else if (c == 't') {
    runTest();
  }
}
