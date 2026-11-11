// ============================================================
//  curve_fitting.ino  (과제코드: 20C10)
//  적외선 거리센서 자동 곡선맞춤(curve fitting) 프로그램
//
//  사용법
//   1) 시리얼 모니터: 1000000 baud, 줄 끝 "Newline"(새 줄) 으로 설정
//   2) 곡선맞춤 방식, 차수를 입력
//   3) 탁구공을 0, 5, 10, ..., 30 cm 위치에 차례로 놓고 엔터 입력
//   4) 측정이 끝나면 최소제곱법으로 곡선맞춤 후
//      아두이노에 바로 붙여넣을 수 있는 volt_to_distance() 함수를 출력
//   5) 이후 메뉴에서 다른 방식/차수로 다시 맞추기, 재측정, 실시간 검증 가능
// ============================================================

#include <math.h>

// ---------------- 설정 ----------------
#define PIN_IR      A0     // 적외선 센서 연결 핀
#define N_SAMPLES   50     // 한 번 측정할 때 읽는 횟수 (ir_sensor_filtered 의 n)
#define SAMPLE_POS  0.5    // 정렬 후 고를 위치 (0.5 = 중간값)
#define N_REPEAT    5      // 한 위치에서 중간값을 몇 번 구해 평균낼지
#define START_MM    0      // 첫 측정 위치 (mm)
#define STEP_MM     50     // 측정 간격 (mm) = 5 cm
#define N_POINTS    7      // 측정 위치 개수: 0, 5, ..., 30 cm
#define MAX_DEG     5      // 허용 최대 차수 (N_POINTS - 1 이하)

// 곡선맞춤 방식
#define METHOD_POLY 1      // d = f(x)          (x: 센서값 0~1023)
#define METHOD_INV  2      // d = f(1000 / x)   (적외선 센서는 거리 ∝ 1/전압 에 가까움)

// ---------------- 전역 변수 ----------------
float xs[N_POINTS];        // 측정한 센서값 (ADC)
float ys[N_POINTS];        // 실제 위치 (mm)
bool  measured = false;

int   method = METHOD_POLY;
int   degree = 2;
float coef[MAX_DEG + 1];   // 정규화 변수 t 에 대한 계수 (낮은 차수부터)
float tMean, tScale;       // t = (u - tMean) / tScale
float r2;

// ============================================================
//  시리얼 입력
// ============================================================
String readLine() {
  String s = "";
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') { s.trim(); return s; }
      if (c != '\r') s += c;
    }
  }
}

int readIntInRange(const __FlashStringHelper *prompt, int lo, int hi) {
  while (true) {
    Serial.print(prompt);
    String s = readLine();
    Serial.println(s);
    int v = s.toInt();
    if (s.length() > 0 && v >= lo && v <= hi) return v;
    Serial.print(F("  -> "));
    Serial.print(lo);
    Serial.print(F(" ~ "));
    Serial.print(hi);
    Serial.println(F(" 사이의 숫자를 입력하세요."));
  }
}

// ============================================================
//  측정
//  - 수업 예제의 ir_sensor_filtered() 로 튀는 잡음을 제거한 중간값을 얻고
//  - 그 중간값을 N_REPEAT 번 구해 평균 → 소수점 단위까지 안정된 값
// ============================================================
int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

// 수업 예제(20_example_1.ino)에서 제공된 필터 함수
unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose)
{
  unsigned int *ir_val, ret_val;
  unsigned int start_time;

  if (verbose >= 2)
    start_time = millis();

  if ((n == 0) || (n > 1000) || (position < 0.0) || (position > 1))
    return 0;

  if (position == 1.0)
    position = 0.999;

  ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL)
    return 0;

  for (int i = 0; i < n; i++)
    ir_val[i] = analogRead(PIN_IR);

  qsort(ir_val, n, sizeof(unsigned int), compare);
  ret_val = ir_val[(unsigned int)(n * position)];
  free(ir_val);

  if (verbose >= 2) {
    Serial.print("Elapsed time: "); Serial.print(millis() - start_time); Serial.println("ms");
  }
  return ret_val;
}

float measureADC() {
  long sum = 0;
  for (int i = 0; i < N_REPEAT; i++) {
    sum += ir_sensor_filtered(N_SAMPLES, SAMPLE_POS, 0);
    delay(10);
  }
  return (float)sum / N_REPEAT;
}

void measureAll() {
  Serial.println();
  Serial.println(F("===== 측정 시작 ====="));
  for (int i = 0; i < N_POINTS; i++) {
    ys[i] = START_MM + (float)STEP_MM * i;
    Serial.print(F("탁구공을 "));
    Serial.print(ys[i] / 10.0, 0);
    Serial.print(F(" cm 에 놓고 엔터를 누르세요... "));
    readLine();
    xs[i] = measureADC();
    Serial.print(F("ADC = "));
    Serial.println(xs[i], 1);
  }
  measured = true;
  Serial.println(F("===== 측정 완료 ====="));
}

// ============================================================
//  곡선맞춤 (최소제곱법, Householder QR)
//  - 변수 u 를 t = (u - 평균) / 반폭 으로 정규화해서 수치 오차를 줄임
// ============================================================
float toU(float x) {
  if (method == METHOD_INV) {
    if (x < 1) x = 1;
    return 1000.0 / x;
  }
  return x;
}

float evalT(float t) {                 // Horner
  float y = coef[degree];
  for (int k = degree - 1; k >= 0; k--) y = coef[k] + t * y;
  return y;
}

float predict(float x) {
  return evalT((toU(x) - tMean) / tScale);
}

bool fitCurve() {
  const int m = N_POINTS, n = degree + 1;
  float A[N_POINTS][MAX_DEG + 1];
  float b[N_POINTS];
  float v[N_POINTS];

  // 정규화 파라미터
  float uMin = toU(xs[0]), uMax = uMin, uSum = 0;
  for (int i = 0; i < m; i++) {
    float u = toU(xs[i]);
    uSum += u;
    if (u < uMin) uMin = u;
    if (u > uMax) uMax = u;
  }
  tMean = uSum / m;
  tScale = (uMax - uMin) / 2.0;
  if (tScale < 1e-6) return false;     // 측정값이 모두 같음

  // 행렬 구성: A[i][j] = t_i^j
  for (int i = 0; i < m; i++) {
    float t = (toU(xs[i]) - tMean) / tScale;
    float p = 1;
    for (int j = 0; j < n; j++) { A[i][j] = p; p *= t; }
    b[i] = ys[i];
  }

  // Householder QR
  for (int k = 0; k < n; k++) {
    float norm = 0;
    for (int i = k; i < m; i++) norm += A[i][k] * A[i][k];
    norm = sqrt(norm);
    if (norm < 1e-9) return false;
    float alpha = (A[k][k] > 0) ? -norm : norm;
    float vv = 0;
    for (int i = k; i < m; i++) {
      v[i] = A[i][k];
      if (i == k) v[i] -= alpha;
      vv += v[i] * v[i];
    }
    if (vv < 1e-12) continue;
    for (int j = k; j < n; j++) {
      float s = 0;
      for (int i = k; i < m; i++) s += v[i] * A[i][j];
      float f = 2 * s / vv;
      for (int i = k; i < m; i++) A[i][j] -= f * v[i];
    }
    float s = 0;
    for (int i = k; i < m; i++) s += v[i] * b[i];
    float f = 2 * s / vv;
    for (int i = k; i < m; i++) b[i] -= f * v[i];
  }

  // 후진 대입: R c = Q^T b
  for (int k = n - 1; k >= 0; k--) {
    float s = b[k];
    for (int j = k + 1; j < n; j++) s -= A[k][j] * coef[j];
    if (fabs(A[k][k]) < 1e-9) return false;
    coef[k] = s / A[k][k];
  }

  // 결정계수 R^2
  float yMean = 0;
  for (int i = 0; i < m; i++) yMean += ys[i];
  yMean /= m;
  float ssRes = 0, ssTot = 0;
  for (int i = 0; i < m; i++) {
    float e = ys[i] - predict(xs[i]);
    ssRes += e * e;
    ssTot += (ys[i] - yMean) * (ys[i] - yMean);
  }
  r2 = (ssTot > 0) ? 1 - ssRes / ssTot : 0;
  return true;
}

// ============================================================
//  결과 출력
// ============================================================
void printResult() {
  Serial.println();
  Serial.println(F("===== 측정값 vs 곡선 ====="));
  Serial.println(F("실제(mm)\tADC\t계산(mm)\t오차(mm)"));
  for (int i = 0; i < N_POINTS; i++) {
    float p = predict(xs[i]);
    Serial.print(ys[i], 1);   Serial.print('\t');
    Serial.print(xs[i], 1);   Serial.print('\t');
    Serial.print(p, 1);       Serial.print('\t');
    Serial.println(p - ys[i], 1);
  }
  Serial.print(F("R^2 = "));
  Serial.println(r2, 6);
  if (degree >= N_POINTS - 2)
    Serial.println(F("* 차수가 높아 과적합(over fitting) 위험이 있습니다."));

  Serial.println();
  Serial.println(F("===== 아래 함수를 복사해서 사용하세요 ====="));
  Serial.print(F("// "));
  Serial.print(method == METHOD_POLY ? F("polynomial") : F("inverse polynomial"));
  Serial.print(F(", degree "));
  Serial.print(degree);
  Serial.print(F(", R^2 = "));
  Serial.println(r2, 6);
  Serial.println(F("float volt_to_distance(int x)"));
  Serial.println(F("{"));
  if (method == METHOD_POLY)
    Serial.println(F("  float u = (float)x;"));
  else
    Serial.println(F("  float u = 1000.0 / (float)(x < 1 ? 1 : x);"));
  Serial.print(F("  float t = (u - ("));
  Serial.print(tMean, 6);
  Serial.print(F(")) / "));
  Serial.print(tScale, 6);
  Serial.println(F(";"));
  Serial.print(F("  return "));
  for (int k = 0; k <= degree; k++) {
    Serial.print(coef[k], 6);
    if (k < degree) Serial.print(F(" + t * ("));
  }
  for (int k = 0; k < degree; k++) Serial.print(')');
  Serial.println(F(";"));
  Serial.println(F("}"));
  Serial.println(F("=========================================="));
}

// ============================================================
//  실시간 검증: 아무 키나 입력하면 종료
// ============================================================
void verifyLoop() {
  Serial.println(F("실시간 측정 중... (엔터를 누르면 메뉴로)"));
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {
    float x = measureADC();
    Serial.print(F("ADC: "));
    Serial.print(x, 1);
    Serial.print(F("\tdistance(mm): "));
    Serial.println(predict(x), 1);
    delay(200);
  }
  readLine();
}

// ============================================================
void chooseSettings() {
  Serial.println();
  Serial.println(F("곡선맞춤 방식을 고르세요."));
  Serial.println(F("  1: 다항식        d = f(x)"));
  Serial.println(F("  2: 역수 다항식   d = f(1000/x)  (적외선 센서에 잘 맞음)"));
  method = readIntInRange(F("방식 > "), 1, 2);
  int maxDeg = min(MAX_DEG, N_POINTS - 1);
  Serial.print(F("차수를 입력하세요 (1~"));
  Serial.print(maxDeg);
  Serial.println(F(")"));
  degree = readIntInRange(F("차수 > "), 1, maxDeg);
}

void fitAndPrint() {
  if (fitCurve()) printResult();
  else Serial.println(F("곡선맞춤 실패: 측정값을 확인하고 다시 측정하세요."));
}

void setup() {
  Serial.begin(1000000);   // 수업 예제와 같은 속도 (1,000,000 bps)
  while (!Serial) {}
  Serial.println(F("적외선 거리센서 자동 곡선맞춤 (20C10)"));
  Serial.println(F("시리얼 모니터 줄 끝 설정을 'Newline'으로 해 주세요."));
  chooseSettings();
  measureAll();
  fitAndPrint();
}

void loop() {
  Serial.println();
  Serial.println(F("메뉴: 1) 다른 방식/차수로 다시 맞추기  2) 다시 측정  3) 실시간 검증"));
  int sel = readIntInRange(F("선택 > "), 1, 3);
  if (sel == 1) { chooseSettings(); fitAndPrint(); }
  else if (sel == 2) { measureAll(); fitAndPrint(); }
  else verifyLoop();
}
