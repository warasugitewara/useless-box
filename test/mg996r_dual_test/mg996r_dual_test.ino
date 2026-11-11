// ===================================================
//  MG996R ×2 同時運用テスト（Arduino UNO）
//
//  ---- 配線 ----
//    フタ MG996R 信号(橙) → D5
//    腕   MG996R 信号(橙) → D6
//    両方の 赤(+) → 電源B（単3×4本）の＋
//    両方の 茶(GND) → 電源B の−
//    電源B の− ↔ UNO の GND   ★必ず共通にする
//    ※サーボの＋を UNO の 5V に繋がないこと
//
//  ---- 動作 ----
//  起動後、シリアルモニタ（9600bps・改行あり）から操作：
//    1 : フタだけ 開→閉
//    2 : 腕だけ 伸→戻
//    3 : 本番と同じ順番（フタ開→腕伸→腕戻→フタ閉）
//    4 : 2個同時に動かす（電池の踏ん張りテスト）
//    r : 連続10回 3 を繰り返す（電池の持ち・発熱の確認）
//    d : 両方脱力（detach）
//
//  ※角度はまず箱に付けない状態で試し、取り付け後に調整する。
// ===================================================
#include <Servo.h>

Servo lid, arm;
const int lidPin = 5, armPin = 6;

// ---- 角度（取り付け後に調整）----
const int LID_CLOSED = 90, LID_OPEN = 3;
const int ARM_IN     = 3,  ARM_OUT  = 110;

// MG996R: 約0.17秒/60° → 90°で約255ms、110°で約311ms。余裕込み
const int LID_MS = 300;
const int ARM_MS = 420;

void attachBoth() {
  if (!lid.attached()) lid.attach(lidPin);
  if (!arm.attached()) arm.attach(armPin);
}

void lidTest() {
  Serial.println(F("フタ: 開→閉"));
  attachBoth();
  lid.write(LID_OPEN);   delay(LID_MS + 300);
  lid.write(LID_CLOSED); delay(LID_MS + 300);
}

void armTest() {
  Serial.println(F("腕: 伸→戻"));
  attachBoth();
  arm.write(ARM_OUT); delay(ARM_MS + 300);
  arm.write(ARM_IN);  delay(ARM_MS + 300);
}

void sequence() {
  attachBoth();
  lid.write(LID_OPEN);   delay(LID_MS);
  arm.write(ARM_OUT);    delay(ARM_MS);
  arm.write(ARM_IN);     delay(ARM_MS);
  lid.write(LID_CLOSED); delay(LID_MS);
}

void simultaneous() {
  Serial.println(F("2個同時: 開+伸 → 戻+閉"));
  attachBoth();
  lid.write(LID_OPEN);   arm.write(ARM_OUT); delay(ARM_MS + 300);
  arm.write(ARM_IN);     lid.write(LID_CLOSED); delay(ARM_MS + 300);
}

void setup() {
  Serial.begin(9600);
  lid.attach(lidPin);  lid.write(LID_CLOSED);
  arm.attach(armPin);  arm.write(ARM_IN);
  delay(1000);
  Serial.println(F("=== MG996R x2 テスト ==="));
  Serial.println(F("1:フタ 2:腕 3:本番順 4:同時 r:10回連続 d:脱力"));
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r') return;

  if      (c == '1') lidTest();
  else if (c == '2') armTest();
  else if (c == '3') { Serial.println(F("本番順")); sequence(); }
  else if (c == '4') simultaneous();
  else if (c == 'r') {
    Serial.println(F("10回連続"));
    for (int i = 1; i <= 10; i++) {
      Serial.print(F("  ")); Serial.println(i);
      sequence(); delay(500);
    }
  }
  else if (c == 'd') { lid.detach(); arm.detach(); Serial.println(F("脱力")); }
  else Serial.println(F("不明なコマンド"));
  Serial.println(F("OK"));
}
