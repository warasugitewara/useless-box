// ===================================================
//  意味のない箱 (Useless Box) — バリエーション5種
//  Hardware : Arduino UNO 3
//             Servo lid (フタ) → D5  SG90
//             Servo arm (腕)   → D6  MG996R
//             Switch            → D12 (INPUT_PULLUP, ONでHIGH)
//
//  ---- サーボ速度メモ ----
//  SG90   (lid) : 60° / 100ms = 1.67ms/度
//                 90° 動作 → 最低 150ms、余裕込みで LID_MOVE_MS = 200ms
//  MG996R (arm) : 60° / 170ms = 2.83ms/度
//                 110° 動作 → 最低 311ms、余裕込みで ARM_MOVE_MS = 420ms
// ===================================================

#include <Servo.h>

Servo lidServo;   // SG90  : フタ
Servo armServo;   // MG996R: 腕

const int lidPin    = 5;
const int armPin    = 6;
const int switchPin = 12;

// --- 角度設定 ---
// SG90 の取り付け方向に合わせて 90° が閉じ、3° が開き
const int lidClosed    = 90;   // フタが閉まる角度
const int lidOpen      = 3;    // フタが開く角度（壁ドン防止ストッパー）
const int armRetracted = 3;    // 腕が格納された角度（壁ドン防止）
const int armExtended  = 110;  // 腕が伸びた角度

// --- 動作完了を保証する最低待機時間 ---
// SG90 が 90° 動ききるのに必要
const int LID_MOVE_MS = 200;
// MG996R が 110° 動ききるのに必要（重いので余裕を持たせる）
const int ARM_MOVE_MS = 420;

// ============================================================
//  sweep() — SG90(lid) 専用のゆっくり動作ヘルパー
//  1° ずつ動かすことで「ためらい」や「フェイント」を表現
//
//  stepDelay の目安 [ms/度]:
//    6  → SG90 速め  (90° で約 540ms)
//   10  → SG90 普通  (90° で約 900ms)
//   15  → SG90 ゆっくり (90° で約 1350ms)
//
//  ※ MG996R(arm) はトルクが大きく sweep 中の停止が難しいため
//    write() 直接を推奨
// ============================================================
void sweep(Servo &s, int from, int to, int stepDelay) {
  int step = (from < to) ? 1 : -1;
  for (int pos = from; pos != to + step; pos += step) {
    s.write(pos);
    delay(stepDelay);
  }
}

// ============================================================
//  setup
// ============================================================
void setup() {
  randomSeed(analogRead(A0));  // 未接続ピンの浮遊ノイズをシードに使用（A0は何も繋がないこと）

  pinMode(switchPin, INPUT_PULLUP);

  lidServo.attach(lidPin);
  armServo.attach(armPin);

  // MG996R が戻りきってから SG90 を閉じる（初期化順序を守る）
  armServo.write(armRetracted);
  delay(ARM_MOVE_MS);
  lidServo.write(lidClosed);
  delay(LID_MOVE_MS);
}

// ============================================================
//  loop
// ============================================================
void loop() {
  if (digitalRead(switchPin) == HIGH) {

    int x = random(1, 6);  // 1〜5 の乱数

    switch (x) {

      // --------------------------------------------------
      //  Case 1 : ノーマル
      //  迷わずスパッと処理する "仕事人" タイプ
      // --------------------------------------------------
      case 1:
        lidServo.write(lidOpen);      // SG90: 全開
        delay(LID_MOVE_MS);

        armServo.write(armExtended);  // MG996R: 伸ばす
        delay(ARM_MOVE_MS);

        armServo.write(armRetracted); // MG996R: 戻る
        delay(ARM_MOVE_MS);

        lidServo.write(lidClosed);    // SG90: 閉じる
        delay(LID_MOVE_MS);
        break;

      // --------------------------------------------------
      //  Case 2 : ためらいがち
      //  SG90 の応答速度を活かした細かい動きで
      //  「躊躇→引き戻し→諦め」を表現する "しぶしぶ" タイプ
      // --------------------------------------------------
      case 2:
        sweep(lidServo, lidClosed, 48, 12);      // SG90: ゆっくり半開き
        delay(650);                               // 躊躇して止まる

        sweep(lidServo, 48, lidClosed, 8);       // SG90: 引き戻そうとする
        delay(450);

        sweep(lidServo, lidClosed, lidOpen, 10); // SG90: 結局諦めて全開
        delay(150);

        armServo.write(armExtended);             // MG996R
        delay(ARM_MOVE_MS);

        armServo.write(armRetracted);            // MG996R
        delay(ARM_MOVE_MS);

        sweep(lidServo, lidOpen, lidClosed, 9);  // SG90: ゆっくり閉じる
        delay(100);
        break;

      // --------------------------------------------------
      //  Case 3 : 即ブチ切れ
      //  SG90 は超高速で開くが、MG996R は物理限界があるため
      //  ARM_MOVE_MS を守った「演出上の最速」タイプ
      // --------------------------------------------------
      case 3:
        lidServo.write(lidOpen);      // SG90: 即全開（200ms で完了）
        delay(LID_MOVE_MS);

        armServo.write(armExtended);  // MG996R: 物理的に必要な 420ms は省けない
        delay(ARM_MOVE_MS);

        armServo.write(armRetracted); // MG996R
        delay(ARM_MOVE_MS);

        lidServo.write(lidClosed);    // SG90
        delay(LID_MOVE_MS);
        break;

      // --------------------------------------------------
      //  Case 4 : フェイント2連
      //  SG90 の速さを活かした素早いフェイント2回から
      //  MG996R の本番動作に移る "煽り" タイプ
      // --------------------------------------------------
      case 4:
        // フェイント1: SG90 が 32° チラ見せ（90→58: 32°分開く）
        sweep(lidServo, lidClosed, 58, 6);
        delay(150);
        sweep(lidServo, 58, lidClosed, 6);
        delay(280);

        // フェイント2: SG90 が 55° まで開く（90→35: 55°分開く）
        sweep(lidServo, lidClosed, 35, 6);
        delay(120);
        sweep(lidServo, 35, lidClosed, 6);
        delay(220);

        // 本番: SG90 一気に全開 → MG996R 出撃
        lidServo.write(lidOpen);
        delay(LID_MOVE_MS);

        armServo.write(armExtended);  // MG996R
        delay(ARM_MOVE_MS);

        armServo.write(armRetracted); // MG996R
        delay(ARM_MOVE_MS);

        lidServo.write(lidClosed);
        delay(LID_MOVE_MS);
        break;

      // --------------------------------------------------
      //  Case 5 : 連続ノック（怒り表現）
      //  MG996R を2回フルストロークさせる "激おこ" タイプ
      //  重量ある金属ギアの2打が重厚な怒りを演出
      // --------------------------------------------------
      case 5:
        lidServo.write(lidOpen);      // SG90
        delay(LID_MOVE_MS);

        armServo.write(armExtended);  // MG996R: 1打目
        delay(ARM_MOVE_MS);
        armServo.write(armRetracted);
        delay(ARM_MOVE_MS);

        armServo.write(armExtended);  // MG996R: 2打目
        delay(ARM_MOVE_MS);
        armServo.write(armRetracted);
        delay(ARM_MOVE_MS);

        lidServo.write(lidClosed);    // SG90
        delay(LID_MOVE_MS);
        break;
    }

    // チャタリング防止
    delay(100);
  }
}
