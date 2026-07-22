// ===================================================
//  意味のない箱 (Useless Box) — バリエーション5種
//
//  Hardware : Arduino UNO 3
//             Servo lid (フタ) → D5  SG90
//             Servo arm (腕)   → D6  MG996R
//             Switch            → D12 (INPUT_PULLUP / 論理反転・下記参照)
//             7seg 2桁          → D2/D3/D4 経由で 74HC595 ×2（カスケード）
//
//  ---- ピン割り当ての方針 ----
//  既存の駆動系（D5/D6/D12/A0）を最優先で維持し、後付けの7セグを空きピンへ寄せた。
//  D10/D11 は DFPlayer Mini 用に予約（後で再配線しないため空けてある）。
//  D7〜D9・D13 は将来の拡張（キャタピラー駆動など）用に残している。
//
//  ---- スイッチ論理 ----
//  スイッチが物理的に逆向きに取り付けられているため
//  「OFF位置 = HIGH / ON位置 = LOW」の反転論理。HIGH をトリガーとする。
//  配線を付け直した場合は判定を == LOW に戻すこと。
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

// --- 演出用の中間角度（旧コードの直書きを定数化）---
const int lidPeek   = 48;  // Case2: ためらい時の半開き角度
const int lidFeint1 = 58;  // Case4: フェイント1のチラ見せ（90→58 で 32°分開く）
const int lidFeint2 = 35;  // Case4: フェイント2（90→35 で 55°分開く）

// --- 動作完了を保証する最低待機時間 ---
const int LID_MOVE_MS = 200;   // SG90 が 90° 動ききるのに必要（4.5V基準）
const int LID_FAST_MS = 140;   // SG90 の実測の速い側（Case3「最速」演出用）
const int ARM_MOVE_MS = 420;   // MG996R が 110° 動ききるのに必要（重いので余裕込み）

// --- スイッチのデバウンス時間 ---
const int DEBOUNCE_MS = 20;

// --- アイドル時にサーボを切り離すか ---
//  1: 脱力させて無駄な発熱・電力消費を抑える（本機の設計思想）
//     ※フタは縁に引っかかる構造のため脱力しても位置がずれない。
//       Servoライブラリはattach時に前回write値でパルスを再開するためジャンプも起きない。
//  0: 常時保持トルク（自重でずれる構造に変更した場合のみ）
#define IDLE_DETACH 1

// ============================================================
//  7セグメント表示（A-552SRD 2桁 / 74HC595 ×2 カスケード）
//
//  ---- 駆動方式: スタティック点灯 ----
//  595 を1個ずつ各桁に固定割り当てし、全セグメントを常時直接駆動する。
//  ダイナミック点灯（1個で両桁を高速切替）は採用しない——本スケッチは
//  delay() ブロッキング設計であり、動作中にリフレッシュを回せないため。
//  ラッチした表示は次に書き換えるまで保持されるので、この方式なら
//  delay(650) 等で止まっている間も表示は消えない。
//
//  ---- ビット順とテーブルについて ----
//  test/count.ino と同一の規約を採用している（配線の互換性を保つため）。
//  595 は「最初に送ったビットが QH まで押し出される」ので、MSBFIRST では
//  bit0 が最後に出て QA に残る → bit0=a, bit1=b, … bit7=dp と対応する。
//
//  ---- 595 側の固定配線（コード外・忘れやすい）----
//    OE   (13) → GND   出力を常時有効化
//    SRCLR(10) → 5V    クリアを無効化
//    VCC  (16) → 5V  ／ GND (8) → GND
//    IC1 の QH'(9) → IC2 の DS(14)      ★カスケード線
//    SH_CP / ST_CP は IC1・IC2 で共通接続
//    各 QA〜QH → 470Ω → 7セグの a,b,c,d,e,f,g,dp
//    7セグ コモン（14pin=DIG.1 / 13pin=DIG.2）→ 5V
//
//  ---- アノードコモンについて ----
//  A-552SRD は COMMON ANODE。コモン側が＋なので、595 の出力が LOW の
//  ときにセグメントへ電流が流れ込んで点灯する。よって送出時に ~ で反転する。
//
//  ---- 電流について ----
//  74HC595 は 1ピン±35mA・チップ合計±70mA。桁ごとに 595 を分けているので
//  1チップが受け持つのは最大7セグメント。470Ω・4.5V で 1セグ約5mA、
//  "8" 表示でも 1チップ約35mA に収まる。暗く感じる場合は 330Ω まで
//  下げてよい（約7mA/セグ、1チップ約50mA でまだ余裕がある）。
// ============================================================
const int dataPin  = 2;  // DS    (595 14ピン)
const int clockPin = 3;  // SH_CP (595 11ピン) — IC1/IC2 共通
const int latchPin = 4;  // ST_CP (595 12ピン) — IC1/IC2 共通

// 0〜9の点灯パターン (Q0=A, Q1=B... と配線している前提)
//   bit0=a, bit1=b, bit2=c, bit3=d, bit4=e, bit5=f, bit6=g, bit7=dp
const byte digits[10] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111  // 9
};
const byte SEG_BLANK = 0b00000000;  // 全消灯

// --- 動作完了後に押下回数（00〜99でラップ）を表示するか ---
//  1: 動作中は case 番号、待機中は通算の押下回数を表示
//  0: case 番号を出したまま次の起動まで保持
#define SHOW_PRESS_COUNT 1

// ------------------------------------------------------------
//  両桁へ一括送出
//  カスケードでは「先に送ったバイトが奥のIC2へ押し出され、
//  後に送ったバイトが手前のIC1に留まる」ため、左桁→右桁の順で送る。
//  左右が入れ替わって表示されたら、この2行を交換すればよい。
// ------------------------------------------------------------
void writeDigits(byte segLeft, byte segRight) {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, ~segLeft);   // 先に送る → IC2（十の位・左）
  shiftOut(dataPin, clockPin, MSBFIRST, ~segRight);  // 後に送る → IC1（一の位・右）
  digitalWrite(latchPin, HIGH);                      // ラッチして2桁同時に反映
}

// 0〜99 を表示。leadingZero=false なら十の位が 0 のとき左桁を消灯する
void showNumber(int n, bool leadingZero) {
  n = constrain(n, 0, 99);
  byte tens = n / 10;
  byte ones = n % 10;
  byte left = (tens == 0 && !leadingZero) ? SEG_BLANK : digits[tens];
  writeDigits(left, digits[ones]);
}

void clearDisplay() { writeDigits(SEG_BLANK, SEG_BLANK); }

void setupDisplay() {
  pinMode(dataPin,  OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  clearDisplay();
}

// ============================================================
//  サーボの有効化／無効化
//  IDLE_DETACH=1 のときだけ attach/detach を行う。
//  =0 のときは setup で attach したまま常時保持する。
// ============================================================
void enableServos() {
#if IDLE_DETACH
  lidServo.attach(lidPin);
  armServo.attach(armPin);
#endif
}
void disableServos() {
#if IDLE_DETACH
  lidServo.detach();
  armServo.detach();
#endif
}

// ============================================================
//  sweep() — SG90(lid) 専用のゆっくり動作ヘルパー
//  1° ずつ動かすことで「ためらい」や「フェイント」を表現
//
//  stepDelay の目安 [ms/度]:
//    6  → SG90 速め  (90° で約 540ms)
//   10  → SG90 普通  (90° で約 900ms)
//   15  → SG90 ゆっくり (90° で約 1350ms)
//
//  ※ MG996R(arm) はトルクが大きく sweep 中の停止が難しいため write() 直接を推奨
// ============================================================
void sweep(Servo &s, int from, int to, int stepDelay) {
  int step = (from < to) ? 1 : -1;
  for (int pos = from; pos != to + step; pos += step) {
    s.write(pos);
    delay(stepDelay);
  }
}

// ============================================================
//  動作ヘルパー（重複排除）
// ============================================================
void openLid(int waitMs = LID_MOVE_MS)  { lidServo.write(lidOpen);   delay(waitMs); }
void closeLid(int waitMs = LID_MOVE_MS) { lidServo.write(lidClosed); delay(waitMs); }

// 腕を1回フルストロークさせる（伸ばす→戻す）
void poke() {
  armServo.write(armExtended);  delay(ARM_MOVE_MS);
  armServo.write(armRetracted); delay(ARM_MOVE_MS);
}

// ============================================================
//  setup
// ============================================================
void setup() {
  randomSeed(analogRead(A0));  // 未接続ピンの浮遊ノイズをシードに使用（A0は何も繋がないこと）

  pinMode(switchPin, INPUT_PULLUP);  // スイッチ逆付けのため HIGH をトリガーに採用（冒頭コメント参照）

  setupDisplay();  // 7セグを消灯状態で初期化

  // 初期姿勢へ移動（MG996R が戻りきってから SG90 を閉じる順序を守る）
  lidServo.attach(lidPin);
  armServo.attach(armPin);
  armServo.write(armRetracted);
  delay(ARM_MOVE_MS);
  lidServo.write(lidClosed);
  delay(LID_MOVE_MS);

  disableServos();  // 初期化完了後はアイドル（IDLE_DETACH=1 のとき切り離し）

#if SHOW_PRESS_COUNT
  showNumber(0, true);  // 起動時は "00"
#endif
}

// ============================================================
//  loop
// ============================================================
void loop() {
  // --- トリガー判定 + 簡易デバウンス ---
  if (digitalRead(switchPin) != HIGH) return;
  delay(DEBOUNCE_MS);
  if (digitalRead(switchPin) != HIGH) return;  // 振動などによる誤検出を除去

  enableServos();

  // --- 同じバリエーションの連続を避けて抽選（1〜5）---
  static int prev = 0;
  int x;
  do { x = random(1, 6); } while (x == prev);
  prev = x;

  showNumber(x, false);  // 動作中はどの case を演じているかを表示（左桁は消灯）

  switch (x) {

    // --------------------------------------------------
    //  Case 1 : ノーマル
    //  迷わずスパッと処理する "仕事人" タイプ
    // --------------------------------------------------
    case 1:
      openLid();
      poke();
      closeLid();
      break;

    // --------------------------------------------------
    //  Case 2 : ためらいがち
    //  「躊躇→引き戻し→諦め」を表現する "しぶしぶ" タイプ
    // --------------------------------------------------
    case 2:
      sweep(lidServo, lidClosed, lidPeek, 12);   // ゆっくり半開き
      delay(650);                                 // 躊躇して止まる
      sweep(lidServo, lidPeek, lidClosed, 8);    // 引き戻そうとする
      delay(450);
      sweep(lidServo, lidClosed, lidOpen, 10);   // 結局諦めて全開
      delay(150);

      poke();                                     // 腕を出して戻す

      sweep(lidServo, lidOpen, lidClosed, 9);    // ゆっくり閉じる
      delay(100);
      break;

    // --------------------------------------------------
    //  Case 3 : 即ブチ切れ
    //  間を一切置かず、SG90 を速い側で動かす "演出上の最速" タイプ
    //  （MG996R の 420ms は物理限界のため省けない）
    // --------------------------------------------------
    case 3:
      openLid(LID_FAST_MS);   // フタを最速で開ける
      poke();                 // 即出撃・即帰還
      closeLid(LID_FAST_MS);  // フタを最速で閉じる
      break;

    // --------------------------------------------------
    //  Case 4 : フェイント2連
    //  2回チラ見せしてから本番動作に移る "煽り" タイプ
    // --------------------------------------------------
    case 4:
      sweep(lidServo, lidClosed, lidFeint1, 6);  // フェイント1: 32°チラ見せ
      delay(150);
      sweep(lidServo, lidFeint1, lidClosed, 6);
      delay(280);

      sweep(lidServo, lidClosed, lidFeint2, 6);  // フェイント2: 55°まで開く
      delay(120);
      sweep(lidServo, lidFeint2, lidClosed, 6);
      delay(220);

      openLid();   // 本番: 一気に全開
      poke();
      closeLid();
      break;

    // --------------------------------------------------
    //  Case 5 : 連続ノック（怒り表現）
    //  腕を2回フルストロークさせる "激おこ" タイプ
    // --------------------------------------------------
    case 5:
      openLid();
      poke();      // 1打目
      poke();      // 2打目
      closeLid();
      break;
  }

  disableServos();  // アイドルへ戻す

#if SHOW_PRESS_COUNT
  // 待機中は通算の押下回数を表示（00〜99でラップ）
  static unsigned int pressCount = 0;
  pressCount++;
  showNumber(pressCount % 100, true);
#endif

  delay(100);       // 動作後のチャタリング防止
}
