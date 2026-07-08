// ===================================================
//  意味のない箱 (Useless Box) — バリエーション5種 [改善版]
//
//  Created : 2026-06-25
//  Author  : Claude Code (AI agent)
//  Base    : useless_box.ino をレビューし改善したもの
//
//  Hardware : Arduino UNO 3
//             Servo lid (フタ) → D5  SG90
//             Servo arm (腕)   → D6  MG996R
//             Switch            → D12 (INPUT_PULLUP / 後述の事情で論理反転)
//
//  ---- スイッチ論理についての注意 ----
//  スイッチが物理的に逆向きに取り付けられているため、
//  「スイッチOFF位置 = HIGH / ON位置 = LOW」という反転した論理になっている。
//  本来 INPUT_PULLUP は「押下=LOW」が標準だが、本機では HIGH をトリガーとして
//  採用せざるを得なかった（再はんだ付けでの付け直しを避けるための妥協）。
//  ※配線を直せる場合は、スイッチを正しい向きに付け直したうえで
//    判定を == LOW に戻すのが本来あるべき形。
//
//  ---- 改善点（オリジナルからの差分） ----
//   1. 開閉・ノック動作をヘルパー関数化（openLid / closeLid / poke）して重複を排除
//   2. Case3「即ブチ切れ」をオリジナル（Case1と同一だった）から差別化し、
//      間を一切置かない演出上の最速動作に変更
//   3. アイドル時にサーボを detach（保持トルクのうなり・消費電流・ジッタを抑制）
//      ※フタ/腕が自重でずれる構造の場合は IDLE_DETACH を 0 にすること
//   4. sweep の角度マジックナンバーを名前付き定数化（lidPeek / lidFeint1 / lidFeint2）
//   5. 同じバリエーションの連続を回避（前回値を覚えて再抽選）
//   6. 動作開始前にも簡易デバウンスを追加（振動などによる誤発火を防止）
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
//  1: 静音・省電力（推奨）
//  0: 常時保持トルク（フタ/腕が自重でずれる構造の場合はこちら）
#define IDLE_DETACH 1

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

  // 初期姿勢へ移動（MG996R が戻りきってから SG90 を閉じる順序を守る）
  lidServo.attach(lidPin);
  armServo.attach(armPin);
  armServo.write(armRetracted);
  delay(ARM_MOVE_MS);
  lidServo.write(lidClosed);
  delay(LID_MOVE_MS);

  disableServos();  // 初期化完了後はアイドル（IDLE_DETACH=1 のとき切り離し）
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
  delay(100);       // 動作後のチャタリング防止
}
