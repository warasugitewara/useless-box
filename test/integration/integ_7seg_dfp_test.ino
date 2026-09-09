// ===================================================
//  integ_7seg_dfp_test.ino
//  7セグ(74HC595×2) + DFPlayer Mini 統合テスト
//
//  目的: 本体組み込み前に、7セグ表示とDFP再生が
//        同じスケッチ内で干渉なく同居できるかを確認する。
//        番号を指定すると「7セグ表示 + 対応mp3再生」を同時に行う。
//
//  ---- ハードウェア / 対応ボード ----
//    Arduino UNO R3 / Nano 互換（どちらも同じピン番号で動く）
//
//  ---- ピン割り当て（本体 useless_box.ino と同一）----
//    7seg  : D2=DS / D3=SH_CP / D4=ST_CP  → 74HC595 ×2（カスケード）
//    DFP   : D10=DFのTX / D11=DFのRX(1kΩ直列)
//    SW1-5 : D7,D8,D9,D12,D13（INPUT_PULLUP・押すとLOW）
//            もう片側はGNDへ共通接続。抵抗不要。サーボ用D5/D6は避けている。
//            タクトスイッチは対角の2足を使うこと。
//
//  ---- 595 側の固定配線（コード外・両ICに必要）----
//    16(VCC)→5V / 8(GND)→GND / 13(OE)→GND / 10(SRCLR)→5V
//    #1 の 9(QH') → #2 の 14(DS)   ★カスケード
//    各 QA〜QG →470Ω→ 7セグ a〜g / コモン(14,13)→5V（アノードコモン）
//
//  ---- DFP / SD ----
//    /mp3/0001.mp3 〜 0005.mp3（4桁ゼロ埋め）。playMp3Folder(n) で再生。
//    チップ TD5580A 対策で begin 失敗でも停止しない・ACK無効。
//
//  ---- 操作（シリアル 9600 baud）----
//    1〜5 : その番号を 7セグ表示 ＋ 000n.mp3 を再生
//    r    : 1〜5 をランダム発火（直前と同じ番号は避ける）
//    0    : 表示クリア＋停止
//    +/-  : 音量調整
//    ?    : 状態（ファイル数・音量）
//    物理スイッチ SW1〜SW5: 押した瞬間に対応番号を発火（シリアルと併用）
// ===================================================

#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

// ---- 7セグ（74HC595）----
const int dataPin  = 2;  // DS
const int clockPin = 3;  // SH_CP
const int latchPin = 4;  // ST_CP

// bit0=a, bit1=b, ... bit6=g, bit7=dp（MSBFIRSTで送るとbit0がQAに残る）
const byte digits[10] = {
  0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110,
  0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111
};
const byte SEG_BLANK = 0b00000000;

// ---- DFPlayer ----
SoftwareSerial dfSerial(10, 11);  // RX=D10(DFのTXへ), TX=D11(DFのRXへ)
DFRobotDFPlayerMini dfPlayer;
int vol = 22;

// ---- 物理スイッチ（1〜5に対応・押した瞬間に発火）----
const int swPins[5] = {7, 8, 9, 12, 13};  // SW1..SW5
bool swPrev[5];                            // 直前の読み値（HIGH=非押下）
unsigned long swLastMs[5];                 // デバウンス用の最終変化時刻
const unsigned long SW_DEBOUNCE = 25;      // ms

// ============================================================
//  7セグ表示
//  カスケード: 先に送ったバイトが奥のIC2(左桁)へ押し出される
// ============================================================
void writeDigits(byte segLeft, byte segRight) {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, ~segLeft);   // → IC2（左桁）
  shiftOut(dataPin, clockPin, MSBFIRST, ~segRight);  // → IC1（右桁）
  digitalWrite(latchPin, HIGH);
}
// 2桁ゼロ埋め表示（1→"01", 5→"05"）
void showDigit(int n) {
  if (n < 0 || n > 99) { writeDigits(SEG_BLANK, SEG_BLANK); return; }
  writeDigits(digits[n / 10], digits[n % 10]);  // 左=十の位, 右=一の位
}
void clearDisplay() { writeDigits(SEG_BLANK, SEG_BLANK); }

// ============================================================
//  指定番号を「表示＋再生」
// ============================================================
void fire(int n) {
  showDigit(n);
  dfPlayer.playMp3Folder(n);   // /mp3/000n.mp3
  Serial.print(F("[fire] no=")); Serial.println(n);
}

void setup() {
  Serial.begin(9600);
  dfSerial.begin(9600);

  pinMode(dataPin,  OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  clearDisplay();

  Serial.println(F("=== 7seg + DFPlayer 統合テスト ==="));
  // TD5580A 対策: ACK無効(false) / リセットあり(true)
  if (!dfPlayer.begin(dfSerial, false, true)) {
    Serial.println(F("[warn] begin() false（TD5580Aではよくある）。続行します。"));
  } else {
    Serial.println(F("[ok] begin() 成功。"));
  }
  delay(200);
  dfPlayer.volume(vol);

  randomSeed(analogRead(A0));  // A0未接続の浮遊ノイズをシードに

  // 物理スイッチを INPUT_PULLUP で初期化（押すとLOW）
  for (int i = 0; i < 5; i++) {
    pinMode(swPins[i], INPUT_PULLUP);
    swPrev[i]   = HIGH;
    swLastMs[i] = 0;
  }

  delay(150);
  Serial.print(F("SDファイル数 = ")); Serial.println(dfPlayer.readFileCounts());
  Serial.print(F("音量 = ")); Serial.println(vol);
  Serial.println(F("操作: 1-5=指定 / r=ランダム / 0=停止 / +,-=音量 / ?=状態"));
}

// 物理スイッチをスキャンし、HIGH→LOW の立ち下がりで発火（デバウンス付き）
void scanSwitches() {
  for (int i = 0; i < 5; i++) {
    bool now = digitalRead(swPins[i]);
    if (now != swPrev[i] && (millis() - swLastMs[i]) > SW_DEBOUNCE) {
      swLastMs[i] = millis();
      if (swPrev[i] == HIGH && now == LOW) {   // 押した瞬間だけ
        Serial.print(F("[switch] SW")); Serial.println(i + 1);
        fire(i + 1);                            // SW1→1 … SW5→5
      }
      swPrev[i] = now;
    }
  }
}

void loop() {
  scanSwitches();

  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') return;

    if (c >= '1' && c <= '5') {
      fire(c - '0');
    } else if (c == 'r') {
      static int prev = 0;
      int n;
      do { n = random(1, 6); } while (n == prev);  // 直前と同じは避ける
      prev = n;
      Serial.print(F("[random] "));
      fire(n);
    } else if (c == '0') {
      clearDisplay();
      dfPlayer.stop();
      Serial.println(F("[stop] 表示クリア＋停止"));
    } else if (c == '+') {
      vol = min(30, vol + 3); dfPlayer.volume(vol);
      Serial.print(F("音量 = ")); Serial.println(vol);
    } else if (c == '-') {
      vol = max(0, vol - 3); dfPlayer.volume(vol);
      Serial.print(F("音量 = ")); Serial.println(vol);
    } else if (c == '?') {
      Serial.print(F("ファイル数 = ")); Serial.println(dfPlayer.readFileCounts());
      Serial.print(F("音量 = ")); Serial.println(dfPlayer.readVolume());
    } else {
      Serial.print(F("未知のコマンド: ")); Serial.println(c);
    }
  }

  // DFPlayer からの通知
  if (dfPlayer.available()) {
    uint8_t type = dfPlayer.readType();
    int value = dfPlayer.read();
    if (type == DFPlayerPlayFinished) {
      Serial.print(F("[info] 再生完了: ")); Serial.println(value);
    } else if (type == DFPlayerError) {
      Serial.print(F("[err] コード: ")); Serial.println(value);
    }
  }
}
