// ===================================================
//  test/count.ino — 7セグ2桁 動作確認用スケッチ
//
//  74HC595 ×2（カスケード）+ A-552SRD（2桁・COMMON ANODE）で
//  00〜99 を1秒ごとにカウントアップするだけの単体テスト。
//  本体スケッチ（useless_box.ino）とは独立して動かすこと。
//
//  ---- 595 側の固定配線（コード外・忘れやすい）----
//    OE   (13) → GND   出力を常時有効化
//    SRCLR(10) → 5V    クリアを無効化
//    VCC  (16) → 5V  ／ GND (8) → GND
//    IC1 の QH'(9) → IC2 の DS(14)      ★カスケード線
//    SH_CP / ST_CP は IC1・IC2 で共通接続
//    各 QA〜QH → 470Ω → 7セグの a,b,c,d,e,f,g,dp
//    7セグ コモン（14pin=DIG.1 / 13pin=DIG.2）→ 5V
// ===================================================

// 74HC595とArduinoの接続ピン設定
const int latchPin = 8;  // ST_CP (12ピン)
const int clockPin = 12; // SH_CP (11ピン)
const int dataPin  = 11; // DS    (14ピン)

// 0〜9の点灯パターン (Q0=A, Q1=B... と配線している前提)
//   bit0=a, bit1=b, bit2=c, bit3=d, bit4=e, bit5=f, bit6=g, bit7=dp
//   MSBFIRST で送ると bit0 が最後に出て QA に残るため、この並びで整合する
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

void setup() {
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin,  OUTPUT);
}

void loop() {
  // 00から99までカウントアップ
  for (int i = 0; i < 100; i++) {
    int tens = i / 10; // 十の位の数値 (左側・IC2用)
    int ones = i % 10; // 一の位の数値 (右側・IC1用)

    // 送信準備
    digitalWrite(latchPin, LOW);

    // 【1回目の送信】
    // 先に送ったデータは、次の送信によって奥のIC(IC2: 十の位)に押し出されます。
    // アノードコモンなので「~」でビット反転させます。
    shiftOut(dataPin, clockPin, MSBFIRST, ~digits[tens]);

    // 【2回目の送信】
    // 後から送ったデータは、手前のIC(IC1: 一の位)に留まります。
    shiftOut(dataPin, clockPin, MSBFIRST, ~digits[ones]);

    // データをラッチして2桁同時に出力へ反映
    digitalWrite(latchPin, HIGH);

    delay(1000); // 1秒待機
  }
}
