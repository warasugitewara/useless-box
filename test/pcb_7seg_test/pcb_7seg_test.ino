// ===================================================
//  7seg基板 単体検証スケッチ（Arduino UNO）
//
//  ---- 配線（5本）----
//    UNO D2  → 右IC(595#1) 14番 SER
//    UNO D3  → 11番 SRCLK（2個共通）
//    UNO D4  → 12番 RCLK （2個共通）
//    UNO 5V  → 基板 5V
//    UNO GND → 基板 GND
//
//  ---- 動作 ----
//  起動時: 全点灯(2秒) → セグメント1本ずつ点灯 → 00〜99カウント を自動で1周。
//  その後はシリアルモニタ（9600bps・改行あり）から操作できる。
//    0〜99 の数字 : その数を表示
//    a           : 全点灯（全セグ＋DP）
//    s           : セグメントを a→b→…→g→dp の順に1本ずつ点灯
//    c           : 00〜99 カウント
//    x           : 右桁の d/f 補正 ON/OFF 切り替え
//    o           : 全消灯
// ===================================================

const int dataPin  = 2;  // SER   (595#1 14番)
const int clockPin = 3;  // SRCLK (11番)
const int latchPin = 4;  // RCLK  (12番)

// 基板では右桁（595#1 → DIG2）の d と f が入れ替わって配線されている。
//  true : 補正する（基板用）  false : 補正しない（ブレッドボード用）
bool dfSwap = true;

// bit0=a, bit1=b, bit2=c, bit3=d, bit4=e, bit5=f, bit6=g, bit7=dp
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
const char* SEG_NAME[8] = {"a", "b", "c", "d", "e", "f", "g", "dp"};

// 右桁のパターンを基板の配線に合わせて並べ替える（bit3=d と bit5=f を交換）
byte remapRight(byte seg) {
  if (!dfSwap) return seg;
  byte d = (seg >> 3) & 1;
  byte f = (seg >> 5) & 1;
  seg &= ~((1 << 3) | (1 << 5));
  seg |= (d << 5) | (f << 3);
  return seg;
}

// 左桁(十の位)→右桁(一の位)の順に送る。アノードコモンなので ~ で反転
void writeDigits(byte left, byte right) {
  right = remapRight(right);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, ~left);   // 先に送る → 595#2（左）
  shiftOut(dataPin, clockPin, MSBFIRST, ~right);  // 後に送る → 595#1（右）
  digitalWrite(latchPin, HIGH);
}

void showNumber(int n) {
  n = constrain(n, 0, 99);
  writeDigits(digits[n / 10], digits[n % 10]);
}

void allOn()  { writeDigits(0xFF, 0xFF); Serial.println(F("全点灯")); }
void allOff() { writeDigits(0x00, 0x00); Serial.println(F("全消灯")); }

// 両桁で同じセグメントを1本ずつ点灯。シリアルの名前と光った場所が一致するか見る
void segmentTest() {
  Serial.println(F("セグメント順次点灯（両桁同時）"));
  for (int i = 0; i < 8; i++) {
    byte s = 1 << i;
    writeDigits(s, s);
    Serial.print(F("  点灯中: "));
    Serial.println(SEG_NAME[i]);
    delay(800);
  }
  allOff();
}

void countTest() {
  Serial.println(F("00〜99 カウント"));
  for (int n = 0; n <= 99; n++) {
    showNumber(n);
    delay(150);
  }
}

void setup() {
  pinMode(dataPin,  OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  Serial.begin(9600);
  Serial.println(F("=== 7seg基板 検証 ==="));
  Serial.print(F("d/f補正: "));
  Serial.println(dfSwap ? F("ON（基板用）") : F("OFF"));

  allOn();
  delay(2000);
  segmentTest();
  countTest();
  showNumber(88);
  Serial.println(F("コマンド: 0-99 / a / s / c / x / o"));
}

void loop() {
  if (!Serial.available()) return;
  String in = Serial.readStringUntil('\n');
  in.trim();
  if (in.length() == 0) return;

  char c = in.charAt(0);
  if (isDigit(c)) {
    int n = in.toInt();
    showNumber(n);
    Serial.print(F("表示: "));
    if (n < 10) Serial.print('0');
    Serial.println(constrain(n, 0, 99));
  } else if (c == 'a') allOn();
  else if (c == 's') segmentTest();
  else if (c == 'c') countTest();
  else if (c == 'o') allOff();
  else if (c == 'x') {
    dfSwap = !dfSwap;
    Serial.print(F("d/f補正: "));
    Serial.println(dfSwap ? F("ON") : F("OFF"));
    showNumber(25);  // 右桁に d/f の差が出る数字（5）で確認
    Serial.println(F("  → 25 を表示。右桁が 5 に見えれば正しい"));
  } else {
    Serial.println(F("不明なコマンド"));
  }
}
