// ===================================================
//  dfplayer_test.ino — DFPlayer Mini 単体テスト（対話式）
//
//  目的: 音出し担当の切り分け用。シリアルモニタから文字を打つと
//        対応する再生方式を試せる。配線・SD配置・音声長を分離検証する。
//
//  ---- ハードウェア ----
//    DFPlayer Mini（HW-247A / チップ: TD5580A）＋ Arduino UNO 3
//
//  ---- 配線 ----
//    DFPlayer VCC(1)   → Arduino 5V
//    DFPlayer RX(2)    → 1kΩ → Arduino D11
//    DFPlayer TX(3)    → Arduino D10
//    DFPlayer SPK_1(6) → スピーカー（8Ω 2W 推奨）
//    DFPlayer GND(7)   → Arduino GND
//    DFPlayer SPK_2(8) → スピーカー（GNDには繋がない・BTL出力）
//
//  ---- SDカード ----
//    FAT32 / 4桁ゼロ埋め / 1個ずつ順にコピー
//    0006.mp3 は切り分け用ビープ（440Hz 2秒・確実に聞こえる基準音）
//
//  ---- 使い方 ----
//    シリアルモニタを 9600 baud・改行「なし」または「LFのみ」で開き、
//    下のコマンド文字を送る。
//
//      b : ビープ(6番)を playMp3Folder で再生   ← まずこれ
//      r : 6番を play() で再生（ルート直下方式）
//      m : 6番を playMp3Folder() で再生（mp3フォルダ方式）
//      1〜5 : その番号を playMp3Folder で再生
//      +  : 音量アップ    -  : 音量ダウン
//      ? : SD状態・ファイル数を問い合わせ
// ===================================================

#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

SoftwareSerial dfSerial(10, 11);  // RX=D10(DFのTXへ), TX=D11(DFのRXへ)
DFRobotDFPlayerMini player;

int vol = 22;  // 初期音量 0..30

void setup() {
  Serial.begin(9600);      // ★モニタも 9600 に合わせる
  dfSerial.begin(9600);

  Serial.println(F("=== DFPlayer 対話テスト ==="));
  Serial.println(F("初期化中..."));

  // TD5580A 対策: ACKを待たない(false) / リセットは行う(true)
  if (!player.begin(dfSerial, false, true)) {
    Serial.println(F("[warn] begin() が false。TD5580Aではよくあるので続行します。"));
  } else {
    Serial.println(F("[ok] begin() 成功。"));
  }

  delay(300);
  player.volume(vol);
  Serial.print(F("音量 = ")); Serial.println(vol);

  // SDのファイル総数を問い合わせ（-1 や 0 なら SD を読めていない疑い）
  delay(200);
  int n = player.readFileCounts();
  Serial.print(F("SD上のファイル数 = ")); Serial.println(n);
  Serial.println(F("(-1 なら SD を認識できていない → フォーマット/配線を疑う)"));

  Serial.println();
  Serial.println(F("コマンド: b=ビープ / r=root再生 / m=mp3folder再生"));
  Serial.println(F("          1-5=各トラック / +,-=音量 / ?=状態"));
  Serial.println(F(">> まず b を送ってビープが鳴るか確認してください"));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') return;

    switch (c) {
      case 'b':
        Serial.println(F("[cmd] ビープ(6) を playMp3Folder で再生"));
        player.playMp3Folder(6);
        break;
      case 'r':
        Serial.println(F("[cmd] 6 を play()＝ルート直下方式で再生"));
        player.play(6);
        break;
      case 'm':
        Serial.println(F("[cmd] 6 を playMp3Folder()＝mp3フォルダ方式で再生"));
        player.playMp3Folder(6);
        break;
      case '1': case '2': case '3': case '4': case '5': {
        int t = c - '0';
        Serial.print(F("[cmd] playMp3Folder(")); Serial.print(t); Serial.println(F(")"));
        player.playMp3Folder(t);
        break;
      }
      case '+':
        vol = min(30, vol + 3); player.volume(vol);
        Serial.print(F("音量 = ")); Serial.println(vol);
        break;
      case '-':
        vol = max(0, vol - 3); player.volume(vol);
        Serial.print(F("音量 = ")); Serial.println(vol);
        break;
      case '?':
        Serial.print(F("ファイル数 = ")); Serial.println(player.readFileCounts());
        Serial.print(F("現在の音量 = ")); Serial.println(player.readVolume());
        break;
      default:
        Serial.print(F("未知のコマンド: ")); Serial.println(c);
        break;
    }
  }

  // DFPlayer からの通知を表示
  if (player.available()) {
    uint8_t type = player.readType();
    int value = player.read();
    switch (type) {
      case DFPlayerPlayFinished:
        Serial.print(F("[info] 再生完了: ")); Serial.println(value); break;
      case DFPlayerError:
        Serial.print(F("[err] エラーコード: ")); Serial.println(value);
        Serial.println(F("      (TD5580Aは未定義コードを返すことあり)"));
        break;
      case DFPlayerCardOnline:  Serial.println(F("[info] SDオンライン")); break;
      case DFPlayerCardInserted:Serial.println(F("[info] SD挿入")); break;
      case DFPlayerCardRemoved: Serial.println(F("[info] SD抜去")); break;
      default: break;
    }
  }
}
