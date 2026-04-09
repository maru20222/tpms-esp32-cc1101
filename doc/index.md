# TPMS受信機 ハードウェア構成

## CC1101とESP32-S3-WROOM-2 N32R16V ピン対応表

### SPI接続

| CC1101 ピン番号 | CC1101 機能 | ESP32-S3 GPIO | 説明 |
|----------------|------------|---------------|------|
| 1 | GND | GND | グランド |
| 2 | VCC | 3.3V | 電源（3.3V） |
| 3 | GDO0 | GPIO16 | 汎用デジタル出力0（キャリア検出等に使用可能） |
| 4 | CSN | GPIO10 | SPI チップセレクト |
| 5 | SCK | GPIO12 | SPI クロック |
| 6 | MOSI (SI) | GPIO11 | SPI データ入力 (Master Out Slave In) |
| 7 | MISO (SO) | GPIO13 | SPI データ出力 (Master In Slave Out) |
| 8 | GDO2 | GPIO15 | 非同期データ出力（割り込み用） |

### 備考

- **ESP32-S3-WROOM-2 N32R16V** はOctal Flash + Octal PSRAMを搭載
- **CC1101** は315MHz/433MHz/868MHz/915MHz帯のサブGHz無線トランシーバ
- **GDO0 (GPIO16)** : キャリアセンス出力（信号検出時Highでノイズ除去に使用）
- **GDO2 (GPIO15)** : CC1101から非同期データを受信し、エッジ検出割り込みで処理
- **電源** : CC1101は3.3V動作、ESP32-S3から直接供給可能

### ソフトウェア設定

```cpp
// SPI ピン定義 (src/main.cpp)
static const int PIN_CS   = 10;   // CC1101 pin4 CSN
static const int PIN_SCK  = 12;   // CC1101 pin5 SCK
static const int PIN_MOSI = 11;   // CC1101 pin6 MOSI
static const int PIN_MISO = 13;   // CC1101 pin7 MISO
static const int PIN_GDO0 = 16;   // CC1101 pin3 GDO0 (Carrier Sense)
static const int PIN_GDO2 = 15;   // CC1101 pin8 GDO2 (Async Data)
```

### 動作モード

このプロジェクトでは、CC1101を**非同期モード (Async Serial Mode)** で使用し、
GDO2ピンから出力される生データをESP32-S3で直接キャプチャして、
マンチェスター符号化されたTPMSセンサーデータをデコードします。

#### GDO0/GDO2の連携動作

GDO0（キャリアセンス）とGDO2（非同期データ）を組み合わせることで、
ノイズを除外して本物の信号のみを確実にキャプチャします：

```
┌─────────────────────────────────────────────────────────┐
│ GDO0 (Carrier Sense)                                    │
│          ____________________________                   │
│ ________|                            |_________________  │
│         ↑ 信号検出                    ↑ 信号消失         │
│         キャプチャ開始                 キャプチャ終了     │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│ GDO2 (Async Data)                                       │
│          ‾|_‾|__|‾‾|_|‾|__|‾‾|_|‾|__|                  │
│          ← マンチェスター符号化されたデータ              │
└─────────────────────────────────────────────────────────┘
          ↑                            ↑
        この区間のみGDO2をキャプチャ
        （ノイズは自動的に除外）
```

**動作の流れ（当初設計）：**
1. GDO0がHigh → CC1101がキャリア検出 → ESP32がキャプチャ開始
2. キャプチャ中、GDO2のエッジをすべて記録
3. GDO0がLow → キャリア消失 → ESP32がキャプチャ終了
4. 記録したデータをマンチェスターデコード→TPMS解析

> **補記（現在の実装）:** GDO0によるキャプチャ開始・終了のゲーティングは断念。  
> 現在は **GDO2のエッジ割り込みを常時稼働** し、エッジ間隔が一定時間（ギャップ）を超えたらバースト終了と判定する方式（ギャップ検出）を採用。  
> GDO0はバースト中に8エッジごとにポーリングし、一度でもHighになれば真のRF信号と判定するノイズフィルタ（`burstHadCarrier`）としてのみ使用している。

### フレーム構造（2026-04-04 確定版）

433.92MHz / 9.6kbps / Manchester符号化

```
物理信号 (GDO2 raw edge stream)
──────────────────────────────────────────────────────────────────

[ プリアンブル             ][ SYNC  ][ データ部 (8バイト)         ]
  AA AA AA ... AA AA           55 44    d[0]..d[6] d[7]=d[0]
  (0xAA繰り返し)               2バイト  7Bパケット固定

  ※ ビット反転版 (inv=1):
  55 55 55 ... 55 55           AA BB    ...

──────────────────────────────────────────────────────────────────
Manchester符号化 (half-bit単位, halfUs=52µs = 9.6kbps)

  bit 0 → 01  (Low→High)
  bit 1 → 10  (High→Low)

──────────────────────────────────────────────────────────────────
【確定】7Bパケット構造 (急変・定期送信共通)

  総バイト数: 8バイト (7バイトデータ + d[7]=d[0] の繰り返し)

  +--------+--------+--------+--------+--------+--------+--------+--------+
  |  d[0]  |  d[1]  |  d[2]  |  d[3]  |  d[4]  |  d[5]  |  d[6]  |  d[7]  |
  +--------+--------+--------+--------+--------+--------+--------+--------+
  C0-P*    C0'-P*   P*-S     C3-P*      P*      C5-P*    P*+Q     =d[0]

  凡例:
    P*  = d[4]  センサー固有の圧力値（生バイト）
    Q   = (d[6] - d[4]) mod 256  圧力量子化値
    S   = (d[4] - d[2] + 256) mod 256  温度量子化値
    C0, C3, C5 = センサー識別定数（センサーごとに固定）
    C0' = 通常時 C0-0x10 / 減圧時 C0-0x00 / 加圧時 C0-0x20

  ■ 冗長関係（多数決補正に使用）:
    d[0] + P* = C0   (mod 256)
    d[1] + P* = C0'  (mod 256)  ← 急変種別によって変化
    d[3] + P* = C3   (mod 256)
    d[5] + P* = C5   (mod 256)
    d[7]      = d[0]
    直接値 d[4] = P*
    計6票 → 最大票数を得たセンサー定数(C0/C3/C5)でセンサー識別 & P*確定

──────────────────────────────────────────────────────────────────
【確定】圧力・温度デコード式

  Q   = (d[6] - d[4]) mod 256
  kPa = pressConst - 2.5 × Q   （ゲージ圧）
  bar = kPa / 100

  ※ 8bit折り返し補正: kPa ≥ (pressConst - 30) の場合は kPa -= 640

  S   = (d[4] - d[2] + 256) mod 256
  T   = S - tempOffset          （℃）

──────────────────────────────────────────────────────────────────
【確定】センサー識別定数一覧


```

| ラベル | C0   | C3   | C5   | tempOffset | pressConst |
|--------|------|------|------|-----------|-----------|
| F.L A  | 0xF2 | 0x80 | 0x9B | 38        | 670.0     |
| F.R B  | 0xF9 | 0x87 | 0x38 | 45        | 657.0     |
| R.R C  | 0xF2 | 0xC0 | 0xD9 | 38        | 670.0     |
| R.L D  | 0xF2 | 0x70 | 0xDE | 38        | 670.0     |

```

──────────────────────────────────────────────────────────────────
【確定】急変アラート検出

  d1P = d[1] + P*  (mod 256) で判定:
    d1P == C0 - 0x10  → 通常定期送信
    d1P == C0 - 0x00  → ALT:減圧（d[1]が通常より +0x10）
    d1P == C0 - 0x20  → ALT:加圧（d[1]が通常より -0x10）

  ※ votes < 6 の場合は d[1] が外れ票になりやすい（急変時の正常動作）

──────────────────────────────────────────────────────────────────
解決済み課題

  1. 3Bパケット仮説 → 誤り。実際にはすべて7Bパケット
  2. d[6] の正体 → P*+Q（圧力情報）と確定
  3. d[2] 系統誤り → オフセット補正（±1〜2 half-bit試行）で解消
  4. センサーID → C0/C3/C5 定数 + 6票多数決で自動判別
  5. 圧力式 → kPa = pressConst - 2.5×Q で確定
  6. 温度式 → T = S - tempOffset で確定（全センサー確認済み）
  7. F.R B の pressConst/tempOffset → 実測校正済み（657, 45）

──────────────────────────────────────────────────────────────────
```


---

## LCD ハードウェア構成

### 採用モジュール

| 項目 | 内容 |
|------|------|
| 型番 | 秋月電子 M154-240240-RGB |
| コントローラ | ST7789 |
| 解像度 | 240 × 240 px |
| 接続方式 | SPI（4線式, 書き込み専用）|
| 電源 | 3.3V |
| 購入日 | 2026-04-04 |

### ピン接続（ESP32-S3 ← → M154-240240-RGB）

CC1101 とは**独立した SPI3 バス**を使用（速度設定の切り替え不要、干渉なし）。

| M154 ピン | 機能 | ESP32-S3 GPIO | 備考 |
|-----------|------|---------------|------|
| VCC | 電源 | 3.3V | |
| GND | グランド | GND | |
| SCL | SPI クロック | GPIO17 | SPI3 専用 |
| SDA | SPI MOSI | GPIO18 | SPI3 専用 |
| RES | ハードリセット | GPIO4 | |
| DC | データ/コマンド | GPIO5 | |
| CS | チップセレクト | GPIO6 | |
| BLK | バックライト | GPIO7 | High=ON、または3.3V直結 |

### ソフトウェア設定

```cpp
#define LCD_SCK   17   // SPI3 専用
#define LCD_MOSI  18   // SPI3 専用
#define LCD_RST    4
#define LCD_DC     5
#define LCD_CS     6
#define LCD_BLK    7
```

### ライブラリ: Adafruit ST7789 Library, GFX Library

platformio.ini に追加：

```ini
lib_deps =
  adafruit/Adafruit ST7735 and ST7789 Library
  adafruit/Adafruit GFX Library
```

User_Setup.h の主要設定：

```cpp
#define ST7789_DRIVER
#define TFT_WIDTH  240
#define TFT_HEIGHT 240
#define TFT_MOSI   18   // SPI3
#define TFT_SCLK   17   // SPI3
#define TFT_CS      6
#define TFT_DC      5
#define TFT_RST     4
#define TFT_BL      7
#define SPI_FREQUENCY  40000000
```

### 参考資料

- [CC1101 Datasheet](https://www.ti.com/lit/ds/symlink/cc1101.pdf)
- [ESP32-S3-DevKitC-1 User Guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.0.html)
- [RadioLib Documentation](https://github.com/jgromes/RadioLib)
- [Arduino TPMS Tyre Pressure Display](https://www.hackster.io/jsmsolns/arduino-tpms-tyre-pressure-display-b6e544)
- [reddit Need help decoding TPMS sensor](https://www.reddit.com/r/RTLSDR/comments/v0hqqf/need_help_decoding_tpms_sensor/)
- [Printables ESP32 with CC1101 Case](https://www.printables.com/model/1241571-esp32-with-cc1101-case)
