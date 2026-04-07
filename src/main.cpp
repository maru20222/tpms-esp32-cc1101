#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "lcd_display.h"

// ====== あなたのSPI配線に合わせる ======
static const int PIN_CS   = 10;   // CC1101 pin4 CSN
static const int PIN_SCK  = 12;   // CC1101 pin5 SCK
static const int PIN_MOSI = 11;   // CC1101 pin6 MOSI
static const int PIN_MISO = 13;   // CC1101 pin7 MISO

// GDO0 (CC1101 pin3) -> ESP32 GPIO16 (Carrier Sense)
static const int PIN_GDO0 = 16;
// GDO2 (CC1101 pin8) -> ESP32 GPIO15 (Async Data)
static const int PIN_GDO2 = 15;

// ====== CC1101 専用 SPI (HSPI = SPI3_HOST) ======
// TFT_eSPI が FSPI (SPI2_HOST, デフォルト SPI) を使用するため、
// CC1101 は HSPI (SPI3_HOST) を使用して干渉を回避する。
static SPIClass cc1101Spi(HSPI);

// ====== RadioLib ======
CC1101 radio = new Module(PIN_CS, /*GDO0*/ -1, /*RST*/ -1, /*GDO2*/ -1, cc1101Spi);

// ====== CC1101 regs/const ======
static const uint8_t REG_IOCFG2   = 0x00;
static const uint8_t REG_IOCFG0   = 0x02;
static const uint8_t REG_PKTCTRL0 = 0x08;
static const uint8_t REG_AGCCTRL2 = 0x07;
static const uint8_t REG_AGCCTRL1 = 0x06;
static const uint8_t READ_SINGLE  = 0x80;

void ccWrite(uint8_t addr, uint8_t val) {
  digitalWrite(PIN_CS, LOW);
  cc1101Spi.transfer(addr);
  cc1101Spi.transfer(val);
  digitalWrite(PIN_CS, HIGH);
}
uint8_t ccRead(uint8_t addr) {
  digitalWrite(PIN_CS, LOW);
  cc1101Spi.transfer(addr | READ_SINGLE);
  uint8_t v = cc1101Spi.transfer(0);
  digitalWrite(PIN_CS, HIGH);
  return v;
}
void ccSetPktFormat(uint8_t fmt) {
  uint8_t v = ccRead(REG_PKTCTRL0);
  v = (uint8_t)((v & ~0x30) | ((fmt & 0x03) << 4));
  ccWrite(REG_PKTCTRL0, v);
}
void ccEnableAsyncOnGDO2() {
  ccSetPktFormat(3);         // async serial
  ccWrite(REG_IOCFG2, 0x0D); // GDO2 = async data out
  
  // GDO0 = キャリアセンス出力
  // 0x0E: Carrier sense. HIGH when RSSI > threshold (信号受信中はHIGH)
  ccWrite(REG_IOCFG0, 0x0E); // GDO0 = Carrier Sense
  
  // キャリアセンス閾値を下げる（デフォルトより感度を上げる）
  ccWrite(REG_AGCCTRL2, 0x03);  // 絶対閾値を最も低く設定
  ccWrite(REG_AGCCTRL1, 0x00);  // キャリアセンスをより感度高く

  Serial.printf("IOCFG2=0x%02X IOCFG0=0x%02X PKTCTRL0=0x%02X AGCCTRL2=0x%02X\n",
                ccRead(REG_IOCFG2), ccRead(REG_IOCFG0), ccRead(REG_PKTCTRL0), ccRead(REG_AGCCTRL2));
  Serial.println("GDO0: Carrier Sense (HIGH=carrier detected)");
}

// ====== Burst capture ======
static const uint32_t BURST_GAP_US = 5000;  // エッジ間隔5ms以上でバースト終了（データ尾部を取りこぼさないよう）
static const int MAX_EDGES = 6000;          // バッファサイズ
static const int MIN_EDGES_TO_PARSE = 150;  // これ未満は解析しない（ノイズ除外）

volatile uint16_t dtBuf[MAX_EDGES];   // dt(us) 0..65535
volatile uint8_t  lvBuf[MAX_EDGES];   // level after edge
volatile int edgeN = 0;

volatile uint32_t lastEdgeUs = 0;
volatile uint32_t burstStartUs = 0;
volatile uint32_t burstEndUs = 0;
volatile bool burstReady = false;
volatile bool burstHadCarrier = false;  // バースト開始時にGDO0(キャリアセンス)がHIGHだったか

// プリアンブルのみバーストを検出後、次のバーストを強制ダンプするフラグ
static volatile bool dumpNextBurst = false;
static volatile uint32_t lastPreambleOnlyMs = 0;

// GDO2割り込み：全エッジを記録してギャップでバースト境界を検出
void IRAM_ATTR isrGdo2() {
  if (burstReady) return;  // バースト処理待ちの間は記録しない

  uint32_t now = micros();
  uint32_t dt = now - lastEdgeUs;
  lastEdgeUs = now;

  // 極端に短いグリッチを除外（10us未満は無視）
  if (dt > 0 && dt < 10) return;

  // 長いギャップ検出 → 前のバーストを確定
  if (dt > BURST_GAP_US && edgeN > 0) {
    if (edgeN >= MIN_EDGES_TO_PARSE) {
      burstEndUs = now - dt;  // ギャップ前の時刻
      burstReady = true;
      return;
    }
    // 短すぎたら破棄して新規開始
    edgeN = 0;
    burstStartUs = now;
  }

  // 最初のエッジ
  if (edgeN == 0) {
    burstStartUs = now;
    burstHadCarrier = false;  // バースト開始時にリセット
  }

  // キャリアセンスをバースト全体で累積（AGCの応答遅延を考慮）
  // GDO0がバースト中いずれかのタイミングでHIGHになれば真のRF信号
  if (!burstHadCarrier && (edgeN % 8 == 0)) {  // 8エッジごとにサンプリング
    burstHadCarrier = (digitalRead(PIN_GDO0) == HIGH);
  }

  // エッジ記録
  if (edgeN < MAX_EDGES) {
    dtBuf[edgeN] = (dt > 65535 ? 65535 : (uint16_t)dt);
    lvBuf[edgeN] = (uint8_t)digitalRead(PIN_GDO2);
    edgeN++;
  }
}

static inline int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// dt群から half-bit を推定（「20〜40us帯で最頻」のピークを取る）
// 戻り値: halfUs。*peakFrac に ピーク集中度(0.0-1.0)を返す
int estimateHalfBitUs(const uint16_t* dts, int n, float* peakFrac = nullptr) {
  // 1us刻みヒスト（0..200us）
  static uint16_t hist[201];
  memset(hist, 0, sizeof(hist));

  int validCnt = 0;
  for (int i = 0; i < n; i++) {
    int dt = dts[i];
    if (dt < 8 || dt > 200) continue;
    hist[dt]++;
    validCnt++;
  }

  // 19.2kbps(halfUs=26)と9.6kbps(halfUs=52)の両方を探索
  int bestDt = 26;
  uint16_t bestCnt = 0;
  for (int dt = 18; dt <= 65; dt++) {
    if (hist[dt] > bestCnt) {
      bestCnt = hist[dt];
      bestDt = dt;
    }
  }

  // 集中度: halfUt付近 ±4us と 2*halfUs ±4us に入るエッジの割合
  if (peakFrac != nullptr) {
    int inPeak = 0;
    int lo1 = bestDt - 4, hi1 = bestDt + 4;
    int lo2 = bestDt*2 - 6, hi2 = bestDt*2 + 6;
    for (int dt = 8; dt <= 200; dt++) {
      if ((dt >= lo1 && dt <= hi1) || (dt >= lo2 && dt <= hi2)) {
        inPeak += hist[dt];
      }
    }
    *peakFrac = (validCnt > 0) ? (float)inPeak / validCnt : 0.0f;
  }

  return bestDt;
}

// dt+level -> halfbit毎のレベル列へ展開
// 返り値: 展開した半ビット数
int expandToHalfbits(const uint16_t* dts, const uint8_t* lvs, int n,
                     int halfUs, uint8_t* halfLv, int halfMax) {
  int out = 0;

  // 各エッジは「前のレベルが dt続いた」なので、前レベル = !lv(after edge)
  for (int i = 0; i < n; i++) {
    int dt = dts[i];
    if (dt < 8) continue;          // グリッチ除外
    if (dt > 2000) break;          // 長すぎたらバースト終端扱い
    uint8_t prevLv = (uint8_t)(lvs[i] ^ 1);

    // half-bit単位に丸め（最低1）
    int k = (dt + halfUs/2) / halfUs;
    k = clampi(k, 1, 20);

    for (int j = 0; j < k; j++) {
      if (out >= halfMax) return out;
      halfLv[out++] = prevLv;
    }
  }
  return out;
}

// Manchester復号（01/10のどちらを1/0にするかで2通り）
int manchesterDecode(const uint8_t* halfLv, int halfN, uint8_t* bits, int bitMax, bool invert) {
  int out = 0;
  int pairs = halfN / 2;
  for (int i = 0; i < pairs && out < bitMax; i++) {
    uint8_t a = halfLv[i*2 + 0];
    uint8_t b = halfLv[i*2 + 1];
    // Manchester: 01 or 10 のみが理想
    // 01 -> bit0, 10 -> bit1 （invertで反転）
    int bit;
    if (a == 0 && b == 1) bit = 0;
    else if (a == 1 && b == 0) bit = 1;
    else {
      // 不正ペアでも1bitを消費してアラインメント維持
      // （スキップすると後段が詰まり、毎回同じ誤フレームを再現しやすい）
      bit = a;  // 暫定推定: 前半レベルを採用
    }
    if (invert) bit ^= 1;
    bits[out++] = (uint8_t)bit;
  }
  return out;
}

// Manchesterペア品質を評価（低レイヤー誤り指標）
// invalidRate が高いほど「00/11」ペアが多く、ビットスリップ/ノイズ疑いが強い
static float manchesterInvalidRate(const uint8_t* halfLv, int halfN, int maxPairs,
                                   int* outValid, int* outInvalid) {
  int pairs = halfN / 2;
  if (pairs > maxPairs) pairs = maxPairs;
  if (pairs <= 0) {
    if (outValid) *outValid = 0;
    if (outInvalid) *outInvalid = 0;
    return 1.0f;
  }
  int v = 0, bad = 0;
  for (int i = 0; i < pairs; i++) {
    uint8_t a = halfLv[i*2 + 0];
    uint8_t b = halfLv[i*2 + 1];
    if ((a == 0 && b == 1) || (a == 1 && b == 0)) v++;
    else bad++;
  }
  if (outValid) *outValid = v;
  if (outInvalid) *outInvalid = bad;
  return (float)bad / (float)(v + bad);
}

void printBits(const uint8_t* bits, int n, int maxPrint = 128) {
  int m = min(n, maxPrint);
  for (int i = 0; i < m; i++) Serial.print(bits[i] ? '1' : '0');
  if (n > m) Serial.print("...");
  Serial.println();
}

void printBytesFromBits(const uint8_t* bits, int nBits, int maxBytes = 16) {
  int nBytes = min(maxBytes, nBits / 8);
  for (int i = 0; i < nBytes; i++) {
    uint8_t v = 0;
    for (int b = 0; b < 8; b++) {
      v = (uint8_t)((v << 1) | (bits[i*8 + b] & 1));
    }
    Serial.printf("%02X ", v);
  }
  Serial.println();
}

// バイト配列を生成（bits -> bytes変換）
void bitsToBytes(const uint8_t* bits, int nBits, uint8_t* bytes, int* nBytes) {
  *nBytes = nBits / 8;
  for (int i = 0; i < *nBytes; i++) {
    uint8_t v = 0;
    for (int b = 0; b < 8; b++) {
      v = (uint8_t)((v << 1) | (bits[i*8 + b] & 1));
    }
    bytes[i] = v;
  }
}

// CRC-8 計算（多項式 poly, 初期値 init）
uint8_t crc8_generic(const uint8_t* data, int len, uint8_t poly, uint8_t init) {
  uint8_t crc = init;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ poly);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// CRC-8 計算（多項式 0x07, init=0x00）
uint8_t crc8(const uint8_t* data, int len) {
  return crc8_generic(data, len, 0x07, 0x00);
}

// CRC-8 計算（多項式 0x31）
uint8_t crc8_31(const uint8_t* data, int len) {
  uint8_t crc = 0xFF;  // 初期値0xFF
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x31);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// CRC-8 計算（多項式 0x1D）
uint8_t crc8_1d(const uint8_t* data, int len) {
  uint8_t crc = 0x00;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x1D);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// CRC-8 計算（多項式 0x13 - Toyota用）
uint8_t crc8_toyota(const uint8_t* data, int len) {
  uint8_t crc = 0x00;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x13);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// チェックサム計算
uint8_t checksum(const uint8_t* data, int len) {
  uint8_t sum = 0;
  for (int i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum;
}

// データ内に繰り返しパターンがないかチェック（ノイズ除外）
bool hasRepeatingPattern(const uint8_t* data, int len) {
  if (len < 3) return false;
  
  // 同じバイトが3回以上連続
  for (int i = 0; i < len - 2; i++) {
    if (data[i] == data[i+1] && data[i] == data[i+2]) {
      // 例外: 実際のTPMSで出現しうる値は許可
      if (data[i] != 0x55 && data[i] != 0xAA && data[i] != 0x00 && data[i] != 0xFF) {
        continue;  // それ以外の繰り返しはOK
      }
      return true;  // 0x55/0xAA/0x00/0xFFの3連続はNG
    }
  }
  return false;
}

// CRC検証（複数の方式を試す）
bool verifyCRC(const uint8_t* data, int len, bool silent = false) {
  // 中国製TPMSは通常3-8バイト
  if (len < 3 || len > 8) return false;
  
  // 繰り返しパターンチェック（これは有効なので維持）
  if (hasRepeatingPattern(data, len)) {
    return false;
  }
  
  // データ品質チェック：全バイトが0x00や0xFFはNG
  bool allZero = true;
  bool allFF = true;
  for (int i = 0; i < len - 1; i++) {
    if (data[i] != 0x00) allZero = false;
    if (data[i] != 0xFF) allFF = false;
  }
  if (allZero || allFF) return false;
  
  // パターン1: 最後の1バイトがCRC
  uint8_t receivedCRC = data[len - 1];
  
  // CRC-8 (0x1D, init=0x00) - 実測確認済み (1D 0D -> 9D)
  if (crc8_generic(data, len - 1, 0x1D, 0x00) == receivedCRC) {
    if (!silent) Serial.printf("    CRC-8(0x1D,i=00) OK: %02X (len=%d)\n", receivedCRC, len);
    return true;
  }
  // CRC-8 (0x07, init=0xB2) - 中国製TPMS多数派 (7B長パケット向け)
  if (crc8_generic(data, len - 1, 0x07, 0xB2) == receivedCRC) {
    if (!silent) Serial.printf("    CRC-8(0x07,i=B2) OK: %02X (len=%d)\n", receivedCRC, len);
    return true;
  }

#if 0  // 未確認ポリノミアル: 他センサー調査時に有効化(複数パケットで一致確認後)
  // CRC-8 (0xA9, init=0xFF) - POLY FOUND単発ヒット (35 25 80 B3 -> BD)
  if (crc8_generic(data, len - 1, 0xA9, 0xFF) == receivedCRC) {
    if (!silent) Serial.printf("    CRC-8(0xA9,i=FF) OK: %02X (len=%d)\n", receivedCRC, len);
    return true;
  }
  // CRC-8 (0x4B, init=0x00) - POLY FOUND単発ヒット (6F 5F 46 ED -> 83)
  if (crc8_generic(data, len - 1, 0x4B, 0x00) == receivedCRC) {
    if (!silent) Serial.printf("    CRC-8(0x4B,i=00) OK: %02X (len=%d)\n", receivedCRC, len);
    return true;
  }
  // CRC-8 (0x3F, init=0x00) - POLY FOUND単発ヒット (03 F3 B1 81 -> EF)
  if (crc8_generic(data, len - 1, 0x3F, 0x00) == receivedCRC) {
    if (!silent) Serial.printf("    CRC-8(0x3F,i=00) OK: %02X (len=%d)\n", receivedCRC, len);
    return true;
  }
  // CRC-8 (0x07, init=0x00) - 一般的
  if (crc8_generic(data, len - 1, 0x07, 0x00) == receivedCRC) {
    if (!silent) Serial.printf("    CRC-8(0x07,i=00) OK: %02X (len=%d)\n", receivedCRC, len);
    return true;
  }
  // CRC-8 (0x07, init=0xFF)
  if (crc8_generic(data, len - 1, 0x07, 0xFF) == receivedCRC) {
    if (!silent) Serial.printf("    CRC-8(0x07,i=FF) OK: %02X (len=%d)\n", receivedCRC, len);
    return true;
  }
  // CRC-8 (0x31, init=0xFF) - Toyota等
  if (crc8_generic(data, len - 1, 0x31, 0xFF) == receivedCRC) {
    if (!silent) Serial.printf("    CRC-8(0x31,i=FF) OK: %02X (len=%d)\n", receivedCRC, len);
    return true;
  }
  // XOR チェックサム（一部の中国製）
  {
    uint8_t xorSum = 0;
    for (int i = 0; i < len - 1; i++) xorSum ^= data[i];
    if (xorSum == receivedCRC) {
      if (!silent) Serial.printf("    XOR-sum OK: %02X (len=%d)\n", receivedCRC, len);
      return true;
    }
  }
#endif  // 未確認ポリノミアル

  return false;
}

// バーストID記録（繰り返し検出用）
struct BurstRecord {
  uint32_t id;      // 最初の4バイトをIDとする
  uint32_t lastSeen;
  int count;
};

static BurstRecord burstHistory[10];
static int burstHistorySize = 0;
static const uint32_t UNCONFIRMED_ID_TTL_MS = 90000;  // 単発IDは90秒で自動期限切れ

static void pruneBurstHistory() {
  uint32_t now = millis();
  int w = 0;
  for (int i = 0; i < burstHistorySize; i++) {
    bool staleUnconfirmed = (burstHistory[i].count < 2) && ((now - burstHistory[i].lastSeen) > UNCONFIRMED_ID_TTL_MS);
    if (staleUnconfirmed) continue;
    if (w != i) burstHistory[w] = burstHistory[i];
    w++;
  }
  burstHistorySize = w;
}

// IDを記録し、count>=2かつ前回から65秒以内なら true を返す（CONFIRMED）
// TPMS標準送信間隔は約60秒
// 注: ここで扱うIDは「ID専用フレーム」のみ。7B圧力フレーム先頭2BはIDではない。
bool recordBurst(const uint8_t* data, int len) {
  if (len < 2) return false;

  // 先頭2B をセンサーID として抽出（ID専用フレーム向け）
  uint16_t sensorId = ((uint16_t)data[0] << 8) | data[1];
  
  uint32_t now = millis();
  
  // 既存IDを探す
  for (int i = 0; i < burstHistorySize; i++) {
    if (burstHistory[i].id == ((uint32_t)sensorId)) {  // 16bit IDを32bit互換で扱う
      uint32_t elapsed = now - burstHistory[i].lastSeen;
      burstHistory[i].lastSeen = now;
      burstHistory[i].count++;
      // 65秒以内に再度同じIDが来た場合にCONFIRMED（TPMS送信間隔60s対応）
      return (elapsed <= 65000);
    }
  }
  
  // 新しいIDを追加（最古を上書き）
  int slot = burstHistorySize < 10 ? burstHistorySize++ : 0;
  // 満杯の場合は最古スロットを探す
  if (burstHistorySize >= 10) {
    uint32_t oldest = burstHistory[0].lastSeen;
    slot = 0;
    for (int i = 1; i < 10; i++) {
      if (burstHistory[i].lastSeen < oldest) {
        oldest = burstHistory[i].lastSeen;
        slot = i;
      }
    }
  }
  burstHistory[slot].id = (uint32_t)sensorId;
  burstHistory[slot].count = 1;
  burstHistory[slot].lastSeen = now;
  return false;  // 初回はまだCONFIRMEDではない
}

// 同期ワード構造体
struct SyncResult {
  int pos;        // 同期ワード終了位置
  int len;        // 同期ワード長さ
  const char* type; // タイプ
};

// ★ halfLvを直接走査してSYNCワード(55 44 / AA BB)を探す
// Manchester符号化後のhalf-bitパターンを直接マッチング
// → manchesterDecode()の「不正ペアスキップによる位置ズレ」問題を回避
// 戻り値: データ先頭のhalf-bitインデックス（SYNC直後）。見つからなければ-1
// out_inv: true=AA BB SYNC（データビットを反転して読む）
int findSyncInHalfLv(const uint8_t* halfLv, int halfN, bool* out_inv) {
  // 55 = 0101 0101(MSB first) → Manchester halfLv: 01 10 01 10 01 10 01 10
  // 44 = 0100 0100(MSB first) → Manchester halfLv: 01 10 01 01 01 10 01 01
  static const uint8_t pat5544[32] = {
    0,1,1,0, 0,1,1,0, 0,1,1,0, 0,1,1,0,  // 55
    0,1,1,0, 0,1,0,1, 0,1,1,0, 0,1,0,1   // 44
  };
  // AA = 1010 1010 → halfLv: 10 01 10 01 10 01 10 01
  // BB = 1011 1011 → halfLv: 10 01 10 10 10 01 10 10
  static const uint8_t patAABB[32] = {
    1,0,0,1, 1,0,0,1, 1,0,0,1, 1,0,0,1,  // AA
    1,0,0,1, 1,0,1,0, 1,0,0,1, 1,0,1,0   // BB
  };
  // 同期直前プレアンブル整合を採点して、最良候補を採用する
  auto scorePreamble = [&](int syncStart, bool for5544) -> int {
    // 5544の前はAAプリアンブル、AABBの前は55プリアンブルを期待
    static const uint8_t preAA[4] = {1,0,0,1};
    static const uint8_t pre55[4] = {0,1,1,0};
    const uint8_t* pre = for5544 ? preAA : pre55;

    int lookback = syncStart;
    if (lookback > 96) lookback = 96;
    if (lookback < 16) return -1000;  // プレアンブルが短すぎる候補は弱くする

    int start = syncStart - lookback;
    int bestMatch = -1;
    int bestRun = 0;
    for (int phase = 0; phase < 4; phase++) {
      int match = 0;
      int run = 0;
      int bestRunPh = 0;
      for (int j = 0; j < lookback; j++) {
        uint8_t exp = pre[(j + phase) & 0x03];
        if (halfLv[start + j] == exp) {
          match++;
          run++;
          if (run > bestRunPh) bestRunPh = run;
        } else {
          run = 0;
        }
      }
      if (match > bestMatch) {
        bestMatch = match;
        bestRun = bestRunPh;
      }
    }

    // 70%以上一致を最低条件にし、直前連続runも加点
    if (bestMatch * 10 < lookback * 7) return -1000;
    return bestMatch * 8 + bestRun * 2;
  };

  int bestStart = -1;
  bool bestInv = false;
  int bestScore = -100000;

  for (int i = 0; i <= halfN - 32; i++) {
    bool m5544 = true, mAABB = true;
    for (int j = 0; j < 32; j++) {
      uint8_t v = halfLv[i+j];
      if (v != pat5544[j]) m5544 = false;
      if (v != patAABB[j]) mAABB = false;
      if (!m5544 && !mAABB) break;
    }
    if (m5544) {
      int sc = scorePreamble(i, true);
      // 早すぎる候補(前半)をやや減点し、十分な前置きを持つ候補を優先
      if (i < 48) sc -= 20;
      if (sc > bestScore) {
        bestScore = sc;
        bestStart = i + 32;
        bestInv = false;
      }
    }
    if (mAABB) {
      int sc = scorePreamble(i, false);
      if (i < 48) sc -= 20;
      if (sc > bestScore) {
        bestScore = sc;
        bestStart = i + 32;
        bestInv = true;
      }
    }
  }
  if (bestStart >= 0) {
    if (out_inv) *out_inv = bestInv;
    return bestStart;
  }
  return -1;
}

// 拡張プリアンブル/同期ワード検出
// 複数パターンに対応: 0x55, 0xAA, 0x5555, 0x5569, 繰り返しパターンなど
SyncResult findSyncWord(const uint8_t* bits, int nBits) {
  SyncResult result = {-1, 0, "none"};
  
  if (nBits < 16) return result;

  // 0. 中国製TPMS典型同期ワード 0x2DD4 = 0010 1101 1101 0100
  // プリアンブル後に発生する特殊ピットパターンを優先検索
  {
    const uint8_t pat2dd4[] = {0,0,1,0,1,1,0,1,1,1,0,1,0,1,0,0};
    const uint8_t patD391[] = {1,1,0,1,0,0,1,1,1,0,0,1,0,0,0,1}; // 0xD391
    const uint8_t patBB3D[] = {1,0,1,1,1,0,1,1,0,0,1,1,1,1,0,1}; // 0xBB3D
    const uint8_t pat5544[] = {0,1,0,1,0,1,0,1,0,1,0,0,0,1,0,0}; // 0x5544 (AAプリアンブル終端後)
    const uint8_t patAABB[] = {1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,1}; // 0xAABB (inv)
    struct { const uint8_t* p; const char* n; } syncs[] = {
      {pat5544, "0x5544"}, {patAABB, "0xAABB"},
      {pat2dd4, "0x2DD4"}, {patD391, "0xD391"}, {patBB3D, "0xBB3D"}
    };
    for (int si = 0; si < 5; si++) {
      for (int i = 0; i < nBits - 16; i++) {
        bool match = true;
        for (int j = 0; j < 16; j++) {
          if (bits[i+j] != syncs[si].p[j]) { match = false; break; }
        }
        if (match) {
          result.pos = i + 16;
          result.len = 16;
          result.type = syncs[si].n;
          return result; // 特殊パターンは優先
        }
      }
    }
  }
  
  // 1. 0x55 (01010101) パターン - 最低12ビット（緩和）
  for (int i = 0; i < nBits - 16; i++) {
    int len = 0;
    for (int j = i; j < nBits - 1; j += 2) {
      if (bits[j] == 0 && bits[j+1] == 1) {
        len += 2;
      } else {
        break;
      }
    }
    if (len >= 12 && len > result.len) {  // 12ビットに緩和
      result.len = len;
      result.pos = i + len;
      result.type = "0x55";
    }
  }
  
  // 2. 0xAA (10101010) パターン - 最低12ビット
  for (int i = 0; i < nBits - 16; i++) {
    int len = 0;
    for (int j = i; j < nBits - 1; j += 2) {
      if (bits[j] == 1 && bits[j+1] == 0) {
        len += 2;
      } else {
        break;
      }
    }
    if (len >= 12 && len > result.len) {
      result.len = len;
      result.pos = i + len;
      result.type = "0xAA";
    }
  }
  
  // 3. 0x5569 (0101010101101001) などの特殊同期ワード
  // 0x5569 = 0101 0101 0110 1001
  const uint8_t sync5569[] = {0,1,0,1,0,1,0,1,0,1,1,0,1,0,0,1};
  for (int i = 0; i < nBits - 16; i++) {
    bool match = true;
    for (int j = 0; j < 16; j++) {
      if (bits[i+j] != sync5569[j]) {
        match = false;
        break;
      }
    }
    if (match && result.pos < 0) {
      result.pos = i + 16;
      result.len = 16;
      result.type = "0x5569";
      return result; // 特殊パターンは優先
    }
  }
  
  // 4. 0x5555 (0101010101010101) = 16ビットの0x55
  const uint8_t sync5555[] = {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1};
  for (int i = 0; i < nBits - 16; i++) {
    bool match = true;
    for (int j = 0; j < 16; j++) {
      if (bits[i+j] != sync5555[j]) {
        match = false;
        break;
      }
    }
    if (match && result.len < 16) {
      result.pos = i + 16;
      result.len = 16;
      result.type = "0x5555";
    }
  }
  
  return result;
}

// ======== 7B パケット解析ユーティリティ ========
// ★ CRC: poly/init 未確定。d[6] は CRC ではなく圧力情報を保持。
//   旧 crc8_poly2f / crc8_7b_skip2 は廃止。

static constexpr int MAX_7B_PRESSURE_VOTES = 6;

// ===== センサー種別定義 =====
// 各ホイールのTPMSセンサーは (C0, C3, C5) の定数の組み合わせでフレームを識別する
// C0 = (d[0] + P*) & 0xFF = (d[7] + P*) & 0xFF  (センサーにより異なる)
// C3 = (d[3] + P*) & 0xFF  (センサーにより異なる)
// C5 = (d[5] + P*) & 0xFF  (センサーにより異なる)
struct SensorDef {
  uint8_t c0;           // d[0]+P* = d[7]+P* の定数
  uint8_t c3;           // d[3]+P* の定数
  uint8_t c5;           // d[5]+P* の定数
  int8_t  tempOffset;   // 温度式オフセット: T(°C) = S - tempOffset
  float   pressConst;   // 圧力式定数: kPa = pressConst - 2.5*Q
  const char* label;    // 識別ラベル (センサーシール表記)
};
static const SensorDef SENSOR_DEFS[] = {
  { 0xF2, 0x70, 0xDE, 38, 670.0f, "R.L D" },  // リア左センサー (確定)
  { 0xF2, 0x80, 0x9B, 38, 670.0f, "F.L A" },  // フロント左センサー (確定)
  { 0xF9, 0x87, 0x38, 45, 657.0f, "F.R B" },  // フロント右センサー (Q=191→179kPa実測)
  { 0xF2, 0xC0, 0xD9, 38, 670.0f, "R.R C" },  // リア右センサー (C0=F2共通, C3/C5確定)
  // { 0x??, 0x??, 0x??, ??, ???.0f, "?" },  // 未計測
};
static const int SENSOR_DEF_COUNT = (int)(sizeof(SENSOR_DEFS)/sizeof(SENSOR_DEFS[0]));
static const int SENSOR_IDX_UNKNOWN = -1;

// センサー定数を取得 (インデックス範囲外は先頭を返すフォールバック)
static inline const SensorDef& getSensorDef(int idx) {
  if (idx >= 0 && idx < SENSOR_DEF_COUNT) return SENSOR_DEFS[idx];
  return SENSOR_DEFS[0];  // fallback: R.L D
}

// 7Bパケット多数決ローリング値(P*)補正 + センサー種別自動判定
// 各センサー定数 (c0, c3, c5) でd[0]/d[1]/d[3]/d[5]/d[7]から6票多数決を試み、
// 最大票数を得たセンサー種別と P* を返す
// d[6] = P*+Q (圧力情報: kPa = 670 - 2.5*Q)
// sensorIdx: 検出されたセンサー種別 (SENSOR_DEFS配列インデックス, -1=不明)
static uint8_t correct7BRolling(const uint8_t* d, int* votes, int* sensorIdx = nullptr) {
  int bestVotes = 0;
  int bestSensorIdx = SENSOR_IDX_UNKNOWN;
  uint8_t bestP = d[4];  // fallback
  for (int si = 0; si < SENSOR_DEF_COUNT; si++) {
    const uint8_t c0 = SENSOR_DEFS[si].c0;
    uint8_t cands[6] = {
      (uint8_t)(c0 - d[0]),                   // from d[0]
      (uint8_t)(c0 - d[1]),                   // from d[1] (急変時は外れ票になる)
      d[4],                                    // from d[4] (直接)
      (uint8_t)(c0 - d[7]),                   // from d[7]
      (uint8_t)(SENSOR_DEFS[si].c3 - d[3]),   // from d[3]
      (uint8_t)(SENSOR_DEFS[si].c5 - d[5]),   // from d[5]
    };
    uint8_t v = cands[0]; int cnt = 0;
    for (int i = 0; i < 6; i++) {
      int c = 0;
      for (int j = 0; j < 6; j++) if (cands[j] == cands[i]) c++;
      if (c > cnt) { cnt = c; v = cands[i]; }
    }
    if (cnt > bestVotes) { bestVotes = cnt; bestSensorIdx = si; bestP = v; }
  }
  if (votes) *votes = bestVotes;
  if (sensorIdx) *sensorIdx = bestSensorIdx;
  return bestP;
}

struct PressureDecoded {
  float kPa;
  float bar;
  float psi;
};

struct AssocFrame {
  uint32_t tsMs;
  bool is7B;
  int len;
  uint8_t bytes[12];
  float bar;
};

static AssocFrame assocFrames[12];
static int assocFrameCount = 0;
static const uint32_t ASSOC_WINDOW_MS = 500;

static bool is7BPressureFrameCandidate(const uint8_t* pkt, int len, int* outVotes = nullptr, uint8_t* outP = nullptr) {
  if (len < 8) {
    if (outVotes) *outVotes = 0;
    if (outP) *outP = 0;
    return false;
  }
  int votes = 0;
  int si = SENSOR_IDX_UNKNOWN;
  uint8_t P = correct7BRolling(pkt, &votes, &si);
  int ok = 0;
  if (votes >= 5) {
    const SensorDef& sd = getSensorDef(si);
    if (pkt[0] == (uint8_t)(sd.c0 - P)) ok++;
    if (pkt[1] == (uint8_t)(sd.c0 - P)) ok++;
    if (pkt[3] == (uint8_t)(sd.c3 - P)) ok++;
    if (pkt[5] == (uint8_t)(sd.c5 - P)) ok++;
    if (pkt[7] == pkt[0]) ok++;
  }
  if (outVotes) *outVotes = votes;
  if (outP) *outP = P;
  return (votes >= 5 && ok >= 4);
}

static bool is7BPressureFragmentCandidate(const uint8_t* pkt, int len, uint8_t* outP = nullptr) {
  if (len < 6) {
    if (outP) *outP = 0;
    return false;
  }
  uint8_t P = pkt[4];
  // いずれかのセンサー定数で ok>=4 を満たせば合格
  for (int si = 0; si < SENSOR_DEF_COUNT; si++) {
    int ok = 0;
    if (pkt[0] == (uint8_t)(SENSOR_DEFS[si].c0 - P)) ok++;
    if (pkt[1] == (uint8_t)(SENSOR_DEFS[si].c0 - P)) ok++;
    if (pkt[3] == (uint8_t)(SENSOR_DEFS[si].c3 - P)) ok++;
    if (pkt[5] == (uint8_t)(SENSOR_DEFS[si].c5 - P)) ok++;
    if (ok >= 4) {
      if (outP) *outP = P;
      return true;
    }
  }
  // d[2]は温度フィールドのため圧力チェックから除外
  if (outP) *outP = P;
  return false;
}

// 7B圧力フレーム先頭4B断片の検出
// 例: [F2-P, F2-P, x, C3-P] で x(=d[2]温度) が欠損/化けても、
// 先頭2B + d[3] の整合で7B由来断片とみなす (いずれかのセンサー定数で一致)
static bool is7BPressurePrefixCandidate(const uint8_t* pkt, int len, uint8_t* outP = nullptr) {
  if (len < 4) {
    if (outP) *outP = 0;
    return false;
  }
  // いずれかのセンサーの c0 で d[0]/d[1] が一致し、かつ d[3] が c3 と整合するか確認
  for (int si = 0; si < SENSOR_DEF_COUNT; si++) {
    uint8_t p0 = (uint8_t)(SENSOR_DEFS[si].c0 - pkt[0]);
    uint8_t p1 = (uint8_t)(SENSOR_DEFS[si].c0 - pkt[1]);
    if (p0 != p1) continue;
    uint8_t p3 = (uint8_t)(SENSOR_DEFS[si].c3 - pkt[3]);
    if (p0 == p3) {
      if (outP) *outP = p0;
      return true;
    }
  }
  if (outP) *outP = 0;
  return false;
}

// 7B圧力フレームの「ゆるめ」判定
// 1〜2バイト化けで strict 判定を外れるケースをID登録から守るための保護用途。
static bool is7BPressureLikelyCandidate(const uint8_t* pkt, int len, uint8_t* outP = nullptr) {
  if (is7BPressureFrameCandidate(pkt, len, nullptr, outP)) return true;
  if (is7BPressureFragmentCandidate(pkt, len, outP)) return true;
  if (is7BPressurePrefixCandidate(pkt, len, outP)) return true;

  if (len < 7) {
    if (outP) *outP = 0;
    return false;
  }

  // Check voting mechanism - if votes >= 3, frame is likely 7B
  int votes = 0;
  uint8_t P = correct7BRolling(pkt, &votes);
  if (votes >= 3) {
    if (outP) *outP = P;
    return true;
  }

  // Fallback: P-based pattern check on direct d[4] byte (センサー定数を全試行)
  uint8_t P2 = pkt[4];
  for (int si = 0; si < SENSOR_DEF_COUNT; si++) {
    int ok = 0;
    if (pkt[0] == (uint8_t)(SENSOR_DEFS[si].c0 - P2)) ok++;
    if (pkt[3] == (uint8_t)(SENSOR_DEFS[si].c3 - P2)) ok++;
    if (pkt[5] == (uint8_t)(SENSOR_DEFS[si].c5 - P2)) ok++;
    if (ok >= 3) { if (outP) *outP = P2; return true; }
  }
  if (outP) *outP = P2;
  return false;
}

static void pushAssocFrame(bool is7B, const uint8_t* bytes, int len, float bar) {
  if (len <= 0) return;
  int slot = assocFrameCount < 12 ? assocFrameCount++ : 0;
  if (assocFrameCount >= 12) {
    uint32_t oldest = assocFrames[0].tsMs;
    slot = 0;
    for (int i = 1; i < 12; i++) {
      if (assocFrames[i].tsMs < oldest) {
        oldest = assocFrames[i].tsMs;
        slot = i;
      }
    }
  }
  assocFrames[slot].tsMs = millis();
  assocFrames[slot].is7B = is7B;
  assocFrames[slot].len = (len > 12) ? 12 : len;
  assocFrames[slot].bar = bar;
  memset(assocFrames[slot].bytes, 0, sizeof(assocFrames[slot].bytes));
  memcpy(assocFrames[slot].bytes, bytes, assocFrames[slot].len);
}

static void printAssocCandidatesFor7B(const uint8_t* bytes, int len, float bar) {
  uint32_t now = millis();
  bool any = false;
  for (int i = 0; i < assocFrameCount; i++) {
    if (assocFrames[i].is7B) continue;
    uint32_t dt = now - assocFrames[i].tsMs;
    if (dt > ASSOC_WINDOW_MS) continue;
    if (!any) {
      Serial.printf("[ASSOC 7B %.2fbar] nearby non-5544 frames:\n", bar);
      any = true;
    }
    Serial.printf("  dt=%lums len=%d raw=", (unsigned long)dt, assocFrames[i].len);
    for (int b = 0; b < assocFrames[i].len; b++) Serial.printf("%02X", assocFrames[i].bytes[b]);
    Serial.println();
  }
  if (!any) {
    Serial.printf("[ASSOC 7B %.2fbar] no nearby non-5544 frame within %lums\n", bar, (unsigned long)ASSOC_WINDOW_MS);
  }
}

static void printAssocMatchesForNon7B(const uint8_t* bytes, int len) {
  uint32_t now = millis();
  bool any = false;
  for (int i = 0; i < assocFrameCount; i++) {
    if (!assocFrames[i].is7B) continue;
    uint32_t dt = now - assocFrames[i].tsMs;
    if (dt > ASSOC_WINDOW_MS) continue;
    if (!any) {
      Serial.printf("[ASSOC non5544] matched recent 7B frames:\n");
      any = true;
    }
    Serial.printf("  dt=%lums 7Bbar=%.2f raw=", (unsigned long)dt, assocFrames[i].bar);
    for (int b = 0; b < assocFrames[i].len; b++) Serial.printf("%02X", assocFrames[i].bytes[b]);
    Serial.println();
  }
}

static bool isStrongNon5544Candidate(const uint8_t* pkt, int len) {
  if (len < 5) return false;
  // 7B圧力フレームは除外
  if (is7BPressureLikelyCandidate(pkt, len)) return false;
  // 0x55/0xAA/0x00/0xFF 偏重は除外
  int weak = 0;
  for (int i = 0; i < len; i++) {
    uint8_t v = pkt[i];
    if (v == 0x55 || v == 0xAA || v == 0x00 || v == 0xFF || v == 0x44 || v == 0xBB) weak++;
  }
  if (weak >= len - 1) return false;
  return true;
}

// 7B 圧力デコード（2026-04-03 確定式）
// P* = ローリング値 (d[4])、d[6] = P* + Q (mod 256)
// Q = (d[6] - P*) mod 256
// kPa(ゲージ) = 670 - 2.5 * Q
// 7B 圧力デコード（2026-04-03 確定式）
// kPa(ゲージ) = pressConst - 2.5 * Q
// D/A: pressConst=670, F.R B: pressConst=657（Q=191→179kPa実測より）
static PressureDecoded decodePressure7B(uint8_t pStar, uint8_t d6, float pressConst = 670.0f) {
  uint8_t Q = (uint8_t)(d6 - pStar); // mod 256 自動
  PressureDecoded out;
  out.kPa = pressConst - 2.5f * (float)Q;
  // 8bit折り返し補正: kPa >= (pressConst-30) は実際のQが 256+Q の折り返しケース
  // 折り返し後: kPa = (pressConst-640) - 2.5*Q （D/A: threshold=640, F.R B: threshold=627）
  if (out.kPa >= pressConst - 30.0f) out.kPa -= 640.0f;
  if (out.kPa < 0.0f) out.kPa = 0.0f;
  out.bar = out.kPa / 100.0f;
  out.psi = out.kPa / 6.895f;
  return out;
}

// 温度デコード: S = (P* - d[2]) mod 256, T(°C) = S - 38
// D/A センサー: offset=38（複数点検証済み）
// F.R B センサー: offset=45（S=69実測24°Cより）
static int decodeTemp7B(uint8_t pStar, uint8_t d2, int8_t offset = 38) {
  uint8_t S = (uint8_t)(pStar - d2); // mod 256 自動
  return (int)S - (int)offset;
}

// TPMSデータバイト列を解析して圧力・ステータスを表示
// 7Bフォーマット: d[4]=P*(rolling), d[6]=P*+Q(圧力), Q=(d6-P*)%256, kPa=670-2.5*Q
void parseTPMSData(const uint8_t* d, int len) {
  if (len < 3) return;
  Serial.printf("  [PARSE] raw:");
  for (int i = 0; i < len; i++) Serial.printf(" %02X", d[i]);
  Serial.println();

  if (len >= 7) {
    // === 7B 圧力冗長フォーマット ===
    // d[0]=0xF2-P*, d[1]≈d[0], d[2]=P*+~190(冗長), d[3]=0x70-P*,
    // d[4]=P*(rolling,常に奇数), d[5]=0xDE-P*, d[6]=P*+Q(圧力), d[7]=d[0]
    // 圧力式(実測確定): Q=(d[6]-P*)%256, kPa=670-2.5*Q
    int votes7 = 0;
    int si7p = SENSOR_IDX_UNKNOWN;
    uint8_t P7  = (len >= 8) ? correct7BRolling(d, &votes7, &si7p) : d[4];
    if (len < 8) votes7 = 1;  // d[7]なし
    PressureDecoded p7 = decodePressure7B(P7, d[6], getSensorDef(si7p).pressConst);
    uint8_t Q7 = (uint8_t)(d[6] - P7);
    int t7 = decodeTemp7B(P7, d[2], getSensorDef(si7p).tempOffset);
    if (votes7 < MAX_7B_PRESSURE_VOTES) {
      Serial.printf("  [7B] P*=%02X(votes=%d/%d raw_d4=%02X) Q=%u  %.1fPSI=%.0fkPa=%.2fbar T=%d°C\n",
                    P7, votes7, MAX_7B_PRESSURE_VOTES, d[4], Q7, p7.psi, p7.kPa, p7.bar, t7);
    } else {
      Serial.printf("  [7B] P*=%02X Q=%u  %.1fPSI=%.0fkPa=%.2fbar T=%d°C\n", P7, Q7, p7.psi, p7.kPa, p7.bar, t7);
    }

  } else if (len >= 5) {
    // 短いフォーマット（参考表示）
    uint8_t pressure = d[3];
    uint8_t rawTemp  = d[4];
    int     tempC    = (int)rawTemp - 25;
    float   pressBar = pressure / 100.0f;
    float   psi      = pressure / 6.895f;
    bool pressOk = (pressure >= 30 && pressure <= 500);
    bool tempOk  = (tempC >= -40 && tempC <= 100);
    if (!pressOk || !tempOk) {
      Serial.printf("  ID=%02X%02X%02X  P=%d kPa  T=%+d\260C  *** SUSPICIOUS ***\n",
                    d[0],d[1],d[2], pressure, tempC);
      return;
    }
    Serial.printf("  ID=%02X%02X%02X  P=%3d kPa / %.2f bar / %.1f PSI  T=%+d\260C  (3B-ID format)\n",
                  d[0],d[1],d[2], pressure, pressBar, psi, tempC);
  } else if (len == 4) {
    // 4B: IDのみの可能性（CRCが小さい）
    uint32_t sensorId = ((uint32_t)d[0]<<24)|((uint32_t)d[1]<<16)|((uint32_t)d[2]<<8)|d[3];
    // 低エントロピーチェック: 連続する上位ニブルが多すぎる場合は誤検出疎疑
    int ffBits = 0, zeroBits = 0;
    for (int i = 0; i < 4; i++) {
      ffBits   += __builtin_popcount(d[i]);
      zeroBits += __builtin_popcount((uint8_t)~d[i]);
    }
    if (ffBits <= 3 || ffBits >= 29) {
      Serial.printf("  ID=%08lX  *** SUSPICIOUS (low entropy) ***\n", (unsigned long)sensorId);
    } else {
      Serial.printf("  ID=%08lX  (4B: no payload)\n", (unsigned long)sensorId);
    }
  } else if (len == 3) {
    // 3B短縮フォーマット: [temp+11][pressure/25kPa][CRC]
    // 実測確認: d[0]=0x1D(29)=18+11°C, d[1]=0x0D(13)*25=325kPa≈3.3bar
    // Sanity check: d[1]>15 (>375kPa=3.75bar超) は誤検出とみなす
    int      tempC    = (int)d[0] - 11;  // offset=11 (実測より)
    uint16_t pressKPa = (uint16_t)d[1] * 25;  // 25kPa/unit (実測より)
    float    pressBar = pressKPa / 100.0f;
    float    psi      = pressKPa / 6.895f;
    bool pressOk = (d[1] <= 15);   // ≤375kPa (タイヤ最大圧力目安)
    bool tempOk  = (tempC >= -40 && tempC <= 80);
    if (!pressOk || !tempOk) {
      Serial.printf("  d[0]=%d d[1]=%d CRC=%02X  *** SUSPICIOUS (not TPMS: P=%dkPa T=%+dC) ***\n",
                    d[0], d[1], d[2], pressKPa, tempC);
      return;
    }
    Serial.printf("  T=%+d\260C  P=%4d kPa / %.2f bar / %.1f PSI  CRC=%02X  (3B short)\n",
                  tempC, pressKPa, pressBar, psi, d[2]);
    if (pressKPa < 150) Serial.printf("  !! LOW PRESSURE (<1.5 bar)\n");
    if (pressKPa < 100) Serial.printf("  !! FLAT TYRE DETECTED (<1.0 bar)\n");
  } else {
    Serial.printf("  (too short: %d bytes)\n", len);
  }
}

// 繰り返しパターンを検出（バイト単位）
void detectRepeatingPattern(const uint8_t* bits, int nBits) {
  if (nBits < 32) return;
  
  int nBytes = nBits / 8;
  if (nBytes < 4) return;
  
  // バイト配列に変換
  uint8_t bytes[128];
  int maxBytes = min(nBytes, 128);
  for (int i = 0; i < maxBytes; i++) {
    uint8_t v = 0;
    for (int b = 0; b < 8; b++) {
      v = (uint8_t)((v << 1) | (bits[i*8 + b] & 1));
    }
    bytes[i] = v;
  }
  
  // 2バイト繰り返しを探す
  for (int i = 0; i < maxBytes - 3; i++) {
    if (bytes[i] == bytes[i+2] && bytes[i+1] == bytes[i+3]) {
      int repeatCnt = 2;
      for (int j = i+4; j < maxBytes - 1; j+=2) {
        if (bytes[j] == bytes[i] && bytes[j+1] == bytes[i+1]) {
          repeatCnt++;
        } else {
          break;
        }
      }
      if (repeatCnt >= 3) {
        Serial.printf("  REPEAT pattern: %02X %02X (x%d) at byte %d\n", 
                     bytes[i], bytes[i+1], repeatCnt, i);
        return;
      }
    }
  }
  
  // 1バイト繰り返しを探す
  for (int i = 0; i < maxBytes - 2; i++) {
    if (bytes[i] == bytes[i+1] && bytes[i] == bytes[i+2]) {
      int repeatCnt = 3;
      for (int j = i+3; j < maxBytes; j++) {
        if (bytes[j] == bytes[i]) {
          repeatCnt++;
        } else {
          break;
        }
      }
      if (repeatCnt >= 4) {
        Serial.printf("  REPEAT pattern: %02X (x%d) at byte %d\n", 
                     bytes[i], repeatCnt, i);
        return;
      }
    }
  }
}

// dt統計情報を表示（返り値: 平均dt）
int printDtStats(const uint16_t* dts, int n) {
  if (n < 10) return 0;
  
  uint16_t minDt = 65535, maxDt = 0;
  uint32_t sumDt = 0;
  int validCnt = 0;
  
  for (int i = 0; i < n; i++) {
    uint16_t dt = dts[i];
    if (dt > 0 && dt < 2000) {  // 異常値除外
      if (dt < minDt) minDt = dt;
      if (dt > maxDt) maxDt = dt;
      sumDt += dt;
      validCnt++;
    }
  }
  
  int avgDt = 0;
  if (validCnt > 0) {
    avgDt = sumDt / validCnt;
    Serial.printf("  dt(us): min=%u avg=%u max=%u\n", minDt, avgDt, maxDt);
  }
  return avgDt;
}

// ====== Frequency sweep (minimal patch) ======
struct SweepResult {
  float bestFreq;
  int   bestEdgesPerSec;
  int   bestMinDt;
  int   bestMaxDt;
};

void clearBurstState() {
  noInterrupts();
  edgeN = 0;
  burstReady = false;
  lastEdgeUs = micros();
  burstStartUs = lastEdgeUs;
  burstEndUs = lastEdgeUs;
  interrupts();
}

// 1秒だけ観測して edges/s と dt(min/max) を返す
void measureEdges1s(int &edgesPerSec, int &minDt, int &maxDt) {
  clearBurstState();

  uint32_t tStart = millis();
  int startN = 0;
  noInterrupts();
  startN = edgeN;
  interrupts();

  minDt = 999999;
  maxDt = 0;

  // 1秒間、dtのmin/maxを軽く拾う（ISRのdtBufから読むだけ）
  while (millis() - tStart < 1000) {
    int nNow;
    noInterrupts();
    nNow = edgeN;
    interrupts();

    // 追加分だけ見る
    static int lastSeen = 0;
    while (lastSeen < nNow && lastSeen < MAX_EDGES) {
      uint16_t dt = dtBuf[lastSeen];
      if (dt > 0 && dt < minDt) minDt = dt;
      if (dt > maxDt) maxDt = dt;
      lastSeen++;
    }

    delay(10);
  }

  int endN = 0;
  noInterrupts();
  endN = edgeN;
  interrupts();

  edgesPerSec = (endN - startN);

  if (minDt == 999999) minDt = 0;
}

// span=±spanKHz, step=stepKHz で sweep。最良周波数を返す
SweepResult sweepFindBest(float centerMHz, int spanKHz, int stepKHz) {
  SweepResult r;
  r.bestFreq = centerMHz;
  r.bestEdgesPerSec = -1;
  r.bestMinDt = 0;
  r.bestMaxDt = 0;

  int steps = (spanKHz * 2) / stepKHz;
  int nSteps = steps + 1;

  Serial.printf("Sweep: center=%.3f MHz span=±%d kHz step=%d kHz (steps=%d)\n",
                centerMHz, spanKHz, stepKHz, nSteps);

  for (int i = 0; i < nSteps; i++) {
    int offsetKHz = -spanKHz + i * stepKHz;
    float f = centerMHz + (offsetKHz / 1000.0f);

    // ここはあなたの運用に合わせて begin() し直しでOK（最小修正）
    // ※ begin() が重い/不安なら setFrequency 方式に変えるが、まずこれで。
    int st = radio.begin(f, 19.2, 40.0, 200.0);
    radio.setCrcFiltering(false);
    radio.setPromiscuousMode(true, true);
    radio.startReceive();
    ccEnableAsyncOnGDO2();

    int eps, mn, mx;
    measureEdges1s(eps, mn, mx);

    Serial.printf("f=%.3f MHz edges/s=%d dt(us) min=%d max=%d\n", f, eps, mn, mx);

    // スコア：edges/s優先、ただし minDt が 1〜2usみたいなのは雑音の可能性が高いので減点
    // （あなたのログでは min=1〜4usのことが多かったので、閾値は緩めに）
    int score = eps;
    if (mn > 0 && mn < 3) score -= 3000;      // 露骨な超短グリッチを減点
    if (mn == 0) score -= 5000;

    int bestScore = r.bestEdgesPerSec;
    // bestEdgesPerSecにスコアを突っ込むのは気持ち悪いので、ここだけローカルで比較
    static int bestScoreLocal = -1000000;
    if (score > bestScoreLocal) {
      bestScoreLocal = score;
      r.bestFreq = f;
      r.bestEdgesPerSec = eps;
      r.bestMinDt = mn;
      r.bestMaxDt = mx;
    }

    delay(50);
  }

  Serial.printf("SWEEP BEST: f=%.3f MHz edges/s=%d dtmin=%d dtmax=%d\n",
                r.bestFreq, r.bestEdgesPerSec, r.bestMinDt, r.bestMaxDt);

  return r;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // CC1101 専用 SPI バス初期化 (HSPI = SPI3_HOST)
  cc1101Spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  pinMode(PIN_GDO0, INPUT);
  pinMode(PIN_GDO2, INPUT);
  // GDO0は使用しない（配線されていない可能性）
  attachInterrupt(digitalPinToInterrupt(PIN_GDO2), isrGdo2, CHANGE);

  Serial.println("CC1101 burst->Manchester @ 433.92MHz (GDO2=AsyncData only)");

  // 433.92MHz固定で起動
  int st = radio.begin(433.92, 19.2, 40.0, 200.0);
  Serial.printf("radio.begin(433.92MHz)= %d\n", st);
  radio.setCrcFiltering(false);
  radio.setPromiscuousMode(true, true);
  radio.startReceive();
  ccEnableAsyncOnGDO2();

  lastEdgeUs = micros();

  lcdBegin();  // LCD 4分割表示 初期化
}

void loop() {
  static uint32_t t=0;
  static uint32_t hi=0, total=0;
  static uint32_t cntEnd=0, cntTooShort=0, cntDurNG=0, cntParsed=0;
  static uint32_t cntNoSync=0, cntNoCRC=0, cntAvgDtNG=0, cntNoCarrier=0, cntQualNG=0, cntHalfUsNG=0;
  
  // GPIO15統計は削除（不要）
  
  static uint32_t lastStatMs = 0;
  static uint32_t lastKickMs = 0;

  uint32_t nowUs = micros();

  // ---- バースト強制確定条件 ----
  
  // 強制フラッシュ1: バッファ満杯
  if (!burstReady && edgeN >= (MAX_EDGES - 10)) {
    noInterrupts();
    burstReady = true;
    burstEndUs = micros();
    interrupts();
  }

  // 強制フラッシュ2: エッジ記録開始から200ms以上経過
  if (!burstReady && edgeN > 50) {
    if (nowUs - burstStartUs > 200000) {
      noInterrupts();
      burstReady = true;
      burstEndUs = micros();
      interrupts();
    }
  }

  // タイムアウトチェック: 最後のエッジから8ms以上経過してエッジがある場合
  if (!burstReady && edgeN >= MIN_EDGES_TO_PARSE) {
    if (nowUs - lastEdgeUs > BURST_GAP_US) {
      noInterrupts();
      burstReady = true;
      burstEndUs = lastEdgeUs;
      interrupts();
    }
  }

  // ---- 30秒ごとに統計表示 ---- (一時的にオフ)
  if (false && millis() - lastStatMs >= 30000) {
    lastStatMs = millis();
    
    Serial.println("\n=== 30s STATISTICS ===");
    Serial.printf("Total bursts: %lu\n", (unsigned long)cntEnd);
    Serial.printf("  - Duration NG: %lu (too short/long)\n", (unsigned long)cntDurNG);
    Serial.printf("  - Edge count NG: %lu (< %d edges)\n", (unsigned long)cntTooShort, MIN_EDGES_TO_PARSE);
    Serial.printf("  - Avg dt NG: %lu (< 15us)\n", (unsigned long)cntAvgDtNG);
    Serial.printf("  - No carrier (GDO0 LOW): %lu\n", (unsigned long)cntNoCarrier);
    Serial.printf("  - Signal quality NG: %lu (not Manchester-like)\n", (unsigned long)cntQualNG);
    Serial.printf("  - halfUs out of range: %lu (not 18-34us for 9.6-19.2kbps)\n", (unsigned long)cntHalfUsNG);
    Serial.printf("  - Parsed: %lu\n", (unsigned long)cntParsed);
    Serial.printf("    - No SYNC: %lu\n", (unsigned long)cntNoSync);
    Serial.printf("    - No CRC: %lu\n", (unsigned long)cntNoCRC);
    
    // ID統計を表示（単発・古いIDは期限切れで自動削除）
    pruneBurstHistory();
    if (burstHistorySize > 0) {
      Serial.println("\n--- Detected Sensor IDs (16bit) ---");
      for (int i = 0; i < burstHistorySize; i++) {
        uint32_t age = (millis() - burstHistory[i].lastSeen) / 1000;
        const char* status = (burstHistory[i].count >= 2) ? "CONFIRMED" : "count=1";
        Serial.printf("  ID %04lX: count=%d packets [%s] (last %lus ago)\n",
                     (unsigned long)(burstHistory[i].id & 0xFFFF),
                     burstHistory[i].count,
                     status,
                     (unsigned long)age);
      }
    } else {
      Serial.println("  No valid IDs detected yet");
    }
    Serial.println("=====================\n");
    
    // カウンタリセット（burstHistoryはリセットしない）
    cntEnd = 0;
    cntDurNG = 0;
    cntTooShort = 0;
    cntParsed = 0;
    cntNoSync = 0;
    cntNoCRC = 0;
    cntAvgDtNG = 0;
    cntNoCarrier = 0;
    cntQualNG = 0;
    cntHalfUsNG = 0;
  }

  // ---- LCD 定期更新 (200ms) ----
  {
    static uint32_t lastLcdMs = 0;
    if (millis() - lastLcdMs >= 200) {
      lastLcdMs = millis();
      lcdRefresh();
    }
  }

  // ---- BURST確定したら解析＆出力 ----
  if (burstReady) {
    static uint16_t dts[MAX_EDGES];
    static uint8_t  lvs[MAX_EDGES];
    int n;
    uint32_t s, e;

    bool hadCarrier;
    noInterrupts();
    n = edgeN;
    if (n > MAX_EDGES) n = MAX_EDGES;
    for (int i = 0; i < n; i++) { dts[i] = dtBuf[i]; lvs[i] = lvBuf[i]; }
    s = burstStartUs;
    e = burstEndUs;
    hadCarrier = burstHadCarrier;
    edgeN = 0;
    burstReady = false;
    interrupts();

    uint32_t dur = e - s;

    cntEnd++;

    // キャリアセンス状態を記録（非同期モードではGDO0が機能しないため、フィルターは無効）
    if (!hadCarrier) {
      cntNoCarrier++;  // カウントのみ、フィルターはしない
    }

    // ---- dumpNextBurst モード：フラグはリセットするが処理は通常に渡す ----
    if (dumpNextBurst && (millis() - lastPreambleOnlyMs) < 3000) {
      dumpNextBurst = false;
      Serial.printf("  [dumpNextBurst: passing to normal pipeline edges=%d dur=%lums]\n",
                    n, (unsigned long)(dur/1000));
    }
    if (dumpNextBurst) dumpNextBurst = false;  // タイムアウト

    // ★ここで「本物っぽい長さ」だけ通す（5ms〜300ms）
    if (dur < 5000 || dur > 300000) {
      // 短すぎ/長すぎはノイズ扱いで黙殺
      cntDurNG++;
      radio.startReceive();
      return;
    }
    static uint32_t lastBurstPrint = 0;

    if (n < MIN_EDGES_TO_PARSE) {
      cntTooShort++;
      radio.startReceive();
      return;
    }
    cntParsed++;

    // dt平均をチェック（品質チェック）
    int avgDt = 0;
    {
      uint32_t sumDt = 0;
      int validCnt = 0;
      for (int i = 0; i < n; i++) {
        uint16_t dt = dts[i];
        if (dt > 0 && dt < 2000) {
          sumDt += dt;
          validCnt++;
        }
      }
      if (validCnt > 0) avgDt = sumDt / validCnt;
    }
    
    // 平均dtが15us未満はグリッチノイズの可能性が高い
    if (avgDt < 15) {
      cntAvgDtNG++;
      radio.startReceive();
      return;
    }

    float peakFrac = 0.0f;
    int halfUs = estimateHalfBitUs(dts, n, &peakFrac);

    // Manchester信号品質チェック
    // 9.6kbps(halfUs=52)はdtが52/104に分散してpeakFracが低めになるため閾値を分ける
    float pfThresh = (halfUs >= 45) ? 0.30f : 0.55f;
    if (peakFrac < pfThresh) {
      cntQualNG++;
      radio.startReceive();
      return;
    }

    // halfUs範囲チェック：19.2kbpsなら26us、9.6kbpsなら52us
    // 許容範囲: 18〜65us (9.6〜28kbps)
    if (halfUs < 18 || halfUs > 65) {
      cntHalfUsNG++;
      radio.startReceive();
      return;
    }

    static uint8_t halfLv[20000];
    int halfN = expandToHalfbits(dts, lvs, n, halfUs, halfLv, (int)sizeof(halfLv));

    // SYNC直接復号から得た先頭バイト列（ID抽出フォールバック用）
    uint8_t directPktBytes[32] = {};
    int directPktLen = 0;

    // ★ halfLv直接SYNCマッチング（最優先・最も確実な方法）
    // manchesterDecodeを経由しないため位置ズレが起きない
    bool syncDirectInv = false;
    int syncDirectDataStart = findSyncInHalfLv(halfLv, halfN, &syncDirectInv);
    if (syncDirectDataStart >= 0) {
      // SYNCが確実に見つかった → データバイトを正確な位置からデコード
      // サンプリング境界問題対策: オフセット -2〜+2 half-bit を試してCRC OKを優先採用
      static uint8_t syncDirectBits[512];
      static uint8_t sdArrCand[5][32];
      int sdBytesCand[5] = {};
      float invRateCand[5] = {};
      int validPairsCand[5] = {};
      int badPairsCand[5] = {};
      int bestOff = 0, bestScore = -1;
      const int offsets[] = {0, 1, -1, 2, -2};
      for (int oi = 0; oi < 5; oi++) {
        int off = offsets[oi];
        int start = syncDirectDataStart + off;
        if (start < 0 || start >= halfN) continue;

        int llValid = 0, llBad = 0;
        float invRate = 1.0f;
        {
          int pairs = (halfN - start) / 2;
          if (pairs > 160) pairs = 160;
          if (pairs > 0) {
            for (int pi = 0; pi < pairs; pi++) {
              uint8_t a = halfLv[start + pi*2 + 0];
              uint8_t b = halfLv[start + pi*2 + 1];
              if ((a == 0 && b == 1) || (a == 1 && b == 0)) llValid++;
              else llBad++;
            }
            invRate = (float)llBad / (float)(llBad + llValid);
          } else {
            llValid = 0;
            llBad = 0;
            invRate = 1.0f;
          }
        }
        invRateCand[oi] = invRate;
        validPairsCand[oi] = llValid;
        badPairsCand[oi] = llBad;

        int nb = manchesterDecode(halfLv + start, halfN - start,
                                  syncDirectBits, (int)sizeof(syncDirectBits),
                                  syncDirectInv);
        int nb8 = nb / 8;
        if (nb8 > 32) nb8 = 32;
        uint8_t tmp[32] = {};
        for (int b = 0; b < nb8; b++) {
          uint8_t v = 0;
          for (int bit = 0; bit < 8; bit++) v = (v<<1)|(syncDirectBits[b*8+bit]&1);
          tmp[b] = v;
        }
        sdBytesCand[oi] = nb8;
        memcpy(sdArrCand[oi], tmp, nb8);
        // スコア: CRC一致を最優先しつつ、CRC不一致時も7B冗長整合で比較
        // ハード除外ではなく重み付け（ソフト判定）で「拾いつつ誤りを減らす」
        int score = 0;
        if (nb8 >= 8) {
          int votesTmp = 0;
          int siTmp = SENSOR_IDX_UNKNOWN;
          uint8_t pTmp = correct7BRolling(tmp, &votesTmp, &siTmp);
          const SensorDef& sdTmp = getSensorDef(siTmp);
          
          int relOk = 0;
          if (tmp[0] == (uint8_t)(sdTmp.c0 - pTmp)) relOk++;
          if (tmp[1] == (uint8_t)(sdTmp.c0 - pTmp)) relOk++;
          // tmp[2]はd[2]=温度バイトのためスキップ
          if (tmp[3] == (uint8_t)(sdTmp.c3 - pTmp)) relOk++;
          if (tmp[4] == pTmp) relOk++;
          if (tmp[5] == (uint8_t)(sdTmp.c5 - pTmp)) relOk++;
          if (tmp[7] == tmp[0]) relOk++;

          // 基本点
          score += votesTmp * 12;
          score += relOk * 6;

          // d0==d1 は7B構造の強い手掛かり（d[0]=d[1]=0xF2-P* が正しい構造）
          if (tmp[0] == tmp[1]) score += 14;
          else score -= 10;

          // 低品質候補は減点（除外はしない）
          if (votesTmp < 5) score -= 26;
          if (relOk < 6) score -= 20;

          // 圧力現実的範囲チェック: 100〜600kPa(1〜6bar)外は強く減点
          PressureDecoded pTmpDec = decodePressure7B(pTmp, tmp[6], getSensorDef(siTmp).pressConst);
          if (pTmpDec.kPa < 100.0f || pTmpDec.kPa > 600.0f) score -= 120;
        }
        // 低レイヤー品質: invalidRate>0.20 は強く減点、>0.10 は軽減点
        if (invRate > 0.20f) score -= 60;
        else if (invRate > 0.10f) score -= 25;
        else score += 6;

        if (nb8 >= 8) score += 4;  // 同点時は十分長い候補を優先
        if (score > bestScore) { bestScore = score; bestOff = oi; }
      }
      int sdBytes = sdBytesCand[bestOff];
      uint8_t sdArr[32] = {};
      memcpy(sdArr, sdArrCand[bestOff], sdBytes);
      if (sdBytes >= 2) {
        directPktLen = sdBytes;
        if (directPktLen > 32) directPktLen = 32;
        memcpy(directPktBytes, sdArr, directPktLen);
      }
      int usedOff = offsets[bestOff];
      const bool syncQualityOk = (peakFrac >= 0.50f);
      if (syncQualityOk) {
        Serial.printf("[SYNC halfLv=%d inv=%d n=%d avail=%d off=%+d] ",
                      syncDirectDataStart - 32, (int)syncDirectInv,
                      sdBytes, halfN - syncDirectDataStart, usedOff);
        // 7Bパケット部(0..7)のみ表示
        int dispBytes = (sdBytes < 8) ? sdBytes : 8;
        for (int b = 0; b < dispBytes; b++) Serial.printf("%02X ", sdArr[b]);
        // ★ 7Bパケット圧力: votes不足時は値表示しない（ノイズ抑制）
        if (sdBytes >= 7) {
          int votes7 = 0;
          int si7 = SENSOR_IDX_UNKNOWN;
          uint8_t P7 = (sdBytes >= 8) ? correct7BRolling(sdArr, &votes7, &si7) : sdArr[4];
          if (sdBytes < 8) votes7 = 1;
          uint8_t Q7 = (uint8_t)(sdArr[6] - P7);
          PressureDecoded p7s = decodePressure7B(P7, sdArr[6], (si7 >= 0) ? SENSOR_DEFS[si7].pressConst : 670.0f);
          int t7s = decodeTemp7B(P7, sdArr[2], (si7 >= 0) ? SENSOR_DEFS[si7].tempOffset : 38);
          bool p7sRangeOk = (p7s.kPa >= 100.0f && p7s.kPa <= 600.0f);
          // センサーラベル
          const char* sLabel = (si7 >= 0) ? SENSOR_DEFS[si7].label : "?";
          // アラートフラグ判定 (d[1]+P* の値でアラート種別を判定)
          const char* alertStr7 = "";
          if (sdBytes >= 8 && si7 >= 0 && votes7 >= 4) {
            const SensorDef& sd7 = getSensorDef(si7);
            uint8_t d1P7 = (uint8_t)(sdArr[1] + P7);
            if      (d1P7 == sd7.c0)                      alertStr7 = "";           // 定期送信
            else if (d1P7 == (uint8_t)(sd7.c0 - 0x10u))  alertStr7 = " ALT:減圧"; // 減圧アラート
            else if (d1P7 == (uint8_t)(sd7.c0 - 0x30u))  alertStr7 = " ALT:加圧"; // 加圧アラート
            else                                           alertStr7 = " ALT:?";
          }
          if (votes7 >= 4) {
            if (votes7 < MAX_7B_PRESSURE_VOTES) {
              Serial.printf(" [%s] P*=%02X(votes=%d/%d) Q=%u %.1fPSI=%.0fkPa=%.2fbar T=%d°C%s%s",
                            sLabel, P7, votes7, MAX_7B_PRESSURE_VOTES, Q7, p7s.psi, p7s.kPa, p7s.bar, t7s, alertStr7, p7sRangeOk?"":" **P-RANGE!**");
            } else {
              Serial.printf(" [%s] P*=%02X Q=%u %.1fPSI=%.0fkPa=%.2fbar T=%d°C%s%s",
                            sLabel, P7, Q7, p7s.psi, p7s.kPa, p7s.bar, t7s, alertStr7, p7sRangeOk?"":" **P-RANGE!**");
            }
            // LCD 更新 (圧力が現実範囲内 かつ センサー識別済みの場合のみ)
            if (p7sRangeOk && si7 >= 0) {
              lcdUpdateTire(si7, p7s.kPa, p7s.bar, t7s, alertStr7);
            }
          } else {
            Serial.printf(" [%s?] P*=--(votes=%d/%d noisy) d6=%02X",
                          sLabel, votes7, MAX_7B_PRESSURE_VOTES, sdArr[6]);
          }
        }
        Serial.printf(" llq=bad%d/%d(%.2f)",
                badPairsCand[bestOff],
                badPairsCand[bestOff] + validPairsCand[bestOff],
                invRateCand[bestOff]);
        Serial.println();
        // ★ 後続バイト表示 (8バイト目以降)
        if (sdBytes > 8) {
          Serial.printf("[SYNC tail b8..%d] ", sdBytes - 1);
          for (int b = 8; b < sdBytes; b++) Serial.printf("%02X ", sdArr[b]);
          Serial.println();
        }
      }

      // ★ クリーンパケット蓄積: 多数決4票以上 かつ8バイト以上
      // クリーンパケットが3つ溜まったらLIVE CRC探索を実行
      int cleanVotes = 0;
      int cleanSi = SENSOR_IDX_UNKNOWN;
      uint8_t cleanP = (sdBytes >= 8) ? correct7BRolling(sdArr, &cleanVotes, &cleanSi) : 0;
      const char* cleanLabel = (cleanSi >= 0) ? SENSOR_DEFS[cleanSi].label : "?";
      PressureDecoded cleanPDec = decodePressure7B(cleanP, sdArr[6], (cleanSi >= 0) ? SENSOR_DEFS[cleanSi].pressConst : 670.0f);
      bool cleanPRangeOk = (cleanPDec.kPa >= 100.0f && cleanPDec.kPa <= 600.0f);
      if (sdBytes >= 8 && cleanVotes >= 4 && peakFrac >= 0.50f && cleanPRangeOk) {
        // クリーンパケット確認（補正済みP*で格納）
        static uint8_t cleanPkts[8][7];
        static uint32_t cleanTimes[8];  // 受信時刻(millis())
        static int cleanN = 0;
        static uint32_t lastSearchMs = 0;

        // 補正済みP*をsdArr[4]相当として格納用tmpを作成
        uint8_t tmpArr[7];
        for (int b = 0; b < 7; b++) tmpArr[b] = sdArr[b];
        tmpArr[4] = cleanP;  // 補正済みP*

        // 重複チェック（全8バイト一致なら上書き、時刻更新）
        // P*はrolling値なので異なるP*=異なるパケット
        bool dup = false;
        for (int ci = 0; ci < cleanN; ci++) {
          if (cleanPkts[ci][4] == cleanP) {
            for (int b = 0; b < 7; b++) cleanPkts[ci][b] = tmpArr[b];
            cleanTimes[ci] = millis();
            dup = true; break;
          }
        }
        if (!dup) {
          int slot;
          if (cleanN < 8) {
            slot = cleanN++;
          } else {
            // 最古エントリを置き換え (LRU eviction)
            int oldest = 0;
            for (int ci = 1; ci < 8; ci++) {
              if (cleanTimes[ci] < cleanTimes[oldest]) oldest = ci;
            }
            slot = oldest;
          }
          for (int b = 0; b < 7; b++) cleanPkts[slot][b] = tmpArr[b];
          cleanTimes[slot] = millis();
        }
        {
          uint8_t P7c = cleanP;
          uint8_t Q7c = (uint8_t)(sdArr[6] - P7c);
          PressureDecoded p7c = decodePressure7B(P7c, sdArr[6], (cleanSi >= 0) ? SENSOR_DEFS[cleanSi].pressConst : 670.0f);
          int t7c = decodeTemp7B(P7c, sdArr[2], (cleanSi >= 0) ? SENSOR_DEFS[cleanSi].tempOffset : 38);
          // アラートフラグ判定
          const char* alertStr7c = "";
          if (cleanSi >= 0) {
            const SensorDef& sd7c = getSensorDef(cleanSi);
            uint8_t d1Pc = (uint8_t)(sdArr[1] + P7c);
            if      (d1Pc == sd7c.c0)                      alertStr7c = "";
            else if (d1Pc == (uint8_t)(sd7c.c0 - 0x10u))  alertStr7c = " ALT:減圧";
            else if (d1Pc == (uint8_t)(sd7c.c0 - 0x30u))  alertStr7c = " ALT:加圧";
            else                                            alertStr7c = " ALT:?";
          }
          if (cleanVotes < MAX_7B_PRESSURE_VOTES) {
            Serial.printf("[CLEAN PKT] [%s] stored=%d P*=%02X(votes=%d/%d) Q=%u %.1fPSI=%.0fkPa=%.2fbar T=%d°C%s\n",
              cleanLabel, cleanN, P7c, cleanVotes, MAX_7B_PRESSURE_VOTES, Q7c, p7c.psi, p7c.kPa, p7c.bar, t7c, alertStr7c);
          } else {
            Serial.printf("[CLEAN PKT] [%s] stored=%d P*=%02X Q=%u %.1fPSI=%.0fkPa=%.2fbar T=%d°C%s\n",
              cleanLabel, cleanN, P7c, Q7c, p7c.psi, p7c.kPa, p7c.bar, t7c, alertStr7c);
          }
        }

        // 3パケット以上かつ前回探索から5秒以上経過したら探索実行
        // ★ CRC探索無効化: 7Bフレームに CRC なし確定のため
#if 0
        if (cleanN >= 3 && (millis() - lastSearchMs) > 5000) {
          lastSearchMs = millis();
          Serial.printf("=== LIVE CRC search (%d clean pkts) ===\n", cleanN);

          // CRC-8 MSB-first
          auto lCrc8 = [](const uint8_t* d, int len, uint8_t poly, uint8_t init) -> uint8_t {
            uint8_t c = init;
            for (int i = 0; i < len; i++) {
              c ^= d[i];
              for (int b = 0; b < 8; b++)
                c = (c & 0x80) ? (uint8_t)((c << 1) ^ poly) : (uint8_t)(c << 1);
            }
            return c;
          };
          // CRC-8 LSB-first
          auto lCrc8r = [](const uint8_t* d, int len, uint8_t poly, uint8_t init) -> uint8_t {
            uint8_t c = init;
            for (int i = 0; i < len; i++) {
              c ^= d[i];
              for (int b = 0; b < 8; b++)
                c = (c & 1) ? (uint8_t)((c >> 1) ^ poly) : (uint8_t)(c >> 1);
            }
            return c;
          };

          // prefix候補: なし / 0x44 / 0x55,0x44
          // data: d[0..5] または d[2]除外5B, CRC: d[6]
          // poly × init × finalXOR(=fx) の3次元探索
          // fxはcalc^d[6]が全パケットで一定かチェックすることで暗黙的に探索
          struct SearchResult { int cnt; uint8_t poly, init, fx; bool ref; int pfxLen; };
          SearchResult best = {0,0,0,0,false,0};

          uint8_t pfxOptions[3][3] = {{0,0,0},{0x44,0,0},{0x55,0x44,0}};
          int pfxLens[3] = {0,1,2};

          for (int pi = 0; pi < 3; pi++) {
            for (int ref = 0; ref <= 1; ref++) {
              for (int poly = 0; poly <= 255; poly++) {
                for (int init = 0; init <= 255; init++) {
                  uint8_t fxRef = 0; bool first = true; int cnt = 0;
                  for (int pk = 0; pk < cleanN; pk++) {
                    uint8_t buf[10]; int blen = 0;
                    for (int j = 0; j < pfxLens[pi]; j++) buf[blen++] = pfxOptions[pi][j];
                    // d[2]除外(=温度)バリアント: 5B [0,1,3,4,5]
                    static const int pressIdx[] = {0,1,3,4,5};
                    for (int b = 0; b < 5; b++) buf[blen++] = cleanPkts[pk][pressIdx[b]];
                    uint8_t c = ref ? lCrc8r(buf,blen,(uint8_t)poly,(uint8_t)init)
                                    : lCrc8 (buf,blen,(uint8_t)poly,(uint8_t)init);
                    uint8_t fx = c ^ cleanPkts[pk][6];
                    if (first) { fxRef = fx; first = false; cnt = 1; }
                    else if (fx == fxRef) cnt++;
                  }
                  if (cnt > best.cnt) {
                    best = {cnt,(uint8_t)poly,(uint8_t)init,fxRef,(bool)ref,pfxLens[pi]};
                  }
                }
              }
            }
          }
          Serial.printf("  BEST: cnt=%d/%d poly=0x%02X init=0x%02X fx=0x%02X %s pfxLen=%d\n",
                        best.cnt, cleanN, best.poly, best.init, best.fx,
                        best.ref?"LSB":"MSB", best.pfxLen);
          for (int pk = 0; pk < cleanN; pk++) {
            uint8_t buf[10]; int blen = 0;
            for (int j = 0; j < best.pfxLen; j++) buf[blen++] = (best.pfxLen==1)?0x44:(j==0?0x55:0x44);
            for (int b = 0; b < 6; b++) buf[blen++] = cleanPkts[pk][b];
            uint8_t c = best.ref ? lCrc8r(buf,blen,best.poly,best.init)
                                 : lCrc8 (buf,blen,best.poly,best.init);
            uint8_t fx = c ^ cleanPkts[pk][6];
            Serial.printf("    pk%d P*=%02X d6=%02X calc=%02X fx=%02X %s\n",
                          pk, cleanPkts[pk][4], cleanPkts[pk][6], c, fx,
                          (fx==best.fx)?"OK":"NG");
          }
          if (best.cnt == cleanN) Serial.println("  === ALL MATCH! CRC FOUND ===");
          else Serial.println("  (partial/no match)");
          // 圧力表: Q=(d6-P*)%256, kPa=670-2.5*Q （確定式）
          {
            uint32_t nowMs = millis();
            Serial.printf("  >>> kPa = 670 - 2.5*Q, Q=(d6-P*)%%256 / T = S-38, S=(P*-d2)%%256 <<<\n");
            Serial.printf("  pk   age       P*    Q    PSI    kPa   bar   d6  T(C)\n");
            for (int pk = 0; pk < cleanN; pk++) {
              uint8_t P8 = cleanPkts[pk][4];
              uint8_t D8 = cleanPkts[pk][6];
              uint8_t D2 = cleanPkts[pk][2];
              uint8_t Q8 = (uint8_t)(D8 - P8);
              uint32_t ageMs = nowMs - cleanTimes[pk];
              PressureDecoded p8 = decodePressure7B(P8, D8);
              int t8 = decodeTemp7B(P8, D2);
              Serial.printf("  pk%d  %4lus ago  P*=%02X  Q=%3u  %5.1f  %5.0f  %.2f  %02X  %d\n",
                pk, (unsigned long)(ageMs/1000), P8, Q8,
                p8.psi, p8.kPa, p8.bar, D8, t8);
            }
          }
          Serial.println("=== LIVE search done ===");
        }
#endif  // CRC探索無効化
      }
    }

    static uint8_t bits0[8000];
    static uint8_t bits1[8000];
    static uint8_t bits0_shift[8000];
    static uint8_t bits1_shift[8000];
    
    int b0 = manchesterDecode(halfLv, halfN, bits0, (int)sizeof(bits0), false);
    int b1 = manchesterDecode(halfLv, halfN, bits1, (int)sizeof(bits1), true);
    
    // 1半ビットずらして復号（位相補正）
    int b0_shift = 0;
    int b1_shift = 0;
    if (halfN > 2) {
      b0_shift = manchesterDecode(halfLv + 1, halfN - 1, bits0_shift, (int)sizeof(bits0_shift), false);
      b1_shift = manchesterDecode(halfLv + 1, halfN - 1, bits1_shift, (int)sizeof(bits1_shift), true);
    }

    // 拡張同期ワード検出（通常＋シフト）
    SyncResult sync0 = findSyncWord(bits0, b0);
    SyncResult sync1 = findSyncWord(bits1, b1);
    SyncResult sync0_shift = findSyncWord(bits0_shift, b0_shift);
    SyncResult sync1_shift = findSyncWord(bits1_shift, b1_shift);
    
    bool hasSyncWord = (sync0.pos >= 0 || sync1.pos >= 0 || 
                        sync0_shift.pos >= 0 || sync1_shift.pos >= 0);
    
    // SYNCワードベースのCRC検証
    // SYNC WORDがない場合はスキップ（ただし後段でスライディングCRC試行あり）
    if (!hasSyncWord) {
      cntNoSync++;
      // NoSyncバーストも定期的にダンプ（データバーストが見えるか確認）
      static uint32_t lastNoSyncPrint = 0;
      if (millis() - lastNoSyncPrint >= 5000) {
        lastNoSyncPrint = millis();
        Serial.printf("\n[NoSync] pf=%.2f h=%d edges=%d dur=%lums b0=%d b0s=%d\n",
                      peakFrac, halfUs, n, (unsigned long)(dur/1000), b0, b0_shift);
        Serial.printf("  [raw b0 head]: ");
        for (int b = 0; b < 12 && b*8+7 < b0; b++) {
          uint8_t v = 0;
          for (int bit = 0; bit < 8; bit++) v = (v<<1)|(bits0[b*8+bit]&1);
          Serial.printf("%02X ", v);
        }
        Serial.println();
        // halfLv先頭320ビットをダンプ
        Serial.printf("  [halfLv]: ");
        int dispN = (halfN < 320) ? halfN : 320;
        for (int i = 0; i < dispN; i++) {
          Serial.print(halfLv[i] ? '1' : '0');
          if ((i+1) % 8 == 0) Serial.print(' ');
        }
        Serial.println();
      }
      radio.startReceive();
      return;
    }
    
    // ★ CRC検証パイプライン無効化: 7Bフレームに CRC なし確定のため
    //    SYNC ハンドラで7B圧力/温度は処理済み
#if 0
    // 事前CRC検証：有効なデータがあるかチェック（出力なし）
    bool foundValidCRC = false;
    
    // inv=0チェック（4-8バイト）
    if (!foundValidCRC && sync0.pos >= 0 && sync0.pos < b0 - 32) {
      int remainingBits = b0 - sync0.pos;
      for (int tryBytes = 4; tryBytes <= 8; tryBytes++) {  // 4,5,6,7,8全て試す
        if (tryBytes * 8 > remainingBits) continue;
        uint8_t dataBytes[64];
        int nBytes = 0;
        bitsToBytes(bits0 + sync0.pos, tryBytes * 8, dataBytes, &nBytes);
        if (nBytes == tryBytes && verifyCRC(dataBytes, nBytes, true)) {
          foundValidCRC = true;
          break;
        }
      }
    }
    
    // inv=1チェック（4-8バイト）
    if (!foundValidCRC && sync1.pos >= 0 && sync1.pos < b1 - 32) {
      int remainingBits = b1 - sync1.pos;
      for (int tryBytes = 4; tryBytes <= 8; tryBytes++) {
        if (tryBytes * 8 > remainingBits) continue;
        uint8_t dataBytes[64];
        int nBytes = 0;
        bitsToBytes(bits1 + sync1.pos, tryBytes * 8, dataBytes, &nBytes);
        if (nBytes == tryBytes && verifyCRC(dataBytes, nBytes, true)) {
          foundValidCRC = true;
          break;
        }
      }
    }
    
    // inv=0 SHIFTチェック（4-8バイト）
    if (!foundValidCRC && sync0_shift.pos >= 0 && sync0_shift.pos < b0_shift - 32) {
      int remainingBits = b0_shift - sync0_shift.pos;
      for (int tryBytes = 4; tryBytes <= 8; tryBytes++) {
        if (tryBytes * 8 > remainingBits) continue;
        uint8_t dataBytes[64];
        int nBytes = 0;
        bitsToBytes(bits0_shift + sync0_shift.pos, tryBytes * 8, dataBytes, &nBytes);
        if (nBytes == tryBytes && verifyCRC(dataBytes, nBytes, true)) {
          foundValidCRC = true;
          break;
        }
      }
    }
    
    // inv=1 SHIFTチェック（4-8バイト）
    if (!foundValidCRC && sync1_shift.pos >= 0 && sync1_shift.pos < b1_shift - 32) {
      int remainingBits = b1_shift - sync1_shift.pos;
      for (int tryBytes = 4; tryBytes <= 8; tryBytes++) {
        if (tryBytes * 8 > remainingBits) continue;
        uint8_t dataBytes[64];
        int nBytes = 0;
        bitsToBytes(bits1_shift + sync1_shift.pos, tryBytes * 8, dataBytes, &nBytes);
        if (nBytes == tryBytes && verifyCRC(dataBytes, nBytes, true)) {
          foundValidCRC = true;
          break;
        }
      }
    }

    // ペイロードが少ない場合のフォールバック：
    // SYNC位置そのものから数ビット前にデータがある可能性をスライド検索
    // SYNC後のデータが少ない（<4バイト）またはSYNC自体疑わしい場合に使用
    if (!foundValidCRC) {
      // b0全体をスライドしたウィンドウウCRC試行（先頭から最大と50ビット分の間）
      const uint8_t* candidates[4] = {bits0, bits1, bits0_shift, bits1_shift};
      int candBits[4] = {b0, b1, b0_shift, b1_shift};
      for (int ci = 0; ci < 4 && !foundValidCRC; ci++) {
        int maxStart = min(candBits[ci] - 32, 80);  // 開始位置は先頭50ビットまで
        for (int startBit = 0; startBit <= maxStart && !foundValidCRC; startBit += 8) {
          for (int tryBytes = 4; tryBytes <= 8; tryBytes++) {
            if (startBit + tryBytes * 8 > candBits[ci]) break;
            uint8_t dataBytes[8];
            int nBytes = 0;
            bitsToBytes(candidates[ci] + startBit, tryBytes * 8, dataBytes, &nBytes);
            if (nBytes == tryBytes && verifyCRC(dataBytes, nBytes, true)) {
              foundValidCRC = true;
              // NoCRCの待機状態を解除してCRC OKとして出力
              break;
            }
          }
        }
      }
    }

    // データセクション直接検出（9.6kbps境界 = dt > 1.5×halfUs の最初のエッジ以降）
    // halfUs=52(9.6kbps)の場合 threshold=78µs → 104µsエッジを捕捉
    uint8_t dataSectBytes[8] = {};
    int dataSectLen = 0;
    bool dataSectByRepeat = false;  // true=繰り返し検出, false=CRC検証
    if (!foundValidCRC) {
      int nrzTh = (halfUs * 3) / 2;
      int searchFrom2 = n / 3;
      int dsStart = -1;
      for (int i = searchFrom2; i < n; i++) {
        if (dts[i] > (uint16_t)nrzTh) { dsStart = i; break; }
      }
      if (dsStart >= 0) {
        int dsEdges = n - dsStart;
        int dsHalfUs = (halfUs >= 45) ? halfUs : halfUs * 2;
        static uint8_t dsHalfLv[2000];
        int dsHalfN = expandToHalfbits(dts + dsStart, lvs + dsStart,
                                        dsEdges, dsHalfUs, dsHalfLv, (int)sizeof(dsHalfLv));
        for (int off = 0; off <= 1 && !foundValidCRC; off++) {
          if (off >= dsHalfN) break;
          for (int inv = 0; inv <= 1 && !foundValidCRC; inv++) {
            static uint8_t dsBits[512];
            int dsB = manchesterDecode(dsHalfLv + off, dsHalfN - off, dsBits, 512, inv != 0);
            if (dsB < 32) continue;
            int dsByt = dsB / 8;
            if (dsByt > 14) dsByt = 14;
            uint8_t byteArr[16] = {};
            for (int b = 0; b < dsByt; b++)
              for (int bit = 0; bit < 8; bit++)
                byteArr[b] = (byteArr[b]<<1)|(dsBits[b*8+bit]&1);
            // SYNCワード(55 44 / AA BB)が先頭にある場合は2バイト確実にスキップ
            int syncSkip = 0;
            if (dsByt >= 7 &&
                ((byteArr[0]==0x55 && byteArr[1]==0x44) ||
                 (byteArr[0]==0xAA && byteArr[1]==0xBB))) {
              syncSkip = 2;
            }

            // ★ パケット繰り返し検出（double-transmission）
            // センサーが同じパケットを2回連続送信する場合、パケット長4〜8バイトを試す
            // SYNC除去後の先頭4バイトがpktSize後に再出現すれば有効なパケット
            if (!foundValidCRC && syncSkip == 2) {
              for (int pktSize = 4; pktSize <= 8 && !foundValidCRC; pktSize++) {
                if (dsByt < syncSkip + pktSize + 2) continue;  // 2回目最低2バイト必要
                int matchBytes = min(4, pktSize - 1);  // 最大4バイト、パケット長-1まで
                if (matchBytes < 2) continue;
                bool idMatch = true;
                for (int b = 0; b < matchBytes; b++) {
                  if (byteArr[syncSkip + b] != byteArr[syncSkip + pktSize + b]) { idMatch = false; break; }
                }
                if (idMatch) {
                  int aaC = 0;
                  for (int b = 0; b < pktSize; b++) {
                    uint8_t v = byteArr[syncSkip + b];
                    if (v==0xAA||v==0x55||v==0xA5||v==0x5A||v==0xFF||v==0x00||v==0x80) aaC++;
                  }
                  if (aaC < pktSize - 1) {
                    for (int b = 0; b < pktSize; b++) dataSectBytes[b] = byteArr[syncSkip + b];
                    dataSectLen = pktSize;
                    foundValidCRC = true;
                    dataSectByRepeat = true;
                  }
                }
              }
            }

            // さらに残りの0x00/0xFF/0xAA/0x55/0x80を最大2バイトスキップ
            int skipB = syncSkip;
            while (skipB < syncSkip + 2 && skipB < dsByt - 4 &&
                   (byteArr[skipB]==0x00||byteArr[skipB]==0xFF||
                    byteArr[skipB]==0x55||byteArr[skipB]==0xAA||byteArr[skipB]==0x80)) skipB++;
            for (int stB = syncSkip; stB <= skipB && stB + 3 <= dsByt && !foundValidCRC; stB++) {
              for (int tryB = 3; tryB <= 8 && stB + tryB <= dsByt; tryB++) {
                uint8_t raw[8];
                for (int b = 0; b < tryB; b++) raw[b] = byteArr[stB + b];
                if (hasRepeatingPattern(raw, tryB)) continue;
                // rawデータ先頭がSYNCワード(55 44 / AA BB)の場合はスキップ（SYNC混入誤検出防止）
                if (tryB >= 2 && ((raw[0]==0x55 && raw[1]==0x44) || (raw[0]==0xAA && raw[1]==0xBB))) continue;
                // 大部分がpreamble-likeバイトは除外
                int aaC = 0;
                for (int b = 0; b < tryB; b++) {
                  uint8_t v = raw[b];
                  if (v==0xAA||v==0x55||v==0xA5||v==0x5A||v==0xFF||v==0x00||v==0x80||v==0xBB) aaC++;
                }
                if (aaC >= tryB - 1) continue;
                // CRC検証—確認済みポリノミアルのみ
                uint8_t last = raw[tryB-1];
                bool crcOk = (crc8_generic(raw,tryB-1,0x1D,0x00)==last ||
                              crc8_generic(raw,tryB-1,0x07,0xB2)==last);
                // SYNC後半バイト(44/BB)をprefixとしてCRC計算も試す
                if (!crcOk && stB == syncSkip && tryB >= 2) {
                  uint8_t syncTail = (byteArr[0]==0x55) ? 0x44u : 0xBBu;
                  uint8_t rawS[9];
                  rawS[0] = syncTail;
                  for (int b = 0; b < tryB; b++) rawS[1+b] = raw[b];
                  crcOk = (crc8_generic(rawS,tryB,0x1D,0x00)==last ||
                           crc8_generic(rawS,tryB,0x07,0xB2)==last);
                }
                if (crcOk) {
                  for (int b = 0; b < tryB; b++) dataSectBytes[b] = raw[b];
                  dataSectLen = tryB;
                  foundValidCRC = true;
                  break;
                }
              }
            }
          }
        }
      }
    }
    
    // スライディングウィンドウ検証は無効化（誤検出が多すぎるため）
    // SYNCワード位置ベースの検出のみに限定
    
    // 出力判定：CRC成功時のみ表示
    bool shouldPrint = foundValidCRC;
    
    if (!foundValidCRC) {
      cntNoCRC++;

      // CRC不一致デバッグ出力（5秒に1回だけ）
      static uint32_t lastNoCrcPrint = 0;
      if (millis() - lastNoCrcPrint >= 5000) {
        lastNoCrcPrint = millis();
        Serial.printf("\n[NoCRC] peakFrac=%.2f halfUs=%d edges=%d dur=%lums\n",
                      peakFrac, halfUs, n, (unsigned long)(dur/1000));
        Serial.printf("  bits: b0=%d b1=%d b0s=%d b1s=%d\n", b0, b1, b0_shift, b1_shift);
        Serial.printf("  sync: s0=%d s1=%d s0s=%d s1s=%d\n",
          sync0.pos, sync1.pos, sync0_shift.pos, sync1_shift.pos);
        // 全バリアントのバイトとCRCを表示（利用可能な分だけ）
        struct { const uint8_t* bits; int nb; int pos; const char* name; } variants[] = {
          {bits0,      b0,       sync0.pos,       "inv0"},
          {bits1,      b1,       sync1.pos,       "inv1"},
          {bits0_shift,b0_shift, sync0_shift.pos, "inv0s"},
          {bits1_shift,b1_shift, sync1_shift.pos, "inv1s"},
        };
        for (int vi = 0; vi < 4; vi++) {
          if (variants[vi].pos < 0) continue;
          int sp = variants[vi].pos;
          int nb = variants[vi].nb;
          int avail = (nb - sp) / 8;
          if (avail <= 0) continue;
          if (avail > 12) avail = 12;
          Serial.printf("  [%s] pos=%d avail=%dB: ", variants[vi].name, sp, avail);
          for (int b = 0; b < avail; b++) {
            if (sp + b*8 + 7 >= nb) break;
            uint8_t v = 0;
            for (int bit = 0; bit < 8; bit++) v = (v<<1) | (variants[vi].bits[sp+b*8+bit]&1);
            Serial.printf("%02X ", v);
          }
          Serial.println();
          // 各長さのCRC計算結果を表示
          for (int tryB = 4; tryB <= 8 && tryB <= avail; tryB++) {
            uint8_t raw[8];
            bool ok = true;
            for (int b = 0; b < tryB; b++) {
              if (sp + b*8 + 7 >= nb) { ok = false; break; }
              raw[b] = 0;
              for (int bit = 0; bit < 8; bit++) raw[b] = (raw[b]<<1)|(variants[vi].bits[sp+b*8+bit]&1);
            }
            if (!ok) break;
            uint8_t last = raw[tryB-1];
            uint8_t c07_00 = crc8_generic(raw,tryB-1,0x07,0x00);
            uint8_t c07_b2 = crc8_generic(raw,tryB-1,0x07,0xB2);
            uint8_t c07_ff = crc8_generic(raw,tryB-1,0x07,0xFF);
            uint8_t c1d_00 = crc8_generic(raw,tryB-1,0x1D,0x00);
            uint8_t c31_ff = crc8_generic(raw,tryB-1,0x31,0xFF);
            uint8_t xorS = 0; for(int b=0;b<tryB-1;b++) xorS^=raw[b];
            Serial.printf("    len=%d last=%02X 07/00=%02X 07/B2=%02X 07/FF=%02X 1D/00=%02X 31/FF=%02X xor=%02X\n",
              tryB, last, c07_00, c07_b2, c07_ff, c1d_00, c31_ff, xorS);
          }
        }
        // SYNCの後にデータが少ない場合: b0全体のraw先頭バイトも表示
        Serial.printf("  [raw b0 head]: ");
        for (int b = 0; b < 12 && b*8+7 < b0; b++) {
          uint8_t v = 0;
          for (int bit = 0; bit < 8; bit++) v = (v<<1)|(bits0[b*8+bit]&1);
          Serial.printf("%02X ", v);
        }
        Serial.println();
        // halfLv生データ（最初の320ビット）でプリアンブル終端を確認
        Serial.printf("  [halfLv 0..319]: ");
        int dispN = (halfN < 320) ? halfN : 320;
        for (int i = 0; i < dispN; i++) {
          Serial.print(halfLv[i] ? '1' : '0');
          if ((i+1) % 8 == 0) Serial.print(' ');
        }
        Serial.println();
        // halfLvのどこで「不正ペア」が増えるか（プリアンブル終端の手がかり）
        int firstBadPair = -1;
        for (int i = 0; i < halfN - 1; i += 2) {
          if (!(halfLv[i] == 0 && halfLv[i+1] == 1) &&
              !(halfLv[i] == 1 && halfLv[i+1] == 0)) {
            firstBadPair = i;
            break;
          }
        }
        Serial.printf("  first invalid Manchester pair at halfLv[%d]\n", firstBadPair);

        // halfLv内のパターン変化点（プリアンブル→データ境界）を検出
        // 先頭グリッチを回避するため、halfN/2 以降から探索する
        int transitionPos = -1;
        {
          // プリアンブルは通常 60bit以上 -> 120 halfbit以上。少なくともhalfN/2以降から
          int searchStart = (halfN > 80) ? halfN / 2 : 40;
          int consInvalid = 0;
          for (int i = searchStart; i < halfN - 3 && i < (halfN - 4); i += 2) {
            bool valid = (halfLv[i]==0 && halfLv[i+1]==1) || (halfLv[i]==1 && halfLv[i+1]==0);
            if (!valid) {
              consInvalid++;
              if (consInvalid >= 2) { transitionPos = i - 2; break; }
            } else {
              consInvalid = 0;
            }
          }
        }
        Serial.printf("  pattern transition at halfLv[%d] (halfN=%d)\n", transitionPos, halfN);

        // プリアンブル終端以降のテール部をManchester両位相・両向きでデコード
        if (transitionPos > 8 && transitionPos < halfN - 16) {
          // offset 0,1,2,3 の各位相でデコード試行
          for (int startOff = 0; startOff <= 3; startOff++) {
            int sp = transitionPos + startOff;
            if (sp >= halfN) break;
            for (int inv = 0; inv <= 1; inv++) {
              static uint8_t tailBits[512];
              int tb = manchesterDecode(halfLv + sp, halfN - sp, tailBits, 512, inv != 0);
              if (tb < 8) continue;
              int tb_bytes = tb / 8;
              if (tb_bytes > 10) tb_bytes = 10;
              Serial.printf("  [tail @%d inv=%d %dbit]: ", sp, inv, tb);
              for (int b = 0; b < tb_bytes; b++) {
                uint8_t v = 0;
                for (int bit = 0; bit < 8; bit++) v = (v<<1)|(tailBits[b*8+bit]&1);
                Serial.printf("%02X ", v);
              }
              // CRC試行（テール先頭から4-8バイト）
              for (int tryB = 4; tryB <= 8 && tryB*8 <= tb; tryB++) {
                uint8_t raw[8]; for(int b=0;b<tryB;b++){raw[b]=0;for(int bit=0;bit<8;bit++)raw[b]=(raw[b]<<1)|(tailBits[b*8+bit]&1);}
                // false positive排除
                int aaC=0; for(int b=0;b<tryB;b++){uint8_t v=raw[b];if(v==0xAA||v==0x55||v==0xA5||v==0x5A||v==0xFF||v==0x00)aaC++;}
                if (aaC >= tryB - 1) continue;
                uint8_t c07_00=crc8_generic(raw,tryB-1,0x07,0x00);
                uint8_t c07_b2=crc8_generic(raw,tryB-1,0x07,0xB2);
                uint8_t xorS=0; for(int b=0;b<tryB-1;b++) xorS^=raw[b];
                if (c07_00==raw[tryB-1] || c07_b2==raw[tryB-1] || xorS==raw[tryB-1])
                  Serial.printf(" *** CRC HIT len=%d 07/00=%02X b2=%02X xor=%02X ***",tryB,c07_00,c07_b2,xorS);
              }
              Serial.println();
            }
          }
        }

        // 9.6kbps試行（再展開）：halfUs>=45は既に9.6kbps、halfUs<45はpreamble=19.2kbpsでdata=9.6kbps対応
        {
          int halfUs2 = (halfUs >= 45) ? halfUs : halfUs * 2;
          static uint8_t halfLv2[8000];
          int halfN2 = expandToHalfbits(dts, lvs, n, halfUs2, halfLv2, (int)sizeof(halfLv2));
          Serial.printf("  [9.6kbps halfN=%d]: ", halfN2);
          int dispN2 = (halfN2 < 160) ? halfN2 : 160;
          for (int i = 0; i < dispN2; i++) {
            Serial.print(halfLv2[i] ? '1' : '0');
            if ((i+1) % 8 == 0) Serial.print(' ');
          }
          Serial.println();
          // 各位置からデコード
          for (int startOff = 0; startOff <= 1; startOff++) {
            for (int inv = 0; inv <= 1; inv++) {
              static uint8_t bits9k[512];
              int b9 = manchesterDecode(halfLv2 + startOff, halfN2 - startOff, bits9k, 512, inv != 0);
              if (b9 < 16) continue;
              Serial.printf("  [9.6k off=%d inv=%d %dbit]: ", startOff, inv, b9);
              int b9_bytes = b9 / 8; if (b9_bytes > 12) b9_bytes = 12;
              for (int b = 0; b < b9_bytes; b++) {
                uint8_t v = 0;
                for (int bit = 0; bit < 8; bit++) v = (v<<1)|(bits9k[b*8+bit]&1);
                Serial.printf("%02X ", v);
              }
              // CRC試行
              for (int tryB = 4; tryB <= 8 && tryB*8 <= b9; tryB++) {
                uint8_t raw[8]; for(int b=0;b<tryB;b++){raw[b]=0;for(int bit=0;bit<8;bit++)raw[b]=(raw[b]<<1)|(bits9k[b*8+bit]&1);}
                uint8_t c07_00=crc8_generic(raw,tryB-1,0x07,0x00);
                uint8_t c07_b2=crc8_generic(raw,tryB-1,0x07,0xB2);
                uint8_t xorS=0; for(int b=0;b<tryB-1;b++) xorS^=raw[b];
                if (c07_00==raw[tryB-1] || c07_b2==raw[tryB-1] || xorS==raw[tryB-1])
                  Serial.printf(" *** CRC HIT len=%d ***",tryB);
              }
              Serial.println();
            }
          }
        }

        // 末尾40エッジのdt+lv値（データ部タイミング確認）
        {
          int dtStart = (n > 40) ? n - 40 : 0;
          Serial.printf("  [dt+lv tail %d..%d]:\n", dtStart, n-1);
          for (int i = dtStart; i < n; i++) {
            Serial.printf("    [%d] dt=%u lv=%d\n", i, dts[i], lvs[i]);
          }
        }

        // データ境界をdt値から検出（>1.5×halfUs が最初に出るエッジ）
        // halfUs=52(9.6kbps): preamble dt=52µs, data dt=104µs → threshold=78µs
        // halfUs=26(19.2kbps): preamble dt=26µs, data dt=52µs → threshold=39µs
        {
          int nrzThreshold = (halfUs * 3) / 2;  // 78µs for halfUs=52, 39µs for halfUs=26
          // n/3 以降から探索（プリアンブルが最低1/3はある想定）
          int dataStart = -1;
          int searchFrom = n / 3;
          for (int i = searchFrom; i < n; i++) {
            if (dts[i] > (uint16_t)nrzThreshold) { dataStart = i; break; }
          }

          if (dataStart >= 0) {
            int dataEdges = n - dataStart;
            // 展開halfUs: halfUs>=45(9.6kbps)→そのまま, halfUs<45(19.2kbps)→*2で9.6kbps相当
            int dataHalfUsExp = (halfUs >= 45) ? halfUs : halfUs * 2;
            Serial.printf("  Data section starts at edge %d (%d edges), halfUs=%d\n",
                          dataStart, dataEdges, dataHalfUsExp);
            // dt+lv表示（dataStart付近）
            int dumpFrom = (dataStart > 2) ? dataStart - 2 : 0;
            for (int i = dumpFrom; i < n && i < dumpFrom + 20; i++) {
              Serial.printf("    [%d] dt=%u lv=%d\n", i, dts[i], lvs[i]);
            }
            // dataStart以降をdataHalfUsExpでManchester展開してデコード
            static uint8_t dataHalfLv[4000];
            int dataHalfN = expandToHalfbits(dts + dataStart, lvs + dataStart,
                                              dataEdges, dataHalfUsExp,
                                              dataHalfLv, (int)sizeof(dataHalfLv));
            Serial.printf("  [data halfLv (halfUs=%d) N=%d]: ", dataHalfUsExp, dataHalfN);
            int dN2 = (dataHalfN < 128) ? dataHalfN : 128;
            for (int i = 0; i < dN2; i++) {
              Serial.print(dataHalfLv[i] ? '1' : '0');
              if ((i+1) % 8 == 0) Serial.print(' ');
            }
            Serial.println();
            // 両位相(offset 0,1) × 両極性(inv 0,1) でManchesterデコード→CRC試行
            for (int startOff = 0; startOff <= 1; startOff++) {
              if (startOff >= dataHalfN) break;
              for (int inv = 0; inv <= 1; inv++) {
                static uint8_t dataBits[512];
                int db = manchesterDecode(dataHalfLv + startOff,
                                          dataHalfN - startOff,
                                          dataBits, 512, inv != 0);
                if (db < 16) continue;
                int dbBytes = db / 8;
                if (dbBytes > 14) dbBytes = 14;
                Serial.printf("  [data Manchester off=%d inv=%d %dbit]: ", startOff, inv, db);
                uint8_t byteArr[16] = {};
                for (int b = 0; b < dbBytes; b++) {
                  byteArr[b] = 0;
                  for (int bit = 0; bit < 8; bit++) byteArr[b] = (byteArr[b]<<1)|(dataBits[b*8+bit]&1);
                  Serial.printf("%02X ", byteArr[b]);
                }
                // CRC試行（先頭から最大8バイト、false positive排除）
                bool anyHit = false;
                // 先頭に0x00/0xFF/0x55/0xAA が続く場合はスキップオフセットを探す
                int skipBytes = 0;
                while (skipBytes < dbBytes - 4 &&
                       (byteArr[skipBytes]==0x00||byteArr[skipBytes]==0xFF||
                        byteArr[skipBytes]==0x55||byteArr[skipBytes]==0xAA||
                        byteArr[skipBytes]==0x80)) skipBytes++;
                for (int startB = 0; startB <= skipBytes && startB + 4 <= dbBytes; startB++) {
                  for (int tryB = 4; tryB <= 8 && startB + tryB <= dbBytes; tryB++) {
                    uint8_t raw[8];
                    for (int b = 0; b < tryB; b++) raw[b] = byteArr[startB + b];
                    // false positive排除
                    int aaC = 0;
                    for (int b = 0; b < tryB; b++) {
                      uint8_t v = raw[b];
                      if (v==0xAA||v==0x55||v==0xA5||v==0x5A||v==0xFF||v==0x00||v==0x80) aaC++;
                    }
                    if (aaC >= tryB - 1) continue;
                    uint8_t c07_00=crc8_generic(raw,tryB-1,0x07,0x00);
                    uint8_t c07_b2=crc8_generic(raw,tryB-1,0x07,0xB2);
                    uint8_t c07_ff=crc8_generic(raw,tryB-1,0x07,0xFF);
                    uint8_t c1d_00=crc8_generic(raw,tryB-1,0x1D,0x00);
                    uint8_t c31_ff=crc8_generic(raw,tryB-1,0x31,0xFF);
                    uint8_t xorS=0; for(int b=0;b<tryB-1;b++) xorS^=raw[b];
                    uint8_t last=raw[tryB-1];
                    if (c07_00==last||c07_b2==last||c07_ff==last||c1d_00==last||c31_ff==last||xorS==last) {
                      Serial.printf("\n    *** CRC HIT skip=%d len=%d 07/00=%02X b2=%02X ff=%02X 1d=%02X 31=%02X xor=%02X ***",
                                    startB, tryB, c07_00, c07_b2, c07_ff, c1d_00, c31_ff, xorS);
                      anyHit = true;
                    }
                  }
                }
                // ★ 55 44 プレフィックスを含むパケットに対して全ポリノミアル探索
                // byteArr[0]==55, byteArr[1]==44 のとき、[44, data...] でCRC全探索
                if (!anyHit &&
                    dbBytes >= 5 &&
                    byteArr[0]==0x55 && byteArr[1]==0x44) {
                  // 2〜7データバイト（末尾がCRC）試す
                  for (int pLen = 2; pLen <= 7 && (2 + pLen) <= dbBytes; pLen++) {
                    uint8_t dataWithSyncTail[8];
                    dataWithSyncTail[0] = 0x44;  // SYNC後半バイト
                    for (int b = 0; b < pLen; b++) dataWithSyncTail[1+b] = byteArr[2+b];
                    uint8_t wantCRC = dataWithSyncTail[pLen];  // 末尾バイト
                    // preamble-like でないことを確認
                    int aaC2 = 0;
                    for (int b = 1; b < pLen; b++) {
                      uint8_t v = dataWithSyncTail[b];
                      if (v==0xAA||v==0x55||v==0xA5||v==0x5A||v==0xFF||v==0x00||v==0x80) aaC2++;
                    }
                    if (aaC2 >= pLen - 1) continue;
                    // 全256ポリノミアル × {0x00, 0xFF} 探索（全一致を表示）
                    // 2バイトデータでは偶然一致が多いが、複数パケット共通ならば正解
                    for (int poly = 0; poly <= 255; poly++) {
                      for (int initIdx = 0; initIdx <= 1; initIdx++) {
                        uint8_t initVal = initIdx ? 0xFF : 0x00;
                        uint8_t c = crc8_generic(dataWithSyncTail, pLen, (uint8_t)poly, initVal);
                        if (c == wantCRC) {
                          Serial.printf("\n  *** POLY FOUND: 0x%02X init=0x%02X pktLen=%d CRC=%02X [44",
                                        poly, initVal, pLen, wantCRC);
                          for (int b = 1; b <= pLen; b++) Serial.printf(" %02X", dataWithSyncTail[b]);
                          Serial.printf("] ***");
                          anyHit = true;
                          // anyHitは残すが全ポリを走査し続ける（!anyHit条件を外した）
                        }
                      }
                    }
                  }
                }
                // ★ direct mode POLY FOUND (SYNCプレフィックスなし、データ列を直接CRC探索)
                // byteArr[0..1]=55 44 の後ろのデータ byteArr[2..] を対象に
                // crc8(data[0..n-2], poly, init) == data[n-1] を全ポリで探索
                if (dbBytes >= 5 &&
                    byteArr[0]==0x55 && byteArr[1]==0x44) {
                  uint8_t* dPtr = byteArr + 2;
                  int dAvail = dbBytes - 2;
                  for (int pLen = 3; pLen <= 7 && pLen <= dAvail; pLen++) {
                    uint8_t wCRC = dPtr[pLen - 1];
                    // preamble-like チェック（大半がAA/55系なら偶然一致として除外）
                    int aaC = 0;
                    for (int b = 0; b < pLen - 1; b++) {
                      uint8_t v = dPtr[b];
                      if (v==0xAA||v==0x55||v==0xA5||v==0x5A||v==0xFF||v==0x00||v==0x80) aaC++;
                    }
                    if (aaC >= pLen - 1) continue;
                    for (int poly = 0; poly <= 255; poly++) {
                      for (int initIdx = 0; initIdx <= 1; initIdx++) {
                        uint8_t initVal = initIdx ? 0xFF : 0x00;
                        uint8_t c = crc8_generic(dPtr, pLen - 1, (uint8_t)poly, initVal);
                        if (c == wCRC) {
                          Serial.printf("\n  *** POLY FOUND DIRECT: 0x%02X init=0x%02X pktLen=%d CRC=%02X [",
                                        poly, initVal, pLen, wCRC);
                          for (int b = 0; b < pLen; b++) Serial.printf("%02X ", dPtr[b]);
                          Serial.printf("] ***");
                        }
                      }
                    }
                  }
                }
                Serial.println();
              }
            }
          } else {
            Serial.printf("  Data section not found (all dt[%d..] <= %d, halfUs=%d)\n", searchFrom, nrzThreshold, halfUs);
          }
        }

        // プリアンブルのみ（大半が0xAA/0x55）なら次バーストを強制ダンプ
        {
          int aaByteCnt = 0;
          int totalBytes = b0 / 8;
          if (totalBytes > 0) {
            for (int b = 0; b < totalBytes && b < 20; b++) {
              uint8_t v = 0;
              for (int bit = 0; bit < 8; bit++) v = (v<<1)|(bits0[b*8+bit]&1);
              if (v == 0xAA || v == 0x55 || v == 0x6A || v == 0xD5) aaByteCnt++;
            }
            int checkBytes = (totalBytes < 20) ? totalBytes : 20;
            if (aaByteCnt >= checkBytes * 7 / 8) {
              dumpNextBurst = true;
              lastPreambleOnlyMs = millis();
              Serial.println("  >> Preamble-only burst! dumpNextBurst=true");
            }
          }
        }
      }
    }
    
    if (shouldPrint) {
      // foundValidCRCはすでに判定済み。ここでは詳細な出力とrecordBurst実行
      bool recordedID = false;
      bool confirmedID = false;
      const bool burstHas7BMain = is7BPressureLikelyCandidate(directPktBytes, directPktLen);

      // データセクション直接デコード結果を優先使用
      // 7B圧力フレームでは先頭2BはIDではなく、圧力P由来の冗長符号
      if (!recordedID && !burstHas7BMain && !is7BPressureLikelyCandidate(dataSectBytes, dataSectLen) && dataSectLen >= 2) {
        const bool strongNon5544 = isStrongNon5544Candidate(dataSectBytes, dataSectLen);
        if (strongNon5544) {
          uint8_t idBytes[2] = {dataSectBytes[0], dataSectBytes[1]};
          confirmedID = recordBurst(idBytes, 2);
          recordedID = true;
          const char* detLabel = dataSectByRepeat ? "repeat" : "CRC OK";
          Serial.printf("[%s ID=%02X%02X %s] ", detLabel, dataSectBytes[0], dataSectBytes[1],
                        confirmedID ? "CONFIRMED" : "count=1");
        } else {
          Serial.printf("[NON5544 candidate] ");
        }
        for (int i = 0; i < dataSectLen; i++) Serial.printf("%02X", dataSectBytes[i]);
        Serial.printf(" pf=%.2f h=%d\n", peakFrac, halfUs);
        if (dataSectLen >= 3) {
          pushAssocFrame(false, dataSectBytes, dataSectLen, -1.0f);
          printAssocMatchesForNon7B(dataSectBytes, dataSectLen);
        }
        if (confirmedID) {
          Serial.printf("==== TPMS CONFIRMED (ID=%02X%02X) ====\n",
                        dataSectBytes[0], dataSectBytes[1]);
        }
      }

      // dataSect抽出が失敗しても、7B圧力以外のフレームのみID候補として記録
      if (!recordedID && !burstHas7BMain && !is7BPressureLikelyCandidate(directPktBytes, directPktLen) && directPktLen >= 2) {
        const bool strongNon5544 = isStrongNon5544Candidate(directPktBytes, directPktLen);
        if (strongNon5544) {
          uint8_t idBytes[2] = {directPktBytes[0], directPktBytes[1]};
          confirmedID = recordBurst(idBytes, 2);
          recordedID = true;
          Serial.printf("[SYNC ID=%02X%02X %s] ", idBytes[0], idBytes[1],
                        confirmedID ? "CONFIRMED" : "count=1");
        } else {
          Serial.printf("[SYNC non5544 candidate] ");
        }
        for (int i = 0; i < directPktLen && i < 8; i++) Serial.printf("%02X", directPktBytes[i]);
        Serial.printf(" pf=%.2f h=%d\n", peakFrac, halfUs);
        if (directPktLen >= 3) {
          pushAssocFrame(false, directPktBytes, directPktLen, -1.0f);
          printAssocMatchesForNon7B(directPktBytes, directPktLen);
        }
      }

      // 7B圧力フレームはIDなし扱い（先頭2BはP*由来）
      if (!recordedID && is7BPressureFrameCandidate(directPktBytes, directPktLen)) {
        int votes7 = 0;
        uint8_t P7 = 0;
        (void)is7BPressureFrameCandidate(directPktBytes, directPktLen, &votes7, &P7);
        int siTmp7 = SENSOR_IDX_UNKNOWN; int vTmp7 = 0;
        correct7BRolling(directPktBytes, &vTmp7, &siTmp7);
        PressureDecoded p7 = decodePressure7B(P7, directPktBytes[6], getSensorDef(siTmp7).pressConst);
        Serial.printf("[SYNC 7B-noID] %02X%02X... P*=%02X(v=%d/%d) %.2fbar pf=%.2f h=%d\n",
                directPktBytes[0], directPktBytes[1], P7, votes7, MAX_7B_PRESSURE_VOTES, p7.bar, peakFrac, halfUs);
        pushAssocFrame(true, directPktBytes, directPktLen, p7.bar);
        printAssocCandidatesFor7B(directPktBytes, directPktLen, p7.bar);
      }

      if (!recordedID && is7BPressureFragmentCandidate(directPktBytes, directPktLen)) {
        uint8_t P7 = 0;
        (void)is7BPressureFragmentCandidate(directPktBytes, directPktLen, &P7);
        // フラグメントはd[6]がない場合があるので圧力は参考値
        int siFragTmp = SENSOR_IDX_UNKNOWN; int vFragTmp = 0;
        correct7BRolling(directPktBytes, &vFragTmp, &siFragTmp);
        PressureDecoded p7 = (directPktLen >= 7) ? decodePressure7B(P7, directPktBytes[6], getSensorDef(siFragTmp).pressConst) : PressureDecoded{0,0,0};
        Serial.printf("[SYNC 7B-frag] %02X%02X... P*=%02X %.2fbar pf=%.2f h=%d\n",
                      directPktBytes[0], directPktBytes[1], P7, p7.bar, peakFrac, halfUs);
      }

      if (!recordedID && is7BPressurePrefixCandidate(directPktBytes, directPktLen)) {
        uint8_t P7 = 0;
        (void)is7BPressurePrefixCandidate(directPktBytes, directPktLen, &P7);
        // prefix(4B)にはd[6]がないので圧力計算不可
        Serial.printf("[SYNC 7B-prefix] %02X%02X%02X%02X P*~%02X pf=%.2f h=%d\n",
                      directPktBytes[0], directPktBytes[1], directPktBytes[2], directPktBytes[3],
                      P7, peakFrac, halfUs);
      }
      
#if 0  // inv=0 未確認パス: 他センサー調査時に有効化
      // inv=0のデータ抽出（3-8バイト）
      if (!recordedID && sync0.pos >= 0 && sync0.pos < b0 - 24) {
        int remainingBits = b0 - sync0.pos;
        
        // 4,5,6,7,8バイト全て試す
        for (int tryBytes = 3; tryBytes <= 8; tryBytes++) {
          if (tryBytes * 8 > remainingBits) continue;
          uint8_t dataBytes[64];
          int nBytes = 0;
          bitsToBytes(bits0 + sync0.pos, tryBytes * 8, dataBytes, &nBytes);
          
          if (nBytes != tryBytes) continue;
          
          // CRC検証
          if (verifyCRC(dataBytes, nBytes, true)) {  // silentでCRC種別は後で表示
            if (!recordedID) {
              confirmedID = recordBurst(dataBytes, nBytes);
              recordedID = true;
              // 1回目でもIDを表示（デバッグ用）
              Serial.printf("[CRC OK %s] ", confirmedID ? "CONFIRMED" : "count=1");
              for (int i = 0; i < nBytes; i++) Serial.printf("%02X", dataBytes[i]);
              Serial.printf(" pf=%.2f h=%d\n", peakFrac, halfUs);
              if (confirmedID) {
                Serial.printf("==== TPMS CONFIRMED ====\n");
                parseTPMSData(dataBytes, nBytes);
              }
            }
            break;
          }
        }
      }
      
#endif  // inv=0

#if 0  // inv=1 未確認パス: 他センサー調査時に有効化
      // inv=1のデータ抽出
      if (!recordedID && sync1.pos >= 0 && sync1.pos < b1 - 24) {
        int remainingBits = b1 - sync1.pos;
        for (int tryBytes = 3; tryBytes <= 8; tryBytes++) {
          if (tryBytes * 8 > remainingBits) continue;
          uint8_t dataBytes[64];
          int nBytes = 0;
          bitsToBytes(bits1 + sync1.pos, tryBytes * 8, dataBytes, &nBytes);
          if (nBytes != tryBytes) continue;
          if (verifyCRC(dataBytes, nBytes, true)) {
            if (!recordedID) {
              confirmedID = recordBurst(dataBytes, nBytes);
              recordedID = true;
              Serial.printf("[CRC OK inv1 %s] ", confirmedID ? "CONFIRMED" : "count=1");
              for (int i = 0; i < nBytes; i++) Serial.printf("%02X", dataBytes[i]);
              Serial.printf(" pf=%.2f h=%d\n", peakFrac, halfUs);
              if (confirmedID) {
                Serial.printf("==== TPMS CONFIRMED ====\n");
                parseTPMSData(dataBytes, nBytes);
              }
            }
            break;
          }
        }
      }
      
#endif  // inv=1

      // inv=0 SHIFT版のデータ抽出 ★実測確認済み (1D0D9D)
      if (!recordedID && !burstHas7BMain && sync0_shift.pos >= 0 && sync0_shift.pos < b0_shift - 24) {
        int remainingBits = b0_shift - sync0_shift.pos;
        for (int tryBytes = 3; tryBytes <= 8; tryBytes++) {
          if (tryBytes * 8 > remainingBits) continue;
          uint8_t dataBytes[64];
          int nBytes = 0;
          bitsToBytes(bits0_shift + sync0_shift.pos, tryBytes * 8, dataBytes, &nBytes);
          if (nBytes != tryBytes) continue;
          if (verifyCRC(dataBytes, nBytes, true)) {
            if (!recordedID) {
              bool is7BLike = is7BPressureFrameCandidate(dataBytes, nBytes) ||
                              is7BPressureFragmentCandidate(dataBytes, nBytes) ||
                              is7BPressurePrefixCandidate(dataBytes, nBytes);
              bool strongNon5544 = isStrongNon5544Candidate(dataBytes, nBytes);
              if (is7BLike) {
                uint8_t pTmp = 0;
                (void)is7BPressureFragmentCandidate(dataBytes, nBytes, &pTmp);
                int siD = SENSOR_IDX_UNKNOWN; int vD = 0;
                correct7BRolling(dataBytes, &vD, &siD);
                PressureDecoded pD = (nBytes >= 7) ? decodePressure7B(pTmp, dataBytes[6], getSensorDef(siD).pressConst) : PressureDecoded{0,0,0};
                Serial.printf("[CRC OK inv0s 7B-frag] ");
                for (int i = 0; i < nBytes; i++) Serial.printf("%02X", dataBytes[i]);
                Serial.printf(" P*=%02X %.2fbar pf=%.2f h=%d\n", pTmp, pD.bar, peakFrac, halfUs);
                break;
              }
              if (!strongNon5544) {
                Serial.printf("[CRC OK inv0s candidate] ");
                for (int i = 0; i < nBytes; i++) Serial.printf("%02X", dataBytes[i]);
                Serial.printf(" pf=%.2f h=%d\n", peakFrac, halfUs);
                if (nBytes >= 3) {
                  pushAssocFrame(false, dataBytes, nBytes, -1.0f);
                  printAssocMatchesForNon7B(dataBytes, nBytes);
                }
                break;
              }
              confirmedID = recordBurst(dataBytes, nBytes);
              recordedID = true;
              Serial.printf("[CRC OK inv0s %s] ", confirmedID ? "CONFIRMED" : "count=1");
              for (int i = 0; i < nBytes; i++) Serial.printf("%02X", dataBytes[i]);
              Serial.printf(" pf=%.2f h=%d\n", peakFrac, halfUs);
              if (nBytes >= 3) {
                pushAssocFrame(false, dataBytes, nBytes, -1.0f);
                printAssocMatchesForNon7B(dataBytes, nBytes);
              }
              if (confirmedID) {
                Serial.printf("==== TPMS CONFIRMED ====\n");
                parseTPMSData(dataBytes, nBytes);
              }
            }
            break;
          }
        }
      }
      
#if 0  // inv=1s 未確認パス: 他センサー調査時に有効化
      // inv=1 SHIFT版のデータ抽出
      if (!recordedID && sync1_shift.pos >= 0 && sync1_shift.pos < b1_shift - 24) {
        int remainingBits = b1_shift - sync1_shift.pos;
        for (int tryBytes = 3; tryBytes <= 8; tryBytes++) {
          if (tryBytes * 8 > remainingBits) continue;
          uint8_t dataBytes[64];
          int nBytes = 0;
          bitsToBytes(bits1_shift + sync1_shift.pos, tryBytes * 8, dataBytes, &nBytes);
          if (nBytes != tryBytes) continue;
          if (verifyCRC(dataBytes, nBytes, true)) {
            if (!recordedID) {
              confirmedID = recordBurst(dataBytes, nBytes);
              recordedID = true;
              Serial.printf("[CRC OK inv1s %s] ", confirmedID ? "CONFIRMED" : "count=1");
              for (int i = 0; i < nBytes; i++) Serial.printf("%02X", dataBytes[i]);
              Serial.printf(" pf=%.2f h=%d\n", peakFrac, halfUs);
              if (confirmedID) {
                Serial.printf("==== TPMS CONFIRMED ====\n");
                parseTPMSData(dataBytes, nBytes);
              }
            }
            break;
          }
        }
      }

#endif  // inv=1s

#if 0  // スライディングウィンドウ: 未確認パス・誤検出多発のため無効化
      // スライディングウィンドウ（SYNCベース抽出で見つからなかった場合のフォールバック）
      if (!recordedID) {
        const uint8_t* candidates[4] = {bits0, bits1, bits0_shift, bits1_shift};
        int candBits[4] = {b0, b1, b0_shift, b1_shift};
        const char* candNames[4] = {"slide0", "slide1", "slide0s", "slide1s"};
        for (int ci = 0; ci < 4 && !recordedID; ci++) {
          int maxStart = min(candBits[ci] - 32, 80);
          for (int startBit = 0; startBit <= maxStart && !recordedID; startBit += 8) {
            for (int tryBytes = 4; tryBytes <= 8; tryBytes++) {
              if (startBit + tryBytes * 8 > candBits[ci]) break;
              uint8_t dataBytes[8];
              int nBytes = 0;
              bitsToBytes(candidates[ci] + startBit, tryBytes * 8, dataBytes, &nBytes);
              if (nBytes == tryBytes && verifyCRC(dataBytes, nBytes, true)) {
                confirmedID = recordBurst(dataBytes, nBytes);
                recordedID = true;
                Serial.printf("[CRC OK %s @%d %s] ", candNames[ci], startBit, confirmedID ? "CONFIRMED" : "count=1");
                for (int i = 0; i < nBytes; i++) Serial.printf("%02X", dataBytes[i]);
                Serial.printf(" pf=%.2f h=%d\n", peakFrac, halfUs);
                if (confirmedID) {
                  Serial.printf("==== TPMS CONFIRMED ====\n");
                  parseTPMSData(dataBytes, nBytes);
                }
                break;
              }
            }
          }
        }
      }
#endif  // スライディングウィンドウ
    }  // shouldPrintの終わり
#endif  // CRC検証パイプライン無効化

    radio.startReceive();
  }

  // 念のため受信蹴り（任意）
  if (millis() - lastKickMs > 3000) {
    lastKickMs = millis();
    radio.startReceive();
  }
}

