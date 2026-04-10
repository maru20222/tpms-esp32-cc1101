#include "lcd_display.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// ─── ピン定義 ────────────────────────────────────────────────
// doc/index.md の LCD 結線表より
// SPI バス共有
#define LCD_SCK   17
#define LCD_MOSI  18
#define LCD_BL     7    // バックライト（全LCD共用）

// 第1LCD（左側、既存）
#define LCD1_RST   4
#define LCD1_DC    5
#define LCD1_CS    6

// 第2LCD（右側、追加）
#define LCD2_RST  19
#define LCD2_DC   21
#define LCD2_CS   20

// ─── 色定数 (RGB565) ─────────────────────────────────────────
#define COLOR_BLACK      0x0000u
#define COLOR_WHITE      0xFFFFu
#define COLOR_RED        0xF800u
#define COLOR_GREEN      0x07E0u
#define COLOR_YELLOW     0xFFE0u
#define COLOR_ORANGE     0xFC00u
#define COLOR_CYAN       0x07FFu
#define COLOR_DARKGREY   0x7BEFu
#define COLOR_LIGHTGREY  0xC618u

// ─── Adafruit_ST7789 インスタンス ─────────────────────────────
// ESP32-S3 で CC1101(HSPI=SPI3) と競合しないよう FSPI(SPI2) を専用バスとして使う。
// 2台のLCDが同一SPIバスを共有し、CS/DC/RST は個別。
static SPIClass        lcdSpi(FSPI);
static Adafruit_ST7789 tftL(&lcdSpi, LCD1_CS, LCD1_DC, LCD1_RST); // 左LCD
static Adafruit_ST7789 tftR(&lcdSpi, LCD2_CS, LCD2_DC, LCD2_RST); // 右LCD

// ─── グローバル状態（extern 宣言は lcd_display.h に有り）───────
TireState g_tireState[LCD_SENSOR_COUNT];

// 再描画フラグ
static bool g_dirty[LCD_SENSOR_COUNT] = { true, true, true, true };

// ─── スロットレイアウト ──────────────────────────────────────
// 各スロット: 240×120 (上半分 y=0, 下半分 y=120)
// slot 0 = 左LCD上 (F.L A)  slot 1 = 左LCD下 (R.L D)
// slot 2 = 右LCD上 (F.R B)  slot 3 = 右LCD下 (R.R C)
static const int SLOT_Y[4] = { 0, 120, 0, 120 };

static Adafruit_ST7789* slotTft(int slot) {
    return (slot < 2) ? &tftL : &tftR;
}

// スロットラベル (LCD_SDEF_TO_SLOT 逆引き)
static const char* SLOT_LABEL[4] = { "F.L A", "R.L D", "F.R B", "R.R C" };

// ─── ヘルパー ─────────────────────────────────────────────────

static uint16_t pressureColor(float bar, uint8_t alertCode, bool valid) {
    if (!valid)           return COLOR_DARKGREY;
    if (alertCode == 1)   return COLOR_RED;
    if (alertCode == 2)   return COLOR_ORANGE;
    if (alertCode == 3)   return COLOR_YELLOW;
    if (bar < 1.8f)       return COLOR_RED;
    if (bar > 3.5f)       return COLOR_ORANGE;
    return COLOR_GREEN;
}

// ─── 1 スロット描画 (240×120) ────────────────────────────────
static void drawSlot(int slot) {
    if (slot < 0 || slot >= 4) return;

    Adafruit_ST7789* tft = slotTft(slot);
    const int sy = SLOT_Y[slot];

    // 対応する sdi を逆引き
    int matchSdi = -1;
    for (int i = 0; i < LCD_SENSOR_COUNT; i++) {
        if (LCD_SDEF_TO_SLOT[i] == slot) { matchSdi = i; break; }
    }
    const TireState& ts = (matchSdi >= 0) ? g_tireState[matchSdi] : g_tireState[0];
    const bool  valid   = ts.valid;
    uint16_t colText = pressureColor(ts.bar, ts.alertCode, valid);

    // 背景クリア (240×119、分割線1px分を除く)
    tft->fillRect(0, sy, 240, 119, COLOR_BLACK);

    // ── ラベル行 ─────────────────────────────────────────────
    tft->setTextColor(COLOR_WHITE);
    tft->setTextSize(2);
    tft->setCursor(4, sy + 4);
    tft->print(SLOT_LABEL[slot]);

    if (!valid) {
        tft->setTextColor(COLOR_DARKGREY);
        tft->setTextSize(4);
        tft->setCursor(84, sy + 36);
        tft->print("---");
        tft->setTextSize(2);
        tft->setCursor(78, sy + 80);
        tft->print("NO DATA");
        return;
    }

    // ── kPa (ラベル行右側) ───────────────────────────────────
    char bufKpa[12];
    snprintf(bufKpa, sizeof(bufKpa), "%3.0fkPa", ts.kPa);
    tft->setTextColor(COLOR_LIGHTGREY);
    tft->setTextSize(2);
    tft->setCursor(168, sy + 4);
    tft->print(bufKpa);

    // ── 圧力 (bar) 大表示 ────────────────────────────────────
    char bufBar[12];
    dtostrf(ts.bar, 4, 2, bufBar);
    const char* pBar = bufBar;
    while (*pBar == ' ') pBar++;

    tft->setTextColor(colText);
    tft->setTextSize(5);
    tft->setCursor(8, sy + 28);
    tft->print(pBar);

    tft->setTextSize(2);
    tft->setCursor(136, sy + 52);
    tft->print("bar");

    // ── 温度 ──────────────────────────────────────────────────
    char bufTmp[12];
    snprintf(bufTmp, sizeof(bufTmp), "%+d C", ts.tempC);
    tft->setTextColor(COLOR_CYAN);
    tft->setTextSize(3);
    tft->setCursor(8, sy + 80);
    tft->print(bufTmp);

    // ── アラート / OK ─────────────────────────────────────────
    const char* alertDisp = "OK";
    uint16_t    alertCol  = COLOR_GREEN;
    if      (ts.alertCode == 1) { alertDisp = "DEFL!";  alertCol = COLOR_RED; }
    else if (ts.alertCode == 2) { alertDisp = "INFL!";  alertCol = COLOR_ORANGE; }
    else if (ts.alertCode == 3) { alertDisp = "ALT?";   alertCol = COLOR_YELLOW; }
    else if (ts.bar < 1.8f)     { alertDisp = "LOW P!"; alertCol = COLOR_RED; }

    tft->setTextColor(alertCol);
    tft->setTextSize(3);
    int alertLen = strlen(alertDisp);
    tft->setCursor(240 - alertLen * 18, sy + 80);
    tft->print(alertDisp);
}

// ─── 分割線描画 ───────────────────────────────────────────────
static void drawDividers() {
    tftL.drawFastHLine(0, 119, 240, COLOR_WHITE);
    tftR.drawFastHLine(0, 119, 240, COLOR_WHITE);
}

// ─── 公開 API 実装 ─────────────────────────────────────────────

void lcdBegin() {
    // バックライト ON（全LCD共用）
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);

    // FSPI(SPI2) を LCD ピンで初期化。CC1101 の HSPI(SPI3) と別ホストなので衝突なし。
    lcdSpi.begin(LCD_SCK, /*MISO*/ -1, LCD_MOSI, LCD1_CS);

    // 左LCD (ST7789 240×240)
    tftL.init(240, 240, SPI_MODE3);
    tftL.setRotation(0);
    tftL.fillScreen(COLOR_BLACK);

    // 右LCD (ST7789 240×240)
    tftR.init(240, 240, SPI_MODE3);
    tftR.setRotation(0);
    tftR.fillScreen(COLOR_BLACK);

    // 初期状態設定
    const char* initialLabels[LCD_SENSOR_COUNT] = {
        "R.L D",  // sdi=0
        "F.L A",  // sdi=1
        "F.R B",  // sdi=2
        "R.R C",  // sdi=3
    };
    for (int i = 0; i < LCD_SENSOR_COUNT; i++) {
        memset(&g_tireState[i], 0, sizeof(TireState));
        strncpy(g_tireState[i].label, initialLabels[i], sizeof(g_tireState[i].label) - 1);
        g_tireState[i].valid = false;
        g_dirty[i] = true;
    }

    for (int s = 0; s < 4; s++) drawSlot(s);
    drawDividers();

    Serial.println("[LCD] dual init OK (L+R)");
}

void lcdUpdateTire(int sensorDefIdx, float kPa, float bar, int tempC, const char* alertStr) {
    if (sensorDefIdx < 0 || sensorDefIdx >= LCD_SENSOR_COUNT) return;

    TireState& ts   = g_tireState[sensorDefIdx];
    ts.kPa          = kPa;
    ts.bar          = bar;
    ts.tempC        = tempC;
    ts.lastUpdateMs = millis();
    ts.valid        = true;

    if (alertStr == nullptr || alertStr[0] == '\0') {
        ts.alertCode = 0;
    } else if (strstr(alertStr, "減圧")) {
        ts.alertCode = 1;
    } else if (strstr(alertStr, "加圧")) {
        ts.alertCode = 2;
    } else {
        ts.alertCode = 3;
    }

    g_dirty[sensorDefIdx] = true;
}

void lcdRefresh() {
    bool anyDirty = false;
    for (int i = 0; i < LCD_SENSOR_COUNT; i++) {
        if (g_dirty[i]) { anyDirty = true; break; }
    }
    if (!anyDirty) return;

    for (int sdi = 0; sdi < LCD_SENSOR_COUNT; sdi++) {
        if (!g_dirty[sdi]) continue;
        drawSlot(LCD_SDEF_TO_SLOT[sdi]);
        g_dirty[sdi] = false;
    }
    drawDividers();
}
