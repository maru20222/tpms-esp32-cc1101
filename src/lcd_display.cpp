#include "lcd_display.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// ─── ピン定義 ────────────────────────────────────────────────
// doc/index.md の LCD 結線表より
#define LCD_SCK   17
#define LCD_MOSI  18
#define LCD_RST    4
#define LCD_DC     5
#define LCD_CS     6
#define LCD_BL     7

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
// Adafruit_SPITFT の SPIClass* コンストラクタを使うことで
// esp32-hal-periman.h に依存する Arduino_ESP32SPI を回避する。
static SPIClass        lcdSpi(FSPI);
static Adafruit_ST7789 tft(&lcdSpi, LCD_CS, LCD_DC, LCD_RST);

// ─── グローバル状態（extern 宣言は lcd_display.h に有り）───────
TireState g_tireState[LCD_SENSOR_COUNT];

// 再描画フラグ
static bool g_dirty[LCD_SENSOR_COUNT] = { true, true, true, true };

// ─── 象限レイアウト ──────────────────────────────────────────
static const int QUAD_X[4] = {   0, 120,   0, 120 };
static const int QUAD_Y[4] = {   0,   0, 120, 120 };

// 象限ラベル (LCD_SDEF_TO_QUAD 逆引き)
// q=0(TL)=F.L A, q=1(TR)=F.R B, q=2(BL)=R.L D, q=3(BR)=R.R C
static const char* QUAD_LABEL[4] = { "F.L A", "F.R B", "R.L D", "R.R C" };

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

// ─── 1 象限描画 ──────────────────────────────────────────────
static void drawQuadrant(int q) {
    if (q < 0 || q >= 4) return;

    const int qx = QUAD_X[q];
    const int qy = QUAD_Y[q];

    // 対応する sdi を逆引き
    int matchSdi = -1;
    for (int i = 0; i < LCD_SENSOR_COUNT; i++) {
        if (LCD_SDEF_TO_QUAD[i] == q) { matchSdi = i; break; }
    }
    const TireState& ts = (matchSdi >= 0) ? g_tireState[matchSdi] : g_tireState[0];
    const bool  valid   = ts.valid;
    uint16_t colText = pressureColor(ts.bar, ts.alertCode, valid);

    // 背景
    tft.fillRect(qx, qy, 119, 119, COLOR_BLACK);

    // ── ラベル行 ─────────────────────────────────────────────
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(2);
    tft.setCursor(qx + 4, qy + 4);
    tft.print(QUAD_LABEL[q]);

    if (!valid) {
        tft.setTextColor(COLOR_DARKGREY);
        tft.setTextSize(3);
        tft.setCursor(qx + 20, qy + 30);
        tft.print("---");
        tft.setTextSize(2);
        tft.setCursor(qx + 8, qy + 100);
        tft.print("NO DATA");
        return;
    }

    // ── 圧力 (bar) ────────────────────────────────────────────
    char bufBar[12];
    dtostrf(ts.bar, 4, 2, bufBar);
    const char* pBar = bufBar;
    while (*pBar == ' ') pBar++;

    tft.setTextColor(colText);
    tft.setTextSize(3);
    tft.setCursor(qx + 4, qy + 26);
    tft.print(pBar);

    tft.setTextSize(2);
    tft.setCursor(qx + 4, qy + 54);
    tft.print("bar");

    // ── kPa 小表示 ────────────────────────────────────────────
    char bufKpa[12];
    snprintf(bufKpa, sizeof(bufKpa), "%4.0fkPa", ts.kPa);
    tft.setTextColor(COLOR_LIGHTGREY);
    tft.setTextSize(1);
    tft.setCursor(qx + 4, qy + 72);
    tft.print(bufKpa);

    // ── 温度 ──────────────────────────────────────────────────
    char bufTmp[12];
    snprintf(bufTmp, sizeof(bufTmp), "%+d C", ts.tempC);
    tft.setTextColor(COLOR_CYAN);
    tft.setTextSize(2);
    tft.setCursor(qx + 4, qy + 82);
    tft.print(bufTmp);

    // ── アラート / OK ─────────────────────────────────────────
    const char* alertDisp = "OK";
    uint16_t    alertCol  = COLOR_GREEN;
    if      (ts.alertCode == 1) { alertDisp = "DEFL!";  alertCol = COLOR_RED; }
    else if (ts.alertCode == 2) { alertDisp = "INFL!";  alertCol = COLOR_ORANGE; }
    else if (ts.alertCode == 3) { alertDisp = "ALT?";   alertCol = COLOR_YELLOW; }
    else if (ts.bar < 1.8f)     { alertDisp = "LOW P!"; alertCol = COLOR_RED; }

    tft.setTextColor(alertCol);
    tft.setTextSize(2);
    tft.setCursor(qx + 4, qy + 100);
    tft.print(alertDisp);
}

// ─── 分割線描画 ───────────────────────────────────────────────
static void drawDividers() {
    tft.drawFastHLine(0, 119, 240, COLOR_WHITE);
    tft.drawFastVLine(119, 0, 240, COLOR_WHITE);
}

// ─── 公開 API 実装 ─────────────────────────────────────────────

void lcdBegin() {
    // バックライト ON
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);

    // FSPI(SPI2) を LCD ピンで初期化。CC1101 の HSPI(SPI3) と別ホストなので衝突なし。
    lcdSpi.begin(LCD_SCK, /*MISO*/ -1, LCD_MOSI, LCD_CS);

    // ST7789 初期化 (240×240, ピクセル開始オフセットなし)
    tft.init(240, 240, SPI_MODE3);
    tft.setRotation(0);
    tft.fillScreen(COLOR_BLACK);

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

    for (int q = 0; q < 4; q++) drawQuadrant(q);
    drawDividers();

    Serial.println("[LCD] init OK");
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
        drawQuadrant(LCD_SDEF_TO_QUAD[sdi]);
        g_dirty[sdi] = false;
    }
    drawDividers();
}
