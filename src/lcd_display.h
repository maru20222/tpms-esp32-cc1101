#pragma once
#include <Arduino.h>

// ─── タイヤ状態 ─────────────────────────────────────────────
// センサー 1 本分のデコード済みデータ。
// SENSOR_DEFS[] インデックス (0=R.L D, 1=F.L A, 2=F.R B, 3=R.R C) で管理。

struct TireState {
    float    kPa;               // 圧力 [kPa]
    float    bar;               // 圧力 [bar]
    int      tempC;             // 温度 [°C]
    char     label[8];          // "F.L A" など
    uint8_t  alertCode;         // 0=正常, 1=減圧, 2=加圧, 3=不明アラート
    uint32_t lastUpdateMs;      // millis() 更新時刻
    bool     valid;             // 有効データがあるか
};

// SENSOR_DEFS インデックス数: 0..3
static const int LCD_SENSOR_COUNT = 4;

// 各センサーの最新状態（lcd_display.cpp に実体、main.cpp から更新）
extern TireState g_tireState[LCD_SENSOR_COUNT];

// ─── 車両レイアウト (2LCD 左右分割) ──────────────────────────
// 左LCD(既存): 左側タイヤ   右LCD(追加): 右側タイヤ
//
//   ┌──────────┐  ┌──────────┐
//   │ F.L A    │  │ F.R B    │  ← 上段 (Front)
//   │──────────│  │──────────│
//   │ R.L D    │  │ R.R C    │  ← 下段 (Rear)
//   └──────────┘  └──────────┘
//
// SENSOR_DEFS[] インデックス → スロット (0=左上, 1=左下, 2=右上, 3=右下)
static const int LCD_SDEF_TO_SLOT[LCD_SENSOR_COUNT] = { 1, 0, 2, 3 };
//   sensorDefIdx:          0  1  2  3
//   slot:                  ↓  ↓  ↓  ↓
//                         LB LT RT RB  (L=左LCD, R=右LCD, T=top, B=bottom)

// ─── 公開 API ─────────────────────────────────────────────────

// LCD 初期化（setup() 内で呼ぶ）
void lcdBegin();

// センサーデコード結果を TireState に書き込み、描画フラグを立てる
// sensorDefIdx : SENSOR_DEFS[] 配列インデックス (0..3)、-1 なら無効
// kPa / bar    : 圧力値
// tempC        : 温度 [°C]
// alertStr     : main.cpp の alertStr7 (例: "" / " ALT:減圧" / " ALT:加圧")
void lcdUpdateTire(int sensorDefIdx, float kPa, float bar, int tempC, const char* alertStr);

// 変更のあった象限を再描画（loop() 内で周期的に呼ぶ）
void lcdRefresh();
