#pragma once
#include <Arduino.h>

// =============================================================================
// SensorConverter — MCP3208 ADC チャンネルの物理値変換
//
// チャンネルアサイン:
//   ch0  Motor1 電流   INA213 + シャント 10 mΩ
//   ch1  Motor2 電流   INA213 + シャント 10 mΩ
//   ch2  Motor3 電流   INA213 + シャント 10 mΩ
//   ch3  Motor4 電流   INA213 + シャント 10 mΩ
//   ch4  バッテリ電圧  分圧抵抗 100 kΩ / 10 kΩ (実分圧比 1/11)
//   ch5-7 未使用
//
// INA213 電流変換:
//   REF ピン: 3.3V を 20kΩ/10kΩ 分圧 → V_ref = 3.3 × 10/(20+10) = 1.1 V
//   V_out = V_ref + I × R_shunt × gain
//   I [A]  = (V_out − V_ref) / (gain × R_shunt)
//          = (ADC_count − CURRENT_OFFSET_COUNTS) × CURRENT_SCALE
//
//   CURRENT_OFFSET_COUNTS = V_ref × ADC_COUNTS / Vref
//                         = 1.1 × 4096 / 3.3 ≈ 1365.33 count  (0A 時の ADC 値)
//   CURRENT_SCALE = 3.3 / (4096 × 50 × 0.010) ≈ 1.611 mA/count
//   計測範囲: −1.1 A (count=0) ～ +5.5 A (count=4095)
//   最大正電流 = (4095 − 1365.33) × 1.611e-3 ≈ 4.40 A
//
// バッテリ電圧変換:
//   分圧比 = R_lower / (R_upper + R_lower) = 10k / 110k = 1/11
//   V_batt = (ADC_count / 4096 × Vref) × 11
//          ≈ ADC_count × 8.862 mV/count
//   最大計測電圧 = 3.3 × 11 = 36.3 V
// =============================================================================

namespace SensorConverter {

// --- ADC 基本パラメータ ------------------------------------------------------
constexpr float ADC_VREF   = 3.3f;    // 基準電圧 [V]
constexpr float ADC_COUNTS = 4096.0f; // 12bit フルスケール

// --- INA213 電流センスパラメータ ----------------------------------------------
constexpr float INA213_GAIN = 50.0f;  // ゲイン [V/V]
constexpr float SHUNT_OHMS  = 0.010f; // シャント抵抗 [Ω] (10 mΩ)

// INA213 REF ピン分圧 (3.3V → 20kΩ/10kΩ → 1.1V)
constexpr float INA213_REF_R_UPPER = 20000.0f; // 20 kΩ
constexpr float INA213_REF_R_LOWER = 10000.0f; // 10 kΩ
constexpr float INA213_REF_VOLTAGE =
    ADC_VREF * INA213_REF_R_LOWER / (INA213_REF_R_UPPER + INA213_REF_R_LOWER); // = 1.1 V

// 0A 時の ADC カウント (オフセット)
constexpr float CURRENT_OFFSET_COUNTS =
    INA213_REF_VOLTAGE * ADC_COUNTS / ADC_VREF; // ≈ 1365.33 count

// --- バッテリ電圧分圧パラメータ -----------------------------------------------
// 上側抵抗 (バッテリ+ ～ 計測点): 100 kΩ
// 下側抵抗 (計測点 ～ GND)      :  10 kΩ
// 実分圧比: 1/11 (ユーザー指定値 1/10 に近似)
constexpr float VDIV_R_UPPER = 100000.0f; // 100 kΩ
constexpr float VDIV_R_LOWER =  10000.0f; //  10 kΩ
constexpr float VDIV_RATIO   = (VDIV_R_UPPER + VDIV_R_LOWER) / VDIV_R_LOWER; // = 11.0

// --- スケールファクタ (コンパイル時定数) --------------------------------------
// 電流 [A/count]
constexpr float CURRENT_SCALE =
    ADC_VREF / (ADC_COUNTS * INA213_GAIN * SHUNT_OHMS); // ≈ 1.611e-3

// バッテリ電圧 [V/count]
constexpr float VOLTAGE_SCALE =
    ADC_VREF * VDIV_RATIO / ADC_COUNTS; // ≈ 8.862e-3

// --- 変換関数 (inline) -------------------------------------------------------

// ch0-3: モータ電流 [A]  (0A オフセット補正済み)
inline float toCurrentA(int16_t count) {
    return ((float)count - CURRENT_OFFSET_COUNTS) * CURRENT_SCALE;
}

// ch0-3: モータ電流 [mA]  (0A オフセット補正済み)
inline float toCurrentmA(int16_t count) {
    return ((float)count - CURRENT_OFFSET_COUNTS) * CURRENT_SCALE * 1000.0f;
}

// ch4: バッテリ電圧 [V]
inline float toBatteryV(int16_t count) {
    return (float)count * VOLTAGE_SCALE;
}

// チャンネルが電流チャンネルか判定 (ch0-3)
inline bool isCurrentChannel(uint8_t ch) {
    return ch <= 3;
}

// チャンネルがバッテリ電圧チャンネルか判定 (ch4)
inline bool isBatteryChannel(uint8_t ch) {
    return ch == 4;
}

} // namespace SensorConverter
