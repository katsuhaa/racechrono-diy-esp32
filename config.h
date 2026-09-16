#ifndef CONFIG_H
#define CONFIG_H

// Configuration for the Abarth 595 Series 4 (Fiat 500 / 312 platform),
// classic ESP32-WROOM-32E board (Freenove FNK0090B), receive-only build.
// Abarth 595 シリーズ4 / 無印 ESP32-WROOM（Freenove FNK0090B）向け・受信専用の設定。

// Bluetooth device name shown in the RaceChrono app
// RaceChrono アプリに表示される Bluetooth デバイス名
#define DEVICE_NAME "RaceChrono DIY ESP32"

// Serial logging level, select one from the following list :
// LOG_LEVEL_EMERGENCY
// LOG_LEVEL_ALERT
// LOG_LEVEL_CRITICAL
// LOG_LEVEL_ERROR
// LOG_LEVEL_WARNING
// LOG_LEVEL_NOTICE
// LOG_LEVEL_INFO
// LOG_LEVEL_DEBUG
// DEBUG prints the TWAI / ring buffer / BLE frame counts every 10 seconds,
// which is the only on-device proof that CAN frames are being received.
// DEBUG にすると 10 秒ごとに TWAI・リングバッファ・BLE の件数が出る（CAN 受信の唯一の確認手段）。
#define LOG_LEVEL LOG_LEVEL_DEBUG

// GPIO pins connected to the CAN transceiver (SN65HVD230)
// RX_PIN : wired to the transceiver R (RXD / CRX) output
// TX_PIN : wired to the transceiver D (TXD / CTX). On these VP230 modules an open
//          D input floats dominant and R stops toggling, so D must be tied high.
//          In TWAI_MODE_LISTEN_ONLY the controller never drives TX low, so the
//          bus is still never driven.
// RX_PIN : トランシーバの R (RXD / CRX) へ配線
// TX_PIN : トランシーバの D (TXD / CTX) へ配線。このモジュールは D 開放だと R が出なくなるため H に固定する。
//          LISTEN_ONLY では TX が L になることは無いのでバスは駆動されない。
#define RX_PIN 4
#define TX_PIN 5

// Serial baud rate
#define SERIAL_BAUD_RATE 115200

// TWAI RX buffer size
// Reduce to 64 if "Failed to create ring buffer." appears at boot
// 起動時に "Failed to create ring buffer." が出る場合は 64 に減らす
#define TWAI_RX_QUEUE_LENGTH 128

// Bluetooth 5.0 support, 1 to enable, 0 to disable
// The classic ESP32 is BT 4.2 and has no extended advertising: keep this commented out
// 無印 ESP32 は BT 4.2 のため有効化不可。コメントアウトのままとする
// #define CONFIG_BT_NIMBLE_EXT_ADV 1

// Limit the rate of messages sent to RaceChrono to x CAN messages each second
// Can be further refined per PID through the list below
// Start low and raise only the channels that matter in getUpdateRateHz()
// まず低めにし、必要なチャンネルだけ getUpdateRateHz() で引き上げる
#define DEFAULT_UPDATE_RATE_HZ 2

// An optional list of PIDs and their associated rate limit.
// Limits the update rate of the specified PID to the returned value.
// All IDs are 29-bit and were confirmed on the Abarth 595 Series 4 with the
// tools/can_monitor sketch (2026-09-07 in-car logs, build manual chapter 3).
// すべて 29 ビット ID。tools/can_monitor で実車確認済み（2026-09-07、手順書 3 章）。
uint8_t getUpdateRateHz(uint32_t can_id)
{
    switch (can_id)
    {

    case 0x0618A001: // RPM b2-3 BE, accelerator pedal b7 (0-255 = 0-100 %) / 回転数・アクセル
        return 20;
    case 0x0030A002: // Steering angle 20-bit (b3b4b5 >> 4), 0.1 deg, left + / 舵角
        return 20;
    case 0x0210A006: // Vehicle speed b4-5 BE / 128 = km/h / 車速
        return 10;
    case 0x0218A006: // Wheel speeds 4 x 16-bit BE / 16 = km/h / 4 輪速
        return 10;
    case 0x0810A000: // Brake switch b2 upper nibble (1 off, 7 on) / ブレーキ SW
        return 10;
    case 0x0010A006: // Brake pressure b1-2 BE (0 ... ~0x11B) / ブレーキ圧
        return 10;
    case 0x0628A001: // Clutch b5 bit5 (0x20) / クラッチ
        return 5;

    default:
        return DEFAULT_UPDATE_RATE_HZ;
    }
}

// ---------------------------------------------------------------------------
// Shift indicator - WS2812B 24-LED bar / シフトインジケータ
// ---------------------------------------------------------------------------

// Data pin to the strip DIN. Freenove silkscreen "25".
// テープの DIN へ。Freenove のシルク「25」。
#define SHIFT_LED_PIN 25
#define SHIFT_NUM_LEDS 24

// Physical direction of the strip in the case.
// 0 = LED#1 sits at the DIN end, the bar grows away from DIN.
// 1 = LED#1 sits at the far end, the bar grows towards DIN.
// Decided by which way round the strip ends up. Flip this one value only.
// 筐体に入れたときのテープの向き。
// 0 = DIN 側が LED#1。バーは DIN から遠ざかる向きに伸びる。
// 1 = DIN と反対側が LED#1。バーは DIN に向かって伸びる。
// 実物の向きで決める。変えるのはこの 1 個だけでよい。
#define SHIFT_REVERSE 1
#define SHIFT_BAR_LEDS (SHIFT_NUM_LEDS - 1) // LED#2..#24

// 0..255. Applies to every colour. Dashboard brightness to be tuned in the car.
// 全色に掛かる。車載時の明るさは実車で調整する。
#define SHIFT_MASTER_BRIGHTNESS 64
#define SHIFT_GRAY_LEVEL_PCT 25  // dim gray = this percent of master / グレーはマスターのこの割合
#define SHIFT_COLOR_ORDER_RGB 0  // 0 = GRB (WS2812B) / 0 で GRB

// Bar scale, settled 2026-09-12.
// LED#2 1000, LED#3 1700, LED#4 2400, then +200 rpm each -> LED#24 = 2400 + 20*200 = 6400.
// Every colour boundary lands exactly on an LED: 3000=#7, 4000=#12, 5800=#21, 6200=#23.
// バーの割り当て（2026-09-12 確定）。色の境界がすべて LED にぴったり乗る。
#define SHIFT_RPM_BAR_1 1000
#define SHIFT_RPM_BAR_2 1700
#define SHIFT_RPM_BAR_3 2400
#define SHIFT_RPM_STEP 200

// Colour of each LED, fixed by the rpm that LED stands for.
// 各 LED の色は、その LED が担当する回転数で固定。
#define SHIFT_RPM_GRAY_MAX 3000 // below: dim gray  (LED#2..#6)  / 未満はグレー
#define SHIFT_RPM_BLUE_MAX 4000 // below: blue      (LED#7..#11) / 未満は青
#define SHIFT_RPM_YELLOW 5800   // below: green (LED#12..#20); at/above: yellow AND flashing starts
                                // 未満は緑。以上は黄、かつ点滅開始
#define SHIFT_RPM_RED 6200      // at/above: every lit LED turns red / 以上は点灯中の全 LED が赤

#define SHIFT_FLASH_PERIOD_MS 100 // 10 Hz flash / 点滅 10 Hz
#define SHIFT_FRAME_MS 20         // LED refresh period / LED 更新周期

// RPM source. Same frame as the RaceChrono RPM channel: ID 0x0618A001, bytes 2-3
// big endian. The divisor below is NOT yet confirmed against the car - at idle
// (about 850 rpm) the bar must stay dark; if it lights several LEDs the divisor
// is wrong. LOG_LEVEL_DEBUG prints the raw value and the derived rpm every 10 s.
// 回転数の取得元。ID 0x0618A001 の b2-b3（ビッグエンディアン）。
// 下の除数は実車未確認。アイドリング（約 850rpm）でバーが消灯していれば正しい。
// 何個か点灯するなら除数が違う。DEBUG では 10 秒ごとに生値と換算値を出す。
#define SHIFT_RPM_CAN_ID 0x0618A001
#define SHIFT_RPM_DIVISOR 4

// Blank the bar if no RPM frame arrives for this long (engine off / bus quiet).
// この時間 RPM フレームが来なければバーを消す（エンジン停止・バス停止）。
#define SHIFT_RPM_TIMEOUT_US 1000000

#endif // CONFIG_H

