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

// DEBUG also runs the CAN debug dump every CAN_DUMP_INTERVAL_MS (tag can_dump):
// TWAI state and error counters, frames and IDs seen, one line per ID this
// build relies on (MISSING when absent) with the last bytes and decoded value,
// frames forwarded to RaceChrono against the per-ID limit, then the other IDs.
// DEBUG では CAN_DUMP_INTERVAL_MS ごとに CAN デバッグダンプ（タグ can_dump）も出る:
// TWAI の状態とエラーカウンタ、受信フレーム数と ID 数、このビルドが使う ID ごとに 1 行
//（来ていなければ MISSING、最後のデータとデコード値）、ID ごとの上限に対する
// RaceChrono への転送数、続いてその他の ID。
#define CAN_DUMP_INTERVAL_MS 10000
#define CAN_DUMP_MAX_IDS 64 // distinct IDs tracked / 追跡する ID の種類数

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
// Display spec settled 2026-09-17. The bench sketch led_selftest.ino carries
// the same rules and the same numbers; keep the two in step.
// 表示仕様は 2026-09-17 確定版。ベンチ用 led_selftest.ino と同じ規則・同じ数値。
// 片方を変えたらもう片方も合わせること。
// ---------------------------------------------------------------------------

// Data pin to the strip DIN. Freenove silkscreen "25".
// テープの DIN へ。Freenove のシルク「25」。
#define SHIFT_LED_PIN 25
#define SHIFT_NUM_LEDS 24
#define SHIFT_BAR_LEDS (SHIFT_NUM_LEDS - 1)     // bar patterns: LED#2..#24 / バー: LED#2〜#24
#define SHIFT_MIRROR_PAIRS (SHIFT_NUM_LEDS / 2) // mirror pattern: 12 pairs / ミラー: 12 対

// Byte order sent to the tape. Same numbering as COLOR_ORDER in led_selftest.ino.
//   0 = GRB (WS2812B standard)   1 = RGB   2 = RBG   3 = BGR   4 = GBR   5 = BRG
// テープへ送る色順。led_selftest.ino の COLOR_ORDER と同じ番号。
#define SHIFT_COLOR_ORDER 0

// 0..255. Applies to every colour, the power-on show included.
// 128 = daytime level (2026-09-27, raised from 64). Bench fed from the Freenove
// "3.3V" pin: stay at or below 128, the all-white flash of the show is the
// heaviest load. In the car (MP1584EN #1, LED only) anything up to 255 is fine.
// 全色に掛かる（起動イルミも含む）。128 = 昼間用（2026-09-27、64 から引き上げ）。
// Freenove「3.3V」ピンから給電するベンチでは 128 以下にする（イルミの全灯白が最大負荷）。
// 車載（MP1584EN #1、LED 専用）なら 255 まで問題ない。
#define SHIFT_MASTER_BRIGHTNESS 128
#define SHIFT_GRAY_LEVEL_PCT 25 // dim gray = this percent of master / グレーはマスターのこの割合

// Display patterns. Cycled by the button, kept in NVS (namespace "shiftled",
// key "pattern") and restored at boot.
//   0 = bar, grows from the strip START (the DIN end)
//   1 = bar, grows from the strip FAR end (same bar, mirrored)
//   2 = mirror, both ends grow toward the centre
// SHIFT_PATTERN_DEFAULT is used only while NVS holds nothing (first boot).
// Once a pattern has been saved, the button is the only way to change it.
// 表示パターン。ボタンで巡回、NVS に保存し、起動時に復元する。
//   0 = バー、テープ先頭側（DIN 側）から伸びる
//   1 = バー、テープ末尾側から伸びる（同じバーの左右反転）
//   2 = 対称、両端から中央へ
// SHIFT_PATTERN_DEFAULT は NVS が空のとき（初回起動）だけ使われる。
// 一度保存された後はボタンでしか変わらない。
#define SHIFT_PATTERN_COUNT 3
#define SHIFT_PATTERN_DEFAULT 2

// Pattern button: BOOT on the Freenove board (GPIO 0, active low, internal
// pull-up). Each press = next pattern, confirmed on the strip with white LEDs
// (pattern + 1 of them; RED instead of white = the NVS write failed). The same
// LEDs are shown at boot, so a correct restore is visible without a PC.
// Never hold this button while resetting or powering on: GPIO 0 low at reset
// enters the bootloader.
// パターン切替ボタン: Freenove ボードの BOOT（GPIO 0、アクティブ Low、内部プルアップ）。
// 押すたびに次のパターン。白 LED（パターン番号 + 1 個）で確認、白でなく赤なら
// NVS 書き込み失敗。起動時にも同じ表示が出るので PC なしで復元を確認できる。
// リセット・電源投入中に押し続けないこと（GPIO 0 が Low だとブートローダに入る）。
#define SHIFT_BUTTON_PIN 0
#define SHIFT_BUTTON_ACTIVE_LOW 1
#define SHIFT_BUTTON_DEBOUNCE_MS 40
#define SHIFT_INDICATE_MS 600 // white / red confirmation shown this long / 白・赤の確認表示の長さ

// Bar scale. The bar is FULLY LIT at SHIFT_RPM_BAR_FULL (the shift point) in
// every pattern, uniform steps. Change *_FULL and the steps follow.
//   bar    : LED#2 1400 ... LED#24 5800, 200 rpm per LED
//   mirror : pair1 1400 ... pair12 5800, 400 rpm per pair
// LED#1 is always orange, LED#24 is orange until the bar reaches it.
// In the mirror pattern the outer pair is orange below the first step.
// バーの目盛り。全パターンで SHIFT_RPM_BAR_FULL（シフトポイント）で全点灯、等間隔。
// *_FULL を変えれば刻みは自動で追従する。
//   バー  : LED#2 1400 〜 LED#24 5800、1 個 200 rpm
//   ミラー: pair1 1400 〜 pair12 5800、1 対 400 rpm
// LED#1 は常時オレンジ、LED#24 はバーが届くまでオレンジ。
// ミラーでは最初の刻み未満で最外周の対がオレンジ。
#define SHIFT_RPM_BAR_FIRST 1400
#define SHIFT_RPM_BAR_FULL 5800
#define SHIFT_RPM_BAR_STEP ((SHIFT_RPM_BAR_FULL - SHIFT_RPM_BAR_FIRST) / (SHIFT_BAR_LEDS - 1)) // 200
#define SHIFT_RPM_MIRROR_FIRST 1400
#define SHIFT_RPM_MIRROR_FULL 5800
#define SHIFT_RPM_MIRROR_STEP ((SHIFT_RPM_MIRROR_FULL - SHIFT_RPM_MIRROR_FIRST) / (SHIFT_MIRROR_PAIRS - 1)) // 400

// Colour of the WHOLE lit bar, decided by the current rpm (not per LED).
//   below GRAY_MAX : dim gray    below BLUE_MAX : blue    below YELLOW : green
//   YELLOW and up  : yellow      RED and up     : red
// From SHIFT_RPM_FLASH up everything flashes at 10 Hz, bar length kept.
// FLASH is one step AFTER the bar is full, so "full and steady" exists.
// 点灯中のバー全体が、現在回転数で決まる 1 色になる（LED ごとの色ではない）。
//   GRAY_MAX 未満: グレー  BLUE_MAX 未満: 青  YELLOW 未満: 緑  YELLOW 以上: 黄  RED 以上: 赤
// SHIFT_RPM_FLASH 以上は全体が 10 Hz で点滅、バーの長さは維持。
// FLASH は全点灯の 1 段上なので「全点灯・点灯したまま」の段が必ずある。
#define SHIFT_RPM_GRAY_MAX 3000
#define SHIFT_RPM_BLUE_MAX 4000
#define SHIFT_RPM_YELLOW 5400
#define SHIFT_RPM_FLASH (SHIFT_RPM_BAR_FULL + SHIFT_RPM_BAR_STEP) // 6000
#define SHIFT_RPM_RED 6200

#define SHIFT_FLASH_PERIOD_MS 100 // 10 Hz flash / 点滅 10 Hz
#define SHIFT_FRAME_MS 20         // LED refresh period / LED 更新周期

// Power-on illumination, played once by the LED task right after boot:
//   rainbow (whole hue circle scrolling along the strip)
//   -> white flash that fades out
//   -> gauge sweep: the real display driven 0 -> SWEEP_TOP rpm and back, in
//      the active pattern, so every LED, every colour and the flash are seen
//   -> white pattern confirmation (see the button above)
// If a live rpm at/above SHIFT_STARTUP_ABORT_RPM arrives while it plays (unit
// reset while driving) the show stops at once and the normal display takes over.
// 0 disables the show; only the pattern confirmation remains.
// 起動イルミネーション。LED タスクが起動直後に 1 回だけ再生する:
//   レインボー（色相環をテープに並べて流す）
//   → 白フラッシュしてフェードアウト
//   → 回転数スイープ: 本番の表示を 0 → SWEEP_TOP rpm → 0 と動かす。現在のパターンで
//      全 LED・全色・点滅が一通り見える
//   → パターン確認の白表示（上のボタンの項）
// 再生中に SHIFT_STARTUP_ABORT_RPM 以上の実回転数が来たら（走行中のリセット）
// 直ちに中断して通常表示に移る。0 で無効（パターン確認だけ残る）。
#define SHIFT_STARTUP_SHOW 1
#define SHIFT_STARTUP_RAINBOW_MS 1200
#define SHIFT_STARTUP_FLASH_MS 400
#define SHIFT_STARTUP_SWEEP_UP_MS 1000
#define SHIFT_STARTUP_SWEEP_HOLD_MS 300
#define SHIFT_STARTUP_SWEEP_DOWN_MS 500
#define SHIFT_STARTUP_SWEEP_TOP_RPM (SHIFT_RPM_RED + SHIFT_RPM_BAR_STEP) // 6400: red and flashing at the top / 頂点で赤点滅
#define SHIFT_STARTUP_ABORT_RPM SHIFT_RPM_BAR_FIRST

// RPM source. Same frame as the RaceChrono RPM channel: ID 0x0618A001, bytes 2-3
// big endian = rpm as is (divisor 1). Confirmed by the in-car log of 2026-09-07
// (manual Rev.1.8, 3-1: idle 790, blip 1491, engine off 0) and by the RaceChrono
// equation bytesToUint(raw, 2, 2). The earlier guess of 4 kept the bar dark in
// the car (2026-09-30). LOG_LEVEL_DEBUG prints raw and rpm every 10 s.
// 回転数の取得元。ID 0x0618A001 の b2-b3（ビッグエンディアン）がそのまま rpm（除数 1）。
// 2026-09-07 の実車ログ（手順書 Rev.1.8 の 3-1: アイドル 790、空吹かし 1491、停止で 0）と
// RaceChrono の式 bytesToUint(raw, 2, 2) で確定。以前の仮置き 4 では実車でバーが
// 点かなかった（2026-09-30）。DEBUG では 10 秒ごとに生値と rpm を出す。
#define SHIFT_RPM_CAN_ID 0x0618A001
#define SHIFT_RPM_DIVISOR 1

// Blank the bar if no RPM frame arrives for this long (engine off / bus quiet).
// この時間 RPM フレームが来なければバーを消す（エンジン停止・バス停止）。
#define SHIFT_RPM_TIMEOUT_US 1000000

#endif // CONFIG_H

