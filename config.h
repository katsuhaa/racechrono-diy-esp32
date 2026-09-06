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
// TX_PIN : INTENTIONALLY NOT WIRED. The TWAI driver needs a valid GPIO number,
//          but nothing is connected to GPIO 5 and the transceiver D (TXD / CTX)
//          input is left open (open = recessive), so the bus can never be driven.
// RX_PIN : トランシーバの R (RXD / CRX) へ配線
// TX_PIN : 意図的に未配線（ドライバ起動用のダミー指定）。トランシーバの D (TXD / CTX) は開放＝レセッシブ固定。
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
// Fiat 500 (312) family broadcast frames use 29-bit IDs. The candidates below
// come from public Fiat 500 / 500L logs (build manual, chapter 3) and are NOT
// yet confirmed on the Abarth 595: uncomment a case once the ID is verified.
// Fiat 500（312）系の放送フレームは 29 ビット ID。候補は手順書 3 章の公開情報で、
// Abarth 595 では未確認。実車で確認できたらコメントを外す。
uint8_t getUpdateRateHz(uint32_t can_id)
{
    switch (can_id)
    {

    // case 0x0218A006: // Wheel speeds, 4 x 16-bit / 4 輪速
    //     return 20;
    // case 0x0210A006: // Vehicle speed, bytes 4-5 big-endian / 128 = km/h / 車速
    //     return 10;
    // case 0x0618A001: // Engine RPM, bytes 2-3 big-endian / 回転数
    //     return 10;
    // case 0x0810A000: // Brake, byte 2 upper nibble / ブレーキ
    //     return 20;

    default:
        return DEFAULT_UPDATE_RATE_HZ;
    }
}

#endif // CONFIG_H
