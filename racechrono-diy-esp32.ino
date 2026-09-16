// Required libraries
//
// NimBLE-Arduino : https://www.arduino.cc/reference/en/libraries/nimble-arduino/
// EasyLogger : https://www.arduino.cc/reference/en/libraries/easylogger/
// arduino-RaceChrono : https://github.com/timurrrr/arduino-RaceChrono
//
// Special thanks to Timur Iskhodzhanov / timurrr. This project heavily referenced
// his RaceChronoDiyBleDevice project, as well as using his RaceChrono library.
// https://github.com/timurrrr
//

// Copy the "config.h.example" file provided to "config.h"
// and edit as needed, following the instructions provided in README.md
#include "config.h"

#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>
#include <freertos/semphr.h>
#include <esp_mac.h>
#include <EasyLogger.h>
#include <RaceChrono.h>

// arduino-esp32 3.3.x releases the BLE controller memory during boot unless a
// BLE library claims it first. NimBLE-Arduino 1.4.x predates that hook, so on a
// classic ESP32 esp_bt_controller_init() fails with ESP_ERR_INVALID_STATE and
// the board reboots in a loop. Including this header claims the BLE memory
// (it does not exist on older cores, hence the guard).
#if __has_include(<esp32-hal-alloc-ble-mem.h>)
#include <esp32-hal-alloc-ble-mem.h>
#endif

// bool isTwaiDriverStarted = false;
bool isBLEStarted = false;
twai_message_t message;
RingbufHandle_t bufferHandle;

using PidExtra = struct
{
    uint32_t updateIntervalHz = 1000000 / DEFAULT_UPDATE_RATE_HZ;
    uint32_t lastMessageTime = 0;
};
RaceChronoPidMap<PidExtra> pidMap;

// ---------------------------------------------------------------------------
// Shift indicator - WS2812B bar driven by the esp32 core RMT HAL.
// No external LED library. Bit timing and API taken from arduino-esp32 3.3.x
// cores/esp32/esp32-hal-rgb-led.c : 100 ns tick, "1" = 0.8us high + 0.4us low,
// "0" = 0.4us high + 0.8us low. Verified on this board with led_selftest.
// シフトインジケータ。外部ライブラリ不要、esp32 core の RMT HAL で駆動。
// ビットタイミングと API は core 3.3.x の esp32-hal-rgb-led.c に準拠。
// ---------------------------------------------------------------------------

struct ShiftRgb
{
    uint8_t r, g, b;
};

static const ShiftRgb SHIFT_C_ORANGE = {255, 70, 0};
static const ShiftRgb SHIFT_C_GRAY = {255, 255, 255}; // scaled by SHIFT_GRAY_LEVEL_PCT
static const ShiftRgb SHIFT_C_BLUE = {0, 0, 255};
static const ShiftRgb SHIFT_C_GREEN = {0, 255, 0};
static const ShiftRgb SHIFT_C_YELLOW = {255, 220, 0};
static const ShiftRgb SHIFT_C_RED = {255, 0, 0};

static rmt_data_t shiftRmtBuf[SHIFT_NUM_LEDS * 24];
static uint8_t shiftPx[SHIFT_NUM_LEDS][3];
static int shiftBarRpm[SHIFT_BAR_LEDS]; // rpm at which bar LED k (LED#(k+2)) lights
static bool isShiftLedStarted = false;

// Written by taskGetTwaiMessages, read by taskUpdateShiftLed.
// taskGetTwaiMessages が書き、taskUpdateShiftLed が読む。
static volatile uint32_t shiftRpm = 0;
static volatile uint32_t shiftRpmRaw = 0;
// 32-bit on purpose: a 64-bit value is not written atomically on this CPU, so
// the reader could catch a half-updated timestamp. Unsigned wrap keeps the
// difference correct across the 71-minute rollover of the low 32 bits.
// 32 ビットで持つ。64 ビットはこの CPU では不可分に書けず、読み側が更新途中の値を
// 拾い得る。下位 32 ビットが約 71 分で一周しても、符号なしの引き算なら差は正しい。
static volatile uint32_t shiftRpmLastTime = 0;

static inline uint8_t shiftScale8(uint8_t v, uint16_t scale255)
{
    return (uint8_t)(((uint16_t)v * scale255) / 255);
}

static ShiftRgb shiftPct(ShiftRgb c, int percent)
{
    ShiftRgb o;
    o.r = (uint8_t)(((uint16_t)c.r * percent) / 100);
    o.g = (uint8_t)(((uint16_t)c.g * percent) / 100);
    o.b = (uint8_t)(((uint16_t)c.b * percent) / 100);
    return o;
}

static void shiftSetPixel(int i, ShiftRgb c)
{
    if (i < 0 || i >= SHIFT_NUM_LEDS)
    {
        return;
    }

    uint8_t r = shiftScale8(c.r, SHIFT_MASTER_BRIGHTNESS);
    uint8_t g = shiftScale8(c.g, SHIFT_MASTER_BRIGHTNESS);
    uint8_t b = shiftScale8(c.b, SHIFT_MASTER_BRIGHTNESS);

#if SHIFT_COLOR_ORDER_RGB
    shiftPx[i][0] = r;
    shiftPx[i][1] = g;
    shiftPx[i][2] = b;
#else
    shiftPx[i][0] = g;
    shiftPx[i][1] = r;
    shiftPx[i][2] = b;
#endif
}

static void shiftShow()
{
    int k = 0;
    for (int i = 0; i < SHIFT_NUM_LEDS; i++)
    {
        for (int c = 0; c < 3; c++)
        {
            uint8_t v = shiftPx[i][c];
            for (int bit = 7; bit >= 0; bit--)
            {
                if (v & (1 << bit))
                {
                    shiftRmtBuf[k].level0 = 1;
                    shiftRmtBuf[k].duration0 = 8; // 0.8 us
                    shiftRmtBuf[k].level1 = 0;
                    shiftRmtBuf[k].duration1 = 4; // 0.4 us
                }
                else
                {
                    shiftRmtBuf[k].level0 = 1;
                    shiftRmtBuf[k].duration0 = 4;
                    shiftRmtBuf[k].level1 = 0;
                    shiftRmtBuf[k].duration1 = 8;
                }
                k++;
            }
        }
    }

    rmtWrite(SHIFT_LED_PIN, shiftRmtBuf, SHIFT_NUM_LEDS * 24, 100);
    delayMicroseconds(300); // strip latch / リセット期間
}

// Colour of one LED, fixed by the rpm that LED stands for.
// 各 LED の色は、その LED が担当する回転数で固定。
static ShiftRgb shiftColorAtPos(int rpmPos)
{
    if (rpmPos >= SHIFT_RPM_RED)
    {
        return SHIFT_C_RED;
    }
    if (rpmPos >= SHIFT_RPM_YELLOW)
    {
        return SHIFT_C_YELLOW;
    }
    if (rpmPos >= SHIFT_RPM_BLUE_MAX)
    {
        return SHIFT_C_GREEN;
    }
    if (rpmPos >= SHIFT_RPM_GRAY_MAX)
    {
        return SHIFT_C_BLUE;
    }
    return shiftPct(SHIFT_C_GRAY, SHIFT_GRAY_LEVEL_PCT);
}

static int shiftLitCount(uint32_t rpm)
{
    int n = 0;
    for (int k = 0; k < SHIFT_BAR_LEDS; k++)
    {
        if (rpm >= (uint32_t)shiftBarRpm[k])
        {
            n++;
        }
    }
    return n;
}

// LED#1 is always orange. LED#2..#24 form the bar; each keeps its own colour,
// except at/above SHIFT_RPM_RED where every lit LED turns red. From
// SHIFT_RPM_YELLOW up everything flashes, but the BAR LENGTH IS KEPT so the
// last few hundred rpm still resolve LED by LED. LED#24 shows orange until the
// bar reaches it.
// LED#1 は常時オレンジ。LED#2〜#24 がバーで各 LED は自分の色を保つ。
// SHIFT_RPM_RED 以上では点灯中の全 LED が赤。SHIFT_RPM_YELLOW 以上は全体が点滅
// するが、バーの長さは維持するので最後の数百回転も LED 単位で読める。
// LED#24 はバーが到達するまでオレンジ。
static void shiftRender(uint32_t rpm)
{
    memset(shiftPx, 0, sizeof(shiftPx));

    bool flashing = rpm >= SHIFT_RPM_YELLOW;
    if (flashing && ((millis() / (SHIFT_FLASH_PERIOD_MS / 2)) & 1))
    {
        return; // off phase, oranges included / 消灯フェーズ。オレンジも消える
    }

    shiftSetPixel(0, SHIFT_C_ORANGE);

    int n = shiftLitCount(rpm);
    bool allRed = rpm >= SHIFT_RPM_RED;
    for (int k = 0; k < n; k++)
    {
        shiftSetPixel(k + 1, allRed ? SHIFT_C_RED : shiftColorAtPos(shiftBarRpm[k]));
    }

    if (n < SHIFT_BAR_LEDS)
    {
        shiftSetPixel(SHIFT_NUM_LEDS - 1, SHIFT_C_ORANGE);
    }
}

void taskUpdateShiftLed(void *)
{
    LOG_DEBUG("task_shift_led", "Starting up on core " << xPortGetCoreID() << ".");

    uint64_t reportTimerStart = esp_timer_get_time();
    int reportTimerInterval = 10000000; // 10 seconds

    for (;;)
    {
        uint32_t rpm = shiftRpm;

        // No RPM frame for a while: blank the bar but keep the orange markers.
        // しばらく RPM フレームが来なければバーを消す（オレンジは残す）。
        if (((uint32_t)esp_timer_get_time() - shiftRpmLastTime) > SHIFT_RPM_TIMEOUT_US)
        {
            rpm = 0;
        }

        shiftRender(rpm);
        shiftShow();

        // Debug - the only on-device way to check the raw-to-rpm conversion.
        // 生値から回転数への換算を実機で確認する唯一の手段。
        if (LOG_LEVEL == LOG_LEVEL_DEBUG)
        {
            if ((esp_timer_get_time() - reportTimerStart) >= reportTimerInterval)
            {
                LOG_DEBUG("task_shift_led", "raw " << shiftRpmRaw << " -> " << rpm << " rpm, "
                                                   << shiftLitCount(rpm) << "/" << SHIFT_BAR_LEDS << " LEDs lit.");
                reportTimerStart = esp_timer_get_time();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SHIFT_FRAME_MS));
    }
}

// Code taken from the arduino-RaceChrono example files.
class PrintRaceChronoCommands : public RaceChronoBleCanHandler
{
public:
    void allowAllPids(uint16_t updateIntervalMs)
    {
        LOG_INFO("racechrono", "Allow all PIDs.");
        pidMap.allowAllPids(updateIntervalMs);
    }

    void denyAllPids()
    {
        LOG_INFO("racechrono", "Deny all PIDs.");
        pidMap.reset();
    }

    void allowPid(uint32_t pid, uint16_t updateIntervalMs)
    {
        LOG_INFO("racechrono", "Allow PID " << pid << ".");

        if (pidMap.allowOnePid(pid, updateIntervalMs))
        {
            void *entry = pidMap.getEntryId(pid);
            PidExtra *pidExtra = pidMap.getExtra(entry);
            pidExtra->lastMessageTime = esp_timer_get_time();
            pidExtra->updateIntervalHz = 1000000 / getUpdateRateHz(pid);
            LOG_INFO("racechrono", "PID " << pid << " update interval (Hz): " << getUpdateRateHz(pid) << ".");
        }
        else
        {
            LOG_WARNING("racechrono", "Unable to handle this request.");
        }
    }

    void handleDisconnect()
    {
        if (!pidMap.isEmpty() || pidMap.areAllPidsAllowed())
        {
            LOG_NOTICE("racechrono", "RaceChrono disconnected.");
            LOG_INFO("racechrono", "Resetting the map.");
            this->denyAllPids();
        }
    }
} raceChronoHandler;

// Called from setup() on core 1
bool startTwaiDriver()
{
    LOG_NOTICE("twai", "Starting driver.");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_LISTEN_ONLY);
    g_config.rx_queue_len = TWAI_RX_QUEUE_LENGTH;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);

    if (twai_start() == ESP_OK)
    {
        // isTwaiDriverStarted = true;
        LOG_NOTICE("twai", "Driver started.");
        return true;
    }
    else
    {
        LOG_ERROR("twai", "Driver failed to start.");
    }

    return false;
}

// Never called, should probably be removed ...
bool stopTwaiDriver()
{
    LOG_NOTICE("twai", "Stopping driver.");

    if (twai_stop() == ESP_OK)
    {
        LOG_NOTICE("twai", "Driver stopped.");
        if (twai_driver_uninstall() == ESP_OK)
        {
            LOG_NOTICE("twai", "Driver uninstalled.");
            // isTwaiDriverStarted = false;
            return true;
        }
        else
        {
            LOG_ERROR("twai", "Driver failed to uninstall.");
        }
    }
    else
    {
        LOG_ERROR("twai", "Driver failed to stop.");
    }

    return false;
}

// Returns the Bluetooth device name to advertise
// If unspecified in the config file, returns a generic name based on the MAC address
char *getDeviceName()
{
#ifdef DEVICE_NAME
    LOG_INFO("getDeviceName", "Device name is " << DEVICE_NAME);
    return DEVICE_NAME;
#endif

    // Get Bluetooth MAC address
    // https://github.com/espressif/esp-idf/blob/master/examples/system/base_mac_address/main/base_mac_address_example_main.c
    uint8_t baseMac[6];
    static char deviceName[32];

    if (esp_read_mac(baseMac, ESP_MAC_BT) == ESP_OK)
    {
        sprintf(deviceName, "RaceChrono %x:%x:%x", baseMac[3], baseMac[4], baseMac[5]);
    }
    else
    {
        LOG_ERROR("getDeviceName", "Failed to get BT MAC address.");
        sprintf(deviceName, "RaceChrono aa:bb:cc");
    }

    LOG_INFO("getDeviceName", "Device name is " << deviceName);
    return deviceName;
}

// Called from taskManageBLEConnection() task on core 0
bool startBLEConnection()
{
    LOG_NOTICE("task_ble_manage", "Starting BLE.");

    RaceChronoBle.setUp(getDeviceName(), &raceChronoHandler);
    RaceChronoBle.startAdvertising();
    isBLEStarted = true;

    LOG_NOTICE("task_ble_manage", "BLE started.");

    return true;
}

void taskManageBLEConnection(void *)
{
    LOG_DEBUG("task_ble_manage", "Starting up on core " << xPortGetCoreID() << ".");
    bool connectedNotice = false;

    for (;;)
    {
        if (!isBLEStarted)
        {
            startBLEConnection();
        }

        if (!RaceChronoBle.isConnected())
        {
            connectedNotice = false;
            raceChronoHandler.handleDisconnect();
            LOG_INFO("task_ble_manage", "Waiting for a new connection.");
        }
        else if (!connectedNotice)
        {
            connectedNotice = true;
            LOG_NOTICE("task_ble_manage", "RaceChrono connected.");
        }

        vTaskDelay(10000);
    }
}

void taskGetTwaiMessages(void *)
{
    LOG_DEBUG("task_twai", "Starting up on core " << xPortGetCoreID() << ".");

    uint64_t taskGetTwaiMessagesTimerStart = esp_timer_get_time();
    int taskGetTwaiMessagesTimerInterval = 10000000; // 10 seconds
    uint64_t msgCountRx = 0;
    uint64_t msgCountBufferTx = 0;

    for (;;)
    {
        if (twai_receive(&message, portMAX_DELAY) == ESP_OK)
        {
            msgCountRx++;

            // Shift indicator: read the RPM straight off the bus, independently
            // of pidMap - pidMap is emptied whenever RaceChrono disconnects, so
            // relying on it would kill the bar with no phone connected.
            // シフトインジケータ用の回転数はバスから直接取得する。pidMap は
            // RaceChrono 切断時に空になるため、依存させると BLE 未接続でバーが死ぬ。
            if (!(message.rtr) && message.identifier == SHIFT_RPM_CAN_ID && message.data_length_code >= 4)
            {
                shiftRpmRaw = ((uint32_t)message.data[2] << 8) | (uint32_t)message.data[3];
                shiftRpm = shiftRpmRaw / SHIFT_RPM_DIVISOR;
                shiftRpmLastTime = (uint32_t)esp_timer_get_time();
            }

            if (!(message.rtr))
            {
                if (!(message.data_length_code == 0))
                {
                    void *entry = pidMap.getEntryId(message.identifier);
                    if (entry != NULL)
                    {
                        PidExtra *extra = pidMap.getExtra(entry);
                        if ((esp_timer_get_time() - extra->lastMessageTime) >= extra->updateIntervalHz)
                        {
                            msgCountBufferTx++;
                            if (!(xRingbufferSend(bufferHandle, &message, sizeof(message), 0)))
                            {
                                LOG_WARNING("task_twai", "Receive ring buffer overflow, dropping one message.");
                            }
                            extra->lastMessageTime += extra->updateIntervalHz;
                        }
                    }
                }
            }

            // Debug - Report messages received count from TWAI every 10 seconds
            if (LOG_LEVEL == LOG_LEVEL_DEBUG)
            {
                if ((esp_timer_get_time() - taskGetTwaiMessagesTimerStart) >= taskGetTwaiMessagesTimerInterval)
                {
                    LOG_DEBUG("task_twai", msgCountRx << " messages received from TWAI in the last 10 seconds.");
                    LOG_DEBUG("task_twai", msgCountBufferTx << " messages sent to ring buffer the last 10 seconds.");
                    msgCountRx = 0;
                    msgCountBufferTx = 0;
                    taskGetTwaiMessagesTimerStart = esp_timer_get_time();
                }
            }
        }
    }
}

void taskSendBLEMessages(void *)
{
    LOG_DEBUG("task_ble_send", "Starting up on core " << xPortGetCoreID());

    uint64_t taskSendBLEMessagesStart = esp_timer_get_time();
    int taskSendBLEMessagesInterval = 10000000; // 10 seconds
    uint64_t msgCountBufferRx = 0;

    size_t message_size;

    for (;;)
    {
        twai_message_t *message = (twai_message_t *)xRingbufferReceive(bufferHandle, &message_size, portMAX_DELAY);

        if (message != NULL)
        {
            RaceChronoBle.sendCanData(message->identifier, message->data, message->data_length_code);
            msgCountBufferRx++;

            // Debug - Report message received count from ring buffer every 10 seconds
            if (LOG_LEVEL == LOG_LEVEL_DEBUG)
            {
                if ((esp_timer_get_time() - taskSendBLEMessagesStart) >= taskSendBLEMessagesInterval)
                {
                    LOG_DEBUG("task_ble_send", msgCountBufferRx << " messages sent to RaceChronoBle in the last 10 seconds.");
                    msgCountBufferRx = 0;
                    taskSendBLEMessagesStart = esp_timer_get_time();
                }
            }
        }
        else
        {
            LOG_ERROR("task_ble_send", "Failed to receive message from receive ring buffer.");
        }

        vRingbufferReturnItem(bufferHandle, (void *)message);
    }
}

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    delay(2000);
    LOG_DEBUG("setup", "Starting up on core " << xPortGetCoreID());

    delay(1000);
    startTwaiDriver();

    bufferHandle = xRingbufferCreate(65536, RINGBUF_TYPE_NOSPLIT);
    if (bufferHandle == NULL)
    {
        LOG_ERROR("setup", "Failed to create ring buffer.");
    }

    // When main core is 1, assume dual-core mcu (S3) is used and map the TWAI task to it
    // Otherwise, assume single-core mcu (C3) and map all tasks to core 0
    uint mainCore = xPortGetCoreID();

    delay(1250); // Delay used to avoid multiple serial messages overlapping
    xTaskCreatePinnedToCore(taskManageBLEConnection, "taskManageBLEConnection", 4096, NULL, 1, NULL, 0);

    delay(1250); // Delay used to avoid multiple serial messages overlapping
    xTaskCreatePinnedToCore(taskGetTwaiMessages, "taskGetTwaiMessages", 4096, NULL, 5, NULL, mainCore);

    delay(1250); // Delay used to avoid multiple serial messages overlapping
    xTaskCreatePinnedToCore(taskSendBLEMessages, "taskSendBLEMessages", 4096, NULL, 5, NULL, 0);

    // Shift indicator. Pinned to the TWAI core: that task blocks on
    // twai_receive() so it yields, and core 0 is left to the BLE stack.
    // Priority 1, below the TWAI and BLE tasks - the bar must never delay them.
    // シフトインジケータ。TWAI と同じコアに固定（TWAI タスクは twai_receive で
    // ブロックするため譲る）。コア 0 は BLE スタックに残す。
    // 優先度は 1 で TWAI・BLE より低い。バーが他を遅らせないこと。
    shiftBarRpm[0] = SHIFT_RPM_BAR_1;
    shiftBarRpm[1] = SHIFT_RPM_BAR_2;
    for (int k = 2; k < SHIFT_BAR_LEDS; k++)
    {
        shiftBarRpm[k] = SHIFT_RPM_BAR_3 + (k - 2) * SHIFT_RPM_STEP;
    }

    delay(1250); // Delay used to avoid multiple serial messages overlapping
    if (rmtInit(SHIFT_LED_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000)) // 100 ns tick
    {
        isShiftLedStarted = true;
        LOG_NOTICE("shift_led", "RMT started on GPIO " << SHIFT_LED_PIN << ", bar tops out at "
                                                       << shiftBarRpm[SHIFT_BAR_LEDS - 1] << " rpm.");
        xTaskCreatePinnedToCore(taskUpdateShiftLed, "taskUpdateShiftLed", 4096, NULL, 1, NULL, mainCore);
    }
    else
    {
        LOG_ERROR("shift_led", "rmtInit failed on GPIO " << SHIFT_LED_PIN << ", shift indicator disabled.");
    }
}

// The default Arduino loop is not used, so no point in keeping the task running
void loop()
{
    vTaskDelete(NULL);
}
