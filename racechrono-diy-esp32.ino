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
#include <Preferences.h> // esp32 core, NVS - shift indicator pattern / シフトインジケータのパターン保存

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
//
// Display rules (settled 2026-09-17, same as the bench sketch led_selftest.ino):
//   three patterns (bar from the strip start / bar from the far end / mirror),
//   cycled by the BOOT button and kept in NVS; the bar is full at
//   SHIFT_RPM_BAR_FULL; the whole lit bar is ONE colour chosen by the current
//   rpm; 10 Hz flash from SHIFT_RPM_FLASH with the bar length kept; all red from
//   SHIFT_RPM_RED. Numbers live in config.h. A power-on illumination runs once.
// シフトインジケータ。外部ライブラリ不要、esp32 core の RMT HAL で駆動。
// ビットタイミングと API は core 3.3.x の esp32-hal-rgb-led.c に準拠。
// 表示規則（2026-09-17 確定、ベンチ用 led_selftest.ino と同じ）:
//   3 パターン（テープ先頭から / 末尾から / 対称）を BOOT ボタンで巡回し NVS に保存。
//   SHIFT_RPM_BAR_FULL で全点灯。点灯中のバー全体が現在回転数で決まる 1 色。
//   SHIFT_RPM_FLASH 以上は長さを保ったまま 10 Hz 点滅、SHIFT_RPM_RED 以上は全部赤。
//   数値は config.h。起動時にイルミネーションを 1 回再生する。
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
static const ShiftRgb SHIFT_C_WHITE = {255, 255, 255};

static rmt_data_t shiftRmtBuf[SHIFT_NUM_LEDS * 24];
static uint8_t shiftPx[SHIFT_NUM_LEDS][3];
static int shiftBarRpm[SHIFT_BAR_LEDS];        // rpm at which bar LED k (LED#(k+2)) lights
static int shiftMirrorRpm[SHIFT_MIRROR_PAIRS]; // rpm at which pair k (LED#(k+1) + LED#(24-k)) lights
static bool isShiftLedStarted = false;
static uint32_t shiftFramesSent = 0;
static uint32_t shiftFramesFailed = 0;

// Display pattern: restored from NVS at boot, changed by the button.
// 表示パターン。起動時に NVS から復元、ボタンで変更。
static Preferences shiftPrefs;
static bool shiftNvsOk = false;
static uint8_t shiftPattern = SHIFT_PATTERN_DEFAULT;
static bool shiftFlipped = false;   // pattern 1: LED#1 is at the far end of the strip
static bool shiftUseMirror = false; // pattern 2
static bool shiftBtnPrev = false;
static uint32_t shiftBtnLastMs = 0;

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

// Physical position on the strip (0 = the DIN end). Every drawing call ends
// up here; master brightness and byte order are applied here only.
// テープ上の物理位置（0 = DIN 側）。描画は全てここを通る。
// マスター輝度と色順はここでだけ掛ける。
static void shiftPutPixel(int i, ShiftRgb c)
{
    if (i < 0 || i >= SHIFT_NUM_LEDS)
    {
        return;
    }

    uint8_t r = shiftScale8(c.r, SHIFT_MASTER_BRIGHTNESS);
    uint8_t g = shiftScale8(c.g, SHIFT_MASTER_BRIGHTNESS);
    uint8_t b = shiftScale8(c.b, SHIFT_MASTER_BRIGHTNESS);

#if SHIFT_COLOR_ORDER == 1
    shiftPx[i][0] = r; shiftPx[i][1] = g; shiftPx[i][2] = b; // RGB
#elif SHIFT_COLOR_ORDER == 2
    shiftPx[i][0] = r; shiftPx[i][1] = b; shiftPx[i][2] = g; // RBG
#elif SHIFT_COLOR_ORDER == 3
    shiftPx[i][0] = b; shiftPx[i][1] = g; shiftPx[i][2] = r; // BGR
#elif SHIFT_COLOR_ORDER == 4
    shiftPx[i][0] = g; shiftPx[i][1] = b; shiftPx[i][2] = r; // GBR
#elif SHIFT_COLOR_ORDER == 5
    shiftPx[i][0] = b; shiftPx[i][1] = r; shiftPx[i][2] = g; // BRG
#else
    shiftPx[i][0] = g; shiftPx[i][1] = r; shiftPx[i][2] = b; // GRB (WS2812B default)
#endif
}

// Logical LED# (0 = LED#1, the low-rpm end) to the strip, honouring the
// direction of the active pattern. Only this mapping knows which physical end
// LED#1 is.
// 論理位置（0 = LED#1、低回転側）から実際の LED へ。現在のパターンの向きを反映する。
// LED#1 がどちらの端かを知っているのはこの変換だけ。
static void shiftSetPixel(int i, ShiftRgb c)
{
    if (i < 0 || i >= SHIFT_NUM_LEDS)
    {
        return;
    }
    shiftPutPixel(shiftFlipped ? (SHIFT_NUM_LEDS - 1 - i) : i, c);
}

static void shiftClear()
{
    memset(shiftPx, 0, sizeof(shiftPx));
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

    bool ok = rmtWrite(SHIFT_LED_PIN, shiftRmtBuf, SHIFT_NUM_LEDS * 24, 100);
    delayMicroseconds(300); // strip latch / リセット期間
    if (ok)
    {
        shiftFramesSent++;
    }
    else
    {
        shiftFramesFailed++;
    }
}

// ---- display / 表示 ----

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

static int shiftMirrorLitPairs(uint32_t rpm)
{
    int n = 0;
    for (int k = 0; k < SHIFT_MIRROR_PAIRS; k++)
    {
        if (rpm >= (uint32_t)shiftMirrorRpm[k])
        {
            n++;
        }
    }
    return n;
}

// ONE colour for the whole lit bar, decided by the CURRENT rpm.
// 点灯中のバー全体の色。現在の回転数で決まる。
static ShiftRgb shiftBarColor(uint32_t rpm)
{
    if (rpm >= SHIFT_RPM_RED)
    {
        return SHIFT_C_RED;
    }
    if (rpm >= SHIFT_RPM_YELLOW)
    {
        return SHIFT_C_YELLOW;
    }
    if (rpm >= SHIFT_RPM_BLUE_MAX)
    {
        return SHIFT_C_GREEN;
    }
    if (rpm >= SHIFT_RPM_GRAY_MAX)
    {
        return SHIFT_C_BLUE;
    }
    return shiftPct(SHIFT_C_GRAY, SHIFT_GRAY_LEVEL_PCT);
}

static const char *shiftBarColorName(uint32_t rpm)
{
    if (rpm >= SHIFT_RPM_RED)
    {
        return "red";
    }
    if (rpm >= SHIFT_RPM_YELLOW)
    {
        return "yellow";
    }
    if (rpm >= SHIFT_RPM_BLUE_MAX)
    {
        return "green";
    }
    if (rpm >= SHIFT_RPM_GRAY_MAX)
    {
        return "blue";
    }
    return "gray";
}

static const char *shiftPatternName(uint8_t p)
{
    switch (p)
    {
    case 0:
        return "bar from strip START (DIN end)";
    case 1:
        return "bar from strip FAR end";
    default:
        return "mirror, ends -> centre";
    }
}

// Bar: LED#1 orange, LED#2..#24 fill up in one colour, bar length kept while
// flashing. LED#24 is orange until the bar reaches it. Direction is handled
// by shiftSetPixel().
// バー: LED#1 オレンジ、LED#2〜#24 が 1 色で伸びる。点滅中も長さは維持。
// LED#24 はバーが届くまでオレンジ。向きは shiftSetPixel() が扱う。
static void shiftRenderBar(uint32_t rpm)
{
    shiftSetPixel(0, SHIFT_C_ORANGE);

    int n = shiftLitCount(rpm);
    ShiftRgb c = shiftBarColor(rpm);
    for (int k = 0; k < n; k++)
    {
        shiftSetPixel(k + 1, c);
    }

    if (n < SHIFT_BAR_LEDS)
    {
        shiftSetPixel(SHIFT_NUM_LEDS - 1, SHIFT_C_ORANGE);
    }
}

// Mirror: both ends grow toward the centre, outer pair orange while nothing is lit.
// 対称: 両端から中央へ。何も点いていない間は最外周の対がオレンジ。
static void shiftRenderMirror(uint32_t rpm)
{
    int n = shiftMirrorLitPairs(rpm);
    if (n == 0)
    {
        shiftSetPixel(0, SHIFT_C_ORANGE);
        shiftSetPixel(SHIFT_NUM_LEDS - 1, SHIFT_C_ORANGE);
        return;
    }

    ShiftRgb c = shiftBarColor(rpm);
    for (int k = 0; k < n; k++)
    {
        shiftSetPixel(k, c);                     // LED#1 .. LED#12
        shiftSetPixel(SHIFT_NUM_LEDS - 1 - k, c); // LED#24 .. LED#13
    }
}

static void shiftRender(uint32_t rpm)
{
    shiftClear();

    bool flashing = rpm >= SHIFT_RPM_FLASH;
    if (flashing && ((millis() / (SHIFT_FLASH_PERIOD_MS / 2)) & 1))
    {
        return; // off phase, oranges included / 消灯フェーズ。オレンジも消える
    }

    if (shiftUseMirror)
    {
        shiftRenderMirror(rpm);
    }
    else
    {
        shiftRenderBar(rpm);
    }
}

// Latest rpm from the bus, or 0 once no RPM frame has arrived for
// SHIFT_RPM_TIMEOUT_US (engine off, bus quiet, transceiver unplugged).
// バスから来た最新の回転数。SHIFT_RPM_TIMEOUT_US の間 RPM フレームが来なければ 0。
static uint32_t shiftLiveRpm()
{
    uint32_t rpm = shiftRpm;
    if (((uint32_t)esp_timer_get_time() - shiftRpmLastTime) > SHIFT_RPM_TIMEOUT_US)
    {
        rpm = 0;
    }
    return rpm;
}

// ---- pattern storage + button / パターン保存とボタン ----

static void shiftApplyPattern()
{
    if (shiftPattern >= SHIFT_PATTERN_COUNT)
    {
        shiftPattern = 0;
    }
    shiftFlipped = (shiftPattern == 1);
    shiftUseMirror = (shiftPattern == 2);
}

// On-device proof: white LEDs = pattern index + 1, RED = storage problem.
// Drawn on the physical strip so it does not mirror with the pattern.
// Runs inside the LED task (it sleeps).
// 実機での証明: 白 LED の数 = パターン番号 + 1、赤なら保存に問題。
// 物理位置に描くのでパターンで反転しない。LED タスク内で実行する（待ちが入る）。
static void shiftIndicatePattern(bool storeOk)
{
    ShiftRgb c = storeOk ? SHIFT_C_WHITE : SHIFT_C_RED;
    shiftClear();
    for (int i = 0; i <= shiftPattern; i++)
    {
        shiftPutPixel(i, c);
    }
    shiftShow();
    vTaskDelay(pdMS_TO_TICKS(SHIFT_INDICATE_MS));
    shiftClear();
    shiftShow();
    vTaskDelay(pdMS_TO_TICKS(120));
}

// Called from setup() before the LED task starts.
// LED タスク開始前に setup() から呼ぶ。
static void shiftLoadPattern()
{
    shiftNvsOk = shiftPrefs.begin("shiftled", false);
    if (!shiftNvsOk)
    {
        LOG_ERROR("shift_led", "NVS open failed, using SHIFT_PATTERN_DEFAULT.");
        shiftPattern = SHIFT_PATTERN_DEFAULT;
        shiftApplyPattern();
        return;
    }

    bool had = shiftPrefs.isKey("pattern");
    shiftPattern = shiftPrefs.getUChar("pattern", (uint8_t)SHIFT_PATTERN_DEFAULT);
    shiftApplyPattern();
    LOG_NOTICE("shift_led", "Pattern " << (int)shiftPattern << " (" << shiftPatternName(shiftPattern) << ") "
                                       << (had ? "restored from NVS." : "- NVS empty, SHIFT_PATTERN_DEFAULT used."));
}

static bool shiftSavePattern()
{
    if (!shiftNvsOk)
    {
        LOG_ERROR("shift_led", "NVS not open, pattern not saved.");
        shiftIndicatePattern(false);
        return false;
    }

    size_t n = shiftPrefs.putUChar("pattern", shiftPattern);
    uint8_t rb = shiftPrefs.getUChar("pattern", 0xFF); // read back: the write is only proven by this
    bool ok = (n > 0) && (rb == shiftPattern);
    if (ok)
    {
        LOG_NOTICE("shift_led", "Pattern " << (int)shiftPattern << " saved to NVS and read back.");
    }
    else
    {
        LOG_ERROR("shift_led", "NVS write failed: wrote " << n << " byte(s), read back " << (int)rb << ".");
    }
    shiftIndicatePattern(ok);
    return ok;
}

#if SHIFT_PATTERN_RESET
// Restart once so the power-on illumination shows the new pattern. Waits for
// the button to be released first, since a button still down at boot would
// be read as another press. Gives up, without restarting, if it is held longer
// than SHIFT_PATTERN_RESET_WAIT_MS. Runs in the LED task.
// 新しいパターンを起動イルミで見せるため一度再起動する。先にボタンが離されるのを
// 待つ（押したまま起動すると、もう一度押されたと判定される）。
// SHIFT_PATTERN_RESET_WAIT_MS より長く押されていたら再起動しない。LED タスク内で実行。
static void shiftRestartAfterRelease()
{
    uint32_t t0 = millis();
    while (digitalRead(SHIFT_BUTTON_PIN) == (SHIFT_BUTTON_ACTIVE_LOW ? LOW : HIGH))
    {
        if ((millis() - t0) >= SHIFT_PATTERN_RESET_WAIT_MS)
        {
            LOG_WARNING("shift_led", "Button still held after " << SHIFT_PATTERN_RESET_WAIT_MS << " ms, restart skipped.");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(SHIFT_BUTTON_DEBOUNCE_MS)); // let the contact settle / 接点が落ち着くまで

    LOG_NOTICE("shift_led", "Restarting to show pattern " << (int)shiftPattern << ".");
    Serial.flush();
    shiftPrefs.end(); // putUChar() has already committed / putUChar() でコミット済み
    ESP.restart();
}
#endif

static void shiftNextPattern()
{
    shiftPattern = (uint8_t)((shiftPattern + 1) % SHIFT_PATTERN_COUNT);
    shiftApplyPattern();
    LOG_NOTICE("shift_led", "Button: pattern -> " << (int)shiftPattern << " (" << shiftPatternName(shiftPattern) << ").");
    bool saved = shiftSavePattern();
#if SHIFT_PATTERN_RESET
    if (saved)
    {
        shiftRestartAfterRelease();
    }
#else
    (void)saved;
#endif
}

static void shiftPollButton()
{
    bool down = (digitalRead(SHIFT_BUTTON_PIN) == (SHIFT_BUTTON_ACTIVE_LOW ? LOW : HIGH));
    uint32_t now = millis();
    if (down != shiftBtnPrev && (now - shiftBtnLastMs) >= SHIFT_BUTTON_DEBOUNCE_MS)
    {
        shiftBtnLastMs = now;
        shiftBtnPrev = down;
        if (down)
        {
            shiftNextPattern();
        }
    }
}

// ---- power-on illumination / 起動イルミネーション ----

#if SHIFT_STARTUP_SHOW

static ShiftRgb shiftRgb(uint8_t r, uint8_t g, uint8_t b)
{
    ShiftRgb o;
    o.r = r;
    o.g = g;
    o.b = b;
    return o;
}

static ShiftRgb shiftDim(ShiftRgb c, uint8_t level255)
{
    return shiftRgb(shiftScale8(c.r, level255), shiftScale8(c.g, level255), shiftScale8(c.b, level255));
}

// Hue 0..359 at full saturation, value v.
// 色相 0〜359（彩度最大）、明度 v。
static ShiftRgb shiftHsv(uint16_t hue, uint8_t v)
{
    hue %= 360;
    uint8_t sector = (uint8_t)(hue / 60);
    uint16_t f = (uint16_t)(((hue % 60) * 255) / 60);
    uint8_t t = (uint8_t)(((uint16_t)v * f) / 255);
    uint8_t q = (uint8_t)(v - t);
    switch (sector)
    {
    case 0:
        return shiftRgb(v, t, 0);
    case 1:
        return shiftRgb(q, v, 0);
    case 2:
        return shiftRgb(0, v, t);
    case 3:
        return shiftRgb(0, q, v);
    case 4:
        return shiftRgb(t, 0, v);
    default:
        return shiftRgb(v, 0, q);
    }
}

typedef void (*ShiftFrameFn)(uint32_t t, uint32_t total);

// One phase of the show, frame by frame. Returns false when the engine is
// already revving (live rpm at/above SHIFT_STARTUP_ABORT_RPM) - the caller
// then drops the rest of the show. The button keeps working meanwhile.
// ショーの 1 区間をフレームごとに描く。実回転数が SHIFT_STARTUP_ABORT_RPM 以上なら
// false を返し、呼び側は残りを打ち切る。ボタンはこの間も効く。
static bool shiftRunPhase(uint32_t totalMs, ShiftFrameFn frame)
{
    uint32_t t0 = millis();
    for (;;)
    {
        uint32_t t = millis() - t0;
        if (t > totalMs)
        {
            t = totalMs;
        }
        if (shiftLiveRpm() >= SHIFT_STARTUP_ABORT_RPM)
        {
            return false;
        }

        shiftClear();
        frame(t, totalMs);
        shiftShow();
        shiftPollButton();

        if (t >= totalMs)
        {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(SHIFT_FRAME_MS));
    }
}

// 1. The whole hue circle laid along the strip, turning twice.
// 1. 色相環をテープ全長に並べ、2 回転させる。
static void shiftShowRainbow(uint32_t t, uint32_t total)
{
    uint32_t turn = (t * 720) / total;
    for (int i = 0; i < SHIFT_NUM_LEDS; i++)
    {
        shiftPutPixel(i, shiftHsv((uint16_t)(((uint32_t)i * 360) / SHIFT_NUM_LEDS + turn), 255));
    }
}

// 2. All white, fading out (quadratic so the tail of the fade is gentle).
// 2. 全灯白からフェードアウト（2 乗で終わりを緩やかに）。
static void shiftShowFlashFade(uint32_t t, uint32_t total)
{
    uint32_t rem = total - t;
    uint8_t level = (uint8_t)((255UL * rem * rem) / (total * total));
    for (int i = 0; i < SHIFT_NUM_LEDS; i++)
    {
        shiftPutPixel(i, shiftDim(SHIFT_C_WHITE, level));
    }
}

// 3. Gauge sweep: the real display, 0 -> SHIFT_STARTUP_SWEEP_TOP_RPM, hold, back to 0.
// 3. 回転数スイープ: 本番の表示を 0 → SHIFT_STARTUP_SWEEP_TOP_RPM → 保持 → 0。
static void shiftShowSweepUp(uint32_t t, uint32_t total)
{
    shiftRender((uint32_t)(((uint64_t)SHIFT_STARTUP_SWEEP_TOP_RPM * t) / total));
}

static void shiftShowSweepHold(uint32_t t, uint32_t total)
{
    (void)t;
    (void)total;
    shiftRender(SHIFT_STARTUP_SWEEP_TOP_RPM);
}

static void shiftShowSweepDown(uint32_t t, uint32_t total)
{
    shiftRender((uint32_t)(SHIFT_STARTUP_SWEEP_TOP_RPM - ((uint64_t)SHIFT_STARTUP_SWEEP_TOP_RPM * t) / total));
}

// Whole show. Returns false when it was cut short by a live rpm.
// ショー全体。実回転数で打ち切られたら false。
static bool shiftStartupShow()
{
    LOG_NOTICE("shift_led", "Power-on illumination.");
    bool done = shiftRunPhase(SHIFT_STARTUP_RAINBOW_MS, shiftShowRainbow) &&
                shiftRunPhase(SHIFT_STARTUP_FLASH_MS, shiftShowFlashFade) &&
                shiftRunPhase(SHIFT_STARTUP_SWEEP_UP_MS, shiftShowSweepUp) &&
                shiftRunPhase(SHIFT_STARTUP_SWEEP_HOLD_MS, shiftShowSweepHold) &&
                shiftRunPhase(SHIFT_STARTUP_SWEEP_DOWN_MS, shiftShowSweepDown);
    if (done)
    {
        LOG_NOTICE("shift_led", "Illumination finished, " << shiftFramesSent << " frames sent, "
                                                          << shiftFramesFailed << " failed.");
    }
    else
    {
        LOG_NOTICE("shift_led", "Illumination cut short: engine already at " << shiftLiveRpm() << " rpm.");
    }
    return done;
}

#endif // SHIFT_STARTUP_SHOW

// ---- task / タスク ----

void taskUpdateShiftLed(void *)
{
    LOG_DEBUG("task_shift_led", "Starting up on core " << xPortGetCoreID() << ".");

    bool showDone = true;
#if SHIFT_STARTUP_SHOW
    showDone = shiftStartupShow();
#endif
    if (showDone)
    {
        // Boot proof of the restored pattern: white LEDs = pattern + 1, red = NVS trouble.
        // Skipped when the show was cut short - the driver needs the bar right now.
        // 復元したパターンの証明: 白 LED の数 = パターン + 1、赤なら NVS に問題。
        // ショーを打ち切ったときは省く。今すぐバーが要る。
        shiftIndicatePattern(shiftNvsOk);
    }

    uint64_t reportTimerStart = esp_timer_get_time();
    int reportTimerInterval = 10000000; // 10 seconds

    for (;;)
    {
        uint32_t rpm = shiftLiveRpm();

        shiftRender(rpm);
        shiftShow();
        shiftPollButton();

        // Debug - the only on-device way to check the raw-to-rpm conversion.
        // 生値から回転数への換算を実機で確認する唯一の手段。
        if (LOG_LEVEL == LOG_LEVEL_DEBUG)
        {
            if ((esp_timer_get_time() - reportTimerStart) >= reportTimerInterval)
            {
                int lit = shiftUseMirror ? shiftMirrorLitPairs(rpm) : shiftLitCount(rpm);
                int litMax = shiftUseMirror ? SHIFT_MIRROR_PAIRS : SHIFT_BAR_LEDS;
                LOG_DEBUG("task_shift_led", "raw " << shiftRpmRaw << " -> " << rpm << " rpm, pattern "
                                                   << (int)shiftPattern << ", " << lit << "/" << litMax
                                                   << (shiftUseMirror ? " pairs" : " LEDs") << " lit ("
                                                   << (lit > 0 ? shiftBarColorName(rpm) : "dark") << "), frames "
                                                   << shiftFramesSent << " sent / " << shiftFramesFailed << " failed.");
                reportTimerStart = esp_timer_get_time();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SHIFT_FRAME_MS));
    }
}

// Called from setup() first thing, so the strip lights up as soon as power
// comes on. Pinned to the core running setup() (the TWAI core): that task
// blocks on twai_receive() so it yields, and core 0 is left to the BLE stack.
// Priority 1, below the TWAI and BLE tasks - the bar must never delay them.
// 電源が入ったらすぐ光るよう、setup() の最初に呼ぶ。setup() を実行しているコア
//（TWAI と同じコア）に固定: TWAI タスクは twai_receive で待つので譲る。
// コア 0 は BLE スタックに残す。優先度は 1 で TWAI・BLE より低い。
void startShiftIndicator()
{
    for (int k = 0; k < SHIFT_BAR_LEDS; k++)
    {
        shiftBarRpm[k] = SHIFT_RPM_BAR_FIRST + k * SHIFT_RPM_BAR_STEP;
    }
    for (int k = 0; k < SHIFT_MIRROR_PAIRS; k++)
    {
        shiftMirrorRpm[k] = SHIFT_RPM_MIRROR_FIRST + k * SHIFT_RPM_MIRROR_STEP;
    }

    pinMode(SHIFT_BUTTON_PIN, SHIFT_BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
    // Start from the real button state, so a button already down at boot is not a press.
    // 起動時の実際の状態から始める。起動時に押されていても押下と数えない。
    shiftBtnPrev = (digitalRead(SHIFT_BUTTON_PIN) == (SHIFT_BUTTON_ACTIVE_LOW ? LOW : HIGH));
    shiftLoadPattern();

    if (rmtInit(SHIFT_LED_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000)) // 100 ns tick
    {
        isShiftLedStarted = true;
        LOG_NOTICE("shift_led", "RMT started on GPIO " << SHIFT_LED_PIN << ", bar full at "
                                                       << shiftBarRpm[SHIFT_BAR_LEDS - 1] << " rpm, flash from "
                                                       << SHIFT_RPM_FLASH << ", red from " << SHIFT_RPM_RED << ".");
        xTaskCreatePinnedToCore(taskUpdateShiftLed, "taskUpdateShiftLed", 8192, NULL, 1, NULL, xPortGetCoreID());
    }
    else
    {
        LOG_ERROR("shift_led", "rmtInit failed on GPIO " << SHIFT_LED_PIN << ", shift indicator disabled.");
    }
}

// ---------------------------------------------------------------------------
// CAN debug dump - LOG_LEVEL_DEBUG only. Every CAN_DUMP_INTERVAL_MS a separate
// low-priority task prints whether the CAN side is alive: TWAI state and
// error counters, frames and IDs seen, one line per ID this build relies on
// (rate, what went to RaceChrono against the per-ID limit, last bytes,
// decoded value; "MISSING" when the ID did not arrive), then the other IDs on
// the bus. taskGetTwaiMessages only updates counters; printing never blocks it.
// CAN デバッグダンプ（LOG_LEVEL_DEBUG のみ）。CAN_DUMP_INTERVAL_MS ごとに別の低優先度
// タスクが、CAN 側が生きているかを出力する: TWAI の状態とエラーカウンタ、受信フレーム数と
// ID 数、このビルドが使う ID ごとに 1 行（レート、RaceChrono へ送った数と ID ごとの上限、
// 最後のデータ、デコード値。来ていなければ MISSING）、続いてバス上のその他の ID。
// taskGetTwaiMessages はカウンタを更新するだけで、出力に待たされることはない。
// ---------------------------------------------------------------------------

struct CanDumpEntry
{
    uint32_t id;
    uint32_t count;     // frames in the current interval / 今の区間の受信数
    uint32_t forwarded; // of which sent to the ring buffer (BLE) / うちリングバッファへ送った数
    uint8_t dlc;
    uint8_t data[8];
};

static CanDumpEntry canDumpTable[CAN_DUMP_MAX_IDS];
static int canDumpUsed = 0;
static uint32_t canDumpFrames = 0;    // per interval / 区間ごと
static uint32_t canDumpForwarded = 0; // per interval / 区間ごと
static uint32_t canDumpOverflow = 0;  // frames from IDs that did not fit the table / 表に入らなかった ID のフレーム数
static portMUX_TYPE canDumpMux = portMUX_INITIALIZER_UNLOCKED;

// IDs that get their own line, in this order. Same IDs as getUpdateRateHz() in config.h.
// 1 行ずつ出す ID（この順）。config.h の getUpdateRateHz() と同じ ID。
static const struct
{
    uint32_t id;
    const char *name;
} canDumpKnown[] = {
    {0x0618A001, "rpm/pedal"},
    {0x0030A002, "steering"},
    {0x0210A006, "speed"},
    {0x0218A006, "wheels"},
    {0x0810A000, "brake sw"},
    {0x0010A006, "brake prs"},
    {0x0628A001, "clutch"},
};
#define CAN_DUMP_KNOWN_COUNT ((int)(sizeof(canDumpKnown) / sizeof(canDumpKnown[0])))

// Called from taskGetTwaiMessages for every received frame.
// 受信フレームごとに taskGetTwaiMessages から呼ぶ。
static void canDumpRecord(const twai_message_t *m, bool forwarded)
{
    portENTER_CRITICAL(&canDumpMux);
    canDumpFrames++;
    if (forwarded)
    {
        canDumpForwarded++;
    }

    int i;
    for (i = 0; i < canDumpUsed; i++)
    {
        if (canDumpTable[i].id == m->identifier)
        {
            break;
        }
    }
    if (i == canDumpUsed)
    {
        if (canDumpUsed >= CAN_DUMP_MAX_IDS)
        {
            canDumpOverflow++;
            portEXIT_CRITICAL(&canDumpMux);
            return;
        }
        canDumpUsed++;
        canDumpTable[i].id = m->identifier;
        canDumpTable[i].count = 0;
        canDumpTable[i].forwarded = 0;
        canDumpTable[i].dlc = 0;
    }

    CanDumpEntry *e = &canDumpTable[i];
    e->count++;
    if (forwarded)
    {
        e->forwarded++;
    }
    if (!m->rtr)
    {
        e->dlc = m->data_length_code > 8 ? 8 : m->data_length_code;
        memcpy(e->data, m->data, e->dlc);
    }
    portEXIT_CRITICAL(&canDumpMux);
}

static const char *canDumpStateName(twai_state_t s)
{
    switch (s)
    {
    case TWAI_STATE_STOPPED:
        return "STOPPED";
    case TWAI_STATE_RUNNING:
        return "RUNNING";
    case TWAI_STATE_BUS_OFF:
        return "BUS_OFF";
    case TWAI_STATE_RECOVERING:
        return "RECOVERING";
    default:
        return "UNKNOWN";
    }
}

// Decoded value of one known ID, using the byte positions written in config.h
// and the manual (Rev.1.8, chapter 3-1).
// config.h と手順書（Rev.1.8 の 3-1）のバイト位置で既知 ID をデコードする。
static void canDumpDecode(const CanDumpEntry *e, char *out, size_t outLen)
{
    const uint8_t *d = e->data;
    switch (e->id)
    {
    case 0x0618A001:
        if (e->dlc >= 8)
        {
            unsigned raw = ((unsigned)d[2] << 8) | d[3];
            snprintf(out, outLen, "rpm raw %u -> %u rpm (raw/%d), pedal %u/255 = %u %%", raw,
                     raw / SHIFT_RPM_DIVISOR, SHIFT_RPM_DIVISOR, d[7], (d[7] * 100u) / 255u);
            return;
        }
        break;
    case 0x0030A002:
        if (e->dlc >= 6)
        {
            unsigned raw20 = (((unsigned)d[3] << 16) | ((unsigned)d[4] << 8) | d[5]) >> 4;
            int off = (int)raw20 - 7400; // centre about 7400, 0.1 deg per count, left + (manual Rev.1.8)
            snprintf(out, outLen, "steering raw20 %u (0x%05X), about %s%d.%d deg (centre 7400, left +)", raw20, raw20,
                     off < 0 ? "-" : "", abs(off) / 10, abs(off) % 10);
            return;
        }
        break;
    case 0x0210A006:
        if (e->dlc >= 6)
        {
            unsigned raw = ((unsigned)d[4] << 8) | d[5];
            unsigned tenths = (raw * 10u) / 128u;
            snprintf(out, outLen, "speed raw %u = %u.%u km/h", raw, tenths / 10, tenths % 10);
            return;
        }
        break;
    case 0x0218A006:
        if (e->dlc >= 8)
        {
            unsigned t[4];
            for (int k = 0; k < 4; k++)
            {
                t[k] = ((((unsigned)d[2 * k] << 8) | d[2 * k + 1]) * 10u) / 16u;
            }
            snprintf(out, outLen, "wheels b0-1 %u.%u, b2-3 %u.%u, b4-5 %u.%u, b6-7 %u.%u km/h", t[0] / 10, t[0] % 10,
                     t[1] / 10, t[1] % 10, t[2] / 10, t[2] % 10, t[3] / 10, t[3] % 10);
            return;
        }
        break;
    case 0x0810A000:
        if (e->dlc >= 3)
        {
            snprintf(out, outLen, "brake switch nibble %u (1 off, 7 on)", d[2] >> 4);
            return;
        }
        break;
    case 0x0010A006:
        if (e->dlc >= 3)
        {
            snprintf(out, outLen, "brake pressure raw %u", ((unsigned)d[1] << 8) | d[2]);
            return;
        }
        break;
    case 0x0628A001:
        if (e->dlc >= 6)
        {
            snprintf(out, outLen, "clutch b5 bit5 = %u", (d[5] & 0x20) ? 1u : 0u);
            return;
        }
        break;
    default:
        break;
    }
    snprintf(out, outLen, "(frame too short to decode, dlc %u)", e->dlc);
}

void taskCanDump(void *)
{
    LOG_DEBUG("task_can_dump", "Starting up on core " << xPortGetCoreID() << ".");

    static CanDumpEntry snap[CAN_DUMP_MAX_IDS]; // static: keep it off the task stack / タスクのスタックに置かない
    static char line[400];
    static char dec[160];
    static char hex[3 * 8 + 1];

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(CAN_DUMP_INTERVAL_MS));

        // Snapshot, then reset the interval counters. Short critical section on purpose.
        // スナップショットを取り、区間カウンタをリセット。クリティカルセクションは短く。
        portENTER_CRITICAL(&canDumpMux);
        int used = canDumpUsed;
        memcpy(snap, canDumpTable, sizeof(CanDumpEntry) * used);
        uint32_t frames = canDumpFrames;
        uint32_t forwarded = canDumpForwarded;
        uint32_t overflow = canDumpOverflow;
        canDumpFrames = 0;
        canDumpForwarded = 0;
        canDumpOverflow = 0;
        for (int i = 0; i < used; i++)
        {
            canDumpTable[i].count = 0;
            canDumpTable[i].forwarded = 0;
        }
        portEXIT_CRITICAL(&canDumpMux);

        int activeIds = 0;
        for (int i = 0; i < used; i++)
        {
            if (snap[i].count > 0)
            {
                activeIds++;
            }
        }

        twai_status_info_t st;
        bool stOk = (twai_get_status_info(&st) == ESP_OK);
        const unsigned intervalS = CAN_DUMP_INTERVAL_MS / 1000;

        if (stOk)
        {
            snprintf(line, sizeof(line),
                     "TWAI %s | %u s: %u frames, %u IDs, %u forwarded to BLE | bus_err %u, rx_missed %u, rx_overrun %u, "
                     "rx_err_cnt %u, arb_lost %u, rx_queued %u | RaceChrono %s, PIDs %s | uptime %u s",
                     canDumpStateName(st.state), intervalS, (unsigned)frames, (unsigned)activeIds, (unsigned)forwarded,
                     (unsigned)st.bus_error_count, (unsigned)st.rx_missed_count, (unsigned)st.rx_overrun_count,
                     (unsigned)st.rx_error_counter, (unsigned)st.arb_lost_count, (unsigned)st.msgs_to_rx,
                     RaceChronoBle.isConnected() ? "connected" : "NOT connected",
                     pidMap.areAllPidsAllowed() ? "all allowed" : (pidMap.isEmpty() ? "none allowed" : "allowed"),
                     (unsigned)(esp_timer_get_time() / 1000000ULL));
        }
        else
        {
            snprintf(line, sizeof(line),
                     "TWAI status NOT AVAILABLE (driver not installed?) | %u s: %u frames, %u IDs, %u forwarded to BLE | "
                     "RaceChrono %s | uptime %u s",
                     intervalS, (unsigned)frames, (unsigned)activeIds, (unsigned)forwarded,
                     RaceChronoBle.isConnected() ? "connected" : "NOT connected",
                     (unsigned)(esp_timer_get_time() / 1000000ULL));
        }
        LOG_DEBUG("can_dump", line);

        // One line per known ID, MISSING when it did not arrive in this interval.
        // 既知 ID ごとに 1 行。この区間に来ていなければ MISSING。
        for (int k = 0; k < CAN_DUMP_KNOWN_COUNT; k++)
        {
            const CanDumpEntry *e = NULL;
            for (int i = 0; i < used; i++)
            {
                if (snap[i].id == canDumpKnown[k].id)
                {
                    e = &snap[i];
                    break;
                }
            }
            if (e == NULL || e->count == 0)
            {
                snprintf(line, sizeof(line), "  %08X %-9s MISSING - no frame in %u s", (unsigned)canDumpKnown[k].id,
                         canDumpKnown[k].name, intervalS);
                LOG_DEBUG("can_dump", line);
                continue;
            }

            hex[0] = '\0';
            for (int b = 0; b < e->dlc; b++)
            {
                snprintf(hex + 3 * b, sizeof(hex) - 3 * b, "%02X ", e->data[b]);
            }
            canDumpDecode(e, dec, sizeof(dec));

            unsigned tenthsHz = (unsigned)((e->count * 10000ULL) / CAN_DUMP_INTERVAL_MS);
            unsigned limitHz = getUpdateRateHz(e->id);
            unsigned limitPerInterval = limitHz * intervalS;
            // The rate limiter is proven here: forwarded must stay at or under the per-ID limit.
            // レート制限の証明はここ。forwarded が ID ごとの上限を超えてはならない。
            bool overLimit = e->forwarded > (limitPerInterval + limitPerInterval / 5 + 2);
            snprintf(line, sizeof(line), "  %08X %-9s %4u frames %3u.%u Hz, fwd %4u (limit %u)%s | %s| %s",
                     (unsigned)e->id, canDumpKnown[k].name, (unsigned)e->count, tenthsHz / 10, tenthsHz % 10,
                     (unsigned)e->forwarded, limitPerInterval, overLimit ? " ** OVER LIMIT **" : "", hex, dec);
            LOG_DEBUG("can_dump", line);
        }

        // Everything else on the bus, id:count, several per line.
        // バス上のその他の ID。id:件数、1 行に複数。
        int others = 0;
        size_t pos = 0;
        line[0] = '\0';
        for (int i = 0; i < used; i++)
        {
            if (snap[i].count == 0)
            {
                continue;
            }
            bool known = false;
            for (int k = 0; k < CAN_DUMP_KNOWN_COUNT; k++)
            {
                if (snap[i].id == canDumpKnown[k].id)
                {
                    known = true;
                    break;
                }
            }
            if (known)
            {
                continue;
            }
            others++;
            if (pos == 0)
            {
                pos = (size_t)snprintf(line, sizeof(line), "  other:");
            }
            pos += (size_t)snprintf(line + pos, sizeof(line) - pos, " %08X:%u", (unsigned)snap[i].id, (unsigned)snap[i].count);
            if (others % 8 == 0)
            {
                LOG_DEBUG("can_dump", line);
                pos = 0;
                line[0] = '\0';
            }
        }
        if (pos > 0)
        {
            LOG_DEBUG("can_dump", line);
        }
        if (overflow > 0)
        {
            snprintf(line, sizeof(line), "  %u frames from IDs beyond the %d-entry table were not listed", (unsigned)overflow,
                     CAN_DUMP_MAX_IDS);
            LOG_DEBUG("can_dump", line);
        }
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

            bool forwardedNow = false;
            if (!(message.rtr))
            {
                if (!(message.data_length_code == 0))
                {
                    void *entry = pidMap.getEntryId(message.identifier);
                    if (entry != NULL)
                    {
                        PidExtra *extra = pidMap.getExtra(entry);
                        if ((uint32_t)((uint32_t)esp_timer_get_time() - extra->lastMessageTime) >= extra->updateIntervalHz)
                        {
                            msgCountBufferTx++;
                            forwardedNow = true;
                            if (!(xRingbufferSend(bufferHandle, &message, sizeof(message), 0)))
                            {
                                LOG_WARNING("task_twai", "Receive ring buffer overflow, dropping one message.");
                            }
                            extra->lastMessageTime += extra->updateIntervalHz;
                        }
                    }
                }
            }

            // CAN debug dump counters (printed by taskCanDump, never from here).
            // CAN デバッグダンプ用カウンタ（出力は taskCanDump、ここでは出さない）。
            if (LOG_LEVEL == LOG_LEVEL_DEBUG)
            {
                canDumpRecord(&message, forwardedNow);
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

    // Shift indicator first: the strip lights up as soon as power comes on,
    // and nothing below is needed for it.
    // シフトインジケータを最初に。電源が入ったらすぐ光り、以降の処理に依存しない。
    startShiftIndicator();

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

    // CAN debug dump, DEBUG builds only. Priority 1 on the TWAI core: it only
    // prints, and must never get in the way of the TWAI or BLE tasks.
    // CAN デバッグダンプ（DEBUG のみ）。TWAI と同じコアで優先度 1。出力するだけで、
    // TWAI・BLE タスクの邪魔をしてはならない。
    if (LOG_LEVEL == LOG_LEVEL_DEBUG)
    {
        xTaskCreatePinnedToCore(taskCanDump, "taskCanDump", 4096, NULL, 1, NULL, mainCore);
    }
}

// The default Arduino loop is not used, so no point in keeping the task running
void loop()
{
    vTaskDelete(NULL);
}
