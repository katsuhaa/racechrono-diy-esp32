/*
 * twai_selftest.ino  -  TWAI (CAN) self-test / TWAI セルフテスト  Rev.5  (2026-09-07)
 *
 * 目的: SN65HVD230 モジュールが実際に CAN 信号を通すかを、ESP32 自身の測定と
 *       TWAI コントローラによる通信で判定する (テスターの DC 読みに頼らない)。
 *
 * 各サイクルは 4 段階:
 *  [STATIC] "5" を GPIO として H (recessive) / L (dominant) に固定し、
 *           "4" のレベル (HIGH / LOW / OPEN: 内部プルアップ読み + プルダウン読みで判別) と
 *           CANH / CANL の電圧 (ADC "34" / "35") を両状態で測る。
 *  [RXTEST] ESP32 の DAC ("25" → CANH, "26" → CANL) で CANH/CANL に電圧を直接印加し、
 *           (Rev.5) "34"/"35" が未配線 (全状態で 0.3 V 未満) のときは読み戻し無しで印加できたとみなして判定する。
 *           受信部 R ("4") が「差動電圧」に反応するか (コモンモードには反応しないか) を試す。
 *           これが車載バス (CANH 2.5→3.5 V, CANL 2.5→1.5 V, コモンモード一定) で受信できるかの判定。
 *           印加中は "5" を HIGH に保ちドライバを recessive にする。印加した電圧は "34"/"35" で読み戻して確認。
 *           終端 120Ω が付いた基板では DAC が差動を作れないので UNTESTABLE になる → R2 を外した基板で行う。
 *  [TWAI-A] NO_ACK モード + 自己受信要求 (ESP-IDF 公式 twai_self_test と同じ方式)。
 *           送信フレームは "5" → 外部経路 → "4" と戻ったときだけ受信される (内部ループなし)。
 *           経路が無ければ必ず FAIL、ジャンパ直結なら必ず PASS → 手順0/1 がテスト自体の検証。
 *           DAC はこの段階の前に必ず無効化 (high-Z) する。
 *  [TWAI-B] 同じ TWAI 通信を、モジュールのドライバを使わずに ESP32 が CANH/CANL を直接駆動して行う:
 *           TWAI の TX 信号を GPIO マトリクスで "26"(CANL) に、反転して "25"(CANH) に出す (B1)、
 *           または "25" を H 固定にして "26" だけ TX にする (B2)。受信部が 500 kbps の差動信号を
 *           正しく復号できるかの動的テスト = 車載バスの代わり。
 *           条件: "5"→"CTX" を外してモジュールのドライバを止める (STATIC で自動判定、つながっていれば skip)、
 *                 120Ω 終端が無いこと (RXTEST で電圧が印加できていること。あれば skip)。
 *
 * 配線 (Freenove ESP32-WROOM のシルク名 / モジュールのシルク名)。電源は USB のみ。
 *   手順0  "4" "5" に何も接続しない                         → 期待: TWAI FAIL (この基板では開放の "4" は LOW と読める)
 *   手順1  "5" と "4" をジャンパ線で直結                       → 期待: TWAI PASS
 *   手順2  ジャンパを外し、モジュールを接続
 *            "3.3V" → "3V3"   "GND" → "GND"   "5" → "CTX"   "4" → "CRX"
 *            "34" → "CANH"    "35" → "CANL"   (STATIC / RXTEST の読み戻し用)
 *            "25" → "CANH"    "26" → "CANL"   (RXTEST の印加用。無ければ RXTEST は UNTESTABLE)
 *   手順D  上記から "5"→"CTX" だけを外す (R2 除去済み基板) → TWAI-B が動く。
 *          この配線が本製作 (D オープン、受信専用) と同じ条件なので、TWAI-B PASS = 車載可。
 *
 * 結果表示 (自己検証、シリアル不要):
 *   RGB LED "16" : 緑 点灯 = RXTEST OK かつ 500 kbps ループ PASS (TWAI-B が動いたときは B、動かないときは A)
 *                  シアン 点灯 = RXTEST 未判定 (印加できず: 120Ω か配線)
 *                  赤 点灯 = RXTEST NG (差動受信できない → 車載では受信できない)
 *                  赤 点滅 = RXTEST OK だがループ FAIL      青 = 測定中
 *   LED "IO2"    : 点灯 = ループ PASS   速い点滅 = ループ FAIL
 *   シリアル 115200: 1 サイクル毎に STATIC / RXTEST / TWAI-A / TWAI-B1 / TWAI-B2 / VERDICT、約 3 秒間隔。
 *
 * ボード: Freenove FNK0090 (ESP32-WROOM-32E), arduino-esp32 3.3.x, 追加ライブラリ不要。
 * Arduino IDE: Board "ESP32 Dev Module", Serial Monitor 115200.
 */

#include <Arduino.h>
#include <string.h>
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_arduino_version.h"
#include "esp_rom_gpio.h"          // esp_rom_gpio_connect_out_signal
#include "soc/gpio_sig_map.h"      // TWAI_TX_IDX

// ---- 配線 (シルク名) ----
static const gpio_num_t PIN_TX    = GPIO_NUM_5; // Freenove "5"  → module "CTX"
static const gpio_num_t PIN_RX    = GPIO_NUM_4; // Freenove "4"  ← module "CRX"
static const uint8_t    PIN_CANH  = 34;         // Freenove "34" ← module "CANH" (ADC1, input only)
static const uint8_t    PIN_CANL  = 35;         // Freenove "35" ← module "CANL" (ADC1, input only)
static const uint8_t    PIN_DAC_H = 25;         // Freenove "25" → module "CANH" (DAC1)
static const uint8_t    PIN_DAC_L = 26;         // Freenove "26" → module "CANL" (DAC2)
static const uint8_t    PIN_LED   = 2;          // onboard LED "IO2"
static const uint8_t    PIN_RGB   = 16;         // onboard WS2812 "16"

// ---- テスト条件 ----
static const uint32_t TEST_ID          = 0x595; // 11-bit ID
static const int      FRAMES_PER_CYCLE = 20;    // 20 x TEC+8 = 160 < 256: 1 サイクルでは bus-off に達しない
static const uint32_t TX_WAIT_MS       = 20;
static const uint32_t RX_WAIT_MS       = 20;
static const uint32_t IDLE_MS          = 1000;
static const int      DOM_DIFF_MIN_MV  = 1200;  // dominant と認める CANH-CANL の最小差 (SN65HVD230 spec min 1.5 V @60Ω)
static const int      REC_DIFF_MAX_MV  = 500;   // recessive と認める CANH-CANL の最大差 (受信しきい値 0.5 V)
static const int      APPLY_TOL_MV     = 400;   // RXTEST: DAC 印加電圧の読み戻し許容差

// ================= 構造体 (Arduino のプロトタイプ自動生成対策で全関数より前に置く) =================
struct StaticResult {
  const char *rx_rec, *rx_dom;                       // "4" のレベル: HIGH / LOW / OPEN
  int canh_rec, canl_rec, canh_dom, canl_dom;        // mV
  const char *receiver, *driver;                     // ラベル (数値からの単純判定)
};

struct RxState {
  const char *name;
  uint8_t  vh, vl;          // DAC 値 (3.3 V * v / 255)
  int      mvh, mvl;        // 目標電圧 mV (読み戻しの照合用)
  bool     expectLow;       // 正しい差動受信部なら "4" が LOW になるべきか
};

static const RxState RX_STATES[] = {
  {"S1 rec",     128, 128, 1650, 1650, false},  // diff 0,    CM 1.65 V
  {"S2 dom",     205,  50, 2650,  650, true },  // diff +2.0, CM 1.65 V
  {"S3 rev",      50, 205,  650, 2650, false},  // diff -2.0 (逆極性 = recessive のはず)
  {"S4 recLoCM",  23,  23,  300,  300, false},  // diff 0,    CM 0.3 V (コモンモードだけで反応する受信部はここで LOW になる)
  {"S5 recHiCM", 224, 224, 2900, 2900, false},  // diff 0,    CM 2.9 V
  {"S6 domCar",  232, 108, 3000, 1400, true },  // diff +1.6, CM 2.2 V (車載に近い)
  {"S7 dom1.2",  174,  81, 2250, 1050, true },  // diff +1.2 (しきい値 0.9 V に対する余裕)
};
static const int RX_N = sizeof(RX_STATES) / sizeof(RX_STATES[0]);

enum RxVerdict { RX_OK, RX_NG, RX_UNTESTABLE };

struct RxTestResult {
  int canh[RX_N], canl[RX_N];
  const char *lvl[RX_N];
  bool applied[RX_N], ok[RX_N];
  bool noReadback;                 // "34"/"35" 未配線: 印加できたとみなして判定 (Rev.5)
  int nFail, nNotApplied;
  RxVerdict verdict;
};

struct CycleResult {
  int sent, tx_timeout, tx_err, rx_ok, rx_bad, rx_timeout;
  uint32_t alerts;
  twai_status_info_t st;
  bool pass;
  const char *hint;
};

enum LedVerdict { LED_ALL_OK, LED_RX_UNTESTED, LED_RX_NG, LED_TWAI_FAIL };

// TWAI の送信経路
enum TwaiMode {
  TWAI_VIA_5,             // A : "5" → CTX → モジュールのドライバ → CANH/CANL → 受信部 → CRX → "4"
  TWAI_DIRECT_ANTIPHASE,  // B1: "25" = /TX (CANH), "26" = TX (CANL)  … コモンモード 1.65 V 一定
  TWAI_DIRECT_CANH_HIGH,  // B2: "25" = H 固定 (CANH), "26" = TX (CANL) … recessive で差動 0
};

// ================= 共通 =================
// 内部プルアップで読む → プルダウンで読む: 両方 1 = HIGH に駆動, 両方 0 = LOW に駆動, 1/0 = OPEN (未駆動)
static const char *probeLevel(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
  delayMicroseconds(100);
  int up = digitalRead(pin);
  pinMode(pin, INPUT_PULLDOWN);
  delayMicroseconds(100);
  int dn = digitalRead(pin);
  pinMode(pin, INPUT);
  if (up && dn)   return "HIGH";
  if (!up && !dn) return "LOW";
  if (up && !dn)  return "OPEN";
  return "?";
}

static int readMv(uint8_t pin) {
  uint32_t sum = 0;
  for (int i = 0; i < 2; i++) sum += analogReadMilliVolts(pin);
  return (int)(sum / 2);
}

static void dacOff() {
  dacDisable(PIN_DAC_H);
  dacDisable(PIN_DAC_L);
  pinMode(PIN_DAC_H, INPUT);                    // high-Z
  pinMode(PIN_DAC_L, INPUT);
}

// ================= STATIC (DC) test =================
static void runStatic(StaticResult &s) {
  memset(&s, 0, sizeof(s));
  pinMode((uint8_t)PIN_TX, OUTPUT);
  digitalWrite((uint8_t)PIN_TX, HIGH);          // recessive
  delay(2);
  s.canh_rec = readMv(PIN_CANH);
  s.canl_rec = readMv(PIN_CANL);
  s.rx_rec   = probeLevel((uint8_t)PIN_RX);

  digitalWrite((uint8_t)PIN_TX, LOW);           // dominant (約 0.4 ms だけ)
  delayMicroseconds(30);
  s.canh_dom = readMv(PIN_CANH);
  s.canl_dom = readMv(PIN_CANL);
  s.rx_dom   = probeLevel((uint8_t)PIN_RX);
  digitalWrite((uint8_t)PIN_TX, HIGH);          // recessive に戻す

  int d_rec = s.canh_rec - s.canl_rec;
  int d_dom = s.canh_dom - s.canl_dom;
  bool drvOk = (d_dom >= DOM_DIFF_MIN_MV) && (abs(d_rec) <= REC_DIFF_MAX_MV);
  s.driver = drvOk ? "OK" : "no dominant on CANH/CANL";

  if (!strcmp(s.rx_rec, "LOW"))        s.receiver = "stuck LOW at rest";
  else if (!strcmp(s.rx_rec, "OPEN"))  s.receiver = "open (nothing drives \"4\")";
  else if (!strcmp(s.rx_rec, "HIGH")) {
    if (!strcmp(s.rx_dom, "LOW"))      s.receiver = "follows D (H at rec, L at dom)";
    else if (drvOk)                    s.receiver = "not following (stays HIGH)";
    else                               s.receiver = "idle HIGH, dominant untestable (driver)";
  } else                               s.receiver = "?";
}

static void printStatic(uint32_t cycle, const StaticResult &s) {
  Serial.printf("#%lu STATIC rec: \"4\"=%s CANH=%d CANL=%d (diff %d) | dom: \"4\"=%s CANH=%d CANL=%d (diff %d) mV"
                " -> receiver: %s / driver: %s\n",
                (unsigned long)cycle,
                s.rx_rec, s.canh_rec, s.canl_rec, s.canh_rec - s.canl_rec,
                s.rx_dom, s.canh_dom, s.canl_dom, s.canh_dom - s.canl_dom,
                s.receiver, s.driver);
}

// ================= RXTEST (DAC で差動電圧を印加して受信部を試す) =================
static void runRxTest(RxTestResult &t) {
  memset(&t, 0, sizeof(t));
  pinMode((uint8_t)PIN_TX, OUTPUT);
  digitalWrite((uint8_t)PIN_TX, HIGH);          // ドライバは recessive のまま (印加と衝突させない)

  for (int i = 0; i < RX_N; i++) {
    dacWrite(PIN_DAC_H, RX_STATES[i].vh);
    dacWrite(PIN_DAC_L, RX_STATES[i].vl);
    delay(3);
    t.canh[i] = readMv(PIN_CANH);
    t.canl[i] = readMv(PIN_CANL);
    t.lvl[i]  = probeLevel((uint8_t)PIN_RX);
    t.applied[i] = (abs(t.canh[i] - RX_STATES[i].mvh) <= APPLY_TOL_MV) &&
                   (abs(t.canl[i] - RX_STATES[i].mvl) <= APPLY_TOL_MV);
  }
  // "34"/"35" が全状態で 0.3 V 未満 = 未配線 (浮き)。読み戻しは諦め、DAC 出力が届いているとみなす (要: R2 除去済み)
  t.noReadback = true;
  for (int i = 0; i < RX_N; i++)
    if (t.canh[i] >= 300 || t.canl[i] >= 300) t.noReadback = false;
  if (t.noReadback)
    for (int i = 0; i < RX_N; i++) t.applied[i] = true;

  for (int i = 0; i < RX_N; i++) {
    if (t.applied[i]) {
      t.ok[i] = RX_STATES[i].expectLow ? !strcmp(t.lvl[i], "LOW") : !strcmp(t.lvl[i], "HIGH");
      if (!t.ok[i]) t.nFail++;
    } else {
      t.nNotApplied++;
    }
  }
  dacOff();                                     // TWAI の前に必ず high-Z へ

  if (t.nFail > 0)             t.verdict = RX_NG;
  else if (t.nNotApplied > 0)  t.verdict = RX_UNTESTABLE;
  else                         t.verdict = RX_OK;
}

static void printRxTest(uint32_t cycle, const RxTestResult &t) {
  Serial.printf("#%lu RXTEST", (unsigned long)cycle);
  for (int i = 0; i < RX_N; i++) {
    Serial.printf(" | %s %d/%d \"4\"=%s %s", RX_STATES[i].name, t.canh[i], t.canl[i], t.lvl[i],
                  !t.applied[i] ? "n/a" : (t.ok[i] ? "ok" : "FAIL"));
  }
  const char *v = (t.verdict == RX_OK) ? (t.noReadback ? "OK (differential receiver; 34/35 unwired, DAC assumed applied)" : "OK (differential receiver)") :
                  (t.verdict == RX_NG) ? (t.noReadback ? "NG (34/35 unwired: check 25/26 wiring and R2 removal before trusting this)" : "NG (does not decode differential voltage correctly)") :
                  "UNTESTABLE (voltage not applied: 120 ohm termination on board? wires 25/26/34/35?)";
  Serial.printf(" => receiver: %s\n", v);
}

// ================= TWAI (dynamic) test =================
static const char *stateName(twai_state_t s) {
  switch (s) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
    default:                    return "?";
  }
}

static void ledTesting() {
  digitalWrite(PIN_LED, LOW);
  rgbLedWrite(PIN_RGB, 0, 0, 32);           // 青: 測定中
}

static void ledResult(LedVerdict v) {
  if (v != LED_TWAI_FAIL) {
    digitalWrite(PIN_LED, HIGH);
    if (v == LED_ALL_OK)            rgbLedWrite(PIN_RGB, 0, 40, 0);    // 緑
    else if (v == LED_RX_UNTESTED)  rgbLedWrite(PIN_RGB, 0, 32, 32);   // シアン
    else                            rgbLedWrite(PIN_RGB, 48, 0, 0);    // 赤 点灯
    delay(IDLE_MS);
    return;
  }
  bool on = false;                          // 赤 点滅 + IO2 速い点滅
  uint32_t t0 = millis();
  while (millis() - t0 < IDLE_MS) {
    on = !on;
    digitalWrite(PIN_LED, on ? HIGH : LOW);
    rgbLedWrite(PIN_RGB, on ? 48 : 0, 0, 0);
    delay(100);
  }
}

// ESP32 が CANH/CANL を直接駆動する経路を GPIO マトリクスで作る (TWAI ドライバ install 後に呼ぶ)
static void routeDirectDrive(TwaiMode mode) {
  pinMode(PIN_DAC_H, OUTPUT);
  pinMode(PIN_DAC_L, OUTPUT);
  if (mode == TWAI_DIRECT_ANTIPHASE)
    esp_rom_gpio_connect_out_signal(PIN_DAC_H, TWAI_TX_IDX, true, false);   // CANH = /TX
  else
    digitalWrite(PIN_DAC_H, HIGH);                                           // CANH = 3.3 V 固定
  esp_rom_gpio_connect_out_signal(PIN_DAC_L, TWAI_TX_IDX, false, false);    // CANL = TX
}

static void unrouteDirectDrive() {
  pinMode(PIN_DAC_H, INPUT);                    // high-Z に戻す
  pinMode(PIN_DAC_L, INPUT);
}

static bool runCycle(uint32_t cycle, CycleResult &r, TwaiMode mode) {
  memset(&r, 0, sizeof(r));
  r.hint = "";

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PIN_TX, PIN_RX, TWAI_MODE_NO_ACK);
  g.tx_queue_len   = 1;
  g.rx_queue_len   = 32;
  g.alerts_enabled = TWAI_ALERT_ALL;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t e = twai_driver_install(&g, &t, &f);
  if (e != ESP_OK) {
    Serial.printf("twai_driver_install failed: %s\n", esp_err_to_name(e));
    r.hint = "driver install failed";
    return false;
  }
  gpio_pullup_en(PIN_RX);
  if (mode != TWAI_VIA_5) routeDirectDrive(mode);

  e = twai_start();
  if (e != ESP_OK) {
    Serial.printf("twai_start failed: %s\n", esp_err_to_name(e));
    twai_driver_uninstall();
    if (mode != TWAI_VIA_5) unrouteDirectDrive();
    r.hint = "driver start failed";
    return false;
  }

  for (int i = 0; i < FRAMES_PER_CYCLE; i++) {
    twai_message_t tx = {};
    tx.self = 1;                              // self reception request
    tx.ss   = 1;                              // single shot: 失敗しても再送しない (1 フレーム = 1 試行)
    tx.identifier       = TEST_ID;
    tx.data_length_code = 8;
    tx.data[0] = (uint8_t)cycle;
    tx.data[1] = (uint8_t)i;
    tx.data[2] = 0xA5;
    tx.data[3] = 0x5A;
    tx.data[4] = 0xFF;
    tx.data[5] = 0x00;
    tx.data[6] = (uint8_t)~i;
    tx.data[7] = (uint8_t)(cycle + i);

    e = twai_transmit(&tx, pdMS_TO_TICKS(TX_WAIT_MS));
    if (e == ESP_OK)               r.sent++;
    else if (e == ESP_ERR_TIMEOUT) r.tx_timeout++;   // キューに入らない = 前のフレームが送信開始できない
    else                           r.tx_err++;       // 例: BUS_OFF で not running

    twai_message_t rx = {};
    e = twai_receive(&rx, pdMS_TO_TICKS(RX_WAIT_MS));
    if (e == ESP_OK) {
      bool same = (rx.identifier == tx.identifier) && (rx.data_length_code == 8) &&
                  !rx.extd && !rx.rtr && (memcmp(rx.data, tx.data, 8) == 0);
      if (same) r.rx_ok++; else r.rx_bad++;
    } else {
      r.rx_timeout++;
    }
  }

  twai_message_t extra;
  while (twai_receive(&extra, 0) == ESP_OK) r.rx_bad++;   // 予期しない残りフレーム
  uint32_t a;
  while (twai_read_alerts(&a, 0) == ESP_OK) r.alerts |= a;
  twai_get_status_info(&r.st);

  if (r.st.state == TWAI_STATE_RUNNING) twai_stop();
  e = twai_driver_uninstall();
  if (mode != TWAI_VIA_5) unrouteDirectDrive();
  if (e != ESP_OK) {
    Serial.printf("twai_driver_uninstall failed: %s -> restart\n", esp_err_to_name(e));
    delay(100);
    ESP.restart();
  }

  r.pass = (r.sent == FRAMES_PER_CYCLE) && (r.rx_ok == FRAMES_PER_CYCLE) &&
           (r.rx_bad == 0) && (r.tx_timeout == 0) && (r.tx_err == 0) &&
           (r.st.tx_failed_count == 0) && (r.st.bus_error_count == 0) &&
           (r.st.state == TWAI_STATE_RUNNING);

  // hint は参考情報 (判定は pass のみ)
  if (r.pass)
    r.hint = "";
  else if (r.st.tx_failed_count > 0 || (r.alerts & TWAI_ALERT_BUS_OFF))
    r.hint = "TX not seen on RX: \"4\" open / stuck HIGH / not following \"5\"";
  else if (r.tx_timeout > 0)
    r.hint = "bus never idle: \"4\" LOW (stuck dominant, or open pin reading LOW)";
  else if (r.rx_ok > 0)
    r.hint = "partial: some frames lost";
  else
    r.hint = "no frames received";
  return true;
}

static void printResult(uint32_t cycle, const char *label, const CycleResult &r) {
  Serial.printf("#%lu %s %s sent=%d rx_ok=%d rx_bad=%d rx_to=%d tx_to=%d tx_err=%d | "
                "tx_fail=%lu tec=%lu rec=%lu bus_err=%lu arb_lost=%lu rx_miss=%lu "
                "state=%s alerts=0x%05lX%s%s\n",
                (unsigned long)cycle, label, r.pass ? "PASS" : "FAIL",
                r.sent, r.rx_ok, r.rx_bad, r.rx_timeout, r.tx_timeout, r.tx_err,
                (unsigned long)r.st.tx_failed_count, (unsigned long)r.st.tx_error_counter,
                (unsigned long)r.st.rx_error_counter, (unsigned long)r.st.bus_error_count,
                (unsigned long)r.st.arb_lost_count, (unsigned long)r.st.rx_missed_count,
                stateName(r.st.state), (unsigned long)r.alerts,
                r.hint[0] ? "  hint: " : "", r.hint);
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_DAC_H, INPUT);                // DAC ピンは使うときだけ出力にする
  pinMode(PIN_DAC_L, INPUT);
  ledTesting();
  Serial.begin(115200);
  delay(300);
  analogReadMilliVolts(PIN_CANH);           // ADC 初期化 (初回は時間がかかる)
  analogReadMilliVolts(PIN_CANL);
  Serial.println();
  Serial.println("=== TWAI self-test Rev.5 (STATIC + DAC RXTEST + TWAI-A via module + TWAI-B ESP32 direct drive) ===");
  Serial.printf("arduino-esp32 %s / ESP-IDF %s\n", ESP_ARDUINO_VERSION_STR, esp_get_idf_version());
  Serial.printf("TX=\"5\" RX=\"4\" ADC CANH->\"34\" CANL->\"35\" DAC \"25\"->CANH \"26\"->CANL, 500 kbps, %d frames/cycle, ID=0x%03lX\n",
                FRAMES_PER_CYCLE, (unsigned long)TEST_ID);
  Serial.println("LED: green = RX OK + loop PASS, cyan = RX untestable, red solid = RX NG, red blink = RX OK but loop FAIL");
}

void loop() {
  static uint32_t cycle = 0;
  StaticResult s;
  RxTestResult t;
  CycleResult a, b1, b2;
  cycle++;
  ledTesting();
  runStatic(s);
  printStatic(cycle, s);
  runRxTest(t);
  printRxTest(cycle, t);

  bool aOk = runCycle(cycle, a, TWAI_VIA_5);
  if (aOk) printResult(cycle, "TWAI-A (via \"5\"->CTX)", a);
  else     Serial.printf("#%lu TWAI-A FAIL (%s)\n", (unsigned long)cycle, a.hint);

  // TWAI-B の条件: "5" がモジュールを駆動していない (STATIC dom で CANL が下がらず "4" も HIGH)、かつ RXTEST で電圧が印加できた (120Ω なし)
  // "35" 未配線 (canl が浮き) のときは CANL の判定は使えないので "4" の応答だけで判断する (Rev.5)
  bool fiveDrivesModule = t.noReadback ? !strcmp(s.rx_dom, "LOW")
                                       : ((s.canl_dom < 800) || !strcmp(s.rx_dom, "LOW"));
  bool noTermination    = (t.nNotApplied == 0);
  bool bRan = false;
  if (fiveDrivesModule) {
    Serial.printf("#%lu TWAI-B skipped: \"5\" still drives the module (remove \"5\"->\"CTX\")\n", (unsigned long)cycle);
  } else if (!noTermination) {
    Serial.printf("#%lu TWAI-B skipped: RXTEST voltages not applied (120 ohm termination? wires 25/26/34/35?)\n", (unsigned long)cycle);
  } else {
    bRan = true;
    if (runCycle(cycle, b1, TWAI_DIRECT_ANTIPHASE)) printResult(cycle, "TWAI-B1 (CANH=/TX \"25\", CANL=TX \"26\")", b1);
    else Serial.printf("#%lu TWAI-B1 FAIL (%s)\n", (unsigned long)cycle, b1.hint);
    if (runCycle(cycle, b2, TWAI_DIRECT_CANH_HIGH)) printResult(cycle, "TWAI-B2 (CANH=HIGH \"25\", CANL=TX \"26\")", b2);
    else Serial.printf("#%lu TWAI-B2 FAIL (%s)\n", (unsigned long)cycle, b2.hint);
  }

  bool loopPass = bRan ? (b1.pass || b2.pass) : (aOk && a.pass);
  const char *verdict;
  LedVerdict led;
  if (t.verdict == RX_UNTESTABLE)      { led = LED_RX_UNTESTED; verdict = "RXTEST untestable"; }
  else if (t.verdict == RX_NG)         { led = LED_RX_NG;       verdict = "receiver NG -> not usable in the car"; }
  else if (loopPass && bRan)           { led = LED_ALL_OK;      verdict = "CAR-READY: differential receiver OK + 500 kbps loop PASS with ESP32 direct drive"; }
  else if (loopPass)                   { led = LED_ALL_OK;      verdict = "receiver OK + loop PASS via module driver (remove \"5\"->\"CTX\" to run TWAI-B)"; }
  else                                 { led = LED_TWAI_FAIL;   verdict = bRan ? "receiver OK at DC but 500 kbps loop FAIL" : "receiver OK at DC, loop via module FAIL (run TWAI-B: remove \"5\"->\"CTX\")"; }
  Serial.printf("#%lu VERDICT %s\n", (unsigned long)cycle, verdict);
  ledResult(led);
}
