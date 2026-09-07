/*
 * can_monitor.ino  -  車載 CAN 受信モニタ / CAN-ID 特定ツール  Rev.4  (2026-09-07)
 *
 * 目的:
 *  1. 車両に接続した状態で CAN フレームが本当に受信できているかを、ESP32 自身の判定 (LED + シリアル) で示す。
 *  2. どの CAN-ID のどのバイトが「操作 (ハンドル・ブレーキ・アクセル)」で変化するかを見つける。
 *
 * 動作 (操作は一切不要。書き込んで車両に挿すだけ):
 *  [WAIT]     最初のフレームが来るまで待つ。RGB LED 青。5 秒ごとに HB 行 (受信 0 なら原因ヒント付き)。
 *  [BASELINE] 最初のフレームから BASELINE_SEC 秒間、何も操作せずに待つ。
 *             この間の各バイトの最小値〜最大値を ID ごとに記憶 (Rev.4: 範囲で覚える)。
 *             変化したバイト (カウンタ・チェックサム・回転数・センサのジッタ) は「ノイズ」表示 (小文字)。
 *             終了時に ID 一覧表を出す。エンジンをかけたままでも可。
 *  [WATCH]    バイトの値が BASELINE の範囲 ± RANGE_MARGIN を外れた瞬間だけ EVENT 行を出す (ID ごと最短 EVENT_MIN_MS 間隔)。
 *             ジッタで少し動くだけの舵角バイトも、ハンドルを切れば範囲を外れて EVENT になる。
 *             カウンタ / チェックサムは BASELINE 中に全範囲を取るので EVENT にならない。
 *             → ハンドルを切る / ブレーキを踏む / アクセルを踏むと、対応する ID の EVENT 行だけが流れる。
 *             何もしなければ EVENT は出ない (出るなら BASELINE 中に動いていなかっただけのノイズ)。
 *             5 秒ごとに HB 行、TABLE_MS ごとに全 ID 一覧表。
 *
 * 表示規則:
 *   バイト表記  大文字 "3C"  = BASELINE 中に変化しなかったバイト
 *               小文字 "3c"  = ノイズ (BASELINE 中に変化したバイト)
 *               角括弧 [3C]  = 今回の EVENT で変化した (ノイズ以外の) バイト ← これが探し物
 *   ID 表記     16 進 (29 ビットは 8 桁, 11 ビットは 3 桁) と 10 進の両方 (RaceChrono の PID 入力用)
 *
 * 受信 0 件のときの切り分け (Rev.2、HB 行の 2 行目に毎回表示):
 *   "4" の生レベルを 20 ms 間サンプリングし、エッジ数と固定レベルを出す。
 *     edges > 0            → 信号は "4" まで来ている。TWAI が復号できていない (ビットレート違い等)
 *     edges = 0, level = H → モジュールは生きているがバスにトラフィックが無い (IG OFF / CANH,CANL がバスに届いていない)
 *     edges = 0, level = L → CRX が駆動されていない (モジュール 3V3/GND 未給電 or CRX→"4" 未接続。開放の "4" はこの基板では L と読める)
 *   任意: "34"→CANH, "35"→CANL をジャンパで足すと CANH/CANL の電圧 (mV) も出る。
 *     IG ON でバスが届いていれば両方 ≈ 2500 mV (recessive)。0 mV 付近なら OBD ピン 6/14 がモジュールまで来ていない。
 *     未配線なら値は無意味 (浮き) なので無視する。
 *   alerts=0x1000 (ERR_PASS) は LISTEN_ONLY の仕様 (コントローラは常にエラーパッシブ扱い) で異常ではない。
 *
 * MARK (Rev.3、任意): シリアルに文字列を送って Enter すると "MARK t=..s <文字列>" の行が出る。
 *   操作の前に "steer" "brake" "throttle" などを送っておくと、後でログを読むときに EVENT と対応づけやすい。
 *   送らなくても動作は変わらない (対話待ちは無い)。
 *
 * LED (自己判定、シリアル不要):
 *   RGB "16"  青 = フレーム未受信      緑 = 受信中・エラー無し
 *             黄 = 受信中だがバスエラーあり  赤 = 2 秒以上フレーム無し (受信できていた後に途絶)
 *   LED "IO2" 100 フレームごとに反転 (受信レートの目視)
 *
 * 配線: 本製作と同一 (Freenove "4" ← モジュール CRX / R, "5" → モジュール CTX / D)。
 *       このモジュールは D 開放だと R が H を出せないので D は "5" に接続する。LISTEN_ONLY では "5" は常に H (レセッシブ)。
 *       受信専用 (TWAI_MODE_LISTEN_ONLY)。ACK も送らないので車両側に一切影響しない。
 *       シリアルモニタを使うときは USB 給電のみ (OBD ピン 16 の 12 V 線は外す)。
 *
 * ボード: Freenove FNK0090 (ESP32-WROOM-32E), arduino-esp32 3.3.x, 追加ライブラリ不要。
 * Arduino IDE: Board "ESP32 Dev Module", Serial Monitor 115200.
 */

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include "driver/twai.h"
#include "driver/gpio.h"

// ---- 配線 (シルク名) ----
static const gpio_num_t PIN_TX  = GPIO_NUM_5;  // Freenove "5"  → module "CTX" (LISTEN_ONLY なので常に H)
static const gpio_num_t PIN_RX  = GPIO_NUM_4;  // Freenove "4"  ← module "CRX"
static const uint8_t    PIN_LED = 2;           // onboard LED "IO2"
static const uint8_t    PIN_RGB = 16;          // onboard WS2812 "16"
static const uint8_t    PIN_CANH_ADC = 34;     // Freenove "34" ← module "CANH" (任意、切り分け用)
static const uint8_t    PIN_CANL_ADC = 35;     // Freenove "35" ← module "CANL" (任意、切り分け用)

// ---- 動作パラメータ ----
static const uint32_t SERIAL_BAUD    = 115200;
static const uint32_t BASELINE_SEC   = 10;     // ノイズ学習時間 (この間は何も操作しない)
static const uint32_t HB_MS          = 5000;   // HB 行の間隔
static const uint32_t TABLE_MS       = 30000;  // 全 ID 一覧表の間隔
static const uint32_t EVENT_MIN_MS   = 250;    // 同一 ID の EVENT 行の最短間隔
static const uint32_t NO_RX_RED_MS   = 2000;   // この時間フレームが無ければ赤
static const int      MAX_IDS        = 160;
static const uint32_t RX_QUEUE_LEN   = 512;    // 一覧表の出力中 (~300 ms) に取りこぼさない量
static const int      RANGE_MARGIN   = 1;      // BASELINE の min/max からこれ以上外れたら EVENT

// ---- 候補 ID (手順書 3 章、Fiat 500/312 系の公開情報。Abarth 595 では未確認) ----
static const uint32_t ID_RPM    = 0x0618A001;  // bytes 2-3 big-endian
static const uint32_t ID_SPEED  = 0x0210A006;  // bytes 4-5 big-endian / 128 = km/h
static const uint32_t ID_WHEELS = 0x0218A006;  // 4 x 16-bit
static const uint32_t ID_BRAKE  = 0x0810A000;  // byte 2 upper nibble

// ================= 構造体 (Arduino のプロトタイプ自動生成対策で全関数より前に置く) =================
struct IdEntry {
  uint32_t id;
  bool     ext;
  uint8_t  dlc;
  uint8_t  last[8];
  uint8_t  bmin[8], bmax[8];   // BASELINE 中の各バイトの範囲
  uint32_t total;        // 累計
  uint32_t window;       // HB 間の件数 (レート計算用)
  uint8_t  noiseMask;    // bit n = byte n が BASELINE 中に変化した
  uint8_t  pendingMask;  // bit n = byte n が前回 EVENT 以降に変化した (ノイズ除外)
  uint32_t lastEventMs;
  uint32_t firstSeenMs;
  bool     seenInBaseline;
};

enum Phase { PH_WAIT, PH_BASELINE, PH_WATCH };

// ================= 状態 =================
static IdEntry  g_ids[MAX_IDS];
static int      g_numIds = 0;
static int      g_overflowIds = 0;
static Phase    g_phase = PH_WAIT;
static uint32_t g_firstFrameMs = 0;
static uint32_t g_lastFrameMs  = 0;
static uint32_t g_totalFrames  = 0;
static uint32_t g_windowFrames = 0;
static uint32_t g_lastHbMs = 0, g_lastTableMs = 0;
static uint32_t g_alerts = 0;
static uint32_t g_ledToggleCount = 0;
static bool     g_ledState = false;
static uint32_t g_hbWindowMs = 0;

// ================= ユーティリティ =================
static void idToStr(const IdEntry &e, char *buf, size_t n) {
  if (e.ext) snprintf(buf, n, "%08lX", (unsigned long)e.id);
  else       snprintf(buf, n, "%03lX",  (unsigned long)e.id);
}

// data 表示: 大文字 = 安定, 小文字 = ノイズ, [..] = 今回変化
static void dataToStr(const IdEntry &e, uint8_t changedMask, char *buf, size_t n) {
  size_t pos = 0;
  for (int i = 0; i < 8; i++) {
    if (pos + 6 >= n) break;
    if (i >= e.dlc) { pos += snprintf(buf + pos, n - pos, " --"); continue; }
    bool noise   = e.noiseMask & (1 << i);
    bool changed = changedMask & (1 << i);
    const char *fmt = changed ? "[%02X]" : (noise ? " %02x" : " %02X");
    pos += snprintf(buf + pos, n - pos, fmt, e.last[i]);
  }
  buf[pos] = 0;
}

static int cmpId(const void *a, const void *b) {
  const IdEntry *x = (const IdEntry *)a, *y = (const IdEntry *)b;
  if (x->ext != y->ext) return x->ext ? 1 : -1;
  if (x->id < y->id) return -1;
  if (x->id > y->id) return 1;
  return 0;
}

static IdEntry *findEntry(uint32_t id, bool ext) {
  for (int i = 0; i < g_numIds; i++)
    if (g_ids[i].id == id && g_ids[i].ext == ext) return &g_ids[i];
  return nullptr;
}

static const char *stateName(twai_state_t s) {
  switch (s) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
    default:                    return "?";
  }
}

static void setRgb(uint8_t r, uint8_t g, uint8_t b) { rgbLedWrite(PIN_RGB, r, g, b); }

static uint16_t be16(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }

// "4" を 20 ms 間ポーリングしてエッジ数と最後のレベルを返す (TWAI と並行して GPIO 入力は読める)
static uint32_t sampleRxPin(int *lastLevel, uint32_t windowUs = 20000) {
  uint32_t edges = 0;
  int prev = gpio_get_level(PIN_RX);
  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < windowUs) {
    int v = gpio_get_level(PIN_RX);
    if (v != prev) { edges++; prev = v; }
  }
  *lastLevel = prev;
  return edges;
}

static void printRxPinDiag() {
  int lvl = 0;
  uint32_t edges = sampleRxPin(&lvl);
  uint32_t h = analogReadMilliVolts(PIN_CANH_ADC);
  uint32_t l = analogReadMilliVolts(PIN_CANL_ADC);
  Serial.printf("  pin\"4\": level=%s edges/20ms=%lu | \"34\"(CANH)=%lu mV \"35\"(CANL)=%lu mV (34/35 未配線なら無視)\r\n",
                lvl ? "H" : "L", (unsigned long)edges, (unsigned long)h, (unsigned long)l);
  if (edges > 0)
    Serial.println("  → 信号は \"4\" まで来ている。TWAI が復号できていない (ビットレート / CANH-CANL 逆)");
  else if (lvl)
    Serial.println("  → \"4\"=H 固定: モジュールは動作、バスに信号が無い (IG ON? OBD ピン 6/14 がモジュール CANH/CANL まで来ているか)");
  else
    Serial.println("  → \"4\"=L 固定: CRX が駆動されていない (モジュール 3V3/GND 給電、CRX→\"4\" の接続)");
}

// ================= 表示 =================
static void printTable(const char *title) {
  static IdEntry sorted[MAX_IDS];   // static: loop タスクのスタックに置かない
  memcpy(sorted, g_ids, sizeof(IdEntry) * g_numIds);
  qsort(sorted, g_numIds, sizeof(IdEntry), cmpId);

  uint32_t now = millis();
  Serial.printf("\n==== %s  t=%.1fs  IDs=%d%s ====\r\n", title, millis() / 1000.0, g_numIds,
                g_overflowIds ? " (+overflow)" : "");
  Serial.println("  ID(hex)   ID(dec)     type   Hz  len  data (UPPER=stable lower=noise)   noise  first");
  for (int i = 0; i < g_numIds; i++) {
    const IdEntry &e = sorted[i];
    char idbuf[12], dbuf[48];
    idToStr(e, idbuf, sizeof idbuf);
    dataToStr(e, 0, dbuf, sizeof dbuf);
    int noiseCnt = __builtin_popcount(e.noiseMask & ((1 << e.dlc) - 1));
    uint32_t aliveMs = now - e.firstSeenMs; if (aliveMs < 1000) aliveMs = 1000;
    Serial.printf("  %-8s %10lu  %s  %5.1f  %d  %s   %d/%d  %s\r\n",
                  idbuf, (unsigned long)e.id, e.ext ? "29bit" : "11bit",
                  e.total * 1000.0 / aliveMs, e.dlc, dbuf, noiseCnt, e.dlc,
                  e.seenInBaseline ? "" : "NEW(after baseline)");
  }
  Serial.println();
}

static void printCandidates() {
  IdEntry *e;
  bool any = false;
  if ((e = findEntry(ID_RPM, true)) && e->dlc >= 4) {
    Serial.printf("  cand RPM   0618A001 b2-3 = %u", be16(&e->last[2])); any = true;
  }
  if ((e = findEntry(ID_SPEED, true)) && e->dlc >= 6) {
    Serial.printf("  cand SPEED 0210A006 b4-5/128 = %.1f km/h", be16(&e->last[4]) / 128.0); any = true;
  }
  if ((e = findEntry(ID_WHEELS, true)) && e->dlc >= 8) {
    Serial.printf("  cand WHEELS 0218A006 = %u %u %u %u", be16(&e->last[0]), be16(&e->last[2]),
                  be16(&e->last[4]), be16(&e->last[6])); any = true;
  }
  if ((e = findEntry(ID_BRAKE, true)) && e->dlc >= 3) {
    Serial.printf("  cand BRAKE 0810A000 b2>>4 = %u", e->last[2] >> 4); any = true;
  }
  if (any) Serial.println();
}

static void printHeartbeat(uint32_t now) {
  twai_status_info_t st = {};
  twai_get_status_info(&st);
  uint32_t a = 0;
  while (twai_read_alerts(&a, 0) == ESP_OK) g_alerts |= a;

  uint32_t winMs = now - g_lastHbMs;
  g_hbWindowMs = winMs ? winMs : 1;
  float fps = g_windowFrames * 1000.0 / g_hbWindowMs;

  const char *ph = g_phase == PH_WAIT ? "WAIT" : g_phase == PH_BASELINE ? "BASELINE" : "WATCH";
  Serial.printf("HB t=%.1fs %-8s fps=%.0f total=%lu ids=%d | bus=%s rx_err=%lu bus_err=%lu missed=%lu qfull=%lu alerts=0x%lx\r\n",
                now / 1000.0, ph, fps, (unsigned long)g_totalFrames, g_numIds,
                stateName(st.state), (unsigned long)st.rx_error_counter, (unsigned long)st.bus_error_count,
                (unsigned long)st.rx_missed_count, (unsigned long)st.rx_overrun_count, (unsigned long)g_alerts);

  bool rxRecent = g_totalFrames && (now - g_lastFrameMs) < NO_RX_RED_MS;
  bool busErr   = st.bus_error_count > 0 || st.state != TWAI_STATE_RUNNING;

  if (g_windowFrames == 0) {
    if (g_totalFrames == 0) {
      Serial.println("  RX NONE: フレーム未受信。");
      printRxPinDiag();
      if (st.bus_error_count > 0 || st.rx_error_counter > 0)
        Serial.println("  hint: バスエラーあり = 信号は来ているが復号できない → ビットレート違い (B-CAN 50k のピン 1/9 に挿していないか) / CANH-CANL 逆");
      else
        Serial.println("  hint: エラーも無し = 信号が届いていない → IG ON か / OBD ピン 6=CANH 14=CANL / モジュール CRX→\"4\" / モジュール 3V3・GND");
    } else {
      Serial.println("  RX LOST: 受信が途絶えた (IG OFF? コネクタ抜け?)");
      printRxPinDiag();
    }
  } else {
    Serial.printf("  RX OK: %s\r\n", busErr ? "受信中だがバスエラーあり (bus_err を監視)" : "受信中・エラー無し");
    printCandidates();
  }

  if (!g_totalFrames)   setRgb(0, 0, 32);      // 青
  else if (!rxRecent)   setRgb(48, 0, 0);      // 赤
  else if (busErr)      setRgb(40, 32, 0);     // 黄
  else                  setRgb(0, 40, 0);      // 緑

  g_windowFrames = 0;
  for (int i = 0; i < g_numIds; i++) g_ids[i].window = 0;
  g_lastHbMs = now;
}

static void printEvent(IdEntry &e, uint32_t now) {
  char idbuf[12], dbuf[48];
  idToStr(e, idbuf, sizeof idbuf);
  dataToStr(e, e.pendingMask, dbuf, sizeof dbuf);
  Serial.printf("EVENT t=%.1fs %-8s (%lu) %s\r\n", now / 1000.0, idbuf, (unsigned long)e.id, dbuf);
  e.pendingMask = 0;
  e.lastEventMs = now;
}

// ================= 受信処理 =================
static void handleFrame(const twai_message_t &m, uint32_t now) {
  g_totalFrames++;
  g_windowFrames++;
  g_lastFrameMs = now;
  if (++g_ledToggleCount >= 100) { g_ledToggleCount = 0; g_ledState = !g_ledState; digitalWrite(PIN_LED, g_ledState); }

  if (g_phase == PH_WAIT) {
    g_phase = PH_BASELINE;
    g_firstFrameMs = now;
    Serial.printf("\nFIRST FRAME at t=%.1fs → BASELINE 開始: %lu 秒間なにも操作しないでください\r\n\r\n",
                  now / 1000.0, (unsigned long)BASELINE_SEC);
  }

  bool ext = m.extd;
  IdEntry *e = findEntry(m.identifier, ext);
  if (!e) {
    if (g_numIds >= MAX_IDS) { g_overflowIds++; return; }
    e = &g_ids[g_numIds++];
    memset(e, 0, sizeof *e);
    e->id = m.identifier; e->ext = ext; e->dlc = m.data_length_code;
    memcpy(e->last, m.data, 8);
    memcpy(e->bmin, m.data, 8);
    memcpy(e->bmax, m.data, 8);
    e->firstSeenMs = now;
    e->seenInBaseline = (g_phase != PH_WATCH);
    e->total = 1; e->window = 1;
    if (g_phase == PH_WATCH) {
      char idbuf[12], dbuf[48];
      idToStr(*e, idbuf, sizeof idbuf);
      dataToStr(*e, 0, dbuf, sizeof dbuf);
      Serial.printf("NEWID t=%.1fs %-8s (%lu) len=%d %s\r\n", now / 1000.0, idbuf, (unsigned long)e->id, e->dlc, dbuf);
    }
    return;
  }

  e->total++; e->window++;
  uint8_t diff = 0;
  uint8_t dlc = m.data_length_code;
  for (int i = 0; i < dlc && i < 8; i++)
    if (e->last[i] != m.data[i]) diff |= (1 << i);
  if (e->dlc != dlc) diff |= 0xFF;
  memcpy(e->last, m.data, 8);
  e->dlc = dlc;

  if (g_phase == PH_BASELINE) {
    e->noiseMask |= diff;
    for (int i = 0; i < dlc && i < 8; i++) {
      if (m.data[i] < e->bmin[i]) e->bmin[i] = m.data[i];
      if (m.data[i] > e->bmax[i]) e->bmax[i] = m.data[i];
    }
  } else {
    uint8_t out = 0;
    for (int i = 0; i < dlc && i < 8; i++) {
      if (!(diff & (1 << i))) continue;
      int lo = (int)e->bmin[i] - RANGE_MARGIN, hi = (int)e->bmax[i] + RANGE_MARGIN;
      if (m.data[i] < lo || m.data[i] > hi) out |= (1 << i);
    }
    if (!e->seenInBaseline) out |= diff;        // BASELINE 後に現れた ID は範囲が無いので変化＝EVENT
    e->pendingMask |= out;
    if (e->pendingMask && (now - e->lastEventMs) >= EVENT_MIN_MS) printEvent(*e, now);
  }
}

// ================= setup / loop =================
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  setRgb(0, 0, 32);

  Serial.begin(SERIAL_BAUD);
  delay(300);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_CANH_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_CANL_ADC, ADC_11db);
  Serial.println("\n\ncan_monitor Rev.4 - listen-only CAN monitor, 500 kbps, RX=\"4\"");
  Serial.println("phase WAIT: waiting for the first frame ...");

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PIN_TX, PIN_RX, TWAI_MODE_LISTEN_ONLY);
  g.rx_queue_len = RX_QUEUE_LEN;
  g.alerts_enabled = TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ERR_PASS |
                     TWAI_ALERT_BUS_OFF | TWAI_ALERT_ARB_LOST;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t e = twai_driver_install(&g, &t, &f);
  if (e != ESP_OK) { Serial.printf("twai_driver_install failed: %s\r\n", esp_err_to_name(e)); while (1) { setRgb(48, 0, 0); delay(200); setRgb(0, 0, 0); delay(200); } }
  e = twai_start();
  if (e != ESP_OK) { Serial.printf("twai_start failed: %s\r\n", esp_err_to_name(e)); while (1) { setRgb(48, 0, 0); delay(200); setRgb(0, 0, 0); delay(200); } }
  Serial.println("TWAI driver started (LISTEN_ONLY, ACCEPT_ALL).");

  g_lastHbMs = g_lastTableMs = millis();
}

void loop() {
  twai_message_t m;
  // 溜まっている分をまとめて処理
  for (int n = 0; n < 64 && twai_receive(&m, 0) == ESP_OK; n++) handleFrame(m, millis());

  uint32_t now = millis();

  if (g_phase == PH_BASELINE && (now - g_firstFrameMs) >= BASELINE_SEC * 1000) {
    g_phase = PH_WATCH;
    printTable("BASELINE DONE - ID inventory");
    Serial.println("phase WATCH: 操作してください。ノイズ以外のバイトが変わった ID だけ EVENT 行が出ます。\r\n");
    g_lastTableMs = now;
  }

  if (now - g_lastHbMs >= HB_MS) printHeartbeat(now);

  // MARK: シリアル入力を 1 行受けてタイムスタンプ付きで echo (ログ注釈用、任意)
  static char markBuf[48]; static uint8_t markLen = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (markLen) { markBuf[markLen] = 0; Serial.printf("MARK t=%.1fs %s\r\n", now / 1000.0, markBuf); markLen = 0; }
    } else if (markLen < sizeof(markBuf) - 1 && c >= ' ') markBuf[markLen++] = c;
  }

  if (g_phase == PH_WATCH && now - g_lastTableMs >= TABLE_MS) {
    printTable("TABLE");
    g_lastTableMs = now;
  }

  // 受信が無いときのみ待つ (受信中は即ループ)
  if (twai_receive(&m, pdMS_TO_TICKS(5)) == ESP_OK) handleFrame(m, millis());
}
