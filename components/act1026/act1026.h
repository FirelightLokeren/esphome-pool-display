#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/log.h"

#include <vector>
#include <queue>

namespace esphome {
namespace act1026 {

// espbt is ESPHome's internal alias for esphome::esp32_ble_tracker
// Replicate it here so our code matches ESPHome's own calling conventions.
namespace espbt = esphome::esp32_ble_tracker;

static const char *const TAG = "act1026";

// ACT1026 actual BLE UUIDs (confirmed via nRF Connect log)
// Command service: 0x00FA
// Write char 0xFA02: WRITE + WRITE NO RESPONSE  — send commands here
// Notify char 0xFA03: NOTIFY — panel echoes commands back here as ACK
static const uint16_t SERVICE_UUID = 0x00FA;
static const uint16_t WRITE_CHAR   = 0xFA02;
static const uint16_t NOTIFY_CHAR  = 0xFA03;

// Minimum gap between consecutive ATT writes (ms)
static const uint32_t WRITE_GAP_MS = 30;

// ─── Protocol helpers ─────────────────────────────────────────────────────────

static uint32_t crc32_bytes(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
  }
  return ~crc;
}

// Build a raw command frame — NO length prefix.
// The panel protocol sends the raw payload bytes directly to 0xFA02.
// The 2-byte length prefix used by pypixelcolor's single_window_plan is NOT sent.
static std::vector<uint8_t> raw_cmd(std::initializer_list<uint8_t> bytes) {
  return std::vector<uint8_t>(bytes);
}

// ─── Command builders (exact pypixelcolor byte sequences) ─────────────────────

static std::vector<uint8_t> cmd_set_time(uint8_t h, uint8_t m, uint8_t s) {
  return raw_cmd({8, 0, 1, 0x80, h, m, s, 0});
}

static std::vector<uint8_t> cmd_set_clock_mode(uint8_t style,
                                                uint8_t year, uint8_t month,
                                                uint8_t day, uint8_t dow,
                                                bool fmt24 = true, bool show_date = true) {
  return raw_cmd({11, 0, 6, 1,
                  style,
                  (uint8_t)(fmt24 ? 1 : 0),
                  (uint8_t)(show_date ? 1 : 0),
                  year, month, day, dow});
}

static std::vector<uint8_t> cmd_set_brightness(uint8_t level) {
  return raw_cmd({5, 0, 4, 0x80, level});
}

static std::vector<uint8_t> cmd_set_power(bool on) {
  return raw_cmd({5, 0, 7, 1, (uint8_t)(on ? 1 : 0)});
}

static std::vector<uint8_t> cmd_set_orientation(uint8_t o) {
  return raw_cmd({5, 0, 6, 0x80, o});
}

// Multi-window image/large-data framing — raw frame content, no length prefix
static std::vector<std::vector<uint8_t>> make_send_plan(const std::vector<uint8_t> &data,
                                                         uint8_t slot = 0,
                                                         size_t win = 12 * 1024) {
  uint32_t crc  = crc32_bytes(data.data(), data.size());
  uint32_t sz   = (uint32_t)data.size();
  std::vector<std::vector<uint8_t>> out;
  size_t pos = 0; uint32_t idx = 0;
  while (pos < data.size()) {
    size_t end  = std::min(pos + win, data.size());
    uint8_t opt = (idx == 0) ? 0x00 : 0x02;
    std::vector<uint8_t> fc = {
        0x00, 0x01, opt,
        (uint8_t)(sz), (uint8_t)(sz>>8), (uint8_t)(sz>>16), (uint8_t)(sz>>24),
        (uint8_t)(crc),(uint8_t)(crc>>8),(uint8_t)(crc>>16),(uint8_t)(crc>>24),
        0x00, slot};
    fc.insert(fc.end(), data.begin()+pos, data.begin()+end);
    out.push_back(fc);  // raw, no length prefix
    pos = end; idx++;
  }
  return out;
}

// ─── Write queue ──────────────────────────────────────────────────────────────

struct WriteFrame {
  std::vector<uint8_t> data;
  bool ack;  // true = ESP_GATT_WRITE_TYPE_RSP
};

// ─── Component ────────────────────────────────────────────────────────────────

class ACT1026 : public Component, public ble_client::BLEClientNode {
 public:
  void set_time_source(time::RealTimeClock *t) { time_ = t; }
  void set_brightness(uint8_t v)   { brightness_ = v; }
  void set_clock_style(uint8_t v)  { clock_style_ = v; }
  void set_auto_sync_time(bool v)  { auto_sync_time_ = v; }

  // ── ESPHome lifecycle ──────────────────────────────────────────────────────

  void setup() override {
    ESP_LOGCONFIG(TAG, "ACT1026 setup");
    // The BLEClient may have already connected before our setup() ran —
    // State: ESTABLISHED in dump_config means we missed OPEN/SEARCH_CMPL events.
    this->check_already_connected_();
  }

  void loop() override {
    this->check_already_connected_();

    if (!connected_ || write_char_handle_ == 0)
      return;

    // Stuck-pending watchdog
    if (write_pending_ && (millis() - last_write_ms_ > 3000)) {
      ESP_LOGW(TAG, "Write ACK timeout — clearing stuck pending");
      write_pending_ = false;
    }

    // Drain queue with inter-write gap
    if (!write_queue_.empty() && !write_pending_) {
      if (millis() - last_write_ms_ >= WRITE_GAP_MS)
        flush_next_();
    }
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "ACT1026 BLE LED Panel:");
    ESP_LOGCONFIG(TAG, "  Brightness: %d", brightness_);
    ESP_LOGCONFIG(TAG, "  Clock style: %d", clock_style_);
    ESP_LOGCONFIG(TAG, "  Auto sync time: %s", auto_sync_time_ ? "yes" : "no");
    ESP_LOGCONFIG(TAG, "  Connected: %s", connected_ ? "yes" : "no");
    ESP_LOGCONFIG(TAG, "  Write char handle: 0x%04X", write_char_handle_);
  }

  // ── BLEClientNode GATT event handler ──────────────────────────────────────

  void gattc_event_handler(esp_gattc_cb_event_t event,
                            esp_gatt_if_t gattc_if,
                            esp_ble_gattc_cb_param_t *param) override {
    switch (event) {

      case ESP_GATTC_OPEN_EVT:
        ESP_LOGI(TAG, "OPEN_EVT status=%d", param->open.status);
        if (param->open.status != ESP_GATT_OK) break;
        gattc_if_ = gattc_if;
        conn_id_   = param->open.conn_id;
        break;

      case ESP_GATTC_SEARCH_CMPL_EVT:
        gattc_if_ = gattc_if;
        conn_id_   = param->search_cmpl.conn_id;
        bootstrapped_ = true;
        discover_handles_();
        break;

      case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        ESP_LOGI(TAG, "Notify registered — panel ready");
        on_ready_();
        break;

      case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGV(TAG, "Notify/ACK len=%d", param->notify.value_len);
        write_pending_ = false;
        break;

      case ESP_GATTC_WRITE_CHAR_EVT:
        if (param->write.status != ESP_GATT_OK)
          ESP_LOGW(TAG, "Write status=%d", param->write.status);
        else
          ESP_LOGV(TAG, "Write confirmed");
        write_pending_ = false;
        break;

      case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGW(TAG, "Disconnected — will reconnect");
        connected_          = false;
        bootstrapped_       = false;
        write_char_handle_  = 0;
        notify_char_handle_ = 0;
        write_queue_        = {};
        write_pending_      = false;
        break;

      default:
        break;
    }
  }

  // ── Public API ────────────────────────────────────────────────────────────

  void send_clock_mode(uint8_t style = 0xFF) {
    if (!connected_) { ESP_LOGD(TAG, "not connected"); return; }
    uint8_t s = (style == 0xFF) ? clock_style_ : style;
    uint8_t yr=0,mo=1,dy=1,dow=1,hr=0,mi=0,sc=0;
    if (time_ && time_->now().is_valid()) {
      auto now = time_->now();
      yr=now.year%100; mo=now.month; dy=now.day_of_month;
      dow=now.day_of_week; hr=now.hour; mi=now.minute; sc=now.second;
    }
    enqueue_(cmd_set_time(hr,mi,sc));
    enqueue_(cmd_set_clock_mode(s,yr,mo,dy,dow));
  }

  void send_brightness(uint8_t level) {
    if (!connected_) return;
    enqueue_(cmd_set_brightness(level));
  }

  void send_power(bool on) {
    if (!connected_) return;
    enqueue_(cmd_set_power(on));
  }

  void send_orientation(uint8_t o) {
    if (!connected_) return;
    enqueue_(cmd_set_orientation(o));
  }

  void send_solid_color(uint8_t r, uint8_t g, uint8_t b) {
    if (!connected_) return;
    uint16_t rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    auto bmp = make_bmp_solid_(rgb565, 32, 32);
    for (auto &f : make_send_plan(bmp))
      enqueue_(f);
  }

  bool is_connected() const { return connected_; }

 protected:
  time::RealTimeClock *time_{nullptr};
  uint8_t  brightness_{70};
  uint8_t  clock_style_{1};
  bool     auto_sync_time_{true};

  bool     connected_{false};
  bool     bootstrapped_{false};

  esp_gatt_if_t gattc_if_{0};
  uint16_t      conn_id_{0};
  uint16_t      write_char_handle_{0};
  uint16_t      notify_char_handle_{0};

  std::queue<WriteFrame> write_queue_;
  bool     write_pending_{false};
  uint32_t last_write_ms_{0};

  // ── Boot-race fix ──────────────────────────────────────────────────────────
  // If BLEClient already ESTABLISHED before our setup() ran, we missed all
  // GATT events. Use the parent's cached state directly.

  void check_already_connected_() {
    if (bootstrapped_ || parent_ == nullptr) return;
    if (!parent()->connected()) return;
    gattc_if_ = parent()->get_gattc_if();
    conn_id_  = parent()->get_conn_id();
    ESP_LOGI(TAG, "Bootstrapping from existing connection");
    bootstrapped_ = true;
    discover_handles_();
  }

  // ── Characteristic handle lookup via ESPHome's cached service table ────────

  void discover_handles_() {
    auto svc_uuid   = espbt::ESPBTUUID::from_uint16(SERVICE_UUID);
    auto write_uuid = espbt::ESPBTUUID::from_uint16(WRITE_CHAR);
    auto notif_uuid = espbt::ESPBTUUID::from_uint16(NOTIFY_CHAR);

    auto *wc = parent()->get_characteristic(svc_uuid, write_uuid);
    auto *nc = parent()->get_characteristic(svc_uuid, notif_uuid);

    if (wc == nullptr) {
      ESP_LOGE(TAG, "0xFFF1 write char NOT FOUND — check service UUIDs in BLE scan logs");
      // Enable DEBUG level logging and look for [esp32_ble_client] service/char lines
      // to find the actual UUIDs your panel uses.
      return;
    }

    write_char_handle_ = wc->handle;
    ESP_LOGI(TAG, "Write char handle: 0x%04X", write_char_handle_);
    connected_ = true;

    if (nc != nullptr) {
      notify_char_handle_ = nc->handle;
      ESP_LOGI(TAG, "Notify char handle: 0x%04X", notify_char_handle_);
      esp_ble_gattc_register_for_notify(gattc_if_,
                                         parent()->get_remote_bda(),
                                         notify_char_handle_);
      // on_ready_() triggered from REG_FOR_NOTIFY_EVT
    } else {
      ESP_LOGW(TAG, "0xFFF2 notify char not found — no ACKs");
      on_ready_();
    }
  }

  void on_ready_() {
    ESP_LOGI(TAG, "Panel ready — sending initial commands");
    // Request device info first (panel responds with width/height/firmware)
    enqueue_(raw_cmd({8, 0, 1, 0x80, 0, 0, 0, 0}));
    send_brightness(brightness_);
    if (auto_sync_time_)
      send_clock_mode();
  }

  // ── Write queue ───────────────────────────────────────────────────────────

  // 0xAE01 is WRITE NO RESPONSE — use ack=false by default.
  // write_pending_ is cleared by notify from 0xAE02, or by the timing gap.
  void enqueue_(const std::vector<uint8_t> &data, bool ack = false) {
    // Use a safe conservative chunk size. The BLEClient negotiates MTU 517
    // with the panel; 512 bytes payload (MTU-3) fits comfortably in one write.
    size_t usable = 512;

    for (size_t off = 0; off < data.size(); off += usable) {
      size_t end = std::min(off + usable, data.size());
      WriteFrame f;
      f.data = std::vector<uint8_t>(data.begin() + off, data.begin() + end);
      f.ack  = ack;
      write_queue_.push(f);
    }
  }

  void flush_next_() {
    if (write_char_handle_ == 0) return;
    WriteFrame f = write_queue_.front();
    write_queue_.pop();

    ESP_LOGD(TAG, "BLE write %d bytes (ack=%d)", (int)f.data.size(), (int)f.ack);

    esp_err_t err = esp_ble_gattc_write_char(
        gattc_if_, conn_id_,
        write_char_handle_,
        (uint16_t)f.data.size(),
        const_cast<uint8_t *>(f.data.data()),
        ESP_GATT_WRITE_TYPE_NO_RSP,  // 0xAE01 is WRITE NO RESPONSE
        ESP_GATT_AUTH_REQ_NONE);

    last_write_ms_ = millis();

    if (err != ESP_OK) {
      ESP_LOGW(TAG, "write_char err=%d", err);
      write_pending_ = false;
    } else {
      write_pending_ = f.ack;
    }
  }

  // ── BMP builder ───────────────────────────────────────────────────────────

  static std::vector<uint8_t> make_bmp_solid_(uint16_t rgb565, uint16_t w, uint16_t h) {
    uint32_t row_bytes = ((w * 2u + 3u) / 4u) * 4u;
    uint32_t px_sz     = row_bytes * h;
    uint32_t file_sz   = 66 + px_sz;  // 14+52 bitmask header

    std::vector<uint8_t> b;
    b.reserve(file_sz);

    auto p16 = [&](uint16_t v){ b.push_back(v&0xFF); b.push_back(v>>8); };
    auto p32 = [&](uint32_t v){ b.push_back(v); b.push_back(v>>8); b.push_back(v>>16); b.push_back(v>>24); };

    b.push_back('B'); b.push_back('M');
    p32(file_sz); p32(0); p32(66);
    p32(52);                    // BITMAPV2INFOHEADER
    p32(w); p32(-(int32_t)h);
    p16(1); p16(16);            // planes, bpp
    p32(3);                     // BI_BITFIELDS
    p32(px_sz); p32(2835); p32(2835); p32(0); p32(0);
    p32(0xF800); p32(0x07E0); p32(0x001F);  // RGB565 masks

    for (uint16_t y=0; y<h; y++) {
      uint32_t rb=0;
      for (uint16_t x=0; x<w; x++) { p16(rgb565); rb+=2; }
      while (rb%4) { b.push_back(0); rb++; }
    }
    return b;
  }
};

}  // namespace act1026
}  // namespace esphome
