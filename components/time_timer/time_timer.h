#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/log.h"

#include <vector>
#include <queue>
#include <cmath>
#include <cstring>

namespace esphome {
namespace time_timer {

namespace espbt = esphome::esp32_ble_tracker;
static const char *const TAG = "time_timer";

// ─── Self-contained CRC32 (no esp_rom dependency) ────────────────────────────
static uint32_t crc32_byte(uint32_t crc, uint8_t b) {
  crc ^= b;
  for (int i = 0; i < 8; i++)
    crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1));
  return crc;
}
static uint32_t crc32(const uint8_t *data, size_t len, uint32_t init = 0xFFFFFFFF) {
  uint32_t c = init;
  for (size_t i = 0; i < len; i++) c = crc32_byte(c, data[i]);
  return c;
}
// PNG chunk CRC: init=0xFFFFFFFF, final XOR 0xFFFFFFFF
static uint32_t png_crc32(const uint8_t *data, size_t len) {
  return crc32(data, len) ^ 0xFFFFFFFF;
}
// Frame CRC: binascii.crc32 = standard CRC32 (init=0xFFFFFFFF, final XOR 0xFFFFFFFF)
static uint32_t frame_crc32(const uint8_t *data, size_t len) {
  return crc32(data, len) ^ 0xFFFFFFFF;
}

// ─── BLE (all confirmed from Bk-Light-AppBypass/display_session.py) ──────────
// ALL writes go to 0xFA02. AE00/AE01 service is NOT used.
static const uint16_t SERVICE_UUID = 0x00FA;
static const uint16_t WRITE_CHAR   = 0xFA02;
static const uint16_t NOTIFY_CHAR  = 0xFA03;

static const uint32_t WRITE_GAP_MS = 50;

// Handshake sequence (exact bytes from display_session.py)
static const uint8_t HANDSHAKE_1[] = {0x08,0x00,0x01,0x80,0x0E,0x06,0x32,0x00};
static const uint8_t HANDSHAKE_2[] = {0x04,0x00,0x05,0x80};

// ─── Display ──────────────────────────────────────────────────────────────────
static const int W = 32, H = 32;
static const float PI_F = 3.14159265f;

struct Color { uint8_t r, g, b; };
static const Color COL_BLACK = {0,0,0};
static const Color COL_WHITE = {255,255,255};
static const Color COL_RED   = {220,0,0};
static const Color COL_DKRED = {80,0,0};
static const Color COL_GRAY  = {40,40,40};

// ─── Helpers ──────────────────────────────────────────────────────────────────

static void push_u16le(std::vector<uint8_t> &v, uint16_t x) {
  v.push_back(x&0xFF); v.push_back(x>>8);
}
static void push_u32be(std::vector<uint8_t> &v, uint32_t x) {
  v.push_back(x>>24); v.push_back(x>>16); v.push_back(x>>8); v.push_back(x);
}
static void push_u32le(std::vector<uint8_t> &v, uint32_t x) {
  v.push_back(x); v.push_back(x>>8); v.push_back(x>>16); v.push_back(x>>24);
}

// PNG chunk: [len(4BE)][tag(4)][data][crc32(4BE)]
// CRC covers tag+data
static void png_chunk(std::vector<uint8_t> &out, const char *tag,
                       const uint8_t *data, size_t len) {
  push_u32be(out, (uint32_t)len);
  uint8_t t[4]={(uint8_t)tag[0],(uint8_t)tag[1],(uint8_t)tag[2],(uint8_t)tag[3]};
  // CRC over tag + data, init=0xFFFFFFFF, final XOR 0xFFFFFFFF
  uint32_t c = crc32(t, 4);
  if (len > 0) c = crc32(data, len, c);
  c ^= 0xFFFFFFFF;
  out.insert(out.end(), t, t+4);
  if (len > 0) out.insert(out.end(), data, data+len);
  push_u32be(out, c);
}

// Build PNG with real DEFLATE compression using fixed Huffman + LZ77
// Self-contained — no external library needed.
static std::vector<uint8_t> make_png(const Color *px, int w, int h) {
  // Build raw scanlines: [filter_byte=0][r g b * w] per row
  std::vector<uint8_t> raw;
  raw.reserve((size_t)h*(1+w*3));
  for (int y=0; y<h; y++) {
    raw.push_back(0);
    for (int x=0; x<w; x++) {
      raw.push_back(px[y*w+x].r);
      raw.push_back(px[y*w+x].g);
      raw.push_back(px[y*w+x].b);
    }
  }

  // ── Minimal DEFLATE with fixed Huffman + simple LZ77 ──────────────────────
  // RFC 1951: fixed Huffman codes, back-references with distance table.
  // We use a simple hash-chain for LZ77 match finding.

  struct BitBuf {
    std::vector<uint8_t> out;
    uint32_t bits{0}; int nbits{0};
    void put(uint32_t val, int n) {
      bits |= (val & ((1u<<n)-1)) << nbits; nbits += n;
      while (nbits >= 8) { out.push_back(bits & 0xFF); bits >>= 8; nbits -= 8; }
    }
    void flush() { if (nbits > 0) { out.push_back(bits & 0xFF); bits=0; nbits=0; } }
  };

  // Reverse n bits (DEFLATE codes are stored LSB-first)
  auto rev = [](uint32_t v, int n) -> uint32_t {
    uint32_t r=0; for (int i=0;i<n;i++){r=(r<<1)|(v&1);v>>=1;} return r;
  };

  // Emit literal byte using fixed Huffman
  auto emit_lit = [&](BitBuf &bb, uint8_t b) {
    if (b <= 143) bb.put(rev(0x30+b, 8), 8);
    else          bb.put(rev(0x190+(b-144), 9), 9);
  };

  // Emit end-of-block (symbol 256, 7-bit code 0)
  auto emit_eob = [&](BitBuf &bb) { bb.put(rev(0,7),7); };

  // Emit length/distance back-reference
  // Length codes 257-285, distance codes 0-29 (fixed Huffman, 7-bit for 256-279)
  auto emit_ref = [&](BitBuf &bb, int len, int dist) {
    // Length code
    struct {int sym,extra_bits,base;} lc;
    if      (len==3)  lc={257,0,3};
    else if (len==4)  lc={258,0,4};
    else if (len==5)  lc={259,0,5};
    else if (len==6)  lc={260,0,6};
    else if (len==7)  lc={261,0,7};
    else if (len==8)  lc={262,0,8};
    else if (len==9)  lc={263,0,9};
    else if (len==10) lc={264,0,10};
    else if (len<=12) lc={265,1,11};
    else if (len<=14) lc={266,1,13};
    else if (len<=16) lc={267,1,15};
    else if (len<=18) lc={268,1,17};
    else if (len<=22) lc={269,2,19};
    else if (len<=26) lc={270,2,23};
    else if (len<=30) lc={271,2,27};
    else if (len<=34) lc={272,2,31};
    else if (len<=42) lc={273,3,35};
    else if (len<=50) lc={274,3,43};
    else if (len<=58) lc={275,3,51};
    else if (len<=66) lc={276,3,59};
    else if (len<=82) lc={277,4,67};
    else if (len<=98) lc={278,4,83};
    else if (len<=114) lc={279,4,99};
    else if (len<=130) lc={280,4,115};
    else if (len<=162) lc={281,5,131};
    else if (len<=194) lc={282,5,163};
    else if (len<=226) lc={283,5,195};
    else if (len<=257) lc={284,5,227};
    else               lc={285,0,258};

    // Fixed Huffman: 256-279 = 7-bit, 280-287 = 8-bit
    if (lc.sym <= 279) bb.put(rev(lc.sym-256, 7), 7);
    else               bb.put(rev(0xC0+(lc.sym-280), 8), 8);
    if (lc.extra_bits > 0) bb.put(len - lc.base, lc.extra_bits);

    // Distance code (5 bits + extra, no Huffman for distance codes)
    int dc, de, db;
    if      (dist==1)  {dc=0;de=0;db=1;}
    else if (dist==2)  {dc=1;de=0;db=2;}
    else if (dist==3)  {dc=2;de=0;db=3;}
    else if (dist==4)  {dc=3;de=0;db=4;}
    else if (dist<=6)  {dc=4;de=1;db=5;}
    else if (dist<=8)  {dc=5;de=1;db=7;}
    else if (dist<=12) {dc=6;de=2;db=9;}
    else if (dist<=16) {dc=7;de=2;db=13;}
    else if (dist<=24) {dc=8;de=3;db=17;}
    else if (dist<=32) {dc=9;de=3;db=25;}
    else if (dist<=48) {dc=10;de=4;db=33;}
    else if (dist<=64) {dc=11;de=4;db=49;}
    else if (dist<=96) {dc=12;de=5;db=65;}
    else if (dist<=128) {dc=13;de=5;db=97;}
    else if (dist<=192) {dc=14;de=6;db=129;}
    else if (dist<=256) {dc=15;de=6;db=193;}
    else if (dist<=384) {dc=16;de=7;db=257;}
    else if (dist<=512) {dc=17;de=7;db=385;}
    else if (dist<=768) {dc=18;de=8;db=513;}
    else if (dist<=1024){dc=19;de=8;db=769;}
    else if (dist<=1536){dc=20;de=9;db=1025;}
    else if (dist<=2048){dc=21;de=9;db=1537;}
    else if (dist<=3072){dc=22;de=10;db=2049;}
    else                {dc=23;de=10;db=3073;}
    bb.put(rev(dc,5), 5);
    if (de > 0) bb.put(dist - db, de);
  };

  // Simple LZ77: hash table for 3-byte matches, max distance 4096, max length 128
  const int HSIZE = 4096;
  const int MAX_DIST = 4096;
  const int MAX_LEN = 128;
  std::vector<int> head(HSIZE, -1);
  std::vector<int> prev(raw.size(), -1);

  auto hash3 = [&](size_t i) -> int {
    return ((raw[i]*31337 + raw[i+1]*1337 + raw[i+2]) & (HSIZE-1));
  };

  BitBuf bb;
  // Block header: BFINAL=1, BTYPE=01 (fixed Huffman)
  bb.put(1,1); bb.put(1,2);

  size_t i = 0;
  const size_t n = raw.size();
  while (i < n) {
    if (i+3 <= n) {
      int h = hash3(i);
      int best_len = 2, best_dist = 0;
      int j = head[h];
      int steps = 0;
      while (j >= 0 && (int)i - j <= MAX_DIST && steps < 32) {
        // Check match length
        int ml = 0;
        while (ml < MAX_LEN && i+ml < n && raw[i+ml] == raw[j+ml]) ml++;
        if (ml > best_len) { best_len = ml; best_dist = (int)i - j; }
        j = prev[j]; steps++;
      }
      // Update hash chain
      prev[i] = head[h];
      head[h] = (int)i;

      if (best_len >= 3 && best_dist > 0) {
        emit_ref(bb, best_len, best_dist);
        // Update hash for skipped bytes
        for (int k=1; k<best_len; k++) {
          if (i+k+3 <= n) {
            int hk = hash3(i+k);
            prev[i+k] = head[hk];
            head[hk] = (int)(i+k);
          }
        }
        i += best_len;
        continue;
      }
    }
    emit_lit(bb, raw[i]);
    i++;
  }
  emit_eob(bb);
  bb.flush();

  // Compute adler32 for zlib wrapper
  uint32_t s1=1, s2=0;
  for (uint8_t b : raw) { s1=(s1+b)%65521; s2=(s2+s1)%65521; }
  uint32_t adler = (s2<<16)|s1;

  // Wrap deflate stream in zlib (78 9C = deflate, default compression)
  std::vector<uint8_t> zlib_stream;
  zlib_stream.push_back(0x78); zlib_stream.push_back(0x9C);
  zlib_stream.insert(zlib_stream.end(), bb.out.begin(), bb.out.end());
  push_u32be(zlib_stream, adler);

  // Assemble PNG
  std::vector<uint8_t> png;
  const uint8_t sig[]={0x89,'P','N','G','\r','\n',0x1a,'\n'};
  png.insert(png.end(), sig, sig+8);
  uint8_t ihdr[13]={(uint8_t)(w>>24),(uint8_t)(w>>16),(uint8_t)(w>>8),(uint8_t)w,
                    (uint8_t)(h>>24),(uint8_t)(h>>16),(uint8_t)(h>>8),(uint8_t)h,
                    8,2,0,0,0};
  png_chunk(png,"IHDR",ihdr,13);
  png_chunk(png,"IDAT",zlib_stream.data(),zlib_stream.size());
  png_chunk(png,"IEND",nullptr,0);
  return png;
}

// build_frame: wraps PNG in the BK-Light frame header
// [total_len(2LE), 0x02, 0x00, 0x00, data_len(2LE), 0x00, 0x00, crc32(4LE), 0x00, 0x65, ...png...]
static std::vector<uint8_t> build_frame(const std::vector<uint8_t> &png) {
  uint16_t dlen = (uint16_t)png.size();
  uint16_t tlen = dlen + 15;
  // Python binascii.crc32 = standard CRC32 with init=0, matches our frame_crc32
  uint32_t c = frame_crc32(png.data(), png.size());
  std::vector<uint8_t> f;
  f.reserve(tlen);
  push_u16le(f, tlen);
  f.push_back(0x02); f.push_back(0x00); f.push_back(0x00);
  push_u16le(f, dlen);
  f.push_back(0x00); f.push_back(0x00);
  push_u32le(f, c);
  f.push_back(0x00); f.push_back(0x65);
  f.insert(f.end(), png.begin(), png.end());
  return f;
}

// ─── Framebuffer ──────────────────────────────────────────────────────────────

struct Framebuffer {
  Color px[H][W];

  void clear(Color c=COL_BLACK) {
    for (int y=0;y<H;y++) for (int x=0;x<W;x++) px[y][x]=c;
  }
  void set(int x, int y, Color c) {
    if (x>=0&&x<W&&y>=0&&y<H) px[y][x]=c;
  }

  void draw_arc(float cx, float cy, float ir, float or_,
                float s_deg, float e_deg, Color col) {
    auto in_arc=[&](float dx, float dy){
      float a=atan2f(dx,-dy)*180.0f/PI_F;
      if(a<0)a+=360.0f;
      if(s_deg<=e_deg) return a>=s_deg&&a<=e_deg;
      return a>=s_deg||a<=e_deg;
    };
    for (int y=(int)(cy-or_-1);y<=(int)(cy+or_+1);y++)
      for (int x=(int)(cx-or_-1);x<=(int)(cx+or_+1);x++) {
        float dx=x-cx,dy=y-cy,r=sqrtf(dx*dx+dy*dy);
        if(r>=ir&&r<=or_&&in_arc(dx,dy)) set(x,y,col);
      }
  }

  void draw_ring(float cx,float cy,float ir,float or_,Color col) {
    draw_arc(cx,cy,ir,or_,0.0f,359.9f,col);
  }

  static const uint8_t FONT[11][5];

  void draw_char(int x, int y, char c, Color col) {
    int idx=(c>='0'&&c<='9')?c-'0':(c==':'?10:-1);
    if(idx<0)return;
    for(int r=0;r<5;r++){
      uint8_t b=FONT[idx][r];
      for(int cb=0;cb<3;cb++) if(b&(0x4>>cb)) set(x+cb,y+r,col);
    }
  }

  void draw_string(int x, int y, const char *s, Color col) {
    while(*s){draw_char(x,y,*s,col);x+=(*s==':')?0:4;s++;}
  }

  std::vector<uint8_t> to_frame() const {
    auto png=make_png(&px[0][0],W,H);
    return build_frame(png);
  }
};

const uint8_t Framebuffer::FONT[11][5]={
  {0b111,0b101,0b101,0b101,0b111}, // 0
  {0b010,0b110,0b010,0b010,0b111}, // 1
  {0b111,0b001,0b111,0b100,0b111}, // 2
  {0b111,0b001,0b111,0b001,0b111}, // 3
  {0b101,0b101,0b111,0b001,0b001}, // 4
  {0b111,0b100,0b111,0b001,0b111}, // 5
  {0b111,0b100,0b111,0b101,0b111}, // 6
  {0b111,0b001,0b001,0b001,0b001}, // 7
  {0b111,0b101,0b111,0b101,0b111}, // 8
  {0b111,0b101,0b111,0b001,0b111}, // 9
  {0b000,0b010,0b000,0b010,0b000}, // :
};

// ─── Write queue ──────────────────────────────────────────────────────────────

enum class WType { CMD, FRAME };

struct WriteFrame {
  std::vector<uint8_t> data;
  WType type{WType::CMD};
};

// ─── Long Write state (for frames > 512 bytes) ────────────────────────────────
// Uses ATT Prepare Write + Execute Write (BLE Long Write procedure)

struct LongWriteState {
  std::vector<uint8_t> data;
  size_t offset{0};
  bool active{false};
  bool waiting_exec{false};
};

// ─── Component ────────────────────────────────────────────────────────────────

class TimeTimer : public Component, public ble_client::BLEClientNode {
 public:
  void set_time_source(time::RealTimeClock *t) { time_=t; }
  void set_brightness(uint8_t v)  { brightness_=v; }
  void set_clock_style(uint8_t v) { clock_style_=v; }

  // Runtime clock configuration (callable from HA)
  void set_clock_style_rt(uint8_t v) {
    clock_style_=v;
    if (connected_&&handshake_done_&&!running_) send_clock_mode_();
  }
  void set_show_24h(bool v) {
    show_24h_=v;
    if (connected_&&handshake_done_&&!running_) send_clock_mode_();
  }
  void set_show_date(bool v) {
    show_date_=v;
    if (connected_&&handshake_done_&&!running_) send_clock_mode_();
  }

  void setup() override {
    ESP_LOGCONFIG(TAG,"Time Timer setup");
    check_already_connected_();
  }

  void loop() override {
    check_already_connected_();
    if (!connected_||write_char_handle_==0) return;

    // Watchdog
    if (write_pending_&&(millis()-last_write_ms_>3000)) {
      ESP_LOGW(TAG,"Write timeout — clearing (lw=%d lw_wait=%d)", (int)lw_.active, (int)lw_.waiting_exec);
      write_pending_=false;
      lw_.active=false; lw_.waiting_exec=false;
    }

    // Log queue state every 5s when timer running
    static uint32_t last_dbg=0;
    if (running_ && millis()-last_dbg>5000) {
      last_dbg=millis();
      ESP_LOGD(TAG,"State: pending=%d lw=%d queue=%d remaining=%d",
               (int)write_pending_, (int)lw_.active, (int)write_queue_.size(), remaining_s_);
    }

    // Long write pump
    if (lw_.active&&!write_pending_) { pump_long_write_(); return; }

    // Normal queue drain
    if (!write_queue_.empty()&&!write_pending_&&!lw_.active) {
      if (millis()-last_write_ms_>=WRITE_GAP_MS) flush_next_();
    }

    // Countdown tick
    if (running_) {
      if (millis()-tick_start_ms_>=1000) {
        tick_start_ms_=millis();
        if (remaining_s_>0) {
          remaining_s_--;
          tick_count_++;
          if (tick_count_ >= 2) { tick_count_=0; redraw_needed_=true; }
        } else { running_=false; on_timer_done_(); }
      }
    }

    if (redraw_needed_&&write_queue_.empty()&&!lw_.active&&!write_pending_
        &&frame_ack_received_&&millis()>=congestion_until_ms_) {
      redraw_needed_=false;
      frame_ack_received_=false;  // require ACK before next frame
      if (running_) send_countdown_frame_();
    }
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG,"Time Timer:");
    ESP_LOGCONFIG(TAG,"  Connected: %s", connected_?"yes":"no");
    ESP_LOGCONFIG(TAG,"  Handshake: %s", handshake_done_?"done":"pending");
  }

  void gattc_event_handler(esp_gattc_cb_event_t event,
                            esp_gatt_if_t gattc_if,
                            esp_ble_gattc_cb_param_t *param) override {
    switch(event) {
      case ESP_GATTC_OPEN_EVT:
        if(param->open.status!=ESP_GATT_OK) break;
        gattc_if_=gattc_if; conn_id_=param->open.conn_id;
        break;

      case ESP_GATTC_SEARCH_CMPL_EVT:
        gattc_if_=gattc_if; conn_id_=param->search_cmpl.conn_id;
        bootstrapped_=true;
        discover_handles_();
        break;

      case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        ESP_LOGI(TAG,"Notify registered — handshake");
        start_handshake_();
        break;

      case ESP_GATTC_NOTIFY_EVT: {
        const uint8_t *v=param->notify.value;
        uint16_t len=param->notify.value_len;
        ESP_LOGD(TAG,"Notify %d: %02X %02X %02X %02X %02X",
                 len,len>0?v[0]:0,len>1?v[1]:0,len>2?v[2]:0,len>3?v[3]:0,len>4?v[4]:0);
        if(len>=4&&v[2]==0x01&&v[3]==0x80&&handshake_stage_==1) {
          // ACK1: device info → send handshake step 2
          handshake_stage_=2; write_pending_=false;
          enqueue_cmd_(std::vector<uint8_t>(HANDSHAKE_2,HANDSHAKE_2+4));
        } else if(len>=4&&v[2]==0x05&&v[3]==0x80&&handshake_stage_==2) {
          // ACK2: time response → ready
          handshake_stage_=3; handshake_done_=true; write_pending_=false;
          ESP_LOGI(TAG,"Handshake complete — panel ready");
          on_ready_();
        } else if(len==5&&v[2]==0x02&&v[3]==0x00&&v[4]==0x03) {
          // Frame ACK — panel accepted the data
          ESP_LOGD(TAG,"Frame ACK");
          frame_ack_received_=true;
          // Only clear pending if we're NOT in the middle of a long write prepare phase
          if(!lw_.active) write_pending_=false;
        } else {
          write_pending_=false;
        }
        break;
      }

      case ESP_GATTC_WRITE_CHAR_EVT:
        if(param->write.status!=ESP_GATT_OK)
          ESP_LOGW(TAG,"Write err=%d",param->write.status);
        // For CMD (no-response) writes, clear pending immediately
        // For FRAME writes we wait for the NOTIFY ACK
        if(!current_is_frame_) write_pending_=false;
        break;

      case ESP_GATTC_PREP_WRITE_EVT:
        // Fires for both prepare and execute write confirmations
        if(param->write.status==ESP_GATT_OK) {
          if(lw_.waiting_exec) {
            // Execute completed
            ESP_LOGD(TAG,"Long write complete");
            lw_.active=false; lw_.waiting_exec=false;
          }
          write_pending_=false;
        } else if(param->write.status==0x8F || param->write.status==143) {
          // ESP_GATT_CONGESTED (0x8F=143) — back off and retry
          ESP_LOGW(TAG,"BLE congested — backing off 2s");
          lw_.active=false; lw_.waiting_exec=false; write_pending_=false;
          congestion_until_ms_=millis()+2000;
          frame_ack_received_=true;  // allow retry after backoff
        } else {
          ESP_LOGW(TAG,"Prepare/execute write err=%d",param->write.status);
          lw_.active=false; lw_.waiting_exec=false; write_pending_=false;
          frame_ack_received_=true;
        }
        break;

      case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGW(TAG,"Disconnected");
        connected_=false; bootstrapped_=false;
        handshake_done_=false; handshake_stage_=0;
        write_char_handle_=0; notify_char_handle_=0;
        write_queue_={}; write_pending_=false;
        lw_.active=false;
        frame_ack_received_=true; congestion_until_ms_=0;
        break;

      default: break;
    }
  }

  // ── Public API ────────────────────────────────────────────────────────────

  void start(uint32_t duration_s) {
    if(!connected_) return;
    total_s_=duration_s; remaining_s_=duration_s;
    running_=true; tick_start_ms_=millis(); redraw_needed_=true; tick_count_=0;
    frame_ack_received_=true;
    ESP_LOGI(TAG,"Timer start %ds",duration_s);
  }

  void pause()  { running_=false; }
  void resume() { if(!running_&&remaining_s_>0){running_=true;tick_start_ms_=millis();} }

  void stop() {
    running_=false; remaining_s_=0; total_s_=0;
    if(connected_&&handshake_done_) send_clock_mode_();
  }

  void show_clock() { if(connected_&&handshake_done_) send_clock_mode_(); }

  void send_brightness(uint8_t level) {
    if(!connected_) return;
    enqueue_cmd_(raw_cmd({5,0,4,0x80,level}));
  }

  void send_power(bool on) {
    if(!connected_) return;
    enqueue_cmd_(raw_cmd({5,0,7,1,(uint8_t)(on?1:0)}));
  }

  bool is_connected()    const { return connected_; }
  bool is_running()      const { return running_; }
  uint32_t remaining_seconds() const { return remaining_s_; }

 protected:
  time::RealTimeClock *time_{nullptr};
  uint8_t brightness_{70}, clock_style_{1};
  bool    show_24h_{true}, show_date_{true};

  bool     connected_{false}, bootstrapped_{false};
  esp_gatt_if_t gattc_if_{0};
  uint16_t conn_id_{0}, write_char_handle_{0}, notify_char_handle_{0};

  bool     handshake_done_{false};
  uint8_t  handshake_stage_{0};
  bool     current_is_frame_{false};

  std::queue<WriteFrame> write_queue_;
  bool     write_pending_{false};
  uint32_t last_write_ms_{0};

  LongWriteState lw_;

  bool     running_{false};
  uint32_t total_s_{0}, remaining_s_{0}, tick_start_ms_{0};
  bool     redraw_needed_{false};
  uint8_t  tick_count_{0};           // count ticks, only redraw every 2nd
  bool     frame_ack_received_{true};
  uint32_t congestion_until_ms_{0};

  Framebuffer fb_;

  // ── Bootstrap ─────────────────────────────────────────────────────────────

  void check_already_connected_() {
    if(bootstrapped_||parent_==nullptr) return;
    if(!parent()->connected()) return;
    gattc_if_=parent()->get_gattc_if();
    conn_id_=parent()->get_conn_id();
    ESP_LOGI(TAG,"Bootstrapping existing connection");
    bootstrapped_=true;
    discover_handles_();
  }

  void discover_handles_() {
    auto s=espbt::ESPBTUUID::from_uint16(SERVICE_UUID);
    auto *w=parent()->get_characteristic(s,espbt::ESPBTUUID::from_uint16(WRITE_CHAR));
    auto *n=parent()->get_characteristic(s,espbt::ESPBTUUID::from_uint16(NOTIFY_CHAR));
    if(!w){ESP_LOGE(TAG,"0xFA02 not found");return;}
    write_char_handle_=w->handle;
    ESP_LOGI(TAG,"Write char: 0x%04X",write_char_handle_);
    connected_=true;
    if(n){
      notify_char_handle_=n->handle;
      esp_ble_gattc_register_for_notify(gattc_if_,parent()->get_remote_bda(),notify_char_handle_);
    } else {
      start_handshake_();
    }
  }

  void start_handshake_() {
    handshake_stage_=1;
    ESP_LOGI(TAG,"Handshake step 1");
    enqueue_cmd_(std::vector<uint8_t>(HANDSHAKE_1,HANDSHAKE_1+8));
  }

  void on_ready_() {
    enqueue_cmd_(raw_cmd({5,0,4,0x80,brightness_}));  // set_brightness
    // Delay clock sync until HA time is available (may take a few seconds)
    this->set_interval("clock_sync", 2000, [this]() {
      if (!connected_ || !handshake_done_) return;
      if (time_ && time_->now().is_valid()) {
        send_clock_mode_();
        this->cancel_interval("clock_sync");
      }
    });
  }

  // ── Display ───────────────────────────────────────────────────────────────

  void send_clock_mode_() {
    uint8_t yr=0,mo=1,dy=1,dow=1,hr=0,mi=0,sc=0;
    if(time_&&time_->now().is_valid()){
      auto now=time_->now();
      yr=now.year%100;mo=now.month;dy=now.day_of_month;
      dow=now.day_of_week;hr=now.hour;mi=now.minute;sc=now.second;
    }
    enqueue_cmd_(raw_cmd({8,0,1,0x80,hr,mi,sc,0}));
    enqueue_cmd_(raw_cmd({11,0,6,1,clock_style_,(uint8_t)(show_24h_?1:0),(uint8_t)(show_date_?1:0),yr,mo,dy,dow}));
  }

  void send_countdown_frame_() {
    if (!handshake_done_) { ESP_LOGD(TAG,"No handshake yet"); return; }
    ESP_LOGD(TAG,"Sending countdown frame: %ds remaining", remaining_s_);
    fb_.clear(COL_BLACK);
    float cx=15.5f,cy=15.5f,outer=14.5f,inner=10.5f;
    fb_.draw_ring(cx,cy,inner,outer,COL_GRAY);
    if(total_s_>0){
      float frac=(float)remaining_s_/(float)total_s_;
      float sweep=frac*360.0f;
      if(sweep>0.5f) fb_.draw_arc(cx,cy,inner,outer,0.0f,sweep,COL_RED);
    }
    uint32_t m=remaining_s_/60, s=remaining_s_%60;
    char buf[8];
    if(m>=100) snprintf(buf,sizeof(buf),"%d:%02d",(int)(m/60),(int)(m%60));
    else       snprintf(buf,sizeof(buf),"%02d:%02d",(int)m,(int)s);
    // Draw MM:SS with manual positioning for precise colon placement
    // MM = 2 digits × 4px = 8px, colon = 1px, SS = 2 digits × 4px = 8px
    // Total = 17px, centered on 32px display
    int tx = (W - 17) / 2, ty = (H - 5) / 2;
    // Clear text area
    for (int row=ty-1; row<=ty+6; row++)
      for (int col=tx-1; col<=tx+18; col++)
        fb_.set(col, row, COL_BLACK);
    // Draw MM
    fb_.draw_char(tx,     ty, buf[0], COL_WHITE);
    fb_.draw_char(tx + 4, ty, buf[1], COL_WHITE);
    // Draw colon at tx+8 (1 pixel left of default tx+9)
    fb_.draw_char(tx + 7, ty, ':', COL_WHITE);
    // Draw SS
    fb_.draw_char(tx + 10, ty, buf[3], COL_WHITE);
    fb_.draw_char(tx + 14, ty, buf[4], COL_WHITE);
    auto frame = fb_.to_frame();
    ESP_LOGD(TAG,"Frame size: %d bytes, queue size: %d", (int)frame.size(), (int)write_queue_.size());
    enqueue_frame_(frame);
  }

  void on_timer_done_() {
    ESP_LOGI(TAG,"Timer done");
    fb_.clear({220,0,0});
    enqueue_frame_(fb_.to_frame());
    this->set_timeout("done",3000,[this](){ if(connected_&&handshake_done_) send_clock_mode_(); });
  }

  // ── Helpers ───────────────────────────────────────────────────────────────

  static std::vector<uint8_t> raw_cmd(std::initializer_list<uint8_t> b) {
    return std::vector<uint8_t>(b);
  }

  void enqueue_cmd_(const std::vector<uint8_t> &d) {
    WriteFrame f; f.data=d; f.type=WType::CMD;
    write_queue_.push(f);
  }

  void enqueue_frame_(const std::vector<uint8_t> &d) {
    WriteFrame f; f.data=d; f.type=WType::FRAME;
    write_queue_.push(f);
  }

  void flush_next_() {
    if(write_char_handle_==0) return;
    WriteFrame f=write_queue_.front(); write_queue_.pop();
    current_is_frame_=(f.type==WType::FRAME);

    if(current_is_frame_&&f.data.size()>512) {
      // Use Long Write (Prepare+Execute) for frames > 512 bytes
      lw_.data=f.data; lw_.offset=0; lw_.active=true; lw_.waiting_exec=false;
      ESP_LOGD(TAG,"Long write start: %d bytes",(int)f.data.size());
      pump_long_write_();
    } else {
      // Single write
      esp_gatt_write_type_t wtype = current_is_frame_
          ? ESP_GATT_WRITE_TYPE_RSP     // frame: with response, wait for notify
          : ESP_GATT_WRITE_TYPE_NO_RSP; // cmd: fire and forget
      ESP_LOGD(TAG,"Write %d bytes (frame=%d)",(int)f.data.size(),(int)current_is_frame_);
      esp_ble_gattc_write_char(gattc_if_,conn_id_,write_char_handle_,
          (uint16_t)f.data.size(),
          const_cast<uint8_t*>(f.data.data()),
          wtype, ESP_GATT_AUTH_REQ_NONE);
      last_write_ms_=millis(); write_pending_=true;
    }
  }

  void pump_long_write_() {
    if(!lw_.active) return;
    const size_t CHUNK=512;

    if(lw_.offset<lw_.data.size()) {
      size_t end=std::min(lw_.offset+CHUNK,lw_.data.size());
      uint16_t chunk_len=(uint16_t)(end-lw_.offset);
      ESP_LOGD(TAG,"Prepare write offset=%d len=%d",(int)lw_.offset,(int)chunk_len);
      esp_ble_gattc_prepare_write(gattc_if_,conn_id_,write_char_handle_,
          (uint16_t)lw_.offset,
          chunk_len,
          const_cast<uint8_t*>(lw_.data.data()+lw_.offset),
          ESP_GATT_AUTH_REQ_NONE);
      lw_.offset=end;
      last_write_ms_=millis(); write_pending_=true;
    } else {
      // All chunks sent — execute
      ESP_LOGD(TAG,"Execute write");
      lw_.waiting_exec=true;
      esp_ble_gattc_execute_write(gattc_if_,conn_id_,true);
      last_write_ms_=millis();
      // Don't set write_pending_ here — just mark long write done immediately.
      // The panel already ACK'd the frame during prepare phase.
      // Execute is fire-and-forget on this panel/stack combo.
      lw_.active=false;
      lw_.waiting_exec=false;
      write_pending_=false;
    }
  }
};

}  // namespace time_timer
}  // namespace esphome
