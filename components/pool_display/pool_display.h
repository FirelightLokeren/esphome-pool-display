#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

#include <vector>
#include <queue>
#include <cstring>
#include <cstdio>

namespace esphome {
namespace pool_display {

static const char *const TAG = "pool_display";
namespace espbt = esphome::esp32_ble_tracker;

// ─── BLE UUIDs (same as ACT1026) ─────────────────────────────────────────────
static const uint16_t SERVICE_UUID = 0x00FA;
static const uint16_t WRITE_CHAR   = 0xFA02;
static const uint16_t NOTIFY_CHAR  = 0xFA03;

static const uint32_t WRITE_GAP_MS = 50;

// Handshake (same as ACT1026)
static const uint8_t HANDSHAKE_1[] = {0x08,0x00,0x01,0x80,0x0E,0x06,0x32,0x00};
static const uint8_t HANDSHAKE_2[] = {0x04,0x00,0x05,0x80};

// ─── Display dimensions (ACT1025 = 64×16) ────────────────────────────────────
static const int W = 64, H = 16;

// ─── Colors ───────────────────────────────────────────────────────────────────
struct Color { uint8_t r, g, b; };
static const Color COL_BLACK  = {0,   0,   0  };
static const Color COL_WHITE  = {255, 255, 255};
static const Color COL_CYAN   = {0,   200, 255};  // temperature
static const Color COL_GREEN  = {0,   220, 80 };  // pH
static const Color COL_YELLOW = {255, 200, 0  };  // ORP
static const Color COL_GRAY   = {80,  80,  80 };  // labels
static const Color COL_RED    = {220, 0,   0  };  // warning

// ─── CRC32 (self-contained) ───────────────────────────────────────────────────
static uint32_t crc32_byte(uint32_t crc, uint8_t b) {
  crc ^= b;
  for (int i=0; i<8; i++) crc = (crc>>1) ^ (0xEDB88320u & -(crc&1));
  return crc;
}
static uint32_t crc32(const uint8_t *d, size_t n, uint32_t init=0xFFFFFFFF) {
  for (size_t i=0; i<n; i++) init = crc32_byte(init, d[i]);
  return init;
}

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

// ─── PNG chunk ────────────────────────────────────────────────────────────────
static void png_chunk(std::vector<uint8_t> &out, const char *tag,
                       const uint8_t *data, size_t len) {
  push_u32be(out, (uint32_t)len);
  uint8_t t[4]={(uint8_t)tag[0],(uint8_t)tag[1],(uint8_t)tag[2],(uint8_t)tag[3]};
  uint32_t c = crc32(t, 4);
  if (len>0) c = crc32(data, len, c);
  c ^= 0xFFFFFFFF;
  out.insert(out.end(), t, t+4);
  if (len>0) out.insert(out.end(), data, data+len);
  push_u32be(out, c);
}

// ─── DEFLATE (fixed Huffman + LZ77) ──────────────────────────────────────────
struct BitBuf {
  std::vector<uint8_t> out;
  uint32_t bits{0}; int nbits{0};
  void put(uint32_t val, int n) {
    bits |= (val&((1u<<n)-1))<<nbits; nbits+=n;
    while(nbits>=8){out.push_back(bits&0xFF);bits>>=8;nbits-=8;}
  }
  void flush() { if(nbits>0){out.push_back(bits&0xFF);bits=0;nbits=0;} }
};

static uint32_t rev(uint32_t v, int n) {
  uint32_t r=0; for(int i=0;i<n;i++){r=(r<<1)|(v&1);v>>=1;} return r;
}

static void emit_lit(BitBuf &bb, uint8_t b) {
  if(b<=143) bb.put(rev(0x30+b,8),8);
  else       bb.put(rev(0x190+(b-144),9),9);
}
static void emit_eob(BitBuf &bb) { bb.put(rev(0,7),7); }

static void emit_ref(BitBuf &bb, int len, int dist) {
  struct LC {int sym,eb,base;};
  LC lc;
  if     (len<=10) lc={257+len-3,0,len};
  else if(len<=12) lc={265,1,11};
  else if(len<=14) lc={266,1,13};
  else if(len<=16) lc={267,1,15};
  else if(len<=18) lc={268,1,17};
  else if(len<=22) lc={269,2,19};
  else if(len<=26) lc={270,2,23};
  else if(len<=30) lc={271,2,27};
  else if(len<=34) lc={272,2,31};
  else if(len<=42) lc={273,3,35};
  else if(len<=50) lc={274,3,43};
  else if(len<=58) lc={275,3,51};
  else if(len<=66) lc={276,3,59};
  else if(len<=82) lc={277,4,67};
  else if(len<=98) lc={278,4,83};
  else if(len<=114)lc={279,4,99};
  else if(len<=130)lc={280,4,115};
  else if(len<=162)lc={281,5,131};
  else if(len<=194)lc={282,5,163};
  else if(len<=226)lc={283,5,195};
  else if(len<=257)lc={284,5,227};
  else             lc={285,0,258};
  if(lc.sym<=279) bb.put(rev(lc.sym-256,7),7);
  else            bb.put(rev(0xC0+(lc.sym-280),8),8);
  if(lc.eb>0) bb.put(len-lc.base,lc.eb);

  int dc,de,db;
  if     (dist==1) {dc=0;de=0;db=1;}
  else if(dist==2) {dc=1;de=0;db=2;}
  else if(dist==3) {dc=2;de=0;db=3;}
  else if(dist==4) {dc=3;de=0;db=4;}
  else if(dist<=6) {dc=4;de=1;db=5;}
  else if(dist<=8) {dc=5;de=1;db=7;}
  else if(dist<=12){dc=6;de=2;db=9;}
  else if(dist<=16){dc=7;de=2;db=13;}
  else if(dist<=24){dc=8;de=3;db=17;}
  else if(dist<=32){dc=9;de=3;db=25;}
  else if(dist<=48){dc=10;de=4;db=33;}
  else if(dist<=64){dc=11;de=4;db=49;}
  else if(dist<=96){dc=12;de=5;db=65;}
  else if(dist<=128){dc=13;de=5;db=97;}
  else if(dist<=192){dc=14;de=6;db=129;}
  else if(dist<=256){dc=15;de=6;db=193;}
  else if(dist<=384){dc=16;de=7;db=257;}
  else if(dist<=512){dc=17;de=7;db=385;}
  else if(dist<=768){dc=18;de=8;db=513;}
  else              {dc=19;de=8;db=769;}
  bb.put(rev(dc,5),5);
  if(de>0) bb.put(dist-db,de);
}

static std::vector<uint8_t> deflate_compress(const std::vector<uint8_t> &raw) {
  const int HSIZE=4096, MAX_DIST=4096, MAX_LEN=128;
  std::vector<int> head(HSIZE,-1), prev(raw.size(),-1);
  auto hash3=[&](size_t i)->int{
    return((raw[i]*31337+raw[i+1]*1337+raw[i+2])&(HSIZE-1));
  };
  BitBuf bb;
  bb.put(1,1); bb.put(1,2);
  size_t i=0, n=raw.size();
  while(i<n){
    if(i+3<=n){
      int h=hash3(i), best_len=2, best_dist=0, j=head[h], steps=0;
      while(j>=0&&(int)i-j<=MAX_DIST&&steps<32){
        int ml=0;
        while(ml<MAX_LEN&&i+ml<n&&raw[i+ml]==raw[j+ml]) ml++;
        if(ml>best_len){best_len=ml;best_dist=(int)i-j;}
        j=prev[j]; steps++;
      }
      prev[i]=head[h]; head[h]=(int)i;
      if(best_len>=3&&best_dist>0){
        emit_ref(bb,best_len,best_dist);
        for(int k=1;k<best_len;k++)
          if(i+k+3<=n){int hk=hash3(i+k);prev[i+k]=head[hk];head[hk]=(int)(i+k);}
        i+=best_len; continue;
      }
    }
    emit_lit(bb,raw[i]); i++;
  }
  emit_eob(bb); bb.flush();
  return bb.out;
}

// ─── PNG generation ───────────────────────────────────────────────────────────
static std::vector<uint8_t> make_png(const Color *px, int w, int h) {
  std::vector<uint8_t> raw;
  raw.reserve((size_t)h*(1+w*3));
  for(int y=0;y<h;y++){
    raw.push_back(0);
    for(int x=0;x<w;x++){
      raw.push_back(px[y*w+x].r);
      raw.push_back(px[y*w+x].g);
      raw.push_back(px[y*w+x].b);
    }
  }
  auto deflated = deflate_compress(raw);
  // Adler32
  uint32_t s1=1,s2=0;
  for(uint8_t b:raw){s1=(s1+b)%65521;s2=(s2+s1)%65521;}
  std::vector<uint8_t> zs;
  zs.push_back(0x78); zs.push_back(0x9C);
  zs.insert(zs.end(),deflated.begin(),deflated.end());
  push_u32be(zs,(s2<<16)|s1);

  std::vector<uint8_t> png;
  const uint8_t sig[]={0x89,'P','N','G','\r','\n',0x1a,'\n'};
  png.insert(png.end(),sig,sig+8);
  uint8_t ihdr[13]={(uint8_t)(w>>24),(uint8_t)(w>>16),(uint8_t)(w>>8),(uint8_t)w,
                    (uint8_t)(h>>24),(uint8_t)(h>>16),(uint8_t)(h>>8),(uint8_t)h,
                    8,2,0,0,0};
  png_chunk(png,"IHDR",ihdr,13);
  png_chunk(png,"IDAT",zs.data(),zs.size());
  png_chunk(png,"IEND",nullptr,0);
  return png;
}

// ─── Frame wrapper ────────────────────────────────────────────────────────────
static std::vector<uint8_t> build_frame(const std::vector<uint8_t> &png) {
  uint16_t dlen=(uint16_t)png.size(), tlen=dlen+15;
  uint32_t c = crc32(png.data(),png.size()) ^ 0xFFFFFFFF;
  std::vector<uint8_t> f;
  push_u16le(f,tlen);
  f.push_back(0x02);f.push_back(0x00);f.push_back(0x00);
  push_u16le(f,dlen);
  f.push_back(0x00);f.push_back(0x00);
  push_u32le(f,c);
  f.push_back(0x00);f.push_back(0x65);
  f.insert(f.end(),png.begin(),png.end());
  return f;
}

// ─── Framebuffer ──────────────────────────────────────────────────────────────
struct Framebuffer {
  Color px[H][W];

  void clear(Color c=COL_BLACK){
    for(int y=0;y<H;y++) for(int x=0;x<W;x++) px[y][x]=c;
  }
  void set(int x,int y,Color c){
    if(x>=0&&x<W&&y>=0&&y<H) px[y][x]=c;
  }

  // 3×5 pixel font: digits 0-9, colon, dot, minus, A-Z subset
  static const uint8_t FONT_DIGITS[10][5];
  static const uint8_t FONT_COLON[5];
  static const uint8_t FONT_DOT[5];
  static const uint8_t FONT_MINUS[5];
  static const uint8_t FONT_DEGREE[5];
  // Letters needed: T,E,M,P,H,O,R,C
  static const uint8_t FONT_LETTERS[8][5];  // T,E,M,P,H,O,R,C

  void draw_glyph(int x, int y, const uint8_t glyph[5], Color col) {
    for(int r=0;r<5;r++){
      uint8_t b=glyph[r];
      for(int c=0;c<3;c++) if(b&(0x4>>c)) set(x+c,y+r,col);
    }
  }

  // Returns width advanced
  int draw_char(int x, int y, char c, Color col) {
    if(c>='0'&&c<='9'){ draw_glyph(x,y,FONT_DIGITS[c-'0'],col); return 4; }
    if(c==':'){ draw_glyph(x,y,FONT_COLON,col); return 2; }
    if(c=='.'){ draw_glyph(x,y,FONT_DOT,col);   return 2; }
    if(c=='-'){ draw_glyph(x,y,FONT_MINUS,col); return 4; }
    if(c=='~'){ draw_glyph(x,y,FONT_DEGREE,col); return 4; }
    // Letters
    const char *letters="TEMPHORС";  // T,E,M,P,H,O,R,C
    for(int i=0;i<8;i++) if(c==letters[i]){ draw_glyph(x,y,FONT_LETTERS[i],col); return 4; }
    return 4; // unknown: skip
  }

  void draw_string(int x, int y, const char *s, Color col) {
    while(*s){ x+=draw_char(x,y,*s,col); s++; }
  }

  // Draw a value+unit string right-aligned within a column of given width
  void draw_value_right(int col_x, int col_w, int y, const char *s, Color col) {
    // Measure width
    int w=0;
    for(const char *p=s;*p;p++) w+=(*p==':'||*p=='.')?2:4;
    int x=col_x+col_w-w;
    draw_string(x,y,s,col);
  }

  std::vector<uint8_t> to_frame(bool rotate180 = false) const {
    if (!rotate180) {
      auto png = make_png(&px[0][0], W, H);
      return build_frame(png);
    }
    // Rotate 180°: reverse pixel order
    Color rotated[H][W];
    for (int y = 0; y < H; y++)
      for (int x = 0; x < W; x++)
        rotated[y][x] = px[H-1-y][W-1-x];
    auto png = make_png(&rotated[0][0], W, H);
    return build_frame(png);
  }
};

// ─── Font data ────────────────────────────────────────────────────────────────
const uint8_t Framebuffer::FONT_DIGITS[10][5]={
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
};
const uint8_t Framebuffer::FONT_COLON[5] = {0b000,0b010,0b000,0b010,0b000};
const uint8_t Framebuffer::FONT_DOT[5]   = {0b000,0b000,0b000,0b000,0b010};
const uint8_t Framebuffer::FONT_MINUS[5] = {0b000,0b000,0b111,0b000,0b000};
const uint8_t Framebuffer::FONT_DEGREE[5]= {0b110,0b110,0b000,0b000,0b000};
// T, E, M, P, H, O, R, C
const uint8_t Framebuffer::FONT_LETTERS[8][5]={
  {0b111,0b010,0b010,0b010,0b010}, // T
  {0b111,0b100,0b111,0b100,0b111}, // E
  {0b101,0b111,0b101,0b101,0b101}, // M
  {0b111,0b101,0b111,0b100,0b100}, // P
  {0b101,0b101,0b111,0b101,0b101}, // H
  {0b111,0b101,0b101,0b101,0b111}, // O
  {0b111,0b101,0b111,0b110,0b101}, // R
  {0b111,0b100,0b100,0b100,0b111}, // C
};

// ─── Write queue ──────────────────────────────────────────────────────────────
enum class WType { CMD, FRAME };
struct WriteFrame { std::vector<uint8_t> data; WType type{WType::CMD}; };

// ─── Component ────────────────────────────────────────────────────────────────
class PoolDisplay : public Component, public ble_client::BLEClientNode {
 public:
  void set_brightness(uint8_t v) { brightness_=v; }

  void toggle_rotation() {
    rotated_ = !rotated_;
    ESPPreferenceObject pref = global_preferences->make_preference<bool>(fnv1_hash("pool_rotation"), true);
    pref.save(&rotated_);
    redraw_();
  }

  bool is_rotated() const { return rotated_; }

  void send_power(bool on) {
    if(!connected_) return;
    enqueue_({5,0,7,1,(uint8_t)(on?1:0)}, WType::CMD);
  }

  void send_brightness(uint8_t v) {
    brightness_=v;
    if(!connected_) return;
    enqueue_({5,0,4,0x80,v}, WType::CMD);
  }

  void set_temperature(float v) { temp_=v; redraw_(); }
  void set_ph(float v)          { ph_=v;   redraw_(); }
  void set_orp(float v)         { orp_=v;  redraw_(); }

  void setup() override {
    ESP_LOGCONFIG(TAG,"Pool Display setup");
    ESPPreferenceObject pref = global_preferences->make_preference<bool>(fnv1_hash("pool_rotation"), true);
    pref.load(&rotated_);
    ESP_LOGI(TAG,"Rotation: %s", rotated_?"180":"0");
  }

  void loop() override {
    if(!connected_) return;

    // Watchdog
    if(write_pending_&&millis()-last_write_ms_>3000){
      ESP_LOGW(TAG,"Write timeout");
      write_pending_=false;
    }

    // Drain queue
    if(!write_queue_.empty()&&!write_pending_)
      if(millis()-last_write_ms_>=WRITE_GAP_MS) flush_next_();
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG,"Pool Display:");
    ESP_LOGCONFIG(TAG,"  Connected: %s", connected_?"yes":"no");
    ESP_LOGCONFIG(TAG,"  Handshake: %s", handshake_done_?"done":"pending");
  }

  bool is_connected() const { return connected_; }

 protected:
  uint8_t brightness_{70};
  bool    rotated_{false};
  float   temp_{NAN}, ph_{NAN}, orp_{NAN};

  bool    connected_{false};
  bool    handshake_done_{false};
  uint8_t handshake_stage_{0};
  bool    current_is_frame_{false};

  ble_client::BLECharacteristic *write_chr_{nullptr};
  ble_client::BLECharacteristic *notify_chr_{nullptr};

  std::queue<WriteFrame> write_queue_;
  bool     write_pending_{false};
  uint32_t last_write_ms_{0};
  bool     frame_ack_{true};

  Framebuffer fb_;

  void gattc_event_handler(esp_gattc_cb_event_t event,
                            esp_gatt_if_t gattc_if,
                            esp_ble_gattc_cb_param_t *param) override {
    switch(event) {

      case ESP_GATTC_SEARCH_CMPL_EVT: {
        ESP_LOGI(TAG,"Service discovery complete");
        connected_=true; handshake_done_=false; handshake_stage_=0;
        frame_ack_=true; write_queue_={}; write_pending_=false;

        auto svc = espbt::ESPBTUUID::from_uint16(SERVICE_UUID);
        write_chr_ = parent()->get_characteristic(svc, espbt::ESPBTUUID::from_uint16(WRITE_CHAR));
        notify_chr_ = parent()->get_characteristic(svc, espbt::ESPBTUUID::from_uint16(NOTIFY_CHAR));

        if(!write_chr_) { ESP_LOGE(TAG,"0xFA02 not found"); connected_=false; return; }
        ESP_LOGI(TAG,"Write char: 0x%04X", write_chr_->handle);

        if(notify_chr_) {
          esp_ble_gattc_register_for_notify(gattc_if, parent()->get_remote_bda(), notify_chr_->handle);
        } else {
          handshake_stage_=1;
          enqueue_(std::vector<uint8_t>(HANDSHAKE_1, HANDSHAKE_1+8), WType::CMD);
        }
        break;
      }

      case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        ESP_LOGI(TAG,"Notify registered — handshake");
        handshake_stage_=1;
        enqueue_(std::vector<uint8_t>(HANDSHAKE_1, HANDSHAKE_1+8), WType::CMD);
        break;

      case ESP_GATTC_NOTIFY_EVT: {
        const uint8_t *v = param->notify.value;
        uint16_t len = param->notify.value_len;
        ESP_LOGD(TAG,"Notify %d: %02X %02X %02X %02X",
                 len,len>0?v[0]:0,len>1?v[1]:0,len>2?v[2]:0,len>3?v[3]:0);
        if(len>=4&&v[2]==0x01&&v[3]==0x80&&handshake_stage_==1) {
          handshake_stage_=2; write_pending_=false;
          enqueue_(std::vector<uint8_t>(HANDSHAKE_2,HANDSHAKE_2+4), WType::CMD);
        } else if(len>=4&&v[2]==0x05&&v[3]==0x80&&handshake_stage_==2) {
          handshake_stage_=3; handshake_done_=true; write_pending_=false;
          ESP_LOGI(TAG,"Handshake complete");
          on_ready_();
        } else if(len==5&&v[2]==0x02&&v[3]==0x00&&v[4]==0x03) {
          ESP_LOGD(TAG,"Frame ACK");
          frame_ack_=true; write_pending_=false;
        } else {
          write_pending_=false;
        }
        break;
      }

      case ESP_GATTC_WRITE_CHAR_EVT:
        if(param->write.status != ESP_GATT_OK)
          ESP_LOGW(TAG,"Write err=%d", param->write.status);
        if(!current_is_frame_) write_pending_=false;
        break;

      case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGW(TAG,"Disconnected");
        connected_=false; handshake_done_=false; handshake_stage_=0;
        write_chr_=nullptr; notify_chr_=nullptr;
        write_queue_={}; write_pending_=false; frame_ack_=true;
        break;

      default: break;
    }
  }

  void on_ready_() {
    enqueue_({5,0,4,0x80,brightness_}, WType::CMD);
    redraw_();
  }

  // ── Display ─────────────────────────────────────────────────────────────────

  void redraw_(){
    if(!connected_||!handshake_done_) return;
    if(!frame_ack_) return;  // previous frame still in flight

    fb_.clear(COL_BLACK);

    // Layout: 3 columns across 64px
    // Col 1 (x=0..20):  Temperature
    // Col 2 (x=22..42): pH
    // Col 3 (x=44..63): ORP

    // ── Temperature ── (manual draw to shift dot 1px left)
    char buf[16];
    Color temp_col = COL_CYAN;
    bool temp_valid = !std::isnan(temp_);
    if(temp_valid){
      if(temp_>35||temp_<10) temp_col=COL_RED;
      snprintf(buf,sizeof(buf),"%.1f",temp_);  // e.g. "28.5"
    }
    {
      if(temp_valid){
        // buf = "DD.D" — buf[0]=tens, buf[1]=units, buf[2]='.', buf[3]=decimal
        fb_.draw_char(4,  1, buf[0], temp_col);
        fb_.draw_char(8,  1, buf[1], temp_col);
        fb_.draw_char(11, 1, '.',    temp_col);
        fb_.draw_char(14, 1, buf[3], temp_col);
        fb_.draw_char(18, 1, '~',    temp_col);
      } else {
        // Show "--" centered, no dot
        fb_.draw_char(6,  1, '-', temp_col);
        fb_.draw_char(10, 1, '-', temp_col);
      }
    }
    fb_.draw_string(4, 9, "TEMP", COL_GRAY);

    // ── pH ── (manual draw to shift dot 1px left)
    Color ph_col = COL_GREEN;
    bool ph_valid = !std::isnan(ph_);
    if(ph_valid){
      if(ph_<7.0||ph_>7.6) ph_col=COL_RED;
      snprintf(buf,sizeof(buf),"%.1f",ph_);
    }
    {
      int sx = 28;
      if(ph_valid){
        // buf = "X.X" — buf[0]=integer, buf[1]='.', buf[2]=decimal
        fb_.draw_char(sx,   1, buf[0], ph_col);
        fb_.draw_char(sx+3, 1, '.',    ph_col);
        fb_.draw_char(sx+6, 1, buf[2], ph_col);
      } else {
        // Show "--" centered, no dot
        fb_.draw_char(sx,   1, '-', ph_col);
        fb_.draw_char(sx+4, 1, '-', ph_col);
      }
    }
    fb_.draw_string(29, 9, "PH", COL_GRAY);

    // ── ORP ──
    Color orp_col = COL_YELLOW;
    if(!std::isnan(orp_)){
      // Warn if ORP out of 650-750
      if(orp_<650||orp_>800) orp_col=COL_RED;
      snprintf(buf,sizeof(buf),"%d",(int)orp_);
    } else {
      snprintf(buf,sizeof(buf),"---");
    }
    fb_.draw_value_right(39, 21, 1, buf, orp_col);
    fb_.draw_string(48, 9, "ORP", COL_GRAY);

    // No dividers — spacing between columns is sufficient

    frame_ack_=false;
    enqueue_(fb_.to_frame(rotated_), WType::FRAME);
  }

  // ── Helpers ─────────────────────────────────────────────────────────────────
  void enqueue_(const std::vector<uint8_t> &d, WType t) {
    WriteFrame f; f.data=d; f.type=t; write_queue_.push(f);
  }

  void flush_next_() {
    if(!write_chr_) return;
    WriteFrame f = write_queue_.front(); write_queue_.pop();
    current_is_frame_ = (f.type == WType::FRAME);

    esp_gatt_write_type_t wtype = current_is_frame_
        ? ESP_GATT_WRITE_TYPE_RSP
        : ESP_GATT_WRITE_TYPE_NO_RSP;

    write_chr_->write_value(
        const_cast<uint8_t*>(f.data.data()),
        (int16_t)f.data.size(),
        wtype);

    last_write_ms_ = millis();
    if(current_is_frame_) write_pending_ = true;
    // CMD writes (no-response) don't set write_pending_
  }
};

}  // namespace pool_display
}  // namespace esphome
