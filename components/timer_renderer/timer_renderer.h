#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include <vector>
#include <cmath>
#include <algorithm>

namespace esphome {
namespace timer_renderer {

static const char *TAG = "timer_renderer";

class TimerRendererComponent : public Component {
 public:
  static const int W    = 32;
  static const int H    = 32;
  static const int FRAME_BYTES = W * H * 3;  // 3072
  static const int CHUNK_SIZE  = 512;         // 6 chunks
  static const int CX = 15, CY = 15, RADIUS = 13;

  void setup() override { ESP_LOGI(TAG, "TimerRenderer ready"); }
  void loop()  override {}
  float get_setup_priority() const override { return setup_priority::LATE; }

  // ── Public API called from YAML lambdas ──────────────────────────────

  std::vector<uint8_t> get_header(uint8_t seq) {
    return {0xAE, 0x01, 32, 32, seq};
  }

  std::vector<uint8_t> get_brightness_cmd(uint8_t pct) {
    return {0xAE, 0x05, pct};
  }

  // Build full frame then return one 1024-byte chunk (index 0, 1 or 2)
  std::vector<uint8_t> get_chunk(int idx, int remain_sec, int total_sec, int state) {
    build_frame_(remain_sec, total_sec, state);
    int offset = idx * CHUNK_SIZE;
    int len    = std::min(CHUNK_SIZE, FRAME_BYTES - offset);
    return std::vector<uint8_t>(buf_ + offset, buf_ + offset + len);
  }

 private:
  uint8_t buf_[FRAME_BYTES];

  // 3×5 font
  const uint8_t FONT_[10][5] = {
    {0b111,0b101,0b101,0b101,0b111},
    {0b010,0b110,0b010,0b010,0b111},
    {0b111,0b001,0b111,0b100,0b111},
    {0b111,0b001,0b111,0b001,0b111},
    {0b101,0b101,0b111,0b001,0b001},
    {0b111,0b100,0b111,0b001,0b111},
    {0b111,0b100,0b111,0b101,0b111},
    {0b111,0b001,0b001,0b001,0b001},
    {0b111,0b101,0b111,0b101,0b111},
    {0b111,0b101,0b111,0b001,0b111},
  };

  void px_(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    int i = (y * W + x) * 3;
    buf_[i] = r; buf_[i+1] = g; buf_[i+2] = b;
  }

  void digit_(int dx, int dy, int d, uint8_t r, uint8_t g, uint8_t b) {
    for (int row = 0; row < 5; row++) {
      uint8_t bits = FONT_[d % 10][row];
      for (int col = 0; col < 3; col++)
        if (bits & (0b100 >> col)) px_(dx + col, dy + row, r, g, b);
    }
  }

  void build_frame_(int remain, int total, int state) {
    // Background
    for (int i = 0; i < FRAME_BYTES; i += 3) { buf_[i]=8; buf_[i+1]=8; buf_[i+2]=8; }

    float frac     = (total > 0) ? (float)remain / total : 0.0f;
    float end_ang  = frac * 2.0f * M_PI;
    bool  done     = (remain == 0 && total > 0);
    bool  flash    = (millis() / 250) % 2 == 0;

    uint8_t ar = 200, ag = 0, ab = 0;
    if (frac > 0.5f)       { ar = 0;   ag = 180; ab = 0; }
    else if (frac > 0.25f) { ar = 180; ag = 140; ab = 0; }

    // Arc
    for (int py = CY - RADIUS; py <= CY + RADIUS; py++) {
      for (int px = CX - RADIUS; px <= CX + RADIUS; px++) {
        float dx = px - CX, dy = py - CY;
        float d  = sqrtf(dx*dx + dy*dy);
        if (d > RADIUS || d < 2) continue;
        float a = atan2f(dy, dx) + M_PI / 2.0f;
        if (a < 0) a += 2.0f * M_PI;
        if (a > 2.0f * M_PI) a -= 2.0f * M_PI;
        if (done)         { px_(px,py, flash?220:30, 0, 0); }
        else if (a<=end_ang) { px_(px,py, ar, ag, ab); }
        else              { px_(px,py, 18, 0, 0); }
      }
    }

    // Tick marks
    for (int i = 0; i < 12; i++) {
      float a = (i/12.0f)*2.0f*M_PI - M_PI/2.0f;
      px_(CX+(int)roundf((RADIUS+1)*cosf(a)), CY+(int)roundf((RADIUS+1)*sinf(a)), 60,60,60);
    }
    px_(CX, CY, 200, 200, 200);  // center dot

    // Paused blink
    if (state == 2 && flash) px_(CX, CY-RADIUS, 255, 200, 0);

    // MM:SS
    uint8_t dr=180,dg=180,db=180;
    if (done)     { dr=220; dg=0;   db=0; }
    if (state==2) { dr=100; dg=100; db=0; }
    bool colon = (millis()/500)%2==0 || state!=1;
    int bx=7, by=26;
    digit_(bx,    by, remain/60/10, dr,dg,db);
    digit_(bx+4,  by, remain/60%10, dr,dg,db);
    if (colon) { px_(bx+8,by+1,dr,dg,db); px_(bx+8,by+3,dr,dg,db); }
    digit_(bx+10, by, remain%60/10, dr,dg,db);
    digit_(bx+14, by, remain%60%10, dr,dg,db);
  }
};

}  // namespace timer_renderer
}  // namespace esphome
