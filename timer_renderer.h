#pragma once
#include "esphome.h"
#include <vector>
#include <cmath>

// ── ACT1026 Time Timer renderer ────────────────────────────────────────────
// Builds the 3072-byte (32×32×3 RGB) pixel frame for the BK-Light ACT1026
// BLE LED matrix and returns it as a vector split into chunks.
//
// Usage in ESPHome YAML:
//   custom_component:
//     - lambda: 'return {new TimerRenderer()};'
//
// The component registers itself under id 'timer_renderer' and exposes:
//   timer_renderer->get_chunk(int chunk, int remain_sec, int total_sec, int state)
//   → std::vector<uint8_t>  (1024 bytes per chunk, 3 chunks = full frame)
//   timer_renderer->get_header(uint8_t seq) → std::vector<uint8_t>  (5 bytes)
//   timer_renderer->get_brightness_cmd(uint8_t pct) → std::vector<uint8_t>  (3 bytes)

class TimerRenderer : public Component {
 public:
  static const int W = 32;
  static const int H = 32;
  static const int PIXELS = W * H;
  static const int FRAME_BYTES = PIXELS * 3;   // 3072
  static const int CHUNK_SIZE  = 1024;         // 3 chunks

  static const int CX = 15;
  static const int CY = 15;
  static const int RADIUS = 13;

  // 3×5 bitmap font
  const uint8_t FONT3x5[10][5] = {
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

  uint8_t buf[FRAME_BYTES];

  void setup() override {}
  void loop()  override {}
  float get_setup_priority() const override { return setup_priority::LATE; }

  // ── Public API ────────────────────────────────────────────────────────
  std::vector<uint8_t> get_header(uint8_t seq) {
    return { 0xAE, 0x01, 32, 32, seq };
  }

  std::vector<uint8_t> get_brightness_cmd(uint8_t pct) {
    return { 0xAE, 0x05, pct };
  }

  // Build full frame then return the requested 1024-byte chunk (0, 1 or 2).
  std::vector<uint8_t> get_chunk(int chunk_idx,
                                  int remain_sec,
                                  int total_sec,
                                  int state) {
    build_frame(remain_sec, total_sec, state);
    int offset = chunk_idx * CHUNK_SIZE;
    int len    = std::min(CHUNK_SIZE, FRAME_BYTES - offset);
    return std::vector<uint8_t>(buf + offset, buf + offset + len);
  }

 private:
  void set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    int i = (y * W + x) * 3;
    buf[i]   = r;
    buf[i+1] = g;
    buf[i+2] = b;
  }

  void fill_all(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < FRAME_BYTES; i += 3) {
      buf[i] = r; buf[i+1] = g; buf[i+2] = b;
    }
  }

  void draw_digit(int dx, int dy, int d, uint8_t r, uint8_t g, uint8_t b) {
    for (int row = 0; row < 5; row++) {
      uint8_t bits = FONT3x5[d % 10][row];
      for (int col = 0; col < 3; col++) {
        if (bits & (0b100 >> col)) set_pixel(dx + col, dy + row, r, g, b);
      }
    }
  }

  void build_frame(int remain_sec, int total_sec, int state) {
    fill_all(8, 8, 8);  // dark background

    float fraction = (total_sec > 0) ? (float)remain_sec / total_sec : 0.0f;
    float endAngle = fraction * 2.0f * M_PI;

    bool done     = (remain_sec == 0 && total_sec > 0);
    bool flash_on = (millis() / 250) % 2 == 0;

    // Arc colours
    uint8_t ar = 200, ag = 0, ab = 0;
    if (fraction > 0.5f)       { ar = 0;   ag = 180; ab = 0; }
    else if (fraction > 0.25f) { ar = 180; ag = 140; ab = 0; }

    // Draw arc pixel-by-pixel
    for (int py = CY - RADIUS; py <= CY + RADIUS; py++) {
      for (int px = CX - RADIUS; px <= CX + RADIUS; px++) {
        float dx   = px - CX, dy = py - CY;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist > RADIUS || dist < 2) continue;

        float angle = atan2f(dy, dx) + M_PI / 2.0f;
        if (angle < 0)          angle += 2.0f * M_PI;
        if (angle > 2.0f * M_PI) angle -= 2.0f * M_PI;

        bool in_arc = (angle <= endAngle);

        if (done) {
          if (flash_on) set_pixel(px, py, 220, 0, 0);
          else          set_pixel(px, py, 30,  0, 0);
        } else if (in_arc) {
          set_pixel(px, py, ar, ag, ab);
        } else {
          set_pixel(px, py, 18, 0, 0);  // elapsed – dark red
        }
      }
    }

    // Tick marks (12 positions)
    for (int i = 0; i < 12; i++) {
      float a = (i / 12.0f) * 2.0f * M_PI - M_PI / 2.0f;
      int tx = CX + (int)roundf((RADIUS + 1) * cosf(a));
      int ty = CY + (int)roundf((RADIUS + 1) * sinf(a));
      set_pixel(tx, ty, 60, 60, 60);
    }

    // Center dot
    set_pixel(CX, CY, 200, 200, 200);

    // Paused blink
    if (state == 2 && flash_on)  // PAUSED
      set_pixel(CX, CY - RADIUS, 255, 200, 0);

    // MM:SS digits in bottom strip (y = 26..30)
    int mins = remain_sec / 60;
    int secs = remain_sec % 60;
    uint8_t dr = 180, dg = 180, db = 180;
    if (done)      { dr = 220; dg = 0;   db = 0; }
    if (state == 2){ dr = 100; dg = 100; db = 0; }  // PAUSED = yellow

    bool colon_on = (millis() / 500) % 2 == 0 || state != 1;  // blink when running
    int bx = 7, by = 26;
    draw_digit(bx,      by, mins / 10, dr, dg, db);
    draw_digit(bx + 4,  by, mins % 10, dr, dg, db);
    if (colon_on) {
      set_pixel(bx + 8, by + 1, dr, dg, db);
      set_pixel(bx + 8, by + 3, dr, dg, db);
    }
    draw_digit(bx + 10, by, secs / 10, dr, dg, db);
    draw_digit(bx + 14, by, secs % 10, dr, dg, db);
  }
};
