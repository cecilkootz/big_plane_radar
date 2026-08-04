#include "panel_display.h"
#include "app_log.h"

#include <algorithm>
#include <ctype.h>
#include <cstring>
#include <esp_display_panel.hpp>
#include <esp_heap_caps.h>
#include <math.h>
#include <pgmspace.h>

using namespace esp_panel::board;
using namespace esp_panel::drivers;

namespace PanelDisplay {

static Board *board = nullptr;
static LCD *lcd = nullptr;
static Touch *touch = nullptr;
static uint32_t presentCounter = 0;
static StaticSemaphore_t refreshFinishedSemaphoreStorage;
static SemaphoreHandle_t refreshFinishedSemaphore = nullptr;

Canvas screen;

static bool IRAM_ATTR onRefreshFinished(void *) {
    if (refreshFinishedSemaphore == nullptr) {
        return false;
    }
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(refreshFinishedSemaphore, &higherPriorityTaskWoken);
    return higherPriorityTaskWoken == pdTRUE;
}

static void drainRefreshSemaphore() {
    while (refreshFinishedSemaphore != nullptr &&
           xSemaphoreTake(refreshFinishedSemaphore, 0) == pdTRUE) {}
}

static constexpr int FONT_W = 5;
static constexpr int FONT_H = 7;
static constexpr int FONT_ADVANCE = 6;
static constexpr int LINE_ADVANCE = 9;
static constexpr int MEDIUM_FONT_W = 8;
static constexpr int MEDIUM_FONT_H = 11;
static constexpr int MEDIUM_FONT_ADVANCE = 10;
static constexpr uint8_t MEDIUM_COLUMN_WIDTHS[FONT_W] = {2, 1, 2, 1, 2};
static constexpr uint8_t MEDIUM_ROW_HEIGHTS[FONT_H] = {2, 1, 2, 1, 2, 1, 2};

static const uint8_t *glyphFor(char c) {
    if (c == GLYPH_COPYRIGHT) {
        static const uint8_t g[7] = {0x0E, 0x11, 0x17, 0x15, 0x17, 0x11, 0x0E};
        return g;
    }
    if (c == GLYPH_ARROW_DOWN) {
        static const uint8_t g[7] = {0x04, 0x04, 0x04, 0x04, 0x15, 0x0E, 0x04};
        return g;
    }
    c = static_cast<char>(toupper(static_cast<unsigned char>(c)));
    switch (c) {
    case 'A': { static const uint8_t g[7] = {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}; return g; }
    case 'B': { static const uint8_t g[7] = {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}; return g; }
    case 'C': { static const uint8_t g[7] = {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}; return g; }
    case 'D': { static const uint8_t g[7] = {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E}; return g; }
    case 'E': { static const uint8_t g[7] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}; return g; }
    case 'F': { static const uint8_t g[7] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}; return g; }
    case 'G': { static const uint8_t g[7] = {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E}; return g; }
    case 'H': { static const uint8_t g[7] = {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}; return g; }
    case 'I': { static const uint8_t g[7] = {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}; return g; }
    case 'J': { static const uint8_t g[7] = {0x01, 0x01, 0x01, 0x01, 0x11, 0x11, 0x0E}; return g; }
    case 'K': { static const uint8_t g[7] = {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}; return g; }
    case 'L': { static const uint8_t g[7] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}; return g; }
    case 'M': { static const uint8_t g[7] = {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}; return g; }
    case 'N': { static const uint8_t g[7] = {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11}; return g; }
    case 'O': { static const uint8_t g[7] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}; return g; }
    case 'P': { static const uint8_t g[7] = {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}; return g; }
    case 'Q': { static const uint8_t g[7] = {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}; return g; }
    case 'R': { static const uint8_t g[7] = {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}; return g; }
    case 'S': { static const uint8_t g[7] = {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}; return g; }
    case 'T': { static const uint8_t g[7] = {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}; return g; }
    case 'U': { static const uint8_t g[7] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}; return g; }
    case 'V': { static const uint8_t g[7] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04}; return g; }
    case 'W': { static const uint8_t g[7] = {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A}; return g; }
    case 'X': { static const uint8_t g[7] = {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}; return g; }
    case 'Y': { static const uint8_t g[7] = {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}; return g; }
    case 'Z': { static const uint8_t g[7] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F}; return g; }
    case '0': { static const uint8_t g[7] = {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}; return g; }
    case '1': { static const uint8_t g[7] = {0x04, 0x0C, 0x14, 0x04, 0x04, 0x04, 0x1F}; return g; }
    case '2': { static const uint8_t g[7] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}; return g; }
    case '3': { static const uint8_t g[7] = {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E}; return g; }
    case '4': { static const uint8_t g[7] = {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}; return g; }
    case '5': { static const uint8_t g[7] = {0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E}; return g; }
    case '6': { static const uint8_t g[7] = {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}; return g; }
    case '7': { static const uint8_t g[7] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}; return g; }
    case '8': { static const uint8_t g[7] = {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}; return g; }
    case '9': { static const uint8_t g[7] = {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}; return g; }
    case ' ': { static const uint8_t g[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; return g; }
    case '-': { static const uint8_t g[7] = {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}; return g; }
    case '_': { static const uint8_t g[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F}; return g; }
    case '.': { static const uint8_t g[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C}; return g; }
    case ',': { static const uint8_t g[7] = {0x00, 0x00, 0x00, 0x00, 0x0C, 0x04, 0x08}; return g; }
    case ':': { static const uint8_t g[7] = {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00}; return g; }
    case '/': { static const uint8_t g[7] = {0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10}; return g; }
    case '^': { static const uint8_t g[7] = {0x04, 0x0E, 0x15, 0x04, 0x04, 0x04, 0x04}; return g; }
    case '+': { static const uint8_t g[7] = {0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00}; return g; }
    case '=': { static const uint8_t g[7] = {0x00, 0x00, 0x1F, 0x00, 0x1F, 0x00, 0x00}; return g; }
    case '>': { static const uint8_t g[7] = {0x10, 0x08, 0x04, 0x02, 0x04, 0x08, 0x10}; return g; }
    case '(': { static const uint8_t g[7] = {0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02}; return g; }
    case ')': { static const uint8_t g[7] = {0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08}; return g; }
    case '\'': { static const uint8_t g[7] = {0x0C, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00}; return g; }
    case '?': default: { static const uint8_t g[7] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04}; return g; }
    }
}

bool Canvas::begin() {
    RADAR_LOGD("[display] ESP32_Display_Panel backend begin\n");
    board = new Board();
    if (board == nullptr) {
        RADAR_LOGE("[display] Board allocation failed\n");
        return false;
    }

    RADAR_LOGD("[display] board.init begin\n");
    if (!board->init()) {
        RADAR_LOGE("[display] board.init failed\n");
        return false;
    }

    lcd = board->getLCD();
    if (lcd == nullptr) {
        RADAR_LOGE("[display] LCD is null after init\n");
        return false;
    }
    lcd->configFrameBufferNumber(2);

    RADAR_LOGD("[display] board.begin begin\n");
    if (!board->begin()) {
        RADAR_LOGE("[display] board.begin failed\n");
        return false;
    }

    lcd = board->getLCD();
    touch = board->getTouch();
    if (lcd == nullptr) {
        RADAR_LOGE("[display] LCD is null after begin\n");
        return false;
    }

    size_t pixels = static_cast<size_t>(WIDTH) * HEIGHT;
    _driverFb[0] = static_cast<uint16_t *>(lcd->getFrameBufferByIndex(0));
    _driverFb[1] = static_cast<uint16_t *>(lcd->getFrameBufferByIndex(1));
    if (_driverFb[0] != nullptr && _driverFb[1] != nullptr) {
        _usingDriverFrameBuffers = true;
        _drawFbIndex = 1;
        _fb = _driverFb[_drawFbIndex];
        std::fill(_driverFb[0], _driverFb[0] + pixels, TFT_BLACK);
        std::fill(_driverFb[1], _driverFb[1] + pixels, TFT_BLACK);
        refreshFinishedSemaphore = xSemaphoreCreateBinaryStatic(&refreshFinishedSemaphoreStorage);
        if (refreshFinishedSemaphore == nullptr || !lcd->attachRefreshFinishCallback(onRefreshFinished)) {
            RADAR_LOGE("[display] refresh synchronization setup failed\n");
            return false;
        }
        drainRefreshSemaphore();
        if (!lcd->switchFrameBufferTo(_driverFb[0])) {
            RADAR_LOGE("[display] initial framebuffer switch failed\n");
            return false;
        }
        drainRefreshSemaphore();
        if (xSemaphoreTake(refreshFinishedSemaphore, pdMS_TO_TICKS(150)) != pdTRUE) {
            RADAR_LOGE("[display] initial framebuffer synchronization failed\n");
            return false;
        }
    } else {
        RADAR_LOGI("[display] driver framebuffers unavailable, using copy framebuffer\n");
        size_t bytes = pixels * sizeof(uint16_t);
        _fb = static_cast<uint16_t *>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (_fb == nullptr) {
            RADAR_LOGI("[display] PSRAM framebuffer unavailable, trying internal heap\n");
            _fb = static_cast<uint16_t *>(malloc(bytes));
        }
        if (_fb == nullptr) {
            RADAR_LOGE("[display] framebuffer allocation failed\n");
            return false;
        }
    }

    RADAR_LOGI("[display] ready lcd=%dx%d touch=%d double=%d free_heap=%u free_psram=%u\n",
               lcd->getFrameWidth(),
               lcd->getFrameHeight(),
               touch != nullptr ? 1 : 0,
               _usingDriverFrameBuffers ? 1 : 0,
               static_cast<unsigned>(ESP.getFreeHeap()),
               static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    RADAR_LOGD("[display] color_bits=%d fb=%p fb0=%p fb1=%p\n",
               lcd->getFrameColorBits(), _fb, _driverFb[0], _driverFb[1]);
    if (_usingDriverFrameBuffers) {
        return true;
    }
    fillScreen(TFT_BLACK);
    return present();
}

bool Canvas::present() {
    if (lcd == nullptr || _fb == nullptr) {
        RADAR_LOGE("[display] present skipped: lcd/fb null\n");
        return false;
    }
    uint32_t start = millis();
    if (_usingDriverFrameBuffers) {
        if (refreshFinishedSemaphore == nullptr) {
            RADAR_LOGE("[display] refresh semaphore unavailable\n");
            return false;
        }

        bool synchronized = false;
        for (uint8_t attempt = 0; attempt < 2 && !synchronized; attempt++) {
            drainRefreshSemaphore();
            if (!lcd->switchFrameBufferTo(_fb)) {
                RADAR_LOGE("[display] framebuffer switch failed attempt=%u\n", attempt + 1);
                continue;
            }
            drainRefreshSemaphore();
            synchronized = xSemaphoreTake(refreshFinishedSemaphore, pdMS_TO_TICKS(150)) == pdTRUE;
            if (!synchronized) {
                RADAR_LOGE("[display] refresh wait timeout attempt=%u\n", attempt + 1);
            }
        }
        if (!synchronized) {
            return false;
        }
        _drawFbIndex ^= 1;
        _fb = _driverFb[_drawFbIndex];
    } else {
        if (!lcd->drawBitmap(0, 0, WIDTH, HEIGHT, reinterpret_cast<const uint8_t *>(_fb), -1)) {
            RADAR_LOGE("[display] drawBitmap failed\n");
            return false;
        }
    }
    presentCounter++;
    if (presentCounter <= 3 || presentCounter % 120 == 0) {
        RADAR_LOGD("[display] present #%lu dt=%lu double=%d\n",
                   static_cast<unsigned long>(presentCounter),
                   static_cast<unsigned long>(millis() - start),
                   _usingDriverFrameBuffers ? 1 : 0);
    }
    return true;
}

bool Canvas::readTouch(uint16_t *x, uint16_t *y) {
    if (touch == nullptr) {
        return false;
    }
    TouchPoint points[1];
    int count = touch->readPoints(points, 1, 0);
    if (count <= 0) {
        return false;
    }
    if (x != nullptr) *x = static_cast<uint16_t>(std::max(0, std::min(WIDTH - 1, points[0].x)));
    if (y != nullptr) *y = static_cast<uint16_t>(std::max(0, std::min(HEIGHT - 1, points[0].y)));
    return true;
}

const uint16_t *Canvas::displayedFrameBuffer() const {
    if (_usingDriverFrameBuffers) {
        return _driverFb[_drawFbIndex ^ 1];
    }
    return _fb;
}

uint16_t Canvas::color565(uint8_t r, uint8_t g, uint8_t b) const {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void Canvas::fillScreen(uint16_t color) {
    if (_fb == nullptr) return;
    std::fill(_fb, _fb + static_cast<size_t>(WIDTH) * HEIGHT, color);
}

void Canvas::fillRect(int x, int y, int w, int h, uint16_t color) {
    if (_fb == nullptr || w <= 0 || h <= 0) return;
    int x0 = std::max(0, x);
    int y0 = std::max(0, y);
    int x1 = std::min(WIDTH, x + w);
    int y1 = std::min(HEIGHT, y + h);
    if (x0 >= x1 || y0 >= y1) return;
    for (int yy = y0; yy < y1; yy++) {
        uint16_t *row = _fb + static_cast<size_t>(yy) * WIDTH;
        std::fill(row + x0, row + x1, color);
    }
}

void Canvas::drawPixel(int x, int y, uint16_t color) {
    if (_fb == nullptr || x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    _fb[static_cast<size_t>(y) * WIDTH + x] = color;
}

void Canvas::drawLine(int x0, int y0, int x1, int y1, uint16_t color) {
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        drawPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void Canvas::drawWideLine(int x0, int y0, int x1, int y1, float width, uint16_t color) {
    if (width <= 1.1f) {
        drawLine(x0, y0, x1, y1, color);
        return;
    }
    int radius = std::max(1, static_cast<int>(lroundf(width * 0.5f)));
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        fillCircle(x0, y0, radius, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void Canvas::drawCircle(int x0, int y0, int r, uint16_t color) {
    if (r <= 0) return;
    int x = -r;
    int y = 0;
    int err = 2 - 2 * r;
    do {
        drawPixel(x0 - x, y0 + y, color);
        drawPixel(x0 - y, y0 - x, color);
        drawPixel(x0 + x, y0 - y, color);
        drawPixel(x0 + y, y0 + x, color);
        int e2 = err;
        if (e2 <= y) err += ++y * 2 + 1;
        if (e2 > x || err > y) err += ++x * 2 + 1;
    } while (x < 0);
}

void Canvas::fillCircle(int x0, int y0, int r, uint16_t color) {
    if (r <= 0) {
        drawPixel(x0, y0, color);
        return;
    }
    for (int y = -r; y <= r; y++) {
        int span = static_cast<int>(sqrtf(static_cast<float>(r * r - y * y)));
        fillRect(x0 - span, y0 + y, span * 2 + 1, 1, color);
    }
}

void Canvas::fillSmoothCircle(int x0, int y0, int r, uint16_t color) {
    fillCircle(x0, y0, r, color);
}

static int edgeValue(int ax, int ay, int bx, int by, int px, int py) {
    return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
}

void Canvas::fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t color) {
    int minX = std::max(0, std::min({x0, x1, x2}));
    int maxX = std::min(WIDTH - 1, std::max({x0, x1, x2}));
    int minY = std::max(0, std::min({y0, y1, y2}));
    int maxY = std::min(HEIGHT - 1, std::max({y0, y1, y2}));
    int area = edgeValue(x0, y0, x1, y1, x2, y2);
    if (area == 0) return;
    for (int y = minY; y <= maxY; y++) {
        for (int x = minX; x <= maxX; x++) {
            int w0 = edgeValue(x1, y1, x2, y2, x, y);
            int w1 = edgeValue(x2, y2, x0, y0, x, y);
            int w2 = edgeValue(x0, y0, x1, y1, x, y);
            if ((area > 0 && w0 >= 0 && w1 >= 0 && w2 >= 0) ||
                (area < 0 && w0 <= 0 && w1 <= 0 && w2 <= 0)) {
                drawPixel(x, y, color);
            }
        }
    }
}

void Canvas::blitRGB565(int x, int y, int w, int h, const uint16_t *pixels, int stride) {
    if (_fb == nullptr || pixels == nullptr || w <= 0 || h <= 0 || stride < w) return;

    int srcX = 0;
    int srcY = 0;
    if (x < 0) {
        srcX = -x;
        w += x;
        x = 0;
    }
    if (y < 0) {
        srcY = -y;
        h += y;
        y = 0;
    }
    w = std::min(w, WIDTH - x);
    h = std::min(h, HEIGHT - y);
    if (w <= 0 || h <= 0) return;

    for (int row = 0; row < h; row++) {
        const uint16_t *src = pixels + static_cast<size_t>(srcY + row) * stride + srcX;
        uint16_t *dst = _fb + static_cast<size_t>(y + row) * WIDTH + x;
        memcpy(dst, src, static_cast<size_t>(w) * sizeof(uint16_t));
    }
}

void Canvas::blendAlphaMask4(
    int x,
    int y,
    int w,
    int h,
    const uint8_t *packedAlpha,
    uint16_t color
) {
    if (_fb == nullptr || packedAlpha == nullptr || w <= 0 || h <= 0) return;

    int sourceX = std::max(0, -x);
    int sourceY = std::max(0, -y);
    int destinationX = std::max(0, x);
    int destinationY = std::max(0, y);
    int drawWidth = std::min(w - sourceX, WIDTH - destinationX);
    int drawHeight = std::min(h - sourceY, HEIGHT - destinationY);
    if (drawWidth <= 0 || drawHeight <= 0) return;

    uint16_t sourceRed = (color >> 11) & 0x1f;
    uint16_t sourceGreen = (color >> 5) & 0x3f;
    uint16_t sourceBlue = color & 0x1f;
    for (int row = 0; row < drawHeight; row++) {
        uint16_t *destination = _fb +
            static_cast<size_t>(destinationY + row) * WIDTH + destinationX;
        size_t sourcePixel = static_cast<size_t>(sourceY + row) * w + sourceX;
        for (int column = 0; column < drawWidth; column++, sourcePixel++) {
            uint8_t packed = pgm_read_byte(packedAlpha + (sourcePixel >> 1));
            uint8_t alpha = (sourcePixel & 1) ? (packed & 0x0f) : (packed >> 4);
            if (alpha == 0) continue;
            if (alpha == 15) {
                destination[column] = color;
                continue;
            }

            uint16_t background = destination[column];
            uint16_t inverseAlpha = 15 - alpha;
            uint16_t red = (
                sourceRed * alpha + ((background >> 11) & 0x1f) * inverseAlpha + 7
            ) / 15;
            uint16_t green = (
                sourceGreen * alpha + ((background >> 5) & 0x3f) * inverseAlpha + 7
            ) / 15;
            uint16_t blue = (
                sourceBlue * alpha + (background & 0x1f) * inverseAlpha + 7
            ) / 15;
            destination[column] = static_cast<uint16_t>((red << 11) | (green << 5) | blue);
        }
    }
}

void Canvas::setTextSize(uint8_t size) {
    _textSize = std::max<uint8_t>(1, size);
}

void Canvas::setTextColor(uint16_t fg) {
    _textFg = fg;
}

void Canvas::setTextColor(uint16_t fg, uint16_t bg) {
    _textFg = fg;
    _textBg = bg;
}

void Canvas::setTextDatum(textdatum_t datum) {
    _datum = datum;
}

int Canvas::textWidth(const char *text) const {
    if (text == nullptr || text[0] == '\0') return 0;
    return static_cast<int>(strlen(text)) * FONT_ADVANCE * _textSize - _textSize;
}

int Canvas::textWidth(const String &text) const {
    return textWidth(text.c_str());
}

int Canvas::mediumTextWidth(const char *text) const {
    if (text == nullptr || text[0] == '\0') return 0;
    return static_cast<int>(strlen(text)) * MEDIUM_FONT_ADVANCE -
        (MEDIUM_FONT_ADVANCE - MEDIUM_FONT_W);
}

int Canvas::mediumTextWidth(const String &text) const {
    return mediumTextWidth(text.c_str());
}

void Canvas::drawChar(char ch, int x, int y) {
    const uint8_t *rows = glyphFor(ch);
    int s = _textSize;
    fillRect(x, y, FONT_W * s, FONT_H * s, _textBg);
    for (int yy = 0; yy < FONT_H; yy++) {
        for (int xx = 0; xx < FONT_W; xx++) {
            if (rows[yy] & (1 << (FONT_W - 1 - xx))) {
                fillRect(x + xx * s, y + yy * s, s, s, _textFg);
            }
        }
    }
}

void Canvas::drawString(const char *text, int x, int y) {
    if (text == nullptr) return;
    int w = textWidth(text);
    int h = FONT_H * _textSize;
    int startX = x;
    int startY = y;
    if (_datum == textdatum_t::top_right) {
        startX = x - w;
    } else if (_datum == textdatum_t::middle_center) {
        startX = x - w / 2;
        startY = y - h / 2;
    }
    for (size_t i = 0; text[i] != '\0'; i++) {
        drawChar(text[i], startX + static_cast<int>(i) * FONT_ADVANCE * _textSize, startY);
    }
}

void Canvas::drawString(const String &text, int x, int y) {
    int newline = text.indexOf('\n');
    if (newline >= 0) {
        int lineY = y;
        int start = 0;
        while (start <= static_cast<int>(text.length())) {
            int next = text.indexOf('\n', start);
            String line = (next >= 0) ? text.substring(start, next) : text.substring(start);
            drawString(line, x, lineY);
            if (next < 0) break;
            start = next + 1;
            lineY += LINE_ADVANCE * _textSize;
        }
        return;
    }
    drawString(text.c_str(), x, y);
}

void Canvas::drawMediumChar(char ch, int x, int y) {
    const uint8_t *rows = glyphFor(ch);
    fillRect(x, y, MEDIUM_FONT_W, MEDIUM_FONT_H, _textBg);

    int destinationY = y;
    for (int sourceY = 0; sourceY < FONT_H; sourceY++) {
        int destinationX = x;
        for (int sourceX = 0; sourceX < FONT_W; sourceX++) {
            if (rows[sourceY] & (1 << (FONT_W - 1 - sourceX))) {
                fillRect(
                    destinationX,
                    destinationY,
                    MEDIUM_COLUMN_WIDTHS[sourceX],
                    MEDIUM_ROW_HEIGHTS[sourceY],
                    _textFg
                );
            }
            destinationX += MEDIUM_COLUMN_WIDTHS[sourceX];
        }
        destinationY += MEDIUM_ROW_HEIGHTS[sourceY];
    }
}

void Canvas::drawMediumString(const char *text, int x, int y) {
    if (text == nullptr) return;
    int width = mediumTextWidth(text);
    int startX = x;
    int startY = y;
    if (_datum == textdatum_t::top_right) {
        startX = x - width;
    } else if (_datum == textdatum_t::middle_center) {
        startX = x - width / 2;
        startY = y - MEDIUM_FONT_H / 2;
    }
    for (size_t i = 0; text[i] != '\0'; i++) {
        drawMediumChar(
            text[i],
            startX + static_cast<int>(i) * MEDIUM_FONT_ADVANCE,
            startY
        );
    }
}

void Canvas::drawMediumString(const String &text, int x, int y) {
    drawMediumString(text.c_str(), x, y);
}

} // namespace PanelDisplay
