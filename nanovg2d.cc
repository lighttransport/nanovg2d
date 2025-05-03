#include "nanovg2d.hh"
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <limits>
#include <fstream>
#include <iostream>
#include <vector>

namespace nanovg2d {

static bool gInitialized = false;

Canvas::Canvas(int width, int height, int supersample)
    : width(width), height(height), supersample(std::max(1, supersample)),
      data(nullptr), hiresData(nullptr), dirty(true), aaLevel(AntialiasingLevel::Fast) {
    int hiresW = width * this->supersample;
    int hiresH = height * this->supersample;
    data = new uint8_t[width * height * 4];
    hiresData = new uint8_t[hiresW * hiresH * 4];
    std::memset(data, 0, width * height * 4);
    std::memset(hiresData, 0, hiresW * hiresH * 4);
}

Canvas::~Canvas() {
    if (data) delete[] data;
    if (hiresData) delete[] hiresData;
}

void Canvas::clear(const Color& color) {
    int hiresW = width * supersample;
    int hiresH = height * supersample;
    for (int y = 0; y < hiresH; ++y) {
        for (int x = 0; x < hiresW; ++x) {
            int idx = (y * hiresW + x) * 4;
            hiresData[idx+0] = color.r;
            hiresData[idx+1] = color.g;
            hiresData[idx+2] = color.b;
            hiresData[idx+3] = color.a;
        }
    }
    dirty = true;
}

void Canvas::setPixel(int x, int y, const Color& color) {
    int hiresW = width * supersample;
    int hiresH = height * supersample;
    if (x < 0 || x >= hiresW || y < 0 || y >= hiresH) return;
    int idx = (y * hiresW + x) * 4;
    hiresData[idx+0] = color.r;
    hiresData[idx+1] = color.g;
    hiresData[idx+2] = color.b;
    hiresData[idx+3] = color.a;
    dirty = true;
}

void Canvas::blendPixel(int x, int y, const Color& color) {
    int hiresW = width * supersample;
    int hiresH = height * supersample;
    if (x < 0 || x >= hiresW || y < 0 || y >= hiresH) return;
    int idx = (y * hiresW + x) * 4;
    uint8_t r = hiresData[idx+0];
    uint8_t g = hiresData[idx+1];
    uint8_t b = hiresData[idx+2];
    uint8_t a = hiresData[idx+3];
    float srcAlpha = color.a / 255.0f;
    float dstAlpha = a / 255.0f;
    float outAlpha = srcAlpha + dstAlpha * (1.0f - srcAlpha);
    if (outAlpha < 1e-6f) { hiresData[idx+3] = 0; return; }
    float invSrcAlpha = 1.0f - srcAlpha;
    hiresData[idx+0] = static_cast<uint8_t>((color.r * srcAlpha + r * dstAlpha * invSrcAlpha) / outAlpha);
    hiresData[idx+1] = static_cast<uint8_t>((color.g * srcAlpha + g * dstAlpha * invSrcAlpha) / outAlpha);
    hiresData[idx+2] = static_cast<uint8_t>((color.b * srcAlpha + b * dstAlpha * invSrcAlpha) / outAlpha);
    hiresData[idx+3] = static_cast<uint8_t>(outAlpha * 255.0f);
    dirty = true;
}

void Canvas::downsample() {
    if (supersample == 1) {
        std::memcpy(data, hiresData, width * height * 4);
        dirty = false;
        return;
    }
    int hiresW = width * supersample;
    int hiresH = height * supersample;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int r=0, g=0, b=0, a=0;
            for (int sy = 0; sy < supersample; ++sy) {
                for (int sx = 0; sx < supersample; ++sx) {
                    int hx = x * supersample + sx;
                    int hy = y * supersample + sy;
                    int hidx = (hy * hiresW + hx) * 4;
                    r += hiresData[hidx+0];
                    g += hiresData[hidx+1];
                    b += hiresData[hidx+2];
                    a += hiresData[hidx+3];
                }
            }
            int samples = supersample * supersample;
            int idx = (y * width + x) * 4;
            data[idx+0] = r / samples;
            data[idx+1] = g / samples;
            data[idx+2] = b / samples;
            data[idx+3] = a / samples;
        }
    }
    dirty = false;
}

uint8_t* Canvas::getData() {
    if (dirty) downsample();
    return data;
}
const uint8_t* Canvas::getData() const {
    if (dirty) const_cast<Canvas*>(this)->downsample();
    return data;
}

// Helper: flatten path to polyline (no Beziers for now)
static void flattenPathSimple(const nanovg2d::Path& path, std::vector<nanovg2d::Vec2>& pts) {
    nanovg2d::Vec2 cur(0,0);
    for (const auto& cmd : path.commands) {
        switch (cmd.type) {
        case nanovg2d::Path::CommandType::MoveTo:
            cur = cmd.points[0];
            pts.push_back(cur);
            break;
        case nanovg2d::Path::CommandType::LineTo:
            cur = cmd.points[0];
            pts.push_back(cur);
            break;
        default: break; // ignore Beziers for now
        }
    }
}

void Canvas::fillPath(const Path& path, const Color& color) {
    std::vector<Vec2> pts;
    flattenPathSimple(path, pts);
    if (pts.size() < 3) return;
    // Find bounds
    float minY = pts[0].y, maxY = pts[0].y;
    for (const auto& p : pts) { minY = std::min(minY, p.y); maxY = std::max(maxY, p.y); }
    int y0 = std::max(0, (int)std::ceil(minY));
    int y1 = std::min(height-1, (int)std::floor(maxY));
    for (int y = y0; y <= y1; ++y) {
        std::vector<float> xints;
        for (size_t i = 0, n = pts.size(); i < n; ++i) {
            const Vec2& a = pts[i];
            const Vec2& b = pts[(i+1)%n];
            if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y)) {
                float t = (y - a.y) / (b.y - a.y);
                xints.push_back(a.x + t * (b.x - a.x));
            }
        }
        std::sort(xints.begin(), xints.end());
        for (size_t i = 0; i+1 < xints.size(); i += 2) {
            int x0 = std::max(0, (int)std::ceil(xints[i]));
            int x1 = std::min(width-1, (int)std::floor(xints[i+1]));
            for (int x = x0; x <= x1; ++x) setPixel(x, y, color);
        }
    }
    dirty = true;
}

void Canvas::strokePath(const Path& path, const Color& color, float width) {
    std::vector<Vec2> pts;
    flattenPathSimple(path, pts);
    if (pts.size() < 2) return;
    size_t n = pts.size();
    for (size_t i = 0; i+1 < n; ++i) {
        int x0 = (int)std::round(pts[i].x), y0 = (int)std::round(pts[i].y);
        int x1 = (int)std::round(pts[i+1].x), y1 = (int)std::round(pts[i+1].y);
        int dx = std::abs(x1-x0), sx = x0<x1?1:-1;
        int dy = -std::abs(y1-y0), sy = y0<y1?1:-1;
        int err = dx+dy, e2;
        while (true) {
            setPixel(x0, y0, color);
            if (x0==x1 && y0==y1) break;
            e2 = 2*err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
    // Draw closing segment if path is closed
    if (path.closed && n > 2) {
        int x0 = (int)std::round(pts[n-1].x), y0 = (int)std::round(pts[n-1].y);
        int x1 = (int)std::round(pts[0].x), y1 = (int)std::round(pts[0].y);
        int dx = std::abs(x1-x0), sx = x0<x1?1:-1;
        int dy = -std::abs(y1-y0), sy = y0<y1?1:-1;
        int err = dx+dy, e2;
        while (true) {
            setPixel(x0, y0, color);
            if (x0==x1 && y0==y1) break;
            e2 = 2*err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
    dirty = true;
}

void Path::moveTo(float x, float y) {
    commands.emplace_back(CommandType::MoveTo, Vec2(x, y));
}

void Path::lineTo(float x, float y) {
    commands.emplace_back(CommandType::LineTo, Vec2(x, y));
}

void Path::close() {
    closed = true;
}

void Path::roundedRect(float x, float y, float w, float h, float r) {
    // Clamp radius to half the smallest dimension
    float rr = std::min(r, std::min(w, h) * 0.5f);
    // 4 corners: top-left, top-right, bottom-right, bottom-left
    moveTo(x + rr, y);
    lineTo(x + w - rr, y);
    // Top-right corner (quarter circle)
    cubicTo(x + w - rr * (1 - 0.5522847f), y, x + w, y + rr * (1 - 0.5522847f), x + w, y + rr);
    lineTo(x + w, y + h - rr);
    // Bottom-right corner
    cubicTo(x + w, y + h - rr * (1 - 0.5522847f), x + w - rr * (1 - 0.5522847f), y + h, x + w - rr, y + h);
    lineTo(x + rr, y + h);
    // Bottom-left corner
    cubicTo(x + rr * (1 - 0.5522847f), y + h, x, y + h - rr * (1 - 0.5522847f), x, y + h - rr);
    lineTo(x, y + rr);
    // Top-left corner
    cubicTo(x, y + rr * (1 - 0.5522847f), x + rr * (1 - 0.5522847f), y, x + rr, y);
    close();
}

void Canvas::drawRect(float x, float y, float w, float h, const Color& color, float strokeWidth) {
    Path p;
    p.rect(x, y, w, h);
    strokePath(p, color, strokeWidth);
}

void Canvas::fillRect(float x, float y, float w, float h, const Color& color) {
    Path p;
    p.rect(x, y, w, h);
    fillPath(p, color);
}

// Dummy minimal implementations to resolve linker errors
// Removed redundant definition of fillPath

// Removed redundant definition of strokePath

bool writeBMP(const std::string& filename, int width, int height, const uint8_t* data) {
    FILE* fp = fopen(filename.c_str(), "wb");
    if (!fp) return false;
    int rowSize = ((3 * width + 3) / 4) * 4;
    int dataSize = rowSize * height;
    int fileSize = 54 + dataSize;
    unsigned char header[54];
    // BMP Header
    header[0] = 'B'; header[1] = 'M';
    header[2] = fileSize & 0xFF;
    header[3] = (fileSize >> 8) & 0xFF;
    header[4] = (fileSize >> 16) & 0xFF;
    header[5] = (fileSize >> 24) & 0xFF;
    header[6] = header[7] = header[8] = header[9] = 0;
    header[10] = 54; header[11] = header[12] = header[13] = 0;
    // DIB Header
    header[14] = 40; header[15] = header[16] = header[17] = 0;
    header[18] = width & 0xFF;
    header[19] = (width >> 8) & 0xFF;
    header[20] = (width >> 16) & 0xFF;
    header[21] = (width >> 24) & 0xFF;
    header[22] = height & 0xFF;
    header[23] = (height >> 8) & 0xFF;
    header[24] = (height >> 16) & 0xFF;
    header[25] = (height >> 24) & 0xFF;
    header[26] = 1; header[27] = 0; // planes
    header[28] = 24; header[29] = 0; // bits per pixel
    header[30] = header[31] = header[32] = header[33] = 0; // compression
    header[34] = dataSize & 0xFF;
    header[35] = (dataSize >> 8) & 0xFF;
    header[36] = (dataSize >> 16) & 0xFF;
    header[37] = (dataSize >> 24) & 0xFF;
    header[38] = 0x13; header[39] = 0x0B; header[40] = 0; header[41] = 0; // 2835 px/meter
    header[42] = 0x13; header[43] = 0x0B; header[44] = 0; header[45] = 0; // 2835 px/meter
    header[46] = header[47] = header[48] = header[49] = 0; // colors used
    header[50] = header[51] = header[52] = header[53] = 0; // important colors
    fwrite(header, 1, 54, fp);
    std::vector<unsigned char> row(rowSize, 0);
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            int idx = (y * width + x) * 4;
            row[x * 3 + 0] = data[idx + 2]; // B
            row[x * 3 + 1] = data[idx + 1]; // G
            row[x * 3 + 2] = data[idx + 0]; // R
        }
        fwrite(row.data(), 1, rowSize, fp);
    }
    fclose(fp);
    return true;
}

bool init() {
    if (gInitialized) return true;
    gInitialized = true;
    return true;
}

void shutdown() {
    gInitialized = false;
}

Path* createPath() {
    return new Path();
}

void destroyPath(Path* path) {
    if (path) delete path;
}

} // namespace nanovg2d
