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

bool init() {
    if (gInitialized) return true;
    gInitialized = true;
    return true;
}   

void shutdown() {
    gInitialized = false;
}

// Font implementation (no FreeType)
Font::Font() {}
Font::~Font() {}

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

// --- TrueType big-endian helpers ---
inline uint16_t ttUSHORT(const unsigned char* p) {
    return (p[0] << 8) | p[1];
}
inline int16_t ttSHORT(const unsigned char* p) {
    return (int16_t)((p[0] << 8) | p[1]);
}
inline uint32_t ttULONG(const unsigned char* p) {
    return (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
}
inline int32_t ttLONG(const unsigned char* p) {
    return (int32_t)((p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]);
}

// --- TrueType Table Directory Parsing ---
static int find_table(const unsigned char* data, int fontOffset, const char* tag) {
    int numTables = nanovg2d::ttUSHORT(data + fontOffset + 4);
    int tableDir = fontOffset + 12;
    for (int i = 0; i < numTables; ++i) {
        const unsigned char* entry = data + tableDir + 16 * i;
        if (std::memcmp(entry, tag, 4) == 0) {
            return (int)nanovg2d::ttULONG(entry + 8); // offset
        }
    }
    return 0;
}


// Minimal TrueType font parsing (stub, inspired by stb_truetype.h)
static bool parse_truetype_font(TrueTypeFontInfo& info, const unsigned char* data, size_t size) {
    if (size < 12) return false;
    int fontOffset = 0;
    if (std::memcmp(data, "\0\1\0\0", 4) == 0) {
        fontOffset = 0;
    } else if (std::memcmp(data, "OTTO", 4) == 0) {
        // CFF not supported
        return false;
    } else {
        return false;
    }
    info.data = data;
    info.fontOffset = fontOffset;
    info.cmap = nanovg2d::find_table(data, fontOffset, "cmap");
    info.glyf = nanovg2d::find_table(data, fontOffset, "glyf");
    info.loca = nanovg2d::find_table(data, fontOffset, "loca");
    info.head = nanovg2d::find_table(data, fontOffset, "head");
    info.hhea = nanovg2d::find_table(data, fontOffset, "hhea");
    info.hmtx = nanovg2d::find_table(data, fontOffset, "hmtx");
    info.maxp = nanovg2d::find_table(data, fontOffset, "maxp");
    info.kern = nanovg2d::find_table(data, fontOffset, "kern");
    if (!info.cmap || !info.glyf || !info.loca || !info.head || !info.hhea || !info.hmtx || !info.maxp) return false;
    // Parse maxp
    info.numGlyphs = nanovg2d::ttUSHORT(data + info.maxp + 4);
    // Parse head
    info.indexLocFormat = nanovg2d::ttUSHORT(data + info.head + 50);
    info.unitsPerEm = nanovg2d::ttUSHORT(data + info.head + 18);
    // Parse hhea
    info.ascent = nanovg2d::ttSHORT(data + info.hhea + 4);
    info.descent = nanovg2d::ttSHORT(data + info.hhea + 6);
    info.lineGap = nanovg2d::ttSHORT(data + info.hhea + 8);
    info.numHMetrics = nanovg2d::ttUSHORT(data + info.hhea + 34);
    // Loca table size
    if (info.indexLocFormat == 0) {
        info.locaTableSize = (info.numGlyphs + 1) * 2;
    } else {
        info.locaTableSize = (info.numGlyphs + 1) * 4;
    }
    return true;
}

// --- Font methods for TrueType ---
int Font::getGlyphIndex(uint32_t codepoint) const {
    // Only supports format 4 cmap (platform 3, encoding 1)
    const unsigned char* cmap = info.data + info.cmap;
    int numTables = nanovg2d::ttUSHORT(cmap + 2);
    const unsigned char* table = cmap + 4;
    for (int i = 0; i < numTables; ++i) {
        int platform = nanovg2d::ttUSHORT(table + 0);
        int encoding = nanovg2d::ttUSHORT(table + 2);
        int offset = nanovg2d::ttULONG(table + 4);
        if ((platform == 3 && encoding == 1) || (platform == 0)) {
            const unsigned char* subtable = cmap + offset;
            int format = nanovg2d::ttUSHORT(subtable);
            if (format == 4) {
                int segCount = nanovg2d::ttUSHORT(subtable + 6) / 2;
                const unsigned char* endCode = subtable + 14;
                const unsigned char* startCode = endCode + 2 + segCount * 2;
                const unsigned char* idDelta = startCode + segCount * 2;
                const unsigned char* idRangeOffset = idDelta + segCount * 2;
                for (int seg = 0; seg < segCount; ++seg) {
                    uint16_t end = nanovg2d::ttUSHORT(endCode + 2 * seg);
                    uint16_t start = nanovg2d::ttUSHORT(startCode + 2 * seg);
                    if (codepoint >= start && codepoint <= end) {
                        uint16_t delta = nanovg2d::ttUSHORT(idDelta + 2 * seg);
                        uint16_t rangeOffset = nanovg2d::ttUSHORT(idRangeOffset + 2 * seg);
                        if (rangeOffset == 0) {
                            return (codepoint + delta) & 0xFFFF;
                        } else {
                            int idx = (rangeOffset / 2 + (codepoint - start) - (segCount - seg)) * 2;
                            const unsigned char* glyphIdPtr = idRangeOffset + 2 * seg + idx;
                            uint16_t glyphId = nanovg2d::ttUSHORT(glyphIdPtr);
                            if (glyphId != 0) {
                                return (glyphId + delta) & 0xFFFF;
                            } else {
                                return 0;
                            }
                        }
                    }
                }
            }
        }
        table += 8;
    }
    return 0;
}

bool Font::getGlyphMetrics(int glyphIndex, int* advance, int* lsb) const {
    if (glyphIndex < 0 || glyphIndex >= info.numGlyphs) return false;
    const unsigned char* hmtx = info.data + info.hmtx;
    if (glyphIndex < info.numHMetrics) {
        if (advance) *advance = nanovg2d::ttUSHORT(hmtx + 4 * glyphIndex);
        if (lsb) *lsb = nanovg2d::ttSHORT(hmtx + 4 * glyphIndex + 2);
    } else {
        if (advance) *advance = nanovg2d::ttUSHORT(hmtx + 4 * (info.numHMetrics - 1));
        if (lsb) *lsb = nanovg2d::ttSHORT(hmtx + 4 * info.numHMetrics + 2 * (glyphIndex - info.numHMetrics));
    }
    return true;
}

bool Font::getGlyphBox(int glyphIndex, int* x0, int* y0, int* x1, int* y1) const {
    if (glyphIndex < 0 || glyphIndex >= info.numGlyphs) return false;
    int offset = 0;
    if (info.indexLocFormat == 0) {
        offset = nanovg2d::ttUSHORT(info.data + info.loca + 2 * glyphIndex) * 2;
    } else {
        offset = nanovg2d::ttULONG(info.data + info.loca + 4 * glyphIndex);
    }
    const unsigned char* glyf = info.data + info.glyf + offset;
    int numberOfContours = nanovg2d::ttSHORT(glyf);
    if (numberOfContours == 0) {
        if (x0) *x0 = *y0 = *x1 = *y1 = 0;
        return true;
    }
    if (x0) *x0 = nanovg2d::ttSHORT(glyf + 2);
    if (y0) *y0 = nanovg2d::ttSHORT(glyf + 4);
    if (x1) *x1 = nanovg2d::ttSHORT(glyf + 6);
    if (y1) *y1 = nanovg2d::ttSHORT(glyf + 8);
    return true;
}

bool Font::getGlyphOutline(int glyphIndex, std::vector<Vec2>& contour, std::vector<uint8_t>& onCurve) const {
    contour.clear();
    onCurve.clear();
    if (glyphIndex < 0 || glyphIndex >= info.numGlyphs) return false;
    int offset = 0;
    if (info.indexLocFormat == 0) {
        offset = nanovg2d::ttUSHORT(info.data + info.loca + 2 * glyphIndex) * 2;
    } else {
        offset = nanovg2d::ttULONG(info.data + info.loca + 4 * glyphIndex);
    }
    const unsigned char* glyf = info.data + info.glyf + offset;
    int numberOfContours = nanovg2d::ttSHORT(glyf);
    if (numberOfContours <= 0) return false;
    // Parse endPtsOfContours
    std::vector<int> endPts(numberOfContours);
    for (int i = 0; i < numberOfContours; ++i) {
        endPts[i] = nanovg2d::ttUSHORT(glyf + 10 + 2 * i);
    }
    int numPoints = endPts.back() + 1;
    int instructionLength = nanovg2d::ttUSHORT(glyf + 10 + 2 * numberOfContours);
    int instructionsOffset = 10 + 2 * numberOfContours + 2;
    int flagsOffset = instructionsOffset + instructionLength;
    // Parse flags
    std::vector<uint8_t> flags;
    int flagPtr = flagsOffset;
    while ((int)flags.size() < numPoints) {
        uint8_t flag = glyf[flagPtr++];
        flags.push_back(flag);
        if (flag & 8) {
            uint8_t repeat = glyf[flagPtr++];
            for (int i = 0; i < repeat; ++i) flags.push_back(flag);
        }
    }
    // Parse x coordinates
    std::vector<int> xCoords(numPoints);
    int x = 0;
    int xPtr = flagPtr;
    for (int i = 0; i < numPoints; ++i) {
        uint8_t flag = flags[i];
        if (flag & 2) {
            int dx = glyf[xPtr++];
            if (!(flag & 16)) dx = -dx;
            x += dx;
        } else if (!(flag & 16)) {
            x += (int16_t)(glyf[xPtr] << 8 | glyf[xPtr + 1]);
            xPtr += 2;
        }
        xCoords[i] = x;
    }
    // Parse y coordinates
    std::vector<int> yCoords(numPoints);
    int y = 0;
    int yPtr = xPtr;
    for (int i = 0; i < numPoints; ++i) {
        uint8_t flag = flags[i];
        if (flag & 4) {
            int dy = glyf[yPtr++];
            if (!(flag & 32)) dy = -dy;
            y += dy;
        } else if (!(flag & 32)) {
            y += (int16_t)(glyf[yPtr] << 8 | glyf[yPtr + 1]);
            yPtr += 2;
        }
        yCoords[i] = y;
    }
    // Output contour points and onCurve flags
    for (int i = 0; i < numPoints; ++i) {
        contour.push_back(Vec2((float)xCoords[i], (float)yCoords[i]));
        onCurve.push_back(flags[i] & 1);
    }
    return true;
}

// Rasterize a glyph outline to a bitmap (simple scanline fill)
static void rasterizeGlyph(const std::vector<Vec2>& contour, const std::vector<uint8_t>& onCurve, int width, int height, std::vector<uint8_t>& bitmap, float scale, float shiftX, float shiftY) {
    bitmap.assign(width * height, 0);
    if (contour.empty()) return;
    // Flatten the outline to a polyline (ignoring Beziers for now)
    std::vector<Vec2> pts;
    for (size_t i = 0; i < contour.size(); ++i) {
        pts.push_back(Vec2(contour[i].x * scale + shiftX, contour[i].y * scale + shiftY));
    }
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
            for (int x = x0; x <= x1; ++x) {
                bitmap[y * width + x] = 255;
            }
        }
    }
}

const Glyph* Font::getGlyph(uint32_t codepoint) {
    auto it = glyphCache.find(codepoint);
    if (it != glyphCache.end()) return &it->second;
    if (!loadGlyph(codepoint)) return nullptr;
    return &glyphCache[codepoint];
}

bool Font::loadGlyph(uint32_t codepoint) {
    int glyphIndex = getGlyphIndex(codepoint);
    if (glyphIndex < 0) return false;
    std::vector<Vec2> contour;
    std::vector<uint8_t> onCurve;
    if (!getGlyphOutline(glyphIndex, contour, onCurve)) return false;
    int x0, y0, x1, y1;
    if (!getGlyphBox(glyphIndex, &x0, &y0, &x1, &y1)) return false;
    int width = x1 - x0;
    int height = y1 - y0;
    if (width <= 0 || height <= 0) return false;
    float scale = fontSize / info.unitsPerEm;
    std::vector<uint8_t> bitmap;
    rasterizeGlyph(contour, onCurve, width, height, bitmap, scale, -x0 * scale, -y0 * scale);
    Glyph glyph;
    glyph.glyphIndex = glyphIndex;
    glyph.width = width;
    glyph.height = height;
    glyph.bitmap = std::move(bitmap);
    int advance = 0, lsb = 0;
    getGlyphMetrics(glyphIndex, &advance, &lsb);
    glyph.advanceX = advance * scale;
    glyph.advanceY = 0;
    glyph.bearingX = lsb * scale;
    glyph.bearingY = 0; // Not used for horizontal layout
    glyphCache[codepoint] = std::move(glyph);
    return true;
}

float Font::getLineHeight() const {
    return (info.ascent - info.descent + info.lineGap) * fontSize / info.unitsPerEm;
}
float Font::getAscender() const {
    return info.ascent * fontSize / info.unitsPerEm;
}
float Font::getDescender() const {
    return info.descent * fontSize / info.unitsPerEm;
}

// Helper: decode next UTF-8 codepoint from a string
static bool utf8_next(const std::string& str, size_t& i, uint32_t& codepoint) {
    if (i >= str.size()) return false;
    unsigned char c = str[i];
    if (c < 0x80) {
        codepoint = c;
        i += 1;
        return true;
    } else if ((c & 0xE0) == 0xC0 && i + 1 < str.size()) {
        codepoint = ((c & 0x1F) << 6) | (str[i+1] & 0x3F);
        i += 2;
        return true;
    } else if ((c & 0xF0) == 0xE0 && i + 2 < str.size()) {
        codepoint = ((c & 0x0F) << 12) | ((str[i+1] & 0x3F) << 6) | (str[i+2] & 0x3F);
        i += 3;
        return true;
    } else if ((c & 0xF8) == 0xF0 && i + 3 < str.size()) {
        codepoint = ((c & 0x07) << 18) | ((str[i+1] & 0x3F) << 12) | ((str[i+2] & 0x3F) << 6) | (str[i+3] & 0x3F);
        i += 4;
        return true;
    }
    // Invalid UTF-8, skip one byte
    i += 1;
    codepoint = 0xFFFD; // replacement char
    return true;
}

void Canvas::drawText(const std::string& text, float x, float y, Font* font, const Color& color) {
    if (font == nullptr || text.empty()) {
        return;
    }
    float penX = x;
    float penY = y;
    uint32_t prevChar = 0;
    size_t i = 0;
    while (i < text.size()) {
        uint32_t codepoint;
        if (!utf8_next(text, i, codepoint)) break;
        if (prevChar != 0) {
            penX += font->getKerning(prevChar, codepoint);
        }
        const Glyph* glyph = font->getGlyph(codepoint);
        if (glyph) {
            float glyphX = penX + glyph->bearingX;
            float glyphY = penY - glyph->bearingY;
            if (!glyph->bitmap.empty()) {
                drawGlyphBitmap(*glyph, glyphX, glyphY, color);
            } else if (!glyph->path.commands.empty()) {
                // Vector font rendering not implemented
            }
            penX += glyph->advanceX;
            penY += glyph->advanceY;
        }
        prevChar = codepoint;
    }
}

float Canvas::measureText(const std::string& text, Font* font) {
    if (font == nullptr || text.empty()) {
        return 0.0f;
    }
    float width = 0.0f;
    uint32_t prevChar = 0;
    size_t i = 0;
    while (i < text.size()) {
        uint32_t codepoint;
        if (!utf8_next(text, i, codepoint)) break;
        if (prevChar != 0) {
            width += font->getKerning(prevChar, codepoint);
        }
        const Glyph* glyph = font->getGlyph(codepoint);
        if (glyph) {
            width += glyph->advanceX;
        }
        prevChar = codepoint;
    }
    return width;
}

void Canvas::drawGlyphBitmap(const Glyph& glyph, float x, float y, const Color& color) {
    if (glyph.bitmap.empty() || glyph.width <= 0 || glyph.height <= 0) {
        return;
    }
    
    // Render the bitmap
    for (int py = 0; py < glyph.height; py++) {
        for (int px = 0; px < glyph.width; px++) {
            int pixelX = static_cast<int>(x) + px;
            int pixelY = static_cast<int>(y) + py;
            
            uint8_t alpha = glyph.bitmap[py * glyph.width + px];
            if (alpha > 0) {
                Color pixelColor = color;
                pixelColor.a = (color.a * alpha) / 255;
                
                if (pixelColor.a > 0) {
                    blendPixel(pixelX, pixelY, pixelColor);
                }
            }
        }
    }
}

// Bezier curve utilities implementation
namespace bezier {
    Vec2 evalQuadratic(const Vec2& p0, const Vec2& p1, const Vec2& p2, float t) {
        float mt = 1.0f - t;
        float mt2 = mt * mt;
        float t2 = t * t;
        return p0 * mt2 + p1 * (2.0f * mt * t) + p2 * t2;
    }

    Vec2 evalCubic(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, float t) {
        float mt = 1.0f - t;
        float mt2 = mt * mt;
        float mt3 = mt2 * mt;
        float t2 = t * t;
        float t3 = t2 * t;
        return p0 * mt3 + p1 * (3.0f * mt2 * t) + p2 * (3.0f * mt * t2) + p3 * t3;
    }

    Vec2 quadraticDerivative(const Vec2& p0, const Vec2& p1, const Vec2& p2, float t) {
        return (p1 - p0) * 2.0f * (1.0f - t) + (p2 - p1) * 2.0f * t;
    }

    Vec2 cubicDerivative(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, float t) {
        float mt = 1.0f - t;
        float mt2 = mt * mt;
        float t2 = t * t;
        return (p1 - p0) * 3.0f * mt2 + (p2 - p1) * 6.0f * mt * t + (p3 - p2) * 3.0f * t2;
    }

    void splitQuadratic(const Vec2& p0, const Vec2& p1, const Vec2& p2, Vec2* left, Vec2* right, float t) {
        Vec2 p01 = p0 + (p1 - p0) * t;
        Vec2 p12 = p1 + (p2 - p1) * t;
        Vec2 p012 = p01 + (p12 - p01) * t;

        if (left) {
            left[0] = p0;
            left[1] = p01;
            left[2] = p012;
        }

        if (right) {
            right[0] = p012;
            right[1] = p12;
            right[2] = p2;
        }
    }

    void splitCubic(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, Vec2* left, Vec2* right, float t) {
        Vec2 p01 = p0 + (p1 - p0) * t;
        Vec2 p12 = p1 + (p2 - p1) * t;
        Vec2 p23 = p2 + (p3 - p2) * t;
        Vec2 p012 = p01 + (p12 - p01) * t;
        Vec2 p123 = p12 + (p23 - p12) * t;
        Vec2 p0123 = p012 + (p123 - p012) * t;

        if (left) {
            left[0] = p0;
            left[1] = p01;
            left[2] = p012;
            left[3] = p0123;
        }

        if (right) {
            right[0] = p0123;
            right[1] = p123;
            right[2] = p23;
            right[3] = p3;
        }
    }

    // Blend2d-style offsetting algorithm for quadratic curves
    void offsetQuadratic(const Vec2& p0, const Vec2& p1, const Vec2& p2, Vec2* result, float distance, int quality) {
        if (quality <= 0) quality = 1;

        // Approximate the curve with multiple cubic segments using adaptive subdivision
        std::vector<Vec2> points(quality + 1);
        
        for (int i = 0; i <= quality; i++) {
            float t = static_cast<float>(i) / quality;
            Vec2 pt = evalQuadratic(p0, p1, p2, t);
            Vec2 deriv = quadraticDerivative(p0, p1, p2, t);
            
            // Get the normalized perpendicular vector at this point
            Vec2 normal = deriv.perpendicular().normalize();
            
            // Offset the point by the distance in the normal direction
            points[i] = pt + normal * distance;
        }

        // Convert the approximation back to a cubic Bezier curve
        // For simple approximation, we'll take the first and last points,
        // and compute control points based on the approximation
        result[0] = points[0];
        result[2] = points[quality];

        // Compute control point as in blend2d: approximate average tangent
        Vec2 tangent = (points[1] - points[0]).normalize() + 
                       (points[quality] - points[quality-1]).normalize();
        float tangentLen = tangent.length();
        
        if (tangentLen > 1e-6f) {
            tangent = tangent * (1.0f / tangentLen);
            // Control point at 1/3 of the curve approximation
            result[1] = points[0] + tangent * (points[quality] - points[0]).length() / 3.0f;
        } else {
            // Fallback if we have numerical issues
            result[1] = (points[0] + points[quality]) * 0.5f;
        }
    }

    // Blend2d-style offsetting algorithm for cubic curves
    void offsetCubic(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, Vec2* result, float distance, int quality) {
        if (quality <= 0) quality = 1;

        // Approximate the curve with multiple line segments using adaptive subdivision
        std::vector<Vec2> points(quality + 1);
        
        for (int i = 0; i <= quality; i++) {
            float t = static_cast<float>(i) / quality;
            Vec2 pt = evalCubic(p0, p1, p2, p3, t);
            Vec2 deriv = cubicDerivative(p0, p1, p2, p3, t);
            
            // Get the normalized perpendicular vector at this point
            Vec2 normal = deriv.perpendicular().normalize();
            
            // Offset the point by the distance in the normal direction
            points[i] = pt + normal * distance;
        }

        // Convert the approximation back to a cubic Bezier curve
        // Using the technique from Blend2d: approximating the offset curve with a cubic
        result[0] = points[0];
        result[3] = points[quality];

        // Compute control points that preserve the tangent directions at endpoints
        Vec2 tangent0 = (points[1] - points[0]).normalize();
        Vec2 tangent1 = (points[quality] - points[quality-1]).normalize();
        
        float chord = (points[quality] - points[0]).length();
        result[1] = points[0] + tangent0 * (chord / 3.0f);
        result[2] = points[quality] - tangent1 * (chord / 3.0f);
    }
}

// Implement path offsetting
Path Path::offsetPath(float distance, int quality) const {
    Path result;
    if (commands.empty()) return result;
    
    Vec2 startPoint;
    Vec2 currentPoint;
    bool hasFirstPoint = false;
    
    for (const auto& cmd : commands) {
        switch (cmd.type) {
            case CommandType::MoveTo: {
                currentPoint = cmd.points[0];
                if (!hasFirstPoint) {
                    startPoint = currentPoint;
                    hasFirstPoint = true;
                }
                result.moveTo(currentPoint.x, currentPoint.y);
                break;
            }
            
            case CommandType::LineTo: {
                Vec2 next = cmd.points[0];
                Vec2 dir = (next - currentPoint).normalize();
                Vec2 normal = dir.perpendicular();
                
                Vec2 offsetStart = currentPoint + normal * distance;
                Vec2 offsetEnd = next + normal * distance;
                
                result.lineTo(offsetEnd.x, offsetEnd.y);
                currentPoint = next;
                break;
            }
            
            case CommandType::QuadraticTo: {
                Vec2 control = cmd.points[0];
                Vec2 end = cmd.points[1];
                
                // Offset the curve using Blend2d-style algorithm
                Vec2 offsetCurve[3];
                bezier::offsetQuadratic(currentPoint, control, end, offsetCurve, distance, quality);
                
                // Add the offset curve to our path
                result.quadraticTo(offsetCurve[1].x, offsetCurve[1].y, offsetCurve[2].x, offsetCurve[2].y);
                currentPoint = end;
                break;
            }
            
            case CommandType::CubicTo: {
                Vec2 control1 = cmd.points[0];
                Vec2 control2 = cmd.points[1];
                Vec2 end = cmd.points[2];
                
                // Offset the curve using Blend2d-style algorithm
                Vec2 offsetCurve[4];
                bezier::offsetCubic(currentPoint, control1, control2, end, offsetCurve, distance, quality);
                
                // Add the offset curve to our path
                result.cubicTo(offsetCurve[1].x, offsetCurve[1].y, offsetCurve[2].x, offsetCurve[2].y, offsetCurve[3].x, offsetCurve[3].y);
                currentPoint = end;
                break;
            }
        }
    }
    
    // Close the path if the original was closed
    if (closed) {
        result.close();
    }
    
    return result;
}

float Font::getKerning(uint32_t first, uint32_t second) const {
    if (info.kern == 0) return 0.0f;
    const unsigned char* kern = info.data + info.kern;
    uint16_t nTables = ttUSHORT(kern + 2);
    size_t offset = 4;
    for (uint16_t t = 0; t < nTables; ++t) {
        const unsigned char* subtable = kern + offset;
        uint16_t version = ttUSHORT(subtable);
        uint16_t length = ttUSHORT(subtable + 2);
        uint16_t coverage = ttUSHORT(subtable + 4);
        if ((coverage & 0xFF) == 0) { // format 0 only
            uint16_t nPairs = ttUSHORT(subtable + 6);
            uint16_t pairOffset = 14;
            int leftGlyph = getGlyphIndex(first);
            int rightGlyph = getGlyphIndex(second);
            for (uint16_t i = 0; i < nPairs; ++i) {
                const unsigned char* pair = subtable + pairOffset + i * 6;
                int left = ttUSHORT(pair);
                int right = ttUSHORT(pair + 2);
                int value = (int16_t)ttUSHORT(pair + 4);
                if (left == leftGlyph && right == rightGlyph) {
                    // Kerning value is in font units, scale to font size
                    return value * (fontSize / info.unitsPerEm);
                }
            }
        }
        offset += length;
    }
    return 0.0f;
}

} // namespace nanovg2d
