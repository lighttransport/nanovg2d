#pragma once

#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <cstring>
#include <string>
#include <map>
#include <memory>

namespace nanovg2d {

// Forward declarations
class Font;

struct Vec2 {
    float x, y;
    Vec2() : x(0.0f), y(0.0f) {}
    Vec2(float _x, float _y) : x(_x), y(_y) {}
    Vec2 operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
    Vec2 operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
    Vec2 operator*(float s) const { return Vec2(x * s, y * s); }
    Vec2 operator/(float s) const { return Vec2(x / s, y / s); }
    float length() const { return std::sqrt(x*x + y*y); }
    Vec2 normalize() const { float len = length(); if (len > 1e-6f) return Vec2(x / len, y / len); return *this; }
    float dot(const Vec2& other) const { return x * other.x + y * other.y; }
    
    // Additional vector operations needed for Bezier offsetting
    Vec2 perpendicular() const { return Vec2(-y, x); }
    float cross(const Vec2& other) const { return x * other.y - y * other.x; }
    Vec2 operator-() const { return Vec2(-x, -y); }
};

// Bezier curve evaluation and offsetting utilities
namespace bezier {
    // Curve evaluation (de Casteljau)
    Vec2 evalQuadratic(const Vec2& p0, const Vec2& p1, const Vec2& p2, float t);
    Vec2 evalCubic(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, float t);
    
    // Derivatives
    Vec2 quadraticDerivative(const Vec2& p0, const Vec2& p1, const Vec2& p2, float t);
    Vec2 cubicDerivative(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, float t);
    
    // Curve splitting
    void splitQuadratic(const Vec2& p0, const Vec2& p1, const Vec2& p2, 
                        Vec2* left, Vec2* right, float t);
    void splitCubic(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3,
                    Vec2* left, Vec2* right, float t);
    
    // Offsetting
    void offsetQuadratic(const Vec2& p0, const Vec2& p1, const Vec2& p2, 
                         Vec2* result, float distance, int quality = 2);
    void offsetCubic(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, 
                     Vec2* result, float distance, int quality = 3);
}

struct Color {
    uint8_t r, g, b, a;
    Color() : r(0), g(0), b(0), a(255) {}
    Color(uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _a = 255) : r(_r), g(_g), b(_b), a(_a) {}
    static Color lerp(const Color& a, const Color& b, float t) {
        float r = a.r * (1.0f - t) + b.r * t;
        float g = a.g * (1.0f - t) + b.g * t;
        float b_val = a.b * (1.0f - t) + b.b * t;
        float alpha = a.a * (1.0f - t) + b.a * t;
        return Color(static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b_val), static_cast<uint8_t>(alpha));
    }
    uint32_t toRGBA() const { return (a << 24) | (b << 16) | (g << 8) | r; }
    Color withAlpha(uint8_t newAlpha) const { return Color(r, g, b, newAlpha); }
    Color scaleAlpha(float factor) const { uint8_t newAlpha = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, a * factor))); return Color(r, g, b, newAlpha); }
};

struct Path {
    enum class CommandType {
        MoveTo, LineTo, QuadraticTo, CubicTo
    };
    struct Command {
        CommandType type;
        Vec2 points[3];
        Command(CommandType t, const Vec2& p1) : type(t) { points[0] = p1; }
        Command(CommandType t, const Vec2& p1, const Vec2& p2) : type(t) { points[0] = p1; points[1] = p2; }
        Command(CommandType t, const Vec2& p1, const Vec2& p2, const Vec2& p3) : type(t) { points[0] = p1; points[1] = p2; points[2] = p3; }
    };
    std::vector<Command> commands;
    bool closed;
    Path() : closed(false) {}
    void moveTo(float x, float y);
    void lineTo(float x, float y);
    void close();
    void rect(float x, float y, float w, float h) {
        moveTo(x, y);
        lineTo(x + w, y);
        lineTo(x + w, y + h);
        lineTo(x, y + h);
        close();
    }
    void roundedRect(float x, float y, float w, float h, float r);
    void quadraticTo(float cx, float cy, float x, float y) { commands.emplace_back(CommandType::QuadraticTo, Vec2(cx, cy), Vec2(x, y)); }
    void cubicTo(float c1x, float c1y, float c2x, float c2y, float x, float y) { commands.emplace_back(CommandType::CubicTo, Vec2(c1x, c1y), Vec2(c2x, c2y), Vec2(x, y)); }
    void reset() { commands.clear(); closed = false; }
    
    // Add offset path capabilities
    Path offsetPath(float distance, int quality = 3) const;
};

// New struct to hold glyph information
struct Glyph {
    Path path;                   // Vector path of the glyph
    float advanceX;              // Horizontal advance width
    float advanceY;              // Vertical advance (usually 0 for horizontal layouts)
    float bearingX;              // Left bearing
    float bearingY;              // Top bearing
    int width;                   // Width of the glyph bitmap
    int height;                  // Height of the glyph bitmap
    std::vector<uint8_t> bitmap; // Optional glyph bitmap for direct rendering
    // Add glyph index for TrueType
    int glyphIndex = 0;
};

enum class AntialiasingLevel { None, Fast, Best };

// Font class for managing TrueType fonts
struct TrueTypeFontInfo {
    const unsigned char* data = nullptr;
    int fontOffset = 0;
    int cmap = 0, glyf = 0, loca = 0, head = 0, hhea = 0, hmtx = 0, maxp = 0, kern = 0;
    int numGlyphs = 0;
    int indexLocFormat = 0;
    int unitsPerEm = 0;
    int ascent = 0, descent = 0, lineGap = 0;
    int numHMetrics = 0;
    int locaTableSize = 0;
};

#if 0
// Helper functions for TrueType parsing
uint16_t ttUSHORT(const unsigned char* p);
int16_t ttSHORT(const unsigned char* p);
uint32_t ttULONG(const unsigned char* p);
int32_t ttLONG(const unsigned char* p);
#endif

class Font {
public:
    Font();
    ~Font();
    bool loadFromFile(const std::string& filename, float fontSize);
    const Glyph* getGlyph(uint32_t codepoint);
    float getKerning(uint32_t first, uint32_t second) const;
    float getLineHeight() const;
    float getAscender() const;
    float getDescender() const;
    // New helpers for TrueType
    int getGlyphIndex(uint32_t codepoint) const;
    bool getGlyphMetrics(int glyphIndex, int* advance, int* lsb) const;
    bool getGlyphBox(int glyphIndex, int* x0, int* y0, int* x1, int* y1) const;
    bool getGlyphOutline(int glyphIndex, std::vector<Vec2>& contour, std::vector<uint8_t>& onCurve) const;
private:
    TrueTypeFontInfo info;
    float fontSize = 0.0f;
    std::map<uint32_t, Glyph> glyphCache;
    std::vector<unsigned char> fontBuffer; // Holds font file data
    bool loadGlyph(uint32_t codepoint);
    int findGlyphIndex(uint32_t codepoint) const;
    // ... add more as needed ...
};

class Canvas {
public:
    Canvas(int width, int height, int supersample = 1);
    ~Canvas();
    void setAntialiasingLevel(AntialiasingLevel level) { aaLevel = level; }
    AntialiasingLevel getAntialiasingLevel() const { return aaLevel; }
    void clear(const Color& color);
    void fillPath(const Path& path, const Color& color);
    void strokePath(const Path& path, const Color& color, float width);
    void drawRect(float x, float y, float w, float h, const Color& color, float strokeWidth = 1.0f);
    void fillRect(float x, float y, float w, float h, const Color& color);
    
    // New text rendering methods
    void drawText(const std::string& text, float x, float y, Font* font, const Color& color);
    float measureText(const std::string& text, Font* font);
    
    uint8_t* getData();
    const uint8_t* getData() const;
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    int getSupersample() const { return supersample; }
private:
    int width, height, supersample;
    uint8_t* data;
    uint8_t* hiresData;
    bool dirty;
    AntialiasingLevel aaLevel;
    void setPixel(int x, int y, const Color& color);
    void blendPixel(int x, int y, const Color& color);
    void downsample();
    void flattenPath(const Path& path, std::vector<Vec2>& outPoints, float scale) const;
    void flattenQuadratic(const Vec2& p0, const Vec2& p1, const Vec2& p2, std::vector<Vec2>& outPoints, float scale) const;
    void flattenCubic(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, std::vector<Vec2>& outPoints, float scale) const;
    bool pointInPath(const std::vector<Vec2>& points, bool closed, const Vec2& point) const;
    void scanlineFill(const Path& path, const Color& color);
    void scanlineFillAA(const Path& path, const Color& color);
    float edgeDistance(const Vec2& p0, const Vec2& p1, const Vec2& point) const;
    void rasterizeLine(const Vec2& p0, const Vec2& p1, const Color& color, float width);
    void rasterizeLineAA(const Vec2& p0, const Vec2& p1, const Color& color, float width);
    
    // New helper methods for text rendering
    void drawGlyphBitmap(const Glyph& glyph, float x, float y, const Color& color);
};

bool init();
void shutdown();

// Font management
Font* createFont();
void destroyFont(Font* font);

// Path management
Path* createPath();
void destroyPath(Path* path);

// File I/O
bool writeBMP(const std::string& filename, int width, int height, const uint8_t* data);

} // namespace nanovg2d
