#pragma once

#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <cstring>
#include <string>

namespace nanovg2d {

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
};

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
};

enum class AntialiasingLevel { None, Fast, Best };

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
};

bool init();
void shutdown();
Path* createPath();
void destroyPath(Path* path);
bool writeBMP(const std::string& filename, int width, int height, const uint8_t* data);

} // namespace nanovg2d
