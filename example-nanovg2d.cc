#include "nanovg2d.hh"

int main() {
    using namespace nanovg2d;
    const int width = 400, height = 300;
    Canvas canvas(width, height, 1); // 2x supersampling for AA
    canvas.clear(Color(255,255,255,255));
    // Filled rectangle
    canvas.fillRect(50, 50, 120, 80, Color(80,180,220,255));
    // Rectangle outline
    canvas.drawRect(200, 100, 150, 120, Color(220,80,80,255), 4.0f);
    // Rounded rectangle
    Path p;
    p.roundedRect(80, 180, 120, 60, 2);
    canvas.fillPath(p, Color(120,200,120,200));
    canvas.strokePath(p, Color(0,0,0,255), 2.0f);
    // Save to BMP
    writeBMP("output_rect_example.bmp", width, height, canvas.getData());
    return 0;
}
