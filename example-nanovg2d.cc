#include <iostream>
#include <cmath>
#include "nanovg2d.hh"

int main(int argc, char** argv) {
    // Initialize NanoVG2D
    nanovg2d::init();

    // Create a 800x600 canvas with 4x supersampling for smoother lines
    const int width = 800;
    const int height = 600;
    nanovg2d::Canvas* canvas = new nanovg2d::Canvas(width, height, 4);
    
    // Clear to white
    canvas->clear(nanovg2d::Color(255, 255, 255, 255));
    
    // Create original path with various curves
    nanovg2d::Path originalPath;
    
    // Start path with some straight segments
    originalPath.moveTo(100, 300);
    originalPath.lineTo(200, 100);
    
    // Add a quadratic Bezier curve
    originalPath.quadraticTo(300, 50, 400, 300);
    
    // Add a cubic Bezier curve
    originalPath.cubicTo(500, 500, 600, 50, 700, 300);
    
    // Draw the original path in black
    canvas->strokePath(originalPath, nanovg2d::Color(0, 0, 0, 255), 2.0f);
    
    // Create offset paths at different distances
    float offsetDistances[3] = {20.0f, 40.0f, 60.0f};
    nanovg2d::Color offsetColors[3] = {
        nanovg2d::Color(255, 0, 0, 255),   // Red
        nanovg2d::Color(0, 255, 0, 255),   // Green
        nanovg2d::Color(0, 0, 255, 255)    // Blue
    };
    
    // Generate and draw offset paths
    for (int i = 0; i < 3; i++) {
        float distance = offsetDistances[i];
        nanovg2d::Path offsetPath = originalPath.offsetPath(distance, 10);
        canvas->strokePath(offsetPath, offsetColors[i], 2.0f);
        
        // Also draw an offset path with negative distance (on the other side)
        nanovg2d::Path negativeOffsetPath = originalPath.offsetPath(-distance, 10);
        canvas->strokePath(negativeOffsetPath, offsetColors[i], 2.0f);
    }
    
    // Add some text explaining the example
    nanovg2d::Path textBg;
    textBg.roundedRect(10, 10, 320, 80, 5);
    canvas->fillPath(textBg, nanovg2d::Color(240, 240, 240, 240));
    
    // Draw a legend
    int legendY = 30;
    canvas->drawRect(20, legendY, 30, 2, nanovg2d::Color(0, 0, 0, 255), 2);
    // No text rendering implemented yet, so we'll just leave space for it
    
    legendY += 15;
    canvas->drawRect(20, legendY, 30, 2, nanovg2d::Color(255, 0, 0, 255), 2);
    
    legendY += 15;
    canvas->drawRect(20, legendY, 30, 2, nanovg2d::Color(0, 255, 0, 255), 2);
    
    legendY += 15;
    canvas->drawRect(20, legendY, 30, 2, nanovg2d::Color(0, 0, 255, 255), 2);
    
    // Save to BMP file
    canvas->getData(); // Make sure we've downsampled
    nanovg2d::writeBMP("bezier_offset_example.bmp", width, height, canvas->getData());
    
    std::cout << "Generated bezier_offset_example.bmp" << std::endl;
    
    // Clean up
    delete canvas;
    nanovg2d::shutdown();
    return 0;
}
