#ifndef colour_h
#define colour_h

#include "utilities_package.h"
#include "vec3.h"

#include <iostream>
#include <fstream>


void write_colour(std::ostream& out, colour pixel_colour, int samples_per_pixel) {
    
    auto r = pixel_colour.x();
    auto g = pixel_colour.y();
    auto b = pixel_colour.z();

    // Replace NaN components with zero. See explanation in Ray Tracing: The Rest of Your Life.
    if (r != r) r = 0.0;
    if (g != g) g = 0.0;
    if (b != b) b = 0.0;

    // Divide the color by the number of samples and gamma-correct for gamma=2.0.
    auto scale = 1.0 / samples_per_pixel;
    r = sqrt(scale * r);
    g = sqrt(scale * g);
    b = sqrt(scale * b);

    
    // Write the translated [0,255] value of each color component.
    out << static_cast<int>(256 * clamp(r, 0.0, 0.999)) << ' '
        << static_cast<int>(256 * clamp(g, 0.0, 0.999)) << ' '
        << static_cast<int>(256 * clamp(b, 0.0, 0.999)) << '\n';
    
    /*
    // Convert values to integer values
    r = static_cast<int>(256 * clamp(r, 0.0, 0.999));
    g = static_cast<int>(256 * clamp(g, 0.0, 0.999));
    b = static_cast<int>(256 * clamp(b, 0.0, 0.999));
    */


}


void write_colour(std::ofstream& out, colour pixel_colour, int samples_per_pixel) {

    auto r = pixel_colour.x();
    auto g = pixel_colour.y();
    auto b = pixel_colour.z();

    // Replace NaN components with zero. See explanation in Ray Tracing: The Rest of Your Life.
    if (r != r) r = 0.0;
    if (g != g) g = 0.0;
    if (b != b) b = 0.0;

    // Divide the color by the number of samples and gamma-correct for gamma=2.0.
    auto scale = 1.0 / samples_per_pixel;
    r = sqrt(scale * r);
    g = sqrt(scale * g);
    b = sqrt(scale * b);


    // Write the translated [0,255] value of each color component.
    out << static_cast<int>(256 * clamp(r, 0.0, 0.999)) << ' '
        << static_cast<int>(256 * clamp(g, 0.0, 0.999)) << ' '
        << static_cast<int>(256 * clamp(b, 0.0, 0.999)) << '\n';

    /*
    // Convert values to integer values
    r = static_cast<int>(256 * clamp(r, 0.0, 0.999));
    g = static_cast<int>(256 * clamp(g, 0.0, 0.999));
    b = static_cast<int>(256 * clamp(b, 0.0, 0.999));
    */


}


#endif
