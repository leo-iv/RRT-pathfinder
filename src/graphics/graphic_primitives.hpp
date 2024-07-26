#pragma once
/**
 * Collection of basic graphic primitives (such as points, colors, ...)
 */

#include <cstdint>

struct Point_2D {
    double coords[2];

    Point_2D(double x, double y) {
        coords[0] = x;
        coords[1] = y;
    }
    // getters and setters to remove the need to use coords array indices
    double get_x() const { return coords[0]; }
    void set_x(double x) { coords[0] = x; }
    double get_y() const { return coords[1]; }
    void set_y(double y) { coords[1] = y; }
};

struct Color {
    uint8_t R, G, B;
};

struct Triangle_2D {
    Point_2D vertices[3];
};
