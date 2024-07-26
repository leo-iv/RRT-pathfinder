#pragma once

#include <cairo/cairo.h>
#include <vector>

#include "model_2D.hpp"

/**
 * Wrapper class for the cairo graphic library.
 * Adds support for drawing the Model_2D class.
 */
class Renderer {
  private:
    cairo_surface_t *surface;
    cairo_t *cr;
    int image_width, image_height;
    double world_width, world_height;

  public:
    Renderer() = delete;
    Renderer(int image_width, int image_height, double world_width, double world_height);
    ~Renderer();
    /** Fills current image with color */
    void fill(const Color &color);
    /** Draws model to the current image */
    void draw_model(const Model_2D &model);
    /** Saves the current image state to png file */
    void save_to_png(const char *file_name);

  private:
    void draw_triangle_path(const Triangle_2D &triangle);
    Point_2D transform_point(const Point_2D &point);
};
