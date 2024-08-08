#pragma once

#include <cairo/cairo.h>
#include <vector>

#include "graph.hpp"
#include "model_2D.hpp"
#include "msf_gif.h"

/**
 * Wrapper class for the cairo graphics library and msf_gif library.
 * Adds support for drawing the Model_2D class and rendering gifs.
 */
class Renderer {
  private:
    cairo_surface_t *surface;
    cairo_t *cr;
    int image_width, image_height;
    double world_width, world_height;

    // msg_gif support:
    MsfGifState gif_state;

  public:
    Renderer() = delete;
    Renderer(int image_width, int image_height, double world_width, double world_height);
    ~Renderer();
    /** Fills current image with color */
    void fill(const Color &color);
    /** Draws model to the current image */
    void draw_model(const Model_2D &model);
    /** Draws line in the current image. */
    void draw_line(const Point_2D &from, const Point_2D &to, const Color &color, double width);
    /** Draws a circular point in the current image*/
    void draw_point(const Point_2D &point, double radius, const Color &color);
    /** Saves the current image state to png file */
    void save_to_png(const char *file_name);

    /** Initializes an empty gif.*/
    void start_gif();
    /** Adds current image surface to the gif. start_gif() method needs to be called before this method.*/
    void add_to_gif(int centi_seconds);
    /** Creates the result gif file and frees other gif data*/
    void save_gif(const char *filename);

  private:
    void draw_triangle_path(const Triangle_2D &triangle);
    Point_2D transform_point(const Point_2D &point);
};
