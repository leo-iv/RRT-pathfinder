#include "renderer.hpp"

Renderer::Renderer(int image_width, int image_height, double world_width, double world_height)
    : image_width(image_width), image_height(image_height), world_width(world_width), world_height(world_height) {
    surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, image_width, image_height);
    cr = cairo_create(surface);
    cairo_scale(cr, image_width, image_height); // scaling to 1.0 x 1.0 coordinate space
}

Renderer::~Renderer() {
    cairo_destroy(cr);
    cairo_surface_destroy(surface);
}

void Renderer::fill(const Color &color) {
    cairo_set_source_rgb(cr, (double)color.R / 255.0, (double)color.G / 255.0, (double)color.B / 255.0);
    cairo_paint(cr);
}

void Renderer::draw_model(const Model_2D &model) {
    for (const Triangle_2D &triangle : model.get_model()) {
        draw_triangle_path(triangle);
    }

    Color color = model.get_color();
    cairo_set_source_rgb(cr, (double)color.R / 255.0, (double)color.G / 255.0, (double)color.B / 255.0);
    cairo_fill(cr);
}

void Renderer::save_to_png(const char *file_name) { cairo_surface_write_to_png(surface, file_name); }

void Renderer::draw_triangle_path(const Triangle_2D &triangle) {
    Point_2D transformed_point = transform_point(triangle.vertices[0]);
    cairo_move_to(cr, transformed_point.get_x(), transformed_point.get_y());

    transformed_point = transform_point(triangle.vertices[1]);
    cairo_line_to(cr, transformed_point.get_x(), transformed_point.get_y());

    transformed_point = transform_point(triangle.vertices[2]);
    cairo_line_to(cr, transformed_point.get_x(), transformed_point.get_y());

    cairo_close_path(cr);
}

/**
 * Transforms point from user (model) coordinate system to cairos 1.0x1.0 coordinate system
 */
Point_2D Renderer::transform_point(const Point_2D &point) {
    return Point_2D(point.get_x() / world_width, (world_height - point.get_y()) / world_height);
}
