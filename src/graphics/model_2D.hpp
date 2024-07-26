#pragma once

#include <RAPID.H>
#include <memory>
#include <vector>

#include "graphic_primitives.hpp"

/**
 * Class representing a model in a two dimensinal world consisting of triangles.
 */
class Model_2D {
  private:
    std::vector<Triangle_2D> triangles;
    std::vector<Triangle_2D> triangles_transformed; // triangles model transformed according to the rotation
                                                    // and translation

    Color color; // color of the model

    double rot_matrix[3][3];   // rotation matrix (defines current rotation of the model in the world)
    double translation_vec[3]; // current model position in the world (3rd coordinate is always zero)
                               // -- 3D transformations are used above for easier usage of the RAPID lib, but Model_2D
                               // -- supports only 2D rendering (via renderer class)

    std::unique_ptr<RAPID_model> rapid_model; // model for collision detection

  public:
    Model_2D() = delete; // model should be initalized with the triangles parameter (see other contructor)
    Model_2D(const std::vector<Triangle_2D> &triangles, const Color &color);

    /** Rotates the model by the angle and places it in the word at (x, y) coordinates */
    void move(double x, double y, double angle);
    /** Checks collision between models and returns true if they collide, false if not. */
    bool collides_with(Model_2D &other_model);
    void set_color(const Color &color);
    Color get_color() const;
    /** Returns the transformed triangles */
    const std::vector<Triangle_2D> &get_model() const;

  private:
    void transform_triangle(Triangle_2D &triangle);
};
