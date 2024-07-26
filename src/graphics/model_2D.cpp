#include "model_2D.hpp"

#include <cmath>
#include <iostream> // TODO: DELETE

Model_2D::Model_2D(const std::vector<Triangle_2D> &triangles, const Color &color) : triangles(triangles), color(color) {
    // initializing RAPID model for collision detection
    rapid_model = std::make_unique<RAPID_model>();

    rapid_model->BeginModel();
    for (size_t i = 0; i < triangles.size(); i++) {
        Triangle_2D tri = triangles[i];
        // transforming 2D triangle to RAPID'S 3D model
        double first[3] = {tri.vertices[0].get_x(), tri.vertices[0].get_y(), 0.0};
        double second[3] = {tri.vertices[1].get_x(), tri.vertices[1].get_y(), 0.0};
        double third[3] = {tri.vertices[2].get_x(), tri.vertices[2].get_y(), 0.0};
        rapid_model->AddTri(first, second, third, i);
    }
    rapid_model->EndModel();

    move(0.0, 0.0, 0.0);
}

void Model_2D::move(double x, double y, double angle) {
    // computing the rotation matrix:
    rot_matrix[0][0] = cos(angle);
    rot_matrix[0][1] = -1.0 * sin(angle);
    rot_matrix[0][2] = 0.0;
    rot_matrix[1][0] = sin(angle);
    rot_matrix[1][1] = cos(angle);
    rot_matrix[1][2] = 0.0;
    rot_matrix[2][0] = 0.0;
    rot_matrix[2][1] = 0.0;
    rot_matrix[2][2] = 1.0;

    translation_vec[0] = x;
    translation_vec[1] = y;
    translation_vec[2] = 0.0; // z is always zero - Model "lives" only in 2D

    // transforming to the new model:
    triangles_transformed = triangles;
    for (Triangle_2D &triangle : triangles_transformed) {
        transform_triangle(triangle);
    }
}

bool Model_2D::collides_with(Model_2D &other_model) {
    RAPID_Collide(rot_matrix, translation_vec, rapid_model.get(), other_model.rot_matrix, other_model.translation_vec,
                  other_model.rapid_model.get());
    std::cout << "Num of contacts: " << RAPID_num_contacts << std::endl;
    return RAPID_num_contacts != 0;
}

void Model_2D::set_color(const Color &color) { this->color = color; }

Color Model_2D::get_color() const { return color; }

const std::vector<Triangle_2D> &Model_2D::get_model() const { return triangles_transformed; }

/**
 * Transforms given triangle according to the current rot_matrix and translation_vec
 */
void Model_2D::transform_triangle(Triangle_2D &triangle) {
    for (Point_2D &point : triangle.vertices) {
        double x = point.get_x();
        double y = point.get_y();
        point.set_x(rot_matrix[0][0] * x + rot_matrix[0][1] * y + translation_vec[0]);
        point.set_y(rot_matrix[1][0] * x + rot_matrix[1][1] * y + translation_vec[1]);
    }
}
