
#include "graph.hpp"
#include "renderer.hpp"
#include <cmath>
#include <iostream>
#include <vector>

void graphics_test() {
    Color blue = {0, 0, 255};
    Color green = {0, 255, 0};
    Color white = {255, 255, 255};

    std::vector<Triangle_2D> model1_tri;
    model1_tri.push_back({Point_2D(-0.5, 0.0), Point_2D(0.5, 0.0), Point_2D(0.0, 0.5)});
    model1_tri.push_back({Point_2D(-0.5, 0.0), Point_2D(-0.5, 1.0), Point_2D(0.0, 0.5)});
    model1_tri.push_back({Point_2D(0.5, 1.0), Point_2D(0.5, 0.0), Point_2D(0.0, 0.5)});
    Model_2D model1(model1_tri, blue);

    std::vector<Triangle_2D> model2_tri;
    model2_tri.push_back({Point_2D(-0.5, 0.0), Point_2D(0.5, 0.0), Point_2D(0.0, 0.5)});
    Model_2D model2(model2_tri, green);

    model1.move(2.2, 2.8, 3.1);
    model2.move(2.5, 3.0, -0.5);

    std::cout << model1.collides_with(model2) << std::endl;

    Renderer renderer(1000, 1000, 5.0, 5.0);
    renderer.fill(white);
    renderer.draw_model(model1);
    renderer.draw_model(model2);
    renderer.save_to_png("output.png");
}

int main() { graphics_test(); }
