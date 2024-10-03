#pragma once

#include "model_2D.hpp"
#include "renderer.hpp"
#include "rrt.hpp"
#include <string>
#include <vector>

class Environment : public Collision_detector {
  private:
    static constexpr int IMAGE_WIDTH = 1920;
    const Color BACKGROUND_COLOR = {255, 255, 255};
    const Color ROBOT_COLOR = {111, 159, 156};
    const Color OBSTACLES_COLOR = {222, 196, 132};
    const Color GRAPH_COLOR = {87, 126, 137};
    const Color RESULT_COLOR = {225, 163, 111};
    const Color START_COLOR = {225, 163, 111};
    const Color GOAL_COLOR = {225, 163, 111};
    double START_GOAL_WIDTH = 30;
    double GRAPH_WIDTH = 10;
    double VERTEX_WIDTH = 10;
    int ANIM_SPEED = 10; // in centi seconds

    std::string name; // name for this environment

  protected:
    Model_2D robot;
    std::vector<Model_2D *> obstacles;

    double width;  // world width
    double height; // world height

    Renderer renderer;

    RRT_solver<3> solver;

  public:
    Environment(const std::string &name, const std::vector<Triangle_2D> &robot_model, double width, double height);
    ~Environment();
    /** Adds obstacle to the environment on the [x, y] position. */
    void add_obstacle(const std::vector<Triangle_2D> &model, double x, double y, double angle);
    /** Adds rectangular obstacle to the environment. */
    void add_rect_obstacle(double width, double height, double x, double y, double angle);
    /** Runs tests. */
    void run(std::array<double, 3> &start, std::array<double, 3> &goal, int rrt_iters, int rrts_iters, double rrts_step,
             double delta);
    bool check_collision(double state[]) override; // override from Collsion_detector interface

    /** Customization of visualization parameters */
    void set_start_and_goal_width(double width);
    void set_graph_width(double vertex_radius, double line_width);
    void set_anim_speed(int centi_seconds);

  private:
    void draw_result(std::list<std::array<double, 3>> &result_plan);
    void create_result_animation(std::list<std::array<double, 3>> &result_plan, std::string file_name);
    void draw_tree(const Graph<3> &graph);
    void draw_tree_hlp(const Graph<3>::Vertex *root);
    void draw_env(std::array<double, 3> &start, std::array<double, 3> &goal);
};
