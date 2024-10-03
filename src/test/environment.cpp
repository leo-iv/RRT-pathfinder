#include "environment.hpp"

Environment::Environment(const std::string &name, const std::vector<Triangle_2D> &robot_model, double width,
                         double height)
    : name(name), robot(robot_model, ROBOT_COLOR), width(width), height(height),
      renderer(IMAGE_WIDTH, IMAGE_WIDTH * (height / width), width, height),
      solver({{{0.0, width}, {0.0, height}, {0.0, 2 * M_PI}}}, this) {}

Environment::~Environment() {
    for (Model_2D *obstacle : obstacles) {
        delete obstacle;
    }
}

void Environment::add_obstacle(const std::vector<Triangle_2D> &model, double x, double y, double angle) {
    Model_2D *new_obstacle = new Model_2D(model, OBSTACLES_COLOR);
    obstacles.push_back(new_obstacle);
    new_obstacle->move(x, y, angle);
}

void Environment::add_rect_obstacle(double width, double height, double x, double y, double angle) {
    std::vector<Triangle_2D> rect;
    rect.push_back({Point_2D(-1.0 * width / 2.0, -1.0 * height / 2.0), Point_2D(-1.0 * width / 2.0, height / 2.0),
                    Point_2D(width / 2.0, height / 2.0)});
    rect.push_back({Point_2D(-1.0 * width / 2.0, -1.0 * height / 2.0), Point_2D(width / 2.0, -1.0 * height / 2.0),
                    Point_2D(width / 2.0, height / 2.0)});
    add_obstacle(rect, x, y, angle);
}

void Environment::run(std::array<double, 3> &start, std::array<double, 3> &goal, int rrt_iters, int rrts_iters,
                      double rrts_step, double delta) {
    robot.move(start[0], start[1], start[2]); // move robot to starting position
    draw_env(start, goal);
    renderer.save_to_png((name + ".png").c_str());

    // test rrt:
    std::list<std::array<double, 3>> result_plan_rrt;
    solver.solve_rrt(result_plan_rrt, start, goal, rrt_iters, delta);
    Graph<3> &graph_rrt = solver.get_tree();

    draw_tree(graph_rrt);
    renderer.save_to_png((name + "_rrt_tree.png").c_str());

    if (!result_plan_rrt.empty()) {
        draw_result(result_plan_rrt);
        renderer.save_to_png((name + "_rrt_result.png").c_str());

        create_result_animation(result_plan_rrt, name + "_rrt_anim.gif");
    }

    // test rrt*:
    robot.move(start[0], start[1], start[2]); // move robot to starting position
    draw_env(start, goal);

    std::list<std::array<double, 3>> result_plan_rrts;
    solver.solve_k_rrts(result_plan_rrts, start, goal, rrts_iters, rrts_step, delta);
    Graph<3> &graph_rrts = solver.get_tree();

    draw_tree(graph_rrts);
    renderer.save_to_png((name + "_rrts_tree.png").c_str());

    if (!result_plan_rrts.empty()) {
        draw_result(result_plan_rrts);
        renderer.save_to_png((name + "_rrts_result.png").c_str());

        create_result_animation(result_plan_rrts, name + "_rrts_anim.gif");
    }
}

void Environment::draw_result(std::list<std::array<double, 3>> &result_plan) {
    auto &last_state = result_plan.front();
    for (const auto &state : result_plan) {
        renderer.draw_line(Point_2D(last_state[0], last_state[1]), Point_2D(state[0], state[1]), RESULT_COLOR,
                           GRAPH_WIDTH);
        renderer.draw_point(Point_2D(state[0], state[1]), VERTEX_WIDTH, RESULT_COLOR);
        last_state = state;
    }
}

void Environment::create_result_animation(std::list<std::array<double, 3>> &result_plan, std::string file_name) {
    auto &start = result_plan.front();
    auto &goal = result_plan.back();

    renderer.start_gif();
    for (auto &state : result_plan) {
        robot.move(state[0], state[1], state[2]);
        draw_env(start, goal);
        renderer.add_to_gif(ANIM_SPEED);
    }
    renderer.save_gif(file_name.c_str());
}

bool Environment::check_collision(double state[]) {
    robot.move(state[0], state[1], state[2]);
    for (Model_2D *obstacle : obstacles) {
        if (robot.collides_with(*obstacle)) {
            return false;
        }
    }

    return true;
}

void Environment::set_start_and_goal_width(double width) { START_GOAL_WIDTH = width; }

void Environment::set_graph_width(double vertex_radius, double line_width) {
    VERTEX_WIDTH = vertex_radius;
    GRAPH_WIDTH = line_width;
}

void Environment::set_anim_speed(int centi_seconds) { ANIM_SPEED = centi_seconds; }

void Environment::draw_tree_hlp(const Graph<3>::Vertex *root) {
    for (const std::pair<Graph<3>::Vertex *, double> &edge : root->edges) {
        Graph<3>::Vertex *child = edge.first;
        draw_tree_hlp(child); // recursively drawing subtrees
        // drawing line from root to child:
        renderer.draw_line(Point_2D(root->coords[0], root->coords[1]), Point_2D(child->coords[0], child->coords[1]),
                           GRAPH_COLOR, GRAPH_WIDTH);
    }
}

void Environment::draw_tree(const Graph<3> &graph) {
    draw_tree_hlp(graph.get_root()); // drawing edges
    // drawing verticies
    for (auto vertex : graph.vertices) {
        renderer.draw_point(Point_2D(vertex->coords[0], vertex->coords[1]), VERTEX_WIDTH, GRAPH_COLOR);
    }
}

void Environment::draw_env(std::array<double, 3> &start, std::array<double, 3> &goal) {
    renderer.fill(BACKGROUND_COLOR);
    renderer.draw_model(robot);
    for (Model_2D *obstacle : obstacles) {
        renderer.draw_model(*obstacle);
    }
    renderer.draw_point(Point_2D(start[0], start[1]), START_GOAL_WIDTH, START_COLOR); // drawing start position
    renderer.draw_point(Point_2D(goal[0], goal[1]), START_GOAL_WIDTH, GOAL_COLOR);    // drawing goal position
}
