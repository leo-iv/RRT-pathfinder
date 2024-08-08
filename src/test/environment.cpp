#include "environment.hpp"

Environment::Environment(const std::string &name, const std::vector<Triangle_2D> &robot_model, double width,
                         double height)
    : name(name), robot(robot_model, ROBOT_COLOR), width(width), height(height),
      renderer(IMAGE_WIDTH, IMAGE_WIDTH * (height / width), width, height),
      rrt({{{0.0, width}, {0.0, height}, {0.0, M_PI}}}, this) {}

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

void Environment::run(std::array<double, 3> &start, std::array<double, 3> &goal, int iters, double delta) {
    robot.move(start[0], start[1], start[2]); // move robot to starting position
    draw_env(start, goal);
    renderer.save_to_png((name + std::string(".png")).c_str());

    std::list<std::array<double, 3>> result_plan;
    Graph<3> &graph = rrt.build_tree(result_plan, start, goal, iters, delta);

    draw_tree(graph); // drawing edges
    renderer.save_to_png((name + std::string("_tree.png")).c_str());

    if (!result_plan.empty()) {
        draw_result(result_plan);
        renderer.save_to_png((name + std::string("_result.png")).c_str());

        create_result_animation(result_plan);
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

void Environment::create_result_animation(std::list<std::array<double, 3>> &result_plan) {
    auto &start = result_plan.front();
    auto &goal = result_plan.back();

    renderer.start_gif();
    for (auto &state : result_plan) {
        robot.move(state[0], state[1], state[2]);
        draw_env(start, goal);
        renderer.add_to_gif(10);
    }
    renderer.save_gif((name + ".gif").c_str());
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
