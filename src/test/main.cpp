#include "environment.hpp"

void test1() {
    std::vector<Triangle_2D> robot;
    robot.push_back({Point_2D(-3, 0), Point_2D(3, 0), Point_2D(0, 3)});
    robot.push_back({Point_2D(-3, 0), Point_2D(-3, 6), Point_2D(0, 3)});
    robot.push_back({Point_2D(3, 6), Point_2D(3, 0), Point_2D(0, 3)});
    robot.push_back({Point_2D(-3, 0), Point_2D(3, 0), Point_2D(0, -3)});
    robot.push_back({Point_2D(-3, 0), Point_2D(0, -3), Point_2D(-3, -6)});
    robot.push_back({Point_2D(3, 0), Point_2D(3, -6), Point_2D(0, -3)});
    robot.push_back({Point_2D(-3, 0), Point_2D(-3, 3), Point_2D(-6, 3)});
    robot.push_back({Point_2D(-3, 0), Point_2D(-3, -3), Point_2D(-6, -3)});
    robot.push_back({Point_2D(3, 0), Point_2D(3, -3), Point_2D(6, -3)});
    robot.push_back({Point_2D(3, 0), Point_2D(3, 3), Point_2D(6, 3)});

    Environment env(std::string("test1"), robot, 50.0, 50.0);

    env.add_rect_obstacle(6.0, 5.0, 18, 30, M_PI_2);
    env.add_rect_obstacle(10.0, 5.0, 40, 40, 3 * M_PI_4);
    env.add_rect_obstacle(30.0, 5.0, 10, 25, 0.0);
    env.add_rect_obstacle(30.0, 5.0, 50, 10, 0);

    std::array<double, 3> start_state = {8.0, 8.0, 0.0};
    std::array<double, 3> goal_state = {10.0, 40.0, 0.0};
    env.run(start_state, goal_state, 10000, 1.0);
}

int main() { test1(); }
