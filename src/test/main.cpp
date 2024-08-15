#include "environment.hpp"

void test0() {
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

    Environment env(std::string("test0"), robot, 50.0, 50.0);

    env.add_rect_obstacle(1.0, 10.0, 25, 25, 0);

    std::array<double, 3> start_state = {8.0, 25.0, 0.0};
    std::array<double, 3> goal_state = {42.0, 25.0, 0.0};
    env.run(start_state, goal_state, 500, 500, 2.0, 1.0);
}

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
    env.run(start_state, goal_state, 50000, 10000, 1.0, 1.0);
}

void test2() {
    std::vector<Triangle_2D> robot;
    robot.push_back({Point_2D(-6, -1.5), Point_2D(6, -1.5), Point_2D(-6, 1.5)});
    robot.push_back({Point_2D(6, 1.5), Point_2D(6, -1.5), Point_2D(-6, 1.5)});
    robot.push_back({Point_2D(-9, -6), Point_2D(-6, -6), Point_2D(-9, 6)});
    robot.push_back({Point_2D(-6, 6), Point_2D(-6, -6), Point_2D(-9, 6)});
    robot.push_back({Point_2D(9, 6), Point_2D(6, 6), Point_2D(9, -6)});
    robot.push_back({Point_2D(6, -6), Point_2D(6, 6), Point_2D(9, -6)});

    Environment env(std::string("test2"), robot, 100.0, 50.0);
    env.set_start_and_goal_width(20);
    env.set_graph_width(6, 6);

    env.add_rect_obstacle(3, 20, 30, 52, 0);
    env.add_rect_obstacle(3, 30, 30, 12, 0);

    env.add_rect_obstacle(3, 20, 60, -2, 0);
    env.add_rect_obstacle(3, 32, 60, 40, 0);

    std::array<double, 3> start_state = {8.0, 25.0, M_PI_2};
    std::array<double, 3> goal_state = {92.0, 25.0, M_PI_2};
    env.run(start_state, goal_state, 50000, 10000, 2.0, 1.0);
}

void test3() {
    std::vector<Triangle_2D> robot;
    robot.push_back({Point_2D(-4.5, -1.125), Point_2D(4.5, -1.125), Point_2D(-4.5, 1.125)});
    robot.push_back({Point_2D(4.5, 1.125), Point_2D(4.5, -1.125), Point_2D(-4.5, 1.125)});
    robot.push_back({Point_2D(-6.75, -4.5), Point_2D(-4.5, -4.5), Point_2D(-6.75, 4.5)});
    robot.push_back({Point_2D(-4.5, 4.5), Point_2D(-4.5, -4.5), Point_2D(-6.75, 4.5)});
    robot.push_back({Point_2D(6.75, 4.5), Point_2D(4.5, 4.5), Point_2D(6.75, -4.5)});
    robot.push_back({Point_2D(4.5, -4.5), Point_2D(4.5, 4.5), Point_2D(6.75, -4.5)});

    Environment env(std::string("test3"), robot, 100.0, 50.0);
    env.set_start_and_goal_width(20);
    env.set_graph_width(6, 6);

    env.add_rect_obstacle(100, 1, 50, 50, 0);
    env.add_rect_obstacle(100, 1, 50, 0, 0);
    env.add_rect_obstacle(1, 50, 0, 25, 0);
    env.add_rect_obstacle(1, 50, 100, 25, 0);

    env.add_rect_obstacle(3, 22, 32, 25, 0);
    env.add_rect_obstacle(3, 16, 25.5, 36, M_PI_2);
    env.add_rect_obstacle(3, 16, 25.5, 14, M_PI_2);

    env.add_obstacle({{Point_2D(-6.0, 0.0), Point_2D(6.0, 0.0), Point_2D(6.0, 10.0)}}, 60, 30, 0.2);
    env.add_obstacle({{Point_2D(-5.0, 0.0), Point_2D(5.0, 0.0), Point_2D(5.0, 9.0)}}, 58, 7, 2.0);
    env.add_obstacle({{Point_2D(-7.0, 0.0), Point_2D(7.0, 0.0), Point_2D(0.0, 7.0)}}, 75, 15, 2.0);
    env.add_obstacle({{Point_2D(-4.0, 0.0), Point_2D(4.0, 0.0), Point_2D(0.0, 4.0)}}, 80, 40, 0.0);

    std::array<double, 3> start_state = {24.0, 25.0, M_PI_2};
    std::array<double, 3> goal_state = {92.0, 25.0, M_PI_2};
    env.run(start_state, goal_state, 50000, 10000, 2.0, 1.0);
}

int main() {
    test0();
    test1();
    test2();
    test3();
}
