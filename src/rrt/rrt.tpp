#pragma once

#include "rrt.hpp"
#include <array>
#include <array_math.hpp>
#include <limits>

template <int dimension>
RRT_solver<dimension>::RRT_solver(const std::array<std::array<double, 2>, dimension> &boundaries,
                                  Collision_detector *detector)
    : boundaries(boundaries), detector(detector), gen(/*rd()*/ 42){};

template <int dimension>
void RRT_solver<dimension>::solve_rrt(std::list<std::array<double, dimension>> &result_plan,
                                      std::array<double, dimension> &start_state,
                                      std::array<double, dimension> &goal_state, int iters, double delta) {
    graph.clear();
    graph.add_vertex(start_state);
    bool quit = false;

    for (int i = 0; !quit && i < iters; i++) {

        std::array<double, dimension> random_state;
        if ((i % GOAL_INSERTION_ITER) == 0) {
            // try adding goal state instead of random
            random_state = goal_state;
        } else {
            random_state = get_random_state();
        }

        auto nearest_vertex = graph.get_nearest(random_state);

        std::vector<std::array<double, dimension>> new_states;
        bool path_is_collision_free = get_free_states(new_states, nearest_vertex->coords, random_state, delta);

        if (((i % GOAL_INSERTION_ITER) == 0) && path_is_collision_free) { // adding goal state and path to it is free
            quit = true; // goal state reached -> quitting in the next iter
        }

        // adding new states to the graph:
        for (auto &new_state : new_states) {
            auto new_vertex =
                graph.connect_new_vertex(new_state, nearest_vertex, vector_distance(nearest_vertex->coords, new_state));
            nearest_vertex = new_vertex;
        }
    }

    if (quit) { // found path to goal
        construct_result_plan(result_plan, graph.get_last());
    } else {
        result_plan.clear();
    }
}

template <int dimension>
void RRT_solver<dimension>::solve_k_rrts(std::list<std::array<double, dimension>> &result_plan,
                                         std::array<double, dimension> &start_state,
                                         std::array<double, dimension> &goal_state, int iters, double step,
                                         double delta) {
    graph.clear();
    graph.add_vertex(start_state);

    for (int i = 0; i < iters; i++) {
        std::array<double, dimension> random_state = get_random_state();
        auto nearest = graph.get_nearest(random_state);
        std::array<double, dimension> new_state = move_a_step(nearest->coords, random_state, step);

        if (is_collision_free(nearest->coords, new_state, delta)) {
            int k = (int)(2 * M_E * std::log(graph.size())); // number of nearest neighbours
            std::vector<typename Graph<dimension>::Vertex *> k_nearest;
            graph.get_k_nearest(k_nearest, new_state, k);

            auto min_state = nearest;
            double min_cost = nearest->cost + vector_distance(nearest->coords, new_state);
            for (auto neighbour : k_nearest) { // connectiong new vertex along minimum cost path
                if (is_collision_free(neighbour->coords, new_state, delta) &&
                    (neighbour->cost + vector_distance(neighbour->coords, new_state) < min_cost)) {
                    min_state = neighbour;
                    min_cost = neighbour->cost + vector_distance(neighbour->coords, new_state);
                }
            }

            auto new_vertex =
                graph.connect_new_vertex(new_state, min_state, vector_distance(min_state->coords, new_state));

            for (auto neighbour : k_nearest) { // rewiring the tree
                if (is_collision_free(new_vertex->coords, neighbour->coords, delta) &&
                    ((new_vertex->cost + vector_distance(new_vertex->coords, neighbour->coords)) < neighbour->cost)) {
                    graph.rewire_vertex(neighbour, new_vertex, vector_distance(new_vertex->coords, neighbour->coords));
                }
            }
        }
    }

    // try adding goal vertex at the end:
    typename Graph<dimension>::Vertex *min_vertex = nullptr;
    double min_cost = std::numeric_limits<double>::max();
    for (auto vertex : graph.vertices) {
        if (is_collision_free(vertex->coords, goal_state, delta) &&
            (vertex->cost + vector_distance(vertex->coords, goal_state) < min_cost)) {
            min_vertex = vertex;
            min_cost = vertex->cost + vector_distance(vertex->coords, goal_state);
        }
    }

    if (min_vertex != nullptr) {
        auto goal_vertex =
            graph.connect_new_vertex(goal_state, min_vertex, vector_distance(min_vertex->coords, goal_state));
        construct_result_plan(result_plan, goal_vertex, delta);
    } else {
        result_plan.clear();
    }
}

template <int dimension> Graph<dimension> &RRT_solver<dimension>::get_tree() { return graph; }

template <int dimension> std::array<double, dimension> RRT_solver<dimension>::get_random_state() {
    std::array<double, dimension> state;
    for (int i = 0; i < dimension; i++) {
        std::uniform_real_distribution<double> dis(boundaries[i][0], boundaries[i][1]);
        state[i] = dis(gen);
    }
    return state;
}

template <int dimension>
bool RRT_solver<dimension>::get_free_states(std::vector<std::array<double, dimension>> &new_states,
                                            std::array<double, dimension> &start, std::array<double, dimension> &stop,
                                            double delta) {
    std::array<double, dimension> direction = vector_diff(stop, start);
    double distance = vector_norm(direction);

    direction = vector_mult(1.0 / distance, direction); // normalizing to 1.0 norm

    int iters = (int)(distance / delta);
    for (int i = 1; i <= iters; i++) {
        auto new_state = vector_add(start, vector_mult(i * delta, direction));

        if (!detector->check_collision(new_state.data())) {
            return false;
        }
        new_states.push_back(new_state);
    }

    // try adding goal state at the end:
    if (detector->check_collision(stop.data())) {
        new_states.push_back(stop);
        return true;
    }

    return false;
}

template <int dimension>
bool RRT_solver<dimension>::is_collision_free(std::array<double, dimension> &start, std::array<double, dimension> &stop,
                                              double delta) {
    std::array<double, dimension> direction = vector_diff(stop, start);
    double distance = vector_norm(direction);

    direction = vector_mult(1.0 / distance, direction); // normalizing to 1.0 norm

    int iters = (int)(distance / delta);
    for (int i = 1; i <= iters; i++) {
        auto new_state = vector_add(start, vector_mult(i * delta, direction));

        if (!detector->check_collision(new_state.data())) {
            return false;
        }
    }

    // checking stop state aswell
    if (detector->check_collision(stop.data())) {
        return true;
    }

    return false;
}

template <int dimension>
std::array<double, dimension> RRT_solver<dimension>::move_a_step(std::array<double, dimension> &start,
                                                                 std::array<double, dimension> &direction,
                                                                 double step_size) {
    std::array<double, dimension> diff = vector_diff(direction, start);
    double distance = vector_norm(diff);
    if (distance <= step_size) {
        return direction;
    }

    diff = vector_mult(1.0 / distance, diff); // normalizing to 1.0 norm
    return vector_add(start, vector_mult(step_size, diff));
}

template <int dimension>
void RRT_solver<dimension>::construct_result_plan(std::list<std::array<double, dimension>> &result_plan,
                                                  Graph<dimension>::Vertex *goal_vertex) {
    auto current = goal_vertex;
    while (current != nullptr) {
        result_plan.push_front(current->coords);
        current = current->parent;
    }
}

template <int dimension>
void RRT_solver<dimension>::construct_result_plan(std::list<std::array<double, dimension>> &result_plan,
                                                  Graph<dimension>::Vertex *goal_vertex, double delta) {
    auto current = goal_vertex;
    while (current->parent != nullptr) {
        std::vector<std::array<double, dimension>> new_states;
        get_free_states(new_states, current->coords, current->parent->coords, delta);
        for (const auto &state : new_states) {
            result_plan.push_front(state);
        }
        current = current->parent;
    }

    result_plan.push_front(current->coords);
}
