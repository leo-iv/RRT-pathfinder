#pragma once

#include "rrt.hpp"
#include <array>
#include <array_math.hpp>

template <int dimension>
RRT_solver<dimension>::RRT_solver(const std::array<std::array<double, 2>, dimension> &boundaries,
                                  Collision_detector *detector)
    : boundaries(boundaries), detector(detector), gen(rd()){};

template <int dimension>
Graph<dimension> &RRT_solver<dimension>::build_tree(std::list<std::array<double, dimension>> &result_plan,
                                                    std::array<double, dimension> &start_state,
                                                    std::array<double, dimension> &goal_state, int iters,
                                                    double delta) {
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

        if (((i % GOAL_INSERTION_ITER) == 0) && path_is_collision_free) { // adding gola state and path to it is free
            quit = true; // goal state reached -> quitting in the next iter
        }

        // adding new states to the graph:
        for (auto &new_state : new_states) {
            auto new_vertex = graph.add_vertex(new_state, nearest_vertex);
            graph.add_edge(nearest_vertex, new_vertex, vector_distance(nearest_vertex->coords, new_vertex->coords));
            nearest_vertex = new_vertex;
        }
    }

    if (quit) { // found path to goal
        construct_result_plan(result_plan, graph.get_last());
    } else {
        result_plan.clear();
    }
    return graph;
};

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
void RRT_solver<dimension>::construct_result_plan(std::list<std::array<double, dimension>> &result_plan,
                                                  Graph<dimension>::Vertex *goal_vertex) {
    auto current = goal_vertex;
    while (current != nullptr) {
        result_plan.push_front(current->coords);
        current = current->parent;
    }
}
