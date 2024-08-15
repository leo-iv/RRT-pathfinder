#pragma once

#include "graph.hpp"
#include <cmath>
#include <iostream>
#include <random>

/**
 * Interface class to provide the rrt algorithm with a collision detection method
 */
class Collision_detector {
  public:
    /**
     * Checks if the given state (robot configuration) is permissible (robot does not collide with walls etc)
     *
     * @return true if the state is permissible, false if the state is not allowed
     */
    virtual bool check_collision(double state[]) = 0;
};

/**
 * @brief Wrapper object for the family of rrt algorithms
 *
 * @tparam dimension - number of dimensions of the configuration space
 */
template <int dimension> class RRT_solver {
  private:
    // for every GOAL_INSERTION_ITER iteratio the RRT algorithm tries to insert the goal state into the tree
    static constexpr int GOAL_INSERTION_ITER = 15;

    Graph<dimension> graph;
    // lower and upper bounds for each of the coordinates in the configuration space - 2D array: [dimension][2]
    std::array<std::array<double, 2>, dimension> boundaries;

    Collision_detector *detector;

    // support for random number generation:
    std::random_device rd;
    std::mt19937 gen;

  public:
    /**
     * @param boundaries array with 'dimension' rows and 2 columns containing lower (first) and upper (second) bounds
     * for each of the coordinates in the configuration space
     * @param detector implementation of the Collision_detector interface
     */
    RRT_solver(const std::array<std::array<double, 2>, dimension> &boundaries, Collision_detector *detector);

    /**
     * Constructs the RRT tree and finds permissible plan
     * @param result_plan reference to std::list in which the result plan is stored
     * @param start_state starting position from which the RRT tree is built
     * @param goal_state goal position to which we need to find path to
     * @param iters maximal number of iterations of the RRT algorithm
     * @param delta distance between tree nodes for which the collision is checked (lower value is better, but runs
     * slower and needs more memory)
     */
    void solve_rrt(std::list<std::array<double, dimension>> &result_plan, std::array<double, dimension> &start_state,
                   std::array<double, dimension> &goal_state, int iters, double delta);

    /**
     * Finds permissible plan using the k-nearest RRT* algorithm
     *
     * @param result_plan reference to std::list in which the result plan is stored
     * @param start_state starting position from which the RRT tree is built
     * @param goal_state goal position to which we need to find path to
     * @param iters number of iterations of the algorithm
     * @param step step size of the algorithm - distance between new and nearest vertex
     * @param delta distance between vertices in the result plan, for which the collision is checked (lower value is
     * better, but runs slower and needs more memory)
     */
    void solve_k_rrts(std::list<std::array<double, dimension>> &result_plan, std::array<double, dimension> &start_state,
                      std::array<double, dimension> &goal_state, int iters, double step, double delta);

    /**
     * Returns the result tree
     *
     * @return reference to the tree constructed by the previouse "solve" call (reference to graph stored inside the
     * class - accessible only for the lifespan of the RRT_Solver class)
     */
    Graph<dimension> &get_tree();

  private:
    std::array<double, dimension> get_random_state();
    /** Finds all free configurations between start and stop by by moving an incremental distance delta. Returns true if
     * even the goal state is collision free (is also added to the new_states vector).*/
    bool get_free_states(std::vector<std::array<double, dimension>> &new_states, std::array<double, dimension> &start,
                         std::array<double, dimension> &stop, double delta);
    /** Checks if the straight path from start to stop is collision free by moving an incremental distance delta (works
     * in the same way as get_free_states but doesn't return any new states)  */
    bool is_collision_free(std::array<double, dimension> &start, std::array<double, dimension> &stop, double delta);
    /**  Returns state in the step_size distance from start in the given direction. If distance from start to direction
     * state is lower than step_size, then direction is returned.*/
    std::array<double, dimension> move_a_step(std::array<double, dimension> &start,
                                              std::array<double, dimension> &direction, double step_size);
    void construct_result_plan(std::list<std::array<double, dimension>> &result_plan,
                               Graph<dimension>::Vertex *goal_vertex);
    /** Constructs result plan, but also splits all edges so the disance between vertices is delta at maximum */
    void construct_result_plan(std::list<std::array<double, dimension>> &result_plan,
                               Graph<dimension>::Vertex *goal_vertex, double delta);
};

#include "rrt.tpp"
