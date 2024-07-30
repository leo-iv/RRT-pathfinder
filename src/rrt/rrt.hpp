#pragma once

#include "graph.hpp"

class RRT_solver {
  private:
    int dimension;

  public:
    void build_tree(int k);
};
