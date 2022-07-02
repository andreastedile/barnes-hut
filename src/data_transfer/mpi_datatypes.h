#ifndef BARNES_HUT_MPI_DATATYPES_H
#define BARNES_HUT_MPI_DATATYPES_H

#include <vector>

#include "node.h"

namespace bh::mpi {

typedef struct Body final {
  Body() = default;
  Body(double position_x, double position_y, double velocity_x, double velocity_y, double mass);
  double position_x;
  double position_y;
  double velocity_x;
  double velocity_y;
  double mass;
} Body;

struct Node final {
  enum Type {
    ForkType,
    LeafType,
  };

  typedef struct Fork {
    using AggregateBody = Body;

    Fork() = default;
    Fork(const int children[4], int n_nodes, const AggregateBody& aggregate_body);

    int children[4];
    int n_nodes;
    AggregateBody aggregate_body;
  } Fork;

  typedef struct Leaf {
    Leaf() = default;
    explicit Leaf(const Body& body);

    bool has_value;
    Body body;
  } Leaf;

  Node() = default;
  Node(const Fork& fork,
       double bottom_left_x, double bottom_left_y,
       double top_right_x, double top_right_y);
  Node(const Leaf& leaf,
       double bottom_left_x, double bottom_left_y,
       double top_right_x, double top_right_y);

  double bottom_left_x;
  double bottom_left_y;
  double top_right_x;
  double top_right_y;

  Type type;
  union Data {
    Data() = default;
    explicit Data(const Fork& fork);
    explicit Data(const Leaf& leaf);

    Fork fork;
    Leaf leaf;
  } data;
};

}  // namespace bh::mpi

#endif  // BARNES_HUT_MPI_DATATYPES_H
