#include "mpi_datatypes.h"

namespace bh::mpi {

Body::Body(double position_x, double position_y, double velocity_x, double velocity_y, double mass)
    : position_x(position_x), position_y(position_y), velocity_x(velocity_x), velocity_y(velocity_y), mass(mass) {}

Node::Fork::Fork(int nw_idx, int ne_idx, int se_idx, int sw_idx, int n_nodes, const AggregateBody& aggregate_body)
    : nw_idx{nw_idx}, ne_idx{ne_idx}, se_idx{se_idx}, sw_idx{sw_idx},
      n_nodes(n_nodes),
      aggregate_body(aggregate_body) {}

Node::Leaf::Leaf(const Body& body) : has_value(true), body{body} {}

Node::Data::Data(const Fork& fork) : fork(fork) {}
Node::Data::Data(const Leaf& leaf) : leaf(leaf) {}

Node::Node(const Fork& fork, double bottom_left_x, double bottom_left_y, double top_right_x, double top_right_y)
    : type{ForkType}, data{fork}, bottom_left_x(bottom_left_x), bottom_left_y(bottom_left_y), top_right_x(top_right_x), top_right_y(top_right_y) {}

Node::Node(const Leaf& leaf, double bottom_left_x, double bottom_left_y, double top_right_x, double top_right_y)
    : type{LeafType}, data{leaf}, bottom_left_x(bottom_left_x), bottom_left_y(bottom_left_y), top_right_x(top_right_x), top_right_y(top_right_y) {}

}  // namespace bh::mpi
