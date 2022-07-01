#ifndef BARNES_HUT_NODE_H
#define BARNES_HUT_NODE_H

#include <array>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <memory>  // unique_ptr
#include <nlohmann/json.hpp>
#include <optional>  // Leaf's m_body
#include <variant>   // Node's m_data
#include <vector>

#include "body.h"

using Eigen::AlignedBox2d;
using Eigen::Vector2d;
using json = nlohmann::json;

namespace bh {

class Node {
 public:
  // Used to access the Fork's array of Node children in an index-agnostic way
  enum Subquadrant { NW,
                     NE,
                     SE,
                     SW,
                     OUTSIDE };

  /**
   * A fork in the quadtree is a node with four children.
   */
  struct Fork {
    using AggregateBody = Body;
    Fork(std::array<std::unique_ptr<Node>, 4> children, int n_nodes, AggregateBody aggregate_body);

    std::array<std::unique_ptr<Node>, 4> m_children;
    int m_n_nodes;
    AggregateBody m_aggregate_body;
  };

  /**
   * A leaf in the tree is an empty node or a node containing exactly one body.
   */
  struct Leaf {
    std::optional<Body> m_body;
    /**
     * Constructs an empty node.
     */
    Leaf() = default;
    /**
     * Constructs a node containing a single body.
     * @param body that the node will contain
     */
    explicit Leaf(const Body &body);
  };

  /**
   * Computes the center of mass of this quadtree node.
   * @details
   * <ul>
   * <li> Empty leaf: the center of mass is (0, 0);
   * <li> Leaf: the center of mass are the coordinates of the single body it
   * contains;
   * <li> Fork: it corresponds to the aggregate center of mass over all the
   * bodies it contains.
   * </ul>
   * @return position vector of the center of mass
   **/
  [[nodiscard]] Vector2d center_of_mass() const;

  /**
   * Computes the total mass of this quadtree node.
   * @details
   * <ul>
   * <li> Empty leaf: the total mass is 0;
   * <li> Leaf: the total mass is the mass of the single body it contains;
   * <li> Fork: it corresponds to the aggregate mass over all the bodies it
   * contains.
   * </ul>
   **/
  [[nodiscard]] double total_mass() const;

  /**
   * @return number of nodes contained in the quadtree rooted in this node
   */
  [[nodiscard]] int n_nodes() const;

  [[nodiscard]] const std::variant<Fork, Leaf> &data() const;

  /**
   * Creates an empty quadtree node, in which bodies can be inserted.
   * @param bottom_left coordinates of the bottom-left corner of the node's
   * bounding box
   * @param top_right coordinates of the top-right corner of the node's bounding
   * box
   */
  Node(const Vector2d &bottom_left, const Vector2d &top_right);

  Node(const Vector2d &bottom_left, const Vector2d &top_right, Fork fork) : m_box(bottom_left, top_right), m_data(std::move(fork)) {}

  Node(const Vector2d &bottom_left, const Vector2d &top_right, Leaf leaf) : m_box(bottom_left, top_right), m_data(std::move(leaf)) {}

  /**
   * Inserts a new body in the quadtree. The body must be located inside of the
   * node's bounding box.
   * @details Depending on the node's current type, the insertion of the new
   * body produces a different effect:
   * <ul>
   * <li> Empty leaf: the leaf will contain the new body;
   * <li> Leaf with a body:
   * <ul>
   * <li> If the new body coincides with the body in the leaf, the mass of the
   * body in the leaf increases by the mass of the new body;
   * <li> Otherwise, the node's type becomes Fork (i.e., a node with four
   * children); the body in the leaf is relocated in a child, and the new body
   * is recursively inserted in a child node.
   * </ul>
   * <li> Fork: the new body is recursively inserted in one of the node's
   * children.
   * </ul>
   * The choice of which child node to select when inserting a body depends on
   * the body's subquadrant. All the operations have the effect of changing the
   * node's center of mass and total mass.
   * @param new_body to insert
   * @throw invalid_argument if the body for which the insertion is attempted is
   * located outside of the node's bounding box
   */
  void insert(const Body &new_body);

  /**
   * Computes in which of the node's subquadrants a given point is located.
   * @param point Coordinates of the point
   * @return The subquadrant in which the point is located
   */
  Subquadrant get_subquadrant(const Vector2d &point);

  /**
   * The axis-aligned square bounding box of the node; not necessarily minimum.
   * x() and y() return its bottom-left and top-right corners.
   */
  [[nodiscard]] const AlignedBox2d &bbox() const;

 private:
  AlignedBox2d m_box;

  /**
   * Holds the current type of the node.
   */
  std::variant<Node::Fork, Node::Leaf> m_data;
};

void to_json(json &j, const Node &node);

Node::Fork::AggregateBody compute_aggregate_body(const Node &nw, const Node &ne, const Node &se, const Node &sw);

}  // namespace bh

#endif  // BARNES_HUT_NODE_H
