#ifndef BARNES_HUT_NODE_H
#define BARNES_HUT_NODE_H

#include <array>  // Subquadrants
#include <eigen3/Eigen/Geometry>
#include <memory>  // unique_ptr
#include <nlohmann/json.hpp>
#include <optional>  // Leaf's body
#include <variant>   // Node's m_data
#include <vector>

#include "body.h"

using Eigen::AlignedBox2d;
using Eigen::Vector2d;
using json = nlohmann::json;

namespace bh {

class Node;

enum Subquadrant { NW, NE, SE, SW };

class Node {
 public:
  /**
   * A fork in the quadtree is a node with four children.
   */
  struct Fork {
    using AggregateBody = Body;
    std::array<std::unique_ptr<Node>, 4> m_children;
    AggregateBody m_aggregate_body;
    explicit Fork(std::array<std::unique_ptr<Node>, 4> children);
    void update_aggregate_body();
  };

  /**
   * A leaf in the tree is an empty node or a node containing exactly one body.
   */
  struct Leaf {
    std::optional<Body> m_body;
    /**
     * Constructs an empty subquadrant.
     */
    Leaf() = default;
    /**
     * Constructs a subquadrant containing a single body.
     * @param body
     */
    explicit Leaf(const Body &body);
  };

  /**
   * @return The top left corner of the region.
   */
  [[nodiscard]] inline Vector2d top_left() const {
    return m_box.corner(m_box.TopLeft);
  };
  /**
   * @return The top right corner of the region.
   */
  [[nodiscard]] inline Vector2d top_right() const {
    return m_box.corner(m_box.TopRight);
  };
  /**
   * @return The bottom right corner of the region.
   */
  [[nodiscard]] inline Vector2d bottom_right() const {
    return m_box.corner(m_box.BottomRight);
  };
  /**
   * @return The bottom left corner of the region.
   */
  [[nodiscard]] inline Vector2d bottom_left() const {
    return m_box.corner(m_box.BottomLeft);
  };
  /**
   * @return The length of the sides of the region.
   */
  [[nodiscard]] inline double length() const {
    return (top_right() - top_left()).norm();
  };

  /**
   * @return If the node is a fork in the tree, returns the center of mass of
   *the bodies contained in the subquadrant. If is a non-empty leaf, returns the
   *position of the single body it contains; otherwise, returns {0, 0}.
   **/
  [[nodiscard]] Vector2d center_of_mass() const;

  /**
   * @return If the node is a fork in the tree, returns the aggregate mass of
   * the bodies contained in the subquadrant, If it is a non-empty leaf, returns
   * the mass of the single body it contains; otherwise, returns 0.
   */
  [[nodiscard]] double total_mass() const;

  /**
   * @return If the node is a fork in the tree, returns the aggregate number of
   * nodes contained in the subquadrant, which is 4 or greater. If it is a leaf,
   * returns 1.
   */
  [[nodiscard]] unsigned n_nodes() const;

  [[nodiscard]] const std::variant<Fork, Leaf> &data() const;

  /**
   * Creates a node representing an empty subquadrant.
   * @param bottom_left coordinates of the bottom-left corner of the subquadrant
   * @param top_right coordinates of the top-right corner of the subquadrant
   */
  Node(const Vector2d &bottom_left, const Vector2d &top_right);

  void insert(const Body &new_body);

  /**
   * @param point Coordinates of the point inside the region.
   * @return Subquadrant in which the point is located.
   */
  Subquadrant get_subquadrant(const Vector2d &point);

 private:
  unsigned m_n_nodes;

  AlignedBox2d m_box;

  std::variant<Node::Fork, Node::Leaf> m_data;

  // See "Arbitrary types conversions" in https://github.com/nlohmann/json
  friend void to_json(json &j, const Node &node);
};

}  // namespace bh

#endif  // BARNES_HUT_NODE_H
