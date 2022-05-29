#ifndef BARNES_HUT_NODE_H
#define BARNES_HUT_NODE_H

#include <array>  // Subquadrants
#include <eigen3/Eigen/Geometry>
#include <memory>  // unique_ptr
#include <nlohmann/json.hpp>
#include <ostream>  // overload operator <<
#include <variant>  // Data

#include "body.h"

using Eigen::AlignedBox2f;
using Eigen::Vector2f;
using json = nlohmann::json;

namespace bh {

class Node;

using Empty = std::monostate;
using Subquadrants = std::array<std::unique_ptr<Node>, 4>;
using Data = std::variant<Empty, Body, Subquadrants>;

enum Subquadrant { NW, NE, SE, SW };

class Node {
  inline static unsigned n_nodes = 0;
  const unsigned m_id = n_nodes++;

  Data m_data;

  Vector2f m_center_of_mass{0, 0};
  float m_total_mass = 0;

  void update_center_of_mass();

  // See "Arbitrary types conversions" in https://github.com/nlohmann/json
  friend void to_json(json &j, const Node &node);

 public:
  const AlignedBox2f m_box;
  /**
   * @return The top left corner of the region.
   */
  [[nodiscard]] inline Vector2f top_left() const {
    return m_box.corner(m_box.TopLeft);
  };
  /**
   * @return The top right corner of the region.
   */
  [[nodiscard]] inline Vector2f top_right() const {
    return m_box.corner(m_box.TopRight);
  };
  /**
   * @return The bottom right corner of the region.
   */
  [[nodiscard]] inline Vector2f bottom_right() const {
    return m_box.corner(m_box.BottomRight);
  };
  /**
   * @return The bottom left corner of the region.
   */
  [[nodiscard]] inline Vector2f bottom_left() const {
    return m_box.corner(m_box.BottomLeft);
  };
  /**
   * @return The length of the sides of the region.
   */
  [[nodiscard]] inline float length() const {
    return (top_right() - top_left()).norm();
  };

  [[nodiscard]] const Vector2f &center_of_mass() const;
  [[nodiscard]] float total_mass() const;
  [[nodiscard]] const Data &data() const;

  /**
   * Creates an empty subquadrant.
   * @param bottom_left coordinates of the bottom-left corner of the subquadrant
   * @param top_right coordinates of the top-right corner of the subquadrant
   */
  Node(const Vector2f &bottom_left, const Vector2f &top_right);

  void insert(const Body &new_body);

  /**
   * @param point Coordinates of the point inside the region.
   * @return Subquadrant in which the point is located.
   */
  Subquadrant get_subquadrant(const Vector2f &point);

  friend std::ostream &operator<<(std::ostream &os, const Node &node);
};

// void to_json(json &j, const Node &node);
}  // namespace bh

#endif  // BARNES_HUT_NODE_H
