#ifndef BARNES_HUT_NODE_H
#define BARNES_HUT_NODE_H

#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include <array>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <ostream>
#include <variant>

#include "body.h"

using Eigen::AlignedBox2f;
using Eigen::Vector2f;

namespace bh {

class Node;

using Empty = std::monostate;
using Subquadrants = std::array<std::unique_ptr<Node>, 4>;
using Data = std::variant<Empty, Body, Subquadrants>;

typedef enum { NW, NE, SE, SW } Subquadrant;

class Node {
  inline static unsigned n_nodes = 0;
  const unsigned m_id = n_nodes++;

  Data m_data;

  Vector2f m_center_of_mass = {0, 0};
  float m_total_mass = 0;

  void update_center_of_mass();

  friend void to_json(json &j, const Node &node);

 public:
  const Eigen::AlignedBox2f m_box;
  [[nodiscard]] inline Vector2f top_left() const;
  [[nodiscard]] inline Vector2f top_right() const;
  [[nodiscard]] inline Vector2f bottom_right() const;
  [[nodiscard]] inline Vector2f bottom_left() const;
  [[nodiscard]] inline float length() const;

  [[nodiscard]] const Vector2f &center_of_mass() const;
  [[nodiscard]] const float &total_mass() const;
  [[nodiscard]] const Data &data() const;

  /**
   * Creates an empty subquadrant.
   * @param bottom_left coordinates of the bottom-left corner of the subquadrant
   * @param top_right coordinates of the top-right corner of the subquadrant
   */
  Node(const Vector2f &bottom_left, const Vector2f &top_right);

  void insert(const Body &new_body);

  Subquadrant get_subquadrant(const Vector2f &point);

  friend std::ostream &operator<<(std::ostream &os, const Node &node);
};

Subquadrant get_subquadrant(const Vector2f &top_left, const float &length,
                            const Vector2f &position);

// void to_json(json &j, const Node &node);
}  // namespace bh

#endif  // BARNES_HUT_NODE_H
