#include "node.h"

namespace bh {

void to_json(json &j, const Node::Fork &fork) {
  j = {{"nw", *fork.m_children[Node::NW]},
       {"ne", *fork.m_children[Node::NE]},
       {"se", *fork.m_children[Node::SE]},
       {"sw", *fork.m_children[Node::SW]}};
}

void to_json(json &j, const Node::Leaf &leaf) {
  if (leaf.m_body.has_value()) {
    // Example:
    // "body": {
    //   "mass": 0.25,
    //   "position": [1.5, 2.0]
    //  }
    j = {{"body",
          {{"position",
            {leaf.m_body->m_position.x(), leaf.m_body->m_position.y()}},
           {"mass", leaf.m_body->m_mass}}}};
  } else {
    // "body" : null
    j = {{"body", nullptr}};
  }
}

void to_json(json &j, const std::variant<Node::Fork, Node::Leaf> &data) {
  if (std::holds_alternative<Node::Fork>(data)) {
    j = {"fork", std::get<Node::Fork>(data)};
  } else if (std::holds_alternative<Node::Leaf>(data)) {
    j = {"leaf", std::get<Node::Leaf>(data)};
  }
}

void to_json(json &j, const Node &node) {
  j = {{"bounding_box",
        {{"bottom_left", {node.bbox().min().x(), node.bbox().min().y()}},
         {"top_right", {node.bbox().max().x(), node.bbox().max().y()}}}},
       {"length", node.bbox().sizes().x()},
       {"center_of_mass",
        {node.center_of_mass().x(), node.center_of_mass().y()}},
       {"total_mass", node.total_mass()},
       {"n_nodes", node.n_nodes()},
       node.m_data};
}

}  // namespace bh
