// Do not remove the #include below! It allows serializing Eigen datatypes.
#include "../eigen_json.h"
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
    j = json{{"body", leaf.m_body.value()}};
  } else {
    j = json{{"body", nullptr}};
  }
}

void to_json(json &j, const std::variant<Node::Fork, Node::Leaf> &data) {
  if (std::holds_alternative<Node::Fork>(data)) {
    j = json{"fork", std::get<Node::Fork>(data)};
  } else if (std::holds_alternative<Node::Leaf>(data)) {
    j = json{"leaf", std::get<Node::Leaf>(data)};
  }
}

void to_json(json &j, const Node &node) {
  j = {{"boundingBox", node.bbox()},
       {"length", node.bbox().sizes().x()},
       {"centerOfMass", node.center_of_mass()},
       {"totalMass", node.total_mass()},
       {"nNodes", node.n_nodes()},
       node.data()};
}

}  // namespace bh
