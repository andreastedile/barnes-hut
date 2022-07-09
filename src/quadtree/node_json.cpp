// Do not remove the #include below! It allows serializing Eigen datatypes.
#include "../eigen_json.h"
#include "node.h"

namespace bh {

void to_json(nlohmann::json &j, const Node::Fork &fork) {
  j = {{"nw", *fork.m_nw},
       {"ne", *fork.m_ne},
       {"se", *fork.m_se},
       {"sw", *fork.m_sw}};
}

void to_json(nlohmann::json &j, const Node::Leaf &leaf) {
  if (leaf.m_body.has_value()) {
    j = nlohmann::json{{"body", leaf.m_body.value()}};
  } else {
    j = nlohmann::json{{"body", nullptr}};
  }
}

void to_json(nlohmann::json &j, const std::variant<Node::Fork, Node::Leaf> &data) {
  if (std::holds_alternative<Node::Fork>(data)) {
    j = nlohmann::json{"fork", std::get<Node::Fork>(data)};
  } else if (std::holds_alternative<Node::Leaf>(data)) {
    j = nlohmann::json{"leaf", std::get<Node::Leaf>(data)};
  }
}

void to_json(nlohmann::json &j, const Node &node) {
  j = {{"boundingBox", node.bbox()},
       {"length", node.bbox().sizes().x()},
       {"centerOfMass", node.center_of_mass()},
       {"totalMass", node.total_mass()},
       {"nNodes", node.n_nodes()},
       node.data()};
}

}  // namespace bh
