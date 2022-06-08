#include "node.h"

namespace bh {

using Empty = std::monostate;

void to_json(json &j, const Empty &empty) { j = "empty"; }

void to_json(json &j, const Body &body) {
  j = {{"position", {body.m_position.x(), body.m_position.y()}},
       {"mass", body.m_mass}};
}

void to_json(json &j, const Node::Fork &fork) {
  j = {{"nw", *fork.m_children[Node::NW]},
       {"ne", *fork.m_children[Node::NE]},
       {"se", *fork.m_children[Node::SE]},
       {"sw", *fork.m_children[Node::SW]}};
}

void to_json(  // NOLINT(misc-no-recursion)
    json &j, const std::variant<Node::Fork, Node::Leaf> &data) {
  // Fixme
  if (std::holds_alternative<Node::Fork>(data)) {
    to_json(j, std::get<Node::Fork>(data));
  } else if (std::holds_alternative<Node::Leaf>(data)) {
    to_json(j["body"], std::get<Node::Leaf>(data));
  }
}

void to_json(json &j, const Node &node) {
  json data = node.m_data;
  j = {{"top_left", {node.top_left().x(), node.top_left().y()}},
       {"length", node.length()},
       {"center_of_mass",
        {node.center_of_mass().x(), node.center_of_mass().y()}},
       {"total_mass", node.total_mass()},
       {"n_nodes", node.n_nodes()},
       {"data", data}};
}

}  // namespace bh
