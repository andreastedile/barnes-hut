#include "node.h"

namespace bh {

void to_json(json &j, const Empty &empty) { j = "empty"; }

void to_json(json &j, const Body &body) {
  j = {{"position", {body.m_position.x(), body.m_position.y()}},
       {"mass", body.m_mass}};
}

void to_json(json &j, const Subquadrants &subquadrants) {
  j = {{"nw", *subquadrants[0]},
       {"ne", *subquadrants[1]},
       {"se", *subquadrants[2]},
       {"sw", *subquadrants[3]}};
}

void to_json(json &j, const Data &data) {
  // Fixme
  if (std::holds_alternative<Empty>(data)) {
    to_json(j, std::get<Empty>(data));
  } else if (std::holds_alternative<Body>(data)) {
    to_json(j["body"], std::get<Body>(data));
  } else if (std::holds_alternative<Subquadrants>(data)) {
    to_json(j["subquadrants"], std::get<Subquadrants>(data));
  }
}

void to_json(json &j, const Node &node) {
  json data = node.m_data;
  j = {{"id", node.m_id},
       {"top_left", {node.top_left().x(), node.top_left().y()}},
       {"length", node.length()},
       {"center_of_mass",
        {node.m_center_of_mass.x(), node.m_center_of_mass.y()}},
       {"total_mass", node.m_total_mass},
       {"data", data}};
}

}  // namespace bh
