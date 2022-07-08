#ifndef BARNES_HUT_EIGEN_JSON_H
#define BARNES_HUT_EIGEN_JSON_H

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

namespace nlohmann {

template <>
struct adl_serializer<Eigen::Vector2d> {
  static void to_json(nlohmann::json &j, const Eigen::Vector2d &v) {
    j = nlohmann::json{{"x", v.x()}, {"y", v.y()}};
  }
};

template <>
struct adl_serializer<Eigen::AlignedBox2d> {
  static void to_json(nlohmann::json &j, const Eigen::AlignedBox2d &box) {
    j = nlohmann::json{{"bottomLeft", box.min()}, {"topRight", box.max()}};
  }
};

}  // namespace nlohmann

#endif  // BARNES_HUT_EIGEN_JSON_H
