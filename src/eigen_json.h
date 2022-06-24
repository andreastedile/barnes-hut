#ifndef BARNES_HUT_EIGEN_JSON_H
#define BARNES_HUT_EIGEN_JSON_H

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <nlohmann/json.hpp>

using Eigen::AlignedBox2d;
using Eigen::Vector2d;
using nlohmann::adl_serializer;
using nlohmann::json;

namespace nlohmann {

template <>
struct adl_serializer<Vector2d> {
  static void to_json(json &j, const Vector2d &v) {
    j = json{{"x", v.x()}, {"y", v.y()}};
  }
};

template <>
struct adl_serializer<AlignedBox2d> {
  static void to_json(json &j, const AlignedBox2d &box) {
    j = json{{"bottomLeft", box.min()}, {"topRight", box.max()}};
  }
};

}  // namespace nlohmann

#endif  // BARNES_HUT_EIGEN_JSON_H
