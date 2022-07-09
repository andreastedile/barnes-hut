#include "bounding_box.h"

#include <Eigen/Eigen>
#ifdef WITH_TBB
#include <execution>  // par_unseq
#include <numeric>    // transform_reduce
#else
// https://passlab.github.io/Examples/contents/Examples_udr.html
// https://www.openmp.org/spec-html/5.0/openmpsu107.html
// clang-format off
#pragma omp declare reduction(min : Eigen::Vector2d : \
                              omp_out.x() = omp_in.x() > omp_out.x() ? omp_out.x() : omp_in.x(), \
                              omp_out.y() = omp_in.y() > omp_out.y() ? omp_out.y() : omp_in.y()) \
        initializer(omp_priv = {std::numeric_limits <double>::max(), std::numeric_limits <double>::max()})

#pragma omp declare reduction(max : Eigen::Vector2d : \
                              omp_out.x() = omp_in.x() < omp_out.x() ? omp_out.x() : omp_in.x(), \
                              omp_out.y() = omp_in.y() < omp_out.y() ? omp_out.y() : omp_in.y()) \
        initializer(omp_priv = {std::numeric_limits <double>::lowest(), std::numeric_limits <double>::lowest()})
// clang-format on
#endif

using Eigen::Vector2d;

namespace bh {

Eigen::AlignedBox2d compute_minimum_bounding_box(const std::vector<Body> &bodies) {
  if (bodies.empty()) {
    return {Vector2d{0, 0}, Vector2d{0, 0}};
  } else if (bodies.size() == 1) {
    return {bodies[0].m_position, bodies[0].m_position};
  } else {
    // Compute the (typically rectangular) bounding box containing all bodies
#ifdef WITH_TBB
    auto bottom_left = std::transform_reduce(
        std::execution::par_unseq,
        bodies.begin(), bodies.end(),
        Vector2d(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
        [](const Vector2d &bottom_left, const Vector2d &body) -> Vector2d {
          return {std::min(bottom_left.x(), body.x()), std::min(bottom_left.y(), body.y())};
        },
        [](const Body &body) {
          return body.m_position;
        });
    auto top_right = std::transform_reduce(
        std::execution::par_unseq,
        bodies.begin(), bodies.end(),
        Vector2d(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()),
        [](const Vector2d &top_right, const Vector2d &body) -> Vector2d {
          return {std::max(top_right.x(), body.x()), std::max(top_right.y(), body.y())};
        },
        [](const Body &body) {
          return body.m_position;
        });
#else
    Eigen::Vector2d bottom_left{std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    Eigen::Vector2d top_right{std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};

// clang-format off
#pragma omp parallel for default(none) shared(bodies) reduction(min : bottom_left) reduction(max : top_right)
    for (const auto &body : bodies) {
      if (body.m_position.x() < bottom_left.x()) bottom_left.x() = body.m_position.x();
      if (body.m_position.y() < bottom_left.y()) bottom_left.y() = body.m_position.y();
      if (body.m_position.x() > top_right.x()) top_right.x() = body.m_position.x();
      if (body.m_position.y() > top_right.y()) top_right.y() = body.m_position.y();
    }
// clang-format off
#endif

    return {bottom_left, top_right};
  }
}

Eigen::AlignedBox2d compute_square_bounding_box(const std::vector<Body> &bodies) {
  Eigen::AlignedBox2d min_bbox = compute_minimum_bounding_box(bodies);

  if (double diff = min_bbox.sizes().x() > min_bbox.sizes().y(); diff > 0) {
    // width > height
    min_bbox.min() -= Vector2d(0, diff / 2);
    min_bbox.max() += Vector2d(0, diff / 2);
  } else if (diff = min_bbox.sizes().y() > min_bbox.sizes().x(); diff > 0) {
    // height > width
    min_bbox.min() -= Vector2d(diff / 2, 0);
    min_bbox.max() += Vector2d(diff / 2, 0);
  } else if (min_bbox.min() == min_bbox.max()) {
    // bounding box degenerated to a point
    min_bbox.min() -= Vector2d(0.5, 0.5);
    min_bbox.max() += Vector2d(0.5, 0.5);
  }

  // We now snap the square's corners to the grid, but this may produce a rectangle.
  min_bbox.min() = {std::floor(min_bbox.min().x()), std::floor(min_bbox.min().y())};
  min_bbox.max() = {std::ceil(min_bbox.max().x()), std::ceil(min_bbox.max().y())};

  // We now extend the rectangle into a square.
  if (double diff = min_bbox.sizes().x() - min_bbox.sizes().y(); diff > 0) {
    min_bbox.max() += Vector2d(0, diff);
  } else if (diff = min_bbox.sizes().y() - min_bbox.sizes().x(); diff > 0) {
    min_bbox.max() += Vector2d(diff, 0);
  }

  return min_bbox;
}

}  // namespace bh
