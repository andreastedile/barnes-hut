#ifndef BARNES_HUT_BOUNDING_BOX_H
#define BARNES_HUT_BOUNDING_BOX_H

#include <eigen3/Eigen/Geometry>
#include <vector>

#include "body.h"

using Eigen::AlignedBox2d;

namespace bh {

/**
 * Computes the axis-aligned minimum bounding box containing some bodies.
 * @details The minimum bounding box is defined by:
 * <ul>
 * <li> Its bottom-left corner, whose x and y coordinates are minimal for any of
 * the bodies
 * <li> Its top-right corner, whose x and y coordinates are maximal for any of
 * the bodies
 * </ul>
 * @param bodies for which to compute the minimum bounding box; must be at least
 * two
 * @return the minimum bounding box; x() and y() return its bottom-left and
 * top-right corners
 * @throw invalid_argument if the bodies vector contains less than two bodies
 * @example <a href="https://www.desmos.com/calculator/mintua3fvc?lang=it">on
 * Desmos</a>
 */
AlignedBox2d compute_minimum_bounding_box(const std::vector<Body> &bodies);

/**
 * Computes an axis-aligned square bounding box containing some bodies.
 * @details The bounding box is defined by its bottom-left and top-right
 * corners. The computed bounding box is not minimum, but is
 *
 * @param bodies for which to compute the minimum bounding box; must be at least
 * two
 * @return the minimum bounding box; x() and y() return its bottom-left and
 * top-right corners
 * @throw invalid_argument if the bodies vector contains less than two bodies
 */
AlignedBox2d compute_square_bounding_box(const std::vector<Body> &bodies);

}  // namespace bh

#endif  // BARNES_HUT_BOUNDING_BOX_H
