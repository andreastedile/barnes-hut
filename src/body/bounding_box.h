#ifndef BARNES_HUT_BOUNDING_BOX_H
#define BARNES_HUT_BOUNDING_BOX_H

#include <eigen3/Eigen/Geometry>
#include <vector>

#include "body.h"

namespace bh {

/**
 * Computes the axis-aligned minimum bounding box containing some bodies.
 * @details The minimum bounding box is defined by:
 * <ul>
 * <li> Its bottom-left corner, whose x and y coordinates are minimal for any of the bodies
 * <li> Its top-right corner, whose x and y coordinates are maximal for any of the bodies
 * </ul>
 * In some edge cases, it is dimensionless:
 * <ul>
 * <li> If zero bodies are passed, the minimum bounding box is centered at the origin of the cartesian plane
 * <li> If a single body is passed, it is centered at the coordinates of that body
 * <li> If two or more bodies are passed, and their coordinates coincide, it is centered at those coordinates
 * </ul>
 *
 * @param bodies for which to compute the minimum bounding box; can be empty
 * @return the minimum bounding box; x() and y() return its bottom-left and top-right corners
 * @example <a href="https://www.desmos.com/calculator/mintua3fvc?lang=it">on Desmos</a>
 */
Eigen::AlignedBox2d compute_minimum_bounding_box(const std::vector<Body> &bodies);

/**
 * Computes an axis-aligned square bounding box containing some bodies.
 * @details The bounding box is defined by its bottom-left and top-right corners.
 * Todo
 * @param bodies for which to compute the minimum bounding box; can be empty
 * @return the square bounding box; x() and y() return its bottom-left and top-right corners
 */
Eigen::AlignedBox2d compute_square_bounding_box(const std::vector<Body> &bodies);

}  // namespace bh

#endif  // BARNES_HUT_BOUNDING_BOX_H
