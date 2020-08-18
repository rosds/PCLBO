#ifndef PC_LBO_VORONOI_DIAGRAM_HH
#define PC_LBO_VORONOI_DIAGRAM_HH

#include <Eigen/Core>
#include <vector>

namespace pclbo {

/** \brief Compute the area of the Voronoi Cell corresponding to the first
 * point.
 *
 *  This function creates a Voronoi diagram from the given points using the
 *  CGAL library. Afterwards, it computes the area of the Voronoi Cell
 *  that contains the first point in the array when possible.
 *
 *  \param[in] points Array of 2D points representing the center of the
 * Voronoi cells. \return The area of the Voronoi Cell containing the first
 * point of the array.
 */
double computeVoronoiCenterCellArea(const std::vector<Eigen::Vector2d>& points);

}  // namespace pclbo

#endif  // PC_LBO_VORONOI_DIAGRAM_HH
