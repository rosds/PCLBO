#ifndef PC_LBO_UTILS_HH
#define PC_LBO_UTILS_HH

#include <pcl/common/geometry.h>

namespace pclbo {

/** \brief Compute the average distance between each point and its neighbors.
 *  \param[in] neighbors The number of neighbors to consider in the calculation 
 *  of the distances.
 *  \param[in] cloud Pointer to the cloud with the points.
 *  \param[in] tree KdTree used to compute the radius search.
 *  \return The average distance between the points and its neighbors.
 */
template <class PointT>
double avg_distance(
    const int neighbors,
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const typename pcl::KdTreeFLANN<PointT>::Ptr& tree) {

    int n = 0;
    double avg = 0.0;
    for (int i = 0; i < cloud->size(); i++) {
        const auto& point = cloud->at(i);

        if (!pcl::isFinite(point)) continue;

        std::vector<int> indices;
        std::vector<float> distances;
        tree->nearestKSearch(point, neighbors, indices, distances);

        int m = 0;
        double avg_dist = 0.0;
        for (int j = 0; j < indices.size(); j++) {
            if (i != indices[j]) {
                const auto& neighbor = cloud->at(indices[j]);
                avg_dist += (neighbor.getVector3fMap() - point.getVector3fMap()).norm();
                m++;
            }
        }

        if (m > 0) {
            avg_dist /= static_cast<double>(m);
        }

        avg += avg_dist;
        n++;
    }

    if (n > 0) {
        avg /= static_cast<double>(n);
    }

    return avg;
}


/** \brief Projects the given points to a plane.
 *
 *  \param[in] origin Point in the plane.
 *  \param[in] normal Normal of the plane.
 *  \param[in] point The point to project to the plane.
 *  \return A 3D vector corresponding to the position of the point projected 
 *  onto the plane.
 */
template <class PointT, class NormalT>
Eigen::Vector3d 
project(const PointT& origin, const NormalT& normal, const PointT& point) {
    // Bring the point to the origin
    Eigen::Vector3f p = point.getVector3fMap() - origin.getVector3fMap();
    Eigen::Vector3f n = normal.getNormalVector3fMap();

    n.normalize();
    const double projection = static_cast<double>(p.dot(n));

    return p.cast<double>() - projection * n.cast<double>();
}

/** \brief Computes the angle between two vectors
 *  \param[in] a A vector.
 *  \param[in] b B vector.
 *  \return The angle between the vectors.
 */
double angleBetween(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    Eigen::Vector3f c = a.cross(b);
    return atan2f(c.norm(), a.dot(b));
}

} // namespace pclbo

#endif // PC_LBO_UTILS_HH
