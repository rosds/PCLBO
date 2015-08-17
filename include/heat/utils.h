#ifndef HEAT_DIFF_UTILS_HH
#define HEAT_DIFF_UTILS_HH

#include <pcl/common/geometry.h>

namespace heat {

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

} // namespace heat

#endif // HEAT_DIFF_UTILS_HH
