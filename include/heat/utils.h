#ifndef HEAT_DIFF_UTILS_HH
#define HEAT_DIFF_UTILS_HH

namespace heat {

template <class PointT>
double avg_distance(
    const int neighbors,
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const typename pcl::KdTreeFLANN<PointT>::Ptr& tree) {

    double avg = 0.0;
    for (const auto& point : cloud->points) {
        std::vector<int> indices;
        std::vector<float> distances;
        tree->nearestKSearch(point, neighbors, indices, distances);

        avg += std::accumulate(distances.begin(), distances.end(), 0.0) / static_cast<double>(distances.size());
    }

    return avg / static_cast<double>(cloud->size());
}

} // namespace heat

#endif // HEAT_DIFF_UTILS_HH
