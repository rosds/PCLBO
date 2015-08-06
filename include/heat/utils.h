#ifndef HEAT_DIFF_UTILS_HH
#define HEAT_DIFF_UTILS_HH

namespace heat {

double min_max_distance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    double max = 0.0;

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    for (const auto& point : cloud->points) {
        std::vector<int> indices;
        std::vector<float> distances;
        tree->nearestKSearch(point, 3, indices, distances);

        auto max_elem = std::max_element(std::begin(distances), std::end(distances));

        if (max < *max_elem) {
            max = *max_elem;
        }
    }

    return max;
}

} // namespace heat

#endif // HEAT_DIFF_UTILS_HH
