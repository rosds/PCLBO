#include <heat/heat_diffusion.h>

template <class PointT, class NormalT>
void heat::HeatDiffusion<PointT, NormalT>::computeLBO() {
    typename pcl::KdTreeFLANN<PointT>::Ptr kdt(new pcl::KdTreeFLANN<PointT>());
    kdt->setInputCloud(_cloud);

    double avg_dist = heat::avg_distance<PointT>(10, _cloud, kdt);

    for (const auto& point : _cloud->points) {

        std::vector<int> indices;
        std::vector<float> distances;

        kdt->radiusSearch(point, 6 * avg_dist, indices, distances);

        if (indices.size() > 10) {
        
        }
    }
}
