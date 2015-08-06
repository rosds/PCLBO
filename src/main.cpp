#include <algorithm>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>


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

int main(int argc, char *argv[]) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("../models/scene.pcd", *cloud);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());

    std::cout << min_max_distance(cloud) << std::endl;

    viewer->addPointCloud<pcl::PointXYZ>(cloud);

    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}
