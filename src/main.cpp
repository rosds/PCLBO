#include <algorithm>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <heat/utils.h>
#include <heat/heat_diffusion.h>

int main(int argc, char *argv[]) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::io::loadPCDFile<pcl::PointXYZ>("../models/scene.pcd", *cloud);

    // Create the KdTree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdt(new pcl::search::KdTree<pcl::PointXYZ>());
    kdt->setInputCloud(cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(kdt);
    ne.setKSearch(10);
    ne.compute(*normals);

    heat::HeatDiffusion<pcl::PointXYZ, pcl::Normal> hd;
    hd.setInputCloud(cloud);
    hd.setCloudNormals(normals);
    hd.computeLBO();

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->addPointCloud<pcl::PointXYZ>(cloud);

    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}
