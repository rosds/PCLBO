#include <algorithm>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pclbo/utils.h>
#include <pclbo/pclbo.h>


float shortRainbowColorMap(const float value, const float min, const float max) {
    uint8_t r, g, b;

    // Normalize value to [0, 1]
    float value_normalized = (value - min) / (max - min);

    float a = (1.0f - value_normalized) / 0.25f;
    int X = static_cast<int>(floorf(a));
    int Y = static_cast<int>(floorf(255.0f * (a - X)));

    switch (X) {
        case 0: 
            r = 255;
            g = Y;
            b = 0;
            break;
        case 1: 
            r = 255 - Y;
            g = 255;
            b = 0;
            break;
        case 2: 
            r = 0;
            g = 255;
            b = Y;
            break;
        case 3: 
            r = 0;
            g = 255-Y;
            b = 255;
            break;
        case 4: 
            r = 0;
            g = 0;
            b = 255;
            break;
    }

    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    return *reinterpret_cast<float*>(&rgb);
}

    
int main(int argc, char *argv[]) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::io::loadPCDFile<pcl::PointXYZ>("../models/bunny.pcd", *cloud);

    // Create the KdTree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdt(new pcl::search::KdTree<pcl::PointXYZ>());
    kdt->setInputCloud(cloud);

    //-------------------------------------------------------------------------
    // Compute the normals and concatenate them to the points
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(kdt);
    ne.setKSearch(10);
    ne.compute(*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    //-------------------------------------------------------------------------
    // Compute the LBO
    pclbo::LBOEstimation<pcl::PointNormal, pcl::PointNormal> lbo;
    lbo.setInputCloud(cloud_with_normals);
    lbo.setCloudNormals(cloud_with_normals);
    lbo.compute();

    //-------------------------------------------------------------------------
    // Visualize the Eigenfunctions of the LBO
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (int f = 0; f < lbo.eigenvalues.size(); f++) {

        Eigen::VectorXd v = lbo.eigenfunctions.col(f);

        // Search for the minimum and maximum curvature
        float min = v.minCoeff();
        float max = v.maxCoeff();

        colored_cloud->clear();
        for (int i = 0; i < cloud_with_normals->size(); i++) {
            const auto& point = cloud_with_normals->at(i);

            if (pcl::isFinite(point)) {
                pcl::PointXYZRGB p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                p.rgb = shortRainbowColorMap(v(i), min, max);

                colored_cloud->push_back(p);
            }
        }

        viewer->removePointCloud("Scene");

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> mass_color(colored_cloud);
        viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, mass_color, "Scene");

        while (!viewer->wasStopped ()) {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
        viewer->resetStoppedFlag();
    }

    return 0;
}
