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
    // Compute and visualize the heat diffusion from vertex 591
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdt2(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    kdt2->setInputCloud(cloud);
    double avg_dist = pclbo::avg_distance<pcl::PointXYZ>(10, cloud, kdt2);
    double t = 100000 * avg_dist * avg_dist;

    const auto x = 1591;
    std::vector<double> heat(cloud->size(), 0.0);

    for (double t_step = t; true; t_step += t) {
        for (int y = 0; y < cloud->size(); y++) {

            double sum = 0.0;
            for (int j = 0; j < 200; j++) {
                double lambda = lbo.eigenvalues(j);
                Eigen::VectorXd psi = lbo.eigenfunctions.col(j);
                sum += exp(-lambda * t_step) * psi(x) * psi(y); 
            }

            heat[y] = sum;
        }

        std::cout << "t_step " << t_step << std::endl;

        auto range = std::minmax_element(heat.begin(), heat.end());

        // Display the heat
        colored_cloud->clear();
        for (int i = 0; i < cloud->size(); i++) {
            const auto& point = cloud->at(i);

            if (pcl::isFinite(point)) {
                pcl::PointXYZRGB p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                p.rgb = shortRainbowColorMap(heat[i], *(range.first), *(range.second));

                colored_cloud->push_back(p);
            }
        }

        viewer->removePointCloud("Scene");

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> mass_color(colored_cloud);
        viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, mass_color, "Scene");

        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}
