#include <algorithm>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/intensity_gradient.h>

// Laplace-Beltrami Operator
#include <pclbo/utils.h>
#include <pclbo/pclbo.h>

// HeatKernelSignature
#include <heat/heat.h>


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
    pclbo::LBOEstimation<pcl::PointNormal, pcl::PointNormal>::Ptr
    lbo(new pclbo::LBOEstimation<pcl::PointNormal, pcl::PointNormal>());

    lbo->setInputCloud(cloud_with_normals);
    lbo->setCloudNormals(cloud_with_normals);
    lbo->compute();

    //-------------------------------------------------------------------------
    // Set the HeatKernelSignature
    heat::HeatKernelSignature<pcl::PointNormal>::Ptr hks(new heat::HeatKernelSignature<pcl::PointNormal>());
    hks->setInputCloud(cloud_with_normals);
    hks->setEigenValues(lbo->eigenvalues);
    hks->setEigenFunctions(lbo->eigenfunctions);

    //-------------------------------------------------------------------------
    // Compute the heat signature for a specific time lapse and then the Vector 
    // Field X

    const int x = 1591;
    const double t_step = 3.53917;
    auto hs = hks->compute(x, t_step);

    // Create a intensity cloud to compute the gradient
    pcl::PointCloud<pcl::PointXYZI>::Ptr intensity(new pcl::PointCloud<pcl::PointXYZI>());
    intensity->resize(cloud_with_normals->size());
    for (int i = 0; i < cloud_with_normals->size(); i++) {
        const auto& point = cloud_with_normals->at(i);
        auto& u = intensity->at(i);
        u.x = point.x;
        u.y = point.y;
        u.z = point.z;
        u.intensity = hs->at(i);
    }

    pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::PointNormal, pcl::IntensityGradient>::Ptr \
    grad_estimation(new pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::PointNormal, pcl::IntensityGradient>());

    pcl::PointCloud<pcl::IntensityGradient>::Ptr X(new pcl::PointCloud<pcl::IntensityGradient>());

    grad_estimation->setInputCloud(intensity);
    grad_estimation->setInputNormals(cloud_with_normals);
    grad_estimation->setKSearch(10);
    grad_estimation->compute(*X);

    // Compute the norm of the gradient
    Eigen::VectorXd gradient_norm = Eigen::VectorXd::Zero(hs->size());
    
    // Invert and normalize the vector field
    for (int i = 0; i < X->size(); i++) {
        auto& point = X->at(i);

        Eigen::Vector3d g;
        g(0) = point.gradient[0];
        g(1) = point.gradient[1];
        g(2) = point.gradient[2];

        gradient_norm(i) = g.norm();

        g.normalize();

        point.gradient[0] = - g(0);
        point.gradient[1] = - g(1);
        point.gradient[2] = - g(2);

    }

    // Compute the Gradient Operator
    Eigen::Map<Eigen::VectorXd> u_heat(hs->data(), hs->size());

    Eigen::MatrixXd D = gradient_norm.cross(u_heat);



    //-------------------------------------------------------------------------
    // Compute and visualize the heat diffusion from vertex 1591
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());

    viewer->addPointCloud<pcl::PointNormal>(cloud_with_normals, "Scene");
    //viewer->addPointCloudIntensityGradients<pcl::PointNormal, pcl::IntensityGradient>(cloud_with_normals, X, 1, 0.00001, "gradient");

    while (!viewer->wasStopped()) {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}
