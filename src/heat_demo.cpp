#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <algorithm>
#include <iostream>
#include <thread>

// Laplace-Beltrami Operator
#include <pclbo/pclbo.h>
#include <pclbo/utils.h>

// HeatKernelSignature
#include <heat/heat.h>

float shortRainbowColorMap(const float value, const float min,
                           const float max) {
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
      g = 255 - Y;
      b = 255;
      break;
    case 4:
      r = 0;
      g = 0;
      b = 255;
      break;
  }

  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  return *reinterpret_cast<float *>(&rgb);
}

int main(int argc, char *argv[]) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  pcl::io::loadPCDFile<pcl::PointXYZ>("../models/bunny.pcd", *cloud);

  // Create the KdTree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdt(
      new pcl::search::KdTree<pcl::PointXYZ>());
  kdt->setInputCloud(cloud);

  //-------------------------------------------------------------------------
  // Compute the normals and concatenate them to the points
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(kdt);
  ne.setKSearch(10);
  ne.compute(*normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointNormal>());
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

  //-------------------------------------------------------------------------
  // Compute the LBO
  pclbo::LBOEstimation<pcl::PointNormal, pcl::PointNormal>::Ptr lbo(
      new pclbo::LBOEstimation<pcl::PointNormal, pcl::PointNormal>());

  lbo->setInputCloud(cloud_with_normals);
  lbo->setCloudNormals(cloud_with_normals);
  lbo->compute();

  //-------------------------------------------------------------------------
  // Set the HeatKernelSignature
  heat::HeatKernelSignature<pcl::PointNormal>::Ptr hks(
      new heat::HeatKernelSignature<pcl::PointNormal>());
  hks->setInputCloud(cloud_with_normals);
  hks->setEigenValues(lbo->eigenvalues);
  hks->setEigenFunctions(lbo->eigenfunctions);

  //-------------------------------------------------------------------------
  // Compute and visualize the heat diffusion from vertex 1591
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  const int x = 1591;             // Some vertex id
  const double t_step = 3.53917;  // time step

  for (double t = t_step; true; t += t_step) {
    auto hs = hks->compute(x, t);

    auto range = std::minmax_element(hs->begin(), hs->end());

    // Display the heat
    colored_cloud->clear();
    for (int i = 0; i < cloud->size(); i++) {
      const auto &point = cloud->at(i);

      if (pcl::isFinite(point)) {
        pcl::PointXYZRGB p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.rgb =
            shortRainbowColorMap(hs->at(i), *(range.first), *(range.second));

        colored_cloud->push_back(p);
      }
    }

    viewer->removePointCloud("Scene");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
        mass_color(colored_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, mass_color, "Scene");

    viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }

  return 0;
}
