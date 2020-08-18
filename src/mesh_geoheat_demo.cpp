#include <algorithm>

#include <Eigen/Geometry>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <pclbo/geodesics/meshgeoheat.h>
#include <pclbo/meshlbo.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

/** \brief Color code of a value with a short-raibow pallete
 *  \param[in] value The value to encode
 *  \param[in] min Minimum value in rangle which will be blue.
 *  \param[in] max Maximum value in rage which will be red.
 *  \return The RGB code inside a double
 */
float shortRainbowColorMap(const double value, const double min,
                           const double max) {
  uint8_t r, g, b;

  if (isnan(value)) {
    r = 255;
    g = 0;
    b = 0;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    return *reinterpret_cast<float *>(&rgb);
  }

  // Normalize value to [0, 1]
  double value_normalized = (value - min) / (max - min);

  double a = (1.0f - value_normalized) / 0.25f;
  int X = static_cast<int>(floor(a));
  int Y = static_cast<int>(floor(255.0f * (a - X)));

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

struct callback_args {
  pclbo::MeshGeoHeat::Ptr mgh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::visualization::PCLVisualizer::Ptr viewer;
  pcl::PolygonMesh::Ptr triangles;
};

void geo_callback(const pcl::visualization::PointPickingEvent &event,
                  void *args) {
  struct callback_args *data = (struct callback_args *)args;

  if (event.getPointIndex() == -1)
    return;

  pcl::PointXYZ current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cl(new pcl::PointCloud<pcl::PointXYZ>());
  cl->push_back(current_point);

  int point_index = event.getPointIndex();

  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cl, 255,
                                                                      0, 0);
  data->viewer->removePointCloud("clicked_points");
  data->viewer->addPointCloud(cl, red, "clicked_points");
  data->viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");

  // Compute the geodesics
  std::vector<double> distances = data->mgh->getDistancesFrom(point_index);

  // Display
  double maximum = -1 * std::numeric_limits<double>::infinity();
  for (const auto &v : distances) {
    if (!isnan(v) && v > maximum) {
      maximum = v;
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  color_cloud->points.resize(data->cloud->size());

  for (int i = 0; i < data->cloud->size(); i++) {
    const auto &point = data->cloud->points.at(i);
    auto &point_copy = color_cloud->points.at(i);
    point_copy.x = point.x;
    point_copy.y = point.y;
    point_copy.z = point.z;

    float color = shortRainbowColorMap(distances[i], 0.0, maximum);

    point_copy.rgb = color;
  }

  pcl::PCLPointCloud2::Ptr cloud_with_colors(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(*color_cloud, *cloud_with_colors);
  data->triangles->cloud = *cloud_with_colors;

  data->viewer->removePolygonMesh("mesh");
  data->viewer->addPolygonMesh(*(data->triangles), "mesh");
}

int main(int argc, char *argv[]) {

  pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  // Parse arguments
  po::options_description desc;
  desc.add_options()("help", "Show help message")(
      "downsample,d", po::value<double>(), "Downsample rate.")(
      "time-step,t", po::value<double>(), "Time step for heat diffusion.")(
      "model,m", po::value<std::string>()->required(), "Model cloud");
  po::variables_map vm;

  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (po::required_option &e) {
    std::cout << desc;
    return 1;
  }

  fs::path input_file = vm["model"].as<std::string>();
  if (!fs::exists(input_file)) {
    std::cerr << "The input model file does not exist" << std::endl;
    return 1;
  }

  std::cout << "Reading input file..." << std::flush;
  std::cout.flush();
  pcl::io::loadPolygonFile(input_file.string(), *triangles);
  std::cout << "done" << std::endl;

  // Downsample the cloud if necessary
  if (vm.count("downsample")) {
    pcl::PolygonMesh::Ptr output(new pcl::PolygonMesh());
    double rate = vm["downsample"].as<double>();
    pcl::MeshQuadricDecimationVTK mqd;
    mqd.setInputMesh(triangles);
    mqd.setTargetReductionFactor(rate);
    mqd.process(*output);
    triangles = output;
  }

  pcl::fromPCLPointCloud2<pcl::PointXYZ>(triangles->cloud, *cloud);

  // =======================================================================
  //  Compute the geodesics of the point with index 500
  // =======================================================================

  int x = 500;
  pclbo::MeshGeoHeat::Ptr mgh(new pclbo::MeshGeoHeat());

  if (vm.count("time-step")) {
    mgh->setTimeStep(vm["time-step"].as<double>());
  }

  mgh->setInputMesh(triangles);
  mgh->compute();
  std::vector<double> distances = mgh->getDistancesFrom(x);

  // =======================================================================
  //  This is just about displaying the result
  // =======================================================================

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  color_cloud->points.resize(cloud->size());

  double maximum = -1 * std::numeric_limits<double>::infinity();
  for (const auto &v : distances) {
    if (!isnan(v) && v > maximum) {
      maximum = v;
    }
  }

  for (int i = 0; i < cloud->size(); i++) {
    const auto &point = cloud->points.at(i);
    auto &point_copy = color_cloud->points.at(i);
    point_copy.x = point.x;
    point_copy.y = point.y;
    point_copy.z = point.z;

    float color = shortRainbowColorMap(distances[i], 0.0, maximum);

    point_copy.rgb = color;
  }

  pcl::PCLPointCloud2::Ptr cloud_with_color(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(*color_cloud, *cloud_with_color);
  triangles->cloud = *cloud_with_color;

  callback_args cb_args;
  cb_args.cloud = cloud;
  cb_args.mgh = mgh;
  cb_args.viewer = viewer;
  cb_args.triangles = triangles;

  viewer->registerPointPickingCallback(geo_callback, (void *)&cb_args);
  viewer->addPolygonMesh(*triangles, "mesh");

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }

  return 0;
}
