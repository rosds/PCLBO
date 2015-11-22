//#include <cmath>
//#include <map>
#include <iostream>
#include <algorithm>
//#include <fstream>

#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/CholmodSupport>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/gp3.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <pclbo/meshlbo.h>
#include <pclbo/geodesics/meshgeoheat.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;


/** \brief Computes the angle between two vectors
 *  \param[in] a A vector.
 *  \param[in] b B vector.
 *  \return The angle between the vectors.
 */
double angleBetween(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    Eigen::Vector3f c = a.cross(b);
    return atan2f(c.norm(), a.dot(b));
}

/** \brief Cotangent.
 *  For some reason C++ math doesn't bring a cotangent function?
 *  \param[in] angle Angle.
 *  \return The cotangent of the angle.
 */
double cot(const double angle) {
    return 1.0 / tan(angle);
}

/** \brief Color code of a value with a short-raibow pallete
 *  \param[in] value The value to encode
 *  \param[in] min Minimum value in rangle which will be blue.
 *  \param[in] max Maximum value in rage which will be red.
 *  \return The RGB code inside a double
 */
float shortRainbowColorMap(const double value, const double min, const double max) {
    uint8_t r, g, b;

    if (isnan(value)) {
        r = 255;
        g = 0;
        b = 0;
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        return *reinterpret_cast<float*>(&rgb);
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

/*
 *struct callback_args {
 *    size_t N;
 *    Eigen::SparseMatrix<double> L;
 *    Eigen::SparseMatrix<double> A;
 *    std::shared_ptr<Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double> > > solver;
 *    pcl::PolygonMesh::Ptr triangles;
 *    pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
 *    std::shared_ptr<std::vector<double> > tri_area;
 *    pcl::visualization::PCLVisualizer::Ptr viewer;
 *};
 *
 *void geo_callback (const pcl::visualization::PointPickingEvent& event, void* args) {
 *    struct callback_args* data = (struct callback_args *)args;
 *
 *    if (event.getPointIndex () == -1)
 *        return;
 *
 *    pcl::PointXYZ current_point;
 *    event.getPoint(current_point.x, current_point.y, current_point.z);
 *
 *    pcl::PointCloud<pcl::PointXYZ>::Ptr cl(new pcl::PointCloud<pcl::PointXYZ>());
 *    cl->push_back(current_point);
 *
 *    int point_index = event.getPointIndex();
 *
 *    // Draw clicked points in red:
 *    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (cl, 255, 0, 0);
 *    data->viewer->removePointCloud("clicked_points");
 *    data->viewer->addPointCloud(cl, red, "clicked_points");
 *    data->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
 *
 *
 *    // Do the whole thing again
 *    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(data->N);
 *    u0(point_index) = 1.0f;
 *
 *    // u = A \ u0
 *    data->solver->compute(data->A);
 *    Eigen::VectorXd u = data->solver->solve(u0);
 *    Eigen::VectorXd div = Eigen::VectorXd::Zero(data->N);
 *
 *    // Compute the vector field X
 *    for (int f = 0; f < data->triangles->polygons.size(); f++) {
 *         
 *        const auto& triangle = data->triangles->polygons.at(f);
 *
 *        const auto i = triangle.vertices[0];
 *        const auto j = triangle.vertices[1];
 *        const auto k = triangle.vertices[2];
 *
 *        // Get the vertices of the triangle
 *        const auto& p0 = data->cloud->at(i).getVector3fMap();        
 *        const auto& p1 = data->cloud->at(j).getVector3fMap();        
 *        const auto& p2 = data->cloud->at(k).getVector3fMap();        
 *
 *        // Compute the face normal
 *        const auto& n = (p1 - p0).cross(p2 - p0).normalized();
 *
 *        const auto& eij = n.cross(p1 - p0);
 *        const auto& ejk = n.cross(p2 - p1);
 *        const auto& eki = n.cross(p0 - p2);
 *
 *        const auto& ui = u(i);
 *        const auto& uj = u(j);
 *        const auto& uk = u(k);
 *
 *        Eigen::Vector3f x = (ui * ejk + uj * eki + uk * eij) / data->tri_area->at(f);
 *        x = - x / x.norm();
 *
 *        div(i) += x.dot(ejk);
 *        div(j) += x.dot(eki);
 *        div(k) += x.dot(eij);
 *    }
 *
 *    data->solver->compute(data->L);
 *    Eigen::VectorXd phi = data->solver->solve(div);
 *
 *    double minimum = 9999.0;
 *    double maximum = -9999.0;
 *    for (int i = 0; i < data->N; i++) {
 *        if (phi(i) < minimum) {
 *            minimum = phi(i);
 *        }
 *    }
 *    
 *    phi = phi.array() - minimum;
 *
 *    for (int i = 0; i < data->N; i++) {
 *        if (phi(i) > maximum) {
 *            maximum = phi(i);
 *        }
 *    }
 *
 *    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
 *    color_cloud->points.resize(data->N);
 *    
 *    for (int i = 0; i < data->cloud->size(); i++) {
 *        const auto& point = data->cloud->points.at(i);
 *        auto& point_copy = color_cloud->points.at(i);
 *        point_copy.x = point.x;
 *        point_copy.y = point.y;
 *        point_copy.z = point.z;
 *
 *        //double color = shortRainbowColorMap(u(i), 0.0, 1.0);
 *        float color = shortRainbowColorMap(phi(i), 0.0, maximum);
 *
 *        point_copy.rgb = color;
 *    }
 *
 *    // Display normals
 *    //viewer->addPointCloudNormals<pcl::PointNormal, pcl::PointNormal>(cloud_with_normals, cloud_with_normals, 1, 0.05, "normals");
 *    
 *    pcl::PCLPointCloud2::Ptr mierdita(new pcl::PCLPointCloud2());
 *    pcl::toPCLPointCloud2(*color_cloud, *mierdita);
 *    data->triangles->cloud = *mierdita;
 *
 *    data->viewer->removePolygonMesh("mesh");
 *    data->viewer->addPolygonMesh(*(data->triangles), "mesh");
 *}
 */
    
int main(int argc, char *argv[]) {

    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Parse arguments
    po::options_description desc;
    desc.add_options()
        ("help", "Show help message")
        ("show", "Show the tessellation")
        ("model,m", po::value<std::string>()->required(), "Model cloud");
    po::variables_map vm;

    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (po::required_option& e) {
        std::cout << desc;
        return 1;
    }

    fs::path input_file = vm["model"].as <std::string>();
    if (!fs::exists(input_file)) {
        std::cerr << "The input model file does not exist" << std::endl;
        return 1;
    }

    std::cout << "Reading input file..." << std::flush;
    std::cout.flush();
    pcl::io::loadPolygonFile(input_file.string(), *triangles);
    std::cout << "done" << std::endl;

    pcl::fromPCLPointCloud2<pcl::PointXYZ>(triangles->cloud, *cloud);

    const size_t N = cloud->size();

    pclbo::MeshGeoHeat mgh;
    mgh.setInputMesh(triangles);
    mgh.compute();
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    color_cloud->points.resize(N);

    int x = 500;
    std::vector<double> distances = mgh.getDistancesFrom(x);

    double maximum = *std::max_element(distances.begin(), distances.end());
    
    for (int i = 0; i < cloud->size(); i++) {
      const auto& point = cloud->points.at(i);
      auto& point_copy = color_cloud->points.at(i);
      point_copy.x = point.x;
      point_copy.y = point.y;
      point_copy.z = point.z;

      float color = shortRainbowColorMap(distances[i], 0.0, maximum);

      point_copy.rgb = color;
    }

    
    pcl::PCLPointCloud2::Ptr mierdita(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*color_cloud, *mierdita);
    triangles->cloud = *mierdita;

    //callback_args cb_args;
    //cb_args.N = N;
    //cb_args.L = L;
    //cb_args.A = A;
    //cb_args.cloud = cloud_with_normals;
    //cb_args.triangles = triangles;
    //cb_args.solver = solver;
    //cb_args.tri_area = tri_area;
    //cb_args.viewer = viewer;

    //viewer->registerPointPickingCallback(geo_callback, (void*)&cb_args);
    viewer->addPolygonMesh(*triangles, "mesh");

    ////pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(color_cloud);
    ////viewer->addPointCloud<pcl::PointXYZRGB> (color_cloud, rgb, "sample cloud");

    while (!viewer->wasStopped()) {
        viewer->spinOnce (100);
    }

    return 0;
}
