#include <pclbo/geodesics/meshgeoheat.h>

#include <Eigen/CholmodSupport>
#include <Eigen/Sparse>

std::vector<double> pclbo::MeshGeoHeat::getDistancesFrom(const int x) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromPCLPointCloud2<pcl::PointXYZ>(input_mesh_->cloud, *cloud);

  // Number of vertices
  const size_t N = cloud->size();

  // Number of faces.
  const size_t F = input_mesh_->polygons.size();

  double dt;

  if (time_step < 0.0) {
    dt = sqrt(lbo.avg_edge_length);
  } else {
    dt = time_step;
  }

  // Compute the heat signature
  Eigen::SparseMatrix<double> A = lbo.M + dt * lbo.L;

  // Set the initial heat distribution
  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(N);
  u0(x) = 1.0f;

  // u = A \ u0
  std::shared_ptr<Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>>>
      solver(new Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>>());
  solver->setShift(10e-10);
  solver->compute(A);

  if (solver->info() != Eigen::Success) {
    std::cerr << "Something went wrong at the heat computation " << std::endl;
  }

  Eigen::VectorXd u = solver->solve(u0);

  Eigen::VectorXd div = Eigen::VectorXd::Zero(N);

  // Compute the vector field X
  for (int f = 0; f < F; f++) {
    const auto &face = input_mesh_->polygons.at(f);
    const auto i = face.vertices[0];
    const auto j = face.vertices[1];
    const auto k = face.vertices[2];
    const auto &pi = cloud->at(i).getVector3fMap();
    const auto &pj = cloud->at(j).getVector3fMap();
    const auto &pk = cloud->at(k).getVector3fMap();

    // Compute the face normal
    const auto &n = (pj - pi).cross(pk - pi).normalized();

    const auto &eij = n.cross(pj - pi);
    const auto &ejk = n.cross(pk - pj);
    const auto &eki = n.cross(pi - pk);

    const auto &ui = u(i);
    const auto &uj = u(j);
    const auto &uk = u(k);

    // Compute the gradient vector
    Eigen::Vector3f x = (ui * ejk + uj * eki + uk * eij) / lbo.area->at(f);

    x = -x / x.norm();

    // Compute the divergence
    div(i) += x.dot(ejk);
    div(j) += x.dot(eki);
    div(k) += x.dot(eij);
  }

  solver->compute(lbo.L);

  if (solver->info() != Eigen::Success) {
    std::cerr << "Something went wrong with the Laplacian" << std::endl;
  }

  Eigen::VectorXd phi = solver->solve(div);

  // Convert the distances to a std::vector
  std::vector<double> dist(phi.data(), phi.data() + phi.size());

  // Find the minimum not nan number
  double minimum = std::numeric_limits<double>::infinity();
  for (const auto &v : dist) {
    if (!std::isnan(v) && v < minimum) {
      minimum = v;
    }
  }

  // Shift the values so the minimum is 0
  for (auto &v : dist) {
    v -= minimum;
  }

  return dist;
}
