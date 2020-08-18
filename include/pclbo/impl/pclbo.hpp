#include <pclbo/pclbo.h>

template <class PointT, class NormalT>
void pclbo::LBOEstimation<PointT, NormalT>::compute() {

  typename pcl::KdTreeFLANN<PointT>::Ptr kdt(new pcl::KdTreeFLANN<PointT>());
  kdt->setInputCloud(_cloud);

  const double avg_dist = pclbo::avg_distance<PointT>(10, _cloud, kdt);
  const double h = 5 * avg_dist;

  std::cout << "Average distance between points: " << avg_dist << std::endl;

  int points_with_mass = 0;
  double avg_mass = 0.0;
  B.resize(_cloud->size());

  std::cout << "Computing the Mass matrix..." << std::flush;

  // Compute the mass matrix diagonal B
  for (int i = 0; i < _cloud->size(); i++) {
    const auto &point = _cloud->at(i);
    const auto &normal = _normals->at(i);

    const auto &normal_vector =
        normal.getNormalVector3fMap().template cast<double>();

    if (!pcl::isFinite(point))
      continue;

    std::vector<int> indices;
    std::vector<float> distances;
    kdt->radiusSearch(point, h, indices, distances);

    if (indices.size() < 4) {
      B[i] = 0.0;
      continue;
    }

    // Project the neighbor points in the tangent plane at p_i with normal n_i
    std::vector<Eigen::Vector3d> projected_points;
    for (const auto &neighbor_index : indices) {
      if (neighbor_index != i) {
        const auto &neighbor_point = _cloud->at(neighbor_index);
        projected_points.push_back(project(point, normal, neighbor_point));
      }
    }

    assert(projected_points.size() >= 3);

    // Use the first vector to create a 2D basis
    Eigen::Vector3d u = projected_points[0];
    u.normalize();
    Eigen::Vector3d v = (u.cross(normal_vector));
    v.normalize();

    // Add the points to a 2D plane
    std::vector<Eigen::Vector2d> plane;

    // Add the point at the center
    plane.push_back(Eigen::Vector2d::Zero());

    // Add the rest of the points
    for (const auto &projected : projected_points) {

      double x = projected.dot(u);
      double y = projected.dot(v);

      // Add the 2D point to the vector
      plane.push_back(Eigen::Vector2d(x, y));
    }

    assert(plane.size() >= 4);

    // Compute the voronoi cell area of the point
    double area = VoronoiDiagram::area(plane);
    B[i] = area;
    avg_mass += area;
    points_with_mass++;
  }

  // Average mass
  if (points_with_mass > 0) {
    avg_mass /= static_cast<double>(points_with_mass);
  }

  // Set border points to have average mass
  for (auto &b : B) {
    if (b == 0.0) {
      b = avg_mass;
    }
  }

  std::cout << "done" << std::endl;
  std::cout << "Computing the stiffness matrix..." << std::flush;

  std::vector<double> diag(_cloud->size(), 0.0);

  // Compute the stiffness matrix Q
  for (int i = 0; i < _cloud->size(); i++) {
    const auto &point = _cloud->at(i);

    if (!pcl::isFinite(point))
      continue;

    std::vector<int> indices;
    std::vector<float> distances;
    kdt->radiusSearch(point, h, indices, distances);

    for (const auto &j : indices) {
      if (j != i) {
        const auto &neighbor = _cloud->at(j);

        double d = (neighbor.getVector3fMap() - point.getVector3fMap()).norm();
        double w = B[i] * B[j] * (1.0 / (4.0 * M_PI * h * h)) *
                   exp(-(d * d) / (4.0 * h));

        I.push_back(i);
        J.push_back(j);
        S.push_back(w);

        diag[i] += w;
      }
    }
  }

  // Fill the diagonal as the negative sum of the rows
  for (int i = 0; i < diag.size(); i++) {
    I.push_back(i);
    J.push_back(i);
    S.push_back(-diag[i]);
  }

  // Compute the B^{-1}Q matrix
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(_cloud->size(), _cloud->size());
  for (int i = 0; i < I.size(); i++) {
    const int row = I[i];
    const int col = J[i];
    Q(row, col) = S[i];
  }

  std::cout << "done" << std::endl;
  std::cout << "Computing eigenvectors" << std::endl;

  Eigen::Map<Eigen::VectorXd> B_vec(B.data(), B.size());

  Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> ges;
  ges.compute(Q, B_vec.asDiagonal());

  eigenvalues = ges.eigenvalues();
  eigenfunctions = ges.eigenvectors();

  // Sort the eigenvalues by magnitude
  std::vector<std::pair<double, int>> map_vector(eigenvalues.size());

  for (auto i = 0; i < eigenvalues.size(); i++) {
    map_vector[i].first = std::abs(eigenvalues(i));
    map_vector[i].second = i;
  }

  std::sort(map_vector.begin(), map_vector.end());

  // truncate the first 100 eigenfunctions
  Eigen::MatrixXd eigenvectors(eigenfunctions.rows(), eigenfunctions.cols());
  Eigen::VectorXd eigenvals(eigenfunctions.cols());

  eigenvalues.resize(map_vector.size());
  for (auto i = 0; i < map_vector.size(); i++) {
    const auto &pair = map_vector[i];
    eigenvectors.col(i) = eigenfunctions.col(pair.second);
    eigenvals(i) = pair.first;
  }

  eigenfunctions = eigenvectors;
  eigenvalues = eigenvals;
}
