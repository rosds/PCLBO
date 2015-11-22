#include <pclbo/meshlbo.h>

inline double 
cot(const double angle) {
    return 1.0 / tan(angle);
}

inline double
angleBetween(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    Eigen::Vector3f c = a.cross(b);
    return atan2f(c.norm(), a.dot(b));
}

void
pclbo::MeshLBOEstimation::compute () {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromPCLPointCloud2<pcl::PointXYZ>(input_mesh_->cloud, *cloud);

    // Number of faces
    const size_t F = input_mesh_->polygons.size();

    // Number of vertices
    const size_t N = cloud->size();

    area->resize(F);

    // Initialize the mass matrix
    M = Eigen::SparseMatrix<double>(N, N);

    // For the cotangent accumulation
    std::map<std::pair<int, int>, double> cotangent;

    // Compute the area of each face and the cotangent of opposite angles.
    for (int f = 0; f < F; f++) {

        const auto& face = input_mesh_->polygons.at(f);
        const auto i = face.vertices[0];
        const auto j = face.vertices[1];
        const auto k = face.vertices[2];
        const auto& pi = cloud->at(i).getVector3fMap();        
        const auto& pj = cloud->at(j).getVector3fMap();        
        const auto& pk = cloud->at(k).getVector3fMap();        

        // Compute the face area
        const auto u = pj - pi;
        const auto v = pk - pi;
        area->at(f) = (u.cross(v)).norm() / 2;

        // Add the areas to the corresponding indices in the mass matrix
        M.coeffRef(i, i) += area->at(f) / 3;
        M.coeffRef(j, j) += area->at(f) / 3;
        M.coeffRef(k, k) += area->at(f) / 3;

        std::pair<int, int> ei(std::min(i, j), std::max(i, j));
        std::pair<int, int> ej(std::min(j, k), std::max(j, k));
        std::pair<int, int> ek(std::min(k, i), std::max(k, i));

        const double alpha = angleBetween(pj - pk, pi - pk);
        const double beta  = angleBetween(pk - pi, pj - pi);
        const double gamma = angleBetween(pi - pj, pk - pj);

        cotangent[ei] += cot(alpha) / 2.0f;
        cotangent[ej] += cot(beta)  / 2.0f;
        cotangent[ek] += cot(gamma) / 2.0f;

    }

    // Number of unique edges
    const size_t E = cotangent.size();

    // Compute the surface area just for fun
    surface_area = std::accumulate(area->begin(), area->end(), 0.0f);

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> C(E);
    Eigen::SparseMatrix<double> d(E, N);

    int i = 0;
    avg_edge_length = 0.0f;

    // Construct the stiffness matrix
    for (auto& p : cotangent) {
        const std::pair<int, int>& edge = p.first;

        C.diagonal()[i] = p.second;
        d.insert(i, edge.first)  = 1.0;
        d.insert(i, edge.second) = -1.0;
        i++;

        // This is just to compute the average edge length.
        const auto u = cloud->at(edge.first).getVector3fMap();
        const auto v = cloud->at(edge.second).getVector3fMap();
        avg_edge_length += (u - v).norm();
    }

    // Store the average edge length.
    avg_edge_length /= static_cast<double>(cotangent.size());

    // The Laplace-Beltrami Operator
    L = d.transpose() * C * d;
    L = L + 10e-18 * M;
}
