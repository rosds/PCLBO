#include <heat/heat_diffusion.h>

template <class PointT, class NormalT>
void heat::HeatDiffusion<PointT, NormalT>::computeLBO() {
    typename pcl::KdTreeFLANN<PointT>::Ptr kdt(new pcl::KdTreeFLANN<PointT>());
    kdt->setInputCloud(_cloud);

    const double avg_dist = heat::avg_distance<PointT>(10, _cloud, kdt);
    const double hh = avg_dist * avg_dist;

        
    B.resize(_cloud->size());

    for (int i = 0; i < _cloud->size(); i++) {
        const auto& point = _cloud->at(i);
        const auto& normal = _normals->at(i);

        std::vector<int> indices;
        std::vector<float> distances;

        kdt->radiusSearch(point, 6 * avg_dist, indices, distances);

        if (indices.size() < 3) {
            B[i] = 0.0;
            continue;
        }

        std::vector<Eigen::Vector3f> projected_points;
        for (const auto& neighbor_index : indices) {
            if (neighbor_index != i) {
                const auto& neighbor_point = _cloud->at(neighbor_index);

                Eigen::Vector3f projected;
                pcl::geometry::project(
                    neighbor_point.getVector3fMap(), 
                    point.getVector3fMap(), 
                    normal.getNormalVector3fMap(), 
                    projected);

                projected_points.push_back(projected);
            }
        }

        // Use the first vector to create a basis
        auto u = projected_points[0].normalized();
        auto v = (u.cross(normal.getNormalVector3fMap())).normalized();

        std::vector<Eigen::Vector2d> plane;

        // Add the point at the center
        plane.push_back(Eigen::Vector2d::Constant(0.0));

        for (const auto& projected : projected_points) {
            Eigen::Vector2d p;
            p(0) = projected.dot(u);
            p(1) = projected.dot(v);
            plane.push_back(p);
        }

        // Compute the voronoi cell area of the point
        VoronoiDiagram vd(plane);
        
        B[i] = vd.area();

        for (const auto& neighbor_index : indices) {
            if (neighbor_index != i) {
                const auto& neighbor_point = _cloud->at(neighbor_index);

                const auto d = neighbor_point.getVector3fMap() - point.getVector3fMap();
                const double sqrdist = d.dot(d);
                const double weight = exp(-sqrdist / hh) / (M_PI * hh * hh / 4.0);

                I.push_back(i);
                J.push_back(neighbor_index);
                S.push_back(weight);
            }
        }
    }

    // Compute the average area
    double sum = 0.0;
    double n = 0.0;
    for (const auto& a : B) {
        if (a > 0.0) {
            sum += a;
            n += 1.0;
        }
    }
    sum /= n;

    for (auto& a : B) {
        if (a <= 0.0) {
            a = sum;
        }
    }

    std::vector<double> diag(_cloud->size());
    for (int i = 0; i < I.size(); i++) {
        if (I[i] > J[i]) {
            S[i] *= B[J[i]]; 
            S[i] *= B[I[i]]; 
        } else {
            S[i] *= B[I[i]]; 
            S[i] *= B[J[i]]; 
        } 
        diag[I[i]] += S[i];
    }

    for (int i = 0; i < diag.size(); i++) {
        I.push_back(i);
        J.push_back(i);
        S.push_back(-diag[i]);
    }
}
