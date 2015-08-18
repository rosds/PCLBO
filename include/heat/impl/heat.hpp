#include <heat/heat.h>

template <typename PointT>
typename heat::HeatKernelSignature<PointT>::HeatSignature 
heat::HeatKernelSignature<PointT>::compute(const int x, const double t, const int n_functions) {

    HeatSignature hs(new std::vector<double>(_cloud->size()));

    for (int y = 0; y < _cloud->size(); y++) {

        double sum = 0.0;
        for (int j = 0; j < n_functions; j++) {
            double lambda = _eigenvalue(j);
            Eigen::VectorXd psi = _eigenfunctions.col(j);
            sum += exp(-lambda * t) * psi(x) * psi(y); 
        }

        hs->at(y) = sum;
    }

    return hs;
}
