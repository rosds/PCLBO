#ifndef HEAT_HEAT_HH__
#define HEAT_HEAT_HH__

#include <Eigen/Core>
#include <pclbo/pclbo.h>

namespace heat {

template <typename PointT>
class HeatKernelSignature {
public:
    typedef std::shared_ptr<HeatKernelSignature<PointT> > Ptr;
    typedef std::shared_ptr<std::vector<double> > HeatSignature;

    virtual ~HeatKernelSignature() {}

    void setInputCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud) {
        _cloud = cloud;
    }

    void setEigenValues(const Eigen::VectorXd& values) {
        _eigenvalue = values;
    }

    void setEigenFunctions(const Eigen::MatrixXd& vectors) {
        _eigenfunctions = vectors;
    }

    HeatSignature compute(const int x, const double t, const int n_functions = 200);

private:
    Eigen::MatrixXd _eigenfunctions;
    Eigen::VectorXd _eigenvalue;
    typename pcl::PointCloud<PointT>::Ptr _cloud;

}; // class HeatKernelSignature

} // namespace heat

#include <heat/impl/heat.hpp>

#endif // HEAT_HEAT_HH__
