#ifndef PC_LBO_DIFFUSION_HH
#define PC_LBO_DIFFUSION_HH

#include <cmath>
#include <boost/thread/thread.hpp>
#include <Eigen/Eigen>

#include <pcl/common/geometry.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pclbo/utils.h>
#include <pclbo/voronoi_diagram.h>


namespace pclbo {

/** \brief Estimates the Laplace-Beltrami Operator for the input cloud.
 */
template <class PointT, class NormalT>
class LBOEstimation {
    public:
        typedef pcl::PointCloud<PointT> InputCloud;
        typedef typename InputCloud::Ptr InputCloudPtr;
        typedef pcl::PointCloud<NormalT> NormalCloud;
        typedef typename NormalCloud::Ptr NormalCloudPtr;

        typedef std::shared_ptr<LBOEstimation<PointT, NormalT> > Ptr;

        virtual ~LBOEstimation () {}

        void setInputCloud(const InputCloudPtr& cloud) {
            _cloud = cloud; 
        }

        void setCloudNormals(const NormalCloudPtr& normals) {
            _normals = normals; 
        }

        void compute();
    
        Eigen::MatrixXd eigenfunctions;
        Eigen::VectorXd eigenvalues;

    private:
        InputCloudPtr _cloud;
        NormalCloudPtr _normals;

        // Mass matrix and stiffness matrix
        std::vector<double> B, S;
        std::vector<int> I, J;
}; // class HeatDiffusion

}

#include <pclbo/impl/pclbo.hpp>

#endif // PC_LBO_PC_LBOUSION_HH
