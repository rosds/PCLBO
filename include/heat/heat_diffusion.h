#ifndef HEAT_DIFF_HEAT_DIFFUSION_HH
#define HEAT_DIFF_HEAT_DIFFUSION_HH

#include <pcl/common/geometry.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <heat/utils.h>
#include <heat/voronoi_diagram.h>


namespace heat {

template <class PointT, class NormalT>
class HeatDiffusion {
    public:
        typedef pcl::PointCloud<PointT> InputCloud;
        typedef typename InputCloud::Ptr InputCloudPtr;
        typedef pcl::PointCloud<NormalT> NormalCloud;
        typedef typename NormalCloud::Ptr NormalCloudPtr;

        virtual ~HeatDiffusion () {}

        void setInputCloud(const InputCloudPtr& cloud) {
            _cloud = cloud; 
        }

        void setCloudNormals(const NormalCloudPtr& normals) {
            _normals = normals; 
        }

        void computeLBO();
    
    private:
        InputCloudPtr _cloud;
        NormalCloudPtr _normals;

        std::vector<double> B, S;
        std::vector<int> I, J;

}; // class HeatDiffusion

}

#include <heat/impl/heat_diffusion.hpp>

#endif // HEAT_DIFF_HEAT_DIFFUSION_HH
