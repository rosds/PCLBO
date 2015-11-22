#ifndef PC_LBO_GEODESICS_MESH_HH
#define PC_LBO_GEODESICS_MESH_HH

#include <vector>

#include <Eigen/Sparse>
#include <Eigen/CholmodSupport>
#include <pcl/common/common_headers.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

#include <pclbo/meshlbo.h>

namespace pclbo {

/** \brief Compute geodesic distances on a mesh.
 */
class MeshGeoHeat {
    public:
        typedef std::shared_ptr<MeshGeoHeat> Ptr;

        /** \brief Empty constructor. */
        MeshGeoHeat () : input_mesh_() {}

        /** \brief Empty destructor. */
        virtual ~MeshGeoHeat () {}

        inline void
        setInputMesh (const pcl::PolygonMeshConstPtr &input)
        { input_mesh_ = input; }

        void compute () {
            lbo.setInputMesh(input_mesh_);
            lbo.compute();
        }

        /** \brief Compute the distances from the provided point. */
        std::vector<double> getDistancesFrom(const int x);

    private:

        /** \brief Input polygonal mesh. */
        pcl::PolygonMeshConstPtr input_mesh_;

        pclbo::MeshLBOEstimation lbo;
}; // class MeshGeoHeat

} // namespace pclbo

#endif // PC_LBO_GEODESICS_MESH_HH
