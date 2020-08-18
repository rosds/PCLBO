#ifndef PC_LBO_GEODESICS_MESH_HH
#define PC_LBO_GEODESICS_MESH_HH

#include <vector>

#include <Eigen/CholmodSupport>
#include <Eigen/Sparse>
#include <pcl/PolygonMesh.h>
#include <pcl/common/common_headers.h>
#include <pcl/conversions.h>

#include <pclbo/meshlbo.h>

namespace pclbo {

/** \brief Compute geodesic distances on a mesh.
 */
class MeshGeoHeat {
public:
  typedef std::shared_ptr<MeshGeoHeat> Ptr;

  /** \brief Empty constructor. */
  MeshGeoHeat() : input_mesh_(nullptr), time_step(-1.0) {}

  /** \brief Empty destructor. */
  virtual ~MeshGeoHeat() {}

  /** \brief Set input mesh.
   *  \param[in] input Input mesh.
   */
  inline void setInputMesh(const pcl::PolygonMeshConstPtr &input) {
    input_mesh_ = input;
  }

  /** \brief Set time step for heat diffusion.
   *  \param[in] ts Time step.
   */
  inline void setTimeStep(const double ts) { time_step = ts; }

  /** \brief Compute and store the Laplace-Beltrami operator. **/
  void compute() {
    lbo.setInputMesh(input_mesh_);
    lbo.compute();
  }

  /** \brief Compute the distances from the provided point. */
  std::vector<double> getDistancesFrom(const int x);

private:
  /** \brief Time step for heat diffusion. **/
  double time_step;

  /** \brief Input polygonal mesh. */
  pcl::PolygonMeshConstPtr input_mesh_;

  pclbo::MeshLBOEstimation lbo;

}; // class MeshGeoHeat

} // namespace pclbo

#endif // PC_LBO_GEODESICS_MESH_HH
