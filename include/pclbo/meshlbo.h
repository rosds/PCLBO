#ifndef PC_LBO_MESH_HH
#define PC_LBO_MESH_HH

#include <pcl/PolygonMesh.h>
#include <pcl/common/common_headers.h>
#include <pcl/conversions.h>

#include <Eigen/Sparse>
#include <numeric>
#include <vector>

namespace pclbo {

/** \brief Constructs the Laplace-Beltrami Operator from an input mesh.
 *  This class computes the Laplace-Beltrami Operator from an input mesh.
 */
class MeshLBOEstimation {
 public:
  /** \brief Empty constructor. */
  MeshLBOEstimation() : input_mesh_(), area(new std::vector<double>()) {}

  /** \brief Empty destructor. */
  virtual ~MeshLBOEstimation() {}

  inline void setInputMesh(const pcl::PolygonMeshConstPtr &input) {
    input_mesh_ = input;
  }

  /** \brief Computes and stores internally the LBO and the Mass matrix. */
  void compute();

  /** \brief Surface area */
  double surface_area;

  /** \brief Average edge length. */
  double avg_edge_length;

  /** \brief Input polygonal mesh. */
  pcl::PolygonMeshConstPtr input_mesh_;

  /** \brief Laplace-Beltrami Operator */
  Eigen::SparseMatrix<double> L;

  /** \brief Mass matrix. */
  Eigen::SparseMatrix<double> M;

  /** \brief faces area. */
  std::shared_ptr<std::vector<double>> area;

};  // class MeshLBOEstimation

}  // namespace pclbo

#endif  // PC_LBO_MESH_HH
