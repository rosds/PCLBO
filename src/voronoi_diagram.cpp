#include <pclbo/voronoi_diagram.h>

// includes for defining the Voronoi diagram adaptor
#include <CGAL/Cartesian.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Voronoi_diagram_2.h>

namespace pclbo {
double computeVoronoiCenterCellArea(
    const std::vector<Eigen::Vector2d> &points) {
  // typedefs for defining the adaptor
  using K = CGAL::Exact_predicates_inexact_constructions_kernel;
  using DT = CGAL::Delaunay_triangulation_2<K>;
  using AT = CGAL::Delaunay_triangulation_adaptation_traits_2<DT>;
  using AP =
      CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT>;
  using VD = CGAL::Voronoi_diagram_2<DT, AT, AP>;

  // typedef for the result type of the point location
  using Site_2 = AT::Site_2;
  using Point_2 = AT::Point_2;
  using Locate_result = VD::Locate_result;
  using Vertex_handle = VD::Vertex_handle;
  using Face_handle = VD::Face_handle;
  using Halfedge_handle = VD::Halfedge_handle;
  using Ccb_halfedge_circulator = VD::Ccb_halfedge_circulator;

  using Polygon = CGAL::Polygon_2<CGAL::Cartesian<double>>;

  assert(points.size() >= 4);

  VD vd;
  for (const auto &point : points) {
    vd.insert(Site_2(point(0), point(1)));
  }

  assert(vd.is_valid());

  Locate_result lr = vd.locate(Point_2(0.0, 0.0));

  if (Face_handle *f = boost::get<Face_handle>(&lr)) {
    Ccb_halfedge_circulator ec_start = (*f)->outer_ccb();
    Ccb_halfedge_circulator ec = ec_start;

    Polygon pl;
    do {
      if (!(ec->has_source())) {
        std::cerr << "voronoi_cell_area : error : no source.\n";
        return 0.0;
      }

      double x = ec->source()->point().x();
      double y = ec->source()->point().y();
      pl.push_back(CGAL::Cartesian<double>::Point_2(x, y));

    } while (++ec != ec_start);

    return pl.area();
  } else {
    return 0.0;
  }
}

}  // namespace pclbo
