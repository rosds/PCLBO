#ifndef PC_LBO_VORONOI_DIAGRAM_HH
#define PC_LBO_VORONOI_DIAGRAM_HH

#include <vector>
#include <Eigen/Core>

// includes for defining the Voronoi diagram adaptor
#include <CGAL/Cartesian.h>
#include <CGAL/Polygon_2.h>

// includes for defining the Voronoi diagram adaptor
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>


namespace pclbo {

class VoronoiDiagram {
public:

    // typedefs for defining the adaptor
    typedef CGAL::Exact_predicates_inexact_constructions_kernel                  K;
    typedef CGAL::Delaunay_triangulation_2<K>                                    DT;
    typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT>                 AT;
    typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP;
    typedef CGAL::Voronoi_diagram_2<DT,AT,AP>                                    VD;
    // typedef for the result type of the point location
    typedef AT::Site_2                    Site_2;
    typedef AT::Point_2                   Point_2;
    typedef VD::Locate_result             Locate_result;
    typedef VD::Vertex_handle             Vertex_handle;
    typedef VD::Face_handle               Face_handle;
    typedef VD::Halfedge_handle           Halfedge_handle;
    typedef VD::Ccb_halfedge_circulator   Ccb_halfedge_circulator;

    typedef CGAL::Polygon_2<CGAL::Cartesian<double> > Polygon;

    /** \brief Compute the area of the Voronoi Cell corresponding to the first 
     * point.
     *
     *  This function creates a Voronoi diagram from the given points using the 
     *  CGAL library. Afterwards, it computes the area of the Voronoi Cell 
     *  that contains the first point in the array when possible.
     *
     *  \param[in] points Array of 2D points representing the center of the Voronoi 
     *  cells.
     *  \return The area of the Voronoi Cell containing the first point of the 
     *  array.
     */
    static double area(const std::vector<Eigen::Vector2d>& points) {

        assert(points.size() >= 4);

        VD vd;
        for (const auto& point : points) {
            vd.insert(Site_2(point(0), point(1)));
        }

        assert(vd.is_valid());

        Locate_result lr = vd.locate(Point_2(0.0, 0.0));

        if (Face_handle* f = boost::get<Face_handle>(&lr)) {

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
};

} // namespace pclbo

#endif // PC_LBO_VORONOI_DIAGRAM_HH
