#include <heat/voronoi_diagram.h>

heat::VoronoiDiagram::VoronoiDiagram(const std::vector<Eigen::Vector2d>& points) {
    std::vector<double> pt;
    // double val;

    VD vd;
    Point_2* pt2;
    Site_2* t;

    for (const auto& point : points) {
        vd.insert(Site_2(point(0), point(1)));
    }

    if (!vd.is_valid()) {
        std::cerr << "voronoi_cell_area: not valid \n";
    }

    Locate_result lr = vd.locate(Point_2(points[0](0), points[0](1)));
    Face_handle& fi = boost::get<Face_handle>(lr);
    Ccb_halfedge_circulator ec_start = fi->outer_ccb();
    Ccb_halfedge_circulator ec = ec_start;

    Polygon pl;

    do {
        if (!ec->has_source()) {
            std::cerr << "voronoi_cell_area : error : no source.\n";
        }

        double x = ec->source()->point().x();
        double y = ec->source()->point().y();

        pl.push_back(CGAL::Cartesian<double>::Point_2(x, y));
    } while (++ec != ec_start);


    cell_area = pl.area();
}
