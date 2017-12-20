
#include "raw_to_visible_polygon.h"

// Define the used kernel and arrangement  
typedef CGAL::Exact_predicates_exact_constructions_kernel       Kernel;
typedef Kernel::FT												FT;
typedef Kernel::Point_2											Point_2;
typedef Kernel::Segment_2                                       Segment_2;
typedef CGAL::Arr_segment_traits_2<Kernel>                      Traits_2;
typedef CGAL::Arrangement_2<Traits_2>                           Arrangement_2;
typedef Arrangement_2::Halfedge_const_handle                    Halfedge_const_handle;
typedef Arrangement_2::Face_handle                              Face_handle;
// Define the used visibility class 
typedef CGAL::Triangular_expansion_visibility_2<Arrangement_2, CGAL::Tag_true>  TEV;

void qrcode::raw_to_visible_polygon(Eigen::RowVector2d & query_point, Eigen::MatrixXi & edges, int scale, int col, std::vector<Eigen::Vector2d>& bound)
{
	
	bound.clear();
	//Defining the input geometry
	
	std::vector<Segment_2> segments;
	for (int i = 0; i < edges.rows(); i++) {
		Point_2 s(double(int(edges(i, 0) % col)*scale), double(int(edges(i, 0) / col)*scale));	
		Point_2 d(double(int(edges(i, 1) % col)*scale), double(int(edges(i, 1) / col)*scale));
		
		segments.push_back(Segment_2(s, d));
	}
	//Defining the query point
	Point_2 q(query_point(1), query_point(0));
	// insert geometry into the arrangement 
	Arrangement_2 env;
	CGAL::insert_non_intersecting_curves(env, segments.begin(), segments.end());

	//Find the halfedge whose target is the query point.
	//(usually you may know that already by other means)  
	Face_handle fit;
	for (fit = env.faces_begin(); fit != env.faces_end(); ++fit) {
		if (!fit->is_unbounded()) {
			break;
			std::cout << "Holy shit" << std::endl;
		}
	}
	//visibility query
	Arrangement_2 output_arr;
	TEV tev(env);
	Face_handle fh = tev.compute_visibility(q, fit, output_arr);
	Arrangement_2::Ccb_halfedge_circulator curr = fh->outer_ccb();

	bound.push_back(Eigen::Vector2d(CGAL::to_double(curr->source()->point().x()), CGAL::to_double(curr->source()->point().y())));
	while (++curr != fh->outer_ccb()) {
		bound.push_back(Eigen::Vector2d(CGAL::to_double(curr->source()->point().x()), CGAL::to_double(curr->source()->point().y())));
	}
}

