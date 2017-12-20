
// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef RAW_TO_VISIBLEPOLYGON_H_
#define RAW_TO_VISIBLEPOLYGON_H_

#include<vector>
#include <Eigen/dense>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>

namespace qrcode {
	void raw_to_visible_polygon(Eigen::RowVector2d &query_point, Eigen::MatrixXi &edges, int scale, int col, std::vector<Eigen::Vector2d>&bound);
}
#endif // !RAWTOVISIBLEPOLYGON_H_


