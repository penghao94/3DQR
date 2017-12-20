// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef QR_GLOBAL_H_
#define QR_GLOBAL_H_
#include<Eigen/Core>
#include<igl/Hit.h>
#include "QRinfo.h"

namespace qrcode {
	struct GLOBAL
	{
		qrcode::QRinfo info;  //include pixels, codewords,block_propertys, pixel_propertys(s)

		Eigen::Matrix4f mode;//(s)
		float zoom;//(s)

		Eigen::MatrixXd model_vertices;//(s)
		Eigen::MatrixXi model_facets;//(s)
		Eigen::MatrixXd model_colors;//(s)

		Eigen::MatrixXf source;//(pixels.size+2*border)*scale+1(s)
		Eigen::MatrixXf direct;//(pixels.size+2*border)*scale+1(s)

        /*
		include id, u, v, t
		(pixels.size+2*border)*scale+1+2*scale
		*/
	    std::vector<igl::Hit> hitmap;//(s)
		Eigen::MatrixXd hit_matrix;//(pixels.size+2*border)*scale+1//(s)

		Eigen::MatrixXi under_control;//(s)
		std::vector<Eigen::Vector2i> anti_indicatior;//(s)
		std::vector<std::vector<Eigen::Vector2i>> indicator;//(pixels.size+2*border)*scale(s)
		Eigen::MatrixXd qr_verticals;//(pixels.size+2*border)*scale(s)
		Eigen::MatrixXi qr_facets;//(s)
		Eigen::MatrixXd qr_colors;//(s)

		
		std::vector<Eigen::MatrixXi> islands;//(s)
		std::vector<int> seeds;//(s)

		std::vector<std::vector<int>> component;//(s)
		std::vector<std::vector<int>> hole_facet;//(s)
		std::vector<Eigen::MatrixXi> holes;//(s)
		
		Eigen::MatrixXd rest_verticals;//(s)
		Eigen::MatrixXi rest_facets;//(s)

		std::vector<Eigen::MatrixXi> patches;//(d)
		
		
		Eigen::VectorXd carve_depth;//(pixels.size+2*border)*scale;(s)

		float latitude_upper,latitude_lower,longitude,distance;

		std::vector<Eigen::Vector3i> black_module_segments;

	};
}
#endif // !QR_GLOBAL_H_
