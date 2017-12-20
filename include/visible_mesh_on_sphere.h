// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef VISIBLE_MESH_ON_SPHERE_H_
#define VISIBLE_MESH_ON_SPHERE_H_
#include <vector>
#include <Eigen/dense>
#include <igl/parallel_for.h>
#include "global.h"
#include "sphere_mesh.h"
#include "raw_to_visible_polygon.h"
namespace qrcode {

	//************************************
	// Method:    qrcode::visible_mesh_on_sphere
	// Returns:   std::vector<qrcode::SMesh>
	//
	// @param std::vector<Eigen::Vector3i> & position  (y,x,label)
	// @param std::vector<Eigen::MatrixXi> & decrement
	// @param GLOBAL & global
	//************************************
	std::vector<qrcode::SMesh> visible_mesh_on_sphere(std::vector<Eigen::Vector3i>&position, std::vector<Eigen::MatrixXi>& decrement, GLOBAL& global);
}

#endif
