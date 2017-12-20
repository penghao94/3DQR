// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef RANDOM_POINTS_ON_SPHERICAL_MESH_H_
#define RANDOM_POINTS_ON_SPHERICAL_MESH_H_
#include <Eigen/dense>
#include <igl/PI.h>
#include<igl/cumsum.h>
#include "Histc.h"
#include<igl/ray_mesh_intersect.h>
#include<igl/random_points_on_mesh.h>
namespace qrcode {
	void random_points_on_spherical_mesh(const Eigen::Vector3f&origin, const Eigen::MatrixXd &verticles, const Eigen::MatrixXi &facets, int samples, Eigen::MatrixXf &result);
	
}

#endif // !RANDOM_POINTS_ON_SPHERICAL_MESH_H_


