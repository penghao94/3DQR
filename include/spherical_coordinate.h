// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef SPHERICAL_COORDINATE_H_
#define SPHERICAL_COORDINATE_H_

#include <Eigen/dense>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
namespace qrcode {
	Eigen::MatrixXf rot(const Eigen::Vector3f &before, const Eigen::Vector3f &after);
	float Box(const Eigen::Vector3f&origin, const Eigen::MatrixXd &V, const Eigen::MatrixXf &rot, Eigen::MatrixXf &Box);
}


#endif // !SPHERICAL_COORDINATE_H_

