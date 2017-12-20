// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef LIGHT_H_
#define LIGHT_H_
#include <vector>
#include <Eigen/dense>
#include <igl/embree/EmbreeIntersector.h>
#include <igl/Hit.h>
#include <igl/parallel_for.h>
#include "global.h"
namespace qrcode {

	void light(Eigen::MatrixXd &verticles, Eigen::MatrixXi &facets, Eigen::VectorXf &origin, std::vector<Eigen::Vector3f>&destination, Eigen::Matrix<bool, Eigen::Dynamic, 1> &result);
}
#endif // !LIGHT_H_

