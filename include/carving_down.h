// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef CARVING_DOWN_H_
#define CARVING_DOWN_H_
#include <Eigen/dense>
#include "global.h"
namespace qrcode {
	void carving_down(GLOBAL &global, Eigen::MatrixXd &result);
	void patch(int y, int x, GLOBAL &global,Eigen::Vector4d &patch);
	void patch(int y, int x,Eigen::VectorXf depth , Eigen::MatrixXi &modules, GLOBAL &global);
}

#endif // !CARVING_DOWN_H_

