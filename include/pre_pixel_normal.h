// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef PRE_PIXEL_NORMAL_H_
#define PRE_PIXEL_NORMAL_H_

#include <vector>
#include <Eigen/dense>
#include "global.h"
namespace qrcode {
	
	void pre_pixel_normal(GLOBAL &global, Eigen::MatrixXd &qr_verticales, Eigen::MatrixXf &position, Eigen::MatrixXf &normal);

}

#endif // !PRE_PIXEL_NORMAL_H_

