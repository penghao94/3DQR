// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef FIND_HOLE_H_
#define FIND_HOLE_H_
#include <algorithm>
#include <iterator>
#include <igl/matlab/matlabinterface.h>
#include <Eigen/dense>
#include "global.h"
#include "bwlabel.h"
namespace qrcode {
	void find_hole(Engine *engine,GLOBAL &global);
}
#endif // !FIND_HOLE_H_

