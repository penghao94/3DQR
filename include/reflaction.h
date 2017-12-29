// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef REFLACTION
#include<vector>
#include<algorithm>
#include<igl/serialize.h>
#include<igl/writeOBJ.h>
#include "sphere_mesh.h"
#include "global.h"
#include "carving_down.h"
namespace qrcode {
	void reflaction(GLOBAL &global, Eigen::MatrixXd &verticles, Eigen::MatrixXi&facets);
}
#endif // !REFLACTION

