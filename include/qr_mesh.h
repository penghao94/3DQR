// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef QR_MESH_H_
#define QR_MESH_H_
#include <algorithm>
#include<iterator>
#include<Eigen/dense>
#include<igl/matlab/matlabinterface.h>
#include "global.h"
#include "pixel_to_matrix.h"
#include "bwlabel.h"
namespace qrcode {
	void generate_qr_mesh(Engine *engine, GLOBAL &global);
}
#endif // !QR_MESH_H_
