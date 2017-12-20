// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef IMAGE_ONTO_MESH_H_
#define IMAGE_ONTO_MESH_H_
#include<vector>
#include<igl/matlab/matlabinterface.h>
#include <igl/viewer/Viewer.h>
#include <igl/parallel_for.h>
#include "global.h"
#include "pixel_to_matrix.h"
#include "unproject_onto_mesh.h"
namespace qrcode {
	void image_onto_mesh(igl::viewer::Viewer &viewer, GLOBAL &global);
}
#endif // !IMAGE_ONTO_MESH_H_
