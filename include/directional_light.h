// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef DIRECTIONAL_LIGHT_H_
#define DIRECTIONAL_LIGHT_H_
#include<vector>
#include<string>
#include<Eigen/dense>
#include<igl/viewer/Viewer.h>
#include<igl/matlab/matlabinterface.h>
#include<igl/Timer.h>
#include<igl/writeOBJ.h>
#include "global.h"
#include "carving_down.h"
#include "findhole.h"
#include "makehole.h"
#include "fixhole.h"
#include "visible_mesh_on_sphere.h"
#include "pre_pixel_normal.h"
#include "module_adapter.h"
#include "Light.h"
#include "ambient_occlusion.h"
#include "writePNG.h"
namespace qrcode {

	void directional_light(igl::viewer::Viewer & viewer, Engine * engine, GLOBAL& global, Eigen::MatrixXd &verticles, Eigen::MatrixXi &facets);

}

#endif // !DIRECTIONAL_LIGHT_H_
