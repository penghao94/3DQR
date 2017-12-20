// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#include <Eigen/core>
#include <igl/matlab/matlabinterface.h>
#include "Qrcoder.h"
#include "QRinfo.h"
#include "VisualArea.h"
#include "pixel_to_matrix.h"
namespace qrcode {

	qrcode::QRinfo readQR(Engine *engine,std::string &file_name);
}