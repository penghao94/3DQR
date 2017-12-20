// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef MODULE_ADAPTER_H_
#define MODULE_ADAPTER_H_
#include <algorithm>
#include <Eigen/dense>
#include "global.h"
#include "bwlabel.h"
#include "writePNG.h"
namespace qrcode {
	std::vector<Eigen::MatrixXi> module_adapter(Engine * engine, GLOBAL &global);
}

#endif // !MODULE_ADAPTER_H_

