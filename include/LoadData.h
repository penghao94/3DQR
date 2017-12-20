// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef LOADDATA_H_
#define LOADDATA_H_
#include <Eigen/dense>
namespace qrcode {
	struct mesh_qr 
	{
		Eigen::MatrixXd mesh_ertices;
		Eigen::MatrixXi mesh_facets;
		Eigen::MatrixXi mesh_colors;

	};
}
#endif // !LOADDATA_H_

