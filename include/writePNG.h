// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef WRITE_PNG_H_
#define WRITE_PNG_H_
#include<string>
#include<Eigen/dense>
#include<igl/png/writePNG.h>
namespace qrcode {
	void write_png(std::string file, Eigen::MatrixXi &modules, int scale);
	void write_png(std::string file, Eigen::MatrixXi & modules, Eigen::MatrixXi & modules_c, int scale);
	void write_png1(std::string file, Eigen::MatrixXi & modules, Eigen::MatrixXi & modules_c, int scale);
	void write_png(std::string file, Eigen::MatrixXi &modules);
}
#endif // !WRITE_PNG_H_

