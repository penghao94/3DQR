// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef BWLABEL_H_
#define BWLABEL_H_
#include <cassert>
#include <algorithm>
#include <igl/matlab/matlabinterface.h>
#include <igl/unique.h>
#include <Eigen/dense>
#include "halfedge.h"
namespace qrcode {
	/*label black and withe blocks using 4-connectivity or 8-connectivity*/
	void bwlabel(Engine *engine, Eigen::MatrixXi &bw, int connectivity, Eigen::MatrixXi &label);
	/*exterior bound*/
	void bwbound(Eigen::MatrixXi &label, std::vector<Eigen::MatrixXi>& bound);

	void bwdegenerate(std::vector<Eigen::MatrixXi>&bound,int col, std::vector<Eigen::MatrixXi>& decrement);
}
#endif // !BWlABEL_H_

