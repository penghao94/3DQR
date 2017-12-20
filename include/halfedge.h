// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef HALFEDGE_H_
#define HALFEDGE_H_
#include<Eigen/Core>
namespace qrcode {
	struct eNode
	{
		int s = 0;
		int d = 0;
		int id = 0;
		eNode* next;
	};
	class eList {
	private:
		eNode* head;
	public:

		eList();
		~eList();
		void add(int x, int y, int id);
		void matrix(Eigen::MatrixXi &E);

	};
}
#endif // !HALFEDGE_H_

