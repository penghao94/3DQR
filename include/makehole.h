// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef MAKE_HOLE_H_
#define MAKE_HOLE_H_

#include <algorithm>
#include "global.h"
#include "halfedge.h"
namespace qrcode{

	/*
	For a model mesh set, we should divide it into a rest set and many hole sets.
	For rest set,we should reserve verticals and facets of verticals;
	but for hole sets, we only need their boundary edges.
	As we know, the vertex of these boundary edges are contained by rest verticals set,
	so we only need to build edges set for each hole.
	*/
	void make_hole(GLOBAL &global);
}
#endif // !MAKE_HOLE_H_


