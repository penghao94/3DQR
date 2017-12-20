// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef UNPROJECT_ONTO_MESH_H_
#define UNPROJECT_ONTO_MESH_H_
#include <vector>
#include<Eigen/dense>
#include<igl/Hit.h>
#include<igl/ray_mesh_intersect.h>
#include<igl/unproject_ray.h>
namespace qrcode {
	bool unproject_onto_mesh(const Eigen::Vector2f &pos, const Eigen::Matrix4f &model, const Eigen::Matrix4f &proj, const Eigen::Vector4f &viewport,
		const Eigen::MatrixXd & verticals, Eigen::MatrixXi &facets, Eigen::Vector3f &src, Eigen::Vector3f &dir, std::vector<igl::Hit>& hits);
}

#endif // !UNPROJECT_ONTO_MESH_H_

