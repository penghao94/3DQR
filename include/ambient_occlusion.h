// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef AMBIRNT_OCCLUSION_H_
#define AMBIENT_OCCLUSION_H_
#include <vector>
#include <Eigen/dense>
#include <igl/embree/EmbreeIntersector.h>
#include <igl/parallel_for.h>
#include <igl/random_dir.h>
#include "sphere_mesh.h"
#include "spherical_coordinate.h"
#include "random_points_on_spherical_mesh.h"
#include <igl/serialize.h>
#include<math.h>
namespace qrcode {

	void ambient_occlusion(Eigen::MatrixXd &vecticles, Eigen::MatrixXi &facets,std::vector<Eigen::Vector3f> &position,std::vector<Eigen::Vector3f> &normal, int samples, Eigen::VectorXf &result);
	void ambient_occlusion(Eigen::MatrixXd &verticles, Eigen::MatrixXi &facets, std::vector<Eigen::Vector3f> &position, std::vector<Eigen::Vector3f> &normal, std::vector<qrcode::SMesh>&patch, Eigen::VectorXf &result);
	float refine(float r);
}
#endif // !AMBIRNT_OCCLUSION_H_

