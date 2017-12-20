#include "unproject_onto_mesh.h"
#include<iostream>
bool qrcode::unproject_onto_mesh(const Eigen::Vector2f & pos, const Eigen::Matrix4f & model, const Eigen::Matrix4f & proj, const Eigen::Vector4f & viewport, 
	const Eigen::MatrixXd & verticals, Eigen::MatrixXi & facets, Eigen::Vector3f & src, Eigen::Vector3f & dir, std::vector<igl::Hit>& hits)
{
	igl::unproject_ray(pos, model, proj, viewport, src, dir);

	bool right = igl::ray_mesh_intersect(src, dir, verticals, facets, hits);

	if (!right) {
		igl::Hit hit;
		hit.t = -1;
		hits.push_back(hit);
		//std::cout << "false" << std::endl;
	}

	return right;


}
