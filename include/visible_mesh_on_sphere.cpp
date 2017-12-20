#include "visible_mesh_on_sphere.h"

std::vector<qrcode::SMesh> qrcode::visible_mesh_on_sphere(std::vector<Eigen::Vector3i>& position, std::vector<Eigen::MatrixXi>& decrement, GLOBAL & global)
{
	std::vector<qrcode::SMesh> meshes(position.size());

	int scale = global.info.scale;
	int col= (global.info.pixels.size() + 2 * global.info.border)*scale + 1;
	
	const auto &vis = [&meshes,&position,&decrement,&global,&col](const int p) {
		//std::cout << p << std::endl;

		std::vector<Eigen::Vector2d> bound;

		qrcode::raw_to_visible_polygon(Eigen::RowVector2d(position[p](0) + 0.5, position[p](1) + 0.5), decrement[position[p](2) - 1], 1, col, bound);
		 
		meshes[p].V.resize(bound.size() + 1, 3);

		for (int k = 0; k < bound.size(); k++) {
			int x1 = floor(bound[k](1));
			int y1 = floor(bound[k](0));
			int x2 = x1 + 1;
			int y2 = y1 + 1;
			meshes[p].V.row(k) = global.hit_matrix.row(x1*col + y1) + (bound[k](1) - floor(bound[k](1)))
				*(global.hit_matrix.row((x1 + 1)*col + y1) - global.hit_matrix.row(x1*col + y1)) +
				(bound[k](0) - floor(bound[k](0)))*(global.hit_matrix.row(x1*col + y1 + 1) - global.hit_matrix.row(x1*col + y1));
		}



		meshes[p].V.row(meshes[p].V.rows() - 1) = (global.hit_matrix.row(position[p](0)*col + position[p](1)) + global.hit_matrix.row((position[p](0) + 1)*col 
			+ position[p](1) + 1)) / 2;

		
		Eigen::MatrixXd V = (meshes[p].V - meshes[p].V.row(meshes[p].V.rows() - 1).replicate(meshes[p].V.rows(), 1)) * 100;
		std::vector<Eigen::RowVector3i> facets;
		Eigen::VectorXd area(bound.size());

		for (int k = 0; k < bound.size(); k++) {
			Eigen::Vector3d a = V.row(k).transpose()*100;
			Eigen::Vector3d b = V.row((k + 1)%bound.size()).transpose();
			Eigen::Vector3d c = V.row(bound.size()).transpose();
			Eigen::Vector3d d = (c - a) * 100;
			Eigen::Vector3d f = (c - b) * 100;
			area(k) = d.cross(f).norm();
		}

		area /= area.sum();
		for (int k = 0; k < bound.size(); k++)
			if (area(k) > 0.001)
				facets.emplace_back(k, bound.size(), (k + 1) % bound.size());

		meshes[p].F.resize(facets.size(), 3);
		for (int k = 0; k < facets.size(); k++) meshes[p].F.row(k) = facets[k];
		/*std::cout << meshes[p].V << std::endl;
		std::cout << meshes[p].F << std::endl;
		std::cout << area << std::endl;
		std::string binary_file = "F:/3DQ/3DQR/data/temp";
		igl::serialize(meshes[p].V, "v", binary_file,true);
		igl::serialize(meshes[p].F, "f", binary_file);*/
	};

	

	igl::parallel_for(position.size(), vis, 1000);
	for (int i = 0; i < position.size(); i++) vis(i);
	return meshes;
}
