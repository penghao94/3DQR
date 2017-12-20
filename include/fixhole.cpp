#include "fixhole.h"

void qrcode::fix_hole(Engine * engine, GLOBAL & global)
{
	global.patches.resize(global.component.size());

	for (int i = 0; i < global.component.size(); i++) {

		Eigen::MatrixXf verticals;
		Eigen::MatrixXi edges;
		Eigen::MatrixXf seed(global.component[i].size(), 4);

		/*Reconstruct island verticals and edges*/
		int size = 0;
		for (int j = 0; j < global.component[i].size(); j++) size += global.islands[global.component[i][j]].rows();

		Eigen::MatrixXf island_v(2 * size, 4);
		
		int index = 0;
		std::vector<int> queue; 
		
		for (int j = 0; j < global.component[i].size(); j++) {

			int p = global.seeds[global.component[i][j]];
			seed.row(j) << global.qr_verticals(p, 0), global.qr_verticals(p, 1), global.qr_verticals(p, 2), 0;

			for (int k = 0; k < global.islands[global.component[i][j]].rows(); k++) {

				Eigen::Vector2i pos = global.islands[global.component[i][j]].row(k);

				island_v.row(index++) << global.qr_verticals(pos(0), 0), global.qr_verticals(pos(0), 1), global.qr_verticals(pos(0), 2), 0;
				island_v.row(index++) << global.qr_verticals(pos(1), 0), global.qr_verticals(pos(1), 1), global.qr_verticals(pos(1), 2), 0;

				queue.push_back(pos(0));
				queue.push_back(pos(1));
			}
		}
		Eigen::MatrixXf island_vertex;
		Eigen::VectorXi IAS, ICS;
		igl::unique_rows(island_v, island_vertex, IAS, ICS);
		
		verticals = island_vertex;
		edges.resize(size, 2);
		for (int j = 0; j < edges.rows(); j++)  edges.row(j) << ICS(2 * j), ICS(2 * j + 1);
		
		/*Reconstruct hole verticals and edges*/

		size = global.holes[i].rows();

		Eigen::MatrixXf hole_v(2*size,4);

		index = 0;
		for (int j = 0; j < global.holes[i].rows(); j++) {

			Eigen::Vector2i pos = global.holes[i].row(j);

			hole_v.row(index++) << global.rest_verticals(pos(0), 0), global.rest_verticals(pos(0), 1), global.rest_verticals(pos(0), 2), 0;
			hole_v.row(index++) << global.rest_verticals(pos(1), 0), global.rest_verticals(pos(1), 1), global.rest_verticals(pos(1), 2), 0;

		}

		Eigen::MatrixXf hole_vertex;
		Eigen::VectorXi IAH, ICH;
		igl::unique_rows(hole_v, hole_vertex, IAH, ICH);
		
		verticals.conservativeResize(verticals.rows() + hole_vertex.rows(), 4);
		
		verticals.block(island_vertex.rows(), 0, hole_vertex.rows(), 4) = hole_vertex;

		
		edges.conservativeResize(edges.rows() + size, 2);
		for (int j = -size; j < 0; j++) edges.row(edges.rows() + j) << ICH(2 * (j + size)) + island_vertex.rows(), ICH(2 * (j + size) + 1) + island_vertex.rows();

		verticals = (global.mode*verticals.transpose()).transpose();

		verticals.conservativeResize(verticals.rows(), 2);

		seed = (global.mode*seed.transpose()).transpose();
		seed.conservativeResize(seed.rows(), 2);


	

		Eigen::MatrixXf nouse;
		Eigen::MatrixXi patches;

		igl::triangle::triangulate(verticals, edges, seed, "0.5", nouse,patches);

		Eigen::VectorXi IA(IAS.rows() + IAH.rows());
		for (int j = 0; j < IAS.rows(); j++) IA(j) = queue[IAS(j)];
		for (int j = 0; j < IAH.rows(); j++) IA(IAS.rows() + j) = global.holes[i](IAH(j) / 2, IAH(j) % 2) + global.qr_verticals.rows();

		global.patches[i].resize(patches.rows(), patches.cols());

		for (int r = 0; r < patches.rows(); r++)
			for (int c = 0; c < patches.cols(); c++)
			global.patches[i](r, c) = IA(patches(r, c));

	}

} 
