#include "image_onto_mesh.h"
#include <igl/unproject_onto_mesh.h>
void qrcode::image_onto_mesh(igl::viewer::Viewer & viewer, GLOBAL & global)
{
	Eigen::MatrixXi modules, functions;
	pixel_to_matrix(global.info.pixels, global.info.border, modules, functions);

	/*Cross points*/
	int scale = global.info.scale;
	int size = (global.info.pixels.size()+2*global.info.border)*scale + 1;

	Eigen::MatrixXi Modules;
	Modules.setZero(size, size);

	for (int y = 0; y < modules.rows() - 1; y++)
		for (int x = 0; x < modules.cols() - 1; x++)
			for (int u = 0; u < scale; u++)
				for (int v = 0; v < scale; v++)
					Modules(y*scale + u, x*scale + v) = modules(y, x);
	
	std::vector<Eigen::Vector2i> position;

	for (int y = -scale; y < size + scale; y++) {
		for (int x = -scale; x < size + scale; x++) {
			position.push_back(Eigen::Vector2i(-y, x));
		}
	}

	Eigen::Vector2f center;
	center << viewer.core.viewport(3) / 2 + (static_cast<float>(size) / 2 + scale), viewer.core.viewport(2) / 2 - (static_cast<float>(size) / 2 + scale);

	global.hitmap.resize((size + 2 * scale)*(size + 2 * scale));
	global.source.resize(size*size, 3);
	global.direct.resize(size*size, 3);
	global.hit_matrix.resize(size*size, 3);

	const auto &project = [&viewer,&position, &center, &size, &global](const int p) {

		Eigen::Vector2f pos = position[p].cast<float>() + center;

		Eigen::Vector3f src, dir;

		std::vector<igl::Hit> hits;

		qrcode::unproject_onto_mesh(Eigen::Vector2f(pos(1), pos(0)), viewer.core.view*viewer.core.model, viewer.core.proj, viewer.core.viewport,
			global.model_vertices, global.model_facets, src, dir, hits);

		global.hitmap[p] = hits.front();

		int y = -position[p](0);
		int x = position[p](1);
		
		if (y >= 0 && y < size&&x >= 0 && x < size) {

			global.source.row(y*size + x) << src.transpose();
			global.direct.row(y*size + x) << dir.transpose();

			Eigen::Vector3d v0 = global.model_vertices.row(global.model_facets(hits[0].id, 0));
			Eigen::Vector3d v1 = global.model_vertices.row(global.model_facets(hits[0].id, 1));
			Eigen::Vector3d v2 = global.model_vertices.row(global.model_facets(hits[0].id, 2));
			Eigen::Vector3d v = v0*(1 - hits[0].u - hits[0].v) + v1*hits[0].u + v2*hits[0].v;

			global.hit_matrix.row(y*size + x) = v.transpose();
		}
	};
	igl::parallel_for((size+2*scale)*(size + 2 * scale), project, 1000);

	/*Eigen::MatrixXd V(4 * (size - 1)*(size - 1), 3);
	Eigen::MatrixXi F(2 * (size - 1)*(size - 1), 3);
	Eigen::MatrixXd C(2 * (size - 1)*(size - 1), 3);
	std::vector<Eigen::RowVector3i> A(4 * (size - 1)*(size - 1), Eigen::RowVector3i(-1, -1, -1));

	position.swap(std::vector<Eigen::Vector2i>());

	for (int y = 0; y < size - 1; y++) {
		for (int x = 0; x < size - 1; x++) {
			position.push_back(Eigen::Vector2i(y, x));
		}
	}

	const auto &build = [&Modules,&global, &position,&size,&scale, &V, &F,&C, &A](int p) {

		int y = position[p](0);
		int x = position[p](1);


		int a = 4 * p;
		int b = 4 * p + 1;
		int c = 4 * p + 2;
		int d = 4 * p + 3;

		V.row(a) << global.hit_matrix.row(y*size + x);
		V.row(b) << global.hit_matrix.row((y + 1)*size + x);
		V.row(c) << global.hit_matrix.row(y*size + x + 1);
		V.row(d) << global.hit_matrix.row((y + 1)*size + x + 1);

		F.row(2 * p) << a, b, c;
		F.row(2 * p + 1) << b, d, c;

		if (Modules(position[p](0), position[p](1)) == 1) {
			C.row(2 * p) << 0.0, 0.0, 0.0;
			C.row(2 * p + 1) << 0.0, 0.0, 0.0;
		}
		else if (Modules(position[p](0), position[p](1)) == 0) {
			C.row(2 * p) << 1.0, 1.0, 1.0;
			C.row(2 * p + 1) << 1.0, 1.0, 1.0;
		}

		//left (0, -1)
		if ((position[p](1) - 1) >= 0) {
			int index = position[p](0)*(size - 1) + position[p](1) - 1;
			//int e = 4 * index + 2;
			int f = 4 * index + 3;
			int g = a;
			int h = b;

			A[4 * p] = Eigen::RowVector3i(f, h, g);
		}

		//right (0, 1)
		if ((position[p](1) + 1) < (size - 1)) {
			int index = position[p](0)*(size - 1) + position[p](1) + 1;
			int e = c;
			int f = d;
			int g = 4 * index;
			//int h = 4 * index + 1;

			A[4 * p + 1] = Eigen::RowVector3i(e, f, g);
		}

		//up (-1,0)
		if ((position[p](0) - 1) >= 0) {
			int index = (position[p](0) - 1)*(size - 1) + position[p](1);
			//int e = 4 * index + 1;
			int f = a;
			int g = 4 * index + 3;
			int h = c;

			A[4 * p + 2] = Eigen::RowVector3i(f, h, g);
		}

		//down (1, 0)
		if ((position[p](0) + 1) < size - 1) {
			int index = (position[p](0) + 1)*(size - 1) + position[p](1);
			int e = b;
			int f = 4 * index;
			int g = d;
			//int h = 4 * index + 2;

			A[4 * p + 3] = Eigen::RowVector3i(e, f, g);
		}
	};

	igl::parallel_for((size - 1)*(size - 1), build, 1000);

	Eigen::MatrixXi FP(4 * (size - 1)*(size - 2), 3);

	int index = 0;
	for (int i = 0; i < A.size(); i++) {
		if (A[i](0) != -1 && A[i](1) != -1 && A[i](2) != -1) {
			FP.row(index) << A[i];
			index++;
		}
	}

	global.qr_verticals = V;
	global.qr_colors.setOnes(F.rows() + FP.rows(), 3);
	global.qr_colors.block(0, 0, C.rows(), 3) = C;
	global.qr_facets.resize(F.rows() + FP.rows(), 3);
	global.qr_facets.block(0, 0, F.rows(), 3) = F;
	global.qr_facets.block(F.rows(), 0, FP.rows(), 3) = FP;*/

}
