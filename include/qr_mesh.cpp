#include "qr_mesh.h"

void qrcode::generate_qr_mesh(Engine * engine, GLOBAL & global)
{
	int scale = global.info.scale;
	int size = (2 * global.info.border + global.info.pixels.size())*scale + 1;

	Eigen::MatrixXi modules, functions;
	pixel_to_matrix(global.info.pixels, global.info.border, modules, functions);

	Eigen::MatrixXi Modules;
	Modules.setZero(size, size);

	for (int y = 0; y < modules.rows() - 1; y++)
		for (int x = 0; x < modules.cols() - 1; x++)
			for (int u = 0; u < scale; u++)
				for (int v = 0; v < scale; v++)
					Modules(y*scale + u, x*scale + v) = modules(y, x);


	/*Find island*/
	Eigen::MatrixXi controller = global.under_control.block(scale, scale, size-1, size-1);
	Eigen::MatrixXi label;
	bwlabel(engine, controller, 4, label);
	global.anti_indicatior.clear();
	global.indicator.resize(size-1);

	global.patch_indicator.clear();
	global.patch_indicator.resize(size - 1);

	for (int y = 0; y < size - 1; y++) {
		global.indicator[y] = std::vector<Eigen::Vector2i>(size - 1, Eigen::Vector2i(0, -1));
		for (int x = 0; x < size - 1; x++) {
			if (label(y, x) > 0) {
				global.anti_indicatior.push_back(Eigen::Vector2i(y, x));
				global.indicator[y][x] = Eigen::Vector2i(label(y, x), global.anti_indicatior.size() - 1);
			}
		}
	}

	/*Build island*/
	Eigen::MatrixXd V(4 * global.anti_indicatior.size(), 3);
	Eigen::MatrixXi F(2 * global.anti_indicatior.size(), 3);
	Eigen::MatrixXd C(2 * global.anti_indicatior.size(), 3);
	std::vector<Eigen::RowVector3i> A;
	std::vector<std::vector<Eigen::RowVector2i>> island_mesh(label.maxCoeff());

	global.seeds.resize(label.maxCoeff(),-1);

	const auto &build = [&Modules, &global,&label, &island_mesh,&size, &scale, &V, &F, &C, &A](int p) {

		int y = global.anti_indicatior[p](0);
		int x = global.anti_indicatior[p](1);
		int flag = label(y, x);
		bool inner = true;

		int a = 4 * p;
		int b = 4 * p + 1;
		int c = 4 * p + 2;
		int d = 4 * p + 3;

		V.row(a) << global.hit_matrix.row(y*size + x);
		V.row(b) << global.hit_matrix.row((y + 1)*size + x);
		V.row(c) << global.hit_matrix.row(y*size + x + 1) ;
		V.row(d) << global.hit_matrix.row((y + 1)*size + x + 1);

		F.row(2 * p) << a, b, c;
		F.row(2 * p + 1) << b, d, c;


		if (Modules(y, x) == 1) {
			C.row(2 * p) << 0.0, 0.0, 0.0;
			C.row(2 * p + 1) << 0.0, 0.0, 0.0;
		}
		else if (Modules(y,x) == 0) {
			C.row(2 * p) << 1.0, 1.0, 1.0;
			C.row(2 * p + 1) << 1.0, 1.0, 1.0;
		}

		//left (0,-1)
		if ((x - 1) >= 0) {
			if (label(y, x - 1) == flag) {
				int index = global.indicator[y][x - 1](1);

				//int e = 4 * index + 2;
				int f = 4 * index + 3;
				int g = a;
				int h = b;

				A.push_back(Eigen::RowVector3i(f, h, g));
				global.patch_indicator[p](0) = A.size() - 1;
			}
			else {
				inner = false;
				island_mesh[label(y, x) - 1].push_back(Eigen::RowVector2i(a, b));
				global.patch_indicator[p](0) =-1;
			}
		}
		else {
			inner = false;
			island_mesh[label(y, x) - 1].push_back(Eigen::RowVector2i(a, b));
			global.patch_indicator[p](0) = -1;
		}

		//right (0, 1)
		if ((x + 1) < (size - 1)) {
			if (label(y, x + 1) == flag) {
				int index = global.indicator[y][x + 1](1);
				int e = c;
				int f = d;
				int g = 4 * index;
				//int h = 4 * index + 1;

				A.push_back(Eigen::RowVector3i(e, f, g));
				global.patch_indicator[p](1) = A.size() - 1;
			}
			else {
				inner = false;
				island_mesh[label(y, x) - 1].push_back(Eigen::RowVector2i(d,c));
				global.patch_indicator[p](1) = -1;
			}
		}
		else {
			inner = false;
			island_mesh[label(y, x) - 1].push_back(Eigen::RowVector2i(d, c));
			global.patch_indicator[p](1) =-1;
		}

		//up (-1, 0)
		if ((y - 1) >= 0) {
			if (label(y - 1, x) == flag) {
				int index = global.indicator[y - 1][x](1);
				//int e = 4 * index + 1;
				int f = a;
				int g = 4 * index + 3;
				int h = c;

				A.push_back(Eigen::RowVector3i(f, h, g));
				global.patch_indicator[p](2) = A.size() - 1;
			}
			else {
				inner = false;
				island_mesh[label(y, x) - 1].push_back(Eigen::RowVector2i(c,a));
				global.patch_indicator[p](2) = -1;
			}
		}
		else {
			inner = false;
			island_mesh[label(y, x) - 1].push_back(Eigen::RowVector2i(c, a));
			global.patch_indicator[p](2) = -1;
		}

		//down (1, 0)
		if ((y + 1) < (size - 1)) {
			if (label(y + 1, x) == flag) {
				int index = global.indicator[y + 1][x](1);
				int e = b;
				int f = 4 * index;
				int g = d;
				//int h = 4 * index + 2;

				A.push_back(Eigen::RowVector3i(e, f, g));
				global.patch_indicator[p](3) = A.size() - 1;
			}
			else {
				inner = false;
				island_mesh[label(y, x) - 1].push_back(Eigen::RowVector2i(b,d));
				global.patch_indicator[p](3) = -1;
			}
		}
		else {
			inner = false;
			island_mesh[label(y, x) - 1].push_back(Eigen::RowVector2i(b, d));
			global.patch_indicator[p](3) = -1;
		}

		if ((global.seeds[flag - 1] == -1)) {
			if (((x - 1) >= 0) && ((x + 1) < (size - 1)) && ((y - 1) >= 0) && ((y + 1) < (size - 1))) {
				if (label(x - 1, y - 1) != flag)
					inner=false;
				if (label(x - 1, y + 1) != flag)
					inner= false;
				if (label(x + 1, y - 1) != flag)
					inner= false;
				if (label(x + 1, y + 1) != flag)
					inner= false;
			}
			else {
				inner = false;
			}

			if (inner)
				global.seeds[flag - 1] = a;
		}
	};

	


	for (int i = 0; i < global.anti_indicatior.size(); i++) build(i);

	Eigen::MatrixXi FP(A.size(), 3);
	for(int i=0;i<A.size();i++) FP.row(i) << A[i];

	global.qr_verticals = V;
	global.qr_colors.setOnes(F.rows() + FP.rows(), 3);
	global.qr_colors.block(0, 0, C.rows(), 3) = C;
	global.qr_facets.resize(F.rows() + FP.rows(), 3);
	global.qr_facets.block(0, 0, F.rows(), 3) = F;
	global.qr_facets.block(F.rows(), 0, FP.rows(), 3) = FP;

	/*Extract island boundary*/
	global.islands.resize(label.maxCoeff());
	for (int i = 0; i < island_mesh.size(); i++) {
		global.islands[i].resize(island_mesh[i].size(), 2);
		for (int j = 0; j < island_mesh[i].size(); j++) {
			global.islands[i].row(j) << island_mesh[i][j];
		}
	}
}
