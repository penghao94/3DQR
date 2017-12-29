#include "carving_down.h"

void qrcode::carving_down(GLOBAL & global, Eigen::MatrixXd & result)
{
	result.resize(global.qr_verticals.rows(), 3);
	int size = (global.info.pixels.size() + 2 * global.info.border)*global.info.scale;
	for (int i = 0; i < global.qr_verticals.rows(); i++) {
		int index = i / 4;
		int quot = i % 4;
		int y = global.anti_indicatior[index](0);
		int x = global.anti_indicatior[index](1);
		int u = quot % 2;
		int v = quot / 2;
		result.row(i) = global.qr_verticals.row(i) + global.carve_depth(i)*global.direct.row((y + u)*(size+1) + x + v).cast<double>();
	}
}

void qrcode::patch(int y, int x, GLOBAL & global,Eigen::Vector4d &patch)
{
	std::cout << global.carve_depth.rows() << std::endl;
	int index = global.indicator[y][x](1);
	std::cout << index << std::endl;
	global.carve_depth(4 * index) = patch(0);
	global.carve_depth(4 * index + 1) = patch(1);
	global.carve_depth(4 * index + 2) = patch(2);
	global.carve_depth(4 * index + 3) = patch(3);
}

void qrcode::patch(int y, int x, Eigen::VectorXf depth, Eigen::MatrixXi &modules,GLOBAL & global)
{
	int index_t = global.indicator[y][x](1);
	int index_t_1 = global.indicator[y][x - 1](1);

	global.carve_depth(4 * index_t) = global.carve_depth(4 * index_t_1 + 2);
	global.carve_depth(4 * index_t + 1) = global.carve_depth(4 * index_t_1 + 3);

	if (modules(y, x + 1) == 1) {
		global.carve_depth(4 * index_t + 2) = (depth(index_t) + depth(global.indicator[y][x + 1](1))) / 2;
		global.carve_depth(4 * index_t + 3) = (depth(index_t) + depth(global.indicator[y][x + 1](1))) / 2;
	}
	else {
		global.carve_depth(4 * index_t + 2) = 2 * depth(index_t) - global.carve_depth(4 * index_t);
		global.carve_depth(4 * index_t + 3) = 2 * depth(index_t) - global.carve_depth(4 * index_t+1);
	}
}
