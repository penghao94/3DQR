#include "pre_pixel_normal.h"

void qrcode::pre_pixel_normal(GLOBAL &global,Eigen::MatrixXd &qr_verticals, Eigen::MatrixXf &position, Eigen::MatrixXf &normal)
{
	int size = global.anti_indicatior.size();

	position.resize(size, 3);
	normal.resize(size, 3);

	for (int i = 0; i < size; i++) {

 		Eigen::Vector3f a = qr_verticals.row(4 * i).cast<float>();
		Eigen::Vector3f b = qr_verticals.row(4 * i + 1).cast<float>();
		Eigen::Vector3f c = qr_verticals.row(4 * i + 2).cast<float>();
		Eigen::Vector3f d = qr_verticals.row(4 * i + 3).cast<float>();

		Eigen::Vector3f n1 = (b - a).cross(c - b);
		Eigen::Vector3f n2 = (d - b).cross(c - d);

		position.row(i) = ( b + c) / 2;
		normal.row(i) = ((n1 + n2) / 2).normalized();
	}
}
