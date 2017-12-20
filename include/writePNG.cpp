#include "writePNG.h"
void qrcode::write_png(std::string file, Eigen::MatrixXi & modules, int scale)
{
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
	R.resize((modules.rows())*scale, (modules.cols())*scale);
	G.resize((modules.rows())*scale, (modules.cols())*scale);
	B.resize((modules.rows())*scale, (modules.cols())*scale);
	A.resize((modules.rows())*scale, (modules.cols())*scale);
	for (int i = 0; i < modules.rows(); i++) {
		for (int j = 0; j < modules.cols(); j++) {
			for (int x = 0; x < scale; x++) {
				for (int y = 0; y < scale; y++) {
					if (modules(modules.cols() - 1 - j, i) == 0) {
						R(i*scale + x, j*scale + y) = 255;
						G(i*scale + x, j*scale + y) = 255;
						B(i*scale + x, j*scale + y) = 255;
					}
					else {
						R(i*scale + x, j*scale + y) = 0;
						G(i*scale + x, j*scale + y) = 0;
						B(i*scale + x, j*scale + y) = 0;
					}
					A(i*scale + x, j*scale + y) = 255;
				}
			}
		}
	}
	igl::png::writePNG(R, G, B, A, file);
}

void qrcode::write_png(std::string file, Eigen::MatrixXi & modules,Eigen::MatrixXi &modules_c,int scale)
{
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
	R.resize((modules.rows())*scale, (modules.cols())*scale);
	G.resize((modules.rows())*scale, (modules.cols())*scale);
	B.resize((modules.rows())*scale, (modules.cols())*scale);
	A.resize((modules.rows())*scale, (modules.cols())*scale);
	for (int i = 0; i < modules.rows(); i++) {
		for (int j = 0; j < modules.cols(); j++) {
			for (int x = 0; x < scale; x++) {
				for (int y = 0; y < scale; y++) {
					if (modules(modules.cols() - 1 - j, i) == 0) {
						R(i*scale + x, j*scale + y) = 255;
						G(i*scale + x, j*scale + y) = 255;
						B(i*scale + x, j*scale + y) = 255;
					}
					else if(modules(modules.cols() - 1 - j, i) == 1&& modules_c(modules.cols() - 1 - j, i) == 0){
						R(i*scale + x, j*scale + y) = 255;
						G(i*scale + x, j*scale + y) = 0;
						B(i*scale + x, j*scale + y) = 0;
					}
					else if (modules(modules.cols() - 1 - j, i) == 1 && modules_c(modules.cols() - 1 - j, i) == 1) {
						R(i*scale + x, j*scale + y) = 0;
						G(i*scale + x, j*scale + y) = 0;
						B(i*scale + x, j*scale + y) = 0;
					}
					A(i*scale + x, j*scale + y) = 255;
				}
			}
		}
	}
	igl::png::writePNG(R, G, B, A, file);
}

void qrcode::write_png1(std::string file, Eigen::MatrixXi & modules, Eigen::MatrixXi &modules_c, int scale)
{
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
	R.resize((modules.rows())*scale, (modules.cols())*scale);
	G.resize((modules.rows())*scale, (modules.cols())*scale);
	B.resize((modules.rows())*scale, (modules.cols())*scale);
	A.resize((modules.rows())*scale, (modules.cols())*scale);
	for (int i = 0; i < modules.rows(); i++) {
		for (int j = 0; j < modules.cols(); j++) {
			for (int x = 0; x < scale; x++) {
				for (int y = 0; y < scale; y++) {
					if (modules(modules.cols() - 1 - j, i) == 0 && modules_c(modules.cols() - 1 - j, i) == 0) {
						R(i*scale + x, j*scale + y) = 255;
						G(i*scale + x, j*scale + y) = 255;
						B(i*scale + x, j*scale + y) = 255;
					}
					else if (modules(modules.cols() - 1 - j, i) == 0 && modules_c(modules.cols() - 1 - j, i) == 1) {
						R(i*scale + x, j*scale + y) = 0;
						G(i*scale + x, j*scale + y) = 255;
						B(i*scale + x, j*scale + y) = 0;
					}
					else if (modules(modules.cols() - 1 - j, i) == 1) {
						R(i*scale + x, j*scale + y) = 0;
						G(i*scale + x, j*scale + y) = 0;
						B(i*scale + x, j*scale + y) = 0;
					}
					A(i*scale + x, j*scale + y) = 255;
				}
			}
		}
	}
	igl::png::writePNG(R, G, B, A, file);
}

void qrcode::write_png(std::string file, Eigen::MatrixXi & modules)
{
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
	R.resize(modules.rows(), modules.cols());
	G.resize(modules.rows(), modules.cols());
	B.resize(modules.rows(), modules.cols());
	A.resize(modules.rows(), modules.cols());
	for (int y = 0; y < modules.rows();y++) {
		for (int x = 0; x < modules.cols(); x++) {
			R(y, x) = static_cast<unsigned char>(modules(modules.rows() - 1 - x, y));
			G(y, x) = static_cast<unsigned char>(modules(modules.rows() - 1 - x, y));
			B(y, x) = static_cast<unsigned char>(modules(modules.rows() - 1 - x, y));
			A(y, x) = 255;
		}
	}
	igl::png::writePNG(R, G, B, A, file);
}
