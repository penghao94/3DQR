#include "pixel_to_matrix.h"

void qrcode::pixel_to_matrix(std::vector<std::vector<qrgen::Pixel>>& pixels, int border, Eigen::MatrixXi & Modules, Eigen::MatrixXi & Functions)
{
	Modules.setZero(2 * border + pixels.size() + 1, 2 * border + pixels.size() + 1);
	Functions.setZero(2 * border + pixels.size() + 1, 2 * border + pixels.size() + 1);

	for (int y = 0; y < pixels.size(); y++) {
		for (int x = 0; x < pixels.size(); x++) {
			Modules(y + border, x + border) = pixels[y][x].getPixel();
			qrgen::Pixel::PixelRole role = pixels[y][x].getPixelRole();
			if (role != qrgen::Pixel::DATA&&role != qrgen::Pixel::CHECK&&role != qrgen::Pixel::EXTRA&&pixels[y][x].getPixel() == 1)
				Functions(y + border, x + border) = 1;
		}
	}
}
