#include "module_adapter.h"
typedef qrgen::Pixel::PixelRole PR;
std::vector<Eigen::MatrixXi> qrcode::module_adapter(Engine * engine, GLOBAL & global)
{
	/*Origin modules*/
	auto pixels = global.info.pixels;

	int border = global.info.border; 

	int size = 2 * border + global.info.pixels.size();

	int scale = global.info.scale;

	Eigen::MatrixXi controller = global.under_control.block(scale,scale, size*scale, size*scale);

	Eigen::MatrixXi modules; 

	modules.setZero(size, size);

	for (int y = 0; y < pixels.size(); y++) {
		for (int x = 0; x < pixels.size(); x++) {
			modules(y + border, x + border) = pixels[y][x].getPixel() && controller((y + border)*scale, (x + border)*scale);
		}
	}

	qrcode::write_png("qrcode_origin.png", modules,scale);

	/*Decrease long adjacent region to upper_modules*/

	Eigen::MatrixXi upper_modules = modules;
	Eigen::MatrixXi region = upper_modules.block(border, border, global.info.pixels.size(), global.info.pixels.size());


	Eigen::MatrixXi label;
	qrcode::bwlabel(engine, region, 4, label);

	Eigen::MatrixXi t = label; 
	label.setZero(label.rows(), label.cols() + 1);
	label.block(0, 0, label.rows(), label.rows()) = t; 

	std::vector<Eigen::Vector3i> segments;
	for (int y = 0; y < label.rows(); y++) {
		int count = 0;
		int temp = 0;
		Eigen::Vector2i index;

		for (int x = 0; x < label.cols(); x++) {

			if (label(y, x) != temp&&label(y, x) == 0) {
				temp = 0;
				segments.emplace_back(index(0), index(1), count);
				continue;
			}
			 
			if (label(y, x) != temp&&label(y, x) != 0) {
				temp = label(y, x);
				index << y, x;
				count = 1;
				continue;
			}
			if (label(y, x) == temp&&label(y, x) != 0) {
				count++;
			}

		}
	}

	std:sort(segments.begin(), segments.end(), [](Eigen::Vector3i seg1, Eigen::Vector3i seg2) {return seg1(2) > seg2(2); });

	int down_size = 3;
	int length = 177,index=0;

	bool stop_iter = false;

	Eigen::MatrixXi modefier;
	modefier.setOnes(upper_modules.rows(), upper_modules.cols());


	while (!stop_iter) {

		length = segments[index](2);
		if (length <= 3) {
			break;
		}
		else {
			int dx;
			if (length >= 6)
				dx = 3;
			else
				dx = length / 2;

			qrgen::Pixel pixel = pixels[segments[index](0)][segments[index](1) + dx];

			if (!(pixel.getPixelRole() ==PR::DATA || pixel.getPixelRole() == PR::CHECK || pixel.getPixelRole() == PR::EXTRA)) {

				pixel = pixels[segments[index](0)][segments[index](1) + dx-1];

				if (pixel.getPixelRole() == PR::DATA || pixel.getPixelRole() == PR::CHECK || pixel.getPixelRole() == PR::EXTRA) {

					if (global.info.codewodrs[pixel.getOffset()/8].getStatus()) {

						upper_modules(segments[index](0) + border, segments[index](1) + dx-1 + border) = 0;
						modefier(segments[index](0) + border, segments[index](1) + dx - 1 + border) = 0;

						if (length - dx > 3) {
							auto pos = std::lower_bound(segments.begin(), segments.end(), Eigen::Vector3i(0, 0, length - 3), [](Eigen::Vector3i seg1, Eigen::Vector3i seg2) {return seg1(2) > seg2(2); });
							segments.insert(pos, Eigen::Vector3i(segments[index](0), segments[index](1) + dx, length - dx));
						}		
					}
					else if (global.info.block_propertys[pixel.getBlockIndex()] > 0) {

						global.info.codewodrs[pixel.getOffset()/8].setTrue();
						upper_modules(segments[index](0) + border, segments[index](1) + dx-1 + border) = 0;
						modefier(segments[index](0) + border, segments[index](1) + dx - 1 + border) = 0;
						global.info.block_propertys[pixel.getBlockIndex()] -= 1;

						if (length - 3 > 3) {
							auto pos = std::lower_bound(segments.begin(), segments.end(), Eigen::Vector3i(0, 0, length - 3), [](Eigen::Vector3i seg1, Eigen::Vector3i seg2) {return seg1(2) > seg2(2); });
							segments.insert(pos, Eigen::Vector3i(segments[index](0), segments[index](1) + 3, length - 3));
						}
					}
				}
				else {
					for (dx = dx + 1; dx <= length; dx++) {

						pixel = pixels[segments[index](0)][segments[index](1) + dx];

						if (pixel.getPixelRole() == PR::DATA || pixel.getPixelRole() == PR::CHECK || pixel.getPixelRole() == PR::EXTRA) {

							if (global.info.codewodrs[pixel.getOffset()/8].getStatus()) {

								upper_modules(segments[index](0) + border, segments[index](1) + dx + border) = 0;
								modefier(segments[index](0) + border, segments[index](1) + dx + border) = 0;

								if (length - dx-1 > 3) {
									auto pos = std::lower_bound(segments.begin(), segments.end(), Eigen::Vector3i(0, 0, length - dx - 1), [](Eigen::Vector3i seg1, Eigen::Vector3i seg2) {return seg1(2) > seg2(2); });
									segments.insert(pos, Eigen::Vector3i(segments[index](0), segments[index](1) + dx + 1, length - dx - 1));
								}
							}
							else if (global.info.block_propertys[pixel.getBlockIndex()] > 0) {

								global.info.codewodrs[pixel.getOffset()/8].setTrue();
								upper_modules(segments[index](0) + border, segments[index](1) + dx + border) = 0;
								modefier(segments[index](0) + border, segments[index](1) + dx + border) = 0;
								
								global.info.block_propertys[pixel.getBlockIndex()] -= 1;

								if (length - dx - 1 > 3) {
									auto pos = std::lower_bound(segments.begin(), segments.end(), Eigen::Vector3i(0, 0, length - dx - 1), [](Eigen::Vector3i seg1, Eigen::Vector3i seg2) {return seg1(2) > seg2(2); });
									segments.insert(pos, Eigen::Vector3i(segments[index](0), segments[index](1) + dx + 1, length - dx - 1));
								}
							}
							break;
						}
					}
				}

			}
			else {
				pixel = pixels[segments[index](0)][segments[index](1) + dx];

				if (pixel.getPixelRole() == PR::DATA || pixel.getPixelRole() == PR::CHECK || pixel.getPixelRole() == PR::EXTRA) {

					if (global.info.codewodrs[pixel.getOffset()/8].getStatus()) {

						upper_modules(segments[index](0) + border, segments[index](1) + dx + border) = 0;
						modefier(segments[index](0) + border, segments[index](1) + dx + border) = 0;
						
						if (length - dx - 1 > 3) {
							auto pos = std::lower_bound(segments.begin(), segments.end(), Eigen::Vector3i(0, 0, length - dx - 1), [](Eigen::Vector3i seg1, Eigen::Vector3i seg2) {return seg1(2) > seg2(2); });
							segments.insert(pos, Eigen::Vector3i(segments[index](0), segments[index](1) + dx + 1, length - dx - 1));
						}
					}
					else if (global.info.block_propertys[pixel.getBlockIndex()] > 0) {

						global.info.codewodrs[pixel.getOffset()/8].setTrue();
						upper_modules(segments[index](0) + border, segments[index](1) + dx + border) = 0;
						modefier(segments[index](0) + border, segments[index](1) + dx + border) = 0;

						if (length - dx - 1 > 3) {
							auto pos = std::lower_bound(segments.begin(), segments.end(), Eigen::Vector3i(0, 0, length - dx - 1), [](Eigen::Vector3i seg1, Eigen::Vector3i seg2) {return seg1(2) > seg2(2); });
							segments.insert(pos, Eigen::Vector3i(segments[index](0), segments[index](1) + dx + 1, length - dx - 1));
						}
					}
				}
			}
		}

		bool all_use = true;
		for (int i = 0; i < global.info.block_propertys.size();i++) {
			if (global.info.block_propertys[i] > 0) all_use &= false;
		}
		stop_iter |= all_use;

		index++;
	}

	qrcode::write_png("qrcode_opt.png", modules, upper_modules, scale);
	qrcode::write_png("qrcode_upper.png", upper_modules, scale);

	/*Increase black modules to lower modules*/

	int version = (pixels.size() - 17) / 4;
	int remainder = global.info.num_of_check_byte/2 - global.info.block_propertys[0]+4;

	Eigen::MatrixXi lower_modules;
	lower_modules.setZero(modules.rows(), modules.cols());
	for (int y = 0; y < pixels.size(); y++) {
		for (int x = 0; x < pixels.size(); x++) {
			if (upper_modules(y + border, x + border) == 0 && upper_modules(y + border, x + border + 1) == 0 && upper_modules(y + border, x + border - 1) == 0) {
				if (lower_modules(y + border, x + border + 1) == 0 && lower_modules(y + border, x + border - 1) == 0 && modefier(y + border, x + border + 1) == 1 && modefier(y + border, x + border - 1) == 1) {
					if (pixels[y][x].getPixelRole() == PR::DATA || pixels[y][x].getPixelRole() == PR::CHECK || pixels[y][x].getPixelRole() == PR::EXTRA) {
						if (pixels[y][x].getBlockIndex() == 0 && global.info.codewodrs[pixels[y][x].getOffset() / 8].getStatus() == false) {
							lower_modules(y + border, x + border) = 1;
							remainder--;
							global.info.codewodrs[pixels[y][x].getOffset() / 8].setTrue();
							if (remainder <= 0)
								break;
						}
					}

				}
			}

		}
		if (remainder <= 0)
			break;
	}
	
	qrcode::write_png1("qrcode_lower_append.png", upper_modules, lower_modules, scale);

	/*Find adjacent black modules*/
	Eigen::MatrixXi both_modules = upper_modules + lower_modules;

	qrcode::write_png("qrcode_lower.png",both_modules,scale);

	

	region = both_modules.block(border, border, global.info.pixels.size(), global.info.pixels.size());

	qrcode::bwlabel(engine, region, 4, label);

	Eigen::MatrixXi tmp = label;

	label.setZero(label.rows(), label.cols() + 1);
	
	label.block(0, 0, label.rows(), label.cols()-1) = tmp;
	std::cout << label << std::endl;

	global.black_module_segments.clear();

	for (int y = 0; y < label.rows(); y++) {
		int count = 0;
		int temp = 0;
		Eigen::Vector2i index;

		for (int x = 0; x < label.cols(); x++) {

			if (label(y, x) != temp && label(y, x) == 0) {
				temp = 0;
				global.black_module_segments.emplace_back(index(0), index(1), count);
				continue;
			}

			if (label(y, x) != temp&&label(y, x) != 0) {
				temp = label(y, x);
				index << y, x;
				count = 1;
				continue;
			}
			if (label(y, x) == temp&&label(y, x) != 0) {
				count++;
			}

		}
	}

	for (auto s : global.black_module_segments) std::cout << s.transpose() << std::endl;
	Eigen::MatrixXi upper_Modules, lower_Modules;

	upper_Modules.setZero(controller.rows() + 1, controller.cols() + 1);
	lower_Modules.setZero(controller.rows() + 1, controller.cols() + 1);

	for (int y = 0; y < modules.rows(); y++) {
		for (int x = 0; x < modules.cols(); x++) {
			for (int u = 0; u < scale; u++) {
				for (int v = 0; v < scale; v++) {
					upper_Modules(y*scale + u, x*scale + v) = upper_modules(y, x);
					lower_Modules(y*scale + u, x*scale + v) = lower_modules(y, x);
				}	
			}	
		}	
	}	
	return{ upper_Modules,lower_Modules};
}
