#include "reflaction.h"

void qrcode::reflaction(GLOBAL & global, Eigen::MatrixXd & verticles, Eigen::MatrixXi & facets)
{

	std::string binary_file = "reflaction";

	Eigen::MatrixXi black_module_seg;
	
	igl::deserialize(black_module_seg, "black_module_seg", binary_file);
	
	global.black_module_segments.clear();
	for (int i = 0; i < black_module_seg.rows(); i++) global.black_module_segments.push_back(black_module_seg.row(i).transpose());

	std::cout << global.black_module_segments.size() << std::endl;
	Eigen::MatrixXi modules;
	igl::deserialize(modules, "modules", binary_file);
	int qr_size;
	igl::deserialize(qr_size, "qr_size", binary_file);
	int scale;
	igl::deserialize(scale, "scale", binary_file);
	int border;
	igl::deserialize(border, "border", binary_file);
	Eigen::MatrixXd qr_verticals;
	igl::deserialize(qr_verticals, "qr_verticals", binary_file);
	igl::deserialize(verticles, "verticles", binary_file);
	igl::deserialize(facets, "facets", binary_file);


	std::vector<qrcode::SMesh> appendix;

	std::vector<std::vector<Eigen::Matrix<int, 4, 3>>> mesh_info(qr_size);
	std::vector<std::vector<Eigen::Vector3i>>mesh_status(qr_size);
	Eigen::Matrix<int, 4, 3> info;
	info.setConstant(-1);

	for (int i = 0; i < qr_size; i++) {
		mesh_info[i].resize(qr_size, info);
		mesh_status[i].resize(qr_size, Eigen::Vector3i(0, 0, 0));
	}


	for (int y = 0; y < qr_size; y++)
	{
		for (int x = 0; x < qr_size; x++) {

			mesh_info[y][x](0, 0) = 4 * global.indicator[y][x](1);
			mesh_info[y][x](1, 0) = 4 * global.indicator[y][x](1) + 1;
			mesh_info[y][x](2, 0) = 4 * global.indicator[y][x](1) + 2;
			mesh_info[y][x](3, 0) = 4 * global.indicator[y][x](1) + 3;
		}
	}
	std::cout << "ok2" << std::endl;
	int row = 0;
	for (int i = 0; i < global.black_module_segments.size(); i++) {
		std::cout << "i" << std::endl;

		Eigen::Vector3i segment = global.black_module_segments[i];
		int y = segment(0);
		int x = segment(1);
		int length = segment(2);

		if (length > 1) {

			int temp_len = length;

			if ((x + length) == global.info.pixels.size()) temp_len = 4;

			Eigen::MatrixXd append_verticals(8 * scale*scale*temp_len, 3);
			std::vector<Eigen::Vector3i> append_facets;

			for (int u = 0; u < scale; u++) {

				int lower_index = global.indicator[(y + border)*scale + u][(x + length - 1 + border)*scale + scale - 1](1);
				int upper_index = global.indicator[(y + border)*scale + u][(x + length + border)*scale](1);

				double upper_left_point = qr_verticals(4 * upper_index, 2);
				double upper_right_point = qr_verticals(4 * upper_index + 1, 2);

				double lower_left_point = qr_verticals(4 * lower_index + 2, 2);
				double lower_right_point = qr_verticals(4 * lower_index + 3, 2);

				int seg_size = scale*temp_len;

				double diff_left = (lower_left_point - upper_left_point) / seg_size;
				double diff_right = (lower_right_point - upper_right_point) / seg_size;
				std::cout << "ok3" << std::endl;

				for (int v = 0; v < seg_size; v++) {
					int index_seg = u*seg_size + v;
					int index_seg_1 = (u + 1)*seg_size + v;

					int r = (y + border)*scale + u;
					int c = (x + length + border)*scale + v;

					//append_verticals.row(4 * index_seg) << global.hit_matrix(r*(qr_size + 1) + c,0), global.hit_matrix(r*(qr_size + 1) + c, 1),lower_left_point;
					//append_verticals.row(4 * index_seg + 1) <<global.hit_matrix((r + 1)*(qr_size + 1) + c,0), global.hit_matrix((r + 1)*(qr_size + 1) + c, 1),lower_right_point;
					//append_verticals.row(4 * index_seg + 2) <<global.hit_matrix(r*(qr_size + 1) + c + 1,0), global.hit_matrix(r*(qr_size + 1) + c + 1, 1), lower_left_point;
					//append_verticals.row(4 * index_seg + 3)<< global.hit_matrix((r + 1)*(qr_size + 1) + c + 1,0), global.hit_matrix((r + 1)*(qr_size + 1) + c + 1, 1),lower_right_point;

					append_verticals.row(4 * index_seg) =
						global.hit_matrix.row(r*(qr_size + 1) + c) + global.direct.row(r*(qr_size + 1) + c).cast<double>()*(lower_left_point - upper_left_point) / global.direct(r*(qr_size + 1) + c, 2);
					append_verticals.row(4 * index_seg + 1) =
						global.hit_matrix.row((r + 1)*(qr_size + 1) + c) + global.direct.row((r + 1)*(qr_size + 1) + c).cast<double>()*(lower_right_point - upper_right_point) / global.direct((r + 1)*(qr_size + 1) + c, 2);
					append_verticals.row(4 * index_seg + 2) =
						global.hit_matrix.row(r*(qr_size + 1) + c + 1) + global.direct.row(r*(qr_size + 1) + c + 1).cast<double>()*(lower_left_point - upper_left_point) / global.direct(r*(qr_size + 1) + c + 1, 2);
					append_verticals.row(4 * index_seg + 3) =
						global.hit_matrix.row((r + 1)*(qr_size + 1) + c + 1) + global.direct.row((r + 1)*(qr_size + 1) + c + 1).cast<double>()*(lower_right_point - upper_right_point) / global.direct((r + 1)*(qr_size + 1) + c + 1, 2);


					append_facets.emplace_back(4 * index_seg, 4 * index_seg + 1, 4 * index_seg + 2);
					append_facets.emplace_back(4 * index_seg + 1, 4 * index_seg + 3, 4 * index_seg + 2);





					/*append_verticals.row(4 * scale*seg_size + 4 * index_seg) <<
					global.hit_matrix(r*(qr_size + 1) + c, 0),
					global.hit_matrix(r*(qr_size + 1) + c, 1),
					upper_left_point + diff_right*v;
					append_verticals.row(4 * scale*seg_size + 4 * index_seg + 1) <<
					global.hit_matrix((r + 1)*(qr_size + 1) + c, 0),
					global.hit_matrix((r + 1)*(qr_size + 1) + c, 1),
					upper_right_point + diff_right*v;
					append_verticals.row(4 * scale*seg_size + 4 * index_seg + 2) <<
					global.hit_matrix(r*(qr_size + 1) + c + 1, 0),
					global.hit_matrix(r*(qr_size + 1) + c + 1, 1),
					upper_left_point + diff_left*(v+1);
					append_verticals.row(4 * scale*seg_size + 4 * index_seg + 3) <<
					global.hit_matrix((r + 1)*(qr_size + 1) + c + 1, 0),
					global.hit_matrix((r + 1)*(qr_size + 1) + c + 1, 1),
					upper_right_point + diff_right*(v+1);*/


					append_verticals.row(4 * scale*seg_size + 4 * index_seg) =
						global.hit_matrix.row(r*(qr_size + 1) + c) + global.direct.row(r*(qr_size + 1) + c).cast<double>()* v *diff_left / global.direct(r*(qr_size + 1) + c, 2);
					append_verticals.row(4 * scale*seg_size + 4 * index_seg + 1) =
						global.hit_matrix.row((r + 1)*(qr_size + 1) + c) + global.direct.row((r + 1)*(qr_size + 1) + c).cast<double>()* v *diff_right / global.direct((r + 1)*(qr_size + 1) + c, 2);
					append_verticals.row(4 * scale*seg_size + 4 * index_seg + 2) =
						global.hit_matrix.row(r*(qr_size + 1) + c + 1) + global.direct.row(r*(qr_size + 1) + c + 1).cast<double>()* (v + 1) *diff_left / global.direct(r*(qr_size + 1) + c + 1, 2);
					append_verticals.row(4 * scale*seg_size + 4 * index_seg + 3) =
						global.hit_matrix.row((r + 1)*(qr_size + 1) + c + 1) + global.direct.row((r + 1)*(qr_size + 1) + c + 1).cast<double>()* (v + 1) *diff_right / global.direct((r + 1)*(qr_size + 1) + c + 1, 2);


					append_facets.emplace_back(4 * scale*seg_size + 4 * index_seg, 4 * scale*seg_size + 4 * index_seg + 2, 4 * scale*seg_size + 4 * index_seg + 1);
					append_facets.emplace_back(4 * scale*seg_size + 4 * index_seg + 1, 4 * scale*seg_size + 4 * index_seg + 2, 4 * scale*seg_size + 4 * index_seg + 3);




					if (u == 0) {
						facets.row(global.anti_indicatior.size() * 2 + global.patch_indicator[(r - 1)*qr_size + c](3)) << 0, 0, 0;
						facets.row(global.anti_indicatior.size() * 2 + global.patch_indicator[r*qr_size + c](2)) << 0, 0, 0;

						mesh_info[r][c](0, 2) = verticles.rows() + row + 4 * index_seg;
						mesh_info[r][c](2, 2) = verticles.rows() + row + 4 * index_seg + 2;
						mesh_info[r][c](0, 1) = verticles.rows() + row + 4 * scale*seg_size + 4 * index_seg;
						mesh_info[r][c](2, 1) = verticles.rows() + row + 4 * scale*seg_size + 4 * index_seg + 2;

					}


					if (u == (scale - 1)) {
						facets.row(global.anti_indicatior.size() * 2 + global.patch_indicator[r*qr_size + c](3)) << 0, 0, 0;
						facets.row(global.anti_indicatior.size() * 2 + global.patch_indicator[(r + 1)*qr_size + c](2)) << 0, 0, 0;

						mesh_info[r][c](1, 2) = verticles.rows() + row + 4 * index_seg + 1;
						mesh_info[r][c](3, 2) = verticles.rows() + row + 4 * index_seg + 3;
						mesh_info[r][c](1, 1) = verticles.rows() + row + 4 * scale*seg_size + 4 * index_seg + 1;
						mesh_info[r][c](3, 1) = verticles.rows() + row + 4 * scale*seg_size + 4 * index_seg + 3;


					}
					if (v == 0) {
						facets.row(global.anti_indicatior.size() * 2 + global.patch_indicator[r*qr_size + c](0)) << 0, 0, 0;
						facets.row(global.anti_indicatior.size() * 2 + global.patch_indicator[r*qr_size + c - 1](1)) << 0, 0, 0;
					}

				}
			}
			std::cout << append_verticals.rows() << std::endl;
			std::cout << "ok4" << std::endl;

			for (int u = 0; u < scale-1; u++) {
				int seg_size = scale*temp_len;

				for (int v = 0; v < seg_size; v++) {
					int index_seg = u*seg_size + v;
					int index_seg_1 = (u + 1)*seg_size + v;
					int scope_index = 4 * scale*seg_size;

					double up_l = append_verticals(4 * index_seg + 1, 2) / 2 + append_verticals(4 * index_seg + 3, 2) / 2;
					double dw_l = append_verticals(4 * index_seg_1, 2) / 2 + append_verticals(4 * index_seg_1 + 2, 2) / 2;

					double up_h = append_verticals(scope_index + 4 * index_seg + 1, 2) / 2 + append_verticals(scope_index + 4 * index_seg + 3, 2) / 2;
					double dw_h = append_verticals(scope_index + 4 * index_seg_1, 2) / 2 + append_verticals(scope_index + 4 * index_seg_1 + 2, 2) / 2;



					if (u < scale - 1) {

						if (up_l > dw_l) {
							if (up_l > dw_h) {
								
								append_facets.push_back(Eigen::Vector3i(scope_index + 4 * index_seg_1, 4 * index_seg_1, scope_index + 4 * index_seg_1 + 2));
								append_facets.push_back(Eigen::Vector3i(scope_index + 4 * index_seg_1 + 2, 4 * index_seg_1, 4 * index_seg_1 + 2));
							}
							else {
								
								append_facets.push_back(Eigen::Vector3i(4 * index_seg + 1, 4 * index_seg_1, 4 * index_seg + 3));
								append_facets.push_back(Eigen::Vector3i(4 * index_seg + 3, 4 * index_seg_1, 4 * index_seg_1 + 2));
							}
						}
						if (up_l < dw_l) {//right
							if (up_h < dw_l) {
								
								append_facets.push_back(Eigen::Vector3i(4 * index_seg + 1, scope_index + 4 * index_seg + 1, 4 * index_seg + 3));
								append_facets.push_back(Eigen::Vector3i(4 * index_seg + 3, scope_index + 4 * index_seg + 1, scope_index + 4 * index_seg + 3));
							}
							else {
								
								append_facets.push_back(Eigen::Vector3i(4 * index_seg + 1, 4 * index_seg_1, 4 * index_seg + 3));
								append_facets.push_back(Eigen::Vector3i(4 * index_seg + 3, 4 * index_seg_1, 4 * index_seg_1 + 2));
							}

						}
						/*if (up_l = dw_l) {
						append_facets.emplace_back(4 * index_seg + 1, 4 * index_seg_1, 4 * index_seg + 3);
						append_facets.emplace_back(4 * index_seg + 3, 4 * index_seg_1, 4 * index_seg_1 + 2);
						}*/
					}


					if (u < scale - 1) {


						if (up_h > dw_h) {
							if (up_l > dw_h) {//right
								append_facets.push_back(Eigen::Vector3i(4 * index_seg + 1, scope_index + 4 * index_seg + 1, 4 * index_seg + 3));
								append_facets.push_back(Eigen::Vector3i(4 * index_seg + 3, scope_index + 4 * index_seg + 1, scope_index + 4 * index_seg + 3));
								
							}
							else {
								
								append_facets.push_back(Eigen::Vector3i(scope_index + 4 * index_seg_1, scope_index + 4 * index_seg + 1, scope_index + 4 * index_seg_1 + 2));
								append_facets.push_back(Eigen::Vector3i(scope_index + 4 * index_seg_1 + 2, scope_index + 4 * index_seg + 1, scope_index + 4 * index_seg + 3));
							}

						}
						if (up_h < dw_h) {
							if (up_h < dw_l) {
								
								append_facets.push_back(Eigen::Vector3i(scope_index + 4 * index_seg_1, 4 * index_seg_1, scope_index + 4 * index_seg_1 + 2));
								append_facets.push_back(Eigen::Vector3i(scope_index + 4 * index_seg_1 + 2, 4 * index_seg_1, 4 * index_seg_1 + 2));
							}
							else {//invert
								append_facets.push_back(Eigen::Vector3i(scope_index + 4 * index_seg_1, scope_index + 4 * index_seg + 1, scope_index + 4 * index_seg_1 + 2));
								append_facets.push_back(Eigen::Vector3i(scope_index + 4 * index_seg_1 + 2, scope_index + 4 * index_seg + 1, scope_index + 4 * index_seg + 3));
							}
						}
						/*if (up_h = dw_h) {
						append_facets.emplace_back(scope_index + 4 * index_seg + 1, scope_index + 4 * index_seg_1, scope_index + 4 * index_seg + 3);
						append_facets.emplace_back(scope_index + 4 * index_seg + 3, scope_index + 4 * index_seg_1, scope_index + 4 * index_seg_1 + 2);
						}*/

					}

				}
			}
			std::cout << "ok5" << std::endl;
			Eigen::MatrixXi f(append_facets.size(), 3);
			for (int j = 0; j < append_facets.size(); j++) f.row(j) = append_facets[j].transpose();
			appendix.push_back({ append_verticals, f });
			row += append_verticals.rows();
		}

	}

	std::cout << "ok6" << std::endl;
	for (int i = 0; i < appendix.size(); i++) {
		int v_row = verticles.rows();
		int f_row = facets.rows();

		verticles.conservativeResize(v_row + appendix[i].V.rows(), 3);
		verticles.block(v_row, 0, appendix[i].V.rows(), 3) = appendix[i].V;
		facets.conservativeResize(f_row + appendix[i].F.rows(), 3);
		facets.block(f_row, 0, appendix[i].F.rows(), 3) = (appendix[i].F.array() + v_row).matrix();
	}
	std::cout << "end of block 1" << std::endl;
	std::vector<Eigen::Vector3i> face;

	for (int i = 0; i < global.black_module_segments.size(); i++) {

		Eigen::Vector3i segment = global.black_module_segments[i];
		int y = segment(0);
		int x = segment(1);
		int length = segment(2);

		if (length > 1) {

			int temp_len = length;

			if ((x + length) == global.info.pixels.size()) temp_len = 4;

			for (int u = 0; u < scale; u++) {

				int seg_size = scale*temp_len;

				for (int v = 0; v < seg_size; v++) {

					int r = (y + border)*scale + u;
					int c = (x + length + border)*scale + v;


					if (u == 0) {
						double height_std = (verticles(mesh_info[r][c](0, 1), 2) + verticles(mesh_info[r][c](2, 1), 2)) / 2;
						//upper
						//if (mesh_status[r][c][0] == 0) {

						if (mesh_info[r - 1][c](1, 2) != -1 && mesh_info[r - 1][c](3, 2) != -1 && mesh_info[r - 1][c](1, 1) != -1 && mesh_info[r - 1][c](3, 1) != -1) {

							if ((verticles(mesh_info[r - 1][c](1, 2), 2) / 2 + verticles(mesh_info[r - 1][c](3, 2), 2) / 2) > height_std) {
								if ((verticles(mesh_info[r - 1][c](1, 1), 2) / 2 + verticles(mesh_info[r - 1][c](3, 1), 2) / 2) >= (verticles(mesh_info[r][c](0, 0), 2) / 2 + verticles(mesh_info[r][c](2, 0), 2) / 2)) {
									face.emplace_back(mesh_info[r - 1][c](1, 2), mesh_info[r][c](0, 0), mesh_info[r - 1][c](3, 2));
									face.emplace_back(mesh_info[r - 1][c](3, 2), mesh_info[r][c](0, 0), mesh_info[r][c](2, 0));
								}

								//mesh_status[r][c](0) = 1;
								//mesh_status[r - 1][c](0) = 1;
							}

							else {
								if ((verticles(mesh_info[r - 1][c](1, 0), 2) / 2 + verticles(mesh_info[r - 1][c](3, 0), 2) / 2) > height_std &&
									(verticles(mesh_info[r - 1][c](1, 1), 2) / 2 + verticles(mesh_info[r - 1][c](3, 1), 2) / 2) < (verticles(mesh_info[r][c](0, 0), 2) / 2 + verticles(mesh_info[r][c](2, 0), 2) / 2)) {
									face.emplace_back(mesh_info[r - 1][c](1, 0), mesh_info[r][c](0, 0), mesh_info[r - 1][c](3, 0));
									face.emplace_back(mesh_info[r - 1][c](3, 0), mesh_info[r][c](0, 0), mesh_info[r][c](2, 0));
									//mesh_status[r][c](0) = 1;
									//mesh_status[r - 1][c](0) = 1;
								}
								else {
									face.emplace_back(mesh_info[r][c](0, 1), mesh_info[r][c](0, 0), mesh_info[r][c](2, 1));
									face.emplace_back(mesh_info[r][c](2, 1), mesh_info[r][c](0, 0), mesh_info[r][c](2, 0));
								}
							}
						}
						else {

							/*	if ((verticles(mesh_info[r - 1][c](1, 0), 2) / 2 + verticles(mesh_info[r - 1][c](3, 0), 2) / 2) > height_std &&
							(verticles(mesh_info[r][c](0,0),2)/2+verticles(mesh_info[r-1][c](2,0)/2))>(verticles(mesh_info[r - 1][c](1, 1), 2) / 2 + verticles(mesh_info[r - 1][c](3, 1), 2) / 2)
							) {
							face.emplace_back(mesh_info[r - 1][c](1, 0), mesh_info[r][c](0, 0), mesh_info[r - 1][c](3, 0));
							face.emplace_back(mesh_info[r - 1][c](3, 0), mesh_info[r][c](0, 0), mesh_info[r][c](2, 0));
							//mesh_status[r][c](0) = 1;
							//mesh_status[r - 1][c](0) = 1;
							}*/
							if ((verticles(mesh_info[r - 1][c](1, 0), 2) / 2 + verticles(mesh_info[r - 1][c](3, 0), 2) / 2) < height_std) {
								face.emplace_back(mesh_info[r][c](0, 1), mesh_info[r][c](0, 0), mesh_info[r][c](2, 1));
								face.emplace_back(mesh_info[r][c](2, 1), mesh_info[r][c](0, 0), mesh_info[r][c](2, 0));
								//mesh_status[r][c](0) = 1;
							}
							else {
								face.emplace_back(mesh_info[r - 1][c](1, 0), mesh_info[r][c](0, 0), mesh_info[r - 1][c](3, 0));
								face.emplace_back(mesh_info[r - 1][c](3, 0), mesh_info[r][c](0, 0), mesh_info[r][c](2, 0));
							}

							/*if ((verticles(mesh_info[r - 1][c](1, 0), 2) / 2 + verticles(mesh_info[r - 1][c](3, 0), 2) / 2) > height_std) {
							face.emplace_back(mesh_info[r - 1][c](1, 0), mesh_info[r][c](0, 0), mesh_info[r - 1][c](3, 0));
							face.emplace_back(mesh_info[r - 1][c](3, 0), mesh_info[r][c](0, 0), mesh_info[r][c](2, 0));
							}

							if ((verticles(mesh_info[r][c](0, 0), 2) / 2 + verticles(mesh_info[r][c](2, 0) / 2)) < (verticles(mesh_info[r - 1][c](1, 1), 2) / 2 + verticles(mesh_info[r - 1][c](3, 1), 2) / 2) &&
							(verticles(mesh_info[r][c](0, 1), 2) / 2 + verticles(mesh_info[r][c](2, 1) / 2)) > (verticles(mesh_info[r - 1][c](1, 2), 2) / 2 + verticles(mesh_info[r - 1][c](3, 2), 2) / 2)
							) {
							face.emplace_back(mesh_info[r][c](0, 1), mesh_info[r][c](0, 0), mesh_info[r][c](2, 1));
							face.emplace_back(mesh_info[r][c](2, 1), mesh_info[r][c](0, 0), mesh_info[r][c](2, 0));
							}*/

						}






						//}

						//middle

						//if (mesh_status[r][c](1) == 0) {

						if (mesh_info[r - 1][c](1, 1) != -1 && mesh_info[r - 1][c](3, 1) != -1 && mesh_info[r][c](0, 1) != -1 && mesh_info[r][c](2, 1) != -1) {

							double max = std::max((verticles(mesh_info[r - 1][c](1, 1), 2) + verticles(mesh_info[r - 1][c](3, 1), 2)) / 2, (verticles(mesh_info[r][c](0, 1), 2) + verticles(mesh_info[r][c](2, 1), 2)) / 2);
							double min = std::min((verticles(mesh_info[r - 1][c](1, 1), 2) + verticles(mesh_info[r - 1][c](3, 1), 2)) / 2, (verticles(mesh_info[r][c](0, 1), 2) + verticles(mesh_info[r][c](2, 1), 2)) / 2);

							double up_0 = (verticles(mesh_info[r - 1][c](1, 0), 2) + verticles(mesh_info[r - 1][c](3, 0), 2)) / 2;
							double up_2 = (verticles(mesh_info[r - 1][c](1, 2), 2) + verticles(mesh_info[r - 1][c](3, 2), 2)) / 2;
							double down_0 = (verticles(mesh_info[r][c](0, 0), 2) + verticles(mesh_info[r][c](2, 0), 2)) / 2;
							double down_2 = (verticles(mesh_info[r][c](0, 2), 2) + verticles(mesh_info[r][c](2, 2), 2)) / 2;

							if (!((up_0<max&&up_0>min) || (up_2<max&&up_2>min) || (down_0<max&&down_0>min) || (down_2<max&&down_2>min))) {
								face.emplace_back(mesh_info[r][c](0, 1), mesh_info[r - 1][c](1, 1), mesh_info[r - 1][c](3, 1));
								face.emplace_back(mesh_info[r][c](0, 1), mesh_info[r - 1][c](3, 1), mesh_info[r][c](2, 1));
								//mesh_status[r][c](1) = 1;
								//mesh_status[r - 1][c](1) = 1;
							}
						}
						//}

						//lower

						//if (mesh_status[r][c](2) == 0) {

						double height_std_l = (verticles(mesh_info[r][c](0, 2), 2) + verticles(mesh_info[r][c](2, 2), 2)) / 2;

						if (mesh_info[r - 1][c](1, 2) != -1 && mesh_info[r - 1][c](3, 2) != -1 && mesh_info[r - 1][c](1, 1) != -1 && mesh_info[r - 1][c](3, 1) != -1) {
							if ((verticles(mesh_info[r - 1][c](1, 2), 2) / 2 + verticles(mesh_info[r - 1][c](3, 2), 2) / 2) > height_std_l &&
								(verticles(mesh_info[r - 1][c](1, 2), 2) / 2 + verticles(mesh_info[r - 1][c](3, 2), 2) / 2) < height_std) {
								face.emplace_back(mesh_info[r - 1][c](1, 2), mesh_info[r][c](0, 2), mesh_info[r - 1][c](3, 2));
								face.emplace_back(mesh_info[r - 1][c](3, 2), mesh_info[r][c](0, 2), mesh_info[r][c](2, 2));
								//mesh_status[r][c](2) = 1;
								//mesh_status[r - 1][c](2) = 1;
							}
							else if ((verticles(mesh_info[r - 1][c](1, 2), 2) / 2 + verticles(mesh_info[r - 1][c](3, 2), 2) / 2) > height_std_l &&
								(verticles(mesh_info[r - 1][c](1, 2), 2) / 2 + verticles(mesh_info[r - 1][c](3, 2), 2) / 2) > height_std) {

								face.emplace_back(mesh_info[r][c](0, 1), mesh_info[r][c](0, 2), mesh_info[r][c](2, 1));
								face.emplace_back(mesh_info[r][c](2, 1), mesh_info[r][c](0, 2), mesh_info[r][c](2, 2));
								//mesh_status[r][c](2) = 1;

							}
							if ((verticles(mesh_info[r - 1][c](1, 1), 2) / 2 + verticles(mesh_info[r - 1][c](3, 1), 2) / 2) < height_std_l &&
								(verticles(mesh_info[r - 1][c](1, 0), 2) / 2 + verticles(mesh_info[r - 1][c](3, 0), 2) / 2) > height_std) {
								face.emplace_back(mesh_info[r][c](0, 1), mesh_info[r][c](0, 2), mesh_info[r][c](2, 1));
								face.emplace_back(mesh_info[r][c](2, 1), mesh_info[r][c](0, 2), mesh_info[r][c](2, 2));
							}
							/*else if ((verticles(mesh_info[r - 1][c](1, 1), 2) / 2 + verticles(mesh_info[r - 1][c](3, 2), 2) / 2) > height_std_l &&
							(verticles(mesh_info[r - 1][c](1, 2), 2) / 2 + verticles(mesh_info[r - 1][c](3, 2), 2) / 2) < height_std){
							face.emplace_back(mesh_info[r - 1][c](1, 1), mesh_info[r][c](0, 2), mesh_info[r - 1][c](3, 1));
							face.emplace_back(mesh_info[r - 1][c](3, 1), mesh_info[r][c](0, 2), mesh_info[r][c](2, 2));
							}*/
						}
						else {
							if ((verticles(mesh_info[r - 1][c](1, 0), 2) / 2 + verticles(mesh_info[r - 1][c](3, 0), 2) / 2) < height_std) {

								face.emplace_back(mesh_info[r - 1][c](1, 0), mesh_info[r][c](0, 2), mesh_info[r - 1][c](3, 0));
								face.emplace_back(mesh_info[r - 1][c](3, 0), mesh_info[r][c](0, 2), mesh_info[r][c](2, 2));
							}
							else {
								face.emplace_back(mesh_info[r][c](0, 1), mesh_info[r][c](0, 2), mesh_info[r][c](2, 1));
								face.emplace_back(mesh_info[r][c](2, 1), mesh_info[r][c](0, 2), mesh_info[r][c](2, 2));
							}
						}
						//}

					}


					if (u == (scale - 1)) {
						double height_std = (verticles(mesh_info[r][c](1, 1), 2) + verticles(mesh_info[r][c](3, 1), 2)) / 2;

						//upper
						//if (mesh_status[r][c][0] == 0) {

						if (mesh_info[r + 1][c](0, 2) != -1 && mesh_info[r + 1][c](2, 2) != -1 && mesh_info[r + 1][c](0, 1) != -1 && mesh_info[r + 1][c](2, 1) != -1) {

							if ((verticles(mesh_info[r + 1][c](0, 2), 2) / 2 + verticles(mesh_info[r + 1][c](2, 2), 2) / 2) > height_std) {
								if ((verticles(mesh_info[r + 1][c](0, 1), 2) / 2 + verticles(mesh_info[r + 1][c](2, 1), 2) / 2) >= (verticles(mesh_info[r][c](1, 0), 2) / 2 + verticles(mesh_info[r][c](3, 0), 2) / 2)) {
									face.emplace_back(mesh_info[r][c](1, 0), mesh_info[r + 1][c](0, 2), mesh_info[r][c](3, 0));
									face.emplace_back(mesh_info[r][c](3, 0), mesh_info[r + 1][c](0, 2), mesh_info[r + 1][c](2, 2));
								}

								//mesh_status[r][c](0) = 1;
								//mesh_status[r + 1][c](0) = 1;
							}

							else {
								if ((verticles(mesh_info[r + 1][c](0, 0), 2) / 2 + verticles(mesh_info[r + 1][c](2, 0), 2) / 2) > height_std
									&& (verticles(mesh_info[r + 1][c](0, 1), 2) / 2 + verticles(mesh_info[r + 1][c](2, 1), 2) / 2) < (verticles(mesh_info[r][c](1, 0), 2) / 2 + verticles(mesh_info[r][c](3, 0), 2) / 2)) {
									face.emplace_back(mesh_info[r][c](1, 0), mesh_info[r + 1][c](0, 0), mesh_info[r][c](3, 0));
									face.emplace_back(mesh_info[r][c](3, 0), mesh_info[r + 1][c](0, 0), mesh_info[r + 1][c](2, 0));
									//mesh_status[r][c](0) = 1;
									//mesh_status[r + 1][c](0) = 1;
								}
								else {
									face.emplace_back(mesh_info[r][c](1, 0), mesh_info[r][c](1, 1), mesh_info[r][c](3, 0));
									face.emplace_back(mesh_info[r][c](3, 0), mesh_info[r][c](1, 1), mesh_info[r][c](3, 1));
								}

							}
						}
						else {
							/*if ((verticles(mesh_info[r + 1][c](0, 0), 2) / 2 + verticles(mesh_info[r + 1][c](2, 0), 2) / 2) > height_std&&
							(verticles(mesh_info[r][c](1,0),2)/2+verticles(mesh_info[r][c](3,0),2)/2)>(verticles(mesh_info[r + 1][c](0, 1), 2) / 2 + verticles(mesh_info[r + 1][c](2, 1), 2) / 2)
							) {
							face.emplace_back(mesh_info[r][c](1, 0), mesh_info[r + 1][c](0, 0), mesh_info[r][c](3, 0));
							face.emplace_back(mesh_info[r][c](3, 0), mesh_info[r + 1][c](0, 0), mesh_info[r + 1][c](2, 0));
							//mesh_status[r][c](0) = 1;
							//mesh_status[r + 1][c](0) = 1;
							}*/

							if ((verticles(mesh_info[r + 1][c](0, 0), 2) / 2 + verticles(mesh_info[r + 1][c](2, 0), 2) / 2) < height_std) {
								face.emplace_back(mesh_info[r][c](1, 0), mesh_info[r][c](1, 1), mesh_info[r][c](3, 0));
								face.emplace_back(mesh_info[r][c](3, 0), mesh_info[r][c](1, 1), mesh_info[r][c](3, 1));
								//mesh_status[r][c](0) = 1;
							}
							else {
								face.emplace_back(mesh_info[r][c](1, 0), mesh_info[r + 1][c](0, 0), mesh_info[r][c](3, 0));
								face.emplace_back(mesh_info[r][c](3, 0), mesh_info[r + 1][c](0, 0), mesh_info[r + 1][c](2, 0));
							}

							/*if ((verticles(mesh_info[r + 1][c](0, 0), 2) / 2 + verticles(mesh_info[r + 1][c](2, 0), 2) / 2) > height_std) {
							face.emplace_back(mesh_info[r][c](1, 0), mesh_info[r + 1][c](0, 0), mesh_info[r][c](3, 0));
							face.emplace_back(mesh_info[r][c](3, 0), mesh_info[r + 1][c](0, 0), mesh_info[r + 1][c](2, 0));
							//mesh_status[r][c](0) = 1;
							}

							if ((verticles(mesh_info[r][c](1, 0), 2) / 2 + verticles(mesh_info[r][c](3, 0), 2) / 2) < (verticles(mesh_info[r + 1][c](0, 1), 2) / 2 + verticles(mesh_info[r + 1][c](2, 1), 2) / 2) &&
							(verticles(mesh_info[r][c](1, 1), 2) / 2 + verticles(mesh_info[r][c](3, 1), 2) / 2) > (verticles(mesh_info[r + 1][c](0, 2), 2) / 2 + verticles(mesh_info[r + 1][c](2, 2), 2) / 2)) {
							face.emplace_back(mesh_info[r][c](1, 0), mesh_info[r][c](1, 1), mesh_info[r][c](3, 0));
							face.emplace_back(mesh_info[r][c](3, 0), mesh_info[r][c](1, 1), mesh_info[r][c](3, 1));
							}*/

							//}
						}

						//middle

						//if (mesh_status[r][c](1) == 0) {


						if (mesh_info[r + 1][c](0, 1) != -1 && mesh_info[r + 1][c](2, 1) != -1 && mesh_info[r][c](1, 1) != -1 && mesh_info[r][c](3, 1) != -1) {

							double max = std::max((verticles(mesh_info[r][c](1, 1), 2) + verticles(mesh_info[r][c](3, 1), 2)) / 2, (verticles(mesh_info[r + 1][c](0, 1), 2) + verticles(mesh_info[r + 1][c](2, 1), 2)) / 2);
							double min = std::min((verticles(mesh_info[r][c](1, 1), 2) + verticles(mesh_info[r][c](3, 1), 2)) / 2, (verticles(mesh_info[r + 1][c](0, 1), 2) + verticles(mesh_info[r + 1][c](2, 1), 2)) / 2);

							double up_0 = (verticles(mesh_info[r][c](1, 0), 2) + verticles(mesh_info[r][c](3, 0), 2)) / 2;
							double up_2 = (verticles(mesh_info[r][c](1, 2), 2) + verticles(mesh_info[r][c](3, 2), 2)) / 2;
							double down_0 = (verticles(mesh_info[r + 1][c](0, 0), 2) + verticles(mesh_info[r + 1][c](2, 0), 2)) / 2;
							double down_2 = (verticles(mesh_info[r + 1][c](0, 2), 2) + verticles(mesh_info[r + 1][c](2, 2), 2)) / 2;

							if (!((up_0<max&&up_0>min) || (up_2<max&&up_2>min) || (down_0<max&&down_0>min) || (down_2<max&&down_2>min))) {
								face.emplace_back(mesh_info[r + 1][c](0, 1), mesh_info[r][c](1, 1), mesh_info[r][c](3, 1));
								face.emplace_back(mesh_info[r + 1][c](0, 1), mesh_info[r][c](3, 1), mesh_info[r + 1][c](2, 1));
								//mesh_status[r][c](1) = 1;
								//mesh_status[r - 1][c](1) = 1;
							}
						}
						//}

						//lower

						//if (mesh_status[r][c](2) == 0) {

						double height_std_l = (verticles(mesh_info[r][c](1, 2), 2) + verticles(mesh_info[r][c](3, 2), 2)) / 2;

						if (mesh_info[r + 1][c](0, 2) != -1 && mesh_info[r + 1][c](2, 2) != -1 && mesh_info[r + 1][c](0, 1) != -1 && mesh_info[r + 1][c](2, 1) != -1) {
							if ((verticles(mesh_info[r + 1][c](0, 2), 2) / 2 + verticles(mesh_info[r + 1][c](2, 2), 2) / 2) > height_std_l &&
								(verticles(mesh_info[r + 1][c](0, 2), 2) / 2 + verticles(mesh_info[r + 1][c](2, 2), 2) / 2) < height_std) {
								face.emplace_back(mesh_info[r][c](1, 2), mesh_info[r + 1][c](0, 2), mesh_info[r][c](3, 2));
								face.emplace_back(mesh_info[r][c](3, 2), mesh_info[r + 1][c](0, 2), mesh_info[r + 1][c](2, 2));
								//mesh_status[r][c](2) = 1;
								//mesh_status[r + 1][c](2) = 1;
							}
							else if ((verticles(mesh_info[r + 1][c](0, 2), 2) / 2 + verticles(mesh_info[r + 1][c](2, 2), 2) / 2) > height_std_l &&
								(verticles(mesh_info[r + 1][c](0, 2), 2) / 2 + verticles(mesh_info[r + 1][c](0, 2), 2) / 2) > height_std) {

								face.emplace_back(mesh_info[r][c](1, 2), mesh_info[r][c](1, 1), mesh_info[r][c](3, 1));
								face.emplace_back(mesh_info[r][c](1, 2), mesh_info[r][c](3, 1), mesh_info[r][c](3, 2));
								//mesh_status[r][c](2) = 1;

							}
							if ((verticles(mesh_info[r + 1][c](0, 1), 2) / 2 + verticles(mesh_info[r + 1][c](2, 1), 2) / 2) < height_std_l &&
								(verticles(mesh_info[r + 1][c](0, 0), 2) / 2 + verticles(mesh_info[r + 1][c](2, 0), 2) / 2) > height_std) {
								face.emplace_back(mesh_info[r][c](1, 2), mesh_info[r][c](1, 1), mesh_info[r][c](3, 1));
								face.emplace_back(mesh_info[r][c](1, 2), mesh_info[r][c](3, 1), mesh_info[r][c](3, 2));
							}
							/*else if ((verticles(mesh_info[r + 1][c](1, 1), 2) / 2 + verticles(mesh_info[r + 1][c](3, 2), 2) / 2) > height_std_l &&
							(verticles(mesh_info[r + 1][c](1, 2), 2) / 2 + verticles(mesh_info[r + 1][c](3, 2), 2) / 2) < height_std){
							face.emplace_back(mesh_info[r + 1][c](1, 1), mesh_info[r][c](0, 2), mesh_info[r + 1][c](3, 1));
							face.emplace_back(mesh_info[r + 1][c](3, 1), mesh_info[r][c](0, 2), mesh_info[r][c](2, 2));
							}*/
						}
						else {
							if ((verticles(mesh_info[r + 1][c](0, 0), 2) / 2 + verticles(mesh_info[r + 1][c](2, 0), 2) / 2) < height_std) {

								face.emplace_back(mesh_info[r][c](1, 2), mesh_info[r + 1][c](0, 0), mesh_info[r][c](3, 2));
								face.emplace_back(mesh_info[r][c](3, 2), mesh_info[r + 1][c](0, 0), mesh_info[r + 1][c](2, 0));
							}
							else {
								face.emplace_back(mesh_info[r][c](1, 2), mesh_info[r][c](1, 1), mesh_info[r][c](3, 1));
								face.emplace_back(mesh_info[r][c](1, 2), mesh_info[r][c](3, 1), mesh_info[r][c](3, 2));
							}
						}
						//}


					}

				}
			}

		}

	}

	Eigen::MatrixXi add_facets(face.size(), 3);
	for (int i = 0; i < face.size(); i++) add_facets.row(i) = face[i];
	int origin_row = facets.rows();
	facets.conservativeResize(origin_row + add_facets.rows(), 3);
	facets.block(origin_row, 0, add_facets.rows(), 3) = add_facets;
	std::cout << "end of block 2" << std::endl;
	std::vector<Eigen::Vector3i> face_vec;
	for (int i = 0; i < facets.rows(); i++) face_vec.push_back(facets.row(i));
	std::sort(face_vec.begin(), face_vec.end(), [](Eigen::Vector3i v1, Eigen::Vector3i v2) {
		int a, b;
		int max1 = v1.maxCoeff(&a);
		int min1 = v1.minCoeff(&b);
		int mid1;
		for (int g = 0; g < 3; g++)
			if (g != a&&g != b) mid1 = v1[g];

		int max2 = v2.maxCoeff(&a);
		int min2 = v2.minCoeff(&b);
		int mid2;
		for (int g = 0; g < 3; g++)
			if (g != a&&g != b) mid2 = v2[g];

		if (max1<max2) return true;
		if (max1 > max2) return false;
		if (max1 == max2) {
			if (mid1<mid2) return true;
			if (mid1>mid2) return false;
			if (mid1 == mid2) {
				if (min1 <= min2) return true;
				if (min1 > min2) return false;
			}
		}

	});

	std::cout << "block3" << std::endl;

	face_vec.erase(std::unique(face_vec.begin(), face_vec.end(), [](Eigen::Vector3i v1, Eigen::Vector3i v2) {
		int a, b;
		int max1 = v1.maxCoeff(&a);
		int min1 = v1.minCoeff(&b);
		int mid1;
		for (int g = 0; g < 3; g++)
			if (g != a&&g != b) mid1 = v1[g];

		int max2 = v2.maxCoeff(&a);
		int min2 = v2.minCoeff(&b);
		int mid2;
		for (int g = 0; g < 3; g++)
			if (g != a&&g != b) mid2 = v2[g];

		return((max1 == max2) && (mid1 == mid2) && (min1 == min2));

	}), face_vec.end());

	facets.resize(face_vec.size(), 3);
	for (int i = 0; i < face_vec.size(); i++) facets.row(i) = face_vec[i];


	igl::writeOBJ("Optimization/final_model.obj", verticles, facets);
}
