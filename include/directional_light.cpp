#include "directional_light.h"

void qrcode::directional_light(igl::viewer::Viewer & viewer, Engine * engine, GLOBAL & global, Eigen::MatrixXd & verticles, Eigen::MatrixXi & facets)
{
	igl::Timer timer;

	/*Upper elevation and lower elevation*/

	const auto radian = [](float angle)->float {return angle / 180 * igl::PI; };

	Eigen::Vector3f upper_elevation, lower_elevation;

	upper_elevation << std::cos(radian(global.latitude_upper))*std::cos(radian(global.longitude)),
		std::cos(radian(global.latitude_upper))*std::sin(radian(global.longitude)),
		std::sin(radian(global.latitude_upper));

	lower_elevation << std::cos(radian(global.latitude_lower))*std::cos(radian(global.longitude)),
		std::cos(radian(global.latitude_lower))*std::sin(radian(global.longitude)),
		std::sin(radian(global.latitude_lower));


	/*model center*/
	int scale = global.info.scale;
	int border = global.info.border;
	int qr_size =( global.info.pixels.size()+2*border)*scale;
	
	Eigen::MatrixXi controller = global.under_control.block(scale, scale, qr_size, qr_size);
	std::vector<Eigen::RowVector4f> useful_point;

	for (int y = 0; y < qr_size-2*border*scale; y++) {
		for (int x = 0; x < qr_size-2*border*scale; x++) {
			if (controller(y + border*scale, x + border*scale) == 1) {
				Eigen::RowVector3d p = global.hit_matrix.row((y + border*scale)*(qr_size + 1) + x + border*scale);
				useful_point.push_back(Eigen::RowVector4f(p(0), p(1), p(2), 1));
			}
		}
	}
		
	Eigen::MatrixXf V(useful_point.size(),4);
	
	for (int i = 0; i < useful_point.size(); i++)  V.row(i) << useful_point[i];

	V = (global.mode*(V.transpose())).transpose().block(0, 0, V.rows(), 3);

	Eigen::VectorXf centroid;
	centroid.setZero(3);

	for (int i = 0; i < V.rows(); i++)  centroid = (centroid.array() + V.row(i).transpose().array()).matrix();

	centroid = centroid/V.rows();

	/*Upper light source and lower light source*/

	Eigen::VectorXf upper_source(3), lower_source(3), upper_direct(3), lower_direct(3);

	upper_source = centroid + upper_elevation*global.zoom*global.distance;
	lower_source = centroid + lower_elevation*global.zoom*global.distance;

	Eigen::Matrix4f model = global.mode.inverse().eval();

	centroid.conservativeResize(4);
	centroid(3) = 1.f;
	centroid = (model*centroid).block(0, 0, 3, 1);

	upper_source.conservativeResize(4);
	upper_source(3) = 1.f;
	upper_source = (model*upper_source).block(0, 0, 3, 1);

	upper_direct = (centroid-upper_source).normalized();

	lower_source.conservativeResize(4);
	lower_source(3) = 1.f;
	lower_source = (model*lower_source).block(0, 0, 3, 1);

	lower_direct = (centroid-lower_source).normalized();

	/*upper field angle and lower field angle*/

	const auto upper_field_angle = [&radian,&upper_source, &upper_direct](Eigen::RowVectorXf &destination)->bool {
		Eigen::VectorXf d = (upper_source - destination.transpose()).normalized();
		return (std::sqrt(1.f - d.dot(upper_direct)*d.dot(upper_direct)) > std::sin(radian(5))) ? false : true;
	};

	const auto lower_field_angle = [&radian, &lower_source, &lower_direct](Eigen::RowVectorXf &destination)->bool {
		Eigen::VectorXf d = (lower_source - destination.transpose()).normalized();
		return (std::sqrt(1.f - d.dot(lower_direct)*d.dot(lower_direct)) > std::sin(radian(5))) ? false : true;
	};

	/*Merge meshes*/
	qrcode::find_hole(engine, global);
	qrcode::make_hole(global);
	qrcode::fix_hole(engine, global);

	verticles.resize(global.qr_verticals.rows() + global.rest_verticals.rows(), 3);
	verticles.block(0, 0, global.qr_verticals.rows(), 3) = global.qr_verticals;
	verticles.block( global.qr_verticals.rows(),0, global.rest_verticals.rows(), 3) = global.rest_verticals;

	int size = global.qr_facets.rows() + global.rest_facets.rows();
	

	facets.resize(size, 3);
	facets.block(0, 0, global.qr_facets.rows(), 3) = global.qr_facets;
	facets.block(global.qr_facets.rows(),0, global.rest_facets.rows(), 3) = global.rest_facets;

	for (int i = 0; i < global.component.size(); i++) {
		size = facets.rows();
		facets.conservativeResize(size + global.patches[i].rows(), 3);
		facets.block(size, 0, global.patches[i].rows(), 3) = global.patches[i];
	}




	/*Ambient occlusion visible region*/
	std::vector<Eigen::MatrixXi> modules = qrcode::module_adapter(engine,global);//pixel.size*scale+1;
	Eigen::MatrixXi both_modules = modules[0] + modules[1];

	Eigen::MatrixXi label;
	qrcode::bwlabel(engine, both_modules, 4, label);



	std::vector<Eigen::Vector3i> visible_info;
	for (int y = 0; y < label.rows(); y++)
		for (int x = 0; x < label.cols(); x++)
			if (label(y, x) != 0)
				visible_info.push_back(Eigen::Vector3i(y, x, label(y, x)));
	
	std::vector<Eigen::MatrixXi> visible_bound;
	qrcode::bwbound(label, visible_bound);
	std::vector<qrcode::SMesh> sphere_meshes = qrcode::visible_mesh_on_sphere(visible_info, visible_bound, global);

	std::cout << " Sphere mesh end" << std::endl;

	/*iterator step*/
	float step = 100000;
	for (int y = 0; y < modules[0].rows() - 1; y++) {
		for (int x = 0; x < modules[0].cols() - 1; x++) {
			float length = (global.hit_matrix.row(y*modules[0].cols() + x + 1) - global.hit_matrix.row(y*modules[0].cols() + x)).cast<float>().norm()
				/ abs(global.direct(y*modules[0].cols() + x, 2));
			if (length < step) step = length;
		}
	}

	step = step /1;



	/*Depth initialization*/
	global.carve_depth.setZero(global.qr_verticals.rows());
	Eigen::VectorXf depth;
	depth.setZero(global.anti_indicatior.size());

	for (int i = 0; i < global.anti_indicatior.size(); i++) {
		int y = global.anti_indicatior[i](0);
		int x = global.anti_indicatior[i](1);

		if (modules[0](y, x) == 1)
			depth(i) += step;

		else if (modules[1](y, x) == 1)
			depth(i) += step;
	}

	for (int i = 0; i < global.anti_indicatior.size(); i++) {
		int y = global.anti_indicatior[i](0);
		int x = global.anti_indicatior[i](1);


		if (modules[0](y, x) == 1)
			qrcode::patch(y, x, depth,both_modules, global);
		else if (modules[1](y, x) == 1)
			qrcode::patch(y, x, depth,both_modules, global);
	}

	Eigen::MatrixXd qr_verticals = global.qr_verticals;
	qrcode::carving_down(global,qr_verticals);

	verticles.block(0, 0, global.qr_verticals.rows(), 3) = qr_verticals;

	/*QR code normal and position*/
	Eigen::MatrixXf qr_position, qr_normal;
	qrcode::pre_pixel_normal(global, qr_verticals,qr_position, qr_normal);

	std::vector<Eigen::Vector3f> white_position, white_normal, upper_black_position, upper_black_normal,lower_black_position,lower_black_normal,both_black_position,both_black_normal;

	for (int i = 0; i < global.anti_indicatior.size(); i++) {
		int y = global.anti_indicatior[i](0);
		int x = global.anti_indicatior[i](1);

			if (modules[0](y, x) == 1) {
				upper_black_position.push_back(qr_position.row(i).transpose());
				upper_black_normal.push_back(qr_normal.row(i).transpose());
				both_black_position.push_back(qr_position.row(i).transpose());
				both_black_normal.push_back(qr_normal.row(i).transpose());

			}
			else if (modules[1](y, x) == 1) {
				lower_black_position.push_back(qr_position.row(i).transpose());
				lower_black_normal.push_back(qr_normal.row(i).transpose());
				both_black_position.push_back(qr_position.row(i).transpose());
				both_black_normal.push_back(qr_normal.row(i).transpose());
			}
			else {
				white_position.push_back(qr_position.row(i).transpose());
				white_normal.push_back(qr_normal.row(i).transpose());
			}
	}
	
	const auto light_to_gray = [](float a, float d)->int {
		return static_cast<int>(19.6*pow((0.87*(24 * a + 456 * d)), 0.3441) + 21.24);
	};

	Eigen::MatrixXf AO, DO_upper,DO_lower,DO;

	AO.setOnes(qr_size, qr_size);
	DO_upper.setOnes(qr_size, qr_size);
	DO_lower.setOnes(qr_size, qr_size);

	/*Test white region if lighted or not*/
	Eigen::Matrix<bool, Eigen::Dynamic, 1> white_condition;
	qrcode::light(verticles, facets, upper_source, white_position, white_condition);
	
	bool all_light = true;
	int index_white = 0;

	for (int i = 0; i < global.anti_indicatior.size(); i++) {
		int y = global.anti_indicatior[i](0);
		int x = global.anti_indicatior[i](1);

		if ( modules[0](y, x) == 0 && modules[1](y, x) == 0) {

			if (y >= border*scale&&y < (qr_size - border*scale) && x >= border*scale&&x < (qr_size - border*scale)) {
				all_light &= white_condition(index_white);
			}
			index_white++;
		}
	}

	if (!all_light) {
		std::cout << "White modules can not be lighted!!" << std::endl;
		return;
	}

	std::vector<int> white_gray_value;

	Eigen::MatrixXi simu_gray_scale(qr_size, qr_size);
	simu_gray_scale.setConstant(255);
	
	Eigen::VectorXf white_AO;
	qrcode::ambient_occlusion(verticles, facets, white_position, white_normal, 500, white_AO);
	std::cout << "white ambient occlusion end" << std::endl;

	index_white = 0;
	for (int i = 0; i < global.anti_indicatior.size(); i++) {
		int y = global.anti_indicatior[i](0);
		int x = global.anti_indicatior[i](1);

		if (modules[0](y, x) == 0 && modules[1](y, x) == 0) {
			AO(y, x) = white_AO(index_white);
			
			Eigen::Vector3f dir = (upper_source - white_position[index_white]).normalized();
			DO_upper(y, x) = (white_condition(index_white) ? 1.f : 0.f)*dir.dot(white_normal[index_white]);
			DO_lower(y, x) = DO_upper(y, x);

			white_gray_value.push_back(light_to_gray(AO(y, x), DO_upper(y, x)));
			simu_gray_scale(y, x) = light_to_gray(AO(y, x), DO_upper(y, x));
			index_white++;
		}
	}

	std::sort(white_gray_value.begin(), white_gray_value.end(), [](int a, int b) {return a > b; });

	int top10 = ceil(white_gray_value.size() / 10);

	float white_average = 0;

	for (int i = 0; i < top10; i++) white_average += white_gray_value[i];

	white_average /= top10;

	

	
	/*Optimization*/

	


	bool should_stop = false;
	int iter_count = 0;

	while (!should_stop) {

		should_stop = true;

		Eigen::Matrix<bool, Eigen::Dynamic, 1> upper_black_condition, lower_black_condition,lower_black_upper_condition;

		qrcode::light(verticles, facets, upper_source, upper_black_position, upper_black_condition);
		qrcode::light(verticles, facets, lower_source, lower_black_position, lower_black_condition);
		qrcode::light(verticles, facets, lower_source, upper_black_position, lower_black_upper_condition);

		/*ambient simulation*/
		Eigen::VectorXf black_AO;
		qrcode::ambient_occlusion(verticles, facets, both_black_position, both_black_normal, sphere_meshes, black_AO);

		int index_upper_point = 0;
		int index_lower_point = 0;
		int index_both_point = 0;

		for (int i = 0; i < global.anti_indicatior.size(); i++) {
			int y = global.anti_indicatior[i](0);
			int x = global.anti_indicatior[i](1);

			if (modules[0](y, x) == 1) {

				Eigen::Vector3f upper_dir = (upper_source - upper_black_position[index_upper_point]).normalized();
				Eigen::Vector3f lower_dir = (lower_source - upper_black_position[index_upper_point]).normalized();

				AO(y, x) = black_AO(index_both_point) > 1 ? 1 : black_AO(index_both_point);

				DO_upper(y, x) = (upper_black_condition(index_upper_point) ? 1.f : 0.f)*upper_dir.dot(upper_black_normal[index_upper_point]);
				DO_lower(y, x) = (lower_black_upper_condition(index_upper_point) ? 1.f : 0.f)*lower_dir.dot(upper_black_normal[index_upper_point]);

				int gray_value = light_to_gray(AO(y, x), DO_upper(y, x));

				simu_gray_scale(y, x) = gray_value;

				if (gray_value > (white_average - 255 * 0.2f)) {

					depth(i) += step;
					should_stop = false;
				}

				index_upper_point++;
				index_both_point++;
			}
			else if (modules[1](y, x) == 1) {

				AO(y, x) = black_AO(index_both_point) > 1 ? 1 : black_AO(index_both_point);
				Eigen::Vector3f upper_dir = (upper_source - lower_black_position[index_lower_point]).normalized();
				Eigen::Vector3f lower_dir = (lower_source - lower_black_position[index_lower_point]).normalized();
				DO_upper(y, x) = 1.f*upper_dir.dot(lower_black_normal[index_lower_point]);
				DO_lower(y, x) = (lower_black_condition(index_lower_point) ? 1.f : 0.f)*lower_dir.dot(lower_black_normal[index_lower_point]);

				int gray_value = light_to_gray(AO(y, x), DO_lower(y, x));

				simu_gray_scale(y, x) = gray_value;

				if (gray_value > (white_average - 255 * 0.2f)) {

					depth(i) += step;
					should_stop = false;

				}

				index_lower_point++;
				index_both_point++;
			}

		}

		
		for (int i = 0; i < global.anti_indicatior.size(); i++) {
			int y = global.anti_indicatior[i](0);
			int x = global.anti_indicatior[i](1);

			if (modules[0](y, x) == 1)
				qrcode::patch(y, x, depth,both_modules, global);
			else if (modules[1](y, x) == 1)
				qrcode::patch(y, x, depth,both_modules, global);
		}

		qrcode::carving_down(global, qr_verticals);
		verticles.block(0, 0, global.qr_verticals.rows(), 3) = qr_verticals;

		qrcode::pre_pixel_normal(global,qr_verticals,qr_position, qr_normal);

		upper_black_position.clear();
		upper_black_normal.clear();
		lower_black_position.clear(); 
		lower_black_normal.clear(); 
		both_black_position.clear();
		both_black_normal.clear();

		for (int i = 0; i < global.anti_indicatior.size(); i++) {
			int y = global.anti_indicatior[i](0);
			int x = global.anti_indicatior[i](1);

			if (modules[0](y, x) == 1) {
				upper_black_position.push_back(qr_position.row(i).transpose());
				upper_black_normal.push_back(qr_normal.row(i).transpose());
				both_black_position.push_back(qr_position.row(i).transpose());
				both_black_normal.push_back(qr_normal.row(i).transpose());

			}
			else if (modules[1](y, x) == 1) {
				lower_black_position.push_back(qr_position.row(i).transpose());
				lower_black_normal.push_back(qr_normal.row(i).transpose());
				both_black_position.push_back(qr_position.row(i).transpose());
				both_black_normal.push_back(qr_normal.row(i).transpose());
			}
		}
		igl::writeOBJ("F:/3DQ/3DQR/data/Optimization/iter_" + std::to_string(iter_count) + ".obj", verticles, facets);
		qrcode::write_png("F:/3DQ/3DQR/data/Optimization/iter_" + std::to_string(iter_count) + ".png", simu_gray_scale);

		std::cout << "End of iterator:" << iter_count << std::endl;
		iter_count++;
	}

	int bound = global.info.pixels.size();

	for (int i = 0; i < global.black_module_segments.size(); i++) {

		Eigen::Vector3i segment = global.black_module_segments[i];
		int y = segment(0);
		int x = segment(1);
		int length = segment(2);
		

		if (length == 1) {
			int x_behind = x + length; 
			for (int u = 0; u < scale; u++) {

				int origin_index = global.indicator[(y + border)*scale + u][(x + border)*scale + scale - 1](1);
				int end_index = global.indicator[(y + border)*scale + u][(x_behind + border)*scale + scale - 1](1);
				double origin_upper_point = qr_verticals(4 * origin_index + 2, 2);
				double origin_lower_point = qr_verticals(4 * origin_index + 3, 2);
				double end_upper_point = qr_verticals(4 * end_index + 2, 2);
				double end_lower_point = qr_verticals(4 * end_index + 3, 2);

				for (int v = 0; v < scale; v++) {
					int index = global.indicator[(y + border)*scale + u][(x_behind + border)*scale + v](1);
					double a = ( scale- v)*(end_upper_point - origin_upper_point) / scale / abs(global.direct(((y + border)*scale + u)*(qr_size + 1) + (x_behind + border)*scale + v, 2));
					double b = (scale - v)*(end_lower_point - origin_lower_point) / scale / abs(global.direct(((y + border)*scale + u + 1)*(qr_size + 1) + (x_behind + border)*scale + v, 2));
					double c = (scale - (v+1))*(end_upper_point - origin_upper_point) / scale /abs( global.direct(((y + border)*scale + u)*(qr_size + 1) + (x_behind + border)*scale + v + 1, 2));
					double d = (scale - (v + 1))*(end_upper_point - origin_upper_point) / scale / abs(global.direct(((y + border)*scale + u + 1)*(qr_size + 1) + (x_behind + border)*scale + v + 1, 2));
					qrcode::patch((y + border)*scale + u, (x_behind + border)*scale + v, global, Eigen::Vector4d(a, b, c, d));
					
				}
			}	
		}	
	}
	qrcode::carving_down(global, qr_verticals);

	for (int i = 0; i < global.black_module_segments.size(); i++) {

		Eigen::Vector3i segment = global.black_module_segments[i];
		int y = segment(0);
		int x = segment(1);
		int length = segment(2);

		if (length>1) {
			int x_end = x + length - 1;
			for (int u = 0; u < scale; u++) {
				int lower_index = global.indicator[(y + border)*scale + u][(x_end + border)*scale + scale - 1](1);
				int upper_index = global.indicator[(y + border)*scale + u][(x_end + border + 1)*scale](1);

				Eigen::Vector3d upper_left_point = qr_verticals.row(4 * upper_index).transpose();
				Eigen::Vector3d upper_right_point = qr_verticals.row(4 * upper_index + 1).transpose();

				Eigen::Vector3d lower_left_point = qr_verticals.row(4 * lower_index + 2).transpose();
				Eigen::Vector3d lower_right_point = qr_verticals.row(4 * lower_index + 3).transpose();

				Eigen::Vector3d left_dir = (upper_left_point - upper_source.cast<double>());
				Eigen::Vector3d right_dir = (upper_right_point - upper_source.cast<double>());

				double r_left = (lower_left_point(2) - upper_left_point(2)) / left_dir(2);
				double r_right = (lower_right_point(2) - upper_right_point(2)) / right_dir(2);

				qr_verticals.row(4 * lower_index + 2) = (upper_left_point + r_left*left_dir).transpose();
				qr_verticals.row(4 * lower_index + 3) = (upper_right_point + r_right*right_dir).transpose();

			}
		}

	}
	
	verticles.block(0, 0, global.qr_verticals.rows(), 3) = qr_verticals;



}

