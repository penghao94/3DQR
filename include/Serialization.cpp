#include "Serialization.h"

void qrcode::serialize(GLOBAL & global, std::string & binary_file)
{
	serialize(global.info, binary_file);

	igl::serialize(global.model_vertices, "Vertices", binary_file);
	igl::serialize(global.model_facets, "Facets", binary_file);
	igl::serialize(global.model_colors, "Colors", binary_file);

	igl::serialize(global.source, "Source", binary_file);
	igl::serialize(global.direct, "Direct", binary_file);

	igl::serialize(global.qr_verticals, "QR Verticals", binary_file);
	igl::serialize(global.qr_facets, "QR Facets", binary_file);
	igl::serialize(global.qr_colors, "QR Colors", binary_file);
	igl::serialize(global.hit_matrix, "Hit Matrix", binary_file);

	std::vector<int> id_binary(global.hitmap.size());
	std::vector<float> u_binary(global.hitmap.size());
	std::vector<float> v_binary(global.hitmap.size());
	std::vector<float> t_binary(global.hitmap.size());

	for (int i = 0; i < global.hitmap.size(); i++) {
		id_binary[i] = global.hitmap[i].id;
		u_binary[i] = global.hitmap[i].u;
		v_binary[i] = global.hitmap[i].v;
		t_binary[i] = global.hitmap[i].t;
	}
	igl::serialize(global.mode, "model", binary_file);
	igl::serialize(global.zoom, "Zoom", binary_file);
	igl::serialize(id_binary, "Id", binary_file);
	igl::serialize(u_binary, "U", binary_file);
	igl::serialize(v_binary, "V", binary_file);
	igl::serialize(t_binary, "T", binary_file);
	
	igl::serialize(global.carve_depth, "Carving depth", binary_file);
	igl::serialize(global.under_control, "Under Control", binary_file);

	std::vector<std::vector<int>> anti_indicator_binary(global.anti_indicatior.size());
	for (int i = 0; i < global.anti_indicatior.size(); i++) {
		anti_indicator_binary[i].push_back(global.anti_indicatior[i](0));
		anti_indicator_binary[i].push_back(global.anti_indicatior[i](1));
	}
	igl::serialize(anti_indicator_binary, "anti indicator", binary_file);


	std::vector<std::vector<int>> patch_indicator_binary(global.patch_indicator.size());

	for (int i = 0; i < global.patch_indicator.size(); i++) 
		patch_indicator_binary[i]={ global.patch_indicator[i](0),global.patch_indicator[i](1), global.patch_indicator[i](2), global.patch_indicator[i](3)};
	
	igl::serialize(patch_indicator_binary, "patch indicator", binary_file);

	std::vector<std::vector<int>> indicator_binary_1(global.indicator.size()), indicator_binary_2(global.indicator.size());
	for (int i = 0; i < global.indicator.size(); i++) {
		for (int j = 0; j < global.indicator[i].size(); j++) {
			indicator_binary_1[i].push_back(global.indicator[i][j](0));
			indicator_binary_2[i].push_back(global.indicator[i][j](1));
		}
	}
	igl::serialize(indicator_binary_1, "indicator binary 1", binary_file);
	igl::serialize(indicator_binary_2, "indicator binary 2", binary_file);

	igl::serialize(global.seeds, "Seeds", binary_file);
	for (int i = 0; i < global.islands.size(); i++) {
		std::string binary_name = "Island" + std::to_string(i);
		igl::serialize(global.islands[i], binary_name, binary_file);
	}

	igl::serialize(global.component, "Component", binary_file);

	igl::serialize(global.hole_facet, "Hole facet", binary_file);

	for (int i = 0; i < global.holes.size(); i++) {
		std::string binary_name = "Hole" + std::to_string(i);
		igl::serialize(global.holes[i], binary_name, binary_file);
	}

	igl::serialize(global.rest_verticals, "Rest verticals", binary_file);
	igl::serialize(global.rest_facets, "Rest facets", binary_file);
}

void qrcode::deserialize(GLOBAL & global, std::string & binary_file)
{
	global.info=deserialize(binary_file);

	igl::deserialize(global.model_vertices, "Vertices", binary_file);
	igl::deserialize(global.model_facets, "Facets", binary_file);
	igl::deserialize(global.model_colors, "Colors", binary_file);

	igl::deserialize(global.source, "Source", binary_file);
	igl::deserialize(global.direct, "Direct", binary_file);

	igl::deserialize(global.qr_verticals, "QR Verticals", binary_file);
	igl::deserialize(global.qr_facets, "QR Facets", binary_file);
	igl::deserialize(global.qr_colors, "QR Colors", binary_file);
	igl::deserialize(global.hit_matrix, "Hit Matrix", binary_file);

	std::vector<int> id_binary;
	std::vector<float> u_binary;
	std::vector<float> v_binary;
	std::vector<float> t_binary;
	igl::deserialize(id_binary, "Id", binary_file);
	igl::deserialize(u_binary, "U", binary_file);
	igl::deserialize(v_binary, "V", binary_file);
	igl::deserialize(t_binary, "T", binary_file);

	igl::deserialize(global.mode, "model", binary_file);
	igl::deserialize(global.zoom, "Zoom", binary_file);

	global.hitmap.clear();
	global.hitmap.resize(id_binary.size());
	for (int i = 0; i < global.hitmap.size(); i++) {
		global.hitmap[i].id = id_binary[i];
		global.hitmap[i].u = u_binary[i];
		global.hitmap[i].v = v_binary[i];
		global.hitmap[i].t = t_binary[i];
	}

	igl::deserialize(global.carve_depth, "Carving depth", binary_file);

	igl::deserialize(global.under_control, "Under Control", binary_file);

	global.anti_indicatior.clear();
	std::vector<std::vector<int>> anti_indicator_binary;
	igl::deserialize(anti_indicator_binary, "anti indicator", binary_file);
	for (int i = 0; i <anti_indicator_binary.size(); i++) {
		global.anti_indicatior.push_back(Eigen::Vector2i(anti_indicator_binary[i][0],anti_indicator_binary[i][1]));
	}

	global.patch_indicator.clear();

	std::vector<std::vector<int>> patch_indicator_binary;
	igl::deserialize(patch_indicator_binary, "patch indicator", binary_file);

	for (int i = 0; i < patch_indicator_binary.size(); i++)
		global.patch_indicator.push_back(Eigen::Vector4i(patch_indicator_binary[i][0], patch_indicator_binary[i][1], patch_indicator_binary[i][2], patch_indicator_binary[i][3]));


	std::vector<std::vector<int>> indicator_binary_1, indicator_binary_2;
	igl::deserialize(indicator_binary_1, "indicator binary 1", binary_file);
	igl::deserialize(indicator_binary_2, "indicator binary 2", binary_file);
	global.indicator.clear();
	global.indicator.resize(indicator_binary_1.size());
	for (int i = 0; i < indicator_binary_1.size(); i++) {
		for (int j = 0; j < indicator_binary_1.size(); j++) {
			global.indicator[i].push_back(Eigen::Vector2i(indicator_binary_1[i][j], indicator_binary_2[i][j]));
		}
	}

	igl::deserialize(global.seeds, "Seeds", binary_file);
	global.islands.clear();
	global.islands.resize(global.seeds.size());
	for (int i = 0; i < global.seeds.size(); i++) {
		std::string binary_name = "Island" + std::to_string(i);
		igl::deserialize(global.islands[i], binary_name, binary_file);
	}
	
	igl::deserialize(global.component, "Component", binary_file);

	igl::deserialize(global.hole_facet, "Hole facet", binary_file);

	global.holes.clear();
	global.holes.resize(global.component.size());
	for (int i = 0; i < global.component.size(); i++) {
		std::string binary_name = "Hole" + std::to_string(i);
		igl::deserialize(global.holes[i], binary_name, binary_file);
	}

	igl::deserialize(global.rest_verticals, "Rest verticals", binary_file);
	igl::deserialize(global.rest_facets, "Rest facets", binary_file);

}
