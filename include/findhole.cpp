#include "findhole.h"

void qrcode::find_hole(Engine * engine, GLOBAL & global)
{
	Eigen::MatrixXi label;
	qrcode::bwlabel(engine, global.under_control, 4, label);
	
	int index = label.maxCoeff();

	/*Find holes*/
	std::vector<std::vector<int>> facets(index);

	for (int y = 0; y < label.rows(); y++)
		for (int x = 0; x < label.cols(); x++)
			facets[label(y, x) - 1].push_back(global.hitmap[y*label.cols() + x].id);

	for (int i = 0; i < index; i++) {
		std::vector<int> temp = facets[i];
		std::sort(temp.begin(), temp.end());
		temp.erase(std::unique(temp.begin(), temp.end()), temp.end());
		facets[i] = temp;
	}

	global.component.clear();
	/*Connecting component*/
	std::vector<int> white(label.maxCoeff(), 1);
	int goon = 1;
	while (goon == 1) {

		int temp = 0;
		int indexw = 0;
		while (indexw < white.size()) {
			if (white[indexw] == 1) {
				white[indexw] = 0;
				break;
			}
			indexw++;
		}
		global.component.push_back(std::vector<int>{indexw});
		int it = 0;
		while (it < global.component[global.component.size() - 1].size()) {
			
			for (int i = 0; i < white.size(); i++) {

				if (white[i] == 1) {
					std::vector<int> un;

					int first = global.component[global.component.size() - 1][it];
					std::set_intersection(facets[first].begin(), facets[first].end(),
						facets[i].begin(), facets[i].end(), std::inserter(un, un.begin()));
					if (un.size() != 0) {
						white[i] = 0;
						global.component[global.component.size() - 1].push_back(i);
					}
				}
			}
			++it;
		}
		for (auto i : white) temp |= i;
		goon &= temp;
	}

	global.hole_facet.resize(global.component.size());

	/*Unite segments in component and unique*/
	for (int i = 0; i < global.component.size(); i++) {
		for (int j = 0; j < global.component[i].size(); j++) {
			int num = global.component[i][j];
			global.hole_facet[i].insert(global.hole_facet[i].end(), facets[num].begin(), facets[num].end());
		}
		std::vector<int> temp = global.hole_facet[i];
		std::sort(temp.begin(), temp.end());
		temp.erase(std::unique(temp.begin(), temp.end()), temp.end());
		global.hole_facet[i] = temp;
	}

}
