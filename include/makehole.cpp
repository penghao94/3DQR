#include "makehole.h"

void qrcode::make_hole(GLOBAL & global)
{

	/*Dividing a model facet set into a rest set and many hole sets*/
	std::vector<std::vector<int>::iterator> iter;
	auto facet_id = global.hole_facet;
	for (int i = 0; i < facet_id.size(); i++) iter.push_back(facet_id[i].begin());
	
	std::vector<Eigen::RowVector3i>rest;
	std::vector<std::vector<Eigen::RowVector3i>> hole(facet_id.size());

	for (int index = 0; index < global.model_facets.rows(); index++) {
		bool found = false;// if facet index found in hole's facet set

		for (int j = 0; j < facet_id.size(); j++) {

			if (iter[j] != facet_id[j].end()) {
				if (index == *(iter[j])) {

					hole[j].push_back(global.model_facets.row(index));

					++iter[j];
					found = true;
					break;
				}
			}
		}

		if(found)
			continue;

		rest.push_back(global.model_facets.row(index));
	}

	/*Building a queue to map rest vertical set indices and model vertical set indices*/
	std::vector<int> queue;
	for (int i = 0; i < rest.size(); i++)
		for (int j = 0; j < 3; j++)
			queue.push_back(rest[i](j));

	std::sort(queue.begin(), queue.end());
	queue.erase(std::unique(queue.begin(), queue.end()), queue.end());

	/*rest vertical set*/
	global.rest_verticals.resize(queue.size(), 3);
	for (int i = 0; i < queue.size(); i++)
		global.rest_verticals.row(i) = global.model_vertices.row(queue[i]);

	/*rest facet set*/
	global.rest_facets.resize(rest.size(), 3);

	for (int i = 0; i < rest.size(); i++)
		for (int j = 0; j < 3; j++)
			global.rest_facets(i, j) = std::distance(queue.begin(),std::find(queue.begin(), queue.end(), rest[i](j)))+global.qr_verticals.rows();

	/*rest hole sets*/
	global.holes.resize(hole.size());

	for (int i = 0; i < hole.size(); i++) {
		qrcode::eList *elist = new qrcode::eList();
		for (int j = 0; j < hole[i].size(); j++) {
			elist->add(hole[i][j](0), hole[i][j](1), 0);
			elist->add(hole[i][j](1), hole[i][j](2), 0);
			elist->add(hole[i][j](2), hole[i][j](0), 0);
		}
		elist->matrix(global.holes[i]);
		for (int k = 0; k < global.holes[i].rows(); k++) {
			global.holes[i](k, 0) = std::distance(queue.begin(), std::find(queue.begin(), queue.end(), global.holes[i](k, 0)));
			global.holes[i](k, 1) = std::distance(queue.begin(), std::find(queue.begin(), queue.end(), global.holes[i](k, 1)));
		}
	}
	









}
