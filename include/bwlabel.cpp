#include "bwlabel.h"

void qrcode::bwlabel(Engine * engine, Eigen::MatrixXi & bw, int connectivity, Eigen::MatrixXi & label)
{
	assert(!(connectivity != 4 && connectivity != 8) && "connectivity must be 4 or 8!!!");

	Eigen::MatrixXf BW = bw.cast<float>();

	igl::matlab::mlsetmatrix(&engine, "bw",BW);
	if (connectivity == 8)
		igl::matlab::mleval(&engine, "label=bwlabel(bw,8);");
	else if (connectivity == 4)
		igl::matlab::mleval(&engine, "label=bwlabel(bw,4);");

	Eigen::MatrixXf LABEL;
	igl::matlab::mlgetmatrix(&engine, "label", LABEL);
	label = LABEL.cast<int>();

}

void qrcode::bwbound(Eigen::MatrixXi & label, std::vector<Eigen::MatrixXi>& bound)
{

	Eigen::MatrixXi kinds,edges;
	igl::unique(label, kinds);

	int index = kinds.rows() - 1;
	std::vector<std::vector<int>> raw(index);
	std::vector<Eigen::MatrixXi> temp(index);
	bound.resize(index);

	for (int i = 0; i < label.rows(); i++)
		for (int j = 0; j < label.cols(); j++)
			if (label(i, j) != 0) raw[label(i, j) - 1].push_back(i*label.cols() + j);

	for (int i = 0; i < index; i++) {
		qrcode::eList *elist = new qrcode::eList();
		for (int j = 0; j < raw[i].size(); j++) {
			int a = raw[i][j];
			int b = raw[i][j] + label.cols();
			int c = raw[i][j] + 1;
			int d = raw[i][j] + label.cols() + 1;

			elist->add(a, b, 0);
			elist->add(b, d, 0);
			elist->add(d, c, 0);
			elist->add(c, a, 0);
		}
		elist->matrix(edges);
		temp[i] = edges;
		delete elist;
	}
	bwdegenerate(temp, label.cols(), bound);

}


void qrcode::bwdegenerate(std::vector<Eigen::MatrixXi>& bound, int col, std::vector<Eigen::MatrixXi>& decrement)
{
	decrement.clear();
	decrement.resize(bound.size());

	for (int i = 0; i < bound.size(); i++) {

		std::vector<std::vector<int>> info(bound[i].rows());
		std::vector<bool>visit(bound[i].rows(), false);

		for (int j = 0; j < bound[i].rows(); j++) {

			int start_y = bound[i](j, 0) / col;
			int start_x = bound[i](j, 0) % col;

			int end_y = bound[i](j, 1) / col;
			int end_x = bound[i](j, 1) % col;

			int diff_y = end_y - start_y;
			int diff_x = end_x - start_x;

			info[j] = { start_y,start_x,end_y,end_x,diff_y,diff_x,j };
		}




		std::vector<Eigen::Vector2i> queue;

		bool no_left = false;

		while (!no_left) {

			int min = (1 << 31)-1;
			int index;

			for (int j = 0; j < bound[i].rows(); j++) {
				if (visit[j] == 0 && min > bound[i](j, 0)) {
					index = j;
					min = bound[i](j, 0);
				}	
			}

			std::vector<std::vector<int>>::iterator origin = info.begin() + index;
			std::vector<std::vector<int>>::iterator next;
			std::vector<std::vector<int>>::iterator other;

			next = std::find_if(info.begin(), info.end(), [&origin, &visit](std::vector<int>&obj) {
				return(obj[0] == (*origin)[2] && obj[1] == (*origin)[3] && visit[obj[6]] == false);
			});

			visit[(*next)[6]] = true;// this edge has been visited

			Eigen::Vector2i turning_point;

			turning_point = Eigen::Vector2i((*origin)[0], (*origin)[1]);

			queue.push_back(turning_point);// edge begin

			Eigen::Vector2i diff = Eigen::Vector2i((*origin)[4], (*origin)[5]);

			bool another = false;

			while (next != origin) {



				turning_point = (turning_point.array() + diff.array()).matrix();

				std::vector<int> temp = *next;

				if (!(temp[4] == diff(0) && temp[5] == diff(1))) {
					queue.push_back(turning_point);//edge end
					queue.push_back(turning_point);//edge start
					diff << temp[4], temp[5];

				}
				else if (another) {
					queue.push_back(turning_point);//edge end
					queue.push_back(turning_point);//edge start
				}

				another = false;

				next = std::find_if(info.begin(), info.end(), [&temp, &visit](std::vector<int>&obj) {
					return(obj[0] == temp[2] && obj[1] == temp[3] && visit[obj[6]] == false);
				});

				visit[(*next)[6]] = true;

				other = std::find_if_not(info.begin(), info.end(), [&temp, &next, &info](std::vector<int>&obj) {
					return(!(obj[0] == temp[2] && obj[1] == temp[3]) || obj[6] == static_cast<int>(next - info.begin()));
				});// if other is not empty,that means there is an begin point for another edge
				if (other != info.end()) another = true;
			}

			turning_point = (turning_point.array() + diff.array()).matrix();
			queue.push_back(turning_point);//edge end

			bool all_visit = true;
			for (int j = 0; j < info.size(); j++)
				all_visit &= visit[j];
			no_left |= all_visit;

		}


		int size = queue.size();
		decrement[i].resize(size / 2, 2);
		for (int j = 0; j < size / 2; j++) {
			decrement[i].row(j) << queue[2 * j](0)*col + queue[2 * j](1), queue[2 * j + 1](0)*col + queue[2 * j + 1](1);
		}
	}
}
