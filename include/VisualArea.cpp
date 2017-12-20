#include "VisualArea.h"

void qrcode::visualarea(Engine * engine, Eigen::MatrixXi & modules, Eigen::MatrixXi & functions, std::vector<std::vector<qrgen::PixelProperty>>& pixel_propertys)
{
	int size = modules.rows();

	Eigen::MatrixXi label;
	qrcode::bwlabel(engine, modules, 4, label);
	std::vector<Eigen::MatrixXi> bounds;
	qrcode::bwbound(label, bounds);
	assert(!(bounds.size() != label.maxCoeff()) && "label error!!!");

	pixel_propertys.resize(size);
	for (int i = 0; i < size; i++) pixel_propertys[i].resize(size);
	for (int y = 0; y < size - 1; y++) {
		for (int x = 0; x < size - 1; x++) {
			if (modules(y, x) > 0) {
				std::vector<Eigen::Vector2d> bound;
				qrcode::raw_to_visible_polygon(Eigen::RowVector2d(y + 0.5, x + 0.5), bounds[label(y, x) - 1], 1, size, bound);

				boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> poly;
				for (int i = 0; i < bound.size(); i++) 	poly.outer().emplace_back(bound[i](0), bound[i](1));
				pixel_propertys[y][x].area = boost::geometry::area(poly);
				
			}
		}
	}
}
