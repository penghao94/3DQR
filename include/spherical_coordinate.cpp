#include "spherical_coordinate.h"

Eigen::MatrixXf qrcode::rot(const Eigen::Vector3f & before, const Eigen::Vector3f & after)
{
	Eigen::Vector3f a, b, c;
	a = after;
	b = before;
	c = after;
	c = c.cross(b);
	if (c != Eigen::Vector3f(0, 0, 0))
		c = -c / c.norm();
	float angle = acos(a.dot(b));
	Eigen::MatrixXf I(3, 3), K(3, 3);
	I.setIdentity();
	K << 0, -c(2), c(1),
		c(2), 0, -c(0),
		-c(1), c(0), 0;
	return (I + sin(angle)*K + (1 - cos(angle))*K*K);
}

float qrcode::Box(const Eigen::Vector3f & origin, const Eigen::MatrixXd & V, const Eigen::MatrixXf & rot, Eigen::MatrixXf & Box)
{
	#define PI 3.1415926535898

	// Spherical coordinate system
	namespace bg = boost::geometry;
	typedef	bg::model::point<float, 2, bg::cs::spherical_equatorial<bg::radian>> Point;
	typedef bg::model::polygon<Point> Polygon;

	Polygon polygon;
	Eigen::MatrixXf _V(V.rows(), 3);
	_V = (V.cast<float>() - origin.transpose().replicate(V.rows(), 1)).rowwise().normalized();
	_V.conservativeResize(_V.rows() + 1, 3);
	_V.row(_V.rows() - 1) << _V.row(0);

	//cout << _V << endl;
	Eigen::MatrixXf B(V.rows(), 2);

	//spherical qrcode boundary
	for (int i = 0; i < _V.rows(); i++) {
		float x = _V(i, 0);
		float y = _V(i, 1);
		float z = _V(i, 2);
		float w = sqrt(pow(x, 2) + pow(y, 2));

		if (y >= 0) {
			if (x >= 0)
				polygon.outer().push_back(Point(asin(y / w), asin(z)));
			else
				polygon.outer().push_back(Point(PI - asin(y / w), asin(z)));
		}
		else {
			if (x >= 0)
				polygon.outer().push_back(Point(asin(y / w), asin(z)));
			else
				polygon.outer().push_back(Point(-PI - asin(y / w), asin(z)));
		}
		if (i<_V.rows() - 1)
			B.row(i) << polygon.outer()[i].get<0>(), polygon.outer()[i].get<1>();
	}
	//cout << bg::wkt(polygon) << endl;

	for (int i = 1; i < polygon.outer().size(); i++) {

		if (polygon.outer()[i - 1].get<0>() < 0 && polygon.outer()[i].get<0>() > 0)
		{
			B.conservativeResize(B.rows() + 4, 2);
			int n = B.rows();
			//y=(y2-y1)/(2pi-x2+x1)*(pi+x1)+y1
			float b = (polygon.outer()[i].get<1>() - polygon.outer()[i - 1].get<1>()) / (2 * PI - polygon.outer()[i].get<0>() + polygon.outer()[i - 1].get<0>())*(PI + polygon.outer()[i - 1].get<0>()) + polygon.outer()[i - 1].get<1>();
			polygon.outer().insert(polygon.outer().begin() + i, Point(-PI, b));
			B.row(n - 4) << -PI, b;
			polygon.outer().insert(polygon.outer().begin() + i, Point(-PI, 1.57079632f));
			B.row(n - 3) << -PI, 1.57079632f;
			polygon.outer().insert(polygon.outer().begin() + i, Point(PI, 1.57079632f));
			B.row(n - 2) << PI, 1.57079632f;
			polygon.outer().insert(polygon.outer().begin() + i, Point(PI, b));
			B.row(n - 1) << PI, b;
			break;
		}
	}
	//cout<<B<<endl;
	float area = bg::area(polygon);
	//cout << area << endl;
	Box.resize(2, 2);
	Box.row(0) << B.colwise().maxCoeff();
	Box.row(1) << B.colwise().minCoeff();
	//cout << Box << endl;
	//
	B.resize(0, 0);
	Eigen::MatrixXf R(2, 3);
	Eigen::MatrixXf roti = rot.inverse();
	//cout << rot << endl;
	//cout << roti << endl;
	R << cos(Box(0, 1))*cos(Box(0, 0)), cos(Box(0, 1))*sin(Box(0, 0)), sin(Box(0, 1)),
		cos(Box(1, 1))*cos(Box(1, 0)), cos(Box(1, 1))*sin(Box(1, 0)), sin(Box(1, 1));
	//cout << R << endl;
	R = (roti*R.transpose()).transpose();
	//cout << R << endl;
	for (int i = 0; i < R.rows(); i++) {
		float x = R(i, 0);
		float y = R(i, 1);
		float z = R(i, 2);
		float w = sqrt(pow(x, 2) + pow(y, 2));

		if (y >= 0) {
			if (x >= 0)
				Box.row(i) << asin(y / w), asin(z);
			else
				Box.row(i) << PI - asin(y / w), asin(z);
		}
		else {
			if (x >= 0)
				Box.row(i) << asin(y / w), asin(z);
			else
				Box.row(i) << -PI - asin(y / w), asin(z);
		}
	}
	if (Box(0, 0) < Box(1, 0))
		Box(0, 0) += 2 * PI;

	polygon.clear();
	R.resize(0, 0);
	roti.resize(0, 0);
	return area;
}
