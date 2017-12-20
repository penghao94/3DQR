#include "random_points_on_spherical_mesh.h"

void qrcode::random_points_on_spherical_mesh(const Eigen::Vector3f & origin, const Eigen::MatrixXd & verticles, const Eigen::MatrixXi & facets, int samples, Eigen::MatrixXf & result)
{
	Eigen::MatrixXf _V = (verticles.cast<float>() - origin.transpose().replicate(verticles.rows(), 1)).rowwise().normalized();


	if (samples > 400) {
		std::vector<Eigen::RowVector3f> temp;
		int count = 0;
		// http://www.altdevblogaday.com/2012/05/03/generating-uniformly-distributed-points-on-sphere/
		while (count < samples) {
			float z = abs((float)rand() / (float)RAND_MAX*2.0 - 1.0);
			float t = (float)rand() / (float)RAND_MAX*2.0*igl::PI;

			float r = sqrt(1.0 - z*z);
			float x = r * cos(t);
			float y = r * sin(t);
			igl::Hit hit;
			if (ray_mesh_intersect(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(x, y, z), _V, facets, hit)) {
				temp.emplace_back(x, y, z);
				count++;
			}

		}
		result.resize(samples, 3);
		for (int i = 0; i < samples; i++) result.row(i) = temp[i];
	}
	else {
		Eigen::SparseMatrix<float> B;
		Eigen::VectorXi FI;
		igl::random_points_on_mesh(samples, _V, facets, B, FI);
		result = (B*_V).rowwise().normalized();
	}

	







	/*Eigen::MatrixXf _V = (verticles.cast<float>() - origin.transpose().replicate(verticles.rows(), 1)).rowwise().normalized();
	result.resize(samples, 3);
	Eigen::MatrixXf Angle(facets.rows(), 10), G(facets.rows(), 3);
	Eigen::Vector3f A, B, C;
	for (int i = 0; i < facets.rows(); i++) {
		A = _V.row(facets(i, 0)).transpose();
		B = _V.row(facets(i, 1)).transpose();
		C = _V.row(facets(i, 2)).transpose();
		Angle(i, 0) = acos((C.cross(A)).dot(B.cross(A)) / ((C.cross(A)).norm()*(B.cross(A)).norm()));
		Angle(i, 1) = acos((A.cross(B)).dot(C.cross(B)) / ((A.cross(B)).norm()*(C.cross(B)).norm()));
		Angle(i, 2) = acos((B.cross(C)).dot(A.cross(C)) / ((B.cross(C)).norm()*(A.cross(C)).norm()));
		Angle(i, 3) = acos(B.dot(C));
		Angle(i, 4) = acos(C.dot(A));
		Angle(i, 5) = acos(A.dot(B));
		Angle(i, 6) = Angle(i, 0) + Angle(i, 1) + Angle(i, 2) - igl::PI;
		Angle(i, 7) = (C.cross(A)).dot(B.cross(A)) / ((C.cross(A)).norm()*(B.cross(A)).norm());
		Angle(i, 8) = sqrt(1 - Angle(i, 7)*Angle(i, 7));
		Angle(i, 9) = A.dot(B);
		G.row(i) = ((C - A.dot(C)*A)).normalized();

	}
	Eigen::VectorXf Area = Angle.col(6);
	Area /= Area.sum();
	Eigen::VectorXf CA, A0(Area.size() + 1);
	A0(0) = 0;
	A0.bottomRightCorner(Area.size(), 1) = Area;
	// Even faster would be to use the "Alias Table Method"
	igl::cumsum(A0, 1, CA);
	//std::cout << CA.transpose() << std::endl;
	for (int i = 0; i < samples; i++) {
		int r = qrcode::histc(CA);
		/ *if(p==6449)
		cout << r << endl;* /
		float r1 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
		float r2 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
		float _A = r1*Angle(r, 6);

		float s = sin(_A - Angle(r, 0));
		float t = cos(_A - Angle(r, 0));
		float u = t - Angle(r, 7);
		float v = s + Angle(r, 8)*Angle(r, 9);
		float q = ((v*t - u*s)* Angle(r, 7) - v) / (v*s + u*t)* Angle(r, 8);
		Eigen::RowVectorXf a = _V.row(facets(r, 0));
		Eigen::RowVectorXf b = _V.row(facets(r, 1));
		Eigen::RowVectorXf _C = q*a + sqrt(1 - q*q)*G.row(r);
		float z = 1 - r2*(1 - _C.dot(b));
		result.row(i) = z*b + sqrt(1 - z*z)*((_C - _C.dot(b)*b)).normalized();
	}*/
}
