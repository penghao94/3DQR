#include "ambient_occlusion.h"

void qrcode::ambient_occlusion(Eigen::MatrixXd & verticles, Eigen::MatrixXi & facets, std::vector<Eigen::Vector3f> &position, std::vector<Eigen::Vector3f> &normal, int samples, Eigen::VectorXf & result)
{
	igl::embree::EmbreeIntersector ei;

	ei.init(verticles.cast<float>(), facets);

	const auto & shoot_ray = [&ei](
		const Eigen::Vector3f& s,
		const Eigen::Vector3f& dir)->bool
	{
		igl::Hit hit;
		const float tnear = 1e-3f;
		return ei.intersectRay(s, dir, hit, tnear);
	};

	const int n = position.size();
	result.resize(n);

	Eigen::MatrixXf D = igl::random_dir_stratified(samples).cast<float>();
	const auto & inner = [&position,&normal,&samples,&D,&result,&shoot_ray](const int p)
	{
		const Eigen::Vector3f origin = position[p];
		const Eigen::Vector3f direct = normal[p];

		float a = 0;
		float b = 0;

		for (int s = 0; s < samples; s++)
		{
			Eigen::Vector3f d = D.row(s);
			if (d.dot(direct) < 0)
			{
				// reverse ray
				d *= -1;
			}
			float c = d.dot(direct);
			if (!shoot_ray(origin, d))
			{
				b += c;
			}
			a += c;
		}
		result(p) = b / a;
	};
	igl::parallel_for(n, inner, 1000);


	//ei.global_deinit();
}

void qrcode::ambient_occlusion( Eigen::MatrixXd & verticles, Eigen::MatrixXi & facets, std::vector<Eigen::Vector3f> &position, std::vector<Eigen::Vector3f> &normal,
	std::vector<qrcode::SMesh>& patch, Eigen::VectorXf & result)
{
	static float FLT_LARGE = 1.844E18f;
	igl::embree::EmbreeIntersector ei;

	ei.init(verticles.cast<float>(), facets);

	const auto & shoot_ray = [&ei](
		const Eigen::Vector3f& s,
		const Eigen::Vector3f& dir)->bool
	{
		//std::cout << dir.transpose() << std::endl;
		igl::Hit hit;
		const float tnear = 1e-3f;
		return ei.intersectRay(s, dir, hit, tnear);
	};

	const int n = position.size();

	std::vector<Eigen::MatrixXf> points(n);

	Eigen::VectorXf area(n);
	Eigen::VectorXf ratio(n);

	Eigen::Vector3f axis = Eigen::Vector3f(0.f, 0.f, 1.f);
	
	const auto rander = [&position, &normal, &patch, &points, &axis, &area, &ratio](const int p) {
		//std::cout << p << std::endl;
		
		const Eigen::Vector3f origin = position[p];

		const Eigen::MatrixXd v = patch[p].V;
		const Eigen::MatrixXi f = patch[p].F;
		//std::cout << v << std::endl;
		//std::cout << f << std::endl;

		const Eigen::Vector3f direct = normal[p];
		const Eigen::MatrixXf rot = qrcode::rot(direct, axis);

		//Eigen::MatrixXf box(2, 2);

		//area(p) = qrcode::Box(origin, v, rot, box);

		Eigen::MatrixXf _V = (v.cast<float>() - origin.transpose().replicate(v.rows(), 1)).rowwise().normalized();
		Eigen::MatrixXf Angle(f.rows(), 3);
		Eigen::Vector3f A, B, C;

		float sum = 0;
		for (int i = 0; i < f.rows(); i++) {
			A = _V.row(f(i, 0)).transpose();
			B = _V.row(f(i, 1)).transpose();
			C = _V.row(f(i, 2)).transpose();
			//std::cout << (C.cross(A)).dot(B.cross(A)) / ((C.cross(A)).norm()*(B.cross(A)).norm()) << " ";
			Angle(i, 0) = acos(refine((C.cross(A)).dot(B.cross(A)) / ((C.cross(A)).norm()*(B.cross(A)).norm())));
			//std::cout << (A.cross(B)).dot(C.cross(B)) / ((A.cross(B)).norm()*(C.cross(B)).norm()) << " ";
			Angle(i, 1) = acos(refine((A.cross(B)).dot(C.cross(B)) / ((A.cross(B)).norm()*(C.cross(B)).norm())));
			//std::cout << (B.cross(C)).dot(A.cross(C)) / ((B.cross(C)).norm()*(A.cross(C)).norm()) << std::endl;
			Angle(i, 2) = acos(refine((B.cross(C)).dot(A.cross(C)) / ((B.cross(C)).norm()*(A.cross(C)).norm())));
			float temp = Angle(i, 0) + Angle(i, 1) + Angle(i, 2) - igl::PI;
			if (temp < 0.0f)
				sum += 0;
			else if (temp > igl::PI) 
				sum += igl::PI;
			else 
				sum += temp;

		}

		area(p) = Angle.sum() - f.rows()*igl::PI;
		if (area(p) > 6.28315f) area(p) = 6.28315f;
		if (area(p) < 2 * igl::PI*1e-5f) area(p) = 2 * igl::PI*1e-5f;
		if (area(p) == FP_NAN) area(p) = 6.28315f*static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

		const int sample = std::round(5* log10(area(p) / 2 / igl::PI / 1e-5));
		ratio(p) = static_cast<float>(igl::PI / area(p)*sample*sample);
		qrcode::random_points_on_spherical_mesh(origin, v, f, sample*sample, points[p]);
	};

	igl::parallel_for(n, rander, 1000);
	//for (int i = 0; i < n; i++) rander(i);
	result.resize(n);
	
	const auto & inner = [&position, &normal, &points, &result, &ratio, &area, &shoot_ray](const int p)
	{
		const Eigen::Vector3f origin = position[p];
		const Eigen::Vector3f direct = normal[p];
		float a = 0;
		float b = 0;
		for (int s = 0; s < points[p].rows(); s++)
		{
			Eigen::Vector3f d = points[p].row(s).transpose();
			float c = d.dot(direct);


			if (!shoot_ray(origin, d))
			{
				b += c;
				a += 1;
			}
		}
		result(p) = b / ratio(p);

	};

	igl::parallel_for(n, inner, 1000);
}
//to solve the accuracy of acos
float qrcode::refine(float r)
{
	if (abs(r) - 1.0f > 0) {
		if (r > 0.0f)
			return 0.999999f;
		else
			return -0.999999f;
	}
}
