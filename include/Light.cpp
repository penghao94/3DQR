#include "Light.h"

void qrcode::light(Eigen::MatrixXd & verticles, Eigen::MatrixXi & facets, Eigen::VectorXf & origin, std::vector<Eigen::Vector3f>& destinations, Eigen::Matrix<bool, Eigen::Dynamic, 1>& result)
{
	igl::embree::EmbreeIntersector ei;
	int n = destinations.size();

	result.resize(n);

	ei.init(verticles.cast<float>(), facets);

	const auto &shoot_ray = [&ei](
		const Eigen::Vector3f& s,
		const Eigen::Vector3f& dir)->bool
	{
		igl::Hit hit;
		const float tnear = 1e-3f;
		return ei.intersectRay(s, dir, hit, tnear);
	};

	// shoot rays
	const Eigen::Vector3f d = origin;
	const auto &inner = [&](const int p)
	{
		Eigen::Vector3f s = destinations[p];
		Eigen::Vector3f dir = (d - s).normalized();
		if (!shoot_ray(s, dir))
			result(p) = true;
		else
			result(p) = false;
	};
	igl::parallel_for(n, inner, 1000);
	//ei.deinit();
	//ei.global_deinit();
}
