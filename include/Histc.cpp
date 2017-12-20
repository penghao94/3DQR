#include "Histc.h"

int qrcode::histc(Eigen::VectorXf & C)
{
	float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
	if (r<0 || r>C(C.size() - 1)) {
		histc(C);
	}
	else
	{
		//find r in C
		int l = 0;
		int h = C.size() - 1;
		int k = l;
		while ((h - l) > 1)
		{
			k = (h + l) / 2;
			if (r < C(k)) {
				h = k;
			}
			else {
				l = k;
			}
		}
		if (r == C(h) && r != C(C.size() - 1))
		{
			k = h;
		}
		else
		{
			k = l;
		}
		return k;
	}

}
