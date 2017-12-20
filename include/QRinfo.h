// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef QRINFO_H_
#define QRINFO_H_
#include<Eigen/dense>
#include<igl/serialize.h>
#include "Qrcoder.h"
#include "PixelProperty.h"
#include "CodeWord.h"
namespace qrcode {
	struct QRinfo
	{
		std::vector<std::vector<qrgen::Pixel>> pixels;
		std::vector<int> block_propertys;
		std::vector<std::vector<qrgen::PixelProperty>> pixel_propertys;
		std::vector<qrgen::CodeWord> codewodrs;
		int scale;
		int border;
		int num_of_check_byte;
	};
	void serialize(QRinfo &info, std::string &binary_file);
	QRinfo deserialize(std::string &bianray_file);
}

#endif // !QRINFO_H_

