#include "QRinfo.h"

void qrcode::serialize(QRinfo & info, std::string & binary_file)
{
	/*Pixels serialization*/
	int size = info.pixels.size();

	std::vector<std::vector<int>>pixels_bianry(size);

	for(int y=0;y<size;y++)
		for (int x = 0; x < size;x++)
			pixels_bianry[y].push_back(info.pixels[y][x].getData());

	igl::serialize(pixels_bianry, "Pixels", binary_file, true);

	/*Block property serialization*/
	igl::serialize(info.block_propertys, "BlockPropertys", binary_file);

	/*Code word serialization*/
	std::vector<bool> codeword_binary;
	for (auto i : (info.codewodrs)) codeword_binary.push_back(i.getStatus());
	igl::serialize(codeword_binary, "Codewords", binary_file);

	/*Pixel property serialization*/
	std::vector<std::vector<int>> priority_binary(size);
	std::vector<std::vector<double>>area_binary(size);
	std::vector<std::vector<double>> adaptable_binary(size);

	for (int y = 0; y < size; y++) {
		for (int x = 0; x < size; x++) {
			priority_binary[y].push_back( info.pixel_propertys[y][x].priority);
			area_binary[y].push_back(info.pixel_propertys[y][x].area);
			adaptable_binary[y].push_back(info.pixel_propertys[y][x].adaptable);
		}
	}

	
	igl::serialize(priority_binary, "Priority", binary_file);
	igl::serialize(area_binary, "Area", binary_file);
	igl::serialize(adaptable_binary, "Adaptable", binary_file);

	igl::serialize(info.scale, "Scale", binary_file);
	igl::serialize(info.border, "Border", binary_file);
	igl::serialize(info.num_of_check_byte, "Check Byte", binary_file);
}

qrcode::QRinfo qrcode::deserialize(std::string & binary_file)
{
	QRinfo info;
	/*Pixel deserialization*/
	std::vector<std::vector<int>>pixels_bianry;
	igl::deserialize(pixels_bianry, "Pixels", binary_file);
	int size = pixels_bianry.size();
	
	info.pixels.clear();
	info.pixels.resize(size);
	for (int y = 0; y < size; y++)
		for (int x = 0; x < size; x++)
			info.pixels[y].push_back(qrgen::Pixel(pixels_bianry[y][x]));

	/*Block Property deserialization*/
	igl::deserialize(info.block_propertys, "BlockPropertys", binary_file);

	/*Code word deserialization*/
	info.codewodrs.clear();
	std::vector<bool> codeword_binary;
	igl::deserialize(codeword_binary, "Codewords", binary_file);
	for (int i = 0; i < codeword_binary.size();i++)  info.codewodrs.emplace_back(codeword_binary[i]);

	/*Pixel property deserialization*/
	std::vector<std::vector<int>> priority_binary(size);
	std::vector<std::vector<double>>area_binary(size);
	std::vector<std::vector<double>> adaptable_binary(size);

	igl::deserialize(priority_binary, "Priority", binary_file);
	igl::deserialize(area_binary, "Area", binary_file);
	igl::deserialize(adaptable_binary, "Adaptable", binary_file);

	info.pixel_propertys.clear();
	info.pixel_propertys.resize(size);
	for (int y = 0; y < size; y++) {
		for (int x = 0; x < size; x++) {
			info.pixel_propertys[y].push_back({ priority_binary[y][x],area_binary[y][x],adaptable_binary[y][x] });
		}
	}
	igl::deserialize(info.scale, "Scale", binary_file);
	igl::deserialize(info.border, "Border", binary_file);
	igl::deserialize(info.num_of_check_byte, "Check Byte", binary_file);
	return info;
}
