
// This project is about 3d QR code generating,see more details at https://github.com/swannyPeng/3dqrcode_libigl
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#define ENABLE_SERIALIZATION
#include<string>
#include <igl/viewer/Viewer.h>
#include <nanogui/nanogui.h>
#include <igl/matlab/matlabinterface.h>
#include <igl/Timer.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/png/writePNG.h>
#include "global.h"
#include "readQR.h"
#include "Serialization.h"
#include "image_onto_mesh.h"
#include "Strategy.h"
#include "qr_mesh.h"
#include "carving_down.h"
#include "findhole.h"
#include "makehole.h"
#include "fixhole.h"
#include "directional_light.h"
#include "reflaction.h"
/*global parameters */



int main(int argc, char *argv[])
{
	/*Global Parameter*/
	qrcode::GLOBAL g;
	std::string scene_file = "SCENE";
	std::string data_file = "DATA";
	/*Initiate viewer*/
	igl::viewer::Viewer viewer;
	viewer.core.show_lines = false;
	
	/*Matlab instance*/
	Engine *engine;
	igl::matlab::mlinit(&engine);

	/*Timer*/
	igl::Timer timer;

	/*UI and widget*/
	viewer.callback_init = [&](igl::viewer::Viewer &viewer) {
		/*Window*/
		viewer.ngui->addWindow(Eigen::Vector2i(250, 15), "QR code");

		viewer.ngui->addGroup("Mesh and QR Options");
		/*Load and save mesh*/
		viewer.ngui->addButton("Load mesh", [&]() {
		
			viewer.data.clear();

			std::string file_name = "";
			file_name = igl::file_dialog_open();

			if (file_name != "") {
				if (igl::readOBJ(file_name, g.model_vertices, g.model_facets)) {

					viewer.data.set_mesh(g.model_vertices, g.model_facets);

					g.model_colors = Eigen::RowVector3d(1.0, 1.0, 1.0).replicate(g.model_vertices.rows(), 1);
					viewer.data.set_colors(g.model_colors);
				}
				else {
					assert(false && "Failed to load mesh!!!");
				}
			}
		});

		viewer.ngui->addButton("Save mesh", [&]() {
			std::string file_name = "";
			file_name= igl::file_dialog_save();
			if (file_name != "") {
				igl::writeOBJ(file_name, viewer.data.V, viewer.data.F);
			}
		});

		viewer.ngui->addButton("Load QR code", [&]() {
			std::string file_name = "";
			file_name = igl::file_dialog_open();
			if (file_name != "") {
				timer.start();
				g.info = qrcode::readQR(engine, file_name);
				std::cout << "Version:" << static_cast<int>((g.info.pixels.size() - 17) / 4) << std::endl;
				std::cout << "Load QR time: " << timer.getElapsedTimeInSec() << "s" << std::endl;

			}
		});

		viewer.ngui->addButton("Screen capture", [&]() {
			std::string file_name = "";
			file_name = igl::file_dialog_save();
			if (file_name != "") {

				// Allocate temporary buffers for 1280x800 image
				Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(1280, 800);
				Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(1280, 800);
				Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(1280, 800);
				Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(1280, 800);

				// Draw the scene in the buffers
				viewer.core.draw_buffer(viewer.data, viewer.opengl, false, R, G, B, A);
				// Save it to a PNG
				igl::png::writePNG(R, G, B, A, file_name);
			}
		});

		viewer.ngui->addButton("Serialization", [&]() {
			viewer.save_scene();
			qrcode::serialize(g, data_file);
		});

		viewer.ngui->addButton("Deserialization", [&]() {
			std::string str = igl::file_dialog_open();
			viewer.load_scene(scene_file);
			qrcode::deserialize(g, data_file);
		});

		viewer.ngui->addButton("Image onto mesh", [&]() {
			timer.start();
			g.mode = viewer.core.model;
			g.zoom = viewer.core.model_zoom*viewer.core.camera_zoom;
			
			qrcode::image_onto_mesh(viewer, g);
			qrcode::control_strategy(g);
			qrcode::generate_qr_mesh(engine, g);
			std::cout << "Image onto mesh time: " << timer.getElapsedTimeInSec() << "s" << std::endl;

			Eigen::MatrixXd V(g.qr_verticals.rows()+g.model_vertices.rows(),3), C(g.qr_facets.rows() + g.model_facets.rows(), 3);
			Eigen::MatrixXi F(g.qr_facets.rows()+g.model_facets.rows(),3);
			Eigen::MatrixXd qr_verticals = g.qr_verticals;
			qr_verticals.col(2) = (qr_verticals.col(2).array() + 0.01).matrix();
			V.block(0, 0, g.qr_verticals.rows(), 3) = qr_verticals;
			
			V.block(g.qr_verticals.rows(), 0, g.model_vertices.rows(), 3) = g.model_vertices;

			F.block(0, 0, g.qr_facets.rows(), 3) = g.qr_facets;
			F.block(g.qr_facets.rows(), 0, g.model_facets.rows(), 3) = (g.model_facets.array()+g.qr_verticals.rows()).matrix();
			C.setConstant(1);
			C.block(0, 0, g.qr_colors.rows(), 3) = g.qr_colors;

			viewer.data.clear();
			viewer.data.set_mesh(V, F);



			viewer.data.set_colors(C);
		});
		viewer.ngui->addButton("Test carving", [&]() {
			int scale = g.info.scale;
			Eigen::MatrixXi modules, functions,Modules;
			qrcode::pixel_to_matrix(g.info.pixels, g.info.border, modules, functions);
			int size = (modules.rows() - 1)*scale;
			Modules.resize(size, size);
			for (int y = 0; y < modules.rows() - 1; y++)
				for (int x = 0; x < modules.cols() - 1; x++) 
					for (int u = 0; u < scale; u++) 
						for (int v = 0; v < scale; v++) 
							Modules(y*scale + u, x*scale + v) = modules(y, x);
			
			g.carve_depth.setZero(g.qr_verticals.rows());
			for (int y = 0; y < size; y++) {
				for (int x = 0; x < size; x++) {
					if (Modules(y, x) == 1) {
						qrcode::patch(y, x, g, Eigen::Vector4d(0.001, 0.001, 0.001, 0.001));

					}
				}
			}
			Eigen::MatrixXd result;
			qrcode::carving_down(g, result);
			viewer.data.clear();
			viewer.data.set_mesh(result, g.qr_facets);
			viewer.data.set_colors(g.qr_colors);

		});

		viewer.ngui->addButton("Test make hole",[&]() {
			qrcode::find_hole(engine, g);
			qrcode::make_hole(g);
			qrcode::fix_hole(engine, g);

			Eigen::MatrixXd V(g.qr_verticals.rows() + g.rest_verticals.rows(),3);
			V.block(0, 0, g.qr_verticals.rows(), 3) = g.qr_verticals;
			V.block(g.qr_verticals.rows(), 0, g.rest_verticals.rows(), 3) = g.rest_verticals;
			
			int size = g.qr_facets.rows();
			for (int i = 0; i < g.component.size(); i++) size += g.patches[i].rows();
			Eigen::MatrixXi F(size, 3);
			F.block(0, 0, g.qr_facets.rows(), 3) = g.qr_facets;
			F.block(g.qr_facets.rows(),0, g.patches[0].rows(), 3) = g.patches[0];

			viewer.data.clear();
			viewer.data.set_mesh(V,F);
		});
		
		g.latitude_upper = 45;
		viewer.ngui->addVariable("Upper latitude", g.latitude_upper);

		g.latitude_lower = 40;
		viewer.ngui->addVariable("Lower latitude", g.latitude_lower);

		g.longitude = 180;
		viewer.ngui->addVariable("Longitude", g.longitude);

		g.distance = 1000;
		viewer.ngui->addVariable("Distance", g.distance);


		viewer.ngui->addButton("Direction light", [&]() {
			Eigen::MatrixXd V;
			Eigen::MatrixXi F;
			qrcode::directional_light(viewer, engine, g, V, F);

			viewer.data.clear();
			viewer.data.set_mesh(V, F);
		});
		viewer.ngui->addButton("reflaction", [&]() {
			Eigen::MatrixXd V;
			Eigen::MatrixXi F;
			qrcode::reflaction(g, V, F);

			viewer.data.clear();
			viewer.data.set_mesh(V, F);
		
		});

		viewer.screen->performLayout();
		return false;
	};

	viewer.launch();

}
