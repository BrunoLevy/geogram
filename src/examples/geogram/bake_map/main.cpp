/*
 *  Copyright (c) 2000-2022 Inria
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *  * Neither the name of the ALICE Project-Team nor the names of its
 *  contributors may be used to endorse or promote products derived from this
 *  software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Contact: Bruno Levy
 *
 *     https://www.inria.fr/fr/bruno-levy
 *
 *     Inria,
 *     Domaine de Voluceau,
 *     78150 Le Chesnay - Rocquencourt
 *     FRANCE
 *
 */

#include <geogram/basic/common.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/basic/file_system.h>
#include <geogram/basic/stopwatch.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_baking.h>
#include <geogram/mesh/mesh_remesh.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/parameterization/mesh_atlas_maker.h>
#include <geogram/image/image.h>
#include <geogram/image/image_library.h>
#include <geogram/image/morpho_math.h>


int main(int argc, char** argv) {
    using namespace GEO;
    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
    CmdLine::import_arg_group("standard");
    CmdLine::import_arg_group("algo");
    std::vector<std::string> filenames;

    CmdLine::declare_arg("verbose", false, "display messages");
    CmdLine::declare_arg("image_resolution",1024, "in pixels (square image)");
    CmdLine::declare_arg(
	"parameterizer", "LSCM", "one of none, projection, LSCM, ABF++");
    CmdLine::declare_arg("map", "normal", "one of normal, ambient");
    CmdLine::declare_arg("nb_pts",0,"number of points or 0 for keep same mesh");
    CmdLine::declare_arg(
	"nb_dilations",2,"number of dilations (to hide the seams)"
    );

    if(!CmdLine::parse(argc, argv, filenames, "input_meshfile")) {
        return 1;
    }

    geo_assert(filenames.size() == 1);
    std::string highres_filename = filenames[0];
    std::string param_filename = FileSystem::base_name(highres_filename) +
	"_param.obj";
    std::string image_filename =
	FileSystem::base_name(highres_filename) + "_param_normals.png";

    Mesh highres;
    if(!mesh_load(filenames[0], highres)) {
	Logger::err("Baker") << "Could not load " << filenames[0] << std::endl;
	return 1;
    }

    Mesh lowres;
    bool remeshed = false;
    {
	index_t nb_pts = CmdLine::get_arg_uint("nb_pts");
	if(nb_pts == 0) {
	    lowres.copy(highres);
	} else {
	    remeshed = true;
	    remesh_smooth(highres, lowres, nb_pts);
	}
    }

    bool verbose = CmdLine::get_arg_bool("verbose");
    index_t resolution = CmdLine::get_arg_uint("image_resolution");

    std::string param_str = CmdLine::get_arg("parameterizer");
    if(param_str != "none") {
	ChartParameterizer param = PARAM_LSCM;
	if(param_str == "LSCM") {
	    param = PARAM_LSCM;
	} else if(param_str == "ABF++") {
	    param = PARAM_ABF;
	} else if(param_str == "projection") {
	    param = PARAM_PROJECTION;
	} else {
	    Logger::err("Baker") << "Invalid parameterizer:" << param_str
				 << std::endl;
	    exit(-1);
	}
	ChartPacker pack = PACK_XATLAS;
	mesh_make_atlas(lowres, 45.0, param, pack, verbose);
	mesh_save(lowres, param_filename);
    }

    // TODO: Note: I whoud have expected < 0.0 here (to be understood)
    if(Geom::mesh_enclosed_volume(highres) > 0.0) {
	highres.facets.flip();
    }

    Image_var normal_map = new Image(
	Image::RGB, Image::BYTE, resolution, resolution
    );
    Image_var geometry_image = new Image(
	Image::RGB, Image::FLOAT64, resolution, resolution
    );
    if(remeshed) {
	bake_mesh_geometry(&lowres,geometry_image);
	bake_mesh_facet_normals_indirect(
	    geometry_image, normal_map, &highres
	);
    } else {
	bake_mesh_facet_normals(&lowres, normal_map);
    }

    MorphoMath mm(normal_map);
    mm.dilate(CmdLine::get_arg_uint("nb_dilations"));

    ImageLibrary::instance()->save_image(image_filename, normal_map);
}
