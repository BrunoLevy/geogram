/*
 *  Copyright (c) 2000-2025 Inria
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

#include <geogram/mesh/mesh_CSG_utils.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_repair.h>
#include <fstream>

/********* Value and ArgList *********************************************/

namespace GEOCSG {

    Value::Value() : type(NONE) {
    }

    Value::Value(const std::string& x) : type(STRING), string_val(x) {
    }

    Value::Value(double x) : type(NUMBER), number_val(x) {
    }

    Value::Value(int x) : type(NUMBER), number_val(double(x)) {
    }

    Value::Value(bool x) : type(BOOLEAN), boolean_val(x) {
    }

    Value::Value(const std::filesystem::path& x) :
	type(PATH), string_val(x.string()) {
    }

    std::string Value::to_string() const {
        switch(type) {
        case NUMBER: {
	    // We do not want trailing .000 for integers
	    return (ceil(number_val) == number_val)
		? String::to_string(int(number_val))
		: String::to_string(number_val);
	}
        case BOOLEAN:
            return String::to_string(boolean_val);
        case ARRAY1D: {
            std::string result = "[";
            if(array_val.size() != 0) {
                for(double v: array_val[0]) {
                    result += String::to_string(v);
                    result += " ";
                }
            }
            result += "]";
            return result;
        }
        case ARRAY2D: {
            std::string result = "[";
            for(const vector<double>& row : array_val) {
                result += "[";
                for(double v: row) {
                    result += String::to_string(v);
                    result += " ";
                }
                result += "]";
            }
            result += "]";
            return result;
        }
	case PATH:
        case STRING: {
            return "\"" + string_val + "\"";
        }
	case NONE: {
	    return "<none>";
	}
        }
        return "<unknown>";
    }

    void ArgList::add_arg(const std::string& name, const Value& value) {
        if(has_arg(name)) {
            throw(std::logic_error("Duplicated arg:" + name));
        }
        args_.push_back(std::make_pair(name,value));
    }

    bool ArgList::has_arg(const std::string& name) const {
        for(const Arg& arg : args_) {
            if(arg.first == name) {
                return true;
            }
        }
        return false;
    }

    const Value& ArgList::get_arg(const std::string& name) const {
        for(const Arg& arg : args_) {
            if(arg.first == name) {
                return arg.second;
            }
        }
        geo_assert_not_reached;
    }

    double ArgList::get_arg(const std::string& name, double default_val) const {
        for(const Arg& arg : args_) {
            if(arg.first == name) {
                if(arg.second.type != Value::NUMBER) {
                    throw(std::logic_error("Arg " + name + " has wrong type"));
                }
                return arg.second.number_val;
            }
        }
        return default_val;
    }

    int ArgList::get_arg(const std::string& name, int default_val) const {
        for(const Arg& arg : args_) {
            if(arg.first == name) {
                if(arg.second.type != Value::NUMBER) {
                    throw(std::logic_error(
                              "Arg " + name + " has wrong type"
                          ));
                }
                if(GEO::round(arg.second.number_val) != arg.second.number_val) {
                    throw(std::logic_error(
                              "Arg " + name + " has wrong type"
                          ));
                }
                return int(arg.second.number_val);
            }
        }
        return default_val;
    }

    bool ArgList::get_arg(const std::string& name, bool default_val) const {
        for(const Arg& arg : args_) {
            if(arg.first == name) {
                if(arg.second.type != Value::BOOLEAN) {
                    throw(std::logic_error(
                              "Arg " + name + " has wrong type"
                          ));
                }
                return arg.second.boolean_val;
            }
        }
        return default_val;
    }

    vec2 ArgList::get_arg(const std::string& name, vec2 default_val) const {
        for(const Arg& arg : args_) {
            if(arg.first == name) {
                if(arg.second.type == Value::NUMBER) {
                    return vec2(
                        arg.second.number_val,
                        arg.second.number_val
                    );
                } else if(arg.second.type != Value::ARRAY1D) {
                    throw(std::logic_error(
                              "Arg " + name + " has wrong type"
                          ));
                }
                if(
                    arg.second.array_val.size() != 1 ||
                    arg.second.array_val[0].size() != 2
                ) {
                    throw(std::logic_error(
                              "Arg " + name + " has wrong dimension"
                          ));
                }
                return vec2(
                    arg.second.array_val[0][0],
                    arg.second.array_val[0][1]
                );
            }
        }
        return default_val;
    }

    vec3 ArgList::get_arg(const std::string& name, vec3 default_val) const {
        for(const Arg& arg : args_) {
            if(arg.first == name) {
                if(arg.second.type != Value::ARRAY1D) {
                    throw(std::logic_error(
                              "Arg " + name + " has wrong type"
                          ));
                }
                if(
                    arg.second.array_val.size() != 1 ||
                    arg.second.array_val[0].size() != 3
                ) {
                    throw(std::logic_error(
                              "Arg " + name + " has wrong dimension"
                          ));
                }
                return vec3(
                    arg.second.array_val[0][0],
                    arg.second.array_val[0][1],
                    arg.second.array_val[0][2]
                );
            }
        }
        return default_val;
    }

    vec4 ArgList::get_arg(const std::string& name, vec4 default_val) const {
        for(const Arg& arg : args_) {
            if(arg.first == name) {
                if(arg.second.type != Value::ARRAY1D) {
                    throw(std::logic_error(
                              "Arg " + name + " has wrong type"
                          ));
                }
                if(
                    arg.second.array_val.size() != 1 ||
                    arg.second.array_val[0].size() != 4
                ) {
                    throw(std::logic_error(
                              "Arg " + name + " has wrong dimension"
                          ));
                }
                return vec4(
                    arg.second.array_val[0][0],
                    arg.second.array_val[0][1],
                    arg.second.array_val[0][2],
                    arg.second.array_val[0][3]
                );
            }
        }
        return default_val;
    }

    mat4 ArgList::get_arg(
	const std::string& name, const mat4& default_val
    ) const {
        for(const Arg& arg : args_) {
            if(arg.first == name) {
                if(arg.second.type != Value::ARRAY2D) {
                    throw(std::logic_error(
                              "Arg " + name + " has wrong type"
                          ));
                }
                auto Mvv = arg.second.array_val;
                if(
                    Mvv.size() != 4 ||
                    Mvv[0].size() != 4 ||
                    Mvv[1].size() != 4 ||
                    Mvv[2].size() != 4 ||
                    Mvv[3].size() != 4
                ) {
                    throw(std::logic_error(
                              "Matrix arg has wrong dimension"
                          ));
                }
                mat4 result;
                for(index_t i=0; i<4; ++i) {
                    for(index_t j=0; j<4; ++j) {
                        result(i,j) = Mvv[i][j];
                    }
                }
                return result;
            }
        }
        return default_val;
    }

    std::string ArgList::get_arg(
	const std::string& name, const std::string& default_val
    ) const {
        for(const Arg& arg : args_) {
            if(arg.first == name) {
                if(arg.second.type != Value::STRING) {
                    throw(std::logic_error(
                              "Arg " + name + " has wrong type"
                          ));
                }
                return arg.second.string_val;
            }
        }
        return default_val;
    }
}


/********* Sweep *************************************************/

namespace GEOCSG {

    void sweep(
	std::shared_ptr<Mesh>& M, index_t nv,
	std::function<vec3(index_t, index_t)> sweep_path,
	SweepCapping capping
    ) {

	M->vertices.set_dimension(2);
	index_t nu = M->vertices.nb();

	index_t total_nb_vertices = 0;
	switch(capping) {
	case SWEEP_CAP: {
	    total_nb_vertices = nu*nv;
	} break;
	case SWEEP_POLE: {
	    total_nb_vertices = nu*(nv-1)+1;
	} break;
	case SWEEP_PERIODIC: {
	    total_nb_vertices = nu*(nv-1);
	    M->facets.clear();
	} break;
	}

	index_t nt0 = M->facets.nb();

	M->vertices.set_dimension(3);
	M->vertices.create_vertices(total_nb_vertices - nu);

	// Start from 1: do not touch first slice for now, because it
	// may be used by sweep_path (as the origin of paths)
	for(index_t v=1; v<nv-1; ++v) {
	    for(index_t u=0; u<nu; ++u) {
		M->vertices.point(v*nu+u) = sweep_path(u,v);
	    }
	}

	// Particular case: last slice
	switch(capping) {
	case SWEEP_CAP:
	    for(index_t u=0; u<nu; ++u) {
		M->vertices.point((nv-1)*nu+u) = sweep_path(u,nv-1);
	    }
	    break;
	case SWEEP_POLE:
	    M->vertices.point((nv-1)*nu) = sweep_path(0,nv-1);
	    break;
	case SWEEP_PERIODIC:
	    // Nothing to do, last slice is same as first slice
	    break;
	}

        // Now map first slice
        for(index_t u=0; u<nu; ++u) {
	    M->vertices.point(u) = sweep_path(u,0);
	}

	// creates one row of "brick" for the walls
	auto create_brick_row = [&](index_t v, bool periodic=false) {
	    index_t v1 = v;
	    index_t v2 = v1+1;
	    if(periodic && v2 == nv-1) {
		v2 = 0;
	    }
	    for(index_t e=0; e < M->edges.nb(); ++e) {
		index_t vx1 = v1 * nu + M->edges.vertex(e,0) ;
		index_t vx2 = v1 * nu + M->edges.vertex(e,1) ;
		index_t vx3 = v2 * nu + M->edges.vertex(e,0) ;
		index_t vx4 = v2 * nu + M->edges.vertex(e,1) ;
		const vec3& p1 = M->vertices.point(vx1);
		const vec3& p2 = M->vertices.point(vx2);
		const vec3& p3 = M->vertices.point(vx3);
		const vec3& p4 = M->vertices.point(vx4);

		double l1 = length(p3-p2);
		double l2 = length(p4-p1);
		bool not_significative = (::fabs(l1-l2) < (l1+l2)*1e-6);

		if(not_significative || l1 < l2) {
		    M->facets.create_triangle(vx1, vx2, vx3);
		    M->facets.create_triangle(vx3, vx2, vx4);
		} else {
		    M->facets.create_triangle(vx1, vx2, vx4);
		    M->facets.create_triangle(vx1, vx4, vx3);
		}
	    }
	};

	// generate walls (all brick rows except last one)
	for(index_t v=0; v+2 < nv; ++v) {
	    create_brick_row(v);
	}

	// generate walls (last "brick" row, depends on capping mode)
	switch(capping) {
	case SWEEP_CAP: {
	    create_brick_row(nv-2);
	} break;
	case SWEEP_POLE: {
	    index_t v = nv-2;
	    for(index_t e=0; e < M->edges.nb(); ++e) {
		index_t vx1 = v * nu + M->edges.vertex(e,0) ;
		index_t vx2 = v * nu + M->edges.vertex(e,1) ;
		index_t vx3 = nu * (nv-1);
		M->facets.create_triangle(vx1, vx2, vx3);
	    }
	} break;
	case SWEEP_PERIODIC: {
	    create_brick_row(nv-2, true); // periodic
	} break;
	}

	// generate second capping
	if(capping == SWEEP_CAP) {
	    index_t nt1 = M->facets.nb();
	    index_t v_ofs = nu*(nv-1);
	    M->facets.create_triangles(nt0);
	    for(index_t t=0; t<nt0; ++t) {
		M->facets.set_vertex(t+nt1, 0, v_ofs + M->facets.vertex(t,0));
		M->facets.set_vertex(t+nt1, 1, v_ofs + M->facets.vertex(t,1));
		M->facets.set_vertex(t+nt1, 2, v_ofs + M->facets.vertex(t,2));
	    }
	}

	// flip initial triangles to generate first capping
	if(capping == SWEEP_CAP || capping == SWEEP_POLE) {
	    for(index_t t=0; t<nt0; ++t) {
		M->facets.flip(t);
	    }
	}

	M->edges.clear();
    }

    void keep_z0_only(std::shared_ptr<Mesh>& M) {
	vector<index_t> remove_triangle(M->facets.nb(),0);
	for(index_t t=0; t<M->facets.nb(); ++t) {
	    for(index_t lv=0; lv<3; ++lv) {
		const vec3& p = M->facets.point(t,lv);
		if(p.z != 0.0) {
		    remove_triangle[t] = 1;
		    break;
		}
	    }
	}
	M->facets.delete_elements(remove_triangle);
	M->vertices.remove_isolated();
	M->vertices.set_dimension(2);
    }
}

/*************************************************************************/

namespace {
    /** \brief subdirectory with all cached files converted by OpenSCAD */
    static const char* OpenSCache = "OpenSCache";

    bool OpenSCache_invalidate = false;
    bool OpenSCache_ignore_time = false;
}

namespace GEOCSG {

    void OpenSCAD_cache_invalidate() {
	OpenSCache_invalidate = true;
    }

    void OpenSCAD_cache_ignore_time() {
	OpenSCache_ignore_time = true;
    }

    std::shared_ptr<Mesh> call_OpenSCAD(
	const std::filesystem::path& path, const std::string& command,
	const ArgList& args, bool TWO_D
    ) {
	std::shared_ptr<Mesh> result = std::make_shared<Mesh>();

	std::string mangled = command;
	std::string command_with_args = command + "(";
	for(index_t i=0; i<args.size(); ++i) {
	    command_with_args += args.ith_arg_name(i);
	    command_with_args += '=';
	    command_with_args += args.ith_arg_val(i).to_string();
	    if(i != args.size()-1) {
		command_with_args += ',';
	    }
	    mangled += '_';
	    std::string mangled_arg;
	    if(args.ith_arg_val(i).type == Value::PATH) {
		std::filesystem::path path_val(args.ith_arg_val(i).string_val);
		mangled_arg += path_val.filename().string();
	    } else {
		mangled_arg = args.ith_arg_val(i).to_string();
	    }
	    if(*mangled_arg.begin() == '"' && *mangled_arg.rbegin() == '"') {
		mangled_arg = mangled_arg.substr(1, mangled_arg.length()-2);
		if(mangled_arg.size() == 0) {
		    mangled_arg = "nil";
		}
	    }
	    mangled += mangled_arg;
	}
	command_with_args += ')';

	std::replace(mangled.begin(), mangled.end(), ' ', '_');
	std::replace(mangled.begin(), mangled.end(), '.', '@');
	std::replace(mangled.begin(), mangled.end(), '/', '!');
	std::replace(mangled.begin(), mangled.end(), '\\', '!');

	std::filesystem::path tmp = std::filesystem::temp_directory_path();
	std::filesystem::path cache_path = path / OpenSCache;
	if(!std::filesystem::is_directory(cache_path)) {
	    std::filesystem::create_directory(cache_path);
	}

	std::filesystem::path cached_STL = cache_path / (mangled + ".stl");

	// Cached file exists, load it and return it
	if(
	    !OpenSCache_invalidate &&
	    std::filesystem::is_regular_file(cached_STL)
	) {
	    Logger::out("CSG")<< "Using cached " << cached_STL << std::endl;
	    mesh_load(cached_STL.string(), *result);
	    mesh_repair(
		*result, MeshRepairMode(MESH_REPAIR_DEFAULT | MESH_REPAIR_QUIET)
	    );
	    if(TWO_D) {
		result->vertices.set_dimension(2);
	    }
	    return result;
	}

	// Generate file with OpenSCAD
	std::filesystem::path osc = tmp / "tmpscad.scad";
	std::filesystem::path osc_stl = tmp / "tmpscad.stl";
	std::ofstream gen_osc(osc);
	if(!gen_osc) {
	    throw(std::logic_error(osc.string() + ": could not create"));
	}
	// OpenSCAD cannot save 2d results, so if we are in 2D, we
	// artificially generate a 3d result by extrusion
	if(TWO_D) {
	    gen_osc << "linear_extrude(height=1.0) {";
	}
	gen_osc << command_with_args << ";" << std::endl;
	if(TWO_D) {
	    gen_osc << "}";
	}
	gen_osc.close();


	// Execute openscad
	std::string openscad_command = "openscad "
	    + osc.string() + " -o " + osc_stl.string() ;
	if(system(openscad_command.c_str())) {
	    Logger::warn("CSG") << "Error while running openscad "
				<< std::endl;
	    Logger::warn("CSG") << "(for command: " << command <<") "
				<< std::endl;
	}

	if(!std::filesystem::is_regular_file(osc_stl)) {
	    Logger::warn("CSG") << "Could not open " << osc_stl << std::endl;
	    return result;
	}

	mesh_load(osc_stl.string(), *result);
	std::filesystem::remove(osc);
	std::filesystem::remove(osc_stl);

	// If we are in 2D, we have artificially added an extrusion to
	// have a 3D result (else OpenSCAD does not output anything),
	// so we need to remove these additional triangles
	if(TWO_D) {
	    keep_z0_only(result);
	}
	mesh_save(*result,cached_STL.string());
	mesh_repair(
	    *result, MeshRepairMode(MESH_REPAIR_DEFAULT | MESH_REPAIR_QUIET)
	);
	return result;
    }

    std::string load_OpenSCAD(const std::filesystem::path& input) {
	if(input.extension() == ".scad" || input.extension() == ".SCAD") {
	    std::filesystem::path cache_path=input.parent_path() / OpenSCache;

	    if(!std::filesystem::is_directory(cache_path)) {
		std::filesystem::create_directory(cache_path);
	    }

	    std::filesystem::path cached_csg =
		cache_path / input.filename().replace_extension(".csg");

	    bool generate = !std::filesystem::is_regular_file(cached_csg);

	    if(OpenSCache_invalidate) {
		generate = true;
	    }

	    if(!OpenSCache_ignore_time) {
		generate = generate || (
		    std::filesystem::last_write_time(input) >
		    std::filesystem::last_write_time(cached_csg)
		);
	    }

	    if(generate) {
		Logger::out("CSG") << "Converting " << input << " with OpenSCAD"
				   << std::endl;
		std::string openscad_command = "openscad "
		    + input.string() + " -o " + cached_csg.string() ;
		if(system(openscad_command.c_str())) {
		    Logger::warn("CSG") << "Error while running openscad "
					<< std::endl;
		    Logger::warn("CSG") << "(for converting: " << input << ") "
					<< std::endl;
		    return "";
		}
		if(std::filesystem::is_regular_file(cached_csg)) {
		    Logger::out("CSG") << "Created " << cached_csg << std::endl;
		} else {
		    Logger::out("CSG") << "Could not create"
				       << cached_csg << std::endl;
		}
	    } else {
		Logger::out("CSG") << "Using cached " << cached_csg << std::endl;
	    }
	    return load_OpenSCAD(cached_csg);
	}

	if(input.extension() == ".csg" || input.extension() == ".CSG") {
	    std::string source;
	    if(std::filesystem::is_regular_file(input)) {
		size_t length = std::filesystem::file_size(input);
		source.resize(length);
		FILE* f = fopen(input.string().c_str(),"rb");
		if(f != nullptr) {
		    size_t read_length = fread(source.data(), 1, length, f);
		    if(read_length != length) {
			Logger::err("CSG")
			    << "Problem occured when reading "
			    << input
			    << std::endl;
		    }
		    fclose(f);
		}
	    }
	    return source;
	}
	Logger::err("CSG") << "Unknown extension: " << input << std::endl;
	return "";
    }
}

/************** Functions taken from OpenSCAD ****************************/

namespace {
    // From openscad/Geometry/Grid.h
    static constexpr double GRID_FINE = 0.00000095367431640625;
    // This one often misses so I redeclare it here
    static constexpr double M_DEG2RAD = M_PI / 180.0;

    /*
     https://mathworld.wolfram.com/Helix.html
     For a helix defined as:    F(t) = [r*cost(t), r*sin(t), c*t]  for t in [0,T)
     The helical arc length is          L = T * sqrt(r^2 + c^2)
     Where its pitch is             pitch = 2*PI*c
     Pitch is also height per turn: pitch = height / (twist/360)
     Solving for c gives                c = height / (twist*PI/180)
     Where (twist*PI/180) is just twist in radians, aka "T"
    */
    double helix_arc_length(double r_sqr, double height, double twist) {
	double T = twist * M_DEG2RAD;
	double c = height / T;
	return T * sqrt(r_sqr + c * c);
    }


    /*!
     Returns the number of slices for a linear_extrude with twist.
     Given height, twist, and the three special variables $fn, $fs and $fa
    */
    int get_helix_slices(
	double r_sqr, double height, double twist,
	double fn, double fs, double fa
    ) {
	twist = fabs(twist);
	// 180 twist per slice is worst case, guaranteed non-manifold.
	// Make sure we have at least 3 slices per 360 twist
	int min_slices = std::max(static_cast<int>(ceil(twist / 120.0)), 1);
	if (sqrt(r_sqr) < GRID_FINE || std::isinf(fn) || std::isnan(fn))
	    return min_slices;
	if (fn > 0.0) {
	    int fn_slices = static_cast<int>(ceil(twist / 360.0 * fn));
	    return std::max(fn_slices, min_slices);
	}
	int fa_slices = static_cast<int>(ceil(twist / fa));
	int fs_slices = static_cast<int>(
	    ceil(helix_arc_length(r_sqr, height, twist) / fs)
	);
	return std::max(std::min(fa_slices, fs_slices), min_slices);
    }

   /*
    For linear_extrude with twist and uniform scale (scale_x == scale_y),
    to calculate the limit imposed by special variable $fs, we find the
    total length along the path that a vertex would follow.
    The XY-projection of this path is a section of the Archimedes Spiral.
    https://mathworld.wolfram.com/ArchimedesSpiral.html
    Using the formula for its arc length, then pythagorean theorem with height
    should tell us the total distance a vertex covers.
    */
    double archimedes_length(double a, double theta) {
	return 0.5 * a * (theta * sqrt(1 + theta * theta) + asinh(theta));
    }


    int get_conical_helix_slices(
	double r_sqr, double height, double twist, double scale,
	double fn, double fs, double fa
    ) {
	twist = fabs(twist);
	double r = sqrt(r_sqr);
	int min_slices = std::max(static_cast<int>(ceil(twist / 120.0)), 1);
	if (r < GRID_FINE || std::isinf(fn) || std::isnan(fn)) {
	    return min_slices;
	}
	if (fn > 0.0) {
	    int fn_slices = static_cast<int>(ceil(twist * fn / 360));
	    return std::max(fn_slices, min_slices);
	}

     /*
        Spiral length equation assumes starting from theta=0
        Our twist+scale only covers a section of this length (unless scale=0).
        Find the start and end angles that our twist+scale correspond to.
        Use similar triangles to visualize cross-section of single vertex,
        with scale extended to 0 (origin).

        (scale < 1)        (scale > 1)
                           ______t_  1.5x (Z=h)
       0x                 |    | /
      |\                  |____|/
      | \                 |    / 1x  (Z=0)
      |  \                |   /
      |___\ 0.66x (Z=h)   |  /     t is angle of our arc section (twist, in rads)
      |   |\              | /      E is angle_end (total triangle base length)
      |___|_\  1x (Z=0)   |/ 0x    S is angle_start
            t

        E = t*1/(1-0.66)=3t E = t*1.5/(1.5-1)  = 3t
        B = E - t            B = E - t
      */
	double rads = twist * M_DEG2RAD;
	double angle_end;
	if (scale > 1) {
	    angle_end = rads * scale / (scale - 1);
	} else if (scale < 1) {
	    angle_end = rads / (1 - scale);
	} else {
	    // Don't calculate conical slices on non-scaled extrude!
	    geo_assert_not_reached;
	}
	double angle_start = angle_end - rads;
	double a = r / angle_end; // spiral scale coefficient
	double spiral_length = archimedes_length(
	    a, angle_end) - archimedes_length(a, angle_start
					     );
	// Treat (flat spiral_length,extrusion height) as (base,height)
	// of a right triangle to get diagonal length.
	double total_length = sqrt(
	    spiral_length * spiral_length + height * height
	);

	int fs_slices = static_cast<int>(ceil(total_length / fs));
	int fa_slices = static_cast<int>(ceil(twist / fa));
	return std::max(std::min(fa_slices, fs_slices), min_slices);
    }

   /*
    For linear_extrude with non-uniform scale (and no twist)
    Either use $fn directly as slices,
    or divide the longest diagonal vertex extrude path by $fs

    dr_sqr - the largest 2D delta (before/after scaling)
       for all vertices, squared.
    note: $fa is not considered since no twist
          scale is not passed in since it was already used
  	  to calculate the largest delta.
    */
    int get_diagonal_slices(
	double delta_sqr, double height, double fn, double fs
    ) {
	constexpr int min_slices = 1;
	if (sqrt(delta_sqr) < GRID_FINE || std::isinf(fn) || std::isnan(fn)) {
	    return min_slices;
	}
	if (fn > 0.0) {
	    int fn_slices = static_cast<int>(fn);
	    return std::max(fn_slices, min_slices);
	}
	int fs_slices = static_cast<int>(
	    ceil(sqrt(delta_sqr + height * height) / fs)
	);
	return std::max(fs_slices, min_slices);
    }

}

namespace GEOCSG {

    int get_fragments_from_r_and_twist(
	double r, double twist, double fn, double fs, double fa
    ) {

	if (r < GRID_FINE || std::isinf(fn) || std::isnan(fn)) {
	    return 3u;
	}
	if (fn > 0.0) {
	    return static_cast<int>(fn >= 3 ? fn : 3);
	}
	return static_cast<int>(
	    ceil(fmax(fmin(twist / fa, r * 2.0 * M_PI / fs), 5.0))
	);
    }

    // This one is not part of OpenSCAD, I copied it from
    // extrudePolygon() in geometry/GeometryEvaluator.cc
    // (with some readaptations / reordering to make it
    // easier to understand, at least for me)
    int get_linear_extrusion_slices(
	std::shared_ptr<Mesh> M,
	double height, vec2 scale, double twist,
	double fn, double fs, double fa
    ) {

	if(twist == 0.0 && scale.x == scale.y) {
	    return 1;
	}

	double max_r1_sqr = 0.0;  // r1 is before scaling
	double max_delta_sqr = 0; // delta from before/after scaling
	for(index_t iv=0; iv<M->vertices.nb(); ++iv) {
	    const vec2& v = M->vertices.point<2>(iv);
	    max_r1_sqr = std::max(max_r1_sqr, length2(v));
	    GEO::vec2 scale_v(v.x*scale.x, v.y*scale.y);
	    max_delta_sqr = std::max(
		max_delta_sqr, length2(v - scale_v)
	    );
	}

	if(twist == 0.0) {
	    return get_diagonal_slices(max_delta_sqr, height, fn, fs);
	}

	// Calculate Helical curve length for Twist with no Scaling
	if(scale.x == 1.0 && scale.y == 1.0) {
	    return get_helix_slices(max_r1_sqr, height, twist, fn, fs, fa);
	}

	// non uniform scaling with twist using max slices
	// from twist and non uniform scale
	if(scale.x != scale.y) {
	    int slicesNonUniScale = get_diagonal_slices(
		max_delta_sqr, height, fn, fs
	    );
	    int slicesTwist = get_helix_slices(
		max_r1_sqr, height, twist, fn, fs, fa
	    );
	    return std::max(slicesNonUniScale, slicesTwist);
	}

	// uniform scaling with twist, use conical helix calculation
	return get_conical_helix_slices(
	    max_r1_sqr, height, twist, scale.x, fn, fs, fa
	);
    }
}
