/*
 *  Copyright (c) 2000-2023 Inria
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

#include <geogram/mesh/mesh_CSG_builder.h>
#include <geogram/mesh/mesh_surface_intersection.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_fill_holes.h>
#include <geogram/delaunay/parallel_delaunay_3d.h>
#include <geogram/delaunay/CDT_2d.h>
#include <geogram/image/image.h>
#include <geogram/image/image_library.h>
#include <geogram/basic/line_stream.h>
#include <geogram/basic/command_line.h>

namespace {
    using namespace GEO;

    /**
     * \brief Tests whether the meshes in a scope may have intersections
     * \param[in] scope a scope
     * \retval true if there may be some intersections between the meshes
     * \retval false if there is for sure no intersection between the meshes
     */
    bool may_have_intersections(const CSGScope& scope) {
	std::vector<Box3d> box(scope.size());
	for(index_t i=0; i<scope.size(); ++i) {
	    box[i] = CSGBuilder::get_bbox(scope[i]);
	}
	for(index_t i=0; i<scope.size(); ++i) {
	    for(index_t j=i+1; j<scope.size(); ++j) {
		if(bboxes_overlap(box[i], box[j])) {
		    return true;
		}
	    }
	}
	return false;
    }
}


namespace GEO {

    CSGBuilder::CSGBuilder() {
        reset_defaults();
        reset_file_path();
        STL_epsilon_ = 1e-6;
        verbose_ = false;
	fine_verbose_ = false;
        max_arity_ = 32;
        simplify_coplanar_facets_ = true;
        coplanar_angle_tolerance_ = 0.0;
        delaunay_ = true;
        detect_intersecting_neighbors_ = true;
        fast_union_ = false;
	warnings_ = false;
	noop_ = false;
    }

    CSGBuilder::~CSGBuilder() {
    }

    std::shared_ptr<Mesh> CSGBuilder::square(vec2 size, bool center) {
	std::shared_ptr<Mesh> M = std::make_shared<Mesh>();

        double x1 = 0.0;
        double y1 = 0.0;
        double x2 = size.x;
        double y2 = size.y;

        if(center) {
            x1 -= size.x/2.0;
            x2 -= size.x/2.0;
            y1 -= size.y/2.0;
            y2 -= size.y/2.0;
        }

        M->vertices.set_dimension(2);

        if(size.x <= 0.0 || size.y <= 0.0) {
	    if(warnings_) {
		Logger::warn("CSG")
		    << "square with negative size (returning empty shape)"
		    << std::endl;
	    }
            return M;
        }

        M->vertices.create_vertex(vec2(x1,y1));
        M->vertices.create_vertex(vec2(x2,y1));
        M->vertices.create_vertex(vec2(x1,y2));
        M->vertices.create_vertex(vec2(x2,y2));

        M->facets.create_triangle(0,1,3);
        M->facets.create_triangle(0,3,2);

	M->edges.create_edge(0,1);
	M->edges.create_edge(1,3);
	M->edges.create_edge(3,2);
	M->edges.create_edge(2,0);

        finalize_mesh(M);

	return M;
    }

    std::shared_ptr<Mesh> CSGBuilder::circle(double r, index_t nu) {
	std::shared_ptr<Mesh> M = std::make_shared<Mesh>();

	if(nu == 0) {
	    nu = index_t(GEOCSG::get_fragments_from_r(r, fn_, fs_, fa_));
	}
	nu = std::max(nu, index_t(3));
        M->vertices.set_dimension(2);

        if(r <= 0.0) {
	    if(warnings_) {
		Logger::warn("CSG")
		    << "circle with negative radius (returning empty shape)"
		    << std::endl;
	    }
            return M;
        }

        for(index_t u=0; u<nu; ++u) {
            double theta = double(u)*2.0*M_PI/double(nu);
            double ctheta = cos(theta);
            double stheta = sin(theta);
            double x = ctheta*r;
            double y = stheta*r;
            M->vertices.create_vertex(vec2(x,y));
        }

	// zigzag triangulation
	{
	    index_t low = 0;
	    index_t high = nu-1;
	    for(;;) {
		if(low+1==high) {
		    break;
		}
		M->facets.create_triangle(low, low+1, high);
		++low;
		if(high-1==low) {
		    break;
		}
		M->facets.create_triangle(low, high-1, high);
		--high;
	    }
	}

        for(index_t u=0; u<nu; ++u) {
	    M->edges.create_edge(u, (u+1)%nu);
	}

        finalize_mesh(M);

	return M;
    }

    std::shared_ptr<Mesh> CSGBuilder::cube(vec3 size, bool center) {
	std::shared_ptr<Mesh> M = std::make_shared<Mesh>();

        double x1 = 0.0;
        double y1 = 0.0;
        double z1 = 0.0;
        double x2 = size.x;
        double y2 = size.y;
        double z2 = size.z;

        if(center) {
            x1 -= size.x/2.0;
            x2 -= size.x/2.0;
            y1 -= size.y/2.0;
            y2 -= size.y/2.0;
            z1 -= size.z/2.0;
            z2 -= size.z/2.0;
        }

        if(size.x <= 0.0 || size.y <= 0.0 || size.z <= 0.0) {
	    if(warnings_) {
		Logger::warn("CSG")
		    << "cube with negative size (returning empty shape)"
		    << std::endl;
	    }
            return M;
        }

        M->vertices.create_vertex(vec3(x1,y1,z1));
        M->vertices.create_vertex(vec3(x2,y1,z1));
        M->vertices.create_vertex(vec3(x1,y2,z1));
        M->vertices.create_vertex(vec3(x2,y2,z1));
        M->vertices.create_vertex(vec3(x1,y1,z2));
        M->vertices.create_vertex(vec3(x2,y1,z2));
        M->vertices.create_vertex(vec3(x1,y2,z2));
        M->vertices.create_vertex(vec3(x2,y2,z2));

        M->facets.create_triangle(7,3,6);
        M->facets.create_triangle(6,3,2);
        M->facets.create_triangle(7,5,3);
        M->facets.create_triangle(3,5,1);
        M->facets.create_triangle(3,1,2);
        M->facets.create_triangle(2,1,0);
        M->facets.create_triangle(1,5,0);
        M->facets.create_triangle(0,5,4);
        M->facets.create_triangle(2,0,6);
        M->facets.create_triangle(6,0,4);
        M->facets.create_triangle(6,4,7);
        M->facets.create_triangle(7,4,5);

	M->facets.connect();

	finalize_mesh(M);

	return M;
    }

    std::shared_ptr<Mesh> CSGBuilder::sphere(double r) {
        index_t nu = index_t(GEOCSG::get_fragments_from_r(r,fn_,fs_,fa_));
        index_t nv = nu / 2;
	if(nu >= 5 && (nu & 1) != 0) {
	    ++nv;
	}

	nu = std::max(nu, index_t(3));
	nv = std::max(nv, index_t(2));

	std::shared_ptr<Mesh> result;

        if(r <= 0.0) {
	    if(warnings_) {
		Logger::warn("CSG")
		    << "sphere with negative radius (returning empty shape)"
		    << std::endl;
	    }
            return result;
        }

	result = circle(1.0, nu);

	GEOCSG::sweep(
	    result, nv,
	    [&](index_t u, index_t v)->vec3 {
		double phi = (double(v) + 0.5)*M_PI/double(nv) - M_PI/2.0;
		double cphi = cos(phi);
		double sphi = sin(phi);
                double ctheta = result->vertices.point(u).x;
                double stheta = result->vertices.point(u).y;
		return vec3(
		    r*ctheta*cphi,
		    r*stheta*cphi,
		    r*sphi
		);
	    }
	);

	result->facets.connect();

	finalize_mesh(result);
	return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::cylinder(
	double h, double r1, double r2, bool center
    ) {
        index_t nu = index_t(
	    GEOCSG::get_fragments_from_r(std::max(r1,r2),fn_,fs_,fa_)
	);

	double r[2] = { r1, r2 };

        double z[2] = {
	    center ? -h/2.0 : 0.0,
	    center ?  h/2.0 : h
	};

	std::shared_ptr<Mesh> result;

        if(r1 < 0.0 || r2 < 0.0) {
	    if(warnings_) {
		Logger::warn("CSG")
		    << "cylinder with negative radius (returning empty shape)"
		    << std::endl;
	    }
            return result;
        }

	// If first side's capping is a pole, then flip sides
	// (sweep() supposes that the pole is always on the second side).
        if(r[0] == 0.0) {
            std::swap(r[0],r[1]);
            std::swap(z[0],z[1]);
        }

	result = circle(1.0, nu);
	GEOCSG::sweep(
	    result, 2,
	    [&](index_t u, index_t v)->vec3 {
		vec3 p = result->vertices.point(u);
		return vec3(
		    r[v]*p.x, r[v]*p.y, z[v]
		);
	    },
	    (r[1] == 0.0) ? GEOCSG::SWEEP_POLE : GEOCSG::SWEEP_CAP
	);

	result->facets.connect();

	finalize_mesh(result);
	return result;
    }


    std::shared_ptr<Mesh> CSGBuilder::import(
        const std::filesystem::path& filename, const std::string& layer,
        index_t timestamp, vec2 origin, vec2 scale
    ) {
	std::shared_ptr<Mesh> result;
        std::filesystem::path full_filename = filename;
        if(!find_file(full_filename)) {
            Logger::err("CSG") << filename << ": file not found"
                               << std::endl;
            return result;
        }

        if(
	    full_filename.extension() == ".dxf" ||
	    full_filename.extension() == ".DXF"
	) {
            result = import_with_openSCAD(full_filename, layer, timestamp);
        } else {
	    result = std::make_shared<Mesh>();
            mesh_load(full_filename.string(),*result);
        }

	if(
	    full_filename.extension() == ".stl" ||
	    full_filename.extension() == ".STL"
	) {
	    MeshRepairMode mode = MESH_REPAIR_DEFAULT;
	    if(!verbose_) {
		mode = MeshRepairMode(mode | MESH_REPAIR_QUIET);
	    }
	    mesh_repair(*result, mode, STL_epsilon_);
	}

	if(result->vertices.dimension() == 3) {
	    bool z_all_zero = true;
	    for(const vec3& p : result->vertices.points()) {
		if(p.z != 0.0) {
		    z_all_zero = false;
		    break;
		}
	    }
	    if(z_all_zero) {
		result->vertices.set_dimension(2);
	    }
	}

	if(result->vertices.dimension() == 3) {
	    if(!result->facets.are_simplices()) {
		tessellate_facets(*result, 3);
	    }
	}
        // Apply origin and scale, triangulate
	if(result->vertices.dimension() == 2) {
	    for(index_t v: result->vertices) {
		vec2& p = result->vertices.point<2>(v);
		p = vec2(
		    (p.x - origin.x) * scale.x,
		    (p.y - origin.y) * scale.y
		);
	    }
	    result->facets.compute_borders();
	    triangulate(result);
	}

        finalize_mesh(result);
        return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::text_with_OpenSCAD(
	const std::string& text,
	double size,
	const std::string& font,
	const std::string& halign,
	const std::string& valign,
	double spacing,
	const std::string& direction,
	const std::string& language,
	const std::string& script
    ) {
	GEOCSG::ArgList args;
	args.add_arg("text", text);
	args.add_arg("size", size);
	args.add_arg("font", font);
	args.add_arg("halign", halign);
	args.add_arg("valign", valign);
	args.add_arg("spacing", spacing);
	args.add_arg("direction", direction);
	args.add_arg("language", language);
	args.add_arg("script", script);
	bool TWO_D=true;
	std::shared_ptr<Mesh> result = GEOCSG::call_OpenSCAD(
	    current_path(), "text", args, TWO_D
	);
	finalize_mesh(result);
	return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::surface_with_OpenSCAD(
        const std::filesystem::path& filename, bool center, bool invert
    ) {
	std::shared_ptr<Mesh> result;
        std::filesystem::path full_filename = filename;
        if(!find_file(full_filename)) {
            Logger::err("CSG") << filename << ": file not found"
                               << std::endl;
	    result = std::make_shared<Mesh>();
            return result;
        }
	GEOCSG::ArgList args;
	args.add_arg("file", full_filename);
	args.add_arg("center", center);
	args.add_arg("invert", invert);
	result = GEOCSG::call_OpenSCAD(current_path(), "surface", args);
	finalize_mesh(result);
        return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::multmatrix(
	const mat4& M, const CSGScope& scope
    ) {
	std::shared_ptr<Mesh> result = group(scope);
	for(index_t v: result->vertices) {
            double* p = result->vertices.point_ptr(v);
	    vec3 P = result->vertices.dimension() == 3 ?
		vec3(p[0], p[1], p[2]):
		vec3(p[0], p[1], 0.0);
	    P = transform_point(M,P);
            for(index_t c=0; c<result->vertices.dimension(); ++c) {
                p[c] = P[c];
            }
	}
	// Preserve normals orientations if transform is left-handed
	if(det(M) < 0.0) {
	    for(index_t e: result->edges) {
		result->edges.flip(e);
	    }
	    for(index_t t: result->facets) {
		result->facets.flip(t);
	    }
	}
	finalize_mesh(result);
	return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::union_instr(const CSGScope& scope) {

	if(scope.size() == 0) {
	    return std::make_shared<Mesh>();
	}

	if(scope.size() == 1) {
	    return scope[0];
	}

        // Boolean operations can handle no more than max_arity_ operands.
        // For a union with more than max_arity_ operands, split it into two.
	if(!fast_union_ && scope.size() > max_arity_) {
            CSGScope scope1;
            CSGScope scope2;
            index_t n1 = index_t(scope.size()/2);
            for(index_t i=0; i<scope.size(); ++i) {
                if(i < n1) {
                    scope1.emplace_back(scope[i]);
                } else {
                    scope2.emplace_back(scope[i]);
                }
            }
            CSGScope scope3;
            scope3.emplace_back(union_instr(scope1));
            scope3.emplace_back(union_instr(scope2));
            return union_instr(scope3);
	}

	std::shared_ptr<Mesh> result = append(scope);
	if(may_have_intersections(scope)) {
	    do_CSG(result, "union");
	}
	finalize_mesh(result);
	return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::intersection(const CSGScope& scope) {

	if(scope.size() == 0) {
	    return std::make_shared<Mesh>();
	}

        if(scope.size() == 1) {
            return scope[0];
        }

        // Boolean operations can handle no more than max_arity_ operands.
        // For a intersection with more than max_arity_ operands,
        // split it into two.
        if(scope.size() > max_arity_) {
            CSGScope scope1;
            CSGScope scope2;
            index_t n1 = index_t(scope.size()/2);
            for(index_t i=0; i<scope.size(); ++i) {
                if(i < n1) {
                    scope1.emplace_back(scope[i]);
                } else {
                    scope2.emplace_back(scope[i]);
                }
            }

            CSGScope scope3;
            scope3.emplace_back(intersection(scope1));
            scope3.emplace_back(intersection(scope2));
            return intersection(scope3);
        }

	std::shared_ptr<Mesh> result = append(scope);
	do_CSG(result, "intersection");
	finalize_mesh(result);
        return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::difference(const CSGScope& scope) {
	if(scope.size() == 0) {
	    return std::make_shared<Mesh>();
	}

        if(scope.size() == 1) {
            return scope[0];
        }

        // Boolean operations can handle no more than max_arity_ operands.
        // For a difference with more than max_arity_ operands, split it
        // (by calling union_instr() that in turn splits the list if need be).
        if(scope.size() > max_arity_) {
            CSGScope scope2;
            for(index_t i=1; i<scope.size(); ++i) {
                scope2.emplace_back(scope[i]);
            }
            CSGScope scope3;
            scope3.emplace_back(scope[0]);
            scope3.emplace_back(union_instr(scope2));
            return difference(scope3);
        }

	std::shared_ptr<Mesh> result = append(scope);
        // construct the expression x0-x1-x2...-xn
        std::string expr = "x0";
        for(index_t i=1; i<scope.size(); ++i) {
            expr += "-x" + String::to_string(i);
        }

	do_CSG(result, expr);
	finalize_mesh(result);
        return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::group(const CSGScope& scope) {
        return union_instr(scope);
    }

    std::shared_ptr<Mesh> CSGBuilder::color(vec4 color, const CSGScope& scope) {
	geo_argused(color);
	std::shared_ptr<Mesh> result = append(scope);
	finalize_mesh(result);
	return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::hull(const CSGScope& scope) {
	std::shared_ptr<Mesh> result = append(scope);
	result->edges.clear();
	result->facets.clear();

	// Particular case: no vertex in scope (yes, this happens !)
	if(result->vertices.nb() == 0) {
	    return result;
	}

	// Cmpute Delaunay triangulation, then
	// extract convex hull as triangulation's boundary.

        if(result->vertices.dimension() == 3) {
	    CmdLine::set_arg("algo:delaunay", "PDEL");
        } else {
	    CmdLine::set_arg("algo:delaunay", "BDEL2d");
        }

	Delaunay_var delaunay = Delaunay::create(
	    coord_index_t(result->vertices.dimension())
	);

	delaunay->set_keeps_infinite(true);
        delaunay->set_vertices(
	    result->vertices.nb(), result->vertices.point_ptr(0)
	);

        if(result->vertices.dimension() == 3) {
            // This iterates on the infinite cells
            for(
                index_t t = delaunay->nb_finite_cells();
		t < delaunay->nb_cells(); ++t
            ) {
		index_t v0 = delaunay->cell_vertex(t,0);
		index_t v1 = delaunay->cell_vertex(t,1);
		index_t v2 = delaunay->cell_vertex(t,2);
		index_t v3 = delaunay->cell_vertex(t,3);
		if(v0 == NO_INDEX) {
		    result->facets.create_triangle(v1,v2,v3);
		} else if(v1 == NO_INDEX) {
		    result->facets.create_triangle(v3,v2,v0);
		} else if(v2 == NO_INDEX) {
		    result->facets.create_triangle(v1,v3,v0);
		} else if(v3 == NO_INDEX) {
		    result->facets.create_triangle(v2,v1,v0);
		}
	    }
        } else {
            for(index_t t = delaunay->nb_finite_cells();
                t < delaunay->nb_cells(); ++t
               ) {
                index_t v1= NO_INDEX, v2=NO_INDEX;
                for(index_t lv=0; lv<3; ++lv) {
                    if(delaunay->cell_vertex(t,lv) == NO_INDEX) {
                        v1 = delaunay->cell_vertex(t,(lv+1)%3);
                        v2 = delaunay->cell_vertex(t,(lv+2)%3);
                    }
                }
                geo_assert(v1 != NO_INDEX && v2 != NO_INDEX);
                result->edges.create_edge(v1,v2);
            }
        }
	result->vertices.remove_isolated();
        finalize_mesh(result);
        return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::linear_extrude(
        const CSGScope& scope, double height, bool center, vec2 scale,
        index_t slices, double twist
    ) {
	std::shared_ptr<Mesh> result = union_instr(scope);

        double z1 = center ? -height/2.0 : 0.0;
        double z2 = center ?  height/2.0 : height;

        if(slices == 0) {
	    slices = index_t(
		GEOCSG::get_linear_extrusion_slices(
		    result, height, scale, twist, fn_, fs_, fa_
		)
	    );
	}

	index_t nv = slices+1;

	GEOCSG::sweep(
	    result, nv,
	    [&](index_t u, index_t v)->vec3 {
		vec2 ref = result->vertices.point<2>(u);
		double t = double(v) / double(nv-1);

		double s = 1.0 - t;
		double z = s*z1 + t*z2;
		vec2   sz = s*vec2(1.0, 1.0) + t*scale;

		double x = ref.x * sz.x;
		double y = ref.y * sz.y;

		if(twist != 0.0) {
		    double alpha = twist*t*M_PI/180.0;
		    double ca = cos(alpha);
		    double sa = sin(alpha);
		    double x2 =  ca*x+sa*y;
		    double y2 = -sa*x+ca*y;
		    x = x2;
		    y = y2;
		}

		return vec3(x,y,z);
	    },
	    ((scale.x == 0.0 && scale.y == 0.0)
	     ? GEOCSG::SWEEP_POLE : GEOCSG::SWEEP_CAP)
	);

	result->facets.connect();

	finalize_mesh(result);
	return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::rotate_extrude(
	const CSGScope& scope, double angle
    ) {
	std::shared_ptr<Mesh> result = union_instr(scope);
	result->vertices.set_dimension(2);

        if(angle == 360.0) {
            result->facets.clear();
            result->vertices.remove_isolated();
        }

	// Remove edges that are co-linear with rotation axis.
	// (as well as vertices dangling on rotation axis).
	{
	    index_t nb_edges_to_remove = 0;
	    vector<index_t> remove_edges(result->edges.nb(),0);
	    for(index_t e: result->edges) {
                index_t v1 = result->edges.vertex(e,0);
                index_t v2 = result->edges.vertex(e,1);
		if(
		    (result->vertices.point<2>(v1).x == 0.0) &&
		    (result->vertices.point<2>(v2).x == 0.0)
		) {
		    remove_edges[e] = 1;
		    ++nb_edges_to_remove;
		}
	    }
	    if(nb_edges_to_remove != 0) {
		result->edges.delete_elements(remove_edges);
		result->vertices.remove_isolated();
	    }
	}

        double R = 0.0;
        for(index_t v: result->vertices) {
            R = std::max(R, result->vertices.point<2>(v).x);
        }
        index_t slices = index_t(
	    GEOCSG::get_fragments_from_r_and_twist(R,angle,fn_,fs_,fa_)
	);
	index_t nv = slices+1;

	GEOCSG::sweep(
	    result, nv, [&](index_t u, index_t v)->vec3 {
		vec2 ref = result->vertices.point<2>(u);
		double t = double(v) / double(nv-1);
		double alpha = t * 2.0 * M_PI * angle / 360.0;
		return vec3(
		    ref.x * cos(alpha),
		    ref.x * sin(alpha),
		    ref.y
		);
	    },
	    (angle == 360.0) ? GEOCSG::SWEEP_PERIODIC : GEOCSG::SWEEP_CAP
	);

	// there may be duplicated points around the poles
	mesh_repair(
	    *result, MeshRepairMode(MESH_REPAIR_DEFAULT | MESH_REPAIR_QUIET)
	);

	// Note: we flip when angle is *positive* because X,Y,rotate dir
	// is not a direct basis.
	if(angle > 0) {
	    for(index_t t: result->facets) {
		result->facets.flip(t);
	    }
	}

	result->facets.connect();

	finalize_mesh(result);
	return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::projection(
	const CSGScope& scope, bool cut
    ) {
	std::shared_ptr<Mesh> result = append(scope);
        if(result->vertices.dimension() != 3) {
            Logger::err("CSG") << "projection(): input mesh is not of dimenion 3"
                               << std::endl;
	    result->clear();
            return result;
        }
        if(cut) {
	    // Cut mode: intersection between object and z=0 plane.
            // We reuse the intersection() function as follows:
	    // Compute intersection with (enlarged) half bbox and
	    // keep z=0 facets only.
	    auto [pmin, pmax] = get_bbox_bounds(result);
	    vec3 T(
		pmin.x - (pmax.x - pmin.x),
		pmin.y - (pmax.y - pmin.y),
		0.0
	    );
	    std::shared_ptr<Mesh> C = cube(3.0*(pmax-pmin), false);
            for(index_t v: C->vertices) {
		C->vertices.point<3>(v) += T;
            }
            CSGScope scope2;
            scope2.push_back(result);
            scope2.push_back(C);
            result = difference(scope2);
	    GEOCSG::keep_z0_only(result);
	    result->facets.compute_borders();
	    triangulate(result);
        } else {
	    // Union of all triangles projected in 2D. We project only
	    // half of them (since we have a closed shape). We select
	    // the orientation with the smallest number of triangles.
	    result->edges.clear();
	    vector<GEO::Sign> sign(result->facets.nb());
	    index_t nb_negative = 0;
	    index_t nb_positive = 0;
	    for(index_t t: result->facets) {
		index_t v1 = result->facets.vertex(t,0);
		index_t v2 = result->facets.vertex(t,1);
		index_t v3 = result->facets.vertex(t,2);
		vec2 p1 = result->vertices.point<2>(v1);
		vec2 p2 = result->vertices.point<2>(v2);
		vec2 p3 = result->vertices.point<2>(v3);
		sign[t] = GEO::PCK::orient_2d(p1,p2,p3);
		nb_negative += (sign[t] == GEO::NEGATIVE);
		nb_positive += (sign[t] == GEO::POSITIVE);
	    }
	    GEO::Sign keep_sign =
		(nb_positive > nb_negative) ? GEO::NEGATIVE : GEO::POSITIVE;

	    Attribute<index_t> e_operand_bit(
		result->edges.attributes(),"operand_bit"
	    );
	    for(index_t t: result->facets) {
		if(sign[t] == keep_sign) {
		    index_t v1 = result->facets.vertex(t,0);
		    index_t v2 = result->facets.vertex(t,1);
		    index_t v3 = result->facets.vertex(t,2);
		    index_t e1 = result->edges.create_edge(v1,v2);
		    index_t e2 = result->edges.create_edge(v2,v3);
		    index_t e3 = result->edges.create_edge(v3,v1);
		    // We are using the "cnstr_operand_bits_is_operand_id"
		    // of the "union" operation in CDT2d. It makes it
		    // possible to compute a union operation with more than
		    // 32 operands (and here we got one operand per input
		    // triangle !).
		    e_operand_bit[e1] = t;
		    e_operand_bit[e2] = t;
		    e_operand_bit[e3] = t;
		}
	    }
	    result->vertices.set_dimension(2);
	    triangulate(result,"union_cnstr_operand_bits_is_operand_id");
	    result->facets.compute_borders();
	    triangulate(result);
        }
	finalize_mesh(result);
        return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::append(const CSGScope& scope) {
	if(scope.size() == 0) {
	    return std::make_shared<Mesh>();
	}

        if(scope.size() == 1) {
            return scope[0];
        }

        if(!fast_union_ && scope.size() > max_arity_) {
            Logger::warn("CSG") << "Scope with more than "
                                << max_arity_
                                << " children"
                                << std::endl;
        }

	std::shared_ptr<Mesh> a = std::make_shared<Mesh>();
	index_t dim = 2;
	index_t nb_v = 0;
	index_t nb_e = 0;
	index_t nb_f = 0;
        for(const std::shared_ptr<Mesh>& b: scope) {
	    dim = std::max(dim, b->vertices.dimension());
	    nb_v += b->vertices.nb();
	    nb_e += b->edges.nb();
	    nb_f += b->facets.nb();
	}
        a->vertices.set_dimension(dim);
	a->vertices.create_vertices(nb_v);
	a->edges.create_edges(nb_e);
	a->facets.create_triangles(nb_f);

	index_t v_ofs = 0;
	index_t e_ofs = 0;
	index_t f_ofs = 0;

	Attribute<index_t> f_operand_bit;
	Attribute<index_t> e_operand_bit;

	if(a->vertices.dimension() == 3) {
	    f_operand_bit.bind(a->facets.attributes(), "operand_bit");
	}

	if(a->vertices.dimension() == 2) {
	    e_operand_bit.bind(a->edges.attributes(), "operand_bit");
	}

	index_t operand_bit = index_t(1);
	for(const std::shared_ptr<Mesh>& b : scope) {
	    for(index_t v: b->vertices) {
		for(index_t coord=0; coord<a->vertices.dimension(); ++coord) {
		    a->vertices.point_ptr(v + v_ofs)[coord] =
			(coord < b->vertices.dimension() ?
			 b->vertices.point_ptr(v)[coord] : 0.0);
		}
	    }

	    for(index_t f: b->facets) {
		geo_debug_assert(b->facets.nb_vertices(f) == 3);
		index_t v1 = b->facets.vertex(f,0);
		index_t v2 = b->facets.vertex(f,1);
		index_t v3 = b->facets.vertex(f,2);
		a->facets.set_vertex(f + f_ofs, 0, v1 + v_ofs);
		a->facets.set_vertex(f + f_ofs, 1, v2 + v_ofs);
		a->facets.set_vertex(f + f_ofs, 2, v3 + v_ofs);
		index_t f1 = b->facets.adjacent(f,0);
		index_t f2 = b->facets.adjacent(f,1);
		index_t f3 = b->facets.adjacent(f,2);
		if(f1 != NO_INDEX) {
		    f1 += f_ofs;
		}
		if(f2 != NO_INDEX) {
		    f2 += f_ofs;
		}
		if(f3 != NO_INDEX) {
		    f3 += f_ofs;
		}
		a->facets.set_adjacent(f + f_ofs, 0, f1);
		a->facets.set_adjacent(f + f_ofs, 1, f2);
		a->facets.set_adjacent(f + f_ofs, 2, f3);
		if(f_operand_bit.is_bound()) {
		    f_operand_bit[f + f_ofs] = operand_bit;
		}
	    }

	    for(index_t e: b->edges) {
		index_t v1 = b->edges.vertex(e,0);
		index_t v2 = b->edges.vertex(e,1);
		a->edges.set_vertex(e + e_ofs, 0, v1 + v_ofs);
		a->edges.set_vertex(e + e_ofs, 1, v2 + v_ofs);
		if(e_operand_bit.is_bound()) {
		    e_operand_bit[e + e_ofs] = operand_bit;
		}
	    }

	    v_ofs += b->vertices.nb();
	    e_ofs += b->edges.nb();
	    f_ofs += b->facets.nb();
	    operand_bit = operand_bit << 1;
        }
	return a;
    }

    void CSGBuilder::reset_defaults() {
        fa_ = DEFAULT_FA;
        fs_ = DEFAULT_FS;
        fn_ = DEFAULT_FN;
    }

    bool CSGBuilder::find_file(std::filesystem::path& filename) {
	for(const std::filesystem::path& path: file_path_) {
	    std::filesystem::path current_path = path / filename;
	    if(std::filesystem::is_regular_file(current_path)) {
		filename = current_path;
		return true;
	    }
	}
	return false;
    }

    std::shared_ptr<Mesh> CSGBuilder::import_with_openSCAD(
        const std::filesystem::path& filepath, const std::string& layer,
        index_t timestamp
    ) {
	GEOCSG::ArgList args;
	args.add_arg("file", filepath);
	args.add_arg("layer", layer);
	args.add_arg("timestamp", int(timestamp));
	bool TWO_D = true;
	std::shared_ptr<Mesh> result = GEOCSG::call_OpenSCAD(
	    current_path(), "import", args, TWO_D
	);
	result->facets.compute_borders();
	finalize_mesh(result);
        return result;
    }

    void CSGBuilder::do_CSG(
	std::shared_ptr<Mesh>& mesh, const std::string& boolean_expr
    ) {
	if(mesh->vertices.dimension() == 2) {
	    triangulate(mesh, boolean_expr);  // has all intersections inside
	    mesh->facets.clear();             // now keep edge borders only
	    mesh->vertices.remove_isolated(); // then remove internal vertices
	    triangulate(mesh);                // re-triangulate border edges
	} else {
            MeshSurfaceIntersection I(*mesh);
            I.set_verbose(verbose_);
	    I.set_fine_verbose(fine_verbose_);
            I.set_delaunay(delaunay_);
            I.set_detect_intersecting_neighbors(detect_intersecting_neighbors_);
            I.intersect();
            if(fast_union_ && boolean_expr == "union") {
                I.remove_internal_shells();
            } else {
                I.classify(boolean_expr);
            }
            if(simplify_coplanar_facets_) {
                I.simplify_coplanar_facets(coplanar_angle_tolerance_);
            }
	}
    }

    void CSGBuilder::triangulate(
        std::shared_ptr<Mesh>& mesh, const std::string& boolean_expr
    ) {
	geo_assert(mesh->vertices.dimension() == 2);

	// Keep edges only
	mesh->facets.clear();
	mesh->vertices.remove_isolated();

        bool has_operand_bit = Attribute<index_t>::is_defined(
            mesh->edges.attributes(), "operand_bit"
        );

	geo_assert(has_operand_bit);

        Attribute<index_t> e_operand_bit(
            mesh->edges.attributes(), "operand_bit"
        );

	#ifdef GEO_DEBUG
	{
	    bool all_zero = true;
	    for(index_t e: mesh->edges) {
		if(e_operand_bit[e] != 0 && e_operand_bit[e] != NO_INDEX) {
		    all_zero = false;
		}
	    }
	    geo_debug_assert(!all_zero);
	}
	#endif

	auto [pmin, pmax] = get_bbox_bounds(mesh);
	vec3 D = pmax - pmin;
	double d = std::max(std::max(D.x,D.y)*10.0, 1.0);
	D = vec3(d,d,0.0);
	pmin -= D;
	pmax += D;

	GEO::ExactCDT2d CDT;
        CDT.create_enclosing_rectangle(pmin.x, pmin.y, pmax.x, pmax.y);

        // In case there are duplicated vertices, keep track of indexing
        vector<index_t> vertex_id(mesh->vertices.nb());
        for(index_t v: mesh->vertices) {
            vec2 p = mesh->vertices.point<2>(v);
            vertex_id[v] = CDT.insert(
		GEO::ExactCDT2d::ExactPoint(p.x, p.y, 1.0), v
	    );
        }

        // Memorize current number of vertices to detect vertices
        // coming from constraint intersections
        index_t nv0 = CDT.nv();

        // Insert constraints
	for(index_t e: mesh->edges) {
	    index_t v1 = mesh->edges.vertex(e,0);
	    index_t v2 = mesh->edges.vertex(e,1);
	    CDT.insert_constraint(
		vertex_id[v1], vertex_id[v2], e_operand_bit[e]
	    );
	}

	CDT.classify_triangles(boolean_expr);

        // Create vertices coming from constraint intersections
        for(index_t v=nv0; v<CDT.nv(); ++v) {
	    GEO::vec2 p = GEO::PCK::approximate(CDT.point(v));
            CDT.set_vertex_id(
                v,
                mesh->vertices.create_vertex(vec2(p.x, p.y))
            );
        }

        // Create triangles in target mesh
        for(index_t t=0; t<CDT.nT(); ++t) {
            mesh->facets.create_triangle(
                CDT.vertex_id(CDT.Tv(t,0)),
                CDT.vertex_id(CDT.Tv(t,1)),
                CDT.vertex_id(CDT.Tv(t,2))
            );
        }

	mesh->facets.connect();
	mesh->facets.compute_borders();
    }

    void CSGBuilder::triangulate(std::shared_ptr<Mesh>& mesh) {
	Attribute<index_t> e_operand_bit(
	    mesh->edges.attributes(),"operand_bit"
	);
	for(index_t e: mesh->edges) {
	    e_operand_bit[e] = index_t(1);
	}
	triangulate(mesh, "union");
    }

    void CSGBuilder::finalize_mesh(std::shared_ptr<Mesh>& M) {
	geo_argused(M);
    }

    Box3d CSGBuilder::get_bbox(const std::shared_ptr<Mesh>& M) {
	Box3d result;
	for(index_t c=0; c<3; ++c) {
	    result.xyz_min[c] = Numeric::max_float64();
	    result.xyz_max[c] = -Numeric::max_float64();
	}
	for(index_t v: M->vertices) {
	    const double* pp = M->vertices.point_ptr(v);
	    vec3 p(
		pp[0], pp[1], (M->vertices.dimension() == 3) ? pp[2] : 0.0
	    );
	    for(index_t c=0; c<3; ++c) {
		result.xyz_min[c] = std::min(result.xyz_min[c], p[c]);
		result.xyz_max[c] = std::max(result.xyz_max[c], p[c]);
	    }
	}
	return result;
    }
}
