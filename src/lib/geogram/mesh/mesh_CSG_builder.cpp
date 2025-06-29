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
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/delaunay/parallel_delaunay_3d.h>
#include <geogram/delaunay/CDT_2d.h>
#include <geogram/basic/command_line.h>

/******************************************************************************/
/* Utility functions taken from OpenSCAD/utils/calc.cc                        */
/******************************************************************************/

namespace Calc {

    // From openscad/Geometry/Grid.h
    static constexpr double GRID_FINE = 0.00000095367431640625;
    // This one often misses so I redeclare it here
    static constexpr double M_DEG2RAD = M_PI / 180.0;

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

    int get_fragments_from_r(
	double r, double fn, double fs, double fa
    ) {
	return get_fragments_from_r_and_twist(r, 360.0, fn, fs, fa);
    }

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

    // This one is not part of OpenSCAD, I copied it from
    // extrudePolygon() in geometry/GeometryEvaluator.cc
    // (with some readaptations / reordering to make it
    // easier to understand, at least for me)
    int get_linear_extrusion_slices(
	std::shared_ptr<GEO::Mesh> M,
	double height, GEO::vec2 scale, double twist,
	double fn, double fs, double fa
    ) {

	if(twist == 0.0 && scale.x == scale.y) {
	    return 1;
	}

	double max_r1_sqr = 0.0;  // r1 is before scaling
	double max_delta_sqr = 0; // delta from before/after scaling
	for(GEO::index_t iv: M->vertices) {
	    const GEO::vec2& v = M->vertices.point<2>(iv);
	    max_r1_sqr = std::max(max_r1_sqr, GEO::length2(v));
	    GEO::vec2 scale_v(v.x*scale.x, v.y*scale.y);
	    max_delta_sqr = std::max(
		max_delta_sqr, GEO::length2(v - scale_v)
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

/******************************************************************************/

}

namespace {
    using namespace GEO;

    /**
     * \brief Symbolic constants for sweep()
     */
    enum SweepCapping {
	SWEEP_CAP,
	SWEEP_POLE,
	SWEEP_PERIODIC
    };

    /**
     * \brief The generalized sweeping operation
     * \details Used to implement sphere(), cylinder(), linear_extrude() and
     *  rotate_extrude()
     * \param[in,out] mesh on entry, a 2D mesh. On exit, a 3D mesh. The triangles
     *  present in the mesh are used to generate the caps. They are copied to
     *  generate the second cap if \p capping is set to SWEEP_CAP (default).
     * \param[in] nv number of sweeping steps. Minimum is 2.
     * \param[in] sweep_path a function that maps u,v indices to 3D
     *  points, where u is the index of a initial 2D vertex and v
     *  in [0..nv-1] the sweeping step. One can use the point at vertex
     *  u to evaluate the path (it will not be overwritten before calling
     *  sweep_path()). Note that u vertices are not necessarily ordered.
     * \param[in] capping one of:
     *   - SWEEP_CAP standard sweeping, generate second capping by
     *     copying first one
     *   - SWEEP_POLE if last sweeping step degenerates to a
     *     single point
     *   - SWEEP_PERIODIC if no cappings should be generated and last
     *     sweeping step corresponds to first one
     */
    template <class PATH> void sweep(
	std::shared_ptr<Mesh>& M,
	index_t nv,
	PATH sweep_path,
	SweepCapping capping = SWEEP_CAP
    ) {
	M->vertices.set_dimension(2);
	index_t nu = M->vertices.nb();

	index_t total_nb_vertices;
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
	    for(index_t e: M->edges) {
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
	    for(index_t e: M->edges) {
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
	    nu = index_t(Calc::get_fragments_from_r(r, fn_, fs_, fa_));
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

	finalize_mesh(M);

	return M;
    }

    std::shared_ptr<Mesh> CSGBuilder::sphere(double r) {
        index_t nu = index_t(Calc::get_fragments_from_r(r,fn_,fs_,fa_));
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

	sweep(
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

	finalize_mesh(result);
	return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::cylinder(
	double h, double r1, double r2, bool center
    ) {
        index_t nu = index_t(
	    Calc::get_fragments_from_r(
		std::max(r1,r2),fn_,fs_,fa_
	    )
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
	sweep(
	    result, 2,
	    [&](index_t u, index_t v)->vec3 {
		vec3 p = result->vertices.point(u);
		return vec3(
		    r[v]*p.x, r[v]*p.y, z[v]
		);
	    },
	    (r[1] == 0.0) ? SWEEP_POLE : SWEEP_CAP
	);

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
            MeshIOFlags io_flags;
            io_flags.set_verbose(verbose_);
            if(!mesh_load(full_filename, *result, io_flags)) {
                result->clear();
                return result;
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
	    bool all_z0 = true;
	    for(const vec3& p: result->vertices.points()) {
		if(p.z != 0.0) {
		    all_z0 = false;
		    break;
		}
	    }
	    if(all_z0) {
		result->vertices.set_dimension(2);
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
	    triangulate(result, "union");
	}

        finalize_mesh(result);
        return result;
    }

    std::shared_ptr<Mesh> CSGBuilder::surface_with_OpenSCAD(
        const std::string& filename, bool center, bool invert
    ) {
        Logger::out("CSG") << "Handling surface() with OpenSCAD" << std::endl;

        // Generate a simple linear extrusion, so that we can convert to STL
        // (without it OpenSCAD refuses to create a STL with 2D content)
        std::ofstream tmp("tmpscad.scad");
        tmp << "surface(" << std::endl;
	tmp << "  file=\"" << filename << "\"," << std::endl;
        tmp << "  center=" << String::to_string(center) << "," << std::endl;
        tmp << "  invert=" << String::to_string(invert) << std::endl;
        tmp << ");" << std::endl;

        // Start OpenSCAD and generate output as STL
        if(system("openscad tmpscad.scad -o tmpscad.stl")) {
            Logger::warn("CSG") << "Error while running openscad " << std::endl;
            Logger::warn("CSG") << "(used to generate surface) " << std::endl;
        }

        // Load STL using our own loader
	std::shared_ptr<Mesh> result = import("tmpscad.stl");

	//std::filesystem::remove("tmpscad.scad");
	std::filesystem::remove("tmpscad.stl");

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
        Logger::out("CSG") << "Handling text() with OpenSCAD" << std::endl;

	if(text == "") {
	    std::shared_ptr<Mesh> result = std::make_shared<Mesh>();
	    return result;
	}

        // Generate a simple linear extrusion, so that we can convert to STL
        // (without it OpenSCAD refuses to create a STL with 2D content)
        std::ofstream tmp("tmpscad.scad");
        tmp << "group() {" << std::endl;
        tmp << "   linear_extrude(height=1.0) {" << std::endl;
        tmp << "      text(" << std::endl;
	tmp << "             \"" << text << "\"," << std::endl;
        tmp << "             size=" << size << "," << std::endl;
	if(font != "") {
	    tmp << "             font=\"" << font << "\"," << std::endl;
	}
	tmp << "             halign=\"" << halign << "\"," << std::endl;
	tmp << "             valign=\"" << valign << "\"," << std::endl;
	tmp << "             spacing=" << spacing << "," << std::endl;
	tmp << "             direction=\"" << direction << "\"," << std::endl;
	tmp << "             language=\"" << language<< "\"," << std::endl;
	tmp << "             script=\"" << script << "\"" << std::endl;
        tmp << "      );" << std::endl;
        tmp << "   }" << std::endl;
        tmp << "}" << std::endl;

        // Start OpenSCAD and generate output as STL
        if(system("openscad tmpscad.scad -o tmpscad.stl")) {
            Logger::warn("CSG") << "Error while running openscad " << std::endl;
            Logger::warn("CSG") << "(used to generate text) " << std::endl;
        }

        // Load STL using our own loader
	std::shared_ptr<Mesh> result = import("tmpscad.stl");


	std::filesystem::remove("tmpscad.scad");
	std::filesystem::remove("tmpscad.stl");

        // Delete the facets that are coming from the linear extrusion
	keep_z0_only(result);
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
            scope3.emplace_back(union_instr(scope1));
            scope3.emplace_back(union_instr(scope2));
            return union_instr(scope3);
	}

	std::shared_ptr<Mesh> result = append(scope);
	do_CSG(result, "union");
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
		Calc::get_linear_extrusion_slices(
		    result, height, scale, twist, fn_, fs_, fa_
		)
	    );
	}

	index_t nv = slices+1;

	sweep(
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
	    ((scale.x == 0.0 && scale.y == 0.0) ? SWEEP_POLE : SWEEP_CAP)
	);

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
	    Calc::get_fragments_from_r_and_twist(R,angle,fn_,fs_,fa_)
	);
	index_t nv = slices+1;

	sweep(
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
	    (angle == 360.0) ? SWEEP_PERIODIC : SWEEP_CAP
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
	    keep_z0_only(result);
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
	    for(index_t e: result->edges) {
		e_operand_bit[e] = index_t(1);
	    }
	    triangulate(result,"union");
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
	std::shared_ptr<Mesh> result;
        std::filesystem::path path = filepath;
	path.remove_filename();

	std::filesystem::path base = filepath.filename();
	base.replace_extension("");

	std::string extension = filepath.extension().string().substr(1);

	std::filesystem::path geogram_filepath =
	    path / std::filesystem::path(
		std::string("geogram_") + base.c_str() + "_" + extension + "_"
		+ layer + "_" +  String::to_string(timestamp) + ".stl"
	    );

	if(std::filesystem::is_regular_file(geogram_filepath)) {
	    result = import(geogram_filepath);
	    return result;
	}

        Logger::out("CSG") << "Did not find " << geogram_filepath << std::endl;
        Logger::out("CSG") << "Trying to create it with OpenSCAD" << std::endl;

        // Generate a simple linear extrusion, so that we can convert to STL
        // (without it OpenSCAD refuses to create a STL with 2D content)
        std::ofstream tmp("tmpscad.scad");
        tmp << "group() {" << std::endl;
        tmp << "   linear_extrude(height=1.0) {" << std::endl;
        tmp << "      import(" << std::endl;
        tmp << "          file = \"" << filepath.c_str() << "\"," << std::endl;
        tmp << "          layer = \"" << layer << "\"," << std::endl;
        tmp << "          timestamp = " << timestamp << std::endl;
        tmp << "      );" << std::endl;
        tmp << "   }" << std::endl;
        tmp << "}" << std::endl;

        // Start OpenSCAD and generate output as STL
        if(system("openscad tmpscad.scad -o tmpscad.stl")) {
            Logger::warn("CSG") << "Error while running openscad " << std::endl;
            Logger::warn("CSG") << "(used to import " << filepath << ")"
                                << std::endl;
        }

        // Load STL using our own loader
        result = import("tmpscad.stl");

        std::filesystem::remove("tmpscad.scad");
	std::filesystem::remove("tmpscad.stl");

        // Delete the facets that are coming from the linear extrusion
	keep_z0_only(result);
	result->vertices.set_dimension(3);
        mesh_save(*result, geogram_filepath);
	result->vertices.set_dimension(2);

        return result;
    }

    void CSGBuilder::do_CSG(
	std::shared_ptr<Mesh>& mesh, const std::string& boolean_expr
    ) {
	if(mesh->vertices.dimension() == 2) {
	    triangulate(mesh, boolean_expr);  // has all intersections inside
	    mesh->facets.clear();             // now keep edge borders only
	    mesh->vertices.remove_isolated(); // then remove internal vertices
	    triangulate(mesh, "union");       // re-triangulate border edges
	} else {
	    // Insert your own mesh boolean operation code here !
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

        Attribute<index_t> e_operand_bit(
            mesh->edges.attributes(), "operand_bit"
        );

	for(index_t e: mesh->edges) {
	    if(e_operand_bit[e] == 0 || e_operand_bit[e] == NO_INDEX) {
		has_operand_bit = false;
		break;
	    }
	}

        if(!has_operand_bit) {
            for(index_t e: mesh->edges) {
                e_operand_bit[e] = index_t(1);
            }
        }

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
	for(index_t e: mesh->edges) {
	    e_operand_bit[e] = index_t(1);
	}
    }

    void CSGBuilder::keep_z0_only(std::shared_ptr<Mesh>& M) {
	vector<index_t> remove_triangle(M->facets.nb(),0);
	for(index_t t: M->facets) {
	    for(index_t lv=0; lv<3; ++lv) {
		const vec3& p = M->vertices.point<3>(M->facets.vertex(t,lv));
		if(p.z != 0.0) {
		    remove_triangle[t] = 1;
		    break;
		}
	    }
	}
	M->facets.delete_elements(remove_triangle);
	M->vertices.remove_isolated();
	M->facets.compute_borders();
	M->vertices.set_dimension(2);
	triangulate(M,"union");
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
