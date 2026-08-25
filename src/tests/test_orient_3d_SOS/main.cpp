/*
 *  Copyright (c) 2000-2026 Inria
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
#include <geogram/basic/stopwatch.h>
#include <geogram/numerics/predicates.h>
#include <geogram/mesh/mesh_CSG_builder.h>
#include <geogram/mesh/mesh_io.h>

namespace {
    using namespace GEO;

    /**
     * \brief Builds the Omega surface, that corresponds to a cube minus a corner
     */
    std::shared_ptr<GEO::Mesh> build_Omega() {
        CSGBuilder B;
	return B.difference({
		B.cube(vec3(1.0, 1.0, 1.0), false),
		B.cube(vec3(0.5, 0.5, 0.5), false)
	});
    }

    enum Status {INSIDE, UNDETERMINED, OUTSIDE};

    /**
     * \brief Tests where a point is realtive to Omega
     * \details This version uses hardcoded tests (supposed to be correct).
     * \param[in] p the 3d point to be tested
     * \retval INSIDE if \p p is inside Omega
     * \retval UNDETERMINED if \p p is exactly on the boundary of Omega
     * \retval OUTSIDE if \p p is outside Omega
     */
    Status where_is(const vec3& p) {
	if(p.x < 0.0 || p.y < 0.0 || p.z < 0.0) {
	    return OUTSIDE;
	}
	if(p.x > 1.0 || p.y > 1.0 || p.z > 1.0) {
	    return OUTSIDE;
	}
	if(p.x <= 0.5 && p.y <= 0.5 && p.z <= 0.5) {
	    if(p.x < 0.5 && p.y < 0.5 && p.z < 0.5) {
		return OUTSIDE;
	    }
	    return UNDETERMINED;
	}
	if(
	    p.x > 0.0 && p.y > 0.0 && p.z > 0.0 &&
	    p.x < 1.0 && p.y < 1.0 && p.z < 1.0
	) {
	    return INSIDE;
	}
	return UNDETERMINED;
    }

    /**
     * \brief Tests whether a segment intersects a triangle
     * \param[in] q1 , q2 the two extremities of the segment
     * \param[in] p1 , p2 , p3 the three verties of the triangle
     * \retval true if the segment has an intersection with the
     *  interior of the triangle
     * \retval false otherwise
     * \details Degenerate configurations (segment passing through vertex,
     *  edge, or co-planar with triangle) are symbolically perturbed.
     */
    template <class POINT> bool segment_triangle_intersection_SOS(
        const POINT& q1, const POINT& q2,
        const POINT& p1, const POINT& p2, const POINT& p3
    ) {
        Sign o1 = PCK::orient_3d_SOS(q1,p1,p2,p3);
        Sign o2 = PCK::orient_3d_SOS(q2,p1,p2,p3);

	// There is no intersection if q1 and q2 are on the
	// same side of the supporting plane of (p1,p2,p3)
        if(o1 == o2) {
            return false;
        }

	// There is an intersection if the three tetrahedra
	// formed by [q1,q2] and the three edges of the triangle
	// have the same orientation
        Sign s1 = PCK::orient_3d_SOS(q1,q2,p1,p2);
        Sign s2 = PCK::orient_3d_SOS(q1,q2,p2,p3);
	if(s1*s2 < 0) {
	    return false;
	}
        Sign s3 = PCK::orient_3d_SOS(q1,q2,p3,p1);
        return(s2*s3 > 0 && s3*s1 > 0);
    }

    /**
     * \brief Tests where a point is realtive to Omega
     * \details This version uses orient_3d_SOS. One can use the other version
     *  of where_is() to test whether this one is correct.
     * \param[in] p the 3d point to be tested
     * \retval INSIDE if \p p is inside Omega
     * \retval OUTSIDE if \p p is outside Omega
     * \retval one of INSIDE or OUTSIDE if \p p is on the border of Omega
     */
    Status where_is(const vec3& q1, const vec3& q2, const Mesh& Omega) {
	bool inside = false;
	for(index_t f: Omega.facets) {
	    vec3 p1 = Omega.facets.point(f,0);
	    vec3 p2 = Omega.facets.point(f,1);
	    vec3 p3 = Omega.facets.point(f,2);
	    if(segment_triangle_intersection_SOS(q1,q2,p1,p2,p3)) {
		inside = !inside;
	    }
	}
	return inside ? INSIDE: OUTSIDE;
    }
}

int main(int argc, char** argv) {

    using namespace GEO;

    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
    int result = 0;

    try {
        Stopwatch W("Total time");

        std::vector<std::string> filenames;

        CmdLine::import_arg_group("standard");
        CmdLine::import_arg_group("algo");
	CmdLine::declare_arg(
	    "visual_debug", false, "save domain and points in file"
	);
	CmdLine::declare_arg(
	    "h", 0.25, "space between points (must be in the form of 1/2^n)"
	);
	CmdLine::declare_arg(
	    "dh", 1.0, "increment for directions (must be in the form of 1/2^n)"
	);

	CmdLine::declare_arg(
	    "lo", -0.25, "low bound for coordinates to be tested"
	);

	CmdLine::declare_arg(
	    "hi", 1.25, "high bound for coordinates to be tested"
	);

        if(!CmdLine::parse(argc, argv, filenames)) {
            return 1;
        }

	std::shared_ptr<Mesh> Omega = build_Omega();

	double h  = CmdLine::get_arg_double("h");
	double dh = CmdLine::get_arg_double("dh");
	double lo = CmdLine::get_arg_double("lo");
	double hi = CmdLine::get_arg_double("hi");

	if(CmdLine::get_arg_bool("visual_debug")) {
	    mesh_save(*Omega, "Omega.geogram");
	    {
		Mesh P;
		Attribute<index_t> status(P.vertices.attributes(), "status");
		for(double x=lo; x<=hi; x+=h) {
		    for(double y=lo; y<=hi; y+=h) {
			for(double z=lo; z<=hi; z+=h) {
			    vec3 p(x,y,z);
			    status[P.vertices.create_vertex(p)] =
				index_t(where_is(p));
			}
		    }
		}
		mesh_save(P, "P.geogram");
	    }
	}

	index_t nb_OK = 0;
	index_t nb_KO = 0;

	for(double x=lo; x<=hi; x+=h) {
	    for(double y=lo; y<=hi; y+=h) {
		for(double z=lo; z<=hi; z+=h) {
		    vec3 q1(x,y,z);
		    for(double dx=-1.0; dx<=1.0; dx+=dh) {
			for(double dy=-1.0; dy<=1.0; dy+=dh) {
			    for(double dz=-1.0; dz<=1.0; dz+=dh) {
				if(dx==0.0 && dy==0.0 && dz==0.0) {
				    continue;
				}
				double S = 1024.0;
				vec3 q2(x+S*dx,y+S*dy,z+S*dz);

				Status status = where_is(q1,q2,*Omega);
				Status check_status = where_is(q1);

				if(
				    check_status != UNDETERMINED &&
				    status != check_status
				) {
				    ++nb_KO;
				} else {
				    ++nb_OK;
				}
			    }
			}
		    }
		}
	    }
	}
	Logger::out("test_orient_3d_SOS")
	    << " nb_OK:" << nb_OK
	    << " nb_KO:" << nb_KO
	    << std::endl;
	result = int(nb_KO != 0);
    }
    catch(const std::exception& e) {
        std::cerr << "Received an exception: " << e.what() << std::endl;
        return 1;
    }

    return result;
}
