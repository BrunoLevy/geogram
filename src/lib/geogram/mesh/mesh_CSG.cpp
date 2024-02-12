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

#include <geogram/mesh/mesh_CSG.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_surface_intersection.h>
#include <geogram/mesh/mesh_surface_intersection_internal.h>
#include <geogram/mesh/mesh_fill_holes.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/delaunay/delaunay.h>
#include <geogram/image/image_library.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/file_system.h>
#include <geogram/basic/progress.h>
#include <geogram/basic/line_stream.h>

#include <cstdlib> 

// Silence some warnings in stb_c_lexer.h

#ifdef GEO_COMPILER_MSVC
#pragma warning (push)
#pragma warning (disable: 4505) // stb__strchr unreferenced function
#endif

#ifdef GEO_COMPILER_GCC_FAMILY
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wunused-function"
#ifdef GEO_COMPILER_CLANG
#pragma clang diagnostic ignored "-Wreserved-id-macro"
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma clang diagnostic ignored "-Wself-assign"
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#pragma clang diagnostic ignored "-Wunused-member-function"
#pragma clang diagnostic ignored "-Wcast-qual"
#pragma clang diagnostic ignored "-Wunused-macros"
#endif
#endif

// We need to change stb-c-lexer configuration
// because we need to undefined STB_C_LEX_DISCARD_PREPROCESSOR
// (we need to parse '#', it is an instruction/object 'modifier')
// Begin stb-c-lexer configuration (see geogram/third_party/stb_c_lexer.h)
#if defined(Y) || defined(N)
#error "'Y' or 'N' already defined, cannot use stb-c-lexer"
#endif

#define STB_C_LEX_C_DECIMAL_INTS          Y   
#define STB_C_LEX_C_HEX_INTS              Y   
#define STB_C_LEX_C_OCTAL_INTS            Y   
#define STB_C_LEX_C_DECIMAL_FLOATS        Y   
#define STB_C_LEX_C99_HEX_FLOATS          N   
#define STB_C_LEX_C_IDENTIFIERS           Y   
#define STB_C_LEX_C_DQ_STRINGS            Y   
#define STB_C_LEX_C_SQ_STRINGS            N   
#define STB_C_LEX_C_CHARS                 Y   
#define STB_C_LEX_C_COMMENTS              Y   
#define STB_C_LEX_CPP_COMMENTS            Y   
#define STB_C_LEX_C_COMPARISONS           Y   
#define STB_C_LEX_C_LOGICAL               Y   
#define STB_C_LEX_C_SHIFTS                Y   
#define STB_C_LEX_C_INCREMENTS            Y   
#define STB_C_LEX_C_ARROW                 Y   
#define STB_C_LEX_EQUAL_ARROW             N   
#define STB_C_LEX_C_BITWISEEQ             Y   
#define STB_C_LEX_C_ARITHEQ               Y   
#define STB_C_LEX_PARSE_SUFFIXES          N   
#define STB_C_LEX_DECIMAL_SUFFIXES        ""  
#define STB_C_LEX_HEX_SUFFIXES            ""  
#define STB_C_LEX_OCTAL_SUFFIXES          ""  
#define STB_C_LEX_FLOAT_SUFFIXES          ""  
#define STB_C_LEX_0_IS_EOF                N  
#define STB_C_LEX_INTEGERS_AS_DOUBLES     N  
#define STB_C_LEX_MULTILINE_DSTRINGS      N  
#define STB_C_LEX_MULTILINE_SSTRINGS      N  
#define STB_C_LEX_USE_STDLIB              Y  
#define STB_C_LEX_DOLLAR_IDENTIFIER       Y  
#define STB_C_LEX_FLOAT_NO_DECIMAL        Y  
#define STB_C_LEX_DEFINE_ALL_TOKEN_NAMES  N   
#define STB_C_LEX_DISCARD_PREPROCESSOR    N // we just changed this one
#define STB_C_LEXER_DEFINITIONS
// end stb-c-lexer configuration

#define STB_C_LEXER_IMPLEMENTATION
#include <geogram/third_party/stb/stb_c_lexer.h>

#ifdef GEO_COMPILER_GCC_FAMILY
#pragma GCC diagnostic pop
#endif

#ifdef GEO_COMPILER_MSVC
#pragma warning (pop)
#endif


namespace {
    using namespace GEO;
    static constexpr int CLEX_booleanlit = CLEX_first_unused_token;
    stb_lexer& getlex(void* lex_) {
        return *reinterpret_cast<stb_lexer*>(lex_);
    }
}


namespace GEO {

    CSGMesh::CSGMesh() {
        for(index_t c=0; c<3; ++c) {
            bbox_.xyz_min[c] =  Numeric::max_float64();
            bbox_.xyz_max[c] = -Numeric::max_float64();
        }
    }
    
    CSGMesh::~CSGMesh() {
    }

    bool CSGMesh::bbox_initialized() const {
        return (
            bbox_.xyz_min[0] < bbox_.xyz_max[0] &&
            bbox_.xyz_min[1] < bbox_.xyz_max[1] &&
            bbox_.xyz_min[2] < bbox_.xyz_max[2] 
        );
    }
    
    void CSGMesh::update_bbox() {
        for(index_t c=0; c<3; ++c) {
            bbox_.xyz_min[c] =  Numeric::max_float64();
            bbox_.xyz_max[c] = -Numeric::max_float64();
        }
        for(index_t v: vertices) {
            const double* p = vertices.point_ptr(v);
            for(index_t c=0; c<3; ++c) {
                double coord = (c < vertices.dimension()) ? p[c] : 0.0;
                bbox_.xyz_min[c] = std::min(bbox_.xyz_min[c], coord);
                bbox_.xyz_max[c] = std::max(bbox_.xyz_max[c], coord);
            }
        }
        // Enlarge boxes a bit
        for(index_t c=0; c<3; ++c) {
            bbox_.xyz_min[c] -= 1e-6;
            bbox_.xyz_max[c] += 1e-6;
            /*
            // nextafter triggers FPEs with denormals
            bbox_.xyz_min[c] = std::nextafter(
                bbox_.xyz_min[c], -std::numeric_limits<double>::infinity()
            );
            bbox_.xyz_max[c] = std::nextafter(
                bbox_.xyz_max[c],  std::numeric_limits<double>::infinity()
            );
            */
        }
    }

    void CSGMesh::append_mesh(const CSGMesh* other, index_t operand) {
        Mesh* a = this;
        const Mesh* b = other;
        if(
            a->vertices.nb() == 0 ||
            b->vertices.dimension() > a->vertices.dimension()
        ) {
            a->vertices.set_dimension(b->vertices.dimension());
        }
        geo_assert(a->facets.are_simplices());
        geo_assert(b->facets.are_simplices());
        index_t v_ofs = a->vertices.nb();
        index_t f_ofs = a->facets.nb();
        index_t e_ofs = a->edges.nb();
        a->vertices.create_vertices(b->vertices.nb());
        a->facets.create_triangles(b->facets.nb());
        for(index_t v: b->vertices) {
            for(index_t c=0; c<a->vertices.dimension(); ++c) {
                a->vertices.point_ptr(v + v_ofs)[c] =
                    (c < b->vertices.dimension() ?
                     b->vertices.point_ptr(v)[c] : 0.0);
            }
        }
        for(index_t f: b->facets) {
            index_t v1 = b->facets.vertex(f,0);
            index_t v2 = b->facets.vertex(f,1);
            index_t v3 = b->facets.vertex(f,2);
            a->facets.set_vertex(f + f_ofs, 0, v1 + v_ofs);
            a->facets.set_vertex(f + f_ofs, 1, v2 + v_ofs);
            a->facets.set_vertex(f + f_ofs, 2, v3 + v_ofs);
            index_t f1 = b->facets.adjacent(f,0);
            index_t f2 = b->facets.adjacent(f,1);
            index_t f3 = b->facets.adjacent(f,2);
            if(f1 != index_t(-1)) {
                f1 += f_ofs;
            }
            if(f2 != index_t(-1)) {
                f2 += f_ofs;
            }
            if(f3 != index_t(-1)) {
                f3 += f_ofs;
            }
            a->facets.set_adjacent(f + f_ofs, 0, f1);
            a->facets.set_adjacent(f + f_ofs, 1, f2);
            a->facets.set_adjacent(f + f_ofs, 2, f3); 
        }
        for(index_t e: b->edges) {
            index_t v1 = b->edges.vertex(e,0);
            index_t v2 = b->edges.vertex(e,1);
            a->edges.create_edge(v1 + v_ofs, v2 + v_ofs);
        }
        
        for(index_t c=0; c<3; ++c) {
            bbox_.xyz_min[c] = std::min(
                bbox_.xyz_min[c], other->bbox().xyz_min[c]
            );
            bbox_.xyz_max[c] = std::max(
                bbox_.xyz_max[c], other->bbox().xyz_max[c]
            );
        }
        if(operand != index_t(-1)) {
            Attribute<index_t> f_operand_bit(facets.attributes(),"operand_bit");
            for(index_t f=f_ofs; f<facets.nb(); ++f) {
                f_operand_bit[f] = index_t(1) << operand;
            }
            Attribute<index_t> e_operand_bit(edges.attributes(),"operand_bit");
            for(index_t e=e_ofs; e<edges.nb(); ++e) {
                e_operand_bit[e] = index_t(1) << operand;
            }
        }
    }

    /**********************************************************************/
    
    CSGBuilder::CSGBuilder() {
        reset_defaults();
        reset_file_path();        
        STL_epsilon_ = 1e-6;
        verbose_ = false;
        max_arity_ = 32;
        simplify_coplanar_facets_ = true;
        coplanar_angle_tolerance_ = 0.0;
        delaunay_ = true;
        detect_intersecting_neighbors_ = true;
        fast_union_ = false;
    }
    
    void CSGBuilder::reset_defaults() {
        fa_ = DEFAULT_FA;
        fs_ = DEFAULT_FS;
        fn_ = DEFAULT_FN;
    }

    CSGMesh_var CSGBuilder::square(vec2 size, bool center) {
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

        CSGMesh_var M = new CSGMesh;
        M->vertices.set_dimension(2);

        if(size.x <= 0.0 || size.y <= 0.0) {
            Logger::warn("CSG")
                << "square with negative size (returning empty shape)"
                << std::endl;
            return M;
        }
        
        M->vertices.create_vertex(vec2(x1,y1).data());
        M->vertices.create_vertex(vec2(x2,y1).data());
        M->vertices.create_vertex(vec2(x1,y2).data());
        M->vertices.create_vertex(vec2(x2,y2).data());

        M->facets.create_triangle(0,3,1);
        M->facets.create_triangle(0,2,3);
            
        M->facets.connect();
        M->facets.compute_borders();
        M->update_bbox();
        return M;
    }

    CSGMesh_var CSGBuilder::circle(double r) {
        
        index_t nu = get_fragments_from_r(r);

        CSGMesh_var M = new CSGMesh;
        M->vertices.set_dimension(2);

        if(r <= 0.0) {
            Logger::warn("CSG")
                << "circle with negative radius (returning empty shape)"
                << std::endl;
            return M;
        }
        
        for(index_t u=0; u<nu; ++u) {
            double theta = double(u)*2.0*M_PI/double(nu);
            double ctheta = cos(theta);
            double stheta = sin(theta);
            double x = ctheta*r;
            double y = stheta*r;
            M->vertices.create_vertex(vec2(x,y).data());
        }

        for(index_t u=1; u+1<nu; ++u) {
            M->facets.create_triangle(0,u,u+1);
        }
            
        M->facets.connect();
        M->facets.compute_borders();        
        M->update_bbox();
        return M;
    }
    
    CSGMesh_var CSGBuilder::cube(vec3 size, bool center) {
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

        CSGMesh_var M = new CSGMesh;
        M->vertices.set_dimension(3);

        if(size.x <= 0.0 || size.y <= 0.0 || size.z <= 0.0) {
            Logger::warn("CSG")
                << "cube with negative size (returning empty shape)"
                << std::endl;
            return M;
        }
        
        M->vertices.create_vertex(vec3(x1,y1,z1).data());
        M->vertices.create_vertex(vec3(x2,y1,z1).data());
        M->vertices.create_vertex(vec3(x1,y2,z1).data());
        M->vertices.create_vertex(vec3(x2,y2,z1).data());
        M->vertices.create_vertex(vec3(x1,y1,z2).data());
        M->vertices.create_vertex(vec3(x2,y1,z2).data());
        M->vertices.create_vertex(vec3(x1,y2,z2).data());
        M->vertices.create_vertex(vec3(x2,y2,z2).data());

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
        M->update_bbox();
        return M;
    }
    
    CSGMesh_var CSGBuilder::sphere(double r) {
        index_t nu = get_fragments_from_r(r);
        index_t nv = index_t((nu / 2) + 1); // TODO: I do not have exactly the
                                            // same parameterization as OpenSCAD
                                            // in v ...

        CSGMesh_var M = new CSGMesh;
        M->vertices.set_dimension(3);

        if(r <= 0.0) {
            Logger::warn("CSG")
                << "sphere with negative radius (returning empty shape)"
                << std::endl;
            return M;
        }
        
        // First pole
        M->vertices.create_vertex(vec3(0.0, 0.0, -r).data());
        // All vertices except poles
        for(index_t v=1; v<nv-1; ++v) {
            double phi = double(v)*M_PI/double(nv-1) - M_PI/2.0;

            double cphi = cos(phi);
            double sphi = sin(phi);
            for(index_t u=0; u<nu; ++u) {
                double theta = double(u)*2.0*M_PI/double(nu);
                double ctheta = cos(theta);
                double stheta = sin(theta);
                double x = r*ctheta*cphi;
                double y = r*stheta*cphi;
                double z = r*sphi;
                M->vertices.create_vertex(vec3(x,y,z).data());
            }
        }
        // Second pole
        M->vertices.create_vertex(vec3(0.0, 0.0, r).data());

        // Maps param coordinates to mesh index, taking into
        // account poles (at v=0 and v=nv-1)
        auto vindex = [nu,nv](index_t u, index_t v)->index_t {
            if(v==0) {
                return 0;
            }
            if(v==(nv-1)) {
                return 1 + (nv-2)*nu;
            }
            return 1+(v-1)*nu+(u%nu); // make u wraparound
        };

        for(index_t v=0; v<nv-1; ++v) {
            for(index_t u=0; u<nu; ++u) {
                index_t v00 = vindex(u  ,v  );
                index_t v10 = vindex(u+1,v  );
                index_t v01 = vindex(u  ,v+1);
                index_t v11 = vindex(u+1,v+1);
                // Create triangles, skip degenerate triangles
                // around the poles.
                if(v01 != v11) {
                    M->facets.create_triangle(v00, v11, v01);
                }
                if(v00 != v10) {
                    M->facets.create_triangle(v00, v10, v11);
                }
            }
        }
            
        M->facets.connect();
        M->update_bbox();
        return M;
    }
    
    CSGMesh_var CSGBuilder::cylinder(
        double h, double r1, double r2, bool center
    ) {
        index_t nu = get_fragments_from_r(std::max(r1,r2));

        double z1 = center ? -h/2.0 : 0.0;
        double z2 = center ?  h/2.0 : h;

        CSGMesh_var M = new CSGMesh;
        M->vertices.set_dimension(3);

        if(r1 < 0.0 || r2 < 0.0) {
            Logger::warn("CSG")
                << "cylinder with negative radius (returning empty shape)"
                << std::endl;
            return M;
        }
        
        if(r1 == 0.0) {
            std::swap(r1,r2);
            std::swap(z1,z2);
        }
            
        for(index_t u=0; u<nu; ++u) {
            double theta = double(u)*2.0*M_PI/double(nu);
            double ctheta = cos(theta);
            double stheta = sin(theta);
            double x = ctheta*r1;
            double y = stheta*r1;
            M->vertices.create_vertex(vec3(x,y,z1).data());
        }

        if(r2 == 0.0) {
            M->vertices.create_vertex(vec3(0.0, 0.0, z2).data());
        } else {
            for(index_t u=0; u<nu; ++u) {
                double theta = double(u)*2.0*M_PI/double(nu);
                double ctheta = cos(theta);
                double stheta = sin(theta);
                double x = ctheta*r2;
                double y = stheta*r2;
                M->vertices.create_vertex(vec3(x,y,z2).data());
            }
        }

        // Capping
        for(index_t u=1; u+1<nu; ++u) {
            M->facets.create_triangle(0,u+1,u);
            if(r2 != 0.0) {
                M->facets.create_triangle(nu,nu+u,nu+u+1);
            }
        }

        // Wall
        for(index_t u=0; u<nu; ++u) {
            if(r2 != 0.0) {
                index_t v00 = u;
                index_t v01 = v00 + nu;
                index_t v10 = (u+1)%nu;
                index_t v11 = v10 + nu;
                M->facets.create_triangle(v00, v11, v01);
                M->facets.create_triangle(v00, v10, v11);
            } else {
                M->facets.create_triangle(u, (u+1)%nu, nu);
            }
        }
            
        M->facets.connect();
        M->update_bbox();
        return M;
    }

    CSGMesh_var CSGBuilder::import(
        const std::string& filename, const std::string& layer, index_t timestamp,
        vec2 origin, vec2 scale
    ) {
        CSGMesh_var result;
        
        std::string full_filename = filename;
        if(!find_file(full_filename)) {
            Logger::err("CSG") << filename << ": file not found"
                               << std::endl;
            return result;
        }

        if(String::to_lowercase(FileSystem::extension(filename)) == "dxf") {
            result = import_with_openSCAD(full_filename, layer, timestamp);
        } else {
            result = new CSGMesh;
            MeshIOFlags io_flags;
            io_flags.set_verbose(verbose_);
            if(!mesh_load(full_filename, *result, io_flags)) {
                result.reset();
                return result;
            }
            std::string ext = FileSystem::extension(filename);
            String::to_lowercase(ext);
            if( ext == "stl") {
                MeshRepairMode mode = MESH_REPAIR_DEFAULT;
                if(!verbose_) {
                    mode = MeshRepairMode(mode | MESH_REPAIR_QUIET);
                }
                mesh_repair(*result, mode, STL_epsilon_);
            }
            result->facets.compute_borders();
        }

        // Apply origin and scale
        // TODO: check, is it origin + coord*scale or (origin + coord)*scale ?
        for(index_t v: result->vertices) {
            double* p = result->vertices.point_ptr(v);
            p[0] = (p[0] - origin.x) * scale.x;
            p[1] = (p[1] - origin.y) * scale.y;
        }

        if(result->vertices.dimension() == 3) {
            bool z_all_zero = true;
            for(index_t v: result->vertices) {
                if(result->vertices.point_ptr(v)[2] != 0.0) {
                    z_all_zero = false;
                    break;
                }
            }
            if(z_all_zero) {
                result->vertices.set_dimension(2);
            }
        }
        
        result->update_bbox();
        return result;
    }


    CSGMesh_var CSGBuilder::import_with_openSCAD(
        const std::string& filename, const std::string& layer, index_t timestamp
    ) {
        CSGMesh_var result;

        std::string path = FileSystem::dir_name(filename);
        std::string base = FileSystem::base_name(filename);
        std::string extension = FileSystem::extension(filename);

        std::string geogram_file
            = path + "/" + "geogram_" + base +
              "_" + extension +
              "_" + layer + "_" +
              String::to_string(timestamp) + ".stl";

        if(FileSystem::is_file(geogram_file)) {
            result = import(geogram_file);
            result->vertices.set_dimension(2);
            return result;
        }

        Logger::out("CSG") << "Did not find " << geogram_file << std::endl;
        Logger::out("CSG") << "Trying to create it with OpenSCAD" << std::endl;
        
        // Generate a simple linear extrusion, so that we can convert to STL
        // (without it OpenSCAD refuses to create a STL with 2D content)
        std::ofstream tmp("tmpscad.scad");
        tmp << "group() {" << std::endl;
        tmp << "   linear_extrude(height=1.0) {" << std::endl;
        tmp << "      import(" << std::endl;
        tmp << "          file = \"" << filename << "\"," << std::endl;
        tmp << "          layer = \"" << layer << "\"," << std::endl;
        tmp << "          timestamp = " << timestamp << std::endl;
        tmp << "      );" << std::endl;
        tmp << "   }" << std::endl;
        tmp << "}" << std::endl;

        // Start OpenSCAD and generate output as STL
        if(system("openscad tmpscad.scad -o tmpscad.stl")) {
            Logger::warn("CSG") << "Error while running openscad " << std::endl;
            Logger::warn("CSG") << "(used to import " << filename << ")"
                                << std::endl;
        }

        // Load STL using our own loader
        result = import("tmpscad.stl");

        FileSystem::delete_file("tmpscad.scad");        
        FileSystem::delete_file("tmpscad.stl");
        
        // Delete the facets that are coming from the linear extrusion
        vector<index_t> delete_f(result->facets.nb(),0);
        for(index_t f: result->facets) {
            for(index_t lv=0; lv<result->facets.nb_vertices(f); ++lv) {
                index_t v = result->facets.vertex(f,lv);
                if(result->vertices.point_ptr(v)[2] != 0.0) {
                    delete_f[f] = 1;
                }
            }
        }
        result->facets.delete_elements(delete_f);
        mesh_save(*result, geogram_file);
        result->vertices.set_dimension(2);
        return result;
    }

    CSGMesh_var CSGBuilder::surface(
        const std::string& filename, bool center, bool invert
    ) {
        CSGMesh_var result;
        std::string full_filename = filename;
        if(!find_file(full_filename)) {
            Logger::err("CSG") << filename << ": file not found"
                               << std::endl;
            return result;
        }

        Image_var image;
        
        if(String::to_lowercase(FileSystem::extension(filename)) == "dat") {
            image = load_dat_image(full_filename);
        } else {
            image = ImageLibrary::instance()->load_image(full_filename);
        }

        if(image.is_null()) {
            Logger::err("CSG") << filename << ": could not load"
                               << std::endl;
            return result;
        }

        if(image->color_encoding() != Image::GRAY ||
           image->component_encoding() != Image::FLOAT64
        ) {
            Logger::err("CSG") << "surface: images not supported yet"
                               << std::endl;
            image.reset();
            return result;
        }

        index_t nu = image->width();
        index_t nv = image->height();

        double z1 = Numeric::max_float64();
        
        result = new CSGMesh;
        result->vertices.set_dimension(3);
        result->vertices.create_vertices(image->width() * image->height());
        for(index_t v=0; v < nv; ++v) {
            for(index_t u=0; u<nu; ++u) {
                double* p = result->vertices.point_ptr(v*nu+u);
                double x = double(u);
                double y = double(v);
                double z = image->pixel_base_float64_ptr(u,v)[0];
                if(invert) {
                    z = 1.0 - z;
                }
                if(center) {
                    x -= double(nu-1)/2.0;
                    y -= double(nv-1)/2.0;
                }
                p[0] = x;
                p[1] = y;
                p[2] = z;
                z1 = std::min(z1,z);
            }
        }

        z1 -= 1.0;

        // Could be a bit smarter here (in the indexing, to generate
        // walls and z1 faces directly), but I was lazy...
        
        for(index_t v=0; v+1< nv; ++v) {
            for(index_t u=0; u+1<nu; ++u) {
                index_t v00 = v*nu+u;
                index_t v10 = v*nu+u+1;
                index_t v01 = (v+1)*nu+u;
                index_t v11 = (v+1)*nu+u+1;
                vec3 p00(result->vertices.point_ptr(v00));
                vec3 p10(result->vertices.point_ptr(v10));
                vec3 p01(result->vertices.point_ptr(v01));
                vec3 p11(result->vertices.point_ptr(v11));
                vec3 p = 0.25*(p00+p10+p01+p11);
                index_t w = result->vertices.create_vertex(p.data());
                result->facets.create_triangle(v00,v10,w);
                result->facets.create_triangle(v10,v11,w);
                result->facets.create_triangle(v11,v01,w);
                result->facets.create_triangle(v01,v00,w);
            }
        }

        result->facets.connect();

        vector<index_t> projected(result->vertices.nb(), index_t(-1));
        for(index_t f: result->facets) {
            for(index_t le=0; le<3; ++le) {
                if(result->facets.adjacent(f,le) == index_t(-1)) {
                    for(index_t dle=0; dle<2; ++dle) {
                        index_t v = result->facets.vertex(f, (le+dle)%3);
                        if(projected[v] == index_t(-1)) {
                            vec3 p(result->vertices.point_ptr(v));
                            p.z = z1;
                            projected[v] = result->vertices.create_vertex(p.data());
                        }
                    }
                }
            }
        }
        
        for(index_t f: result->facets) {
            for(index_t le=0; le<3; ++le) {
                if(result->facets.adjacent(f,le) == index_t(-1)) {
                    index_t v1 = result->facets.vertex(f,le);
                    index_t v2 = result->facets.vertex(f,(le+1)%3);
                    result->facets.create_triangle(v2,v1,projected[v2]);
                    result->facets.create_triangle(projected[v2],v1,projected[v1]);
                }
            }
        }

        result->facets.connect();
        fill_holes(*result,1e6);
        tessellate_facets(*result,3);
        result->update_bbox();
        return result;
    }

    Image* CSGBuilder::load_dat_image(const std::string& file_name) {
        LineInput in(file_name);

        index_t nrows  = index_t(-1);
        index_t ncols  = index_t(-1);
        Image* result = nullptr;
        
        try {
            while( !in.eof() && in.get_line() && in.current_line()[0] == '#') {
                in.get_fields();
                if(in.field_matches(1,"type:") && !in.field_matches(2,"matrix")) {
                    Logger::err("CSG") << "dat file: wrong type: "
                                       << in.field(2)
                                       << " (only \'matrix\' is supported)"
                                       << std::endl;
                    return nullptr;
                } else if(in.field_matches(1,"rows:")) {
                    nrows = in.field_as_uint(2);
                } else if(in.field_matches(1,"columns:")) {
                    ncols = in.field_as_uint(2);
                }
            }

            result = new Image(
                Image::GRAY, Image::FLOAT64, ncols, nrows
            );
            
            for(index_t y=0; y<nrows; ++y) {
                in.get_fields();
                for(index_t x=0; x<ncols; ++x) {
                    result->pixel_base_float64_ptr(x,y)[0] = in.field_as_double(x);
                }
                in.get_line();
            }
        } catch(const std::logic_error& ex) {
            Logger::err("CSG") << "invalid dat file:" << ex.what() << std::endl;
            delete result;
            return nullptr;
        }
        
        return result;
    }
        


    bool CSGBuilder::find_file(std::string& filename) {
        if(FileSystem::is_file(filename)) {
            return true;
        }
        
        for(std::string& path: file_path_) {
            std::string full_filename = path + "/" + filename;
            if(FileSystem::is_file(full_filename)) {
                filename = full_filename;
                return true;
            }
        }
        return false;
    }
    
    /****** Instructions ****/
    
    CSGMesh_var CSGBuilder::multmatrix(const mat4& M, const CSGScope& scope) {
        CSGMesh_var result = group(scope);
        index_t dim = result->vertices.dimension();
        for(index_t v: result->vertices) {
            double* p = result->vertices.point_ptr(v);
            double x = p[0];
            double y = p[1];
            double z = (dim == 3) ? p[2] : 0.0;
            vec3 P(x,y,z);
            P = transform_point(M,P);
            for(index_t c=0; c<dim; ++c) {
                p[c] = P[c];
            }
        }
        if(det(M) < 0.0) {
            for(index_t f: result->facets) {
                result->facets.flip(f);
            }
        }
        result->update_bbox();
        return result;
    }

    CSGMesh_var CSGBuilder::union_instr(const CSGScope& scope) {
        if(scope.size() == 1) {
            return scope[0];
        }

        // Boolean operations can handle no more than 32 operands.
        // For a union with more than 32 operands, split it into two.
        if(!fast_union_ && scope.size() > max_arity_) {
            CSGScope scope1;
            CSGScope scope2;
            index_t n1 = index_t(scope.size()/2);
            for(index_t i=0; i<scope.size(); ++i) {
                if(i < n1) {
                    scope1.push_back(scope[i]);
                } else {
                    scope2.push_back(scope[i]);
                }
            }
            CSGScope scope3;
            scope3.push_back(union_instr(scope1));
            scope3.push_back(union_instr(scope2));
            return union_instr(scope3);
        }
        
        bool may_have_intersections = false;
        for(index_t i=0; i<scope.size(); ++i) {
            for(index_t j=i+1; j<scope.size(); ++j) {
                if(scope[i]->may_have_intersections_with(scope[j])) {
                    may_have_intersections = true;
                    break;
                }
            }
        }

        CSGMesh_var result = append(scope);
        
        if(may_have_intersections) {
            do_CSG(result, "union");
        }

        post_process(result);
        result->update_bbox();
        return result;
    }

    
    CSGMesh_var CSGBuilder::intersection(const CSGScope& scope) {
        if(scope.size() == 1) {
            return scope[0];
        }

        // Boolean operations can handle no more than 32 operands.
        // For a intersection with more than 32 operands, split it into two.
        if(scope.size() > max_arity_) {
            CSGScope scope1;
            CSGScope scope2;
            index_t n1 = index_t(scope.size()/2);
            for(index_t i=0; i<scope.size(); ++i) {
                if(i < n1) {
                    scope1.push_back(scope[i]);
                } else {
                    scope2.push_back(scope[i]);
                }
            }

            CSGScope scope3;
            scope3.push_back(union_instr(scope1));
            scope3.push_back(union_instr(scope2));
            return intersection(scope3);
        }

        CSGMesh_var result = append(scope);
        do_CSG(result, "intersection");
        post_process(result);
        result->update_bbox();
        return result;
    }

    CSGMesh_var CSGBuilder::difference(const CSGScope& scope) {
        if(scope.size() == 1) {
            return scope[0];
        }

        // Boolean operations can handle no more than 32 operands.
        // For a difference with more than 32 operands, split it
        // (by calling union_instr() that in turn splits the list if need be).
        if(scope.size() > max_arity_) {
            CSGScope scope2;
            for(index_t i=1; i<scope.size(); ++i) {
                scope2.push_back(scope[i]);
            }
            CSGScope scope3;
            scope3.push_back(scope[0]);
            scope3.push_back(union_instr(scope2));
            return difference(scope3);
        }

        CSGMesh_var result = append(scope);
        // construct the expression x0-x1-x2...-xn
        std::string expr = "x0";
        for(index_t i=1; i<scope.size(); ++i) {
            expr += "-x" + String::to_string(i);
        }

        do_CSG(result, expr);
        post_process(result);
        result->update_bbox();
        return result;
    }

    CSGMesh_var CSGBuilder::append(const CSGScope& scope) {
        if(scope.size() == 1) {
            return scope[0];
        }
        
        CSGMesh_var result = new CSGMesh;
        result->vertices.set_dimension(3);

        if(!fast_union_ && scope.size() > max_arity_) { 
            Logger::warn("CSG") << "Scope with more than "
                                << max_arity_
                                << " children"
                                << std::endl;
        }
        
        for(index_t i=0; i<scope.size(); ++i) {
            result->append_mesh(scope[i], i);
        }
        
        return result;
    }

    CSGMesh_var CSGBuilder::color(vec4 color, const CSGScope& scope) {
        geo_argused(color); // TODO
        return group(scope);
    }

    CSGMesh_var CSGBuilder::hull(const CSGScope& scope) {
        vector<double> points;
        index_t dim = 0;
        index_t nb_pts = 0;
        for(const CSGMesh_var& current : scope) {
            index_t cur_dim = current->vertices.dimension();
            dim = std::max(cur_dim,dim);
            nb_pts += current->vertices.nb();
        }
        points.reserve(nb_pts * dim);
        for(const CSGMesh_var& current : scope) {
            for(index_t v : current->vertices) {
                const double* p = current->vertices.point_ptr(v);
                for(index_t c=0; c<dim; ++c) {
                    points.push_back(
                        c < current->vertices.dimension() ? p[c] : 0.0
                    );
                }
            }
        }

        if(dim == 3) {
            CmdLine::set_arg("algo:delaunay", "PDEL");
        } else {
            CmdLine::set_arg("algo:delaunay", "BDEL2d");
        }
        Delaunay_var delaunay = Delaunay::create(coord_index_t(dim));
        delaunay->set_keeps_infinite(true);
        delaunay->set_vertices(nb_pts, points.data());

        CSGMesh_var result = new CSGMesh;
        result->vertices.set_dimension(dim);
            
        if(dim == 3) {
            vector<index_t> tri2v;
            // This iterates on the infinite cells
            for(
                index_t t = delaunay->nb_finite_cells();
                t < delaunay->nb_cells(); ++t
            ) {
                for(index_t lv=0; lv<4; ++lv) {
                    signed_index_t v = delaunay->cell_vertex(t,lv);
                    if(v != -1) {
                        tri2v.push_back(index_t(v));
                    }
                }
            }
            result->facets.assign_triangle_mesh(3, points, tri2v, true);
            result->vertices.remove_isolated();
        } else {
            result->vertices.set_dimension(2);
            result->vertices.create_vertices(nb_pts);
            Memory::copy(
                result->vertices.point_ptr(0), points.data(),
                sizeof(double)*2*nb_pts
            );
            for(index_t t = delaunay->nb_finite_cells();
                t < delaunay->nb_cells(); ++t
            ) {
                signed_index_t v1=-1,v2=-1;
                for(index_t lv=0; lv<3; ++lv) {
                    if(delaunay->cell_vertex(t,lv) == -1) {
                        v1 = delaunay->cell_vertex(t,(lv+1)%3);
                        v2 = delaunay->cell_vertex(t,(lv+2)%3);
                    }
                }
                geo_assert(v1 != -1 && v2 != -1);
                result->edges.create_edge(index_t(v1),index_t(v2));
            }
            result->vertices.remove_isolated();
        }
        result->update_bbox();            
        return result;
    }

    CSGMesh_var CSGBuilder::linear_extrude(
        const CSGScope& scope, double height, bool center, vec2 scale,
        index_t slices, double twist
    ) {
        double z1 = center ? -height/2.0 : 0.0;
        double z2 = center ?  height/2.0 : height;

        CSGMesh_var M = scope.size() == 1 ? scope[0] : group(scope);
        if(M->vertices.dimension() != 2) {
            throw(std::logic_error(
                      "linear_extrude: mesh is not of dimension 2"
            ));
        }
        if(M->facets.nb() == 0) {
            triangulate(M,"union");
        }
        M->vertices.set_dimension(3);

        index_t nv  = M->vertices.nb();
        index_t nf  = M->facets.nb();
        index_t nv_intern = 0;
        index_t nv_border = 0;
        
        // Reorder vertices so that border vertices come first, then internal
        // vertices
        {
            vector<index_t> reorder_vertices(M->vertices.nb(), index_t(-1));
            for(index_t f: M->facets) {
                for(index_t le=0; le<3; ++le) {
                    if(M->facets.adjacent(f,le) == index_t(-1)) {
                        index_t v = M->facets.vertex(f,le);
                        // There may be non-manifold borders incident to
                        // the same border vertex several times.
                        if(reorder_vertices[v] != index_t(-1)) {
                            continue;
                        }
                        reorder_vertices[v] = nv_border;
                        ++nv_border;
                    }
                }
            }
            for(index_t v: M->vertices) {
                if(reorder_vertices[v] == index_t(-1)) {
                    reorder_vertices[v] = nv_intern + nv_border;
                    ++nv_intern;
                }
            }
        }

        geo_assert(nv_border + nv_intern == nv);

        // Set z coordinates of all original vertices to z1
        for(index_t v: M->vertices) {
            M->vertices.point_ptr(v)[2] = z1;
        }
        
        if(slices == 0) {
            slices = index_t(fn_);
        }
        if(slices == 0 && twist != 0.0) {
            double R = 0;
            for(index_t v: M->vertices) {
                const double* p = M->vertices.point_ptr(v);
                R = std::max(R, geo_sqr(p[0])+geo_sqr(p[1]));
                R = std::max(R, geo_sqr(p[0]*scale.x)+geo_sqr(p[1]*scale.y));
            }
            R = ::sqrt(R);
            slices = get_fragments_from_r(R,twist);
        }
        slices = std::max(slices, index_t(1));

        index_t first_border_offset = 0;
        index_t border_offset = first_border_offset;

        auto extrude_vertex = [&](index_t to_v, index_t from_v, double t) {
            const double* ref = M->vertices.point_ptr(from_v);
            double* target = M->vertices.point_ptr(to_v);
            double s = 1.0 - t;
            double z = s*z1 + t*z2;
            vec2   sz = s*vec2(1.0, 1.0) + t*scale;

            double x = ref[0] * sz.x;
            double y = ref[1] * sz.y;

            if(twist != 0.0) {
                double alpha = twist*t*M_PI/180.0;
                double ca = cos(alpha);
                double sa = sin(alpha);
                double x2 =  ca*x+sa*y;
                double y2 = -sa*x+ca*y;
                x = x2;
                y = y2;
            }
            
            target[0] = x;
            target[1] = y;
            target[2] = z;
        };
        
        for(index_t Z=1; Z<=slices; ++Z) {
            double t = double(Z)/double(slices);

            // Special case: scaling = 0 (create a pole)
            if(Z == slices && scale.x == 0.0 && scale.y == 0.0) {
                double p[3] = {0.0, 0.0, z2};
                index_t pole = M->vertices.create_vertex(p);
                for(index_t f=0; f<nf; ++f) {
                    for(index_t le=0; le<3; ++le) {
                        if(M->facets.adjacent(f,le) == index_t(-1)) {
                            index_t v1 = M->facets.vertex(f,le);
                            index_t v2 = M->facets.vertex(f,(le+1)%3);
                            v1 += border_offset;
                            v2 += border_offset;
                            M->facets.create_triangle(v2,v1,pole);
                        }
                    }
                }
                break;
            }
            
            // Create vertices
            index_t next_border_offset = M->vertices.create_vertices(nv_border);

            // Extrude all vertices on border
            for(index_t dv=0; dv<nv_border; ++dv) {
                extrude_vertex(
                    next_border_offset + dv,
                    first_border_offset + dv,
                    t
                );
            }

            // Create walls
            for(index_t f=0; f<nf; ++f) {
                for(index_t le=0; le<3; ++le) {
                    if(M->facets.adjacent(f,le) == index_t(-1)) {
                        index_t v1 = M->facets.vertex(f,le);
                        index_t v2 = M->facets.vertex(f,(le+1)%3);
                        index_t w1 = v1 + next_border_offset;
                        index_t w2 = v2 + next_border_offset;
                        v1 += border_offset;
                        v2 += border_offset;
                        M->facets.create_triangle(v2,v1,w2);
                        M->facets.create_triangle(w2,v1,w1);
                    }
                }
            }
            
            border_offset = next_border_offset;
        }

        // Capping
        if(scale.x != 0.0 || scale.y != 0.0) {
            index_t vint_offset = M->vertices.create_vertices(nv_intern);
            
            // Create vertices for capping
            for(index_t dv=0; dv<nv_intern; ++dv) {
                extrude_vertex(vint_offset + dv, nv_border + dv, 1.0);
            }
            
            for(index_t f=0; f<nf; ++f) {
                index_t v[3];
                for(index_t lv=0; lv<3; ++lv) {
                    v[lv] = M->facets.vertex(f,lv);
                    v[lv] = (v[lv] < nv_border) ?
                               (border_offset+v[lv]) :
                               (v[lv] - nv_border + vint_offset);
                }
                M->facets.create_triangle(v[2],v[1],v[0]);
            }
        }
            
        M->facets.connect();
        M->edges.clear();

        M->update_bbox();    
        return M;
    }

    
    CSGMesh_var CSGBuilder::rotate_extrude(const CSGScope& scope, double angle) {
        CSGMesh_var M = scope.size() == 1 ? scope[0] : group(scope);
        if(M->vertices.dimension() != 2) {
            throw(std::logic_error(
                      "linear_extrude: mesh is not of dimension 2"
            ));
        }
        
        if(angle == 360.0) {
            M->facets.clear();
            M->vertices.remove_isolated();
        } else if(M->facets.nb() == 0) {
            triangulate(M,"union");
        }

        {
            vector<index_t> remove_edge(M->edges.nb(),0);
            for(index_t e: M->edges) {
                index_t v1 = M->edges.vertex(e,0);
                index_t v2 = M->edges.vertex(e,1);
                if(
                    M->vertices.point_ptr(v1)[0] == 0.0 &&
                    M->vertices.point_ptr(v2)[0] == 0.0
                ) {
                    remove_edge[e] = 1;
                }
            }
            M->edges.delete_elements(remove_edge);
        }
        
        M->vertices.set_dimension(3);

        index_t nv  = M->vertices.nb();
        index_t nf  = M->facets.nb();
        index_t nv_intern = 0;
        index_t nv_border = 0;
        
        // Reorder vertices so that border vertices come first, then internal
        // vertices
        if(M->facets.nb() != 0) {
            vector<index_t> reorder_vertices(M->vertices.nb(), index_t(-1));
            for(index_t f: M->facets) {
                for(index_t le=0; le<3; ++le) {
                    if(M->facets.adjacent(f,le) == index_t(-1)) {
                        index_t v = M->facets.vertex(f,le);
                        // There may be non-manifold borders incident to
                        // the same border vertex several times.
                        if(reorder_vertices[v] != index_t(-1)) {
                            continue;
                        }
                        reorder_vertices[v] = nv_border;
                        ++nv_border;
                    }
                }
            }
            for(index_t v: M->vertices) {
                if(reorder_vertices[v] == index_t(-1)) {
                    reorder_vertices[v] = nv_intern + nv_border;
                    ++nv_intern;
                }
            }
        } else {
            nv_border = nv;
            nv_intern = 0;
        }

        geo_assert(nv_border + nv_intern == nv);

        auto extrude_vertex = [&](index_t to_v, index_t from_v, double t) {
            const double* ref = M->vertices.point_ptr(from_v);
            double* target = M->vertices.point_ptr(to_v);
            double alpha = t * 2.0 * M_PI * angle / 360.0;
            double x = ref[0] * cos(alpha);
            double y = ref[0] * sin(alpha);
            double z = ref[1];
            target[0] = x;
            target[1] = y;
            target[2] = z;
        };


        double R = 0.0;
        for(index_t v: M->vertices) {
            R = std::max(R, M->vertices.point_ptr(v)[0]);
        }

        index_t slices = get_fragments_from_r(R,angle);
        
        index_t first_border_offset = 0;
        index_t border_offset = first_border_offset;

        for(index_t Z=1; Z<=slices; ++Z) {
            double t = double(Z)/double(slices);

            index_t next_border_offset = 0;

            if(Z != slices || angle != 360.0) {
                // Create vertices
                next_border_offset = M->vertices.create_vertices(nv_border);
                
                // Extrude all vertices on border
                for(index_t dv=0; dv<nv_border; ++dv) {
                    extrude_vertex(
                        next_border_offset + dv,
                        first_border_offset + dv,
                        t
                    );
                }
            }

            // Create walls
            for(index_t e: M->edges) {
                index_t v1 = M->edges.vertex(e,0);
                index_t v2 = M->edges.vertex(e,1);
                index_t w1 = v1 + next_border_offset;
                index_t w2 = v2 + next_border_offset;
                v1 += border_offset;
                v2 += border_offset;
                M->facets.create_triangle(v2,v1,w2);
                M->facets.create_triangle(w2,v1,w1);
            }
                
            border_offset = next_border_offset;
        }

        // Capping
        if(angle != 360.0) {
            index_t vint_offset = M->vertices.create_vertices(nv_intern);
            
            // Create vertices for capping
            for(index_t dv=0; dv<nv_intern; ++dv) {
                extrude_vertex(vint_offset + dv, nv_border + dv, 1.0);
            }
            
            for(index_t f=0; f<nf; ++f) {
                index_t v[3];
                for(index_t lv=0; lv<3; ++lv) {
                    v[lv] = M->facets.vertex(f,lv);
                    v[lv] = (v[lv] < nv_border) ?
                               (border_offset+v[lv]) :
                               (v[lv] - nv_border + vint_offset);
                }
                M->facets.create_triangle(v[2],v[1],v[0]);
            }
        }

        // position first slide (I do that in the end, because we needed it u
        // unmodified to use it as a reference before).
        for(index_t v=0; v<nv_border; ++v) {
            extrude_vertex(v, v, 0.0);
        }

        // Merge vertices at the poles if there were any (generated by initial
        // vertices with zero x coordinate).
        // TODO: do it more cleanly
        MeshRepairMode mode = MESH_REPAIR_DEFAULT;
        if(!verbose_) {
            mode = MeshRepairMode(mode | MESH_REPAIR_QUIET);
        }
        mesh_repair(*M,mode);
        
        M->facets.connect();
        M->edges.clear();

        M->update_bbox();    
        return M;
    }

    CSGMesh_var CSGBuilder::projection(const CSGScope& scope, bool cut) {
        CSGMesh_var result = append(scope);
        if(result->vertices.dimension() != 3) {
            Logger::err("CSG") << "projection(): input mesh is not of dimenion 3"
                               << std::endl;
            result.reset();
            return result;
        }
        if(cut) {
            // An unelegant way of doing it: compute intersection with (enlarged)
            // half bbox and keep z=0 facets only.

            double x1 = result->bbox().xyz_min[0];
            double y1 = result->bbox().xyz_min[1];
            double z1 = result->bbox().xyz_min[2];
            double x2 = result->bbox().xyz_max[0];
            double y2 = result->bbox().xyz_max[1];
            double z2 = result->bbox().xyz_max[2];
            double dx = x2-x1;
            double dy = y2-y1;
            double dz = z2 - z1;
            CSGMesh_var C = cube(2.0*vec3(3*dx,3*dy,3*dz), false);
            for(index_t v: C->vertices) {
                C->vertices.point_ptr(v)[0] += (x1 - dx); 
                C->vertices.point_ptr(v)[1] += (y1 - dy); 
            }
            CSGScope scope2;
            scope2.push_back(result);
            scope2.push_back(C);
            result = difference(scope2);
            vector<index_t> remove_f(result->facets.nb(),0);
            for(index_t f: result->facets) {
                for(index_t lv=0; lv<result->facets.nb_vertices(f); ++lv) {
                    index_t v = result->facets.vertex(f,lv);
                    const double* p = result->vertices.point_ptr(v);
                    if(p[2] != 0.0) {
                        remove_f[f] = 1;
                    }
                }
            }
            result->facets.delete_elements(remove_f);
            result->facets.compute_borders();
            result->vertices.set_dimension(2);
            result->update_bbox();
        } else {
            // Super unelegant brute force algorithm !!
            // But just extracting the silhouette does not work because
            // we need a smarter in/out classification algorithm, that
            // seemingly cannot be expressed as a boolean expression
            // passed to triangulate()
            {
                CSGScope scope2;
                for(index_t f: result->facets) {
                    const double* p1 = result->vertices.point_ptr(
                        result->facets.vertex(f,0)
                    );
                    
                    const double* p2 = result->vertices.point_ptr(
                        result->facets.vertex(f,1)
                    );
                    
                    const double* p3 = result->vertices.point_ptr(
                        result->facets.vertex(f,2)
                    );
                    
                    // I thought that I could have said here: != POSITIVE
                    // NOTE: different set of isolated vertices each time,
                    // there is something not normal.
                    if(PCK::orient_2d(p1,p2,p3) == ZERO) {
                        continue;
                    }
                    
                    CSGMesh_var F = new CSGMesh;
                    F->vertices.set_dimension(2);
                    
                    F->vertices.create_vertex(p1);
                    F->vertices.create_vertex(p2);
                    F->vertices.create_vertex(p3);
                    F->facets.create_triangle(0,1,2);
                    F->facets.compute_borders();
                    F->update_bbox();
                    scope2.push_back(F);
                }
                result = union_instr(scope2);
            }
        }
        return result;
    }
    
    /******************************/

    void CSGBuilder::do_CSG(CSGMesh_var mesh, const std::string& boolean_expr) {
        if(mesh->vertices.dimension() == 2) {
            triangulate(mesh, boolean_expr, true); // keep borders only
        } else {
            MeshSurfaceIntersection I(*mesh);
            I.set_verbose(verbose_);
            I.set_delaunay(delaunay_); 
            I.set_detect_intersecting_neighbors(detect_intersecting_neighbors_);
            if(fast_union_ && boolean_expr == "union") {
                I.set_radial_sort(true); // TODO: Needed ? 
            }
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
        CSGMesh_var mesh, const std::string& boolean_expr,
        bool keep_borders_only
    ) {
        mesh->facets.clear();
        mesh->vertices.remove_isolated();

        bool has_operand_bit = Attribute<index_t>::is_defined(
            mesh->edges.attributes(), "operand_bit"
        );
        Attribute<index_t> e_operand_bit(
            mesh->edges.attributes(), "operand_bit"
        );
        if(!has_operand_bit) {
            for(index_t e: mesh->edges) {
                e_operand_bit[e] = index_t(1);
            }
        }
        
        ExactCDT2d CDT;
        double umin = mesh->bbox().xyz_min[0];
        double vmin = mesh->bbox().xyz_min[1];
        double umax = mesh->bbox().xyz_max[0];
        double vmax = mesh->bbox().xyz_max[1];
        double d = std::max(umax-umin, vmax-vmin);
        d *= 10.0;
        d = std::max(d, 1.0);
        umin-=d;
        vmin-=d;
        umax+=d;
        vmax+=d;
        CDT.create_enclosing_rectangle(umin, vmin, umax, vmax);

        // In case there are duplicated vertices, keep track of indexing
        vector<index_t> vertex_id(mesh->vertices.nb());
        for(index_t v: mesh->vertices) {
            vec2 p(mesh->vertices.point_ptr(v));
            vertex_id[v] = CDT.insert(ExactCDT2d::ExactPoint(p),v);
        }

        // Memorize current number of vertices to detect vertices
        // coming from constraint intersections
        index_t nv0 = CDT.nv();

        // Insert constraint
        {
            for(index_t e: mesh->edges) {
                index_t v1 = mesh->edges.vertex(e,0);
                index_t v2 = mesh->edges.vertex(e,1);
                CDT.insert_constraint(
                    vertex_id[v1], vertex_id[v2], e_operand_bit[e]
                );
            }
        }

        CDT.classify_triangles(boolean_expr);

        // Create vertices coming from constraint intersections
        for(index_t v=nv0; v<CDT.nv(); ++v) {
            vec2 p = PCK::approximate(CDT.point(v));
            CDT.set_vertex_id(
                v,
                mesh->vertices.create_vertex(p.data())
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
        if(keep_borders_only) {
            mesh->facets.clear();
        }
        mesh->vertices.remove_isolated();

        for(index_t e: mesh->edges) {
            e_operand_bit[e] = 1;
        }
    }
    
    void CSGBuilder::post_process(CSGMesh_var mesh) {
        // TODO: correct snaprounding instead here

        geo_argused(mesh);

        /*
        double* p = mesh->vertices.point_ptr(0);
        for(
            index_t i=0;
            i<mesh->vertices.nb() * mesh->vertices.dimension(); ++i
        ) {
            p[i] = double(float(p[i])); 
        }
        */

        mesh_colocate_vertices_no_check(*mesh);
        mesh_remove_bad_facets_no_check(*mesh,false); // Do not check duplicates
        mesh_connect_and_reorient_facets_no_check(*mesh);
    }
    
    index_t CSGBuilder::get_fragments_from_r(double r, double twist) {
        if (fn_ > 0.0) {
            return index_t(fn_ >= 3 ? fn_ : 3);
        }
        return index_t(ceil(fmax(fmin(twist / fa_, r*2*M_PI / fs_), 5)));
    }
    
    /************************************************************************/
    
    CSGCompiler::CSGCompiler() : lex_(nullptr), progress_(nullptr), lines_(0) {
        
#define DECLARE_OBJECT(obj) object_funcs_[#obj] = &CSGCompiler::obj;
        DECLARE_OBJECT(square);
        DECLARE_OBJECT(circle);
        DECLARE_OBJECT(cube);
        DECLARE_OBJECT(sphere);
        DECLARE_OBJECT(cylinder);
        DECLARE_OBJECT(polyhedron);
        DECLARE_OBJECT(polygon);
        DECLARE_OBJECT(import);
        DECLARE_OBJECT(surface);
        
#define DECLARE_INSTRUCTION(instr) \
        instruction_funcs_[#instr] = &CSGCompiler::instr;
        DECLARE_INSTRUCTION(multmatrix);
        DECLARE_INSTRUCTION(resize);
        DECLARE_INSTRUCTION(intersection);
        DECLARE_INSTRUCTION(difference);
        DECLARE_INSTRUCTION(group);
        DECLARE_INSTRUCTION(color);
        DECLARE_INSTRUCTION(hull);
        DECLARE_INSTRUCTION(linear_extrude);
        DECLARE_INSTRUCTION(rotate_extrude);
        DECLARE_INSTRUCTION(projection);
        instruction_funcs_["union"]  = &CSGCompiler::union_instr;
        instruction_funcs_["render"] = &CSGCompiler::group;
    }
    
    CSGMesh_var CSGCompiler::compile_file(const std::string& input_filename) {
        if(
            FileSystem::extension(input_filename) == "scad" ||
            FileSystem::extension(input_filename) == "SCAD"
        ) {
            CSGMesh_var result;

            Logger::out("CSG") << "Converting scad file using openscad"
                               << std::endl;
            
            // Ask openscad for help for parsing .scad files !
            std::string command = "openscad "+input_filename+" -o tmpscad.csg";
            
            if(system(command.c_str())) {
                Logger::err("CSG") << "Error while running openscad "
                                   << std::endl;
                Logger::err("CSG") << "(used to parse " << input_filename << ")"
                                   << std::endl;
                return result;
            }

            result = compile_file("tmpscad.csg");
            FileSystem::delete_file("tmpscad.csg");                    
            return result;
        }

        filename_ = input_filename;
        if(
            FileSystem::extension(filename_) != "csg" &&
            FileSystem::extension(filename_) != "CSG"
        ) {
            throw std::logic_error(
                filename_ + ": wrong extension (should be .csg or .CSG)"
            );
        }
        
        FileSystem::Node* root;
        FileSystem::get_root(root);
        std::string source = root->load_file_as_string(filename_);
        if(source.length() == 0) {
            throw std::logic_error(
                filename_ + ": could not open file"
            );
        }

        // Add the directory that contains the file to the builder's file path,
        // so that import() instructions are able to find files in the same
        // directory.
        builder_.add_file_path(FileSystem::dir_name(input_filename));
        CSGMesh_var result = compile_string(source);
        builder_.reset_file_path();

        if(!result.is_null() && result->vertices.dimension() == 2) {
            result->vertices.set_dimension(3);
        }
        
        return result;
    }
        
    CSGMesh_var CSGCompiler::compile_string(const std::string& source) {
        CSGMesh_var result;
        
        static constexpr size_t BUFFER_SIZE = 0x10000;
        char* buffer = new char[BUFFER_SIZE];
        stb_lexer lex;
        lex_ = &lex;
        try {
            stb_c_lexer_init(
                &lex,
                source.c_str(),
                source.c_str()+source.length(),
                buffer, BUFFER_SIZE
            );
            lines_ = index_t(lines());
            ProgressTask progress("CSG", lines_, builder_.verbose());
            progress_ = &progress;
            CSGScope scope;
            while(lookahead_token().type != CLEX_eof) {
                CSGMesh_var current = parse_instruction_or_object();
                // can be null if commented-out with modifier
                if(!current.is_null()) { 
                    scope.push_back(current);
                }
            }
            ArgList args;
            result = group(args, scope);
        } catch(CSGMesh_var reroot) {
            Logger::out("CSG") << "Re-rooted (!) from line " << line()
                               << std::endl;
            result = reroot;
        } catch(const std::logic_error& e) {
            Logger::err("CSG") << "Error while parsing file:"
                               << e.what()
                               << std::endl;
        }
        delete[] buffer;
        lex_ = nullptr;
        progress_ = nullptr;
        lines_ = 0;
        return result;
    }


    /********* Objects *******************************************************/

    CSGMesh_var CSGCompiler::square(const ArgList& args) {
        vec2 size = args.get_arg("size", vec2(1.0, 1.0));
        bool center = args.get_arg("center", true);
        return builder_.square(size,center);
    }

    CSGMesh_var CSGCompiler::circle(const ArgList& args) {
        double r;
        if(
            args.has_arg("r") &&
            args.get_arg("r").type == Value::NUMBER
        ) {
            r = args.get_arg("r").number_val;
        } else if(
            args.has_arg("d") &&
            args.get_arg("d").type == Value::NUMBER
        ) {
            r = args.get_arg("d").number_val / 2.0;
        } else if(
            args.size() >= 1 &&
            args.ith_arg_name(0) == "arg_0" &&
            args.ith_arg_val(0).type == Value::NUMBER 
        ) {
            r = args.ith_arg_val(0).number_val;
        } else {
            r = 1.0;
        }
        return builder_.circle(r);
    }
        
    CSGMesh_var CSGCompiler::cube(const ArgList& args) {
        vec3 size = args.get_arg("size", vec3(1.0, 1.0, 1.0));
        bool center = args.get_arg("center", true);
        return builder_.cube(size,center);
    }

    CSGMesh_var CSGCompiler::sphere(const ArgList& args) {
        double r = args.get_arg("r", 1.0);
        return builder_.sphere(r);
    }

    CSGMesh_var CSGCompiler::cylinder(const ArgList& args) {
        double h    = args.get_arg("h", 1.0);
        double r1   = args.get_arg("r1", 1.0);
        double r2   = args.get_arg("r2", 1.0);
        bool center = args.get_arg("center", true);
        return builder_.cylinder(h,r1,r2,center);
    }

    CSGMesh_var CSGCompiler::polyhedron(const ArgList& args) {
        CSGMesh_var M = new CSGMesh;
        if(!args.has_arg("points") || !args.has_arg("faces")) {
            syntax_error("polyhedron: missing points or facets");
        }
        const Value& points = args.get_arg("points");
        const Value& faces = args.get_arg("faces");

        if(points.type != Value::ARRAY2D || faces.type != Value::ARRAY2D) {
            syntax_error("polyhedron: wrong type (expected array)");
        }

        M->vertices.set_dimension(3);
        M->vertices.create_vertices(points.array_val.size());
        for(index_t v=0; v<points.array_val.size(); ++v) {
            if(points.array_val[v].size() != 3) {
                syntax_error("polyhedron: wrong vertex size (expected 3d)");
            }
            M->vertices.point_ptr(v)[0] = points.array_val[v][0];
            M->vertices.point_ptr(v)[1] = points.array_val[v][1];
            M->vertices.point_ptr(v)[2] = points.array_val[v][2];
        }

        for(index_t f=0; f<faces.array_val.size(); ++f) {
            index_t new_f = M->facets.create_polygon(
                faces.array_val[f].size()
            );
            for(index_t lv=0; lv < faces.array_val[f].size(); ++lv) {
                double v = faces.array_val[f][lv];
                if(v < 0.0 || v >= double(M->vertices.nb())) {
                    syntax_error(
                        String::format(
                            "polyhedron: invalid vertex index %d (max is %d)",
                            int(v), int(M->vertices.nb())-1
                        ).c_str()
                    );
                }
                M->facets.set_vertex(new_f, lv, index_t(v));
            }
        }

        tessellate_facets(*M,3);
            
        M->facets.connect();
        M->update_bbox();
        return M;
    }


    CSGMesh_var CSGCompiler::polygon(const ArgList& args) {
        CSGMesh_var M = new CSGMesh;
        if(!args.has_arg("points") || !args.has_arg("paths")) {
            syntax_error("polyhedron: missing points or paths");
        }

        const Value& points = args.get_arg("points");

        if(points.type != Value::ARRAY2D) {
            syntax_error("polyhedron: wrong points type (expected array)");
        }

        M->vertices.set_dimension(2);
        M->vertices.create_vertices(points.array_val.size());
        for(index_t v=0; v<points.array_val.size(); ++v) {
            if(points.array_val[v].size() != 2) {
                syntax_error("polyhedron: wrong vertex size (expected 2d)");
            }
            M->vertices.point_ptr(v)[0] = points.array_val[v][0];
            M->vertices.point_ptr(v)[1] = points.array_val[v][1];
        }
        
        const Value& paths = args.get_arg("paths");

        if(paths.type == Value::ARRAY2D ) {
            for(const auto& P : paths.array_val) {
                for(double v: P) {
                    if(v < 0.0 || v >= double(M->vertices.nb())) {
                        syntax_error(
                            String::format(
                                "polygon: invalid vertex index %d (max is %d)",
                                int(v), int(M->vertices.nb())-1
                            ).c_str()
                        );
                    }
                }
                for(index_t lv1=0; lv1 < P.size(); ++lv1) {
                    index_t lv2 = (lv1+1)%P.size();
                    index_t v1 = index_t(P[lv1]);
                    index_t v2 = index_t(P[lv2]);
                    // some files do [0,1,2], some others [0,1,2,0], so we need
                    // to test here for null edges.
                    if(v1 != v2) {
                        M->edges.create_edge(v1,v2);
                    }
                }
            }
        } else if(paths.type == Value::NONE) {
            for(index_t v1=0; v1 < points.array_val.size(); ++v1) {
                index_t v2 = (v1+1)%points.array_val.size();
                M->edges.create_edge(v1,v2);
            }
        } else {
            syntax_error(
                "polyhedron: wrong path type (expected array or undef)"
            );
        }
        
        M->update_bbox();
        return M;
    }
    
    CSGMesh_var CSGCompiler::import(const ArgList& args) {
        std::string filename  = args.get_arg("file", std::string(""));
        std::string layer     = args.get_arg("layer", std::string(""));
        index_t     timestamp = index_t(args.get_arg("timestamp", 0));
        vec2        origin    = args.get_arg("origin", vec2(0.0, 0.0));
        vec2        scale     = args.get_arg("scale", vec2(1.0, 1.0));
        CSGMesh_var M = builder_.import(filename,layer,timestamp,origin,scale);
        if(M.is_null()) {
            syntax_error((filename + ": could not load").c_str());
        }
        return M;
    }

    CSGMesh_var CSGCompiler::surface(const ArgList& args) {
        std::string filename  = args.get_arg("file", std::string(""));
        bool center = args.get_arg("center", false);
        bool invert = args.get_arg("invert", false);
        return builder_.surface(filename, center, invert);
    }
    
    /********* Instructions **************************************************/

    CSGMesh_var CSGCompiler::multmatrix(
        const ArgList& args, const CSGScope& scope
    ) {
        mat4 xform;
        xform.load_identity();
        xform = args.get_arg("arg_0",xform);
        return builder_.multmatrix(xform, scope);
    }

    CSGMesh_var CSGCompiler::resize(const ArgList& args, const CSGScope& scope) {
        vec3 newsize(1.0, 1.0, 1.0);
        vec3 autosize(0.0, 0.0, 0.0);
        newsize = args.get_arg("newsize",newsize);
        autosize = args.get_arg("autosize",autosize);

        CSGMesh_var result = builder_.union_instr(scope);
        
        vec3 scaling(1.0, 1.0, 1.0);
        double default_scaling = 1.0;
        for(index_t coord=0; coord<3; ++coord) {
            if(newsize[coord] != 0) {
                scaling[coord] = newsize[coord] / (
                    result->bbox().xyz_max[coord] - result->bbox().xyz_min[coord]
                );
                default_scaling = scaling[coord];
            }
        }
        for(index_t coord=0; coord<3; ++coord) {
            if(newsize[coord] == 0.0) {
                if(autosize[coord] == 1.0) {
                    scaling[coord] = default_scaling;
                } 
            }
        }
        for(index_t v: result->vertices) {
            for(index_t c=0; c<result->vertices.dimension(); ++c) {
                result->vertices.point_ptr(v)[c] *= scaling[c];
            }
        }
        result->update_bbox();
        return result;
    }

    CSGMesh_var CSGCompiler::union_instr(
        const ArgList& args, const CSGScope& scope
    ) {
        geo_argused(args);
        return builder_.union_instr(scope);
    }

    CSGMesh_var CSGCompiler::intersection(
        const ArgList& args, const CSGScope& scope
    ) {
        geo_argused(args);
        return builder_.intersection(scope);
    }

    CSGMesh_var CSGCompiler::difference(
        const ArgList& args, const CSGScope& scope
    ) {
        geo_argused(args);        
        return builder_.difference(scope);
    }

    CSGMesh_var CSGCompiler::group(const ArgList& args, const CSGScope& scope) {
        geo_argused(args);
        return builder_.group(scope); 
    }

    CSGMesh_var CSGCompiler::color(const ArgList& args, const CSGScope& scope) {
        vec4 C(1.0, 1.0, 1.0, 1.0);
        C = args.get_arg("arg_0",C);
        return builder_.color(C,scope);
    }

    CSGMesh_var CSGCompiler::hull(const ArgList& args, const CSGScope& scope) {
        geo_argused(args);
        return builder_.hull(scope);
    }
        
    CSGMesh_var CSGCompiler::linear_extrude(
        const ArgList& args, const CSGScope& scope
    ) { 
        double height = args.get_arg("height", 1.0);
        bool center = args.get_arg("center", true);
        vec2 scale = args.get_arg("scale", vec2(1.0, 1.0));
        index_t slices = index_t(args.get_arg("slices",0));
        double twist = args.get_arg("twist",0.0);
        return builder_.linear_extrude(
            scope, height, center, scale, slices, twist
        );
    }

    CSGMesh_var CSGCompiler::rotate_extrude(
        const ArgList& args, const CSGScope& scope
    ) {
        double angle = args.get_arg("angle", 360.0);
        return builder_.rotate_extrude(scope,angle);
    }


    CSGMesh_var CSGCompiler::projection(
        const ArgList& args, const CSGScope& scope
    ) {
        bool cut = args.get_arg("cut", false);
        return builder_.projection(scope,cut);
    }
    
    /********* Parser ********************************************************/

    CSGMesh_var CSGCompiler::parse_instruction_or_object() {
        Token lookahead = lookahead_token();

        // get modifier, one of:
        // '%', '*': ignore subtree (in OpenSCAD, '%' means transparent display)
        // '#'     : does not change anything (in OpenSCAD, transparent display)
        // '!'     : replace root with subtree
        
        char modifier = ' ';
        if(is_modifier(lookahead.type)) {
            modifier = char(next_token().type);
            lookahead = lookahead_token();
        }
        
        if(lookahead.type != CLEX_id) {
            syntax_error("expected id (object or instruction)", lookahead);
        }
        
        CSGMesh_var result;
        std::string instr_or_object_name = lookahead.str_val;
        if(is_object(instr_or_object_name)) {
            result = parse_object();
        } else if(is_instruction(instr_or_object_name)) {
            result = parse_instruction();
        } else {
            syntax_error("id is no known object or instruction", lookahead);
        }

        // '%': no effect on CSG tree, transparent rendering
        // '*': no effect on CSG tree
        if(modifier == '%' || modifier == '*') {
            // Callers ignore instructions and objects that return a
            // null CSGMesh.
            result.reset();
        }

        // '!': replace root
        if(modifier == '!') {
            // It is caught right after the main parsing loop.
            throw(result);
        }

        if(progress_ != nullptr) {
            if(progress_->is_canceled()) {
                throw(std::logic_error("canceled"));
            }
            progress_->progress(index_t(line()));
        }

        if(builder_.verbose()) {
            index_t cur_line = index_t(line());
            Logger::out("CSG") << "Executed " << instr_or_object_name 
                               << " at line " << cur_line << "/" << lines_
                               << "  (" << index_t(cur_line*100)/lines_ << "%)"
                               << std::endl;
        }
        
        return result;
    }
        
    CSGMesh_var CSGCompiler::parse_object() {
        Token tok = next_token();
        if(tok.type != CLEX_id || !is_object(tok.str_val)) {
            syntax_error("expected object");
        }
        std::string object_name = tok.str_val;

        index_t object_line = index_t(line());
        
        ArgList args = parse_arg_list();
        next_token_check(';');

        auto it = object_funcs_.find(object_name);
        geo_assert(it != object_funcs_.end());

        builder_.set_fa(args.get_arg("$fa",CSGBuilder::DEFAULT_FA));
        builder_.set_fs(args.get_arg("$fs",CSGBuilder::DEFAULT_FS));
        builder_.set_fn(args.get_arg("$fn",CSGBuilder::DEFAULT_FN));

        if(builder_.verbose()) {
            Logger::out("CSG") << object_name << " at line: "
                               << object_line << std::endl;
        }
        
        CSGMesh_var result =  (this->*(it->second))(args);

        builder_.reset_defaults();
        
        return result;
    }

    CSGMesh_var CSGCompiler::parse_instruction() {
        Token tok = next_token();
        if(tok.type != CLEX_id || !is_instruction(tok.str_val)) {
            syntax_error("expected instruction",tok);
        }
        std::string instr_name = tok.str_val;

        index_t instruction_line = index_t(line());
        
        ArgList args = parse_arg_list();

        // In .csg files produced by OpenSCAD it often happens that
        // there are empty instructions without any context. I'm ignoring
        // them by returning a null CSGMesh.
        if(lookahead_token().type == ';') {
            next_token_check(';');
            CSGMesh_var dummy_result;
            return dummy_result;
        }
        
        CSGScope scope;
        next_token_check('{');
        for(;;) {
            if(lookahead_token().type == '}') {
                break;
            }
            CSGMesh_var current = parse_instruction_or_object();
            // Can be null if commented-out with modifier
            if(!current.is_null()) {
                scope.push_back(current);
            }
        }
        next_token_check('}');

        if(builder_.verbose()) {
            Logger::out("CSG") << instr_name << " at line: "
                               << instruction_line << std::endl;
        }
        
        auto it = instruction_funcs_.find(instr_name);
        geo_assert(it != instruction_funcs_.end());

        builder_.set_fa(args.get_arg("$fa",CSGBuilder::DEFAULT_FA));
        builder_.set_fs(args.get_arg("$fs",CSGBuilder::DEFAULT_FS));
        builder_.set_fn(args.get_arg("$fn",CSGBuilder::DEFAULT_FN));
        
        CSGMesh_var result = (this->*(it->second))(args,scope);

        builder_.reset_defaults();

        return result;
    }

    CSGCompiler::ArgList CSGCompiler::parse_arg_list() {
        ArgList result;
        next_token_check('(');
        for(;;) {
            if(lookahead_token().type == ')') {
                break;
            }
            std::string arg_name =
                "arg_" + String::to_string(result.size());
            if(lookahead_token().type == CLEX_id) {
                arg_name = next_token().str_val;
                next_token_check('=');
            }
            result.add_arg(arg_name, parse_value());
            if(lookahead_token().type == ')') {
                break;
            }
            next_token_check(',');
        }
        next_token_check(')');
        return result;
    }
        
    CSGCompiler::Value CSGCompiler::parse_value() {
        if(lookahead_token().type == '[') {
            return parse_array();
        }
        Token tok = next_token();
        if(tok.type == '-') {
            tok = next_token();
            if(tok.type == CLEX_intlit) {
                return Value(-tok.int_val);
            } else if(tok.type == CLEX_floatlit) {
                return Value(-tok.double_val);
            } else {
                syntax_error("Expected number", tok);
            }
        }
            
        if(tok.type == CLEX_intlit) {
            return Value(tok.int_val);
        }

        if(tok.type == CLEX_floatlit) {
            return Value(tok.double_val);
        }

        if(tok.type == CLEX_booleanlit) {
            return Value(tok.boolean_val);
        }

        if(tok.type == CLEX_dqstring) {
            return Value(tok.str_val);
        }

        if(tok.type == CLEX_id && tok.str_val == "undef") {
            return Value(); 
        }
        
        syntax_error("Expected value", tok);
    }

    CSGCompiler::Value CSGCompiler::parse_array() {
        Value result;
        result.type = Value::ARRAY1D;
            
        next_token_check('[');
        for(;;) {
            if(lookahead_token().type == ']') {
                break;
            }
            Value item = parse_value();
                
            if(item.type == Value::NUMBER) {
                result.array_val.resize(1);
                result.array_val[0].push_back(item.number_val);
            } else if(item.type == Value::BOOLEAN) {
                result.array_val.resize(1);
                result.array_val[0].push_back(double(item.boolean_val));
            } else if(item.type == Value::ARRAY1D) {
                result.type = Value::ARRAY2D;
                if(item.array_val.size() == 0) {
                    result.array_val.push_back(vector<double>());
                } else {
                    result.array_val.push_back(item.array_val[0]);
                }
            }
                
            if(lookahead_token().type == ']') {
                break;
            }
            next_token_check(',');
        }
        next_token_check(']');

        return result;
    }
    
    bool CSGCompiler::is_object(const std::string& id) const {
        return (object_funcs_.find(id) != object_funcs_.end());
    }

    bool CSGCompiler::is_instruction(const std::string& id) const {
        return (instruction_funcs_.find(id) != instruction_funcs_.end());
    }

    bool CSGCompiler::is_modifier(int toktype) const {
        return 
            (toktype == int('%')) ||
            (toktype == int('#')) ||
            (toktype == int('!')) ||
            (toktype == int('*')) ;
    }
    
    /********* Parser utilities **********************************************/

    void CSGCompiler::next_token_check(char c) {
        Token tok = next_token();
        if(tok.type != int(c)) {
            syntax_error(
                String::format(
                    "Expected %c, got \"%s\"", c, tok.to_string().c_str()
                ).c_str()
            );
        }
    }
        
    CSGCompiler::Token CSGCompiler::next_token() {
        if(lookahead_token_.type != -1) {
            Token result = lookahead_token_;
            lookahead_token_.type = -1;
            return result;
        }
        return next_token_internal();
    }

    CSGCompiler::Token CSGCompiler::lookahead_token() {
        if(lookahead_token_.type == -1) {
            lookahead_token_ = next_token_internal();
        }
        return lookahead_token_;
    }
        
    CSGCompiler::Token CSGCompiler::next_token_internal() {
        Token result;
        if(stb_c_lexer_get_token(&getlex(lex_))) {
            result.type = int(getlex(lex_).token);
            if(getlex(lex_).token == CLEX_id) {
                result.str_val = getlex(lex_).string;                    
                if(result.str_val == "true") {
                    result.type = CLEX_booleanlit;
                    result.boolean_val = true;
                } else if(result.str_val == "false") {
                    result.type = CLEX_booleanlit;
                    result.boolean_val = false;
                } 
            }
            if(getlex(lex_).token == CLEX_dqstring) {
                result.str_val = getlex(lex_).string;
            }
            result.int_val = int(getlex(lex_).int_number);
                result.double_val = getlex(lex_).real_number;
        } else {
            result.type = CLEX_eof;
        }
        if(getlex(lex_).token == CLEX_parse_error) {
            syntax_error("lexical error");
        }
        return result;
    }

    int CSGCompiler::lines() const {
        int result=0;
        for(
            const char* p = getlex(lex_).input_stream;
            p != getlex(lex_).eof; ++p
        ) {
            if(*p == '\n') {
                ++result;
            }
        }
        return result;
    }
    
    int CSGCompiler::line() const {
        int result=1;
        for(
            const char* p = getlex(lex_).input_stream;
            p != getlex(lex_).parse_point && p != getlex(lex_).eof;
            ++p
        ) {
            if(*p == '\n') {
                ++result;
            }
        }
        return result;
    }

    [[noreturn]] void CSGCompiler::syntax_error(const char* msg) {
        throw(
            std::logic_error(
                String::format(
                    "%s:%d %s",
                    filename_.c_str(), line(), msg
                )
            )
        );
    }
    
    [[noreturn]] void CSGCompiler::syntax_error(
        const char* msg, const Token& tok
    ) {
        throw(
            std::logic_error(
                String::format(
                    "%s:%d %s (got \"%s\")",
                    filename_.c_str(), line(), msg,
                    tok.to_string().c_str()
                )
            )
        );
    }

    /********* Value and ArgList *********************************************/

    CSGCompiler::Value::Value() : type(NONE) {
    }

    CSGCompiler::Value::Value(const std::string& x) :
        type(STRING),
        string_val(x) {
    }
    
    CSGCompiler::Value::Value(double x) :
        type(NUMBER),
        number_val(x) {
    }
            
    CSGCompiler::Value::Value(int x) :
        type(NUMBER),
        number_val(double(x)) {
    }
    
    CSGCompiler::Value::Value(bool x) :
        type(BOOLEAN),
        boolean_val(x) {
    }

    std::string CSGCompiler::Value::to_string() const {
        switch(type) {
        case NONE:
            return "<none>";
        case NUMBER:
            return String::to_string(number_val);
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
        case STRING: {
            return "\"" + string_val + "\"";
        }
        }
        return "<unknown>";
    }

    void CSGCompiler::ArgList::add_arg(
        const std::string& name, const Value& value
    ) {
        if(has_arg(name)) {
            throw(std::logic_error("Duplicated arg:" + name));
        }
        args_.push_back(std::make_pair(name,value));
    }
    
    bool CSGCompiler::ArgList::has_arg(const std::string& name) const {
        for(const Arg& arg : args_) {
            if(arg.first == name) {
                return true;
            }
        }
        return false;
    }

    const CSGCompiler::Value& CSGCompiler::ArgList::get_arg(
        const std::string& name
    ) const {
        for(const Arg& arg : args_) {
            if(arg.first == name) {
                return arg.second;
            }
        }
        geo_assert_not_reached;
    }
            
    double CSGCompiler::ArgList::get_arg(
        const std::string& name,double default_value
    ) const {
        for(const Arg& arg : args_) {
            if(arg.first == name) {
                if(arg.second.type != Value::NUMBER) {
                    throw(std::logic_error(
                              "Arg " + name + " has wrong type"
                          ));
                }
                return arg.second.number_val;
            }
        }
        return default_value;
    }

    int CSGCompiler::ArgList::get_arg(
        const std::string& name, int default_value
    ) const {
        for(const Arg& arg : args_) {
            if(arg.first == name) {
                if(arg.second.type != Value::NUMBER) {
                    throw(std::logic_error(
                              "Arg " + name + " has wrong type"
                          ));
                }
                if(GEO::round(arg.second.number_val) !=
                   arg.second.number_val) {
                    throw(std::logic_error(
                              "Arg " + name + " has wrong type"
                          ));
                }
                return int(arg.second.number_val);
            }
        }
        return default_value;
    }
    
    bool CSGCompiler::ArgList::get_arg(
        const std::string& name, bool default_value
    ) const {
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
        return default_value;
    }
    
    vec2 CSGCompiler::ArgList::get_arg(
        const std::string& name, vec2 default_value
    ) const {
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
        return default_value;
    }
    
    vec3 CSGCompiler::ArgList::get_arg(
        const std::string& name, vec3 default_value
    ) const {
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
        return default_value;
    }
    
    vec4 CSGCompiler::ArgList::get_arg(
        const std::string& name, vec4 default_value
    ) const {
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
        return default_value;
    }
            
    mat4 CSGCompiler::ArgList::get_arg(
        const std::string& name, const mat4& default_value
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
        return default_value;
    }

    std::string CSGCompiler::ArgList::get_arg(
        const std::string& name, const std::string& default_value
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
        return default_value;
    }
    
    
    /***** Token **********************************************************/
    
    CSGCompiler::Token::Token() :
        type(-1),
        int_val(0),
        double_val(0.0),
        boolean_val(false) {
    }

    std::string CSGCompiler::Token::to_string() const {
        if(type < 256) {
            return String::format("\'%c\'",type);
        }
        if(type == CLEX_intlit) {
            return String::to_string(int_val);
        }
        if(type == CLEX_floatlit) {
            return String::to_string(double_val);
        }
        if(type == CLEX_booleanlit) {
            return String::to_string(boolean_val);
        }
        if(type == CLEX_id) {
            return str_val;
        }
        return "<unknown token>";
    }
}

