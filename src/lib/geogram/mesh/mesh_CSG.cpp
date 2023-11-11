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
#include <geogram/mesh/mesh_fill_holes.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/delaunay/delaunay.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/file_system.h>
#include <geogram/basic/progress.h>

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
            a->facets.set_adjacent(f + f_ofs, 0, f1 + f_ofs);
            a->facets.set_adjacent(f + f_ofs, 1, f2 + f_ofs);
            a->facets.set_adjacent(f + f_ofs, 2, f3 + f_ofs); 
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
            Attribute<index_t> operand_bit(facets.attributes(),"operand_bit");
            for(index_t f=f_ofs; f<facets.nb(); ++f) {
                operand_bit[f] = index_t(1 << operand);
            }
        }
    }

    /**********************************************************************/
    
    CSGBuilder::CSGBuilder() {
        reset_defaults();
        reset_file_path();        
        STL_epsilon_ = 1e-6;
        create_center_vertex_ = true;
        verbose_ = false;
        max_arity_ = 32;
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

        M->vertices.create_vertex(vec2(x1,y1).data());
        M->vertices.create_vertex(vec2(x2,y1).data());
        M->vertices.create_vertex(vec2(x1,y2).data());
        M->vertices.create_vertex(vec2(x2,y2).data());

        M->facets.create_triangle(0,3,1);
        M->facets.create_triangle(0,2,3);
            
        M->facets.connect();
        M->update_bbox();
        return M;
    }

    CSGMesh_var CSGBuilder::circle(double r) {
        index_t nu = get_fragments_from_r(r);

        CSGMesh_var M = new CSGMesh;
        M->vertices.set_dimension(2);

        for(index_t u=0; u<nu; ++u) {
            double theta = double(u)*2.0*M_PI/double(nu);
            double ctheta = cos(theta);
            double stheta = sin(theta);
            double x = ctheta*r;
            double y = stheta*r;
            M->vertices.create_vertex(vec2(x,y).data());
        }

        if(create_center_vertex_) {
            M->vertices.create_vertex(vec2(0.0,0.0).data());
            for(index_t u=0; u<nu; ++u) {
                M->facets.create_triangle(u, (u+1)%nu, nu);
            }
        } else {
            for(index_t u=1; u+1<nu; ++u) {
                M->facets.create_triangle(0,u,u+1);
            }
        }
            
        M->facets.connect();
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
        index_t nv = index_t((nu / 2) + 1);

        CSGMesh_var M = new CSGMesh;
        M->vertices.set_dimension(3);
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
                    M->facets.create_triangle(v00, v01, v11);
                }
                if(v00 != v10) {
                    M->facets.create_triangle(v00, v11, v10);
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
        if(create_center_vertex_) {
            index_t v1 = M->vertices.create_vertex(vec3(0,0,z1).data());
            index_t v2 = index_t(-1);
            if(r2 != 0.0) {
                v2 = M->vertices.create_vertex(vec3(0,0,z2).data());
            }
            for(index_t u=0; u<nu; ++u) {
                M->facets.create_triangle(v1,u,(u+1)%nu);
                if(r2 != 0.0) {
                    M->facets.create_triangle(v2,nu+(u+1)%nu,nu+u);
                }
            }
        } else {
            for(index_t u=1; u+1<nu; ++u) {
                M->facets.create_triangle(0,u,u+1);
                if(r2 != 0.0) {
                    M->facets.create_triangle(nu,nu+u+1,nu+u);
                }
            }
        }

        // Wall
        for(index_t u=0; u<nu; ++u) {
            if(r2 != 0.0) {
                index_t v00 = u;
                index_t v01 = v00 + nu;
                index_t v10 = (u+1)%nu;
                index_t v11 = v10 + nu;
                M->facets.create_triangle(v00, v01, v11);
                M->facets.create_triangle(v00, v11, v10);
            } else {
                M->facets.create_triangle(u, (u+1)%nu, nu);
            }
        }
            
        M->facets.connect();
        M->update_bbox();
        return M;
    }

    CSGMesh_var CSGBuilder::import(const std::string& filename) {
        std::string full_filename;
        bool found = false;
        for(std::string& path: file_path_) {
            full_filename = path + "/" + filename;
            if(FileSystem::is_file(full_filename)) {
                found = true;
                break;
            }
        }

        CSGMesh_var result;
        if(!found) {
            Logger::err("CSG") << filename << ": file not found"
                               << std::endl;
            return result;
        }
        
        result = new CSGMesh;
        if(!mesh_load(full_filename, *result)) {
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
        result->update_bbox();
        return result;
    }

    
    /****** Instructions ****/
    
    CSGMesh_var CSGBuilder::multmatrix(const mat4& M, const CSGScope& scope) {
        CSGMesh_var result = group(scope);
        for(index_t v: result->vertices) {
            vec3 p(result->vertices.point_ptr(v));
            p = transform_point(M,p);
            result->vertices.point_ptr(v)[0] = p.x;
            result->vertices.point_ptr(v)[1] = p.y;
            result->vertices.point_ptr(v)[2] = p.z;
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
            CSGMesh_var M1 = union_instr(scope1);
            CSGMesh_var M2 = union_instr(scope2);
            CSGMesh_var result = new CSGMesh;
            mesh_union(*result, *M1, *M2);
            return result;
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
        if(result->vertices.dimension() != 3) {
            throw(std::logic_error("2D CSG operations not implemented yet"));
        }
        
        if(may_have_intersections) {
            MeshSurfaceIntersection I(*result);
            I.set_verbose(verbose_);
            I.intersect();
            I.classify("union");
            post_process(result);
        }
        
        result->update_bbox();
        return result;
    }

    CSGMesh_var CSGBuilder::intersection(const CSGScope& scope) {
        if(scope.size() == 1) {
            return scope[0];
        }

        if(scope[0]->vertices.dimension() != 3) {
            throw(std::logic_error("2D CSG operations not implemented yet"));
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
            CSGMesh_var M1 = intersection(scope1);
            CSGMesh_var M2 = intersection(scope2);
            CSGMesh_var result = new CSGMesh;
            mesh_intersection(*result, *M1, *M2);
            return result;
        }

        CSGMesh_var result = append(scope);
        if(result->vertices.dimension() != 3) {
            throw(std::logic_error("2D CSG operations not implemented yet"));
        }
        
        MeshSurfaceIntersection I(*result);
        I.set_verbose(verbose_);
        I.intersect();
        I.classify("intersection");
        post_process(result);
        return result;
    }

    CSGMesh_var CSGBuilder::difference(const CSGScope& scope) {
        if(scope.size() == 1) {
            return scope[0];
        }

        if(scope[0]->vertices.dimension() != 3) {
            throw(std::logic_error("2D CSG operations not implemented yet"));
        }

        // Boolean operations can handle no more than 32 operands.
        // For a difference with more than 32 operands, split it
        // (by calling union_instr() that in turn splits the list).
        if(scope.size() > max_arity_) {
            CSGScope scope2;
            for(index_t i=1; i<scope.size(); ++i) {
                scope2.push_back(scope[i]);
            }
            CSGMesh_var M1 = scope[0];
            CSGMesh_var M2 = union_instr(scope2);
            CSGMesh_var result = new CSGMesh;
            mesh_difference(*result, *M1, *M2);
            return result;

        }

        CSGMesh_var result = append(scope);
        if(result->vertices.dimension() != 3) {
            throw(std::logic_error("2D CSG operations not implemented yet"));
        }
        
        MeshSurfaceIntersection I(*result);
        I.set_verbose(verbose_);
        I.intersect();

        // construct the expression x0-x1-x2...-xn
        std::string expr = "x0";
        for(index_t i=1; i<scope.size(); ++i) {
            expr += "-x" + String::to_string(i);
        }

        I.classify(expr);
        post_process(result);
        return result;
    }

    CSGMesh_var CSGBuilder::append(const CSGScope& scope) {
        if(scope.size() == 1) {
            return scope[0];
        }
        
        CSGMesh_var result = new CSGMesh;
        result->vertices.set_dimension(3);

        if(scope.size() > 32) {
            Logger::warn("CSG") << "Scope with more than 32 children"
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
            // TODO: 2D hull
            throw(std::logic_error("hull() only implemented in 3d (for now)"));
        }
        result->update_bbox();            
        return result;
    }

    CSGMesh_var CSGBuilder::linear_extrude(
        const CSGScope& scope, double height, bool center, vec2 scale
    ) {
        double z1 = center ? -height/2.0 : 0.0;
        double z2 = center ?  height/2.0 : height;

        CSGMesh_var M = scope.size() == 1 ? scope[0] : group(scope);
        if(M->vertices.dimension() != 2) {
            throw(std::logic_error(
                      "linear_extrude: mesh is not of dimension 2"
            ));
        }
        M->vertices.set_dimension(3);

        index_t nv  = M->vertices.nb();
        index_t nf  = M->facets.nb();
        index_t nbv = 0;
            
        // Reorder vertices so that vertices on border come first
        {
            vector<index_t> reorder_vertices(M->vertices.nb(), index_t(-1));
            for(index_t f: M->facets) {
                for(index_t le=0; le<3; ++le) {
                    if(M->facets.adjacent(f,le) == index_t(-1)) {
                        index_t v = M->facets.vertex(f,le);
                        if(reorder_vertices[v] == index_t(-1)) {
                            reorder_vertices[v] = nbv;
                            ++nbv;
                        }
                    }
                }
            }
            index_t curv = nbv;
            for(index_t v: M->vertices) {
                if(reorder_vertices[v] == index_t(-1)) {
                    reorder_vertices[v] = curv;
                    ++curv;
                }
            }
            M->vertices.permute_elements(reorder_vertices);
        }

        for(index_t v: M->vertices) {
            double x = M->vertices.point_ptr(v)[0];
            double y = M->vertices.point_ptr(v)[1];
            x *= scale.x; y *= scale.y;
            M->vertices.point_ptr(v)[2] = z1;
            M->vertices.create_vertex(vec3(x,y,z2).data());
        }

        for(index_t f=0; f<nf; ++f) {
            M->facets.create_triangle(
                M->facets.vertex(f,2) + nv,
                M->facets.vertex(f,1) + nv,
                M->facets.vertex(f,0) + nv
            );
        }

        for(index_t f=0; f<nf; ++f) {
            for(index_t le=0; le<3; ++le) {
                if(M->facets.adjacent(f,le) == index_t(-1)) {
                    index_t v1 = M->facets.vertex(f,le);
                    index_t v2 = M->facets.vertex(f,(le+1)%3);
                    index_t w1 = v1 + nv;
                    index_t w2 = v2 + nv;
                    M->facets.create_triangle(v2,v1,w2);
                    M->facets.create_triangle(w2,v1,w1);
                }
            }
        }            
            
        M->facets.connect();
        M->update_bbox();    
        return M;
    }

    /******************************/

    void CSGBuilder::post_process(CSGMesh_var mesh) {
        geo_argused(mesh);
        // Do correct snaprounding here
    }
    
    index_t CSGBuilder::get_fragments_from_r(double r) {
        if (fn_ > 0.0) {
            return index_t(fn_ >= 3 ? fn_ : 3);
        }
        return index_t(ceil(fmax(fmin(360.0 / fa_, r*2*M_PI / fs_), 5)));
    }
    
    /************************************************************************/
    
    CSGCompiler::CSGCompiler() : lex_(nullptr), progress_(nullptr) {
        
#define DECLARE_OBJECT(obj) object_funcs_[#obj] = &CSGCompiler::obj;
        DECLARE_OBJECT(square);
        DECLARE_OBJECT(circle);
        DECLARE_OBJECT(cube);
        DECLARE_OBJECT(sphere);
        DECLARE_OBJECT(cylinder);
        DECLARE_OBJECT(polyhedron);
        DECLARE_OBJECT(import);
        
#define DECLARE_INSTRUCTION(instr) \
        instruction_funcs_[#instr] = &CSGCompiler::instr;
        DECLARE_INSTRUCTION(multmatrix);
        DECLARE_INSTRUCTION(intersection);
        DECLARE_INSTRUCTION(difference);
        DECLARE_INSTRUCTION(group);
        DECLARE_INSTRUCTION(color);
        DECLARE_INSTRUCTION(hull);
        DECLARE_INSTRUCTION(linear_extrude);
        instruction_funcs_["union"]  = &CSGCompiler::union_instr;
        instruction_funcs_["render"] = &CSGCompiler::group;
    }
    
    CSGMesh_var CSGCompiler::compile_file(const std::string& input_filename) {
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

        // Strip trailing zeroes
        // (under Windows it happens, I don't know why)
        // TODO: see if this should be moved to FileSystem::load_file_as_string() 
        source.resize(strlen(source.c_str())); 
        
        // Add the directory that contains the file to the builder's file path,
        // so that import() instructions are able to find files in the same
        // directory.
        builder_.add_file_path(FileSystem::dir_name(input_filename));
        CSGMesh_var result = compile_string(source);
        builder_.reset_file_path();
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
            ProgressTask progress("CSG",index_t(lines()), builder_.verbose());
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

        if(
            points.type != Value::ARRAY2D ||
            faces.type != Value::ARRAY2D 
        ) {
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
                if(v < 0.0 || v > double(M->vertices.nb())) {
                    syntax_error("polyhedron: invalid vertex index");
                }
                M->facets.set_vertex(new_f, lv, index_t(v));
            }
        }

        tessellate_facets(*M,3);
            
        M->facets.connect();
        M->update_bbox();
        return M;
    }

    CSGMesh_var CSGCompiler::import(const ArgList& args) {
        std::string filename = "";
        filename = args.get_arg("file", filename);
        CSGMesh_var M = builder_.import(filename);
        if(M.is_null()) {
            syntax_error((filename + ": could not load").c_str());
        }
        return M;
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
        vec2 scale(1.0, 1.0);
        scale = args.get_arg("scale", scale);
        return builder_.linear_extrude(scope, height, center, scale);
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
        if(is_object(lookahead.str_val)) {
            result = parse_object();
        } else if(is_instruction(lookahead.str_val)) {
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
        
        return result;
    }
        
    CSGMesh_var CSGCompiler::parse_object() {
        Token tok = next_token();
        if(tok.type != CLEX_id || !is_object(tok.str_val)) {
            syntax_error("expected object");
        }
        std::string object_name = tok.str_val;
        ArgList args = parse_arg_list();
        next_token_check(';');

        auto it = object_funcs_.find(object_name);
        geo_assert(it != object_funcs_.end());

        builder_.set_fa(args.get_arg("$fa",CSGBuilder::DEFAULT_FA));
        builder_.set_fs(args.get_arg("$fs",CSGBuilder::DEFAULT_FS));
        builder_.set_fn(args.get_arg("$fn",CSGBuilder::DEFAULT_FN));
        
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
        ArgList args = parse_arg_list();
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

        auto it = instruction_funcs_.find(instr_name);
        geo_assert(it != instruction_funcs_.end());
        return (this->*(it->second))(args,scope);
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

