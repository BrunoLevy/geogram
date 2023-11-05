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

#ifndef H_GEO_MESH_ALGO_MESH_CSG_H
#define H_GEO_MESH_ALGO_MESH_CSG_H

#include <geogram/basic/common.h>
#include <geogram/mesh/mesh.h>
#include <geogram/basic/smart_pointer.h>
#include <geogram/basic/memory.h>

/**
 * \file geogram/mesh/mesh_CSG.h
 * \brief Functions and classes for Constructive Solid Geometry with meshes.
 */

namespace GEO {

    /**
     * \brief A Mesh with reference counting and bounding box.
     */
    class GEOGRAM_API CSGMesh : public Mesh, public Counted {
    public:
        ~CSGMesh() override;
        /**
         * \brief Gets the bounding box
         * \return a const reference to the bounding box, as a
         *  a Box3d. If it is a 2d mesh, then z bounds are set to 0
         */
         const Box3d& bbox() const {
            return bbox_;
        }

        /**
         * \brief Computes the bounding box
         * \details This function needs to be called each time the
         *  mesh is modified
         */
        void update_bbox();

        /**
         * \brief Appends a mesh to this mesh
         * \param[in] other a const pointer to the mesh to be appended
         * \details Copies all the vertices and the facets of \p other
         */
        void append_mesh(const CSGMesh* other);

        /**
         * \brief Tests whether this mesh may have an intersection with 
         *  another mesh
         * \details Tests the bounding boxes for intersection. To be used
         *  as a filter before calling the (costly) intersection algorithm.
         * \param[in] other a const pointer to the other mesh to be tested
         * \retval false if there is no intersection with \p other
         * \retval true if there is possibly an intersection, but there may
         *  be false positives
         */
        bool may_have_intersections_with(const CSGMesh* other) const {
            return bboxes_overlap(bbox(), other->bbox());
        }
        
    private:
        Box3d bbox_;
    };

    /**
     * \brief A smart pointer to a CSGMesh.
     */
    typedef SmartPointer<CSGMesh> CSGMesh_var;

    /**
     * \brief A list of CSGMesh.
     * \details The meshes are stored as smart pointers.
     */
    typedef std::vector< CSGMesh_var > CSGScope;    
    
    /**
     * \brief Implements CSG objects and instructions.
     */
    class GEOGRAM_API CSGBuilder {
    public:
        static constexpr double DEFAULT_FA = 12.0;
        static constexpr double DEFAULT_FS = 2.0;        
        static constexpr double DEFAULT_FN = 0.0;
        
        CSGBuilder();

        /****** Objects **********/
        
        CSGMesh_var square(vec2 size = vec2(1.0,1.0), bool center=true);
        CSGMesh_var circle(double r=1.0);
        CSGMesh_var cube(vec3 size = vec3(1.0, 1.0, 1.0), bool center=true);
        CSGMesh_var sphere(double r=1.0);
        CSGMesh_var cylinder(
            double h=1.0, double r1=1.0, double r2=1.0, bool center=true
        );

        /****** Instructions ****/
        
        CSGMesh_var multmatrix(const mat4& M, const CSGScope& scope);
        CSGMesh_var union_instr(const CSGScope& scope);
        CSGMesh_var intersection(const CSGScope& scope);
        CSGMesh_var difference(const CSGScope& scope);
        
        /**
         * \brief synonym for unio
         */
        CSGMesh_var group(const CSGScope& scope) {
            return union_instr(scope);
        }
        
        CSGMesh_var color(vec4 color, const CSGScope& scope);
        
        CSGMesh_var hull(const CSGScope& scope);
        
        CSGMesh_var linear_extrude(
            const CSGScope& scope,
            double height = 1.0,
            bool center = true,
            vec2 scale = vec2(1.0,1.0)
        );
        /**
         * \brief Appends all meshes in scope into a unique mesh,
         *  without testing for intersections.
         */
        CSGMesh_var append(const CSGScope& scope);
        
        /****** Parameters ******/

        void reset_defaults();
        
        void set_fn(double fn) {
            fn_ = std::max(fn, 0.0);
        }
        void set_fs(double fs) {
            fs_ = std::max(fs,0.01);
        }
        void set_fa(double fa) {
            fa_ = std::max(fa,0.01);
        }

    protected:
        index_t get_fragments_from_r(double r);
        
    private:
        bool create_center_vertex_;
        double fn_;
        double fs_;
        double fa_;
    };

    /**************************************************************/
    
    /**
     * \brief Creates meshes from OpenSCAD .csg files.
     * \details Understands a subset of OpenSCAD .csg format.
     */
    class GEOGRAM_API CSGCompiler {
    public:
        
        CSGCompiler();
        CSGMesh_var compile_file(const std::string& input_filename);
        CSGMesh_var compile_string(const std::string& source);

        protected:

        /****** Value, Arglist **********************************/

        struct Value {
            enum Type {NONE, NUMBER, BOOLEAN, ARRAY1D, ARRAY2D};
            
            Value();
            Value(double x);
            Value(int x);
            Value(bool x);
            std::string to_string() const;
            
            Type type;
            bool boolean_val;
            double number_val;
            vector<vector<double> > array_val;
        };


        class ArgList {
        public:
            typedef std::pair<std::string, Value> Arg;
            
            index_t size() const {
                return args_.size();
            }

            const std::string& ith_arg_name(index_t i) const {
                geo_assert(i < size());
                return args_[i].first;
            }
            
            const Value& ith_arg_val(index_t i) const {
                geo_assert(i < size());
                return args_[i].second;
            }
            
            void add_arg(const std::string& name, const Value& value);
            bool has_arg(const std::string& name) const;
            const Value& get_arg(const std::string& name) const;
            double get_arg(const std::string& name,double default_value) const;
            int get_arg(const std::string& name, int default_value) const;
            bool get_arg(const std::string& name, bool default_value) const;
            vec2 get_arg(const std::string& name, vec2 default_value) const;
            vec3 get_arg(const std::string& name, vec3 default_value) const;
            vec4 get_arg(const std::string& name, vec4 default_value) const;
            mat4 get_arg(
                const std::string& name, const mat4& default_value
            ) const;
            
        private:
            vector<Arg> args_;
        };
        
        
        /****** Objects *****************************************/
        
        CSGMesh_var square(const ArgList& args);
        CSGMesh_var circle(const ArgList& args);
        CSGMesh_var cube(const ArgList& args);
        CSGMesh_var sphere(const ArgList& args);
        CSGMesh_var cylinder(const ArgList& args);
        CSGMesh_var polyhedron(const ArgList& args);
        
        /****** Instructions ************************************/

        CSGMesh_var multmatrix(const ArgList& args, const CSGScope& scope);
        CSGMesh_var union_instr(const ArgList& args, const CSGScope& scope);
        CSGMesh_var intersection(const ArgList& args, const CSGScope& scope);
        CSGMesh_var difference(const ArgList& args, const CSGScope& scope);
        CSGMesh_var group(const ArgList& args, const CSGScope& scope);
        CSGMesh_var color(const ArgList& args, const CSGScope& scope);
        CSGMesh_var hull(const ArgList& args, const CSGScope& scope);
        CSGMesh_var linear_extrude(const ArgList& args, const CSGScope& scope);

        /***** Parser *******************************************/

        CSGMesh_var parse_instruction_or_object();
        CSGMesh_var parse_object();
        CSGMesh_var parse_instruction();
        ArgList     parse_arg_list();
        Value       parse_value();
        Value       parse_array();
        bool        is_object(const std::string& id) const;
        bool        is_instruction(const std::string& id) const;
        
        /***** Parser internals ********************************/

        struct Token {
            Token();
            std::string to_string() const;
            int type;
            std::string str_val;
            int int_val;
            double double_val;
            bool boolean_val;
        };
        
        void next_token_check(char c);
        Token next_token();
        Token lookahead_token();
        Token next_token_internal();
        int line() const;
        [[noreturn]] void syntax_error(const char* msg);
        [[noreturn]] void syntax_error(const char* msg, const Token& tok);

    private:
        std::string filename_;
        void* lex_;
        Token lookahead_token_;
        CSGBuilder builder_;
        
        typedef CSGMesh_var (CSGCompiler::*object_funptr)(const ArgList& args);
        typedef CSGMesh_var (CSGCompiler::*instruction_funptr)(
            const ArgList& args, const CSGScope& scope
        );
        std::map<std::string, object_funptr> object_funcs_;
        std::map<std::string, instruction_funptr> instruction_funcs_;
    };
}

#endif
