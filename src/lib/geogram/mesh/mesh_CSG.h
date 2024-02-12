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

    class ProgressTask;
    class Image;
    
    /**
     * \brief A Mesh with reference counting and bounding box.
     */
    class GEOGRAM_API CSGMesh : public Mesh, public Counted {
    public:
        CSGMesh();
        ~CSGMesh() override;
        /**
         * \brief Gets the bounding box
         * \return a const reference to the bounding box, as a
         *  a Box3d. If it is a 2d mesh, then z bounds are set to 0
         */
         const Box3d& bbox() const {
             geo_debug_assert(vertices.nb() == 0 || bbox_initialized());
             return bbox_;
        }

         /**
          * \brief Tests whether the bounding box was initialized
          * \details Used for debugging purposes, for instance, to detect
          *  that the bounding box of a mesh was not properly updated
          * \retval true if the bounding box was not changed since construction
          * \retval false otherwise
          */
         bool bbox_initialized() const;

        /**
         * \brief Computes the bounding box
         * \details This function needs to be called each time the
         *  mesh is modified
         */
        void update_bbox();

        /**
         * \brief Appends a mesh to this mesh
         * \param[in] other a const pointer to the mesh to be appended
         * \param[in] operand an optional operand id, or index_t(-1) if not
         *   setting operand_bit attribute
         * \details Copies all the vertices and the facets of \p other
         */
        void append_mesh(const CSGMesh* other, index_t operand = index_t(-1));

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
     * \details Can be used to construct volumes in C++ with a syntax
     *  very similar to OpenSCAD .csg files.
     */
    class GEOGRAM_API CSGBuilder {
    public:
        /** \see set_fa() */
        static constexpr double DEFAULT_FA = 12.0;

        /** \see set_fs() */
        static constexpr double DEFAULT_FS = 2.0;
        
        /** \see set_fn() */
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
        CSGMesh_var import(
            const std::string& filename, const std::string& layer="",
            index_t timestamp=0,
            vec2 origin = vec2(0.0, 0.0), vec2 scale = vec2(1.0,1.0)
        );
        CSGMesh_var surface(
            const std::string& filename, bool center, bool invert
        );
        
        /****** Instructions ****/
        
        /**
         * \brief Groups several meshes into a single one and transforms
         *   them.
         * \param[in] M the transformation matrix. It follows the same 
         *   convention as OpenSCAD, that is, not the OpenGL convention.
         *   For instance, a translation matrix has the translation vector
         *   as its third column.
         * \param[in] scope one or several meshes to be merged.
         */
        CSGMesh_var multmatrix(const mat4& M, const CSGScope& scope);

        /**
         * \brief Computes the union of two or more meshes.
         * \param[in] scope the meshes 
         */
        CSGMesh_var union_instr(const CSGScope& scope);

        /**
         * \brief Computes the intersection between two or more meshes.
         * \param[in] scope the meshes 
         */
        CSGMesh_var intersection(const CSGScope& scope);

        /**
         * \brief Computes the intersection between two meshes.
         * \details If \p scope contains more than two meshes, it computes
         *   the difference between the first mesh and the union of the rest.
         * \param[in] scope the meshes 
         */
        CSGMesh_var difference(const CSGScope& scope);
        
        /**
         * \brief synonym for union.
         * \details Maybe there's something I did not understand in 
         *  OpenSCAD, but I do not see the difference between group 
         *  and union.
         */
        CSGMesh_var group(const CSGScope& scope) {
            return union_instr(scope);
        }

        /**
         * \brief Groups several meshes into a single one and sets their
         *   color.
         * \param[in] color the color, as r,g,b,a.
         * \details ignored for now, just behaves as group().
         */
        CSGMesh_var color(vec4 color, const CSGScope& scope);

        /**
         * \brief Computes the convex hull of several meshes.
         * \param[in] scope the meshes
         */
        CSGMesh_var hull(const CSGScope& scope);

        /**
         * \brief Computes a 3D extrusion from a 2D shape
         * \param[in] scope one or more 2D shapes
         * \param[in] height total height of the extrusion
         * \param[in] center if set, z will go from -height/2 to height/2,
         *   else from 0 to height
         * \param[in] scale scaling factor to be applied to x and y coordinates
         *   when reaching \p height
         * \param[in] slices number of slices along the z axis
         * \param[in] twist rotation to be applied when sweeping, in degrees 
         */
        CSGMesh_var linear_extrude(
            const CSGScope& scope,
            double height = 1.0,
            bool center = true,
            vec2 scale = vec2(1.0,1.0),
            index_t slices = 0,
            double twist = 0.0
        );


        /**
         * \brief Computes a 3D extrusion from a 2D shape
         * \param[in] scope one or more 2D shapes. Everything should be on the
         *   same side of the Y axis, preferably the positive side.
         * \param[in] angle optional angle
         */
        CSGMesh_var rotate_extrude(const CSGScope& scope, double angle = 360.0);

        /**
         * \brief Creates a 2D mesh from 3D mesh.
         * \param[in] cut if set, computes the boundary of the intersection
         *   between the object and the X,Y plane, else computes the boundary
         *   of the projection.
         */
        CSGMesh_var projection(const CSGScope& scope, bool cut);
        
        /**
         * \brief Appends all meshes in scope into a unique mesh,
         *  without testing for intersections.
         */
        CSGMesh_var append(const CSGScope& scope);
        
        /****** Parameters ******/

        /**
         * \brief Resets defaults value for fn, fs, fa
         * \see set_fn(), set_fs(), set_fa()
         */
        void reset_defaults();

        /**
         * \brief Sets the number of fragments. 
         * \details This corresponds to the number of edges in a polygonal
         *  approximation of a circle. If left to 0, it is automatically
         *  computed from fs and fa
         * \param[in] fn the number of fragments.
         * \see set_fs(), set_fa()
         */
        void set_fn(double fn) {
            fn_ = std::max(fn, 0.0);
        }

        /**
         * \brief Sets the minimum size for a fragment.
         * \param[in] fs minimum size for a fragment.
         * \details This determines the number of edges in a polygonal
         *  approximation of a circle.
         */
        void set_fs(double fs) {
            fs_ = std::max(fs,0.01);
        }

        /**
         * \brief Sets the minimum angle for a fragment.
         * \param[in] fa minimum angle for a fragment, in degrees.
         * \details This determines the number of edges in a polygonal
         *  approximation of a circle.
         */
        void set_fa(double fa) {
            fa_ = std::max(fa,0.01);
        }

        /**
         * \brief If set, compute constrained Delaunay triangulation
         *  in the intersected triangles. If there are intersections
         *  in coplanar facets, it guarantees uniqueness of their
         *  triangulation. Default is set.
         */
        void set_delaunay(bool x) {
            delaunay_ = x;
        }
        
        /** 
         * \brief detect and compute intersections between facets that share 
         *  a facet or an edge. Set to false if input is a set of conformal
         *  meshes. Default is set.
         */
        void set_detect_intersecting_neighbors(bool x) {
            detect_intersecting_neighbors_ = x;
        }
        
        /**
         * \brief Specifies whether coplanar facets should be simplified
         * \param[in] x if set, coplanar facets are simplified, else they
         *  are kept as is (faster but generates many triangles). Default
         *  is set.
         * \param[in] angle_tolerance (in degree) the pairs of 
         *  adjacent facets with normals that make an angle smaller than
         *  this threshold as considered to be coplanar.
         */
        void set_simplify_coplanar_facets(bool x, double angle_tolerance=0.0) {
            simplify_coplanar_facets_ = x;
            coplanar_angle_tolerance_ = angle_tolerance;
        }

        /**
         * \brief Sets fast union mode
         * \details In fast union mode, all intersections are computed and the
         *  external shell is kept. It may give incorrect result if an object
         *  is floating inside another one (completely included).
         * \param[in] x true if fast union mode should be used, false otherwise.
         */
        void set_fast_union(bool x) {
            fast_union_ = x;
        }
        
        /**
         * \brief Displays (lots of) additional information
         * \param[in] x whether additional information should be displayed. 
         *  Default is off
         */
        void set_verbose(bool x) {
            verbose_ = x;
        }

        /**
         * \brief Tests wheter verbose mode is set.
         * \retval true if additional information will be displayed.
         * \retval false otherwise.
         */
        bool verbose() const {
            return verbose_;
        }

        /**
         * \brief Adds a path to the file path
         * \details The file path is where import() searches files. The default
         *  file path contains the current directory "."
         * \param[in] path the file path to be added, without trailing '/'
         */
        void add_file_path(const std::string& path) {
            file_path_.push_back(path);
        }

        /**
         * \brief Resets the file path to its default value, with only the
         *  current directory "."
         */
        void reset_file_path() {
            file_path_.clear();
            file_path_.push_back(".");
        }
        
    protected:

        bool find_file(std::string& filename);
    
        void do_CSG(CSGMesh_var mesh, const std::string& boolean_expr);
    
        /**
         * \brief Triangulates a 2D mesh.
         * \param[in,out] mesh the input is a set of vertices and edges. 
         *   The output has a set of triangles inside.
         * \param[in] keep_border_only if set, then triangles are discarded. It
         *   useful to compute 2D boolean operations, where only the border is
         *   kept.
         */
        void triangulate(
            CSGMesh_var mesh, const std::string& boolean_expr,
            bool keep_border_only=false
        );
    
       /**
        * \brief For the file formats that are not supported by geogram,
        *  get help from OpenSCAD to convert them.
        * \details Converts STEP files.
        */
        CSGMesh_var import_with_openSCAD(
            const std::string& filename, const std::string& layer="",
            index_t timestamp=0
        );

        /**
         * \brief Loads an ascii data file as an image
         * \param[in] file_name the name of the file, containing a matrix in
         *  the octave file format
         * \return a pointer to the created image. Color encoding is Image::GRAY
         *  and component encoding is Image::FLOAT64.
         */
        Image* load_dat_image(const std::string& file_name);
        
        /**
         * \brief Post-processes the result of a previous intersection
         * \details After converting exact coordinates to doubles, some
         *  problems may occur. This function tentatively fixes this
         *  problems. TODO: correct snap-rounding.
         * \param[in] mesh the mesh to be processed
         */
        void post_process(CSGMesh_var mesh);
    
        /**
         * \brief Computes the number of fragments, that is, edges
         *  in a polygonal approximation of a circle.
         * \param[in] r the radius of the circle
         * \param[in] twist the portion of the circle that will be drawn, 
         *   in degrees
         * \details Uses fn,fs,fa
         * \see set_fn(), set_fs(), set_fa()
         */
        index_t get_fragments_from_r(double r, double twist = 360.0);
        
    private:
        double fn_;
        double fs_;
        double fa_;
        double STL_epsilon_;
        bool verbose_;
        index_t max_arity_;
        std::vector<std::string> file_path_;
        bool detect_intersecting_neighbors_;
        bool delaunay_;
        bool simplify_coplanar_facets_;
        double coplanar_angle_tolerance_;
        bool fast_union_;
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

        /**
         * \brief Displays (lots of) additional information
         * \param[in] x whether additional information should be displayed. 
         *  Default is off
         */
        void set_verbose(bool x) {
            builder_.set_verbose(x);
        }

        /**
         * \brief Gets the CSGbuilder
         * \return a reference to the CSGBuilder
         */
        CSGBuilder& builder() {
            return builder_;
        }
        
        protected:

        /****** Value, Arglist **********************************/

        /**
         * \brief A parsed value in a .csg file
         * \details Can be a number, a boolean, a 1d array or a 2d array
         */
        struct Value {
            enum Type {NONE, NUMBER, BOOLEAN, ARRAY1D, ARRAY2D, STRING};
            
            Value();
            Value(double x);
            Value(int x);
            Value(bool x);
            Value(const std::string& x);
            std::string to_string() const;
            
            Type type;
            bool boolean_val;
            double number_val;
            vector<vector<double> > array_val;
            std::string string_val;
        };

        /**
         * \brief A parsed argument list in a .csg file.
         * \details Stores name-value pairs.
         */
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
            std::string get_arg(
                const std::string& name, const std::string& default_value
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
        CSGMesh_var polygon(const ArgList& args);
        CSGMesh_var import(const ArgList& args);
        CSGMesh_var surface(const ArgList& args);
        
        /****** Instructions ************************************/

        CSGMesh_var multmatrix(const ArgList& args, const CSGScope& scope);
        CSGMesh_var resize(const ArgList& args, const CSGScope& scope);
        CSGMesh_var union_instr(const ArgList& args, const CSGScope& scope);
        CSGMesh_var intersection(const ArgList& args, const CSGScope& scope);
        CSGMesh_var difference(const ArgList& args, const CSGScope& scope);
        CSGMesh_var group(const ArgList& args, const CSGScope& scope);
        CSGMesh_var color(const ArgList& args, const CSGScope& scope);
        CSGMesh_var hull(const ArgList& args, const CSGScope& scope);
        CSGMesh_var linear_extrude(const ArgList& args, const CSGScope& scope);
        CSGMesh_var rotate_extrude(const ArgList& args, const CSGScope& scope);
        CSGMesh_var projection(const ArgList& args, const CSGScope& scope);

        /***** Parser *******************************************/

        CSGMesh_var parse_instruction_or_object();
        CSGMesh_var parse_object();
        CSGMesh_var parse_instruction();
        ArgList     parse_arg_list();
        Value       parse_value();
        Value       parse_array();
        bool        is_object(const std::string& id) const;
        bool        is_instruction(const std::string& id) const;
        
        /**
         * \brief Checks if a token corresponds to an instruction or
         *  object modifier
         * \details A modifier is one of '%','#','!','*', where '%' and '*'
         *  discard the subtree, '#' does not change anything and '!' replaces
         *  the result with the subtree (re-root).
         *  Note: in OpenSCAD, '%' and '#' display the subtree as a transparent 
         *  object.
         */
        bool is_modifier(int toktype) const;
        
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

        /**
         * \brief Checks that the next token is a given character
         * \details If the next token is something else than the given character,
         *   then parsing stops with an error message.
         * \param[in] c the character
         */
        void next_token_check(char c);

        /**
         * \brief Gets the next token.
         * \details Parsing proceeds to the next token.
         */
        Token next_token();

        /**
         * \brief Gets the next token without any side effect.
         * \details Parsing position remains at the same token.
         */
        Token lookahead_token();

        /**
         * \brief Function to actually get the next token from the stream.
         * \details next_token() and lookahead_token() use a 1-token
         *  buffer to pretend that one can look at a token in advance
         *  without consuming it.
         */
        Token next_token_internal();

        /**
         * \brief Gets the total number of lines of the currently parsed source.
         */
        int lines() const;
        
        /**
         * \brief Gets the currently parsed line source.
         */
        int line() const;

        
        /**
         * \brief Throws an exception with an error message.
         * \param[in] msg the error message to be displayed
         */
        [[noreturn]] void syntax_error(const char* msg);

        /**
         * \brief Throws an exception with an error message.
         * \param[in] msg the error message to be displayed
         * \param[in] tok the currently parsed token, will be
         *   appended to the error message
         */
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
        ProgressTask* progress_;
        index_t lines_;
    };
}

#endif
