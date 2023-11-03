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


/*
 * OpenSCAD CSG file format:
 * - https://github.com/openscad/openscad/wiki/CSG-File-Format
 * - https://wiki.freecad.org/OpenSCAD_CSG
 * - https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/Other_Language_Features
 *  $fa: minimum angle for a fragment
 *  $fs: minimum size for a fragment
 *  $fn: number of fragment (0: ignored, else overrides $fa and $fs)
 *
 *  int get_fragments_from_r(double r, double fn, double fs, double fa)
 *  {
 *           if (r < GRID_FINE) return 3;
 *           if (fn > 0.0) return (int)(fn >= 3 ? fn : 3);
 *           return (int)ceil(fmax(fmin(360.0 / fa, r*2*M_PI / fs), 5));
 *  }
 */

#include <geogram/basic/common.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/basic/stopwatch.h>
#include <geogram/basic/file_system.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_surface_intersection.h>
#include <geogram/mesh/mesh_fill_holes.h>

// Silence some warnings in stb_c_lexere.h

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
#endif
#endif

#define STB_C_LEXER_IMPLEMENTATION
#include "stb_c_lexer.h"

#ifdef GEO_COMPILER_GCC_FAMILY
#pragma GCC diagnostic pop
#endif

#ifdef GEO_COMPILER_MSVC
#pragma warning (pop)
#endif

namespace {
    using namespace GEO;


    /**
     * \brief a += b
     */
    void append_mesh(Mesh* a, const Mesh* b) {
        geo_assert(a->facets.are_simplices());
        geo_assert(b->facets.are_simplices());
        index_t v_ofs = a->vertices.nb();
        index_t f_ofs = a->facets.nb();
        a->vertices.create_vertices(b->vertices.nb());
        a->facets.create_triangles(b->facets.nb());
        for(index_t v: b->vertices) {
            a->vertices.point_ptr(v + v_ofs)[0] = b->vertices.point_ptr(v)[0];
            a->vertices.point_ptr(v + v_ofs)[1] = b->vertices.point_ptr(v)[1];
            a->vertices.point_ptr(v + v_ofs)[2] = b->vertices.point_ptr(v)[2];
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
    }

    
    class CSGParser {
    public:

        static constexpr int CLEX_booleanlit = CLEX_first_unused_token;
        
        struct Token {
            Token() :
                type(-1),
                int_val(0),
                double_val(0.0),
                boolean_val(false) {
            }
            
            double to_number() const {
                if(type == CLEX_intlit) {
                    return double(int_val);
                }
                if(type == CLEX_floatlit) {
                    return double_val;
                }
                throw(std::logic_error("Token is not a number"));
            }

            std::string to_string() const {
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
            int type;
            std::string str_val;
            int int_val;
            double double_val;
            bool boolean_val;
        };

        enum ValueType {
            VALUETYPE_none,
            VALUETYPE_number,
            VALUETYPE_boolean,
            VALUETYPE_array1d,
            VALUETYPE_array2d
        };

        struct Value {
            Value() : type(VALUETYPE_none) {
            }
            Value(double x) :
                type(VALUETYPE_number),
                number_val(x) {
            }
            Value(int x) :
                type(VALUETYPE_number),
                number_val(double(x)) {
            }
            Value(bool x) :
                type(VALUETYPE_boolean),
                boolean_val(x) {
            }
            
            std::string to_string() const {
                switch(type) {
                case VALUETYPE_none:
                    return "<none>";
                case VALUETYPE_number:
                    return String::to_string(number_val);
                case VALUETYPE_boolean:
                    return String::to_string(boolean_val);
                case VALUETYPE_array1d: {
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
                case VALUETYPE_array2d: {
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
                }
                return "<unknown>";
            }
            
            ValueType type;
            bool boolean_val;
            double number_val;
            vector<vector<double> > array_val;
        };

        class ArgList {
        public:
            typedef std::pair<std::string, Value> Arg;
            
            void add_arg(const std::string& name, const Value& value) {
                if(has_arg(name)) {
                    throw(std::logic_error("Duplicated arg:" + name));
                }
                args_.push_back(std::make_pair(name,value));
            }

            index_t size() const {
                return args_.size();
            }

            bool has_arg(const std::string& name) const {
                for(const Arg& arg : args_) {
                    if(arg.first == name) {
                        return true;
                    }
                }
                return false;
            }

            const Value& get_arg(const std::string& name) const {
                for(const Arg& arg : args_) {
                    if(arg.first == name) {
                        return arg.second;
                    }
                }
                geo_assert_not_reached;
            }
            
            double get_arg(const std::string& name,double default_value) const {
                for(const Arg& arg : args_) {
                    if(arg.first == name) {
                        if(arg.second.type != VALUETYPE_number) {
                            throw(std::logic_error(
                                      "Arg " + name + " has wrong type"
                            ));
                        }
                        return arg.second.number_val;
                    }
                }
                return default_value;
            }

            int get_arg(const std::string& name, int default_value) const {
                for(const Arg& arg : args_) {
                    if(arg.first == name) {
                        if(arg.second.type != VALUETYPE_number) {
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

            bool get_arg(const std::string& name, bool default_value) const {
                for(const Arg& arg : args_) {
                    if(arg.first == name) {
                        if(arg.second.type != VALUETYPE_boolean) {
                            throw(std::logic_error(
                                      "Arg " + name + " has wrong type"
                            ));
                        }
                        return arg.second.boolean_val;
                    }
                }
                return default_value;
            }

            vec3 get_arg(const std::string& name, vec3 default_value) const {
                for(const Arg& arg : args_) {
                    if(arg.first == name) {
                        if(arg.second.type != VALUETYPE_array1d) {
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
            
            vec4 get_arg(const std::string& name, vec4 default_value) const {
                for(const Arg& arg : args_) {
                    if(arg.first == name) {
                        if(arg.second.type != VALUETYPE_array1d) {
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
            
            mat4 get_arg(
                const std::string& name, const mat4& default_value
            ) const {
                for(const Arg& arg : args_) {
                    if(arg.first == name) {
                        if(arg.second.type != VALUETYPE_array2d) {
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
                                result(i,j) = Mvv[j][i];
                            }
                        }
                        return result;
                    }
                }
                return default_value;
            }
            
        private:
            vector<Arg> args_;
        };

        class CSGMesh : public Mesh, public Counted {
        public:
        };

        typedef SmartPointer<CSGMesh> CSGMesh_var;
        
        typedef vector< CSGMesh_var > Scope;
        
        CSGParser(
            const std::string& input_filename,
            const std::string& output_filename
        ) : filename_(input_filename) {
            try {
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
                source_ = root->load_file_as_string(filename_);
                if(source_.length() == 0) {
                    throw std::logic_error(
                        filename_ + ": could not open file"
                    );
                }

                stb_c_lexer_init(
                    &lex_,
                    source_.c_str(),
                    source_.c_str()+source_.length(),
                    (char *)malloc(0x10000),
                    0x10000
                );

                Scope scope;
                
                for(;;) {
                    if(lookahead_token().type == CLEX_eof) {
                        break;
                    }
                    scope.push_back(instruction_or_object());
                }

                ArgList args;
                CSGMesh_var result = group(args, scope);
                mesh_save(*result,output_filename);
                
            } catch(const std::logic_error& e) {
                Logger::err("CSG") << "Error while parsing file:"
                                        << e.what()
                                        << std::endl;
                return;
            }
        }
            
    protected:

        CSGMesh_var instruction_or_object() {
            Token lookahead = lookahead_token();
            if(lookahead.type != CLEX_id) {
                syntax_error("expected id (object or instruction)");
            }
            if(is_object(lookahead.str_val)) {
                return object();
            } else if(is_instruction(lookahead.str_val)) {
                return instruction();
            } else {
                syntax_error("id is no known object or instruction", lookahead);
            }
            return nullptr;
        }
        
        CSGMesh_var object() {
            Token tok = next_token();
            if(tok.type != CLEX_id || !is_object(tok.str_val)) {
                syntax_error("expected object");
            }
            std::string object_name = tok.str_val;
            ArgList args = arg_list();
            next_token_check(';');

            if(object_name == "square") {
                return square(args);
            }

            if(object_name == "circle") {
                return circle(args);
            }
            
            if(object_name == "cube") {
                return cube(args);
            }

            if(object_name == "sphere") {
                return sphere(args);
            }

            if(object_name == "cylinder") {
                return cylinder(args);
            }

            if(object_name == "polyhedron") {
                return polyhedron(args);
            }
            syntax_error("Unknown object (should not have reaced)", tok);
            return nullptr;
        }

        CSGMesh_var instruction() {
            Token tok = next_token();
            if(tok.type != CLEX_id || !is_instruction(tok.str_val)) {
                syntax_error("expected instruction",tok);
            }
            std::string instr_name = tok.str_val;
            ArgList args = arg_list();
            Scope scope;
            next_token_check('{');
            for(;;) {
                if(lookahead_token().type == '}') {
                    break;
                }
                scope.push_back(instruction_or_object());
            }
            next_token_check('}');

            if(instr_name == "multmatrix") {
                return multmatrix(args, scope);
            }

            if(instr_name == "union") {
                return union_instr(args, scope);
            }

            if(instr_name == "intersection") {
                return intersection(args, scope);
            }

            if(instr_name == "difference") {
                return difference(args, scope);
            }

            if(instr_name == "group") {
                return group(args, scope);
            }

            if(instr_name == "color") {
                return color(args, scope);
            }
            
            if(instr_name == "linear_extrude") {
                return linear_extrude(args, scope);
            }

            syntax_error("unknown instruction (should not have reached)", tok);
            return nullptr;
        }

        ArgList arg_list() {
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
                result.add_arg(arg_name, value());
                if(lookahead_token().type == ')') {
                    break;
                }
                next_token_check(',');
            }
            next_token_check(')');
            return result;
        }
        
        Value value() {
            if(lookahead_token().type == '[') {
                return array();
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

            syntax_error("Expected value", tok);
            return Value();
        }

        Value array() {
            Value result;
            result.type = VALUETYPE_array1d;
            
            next_token_check('[');
            for(;;) {
                if(lookahead_token().type == ']') {
                    break;
                }
                Value item = value();
                
                if(item.type == VALUETYPE_number) {
                    result.array_val.resize(1);
                    result.array_val[0].push_back(item.number_val);
                } else if(item.type == VALUETYPE_array1d) {
                    result.type = VALUETYPE_array2d;
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

        /*****************************************************/

        bool is_object(const std::string& id) {
            return
                id == "square"   ||
                id == "circle"   ||                
                id == "cube"     ||
                id == "sphere"   ||
                id == "cylinder" ||
                id == "polyhedron" 
                ;
        }
        
        CSGMesh_var square(const ArgList& args) {
            geo_argused(args);
            syntax_error("square: not implemented yet");
            return nullptr;
        }

        CSGMesh_var circle(const ArgList& args) {
            geo_argused(args);
            syntax_error("circle: not implemented yet");
            return nullptr;
        }
        
        CSGMesh_var cube(const ArgList& args) {
            vec3 size = args.get_arg("size", vec3(1.0, 1.0, 1.0));
            bool center = args.get_arg("center", true);
            
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

            return M;
        }

        CSGMesh_var sphere(const ArgList& args) {

            double r = args.get_arg("r", 1.0);
            if(r <= 0.0) {
                syntax_error(
                    String::format("sphere: radius %f is <= 0.0",r).c_str()
                );
            }
            
            double fa = args.get_arg("$fa",12.0);
            fa = std::max(fa,0.01);

            double fs = args.get_arg("$fs",2.0);
            fs = std::max(fs,0.01);

            int fn = args.get_arg("$fn", 0);

            index_t nu = get_fragments_from_r(r,fn,fs,fa);
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

            return M;
        }

        CSGMesh_var cylinder(const ArgList& args) {
            double h    = args.get_arg("h", 1.0);
            double r1   = args.get_arg("r1", 1.0);
            double r2   = args.get_arg("r2", 1.0);
            bool center = args.get_arg("center", true);
            
            double fa = args.get_arg("$fa",12.0);
            fa = std::max(fa,0.01);

            double fs = args.get_arg("$fs",2.0);
            fs = std::max(fs,0.01);

            int fn = args.get_arg("$fn", 0);

            index_t nu = get_fragments_from_r(std::max(r1,r2),fn,fs,fa);

            double z1 = center ? -h/2.0 : 0.0;
            double z2 = center ?  h/2.0 : 1.0;

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
            bool create_center_vertex = true;
            if(create_center_vertex) {
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

            // Side
            
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
            return M;
        }

        CSGMesh_var polyhedron(const ArgList& args) {
            CSGMesh_var M = new CSGMesh;
            if(!args.has_arg("points") || !args.has_arg("faces")) {
                syntax_error("polyhedron: missing points or facets");
            }
            const Value& points = args.get_arg("points");
            const Value& faces = args.get_arg("faces");

            if(
                points.type != VALUETYPE_array2d ||
                faces.type != VALUETYPE_array2d 
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
                index_t new_f = M->facets.create_polygon(faces.array_val[f].size());
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
            return M;
        }

        static index_t get_fragments_from_r(
            double r, int fn, double fs, double fa
        ) {
            if (fn > 0.0) {
                return index_t(fn >= 3 ? fn : 3);
            }
            return index_t(ceil(fmax(fmin(360.0 / fa, r*2*M_PI / fs), 5)));
        }

        
        /********************************************************/
        
        bool is_instruction(const std::string& id) {
            return
                id == "multmatrix"   ||
                id == "union"        ||
                id == "intersection" ||
                id == "difference"   ||
                id == "group"        ||
                id == "color"        ||
                id == "linear_extrude"
                ;
        }

        CSGMesh_var multmatrix(const ArgList& args, Scope& scope) {
            mat4 xform;
            xform.load_identity();
            xform = args.get_arg("arg_0",xform);

            CSGMesh_var result = group(args, scope);
            for(index_t v: result->vertices) {
                vec3 p(result->vertices.point_ptr(v));
                p = transform_point(p,xform);
                result->vertices.point_ptr(v)[0] = p.x;
                result->vertices.point_ptr(v)[1] = p.y;
                result->vertices.point_ptr(v)[2] = p.z;
            }
            return result;
        }

        CSGMesh_var union_instr(const ArgList& args, Scope& scope) {
            geo_argused(args);
            if(scope.size() == 1) {
                return scope[0];
            }
            CSGMesh_var result = group(args, scope);
            MeshSurfaceIntersection I(*result);
            I.intersect();
            I.remove_internal_shells();
            return result;
        }

        CSGMesh_var intersection(const ArgList& args, Scope& scope) {
            geo_argused(args);
            if(scope.size() == 1) {
                return scope[0];
            }
            if(scope.size() == 2) {
                CSGMesh_var result = new CSGMesh;
                mesh_intersection(*result, *scope[0], *scope[1]);
                return result;
            }

            CSGMesh_var M1 = scope.back();
            scope.pop_back();
            CSGMesh_var M2 = intersection(args, scope);
            CSGMesh_var result = new CSGMesh;
            mesh_intersection(*result, *M1, *M2);
            return result;
        }

        CSGMesh_var difference(const ArgList& args, Scope& scope) {
            geo_argused(args);
            if(scope.size() == 1) {
                return scope[0];
            }
            if(scope.size() == 2) {
                CSGMesh_var result = new CSGMesh;
                mesh_difference(*result, *scope[0], *scope[1]);
                return result;
            }
            
            Scope scope2;
            for(index_t i=1; i<scope.size(); ++i) {
                scope2.push_back(scope[i]);
            }
            CSGMesh_var op2 = union_instr(args, scope2);
            CSGMesh_var result = new CSGMesh;
            mesh_difference(*result, *scope[0], *op2);
            return result;
        }

        CSGMesh_var group(const ArgList& args, Scope& scope) {
            geo_argused(args);
            if(scope.size() == 1) {
                return scope[0];
            }
            CSGMesh_var result = new CSGMesh;
            result->vertices.set_dimension(3);
            for(CSGMesh_var current : scope) {
                append_mesh(result, current);
            }
            return result;
        }

        CSGMesh_var color(const ArgList& args, Scope& scope) {
            vec4 C(1.0, 1.0, 1.0, 1.0);
            C = args.get_arg("arg_0",C);
            geo_argused(C); // TODO: store color in result
            CSGMesh_var result =  group(args, scope);
            return result;
        }
        
        CSGMesh_var linear_extrude(const ArgList& args, Scope& scope) {
            geo_argused(args);
            geo_argused(scope);
            syntax_error("linear_extrude: not implemented yet");
            return nullptr;
        }

        /********************************************************/
        
        void next_token_check(char c) {
            Token tok = next_token();
            if(tok.type != int(c)) {
                syntax_error(
                    String::format(
                        "Expected %c, got \"%s\"", c, tok.to_string().c_str()
                    ).c_str()
                );
            }
        }
        
        Token next_token() {
            if(lookahead_token_.type != -1) {
                Token result = lookahead_token_;
                lookahead_token_.type = -1;
                return result;
            }
            return next_token_internal();
        }

        Token lookahead_token() {
            if(lookahead_token_.type == -1) {
                lookahead_token_ = next_token_internal();
            }
            return lookahead_token_;
        }
        
        Token next_token_internal() {
            Token result;
            if(stb_c_lexer_get_token(&lex_)) {
                result.type = int(lex_.token);
                if(lex_.token == CLEX_id) {
                    result.str_val = lex_.string;                    
                    if(result.str_val == "true") {
                        result.type = CLEX_booleanlit;
                        result.boolean_val = true;
                    } else if(result.str_val == "false") {
                        result.type = CLEX_booleanlit;
                        result.boolean_val = false;
                    } 
                }
                result.int_val = int(lex_.int_number);
                result.double_val = lex_.real_number;
            } else {
                result.type = CLEX_eof;
            }
            if(lex_.token == CLEX_parse_error) {
                syntax_error("lexical error");
            }
            return result;
        }

        int line() const {
            int result=1;
            for(
                const char* p = lex_.input_stream;
                p != lex_.parse_point && p != lex_.eof;
                ++p
            ) {
                if(*p == '\n') {
                    ++result;
                }
            }
            return result;
        }

        void syntax_error(const char* msg) {
            throw(
                std::logic_error(
                    String::format(
                        "%s:%d %s",
                        filename_.c_str(), line(), msg
                    )
                )
            );
        }

        void syntax_error(const char* msg, const Token& tok) {
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

        
    private:
        std::string filename_;
        std::string source_;
        stb_lexer lex_;
        Token lookahead_token_;
   };
}

int main(int argc, char** argv) {
    using namespace GEO;
    try {

        GEO::initialize();
        
        Stopwatch Wtot("Total time");
        
        CmdLine::import_arg_group("standard");
        CmdLine::import_arg_group("algo");

        std::vector<std::string> filenames;

        if(
            !CmdLine::parse(
                argc, argv, filenames, "csgfilename <outputfile|none>"
            )
        ) {
            return 1;
        }
        
        std::string csg_filename = filenames[0];

        std::string output_filename =
            filenames.size() >= 2 ? filenames[1] : std::string("out.meshb");


        CSGParser CSG(csg_filename, output_filename);
        
        
    }
    catch(const std::exception& e) {
        std::cerr << "Received an exception: " << e.what() << std::endl;
        return 1;
    }
    
    Logger::out("") << "Everything OK, Returning status 0" << std::endl;
    return 0;
}

