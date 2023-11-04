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

    class CSGMesh : public Mesh, public Counted {
    };
    
    typedef SmartPointer<CSGMesh> CSGMesh_var;
    typedef vector< CSGMesh_var > CSGScope;    

    class GEOGRAM_API CSGCompiler {
    public:
        
        CSGCompiler();
        CSGMesh_var compile_file(const std::string& input_filename);
        CSGMesh_var compile_source(const std::string& source);

        protected:

        /****** Value, Arglist **********************************/

        enum ValueType {
            VALUETYPE_none,
            VALUETYPE_number,
            VALUETYPE_boolean,
            VALUETYPE_array1d,
            VALUETYPE_array2d
        };

        struct Value {
            Value();
            Value(double x);
            Value(int x);
            Value(bool x);
            std::string to_string() const;
            
            ValueType type;
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
        
        static index_t get_fragments_from_r(
            double r, int fn, double fs, double fa
        );
        
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

        bool is_object(const std::string& id) const;
        bool is_instruction(const std::string& id) const;
        CSGMesh_var instruction_or_object();
        CSGMesh_var object();
        CSGMesh_var instruction();
        ArgList arg_list();
        Value value();
        Value array();
        
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
        bool create_center_vertex_;
        
        typedef CSGMesh_var (CSGCompiler::*object_funptr)(const ArgList& args);
        typedef CSGMesh_var (CSGCompiler::*instruction_funptr)(
            const ArgList& args, const CSGScope& scope
        );
        std::map<std::string, object_funptr> object_funcs_;
        std::map<std::string, instruction_funptr> instruction_funcs_;
        };
}

#endif
