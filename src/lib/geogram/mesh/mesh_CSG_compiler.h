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

#ifndef H_GEOGRAM_MESH_MESH_CSG_COMPILER_H
#define H_GEOGRAM_MESH_MESH_CSG_COMPILER_H

#include <geogram/basic/common.h>
#include <geogram/mesh/mesh_CSG_builder.h>

#include <filesystem>
#include <map>

namespace GEO {

    class ProgressTask;

    /**
     * \brief Creates meshes from OpenSCAD .csg files.
     * \details Understands a subset of OpenSCAD .csg format.
     */
    class GEOGRAM_API CSGCompiler {
    public:

    typedef GEOCSG::Value Value;
    typedef GEOCSG::ArgList ArgList;

    CSGCompiler();

    std::shared_ptr<Mesh> compile_file(
	const std::filesystem::path& input_filename
    );
    std::shared_ptr<Mesh> compile_string(const std::string& source);

    /**
     * \brief Displays (lots of) additional information
     * \param[in] x whether additional information should be displayed.
     *  Default is off
     */
    void set_verbose(bool x) {
        builder_->set_verbose(x);
    }

    /**
     * \brief Displays (even more) additional information
     * \param[in] x whether additional information should be displayed.
     *  Default is off
     */
    void set_fine_verbose(bool x) {
        builder_->set_fine_verbose(x);
    }

    /**
     * \brief Gets the CSGbuilder
     * \return a reference to the CSGBuilder
     */
    CSGBuilder& builder() {
        return *builder_;
    }

    protected:

    /****** Objects *****************************************/

    std::shared_ptr<Mesh> square(const ArgList& args);
    std::shared_ptr<Mesh> circle(const ArgList& args);
    std::shared_ptr<Mesh> cube(const ArgList& args);
    std::shared_ptr<Mesh> sphere(const ArgList& args);
    std::shared_ptr<Mesh> cylinder(const ArgList& args);
    std::shared_ptr<Mesh> polyhedron(const ArgList& args);
    std::shared_ptr<Mesh> polygon(const ArgList& args);
    std::shared_ptr<Mesh> import(const ArgList& args);
    std::shared_ptr<Mesh> surface(const ArgList& args);
    std::shared_ptr<Mesh> text(const ArgList& args);

    /****** Instructions ************************************/

    std::shared_ptr<Mesh> multmatrix(
	const ArgList& args, const CSGScope& scope
    );

    std::shared_ptr<Mesh> resize(
	const ArgList& args, const CSGScope& scope
    );

    std::shared_ptr<Mesh> union_instr(
	const ArgList& args, const CSGScope& scope
    );

    std::shared_ptr<Mesh> intersection(
	const ArgList& args, const CSGScope& scope
    );

    std::shared_ptr<Mesh> difference(const ArgList& args, const CSGScope& scope);
    std::shared_ptr<Mesh> group(const ArgList& args, const CSGScope& scope);
    std::shared_ptr<Mesh> color(const ArgList& args, const CSGScope& scope);
    std::shared_ptr<Mesh> hull(const ArgList& args, const CSGScope& scope);

    std::shared_ptr<Mesh> linear_extrude(
	const ArgList& args, const CSGScope& scope
    );

    std::shared_ptr<Mesh> rotate_extrude(
	const ArgList& args, const CSGScope& scope
    );

    std::shared_ptr<Mesh> projection(
	const ArgList& args, const CSGScope& scope
    );

    /***** Parser *******************************************/

    std::shared_ptr<Mesh> parse_instruction_or_object();
    std::shared_ptr<Mesh> parse_object();
    std::shared_ptr<Mesh> parse_instruction();
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
    std::filesystem::path filename_;
    void* lex_;
    Token lookahead_token_;

    typedef std::shared_ptr<Mesh> (CSGCompiler::*object_funptr)(
	const ArgList& args
    );

    typedef std::shared_ptr<Mesh> (CSGCompiler::*instruction_funptr)(
        const ArgList& args, const CSGScope& scope
    );

    std::map<std::string, object_funptr> object_funcs_;
    std::map<std::string, instruction_funptr> instruction_funcs_;
    ProgressTask* progress_;
    index_t lines_;
    std::shared_ptr<CSGBuilder> builder_;
    };

}

#endif
