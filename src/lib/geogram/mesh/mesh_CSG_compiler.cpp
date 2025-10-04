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
 *     https://www.inria.fr/en/bruno-levy-1
 *
 *     Inria,
 *     Domaine de Voluceau,
 *     78150 Le Chesnay - Rocquencourt
 *     FRANCE
 *
 */

#include <geogram/mesh/mesh_CSG_compiler.h>
#include <geogram/mesh/mesh_fill_holes.h>
#include <geogram/mesh/mesh_topology.h>
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
#endif

#ifdef GEO_COMPILER_CLANG
#pragma clang diagnostic ignored "-Wreserved-id-macro"
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma clang diagnostic ignored "-Wself-assign"
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#pragma clang diagnostic ignored "-Wunused-member-function"
#pragma clang diagnostic ignored "-Wcast-qual"
#pragma clang diagnostic ignored "-Wunused-macros"
#pragma clang diagnostic ignored "-Wimplicit-fallthrough"
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
#ifdef GEOGRAM_USE_BUILTIN_DEPS
#include <geogram/third_party/stb/stb_c_lexer.h>
#else
#include <stb/stb_c_lexer.h>
#endif

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

/******************************************************************************/

namespace GEO {
    CSGCompiler::CSGCompiler() : lex_(nullptr), lines_(0) {
	progress_ = nullptr;
	builder_ = std::make_shared<CSGBuilder>();
    }

    std::shared_ptr<Mesh> CSGCompiler::compile_file(
	const std::filesystem::path& input_filename
    ) {
        // Add the directory that contains the file to the builder's file path,
        // so that import() instructions are able to find files in the same
        // directory.
        builder_->push_file_path(input_filename.parent_path());
	std::string source = GEOCSG::load_OpenSCAD(input_filename);
        if(source.length() == 0) {
            throw std::logic_error(
                filename_.string() + ": could not open file (or file is empty)"
            );
        }

        std::shared_ptr<Mesh> result = compile_string(source);

        if(result != nullptr && result->vertices.dimension() == 2) {
            result->vertices.set_dimension(3);
        }

        builder_->pop_file_path();
        return result;
    }

    std::shared_ptr<Mesh> CSGCompiler::compile_string(
	const std::string& source
    ) {
        std::shared_ptr<Mesh> result;

        static constexpr size_t BUFFER_SIZE = 0x10000;
        char* buffer = new char[BUFFER_SIZE];
        stb_lexer lex;
        lex_ = &lex;
	lines_ = NO_INDEX;
	line_ = NO_INDEX;
	line_ptr_ = nullptr;

        try {
            stb_c_lexer_init(
                &lex,
                source.c_str(),
                source.c_str()+source.length(),
                buffer, BUFFER_SIZE
            );
            ProgressTask progress("CSG", index_t(lines()), builder_->verbose());
            progress_ = &progress;
            CSGScope scope;
	    builder().push_scope();
            while(lookahead_token().type != CLEX_eof) {
                parse_instruction_or_object();
            }
	    result = builder().union_instr(builder().top_scope());
	    builder().pop_scope();
        } catch(std::shared_ptr<Mesh> reroot) {
            Logger::out("CSG") << "Re-rooted (!) from line " << line()
                               << std::endl;
            result = reroot;
        } catch(const std::logic_error& e) {
            Logger::err("CSG") << "Error while parsing file:"
                               << e.what()
                               << std::endl;
	    result = std::make_shared<Mesh>();
        }
        delete[] buffer;
        lex_ = nullptr;
	progress_ = nullptr;
        lines_ = 0;


	if(result->vertices.dimension() == 3) {
	    reorient_connected_components(*result);
	}

        return result;
    }

    /********* Parser ********************************************************/

    void CSGCompiler::parse_instruction_or_object() {
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

        std::string instr_or_object_name = lookahead.str_val;
        if(is_object(instr_or_object_name)) {
	    parse_object();
        } else if(is_instruction(instr_or_object_name)) {
	    parse_instruction();
        } else {
            syntax_error("id is no known object or instruction", lookahead);
        }

        // '%': no effect on CSG tree, transparent rendering
        // '*': no effect on CSG tree
        if(modifier == '%' || modifier == '*') {
	    builder().top_scope().pop_back(); // remove latest generated object
        }

        // '!': replace root
        if(modifier == '!') {
            // It is caught right after the main parsing loop.
	    std::shared_ptr<Mesh> new_root = *builder().top_scope().rbegin();
	    throw(new_root);
        }

        if(progress_ != nullptr) {
            if(progress_->is_canceled()) {
                throw(std::logic_error("canceled"));
            }
            progress_->progress(index_t(line()));
        }

        if(builder_->verbose()) {
            index_t cur_line = index_t(line());
            Logger::out("CSG") << "Executed " << instr_or_object_name
                               << " at line " << cur_line << "/" << lines_
                               << "  (" << index_t(cur_line*100) /
		                        std::max(lines_,index_t(1)) << "%)"
                               << std::endl;
        }
    }

    void CSGCompiler::parse_object() {
        Token tok = next_token();
        if(tok.type != CLEX_id || !is_object(tok.str_val)) {
            syntax_error("expected object");
        }
        std::string object_name = tok.str_val;

        index_t object_line = index_t(line());

        ArgList args = parse_arg_list();
        next_token_check(';');

        builder_->set_fa(args.get_arg("$fa",CSGBuilder::DEFAULT_FA));
        builder_->set_fs(args.get_arg("$fs",CSGBuilder::DEFAULT_FS));
        builder_->set_fn(args.get_arg("$fn",CSGBuilder::DEFAULT_FN));

        if(builder_->verbose()) {
            Logger::out("CSG") << object_name << " at line: "
                               << object_line << std::endl;
        }

	builder_->add_object(object_name, args);
        builder_->reset_defaults();
    }

    void CSGCompiler::parse_instruction() {
        Token tok = next_token();
        if(tok.type != CLEX_id || !is_instruction(tok.str_val)) {
            syntax_error("expected instruction",tok);
        }
        std::string instr_name = tok.str_val;

        index_t instruction_line = index_t(line());

        ArgList args = parse_arg_list();


        // In .csg files produced by OpenSCAD it often happens that
        // there are empty instructions without any scope.
        if(lookahead_token().type == ';') {
            next_token_check(';');
	    builder_->begin_instruction();
	    builder_->end_instruction(instr_name, args);
	    return;
        }

	builder_->begin_instruction();
        next_token_check('{');
        for(;;) {
            if(lookahead_token().type == '}') {
                break;
            }
            parse_instruction_or_object();
        }
        next_token_check('}');

        if(builder_->verbose()) {
            Logger::out("CSG") << instr_name << " at line: "
                               << instruction_line << std::endl;
        }

        builder_->set_fa(args.get_arg("$fa",CSGBuilder::DEFAULT_FA));
        builder_->set_fs(args.get_arg("$fs",CSGBuilder::DEFAULT_FS));
        builder_->set_fn(args.get_arg("$fn",CSGBuilder::DEFAULT_FN));

	builder_->end_instruction(instr_name, args);
        builder_->reset_defaults();
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
        return builder().is_object(id);
    }

    bool CSGCompiler::is_instruction(const std::string& id) const {
        return builder().is_instruction(id);
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
	if(lines_ == NO_INDEX) {
	    compute_lines();
	}
	return int(lines_);
    }

    void CSGCompiler::compute_lines() const {
        index_t result=0;
        for(
            const char* p = getlex(lex_).input_stream;
            p != getlex(lex_).eof; ++p
        ) {
            if(*p == '\n') {
                ++result;
            }
        }
	lines_ = result;
    }

    int CSGCompiler::line() const {
	if(line_ == NO_INDEX) {
	    line_=1;
	    for(
		line_ptr_ = getlex(lex_).input_stream;
		( line_ptr_ != getlex(lex_).parse_point &&
		  line_ptr_ != getlex(lex_).eof            );
		++line_ptr_
	    ) {
		if(*line_ptr_ == '\n') {
		    ++line_;
		}
	    }
	    return int(line_);
	}
	while(line_ptr_ != getlex(lex_).parse_point &&
	      line_ptr_ != getlex(lex_).eof            ) {
	    if(*line_ptr_ == '\n') {
		++line_;
	    }
	    ++line_ptr_;
	}
	return int(line_);
    }

    [[noreturn]] void CSGCompiler::syntax_error(const char* msg) {
        throw(
            std::logic_error(
                String::format(
                    "%s:%d %s",
                    filename_.string().c_str(), line(), msg
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
                    filename_.string().c_str(), line(), msg,
                    tok.to_string().c_str()
                )
            )
        );
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
