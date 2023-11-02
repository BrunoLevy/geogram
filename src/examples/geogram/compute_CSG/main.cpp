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
 */

#include <geogram/basic/common.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/basic/stopwatch.h>
#include <geogram/basic/file_system.h>


#ifdef GEO_COMPILER_GCC_FAMILY
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wunused-function"
#endif

#define STB_C_LEXER_IMPLEMENTATION
#include "stb_c_lexer.h"

#ifdef GEO_COMPILER_GCC_FAMILY
#pragma GCC diagnostic push
#endif

namespace {
    using namespace GEO;

    class CSGParser {
    public:

        struct Token {
            Token() :
                type(-1),
                str_val("<uninitialized>"),
                int_val(0),
                double_val(0.0) {
            }
            int type;
            std::string str_val;
            int int_val;
            double double_val;
        };

        CSGParser(const std::string& filename) : filename_(filename) {
            try {
                if(
                    FileSystem::extension(filename) != "csg" &&
                    FileSystem::extension(filename) != "CSG"
                ) {
                    throw std::logic_error(
                        filename_ + ": wrong extension (should be .csg or .CSG)"
                    );
                }
                
                FileSystem::Node* root;
                FileSystem::get_root(root);
                source_ = root->load_file_as_string(filename);
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

                for(;;) {
                    if(lookahead_token().type == CLEX_eof) {
                        break;
                    }
                    instruction_or_object();
                }
                
            } catch(const std::logic_error& e) {
                Logger::err("CSG") << "Error while parsing file:"
                                        << e.what()
                                        << std::endl;
                return;
            }
        }
            
    protected:

        void instruction_or_object() {
            Token lookahead = lookahead_token();
            if(lookahead.type != CLEX_id) {
                syntax_error("expected id (object or instruction)");
            }
            if(is_object(lookahead.str_val)) {
                object();
            } else if(is_instruction(lookahead.str_val)) {
                instruction();
            } else {
                syntax_error("id is no known object or instruction");
            }
        }
        
        void object() {
            Token tok = next_token();
            if(tok.type != CLEX_id || !is_object(tok.str_val)) {
                syntax_error("expected object");
            }
            std::string object_name = tok.str_val;
            next_token_check('(');
            for(;;) {
                if(lookahead_token().type == ')') {
                    break;
                }
                tok = next_token();
                if(tok.type != CLEX_id) {
                    syntax_error("expected arg name");
                }
                std::string arg_name = tok.str_val;
                next_token_check('=');
                value();
                if(lookahead_token().type == ')') {
                    break;
                }
                next_token_check(',');
            }
            next_token_check(')');
            next_token_check(';');
        }

        void instruction() {
            Token tok = next_token();
            if(tok.type != CLEX_id || !is_instruction(tok.str_val)) {
                syntax_error("expected instruction");
            }
            std::string instr_name = tok.str_val;
            next_token_check('(');
            for(;;) {
                if(lookahead_token().type == ')') {
                    break;
                }
                value();
                if(lookahead_token().type == ')') {
                    break;
                }
                next_token_check(',');
            }
            next_token_check(')');
            next_token_check('{');
            for(;;) {
                if(lookahead_token().type == '}') {
                    break;
                }
                instruction_or_object();
            }
            next_token_check('}');
        }

        void value() {
            if(lookahead_token().type == '[') {
                array();
                return;
            }
            Token tok = next_token();
            if(tok.type == '-') {
                tok = next_token();
                if(
                    tok.type   != CLEX_intlit   &&
                    tok.type   != CLEX_floatlit 
                ) {
                    syntax_error(
                        String::format("Expected number, got \"%c\"", tok.type).c_str()
                    );
                }
            }
            if(
                tok.type   != CLEX_intlit   &&
                tok.type   != CLEX_floatlit &&
                tok.str_val != "true" &&
                tok.str_val != "false"
            ) {
                syntax_error(
                    String::format("Expected value, got \"%c\"", tok.type).c_str()
                );
            }
        }

        void array() {
            next_token_check('[');
            for(;;) {
                if(lookahead_token().type == ']') {
                    break;
                }
                value();
                if(lookahead_token().type == ']') {
                    break;
                }
                next_token_check(',');
            }
            next_token_check(']');
        }
        
        bool is_object(const std::string& id) {
            return
                id == "cube"     ||
                id == "sphere"   ||
                id == "cylinder"
                ;
        }

        bool is_instruction(const std::string& id) {
            return
                id == "multmatrix"   ||
                id == "union"        ||
                id == "intersection" ||
                id == "difference"   ||
                id == "group"        
                ;
        }


        void next_token_check(char c) {
            Token tok = next_token();
            if(tok.type != int(c)) {
                syntax_error(
                    String::format("Expected %c", c).c_str()
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
                result.type = lex_.token;
                if(lex_.token == CLEX_id) {
                    result.str_val = lex_.string;
                }
                result.int_val = lex_.int_number;
                result.double_val = lex_.real_number;
            } else {
                result.type = CLEX_eof;
            }
            if(lex_.token == CLEX_parse_error) {
                syntax_error("lexical error");
            }
            return result;
        }

        index_t line() const {
            index_t result=1;
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


        CSGParser CSG(csg_filename);
        
        
    }
    catch(const std::exception& e) {
        std::cerr << "Received an exception: " << e.what() << std::endl;
        return 1;
    }
    
    Logger::out("") << "Everything OK, Returning status 0" << std::endl;
    return 0;
}

