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

#ifndef GEOGRAM_BASIC_BOOLEAN_EXPRESSION
#define GEOGRAM_BASIC_BOOLEAN_EXPRESSION

#include <geogram/basic/common.h>
#include <geogram/basic/numeric.h>
#include <string>

/**
 * \file geogram/basic/boolean_expression.h
 * \brief Class for parsing and evaluating boolean expressions
 */

namespace GEO {

    /**
     * \brief A simple parser for boolean expressions.
     */
    class GEOGRAM_API BooleanExpression {
    public:

        /**
         * \brief Constructs a boolean expression
         * \param[in] expr the expression, with
         *  the following syntax:
         *  - Variables: A..Z or x0..x31 
         *  - and:        '&' or '*'
         *  - or:         '|' or '+'
         *  - xor:        '^'
         *  - difference: '-'
         *  - special: '*' for union
         *  - one can use '(' and ')' to group subexpression
         */
        BooleanExpression(const std::string& expr);

        /**
         * \brief Evaluates the boolean expression 
         * \param[in] x the different bits of x correspond to
         *  the variables A..Z or x0..x31.
         */
        bool operator()(index_t x);

    protected:
        bool parse_or();
        bool parse_and();
        bool parse_factor();
        bool parse_variable();
        char cur_char() const;
        void next_char();
        
    private:
        std::string expr_;
        std::string::iterator ptr_;
        index_t x_;
    };
    
    
}

#endif
