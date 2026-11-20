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

#include <geogram/basic/boolean_expression.h>

namespace GEO {

    BooleanExpression::BooleanExpression(
        const std::string& expr
    ) : expr_(expr) {
    }

    bool BooleanExpression::operator()(index_t x) const {
	Context C(expr_, x);
	return parse_or(C);
    }

    bool BooleanExpression::parse_or(Context& C) const {
        bool left = parse_and(C);
        while(
            C.cur_char() == '|' ||
            C.cur_char() == '^' ||
            C.cur_char() == '+' ||
            C.cur_char() == '-'
        ) {
            char op = C.cur_char();
            C.next_char();
            bool right = parse_and(C);
            left = (op == '-') ? (left && !right) :
                (op == '^') ? (left ^   right) :
                (left || right) ;
        }
        return left;
    }

    bool BooleanExpression::parse_and(Context& C) const {
        bool left = parse_factor(C);
        while(C.cur_char() == '&' || C.cur_char() == '*') {
            C.next_char();
            bool right = parse_factor(C);
            left = left && right;
        }
        return left;
    }

    bool BooleanExpression::parse_factor(Context& C) const {
        if(C.cur_char() == '!' || C.cur_char() == '~' || C.cur_char() == '-') {
            C.next_char();
            return !parse_factor(C);
        }
        if(C.cur_char() == '(') {
            C.next_char();
            bool result = parse_or(C);
            if(C.cur_char() != ')') {
                throw std::logic_error(
                    std::string("Unmatched parenthesis: ")+C.cur_char()
                );
            }
            C.next_char();
            return result;
        }
        if((C.cur_char() == '*')) {
            C.next_char();
            return (C.x_ != 0);
        }
        if((C.cur_char() >= 'A' && C.cur_char() <= 'Z') || C.cur_char() == 'x') {
            return parse_variable(C);
        }
        throw std::logic_error("Syntax error");
    }

    bool BooleanExpression::parse_variable(Context& C) const {
        int bit = 0;
        if(C.cur_char() >= 'A' && C.cur_char() <= 'Z') {
            bit = int(C.cur_char()) - int('A');
            C.next_char();
        } else {
            if(C.cur_char() != 'x') {
                throw std::logic_error("Syntax error in variable");
            }
            C.next_char();
            while(C.cur_char() >= '0' && C.cur_char() <= '9') {
                bit = bit * 10 + (int(C.cur_char()) - '0');
                C.next_char();
            }
        }
        if(bit > 31) {
            throw std::logic_error("Bit larger than 31");
        }
        return ((C.x_ & (index_t(1u) << bit)) != 0);
    }
}
