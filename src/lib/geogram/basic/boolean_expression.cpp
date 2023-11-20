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

    bool BooleanExpression::operator()(index_t x) {
        x_   = x;
        ptr_ = expr_.begin();
        return parse_or();
    }

    bool BooleanExpression::parse_or() {
        bool left = parse_and();
        while(
            cur_char() == '|' ||
            cur_char() == '^' ||
            cur_char() == '+' ||
            cur_char() == '-'
        ) {
            char op = cur_char();
            next_char();
            bool right = parse_and();
            left = (op == '-') ? (left && !right) :
                (op == '^') ? (left ^   right) :
                (left ||  right) ;
        }
        return left;
    }

    bool BooleanExpression::parse_and() {
        bool left = parse_factor();
        while(cur_char() == '&' || cur_char() == '*') {
            next_char();
            bool right = parse_factor();
            left = left && right;
        }
        return left;
    }

    bool BooleanExpression::parse_factor() {
        if(cur_char() == '!' || cur_char() == '~' || cur_char() == '-') {
            next_char();
            return !parse_factor();
        }
        if(cur_char() == '(') {
            next_char();
            bool result = parse_or();
            if(cur_char() != ')') {
                throw std::logic_error(
                    std::string("Unmatched parenthesis: ")+cur_char()
                );
            }
            next_char();
            return result;
        }
        if((cur_char() == '*')) {
            next_char();
            return (x_ != 0);
        }
        if((cur_char() >= 'A' && cur_char() <= 'Z') || cur_char() == 'x') {
            return parse_variable();
        }
        throw std::logic_error("Syntax error");
    }

    bool BooleanExpression::parse_variable() {
        int bit = 0;
        if(cur_char() >= 'A' && cur_char() <= 'Z') {
            bit = int(cur_char()) - int('A');
            next_char();
        } else {
            if(cur_char() != 'x') {
                throw std::logic_error("Syntax error in variable");
            }
            next_char();
            while(cur_char() >= '0' && cur_char() <= '9') {
                bit = bit * 10 + (int(cur_char()) - '0');
                next_char();
            }
        }
        if(bit > 31) {
            throw std::logic_error("Bit larger than 31");
        }
        return ((x_ & (index_t(1u) << bit)) != 0);
    }

    char BooleanExpression::cur_char() const {
        return (ptr_ == expr_.end()) ? '\0' : *ptr_;
    }
        
    void BooleanExpression::next_char() {
        if(ptr_ == expr_.end()) {
            throw std::logic_error("Unexpected end of string");
        }
        ptr_++;
    }
}
