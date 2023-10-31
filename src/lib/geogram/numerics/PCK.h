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

#ifndef GEOGRAM_NUMERICS_PCK
#define GEOGRAM_NUMERICS_PCK

#include <geogram/basic/common.h>
#include <geogram/basic/numeric.h>
#include <geogram/basic/logger.h>
#include <functional>
#include <algorithm>
#include <atomic>

/**
 * \file geogram/numerics/PCK.h
 * \brief Utilities to write geometric predicates (Predicate Construction Kit).
 */

// Uncomment to get full reporting on predicate statistics
// (but has a non-negligible impact on performance)
// For instance, Early Universe Reconstruction with 2M points:
// with PCK_STATS: 6'36   without PCK_STATS: 3'38

//#define PCK_STATS

namespace GEO {

    namespace PCK {

        /**
         * \brief Logs statistics for predicates. The statistics are
         *  displayed on exit if the command line flag "sys:statistics"
         *  is set. It is used as follows in a predicate:
         * \code
         *   void my_predicate(...) {
         *      static PCK::PredicateStats stats("my_predicate");
         *      my_predicate.log_invoke();
         *      // Filter
         *      {
         *         Sign s = ...;
         *         if(s != ZERO) {
         *            return s;
         *         }
         *      }
         *      // Exact
         *      {
         *         stats.log_exact();
         *         Sign s = ...;
         *         if(s != ZERO) {
         *            return s;
         *         }
         *      }
         *      // SOS
         *      {
         *         stats.log_SOS();
         *         ...
         *      }
         *   }
         * \endcode
         */
        
#ifdef PCK_STATS
        class GEOGRAM_API PredicateStats {
        public:
            PredicateStats(const char* name);
            void log_invoke() {
                ++invoke_count_;
            }
            void log_exact() {
                ++exact_count_;
            }
            void log_SOS() {
                ++SOS_count_;
            }
            void show_stats();
            static void show_all_stats();
        private:
            static PredicateStats* first_;
            PredicateStats* next_;
            const char* name_;
            std::atomic<Numeric::int64> invoke_count_;
            std::atomic<Numeric::int64> exact_count_;
            std::atomic<Numeric::int64> SOS_count_;
        };
#else
        class PredicateStats {
        public:
            PredicateStats(const char* name) {
                geo_argused(name);
            }
            void log_invoke() {
            }
            void log_exact() {
            }
            void log_SOS() {
            }
            static void show_all_stats() {
                Logger::out("Stats") << "Compiled without PCK_STAT (no stats)"
                                     << std::endl;
            }
        };
#endif

        
/**
 * \brief Shorthand for writing lambdas for symbolic perturbations
 * \see SOS()
 */
#define SOS_result(x) [&]()->Sign { return Sign(x); }

        /**
         * \brief template for writing symbolic perturbation in predicates
         * \param[in] compare a comparator for sorting the points
         * \param[in] p1 , p2 , p3 , p4 the four points
         * \param[in] sos_p1 , sos_p2 , sos_p3 , sos_p4 the four lambdas that
         *   compute the symbolic perturbation associated with each point, and
         *   that return the sign of the perturbed predicate. There is a 
         *   SOS_result() macro to help writing these lambdas.
         * \details How to use/example:
         * \code
         * ... beginning of predicate, filter did not hit and exact value 
         * is zero ..
         *
         * return SOS(
         *     vec2HgLexicoCompare<exact_nt>(),
         *        p0, SOS_result( det3_111_sign(p1,p2,p3)),
         *        p1, SOS_result(-det3_111_sign(p0,p2,p3)),
         *        p2, SOS_result( det3_111_sign(p0,p1,p3)),
         *        p3, SOS_result(-det3_111_sign(p0,p1,p2))
         *     );
         * \endcode
         */
        template <
            class POINT, class COMPARE,
            class FUNC1, class FUNC2, class FUNC3, class FUNC4
            > inline Sign SOS(
                COMPARE compare,
                const POINT& p1, FUNC1 sos_p1,
                const POINT& p2, FUNC2 sos_p2,
                const POINT& p3, FUNC3 sos_p3,
                const POINT& p4, FUNC4 sos_p4
            ) {
            static constexpr int N = 4;
            Sign result = ZERO;
            const POINT* p[N] = {&p1, &p2, &p3, &p4};
            std::sort(
                p, p+N,
                [compare](const POINT* A, const POINT* B)->bool{
                    return compare(*A,*B);
                }
            );
            for(int i=0; i<N; ++i) {
                if(p[i] == &p1) {
                    result = sos_p1();
                    if(result != ZERO) {
                        return result;
                    }
                }
                if(p[i] == &p2) {
                    result = sos_p2();
                    if(result != ZERO) {
                        return result;
                    }
                }
                if(p[i] == &p3) {
                    result = sos_p3();
                    if(result != ZERO) {
                        return result;
                    }
                }
                if(p[i] == &p4) {
                    result = sos_p4();
                    if(result != ZERO) {
                        return result;
                    }
                }
            }
            geo_assert_not_reached;
        }

    }
}

#endif
