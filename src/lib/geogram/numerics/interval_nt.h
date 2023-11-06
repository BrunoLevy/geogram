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

#ifndef GEO_INTERVAL_NT
#define GEO_INTERVAL_NT

#include <geogram/numerics/expansion_nt.h>
#include <geogram/basic/vechg.h>
#include <iomanip>
#include <limits>
#include <cmath>

#ifdef __SSE__
#include <xmmintrin.h>
#else
#include <fenv.h>
#endif

// Uncomment to activate checks (keeps an arbitrary precision
// representation of the number and checks that the interval
// contains it).

// #define INTERVAL_CHECK

namespace GEO {

    /**
     * \brief Base class for interval arithmetics.
     * \details Has the verification mechanics 
     *  (define INTERVAL_CHECK to activate it). Verification
     *  stores an additional expansion_nt and checks 
     *  that this expansion_nt is contained by the
     *  interval.
     */
    class intervalBase {
    public:

        static void set_FPU_round_to_nearest() {
#ifdef __SSE__
            _MM_SET_ROUNDING_MODE(_MM_ROUND_NEAREST);
#else
            fesetround(FE_TONEAREST);
#endif
        }

        static void set_FPU_round_to_upper() {
#ifdef __SSE__
            _MM_SET_ROUNDING_MODE(_MM_ROUND_UP);
#else
            fesetround(FE_UPWARD);
#endif
        }
        
        enum Sign2 {
            SIGN2_ERROR = -1,
            SIGN2_ZERO  =  0,
            SIGN2_NP,
            SIGN2_PP,
            SIGN2_ZP,
            SIGN2_NN,
            SIGN2_NZ
	};

        static bool sign_is_determined(Sign2 s) {
            return
                s == SIGN2_ZERO ||
                s == SIGN2_NN   ||
                s == SIGN2_PP   ;
        }

        static bool sign_is_non_zero(Sign2 s) {
            return
                s == SIGN2_NN   ||
                s == SIGN2_PP   ;
        }
        
        static Sign convert_sign(Sign2 s) {
            geo_assert(sign_is_determined(s));
            if(s == SIGN2_NN) {
                return NEGATIVE;
            }
            if(s == SIGN2_PP) {
                return POSITIVE;
            }
            return ZERO;
        }

        intervalBase() {
            control_set(0);
        }

        intervalBase(double x) {
            control_set(x);
        }

        intervalBase(const intervalBase& rhs) = default;

    protected:
#ifdef INTERVAL_CHECK
        void control_set(const expansion_nt& x) {
            control_ = x;
        }
        void control_set(const intervalBase& x) {
            control_ = x.control_;
        }
        void control_set(double x) {
            control_ = expansion_nt(x);
        }
        void control_negate() {
            control_.rep().negate();
        }
        void control_add(const intervalBase& x) {
            control_ += x.control_;
        }
        void control_sub(const intervalBase& x) {
            control_ -= x.control_;
        }
        void control_mul(const intervalBase& x) {
            control_ *= x.control_;
        }
        void control_check(double inf, double sup) {
            typedef std::numeric_limits< double > dbl;
            if(inf > sup) {
                std::cerr.precision(dbl::max_digits10);
                std::cerr << "inf() > sup() !!" << std::endl;
                std::cerr << "inf()=" << inf << std::endl;
                std::cerr << "sup()=" << sup << std::endl;
                geo_assert_not_reached;
            }
            if(control_ < inf || control_ > sup) {
                std::cerr.precision(dbl::max_digits10);
                std::cerr << "[" << inf << "," << sup << "]"
                          << "   " << control_.estimate() << ":"
                          << control_.rep().length()
                          << std::endl;
                geo_assert_not_reached;
            }
        }
        expansion_nt control_; /**< exact represented value, for tests */
#else
        void control_set(double x) {
            geo_argused(x);
        }
        void control_set(const expansion_nt& x) {
            geo_argused(x);
        }
        void control_set(const intervalBase& x) {
            geo_argused(x);            
        }
        void control_negate() {
        }
        void control_add(const intervalBase& x) {
            geo_argused(x);            
        }
        void control_sub(const intervalBase& x) {
            geo_argused(x);            
        }
        void control_mul(const intervalBase& x) {
            geo_argused(x);
        }
        void control_check(double inf, double sup) {
            geo_argused(inf);
            geo_argused(sup);
        }
#endif
    };
    

/*******************************************************************/    

    /**
     * \brief Interval arithmetics in round to upper (RU) mode.
     */
    class intervalRU : public intervalBase {
    public:

        /**
         * \brief Sets FPU rounding mode for using this type of interval.
         * \details Declare one instance at the beginning of a function or
         *  bloc of code that uses intervalRU. Do not mix other floating
         *  point operations in the bloc.
         * \code
         * {
         *    intervalRU::Rounding rounding;
         *    ... do some computations with intervalRU
         * }
         * \endcode
         */
        struct Rounding {
            Rounding() {
                set_FPU_round_to_upper();
            }
            ~Rounding() {
                set_FPU_round_to_nearest();
            }
        };
        
        intervalRU() :
            intervalBase(),
            ln_(0.0),
            u_(0.0)
        {
            control_check();
        }
        
        intervalRU(double x) :
            intervalBase(x),
            ln_(-x),
            u_(x)
        {
            control_check();
        }

        intervalRU(double l, double u) : ln_(-l), u_(u) {
            // note: we cannot control check here.
        }
        
        intervalRU(const intervalRU& rhs) = default;

        intervalRU(const expansion_nt& rhs) {
            *this = rhs;
        }

        intervalRU& operator=(const intervalRU& rhs) = default;
        
        intervalRU& operator=(double rhs) {
            ln_ = -rhs;
            u_ = rhs;
            control_set(rhs);
            control_check();
            return *this;
        }

        intervalRU& operator=(const expansion_nt& rhs) {
            
            // Optimized expansion-to-interval conversion:
            //
            // Add components starting from the one of largest magnitude
            // Stop as soon as next component is smaller than ulp (and then
            // expand interval by ulp).

            index_t l = rhs.length();
            ln_ = -rhs.component(l-1);
            u_ = rhs.component(l-1);

            for(int comp_idx=int(l)-2; comp_idx>=0; --comp_idx) {
                double comp = rhs.component(index_t(comp_idx));
                u_ += comp;
                ln_ -= comp;
                if(comp > 0) {
                    double new_u = u_ + comp;
                    if(new_u == u_) {
                        u_ = std::nextafter(
                            u_, std::numeric_limits<double>::infinity()
                        );
                        break;
                    } else {
                        u_ = new_u;
                    }
                } else {
                    // If we stored l, we would write:
                    //  new_l  =  l  + comp
                    // But we store ln = -l, so we write:
                    // -new_ln = -ln + comp
                    // Which means:
                    //  new_ln =  ln - comp
                    
                    double new_ln = ln_ - comp;
                    if(new_ln == ln_) {
                        ln_ = std::nextafter(
                            ln_, std::numeric_limits<double>::infinity()
                        );
                        break;
                    } else {
                        ln_ = new_ln;
                    }
                }
            }

            control_set(rhs);
            control_check();
            return *this;
        }

        double inf() const {
            return -ln_;
        }
        
        double sup() const {
            return u_; 
        }
        
        double estimate() const {
            // 0.5*(lb+ub) ->
            return 0.5*(-ln_+u_);
        }
        
        bool is_nan() const {
            return !(ln_==ln_) || !(u_==u_);
        }

        Sign2 sign() const {
            // Branchless 
            int lz = int(ln_ == 0);
            int ln = int(ln_ >  0); // inverted, it is ln_ !!!
            int lp = int(ln_ <  0); // inverted, it is ln_ !!!
            int uz = int(u_ ==  0);
            int un = int(u_ <   0);
            int up = int(u_ >   0);
            Sign2 result = Sign2(
                ln*up*SIGN2_NP+
                lp*up*SIGN2_PP+
                lz*up*SIGN2_ZP+
                ln*un*SIGN2_NN+
                ln*uz*SIGN2_NZ
            );
            result = Sign2(
                int(result) +
                int(result==SIGN2_ZERO && !(lz&&uz)) * SIGN2_ERROR
            );
            return result;
        }

        intervalRU& negate() {
            std::swap(ln_, u_);
            control_negate();
            control_check(); 
            return *this;
        }
        
        intervalRU& operator+=(const intervalRU &x) {
            // lb += x.lb -> -lbn += -x.lbn -> lbn += x.lbn
            ln_ += x.ln_;
            u_  += x.u_;
            control_add(x);
            control_check();
            return *this;
        }
        
        intervalRU& operator-=(const intervalRU &x) {
            // +=(x.negate()) ->
            ln_ += x.u_;
            u_  += x.ln_;
            control_sub(x);
            control_check();
            return *this;
        }


        intervalRU& operator*=(const intervalRU &b) {
            // get bounds of both operands
            double aln  = ln_;
            double au   = u_;
            double bln  = b.ln_;
            double bu   = b.u_;

            // negated bounds round to upper
            // (equivalent to bounds round to lower)
            double lln = (-aln)*bln;                
            double lun = aln*bu;
            double uln = au*bln;
            double uun = (-au)*bu;

            // bounds round to upper
            double ll = aln*bln;
            double lu = (-aln)*bu;
            double ul = au*(-bln);
            double uu = au*bu;
            
            ln_ = std::max(std::max(lln,lun),std::max(uln,uun));
            u_  = std::max(std::max(ll,lu),std::max(ul,uu));
            
            control_mul(b);
            control_check();
            return *this;
        }

    protected:
        
#ifdef INTERVAL_CHECK        
        void control_check() {
            // expansion_nt used in control_check() operates
            // in round to nearest mode !!
            set_FPU_round_to_nearest();
            intervalBase::control_check(inf(),sup());
            set_FPU_round_to_upper();
        }
#else
        void control_check() {
        }
#endif
        double ln_;
        double u_;
    };


    inline intervalRU operator+(const intervalRU& a, const intervalRU& b) {
        intervalRU result = a;
        return result += b;
    }

    inline intervalRU operator-(const intervalRU& a, const intervalRU& b) {
        intervalRU result = a;
        return result -= b;
    }

    inline intervalRU operator*(const intervalRU& a, const intervalRU& b) {
        intervalRU result = a;
        return result *= b;
    }

    /*************************************************************************/
    
    /**
     * \brief Number type for interval arithmetics
     * \details Interval class in "round to nearest" mode, by Richard Harris:
     * https://accu.org/journals/overload/19/103/harris_1974/
     * Propagates proportional errors at a rate of 1+/-0.5eps
     * Handles denormals properly (as a special case).
     */
    class intervalRN : public intervalBase {
    public:

        // operates in default rounding mode
        // (so Rounding subclass does nothing)
        struct Rounding {
            Rounding() {
            }
            ~Rounding() {
            }
        };
        
        intervalRN() :
            intervalBase(),
            lb_(0.0),
            ub_(0.0)
        {
            control_check();
        }

        intervalRN(double x) :
            intervalBase(x),
            lb_(x),
            ub_(x)
        {
            control_check();
        }

        intervalRN(double l, double u) : lb_(l), ub_(u) {
            // note: we cannot control check here.
        }

        
        intervalRN(const intervalRN& rhs) = default;

        intervalRN(const expansion_nt& rhs) {
            *this = rhs;
        }

        intervalRN& operator=(const intervalRN& rhs) = default;
        
        intervalRN& operator=(double rhs) {
            lb_ = rhs;
            ub_ = rhs;
            control_set(rhs);
            control_check();
            return *this;
        }

        intervalRN& operator=(const expansion_nt& rhs) {
            
            // Optimized expansion-to-interval conversion:
            //
            // Add components starting from the one of largest magnitude
            // Stop as soon as next component is smaller than ulp (and then
            // expand interval by ulp).
            
            index_t l = rhs.length();
            lb_ = rhs.component(l-1);
            ub_ = rhs.component(l-1);

            for(int comp_idx=int(l)-2; comp_idx>=0; --comp_idx) {
                double comp = rhs.component(index_t(comp_idx));
                if(comp > 0) {
                    double nub = ub_ + comp;
                    if(nub == ub_) {
                        ub_ = std::nextafter(
                            ub_, std::numeric_limits<double>::infinity()
                        );
                        break;
                    } else {
                        ub_ = nub;
                        adjust();
                    }
                } else {
                    double nlb = lb_ + comp;
                    if(nlb == lb_) {
                        lb_ = std::nextafter(
                            lb_, -std::numeric_limits<double>::infinity()
                        );
                        break;
                    } else {
                        lb_ = nlb;
                        adjust();
                    }
                }
            }
            control_set(rhs);
            control_check();
            return *this;
        }
        
        double inf() const {
            return lb_;
        }
        
        double sup() const {
            return ub_; 
        }
        
        double estimate() const {
            return 0.5*(lb_ + ub_);
        }
        
        bool is_nan() const {
            return !(lb_==lb_) || !(ub_==ub_);
        }

        Sign2 sign() const {
            if(is_nan()) {
                return SIGN2_ERROR;
            }
            // Branchless (not sure it is super though...)
            int lz = int(lb_ ==  0);
            int ln = int(lb_ <   0); 
            int lp = int(lb_ >   0); 
            int uz = int(ub_ ==  0);
            int un = int(ub_ <   0);
            int up = int(ub_ >   0);
            Sign2 result = Sign2(
                ln*up*SIGN2_NP+
                lp*up*SIGN2_PP+
                lz*up*SIGN2_ZP+
                ln*un*SIGN2_NN+
                ln*uz*SIGN2_NZ
            );
            result = Sign2(
                int(result) + 
                int(result==SIGN2_ZERO && !(lz&&uz)) * SIGN2_ERROR
            );
            return result;
        }
        
        intervalRN& negate() {
            lb_ = -lb_;
            ub_ = -ub_;
            std::swap(lb_, ub_);
            control_negate();
            control_check();
            return *this;
        }
        
        intervalRN& operator+=(const intervalRN &x) {
            lb_ += x.lb_;
            ub_ += x.ub_;
            adjust();
            control_add(x);
            control_check();
            return *this;
        }
        
        intervalRN& operator-=(const intervalRN &x) {
            lb_ -= x.ub_;
            ub_ -= x.lb_;
            adjust();
            control_sub(x);
            control_check();
            return *this;
        }
        
        intervalRN& operator*=(const intervalRN &x) {
            if(!is_nan() && !x.is_nan()) {
                double ll = lb_*x.lb_;
                double lu = lb_*x.ub_;
                double ul = ub_*x.lb_;
                double uu = ub_*x.ub_;
                
                if(!(ll==ll)) ll = 0.0;
                if(!(lu==lu)) lu = 0.0;
                if(!(ul==ul)) ul = 0.0;
                if(!(uu==uu)) uu = 0.0;

                if(lu<ll) std::swap(lu, ll);
                if(ul<ll) std::swap(ul, ll);
                if(uu<ll) std::swap(uu, ll);

                if(lu>uu) uu = lu;
                if(ul>uu) uu = ul;
                
                lb_ = ll;
                ub_ = uu;
                
                adjust();
            } else {
                lb_ = std::numeric_limits<double>::quiet_NaN();
                ub_ = std::numeric_limits<double>::quiet_NaN();
            }
            control_mul(x);
            control_check();
            return *this;            
        }
        
    protected:
        
        void adjust() {
            static constexpr double i = std::numeric_limits<double>::infinity();
            static constexpr double e = std::numeric_limits<double>::epsilon();
                      // nextafter(1.0) - 1.0
            static constexpr double m = std::numeric_limits<double>::min();
                      // smallest normalized
            static constexpr double l = 1.0-e;
            static constexpr double u = 1.0+e;
            static constexpr double em = e*m;

            if(lb_==lb_ && ub_==ub_ && (lb_!=ub_ || (lb_!=i && lb_!=-i))) {

                if(lb_>ub_) {
                    std::swap(lb_, ub_);
                }

                if(lb_>m) {
                    lb_ *= l;
                } else if(lb_<-m) {
                    lb_ *= u;
                } else {
                    lb_ -= em;
                }

                if(ub_>m) {
                    ub_ *= u;
                } else if(ub_<-m) {
                    ub_ *= l;
                } else {
                    ub_ += em;
                }
            } else {
                lb_ = std::numeric_limits<double>::quiet_NaN();
                ub_ = std::numeric_limits<double>::quiet_NaN();
            }
        }

#ifdef INTERVAL_CHECK        
        void control_check() {
            intervalBase::control_check(inf(),sup());
        }
#else
        void control_check() {
        }
#endif        
        
    private:
        double lb_; /**< lower bound */
        double ub_; /**< upper bound */
    };

    inline intervalRN operator+(const intervalRN& a, const intervalRN& b) {
        intervalRN result = a;
        return result += b;
    }

    inline intervalRN operator-(const intervalRN& a, const intervalRN& b) {
        intervalRN result = a;
        return result -= b;
    }

    inline intervalRN operator*(const intervalRN& a, const intervalRN& b) {
        intervalRN result = a;
        return result *= b;
    }
    
    typedef intervalRN interval_nt; // Seems that valgrind does not support RU
    //typedef intervalRU interval_nt;


    /**
     * \brief Specialization of 2d homogeneous-coord vector difference
     *  for interval_nt coordinates
     * \details We cannot check for p1.w and p2.w equality (since they
     *  might be in the same interval without being equal), so we always
     *  reduce to the same denominator.
     */
    template <> inline vec2Hg<interval_nt> operator-(
        const vec2Hg<interval_nt>& p1, const vec2Hg<interval_nt>& p2
    ) {
        return vec2Hg<interval_nt>(
            det2x2(p1.x,p1.w,p2.x,p2.w),
            det2x2(p1.y,p1.w,p2.y,p2.w),
            p1.w*p2.w
        );
    }


    /**
     * \brief Specialization of 3d homogeneous-coord vector difference
     *  for interval_nt coordinates
     * \details We cannot check for p1.w and p2.w equality (since they
     *  might be in the same interval without being equal), so we always
     *  reduce to the same denominator.
     */
    template <> inline vec3Hg<interval_nt> operator-(
        const vec3Hg<interval_nt>& p1, const vec3Hg<interval_nt>& p2
    ) {
        return vec3Hg<interval_nt>(
            det2x2(p1.x,p1.w,p2.x,p2.w),
            det2x2(p1.y,p1.w,p2.y,p2.w),
            det2x2(p1.z,p1.w,p2.z,p2.w),            
            p1.w * p2.w
        );
    }
    
}
        
#endif
        
