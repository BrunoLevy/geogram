// Copyright (c) 1998-2008
// Utrecht University (The Netherlands),
// ETH Zurich (Switzerland),
// INRIA Sophia-Antipolis (France),
// Max-Planck-Institute Saarbruecken (Germany),
// and Tel-Aviv University (Israel).  All rights reserved.
//
// This file is part of CGAL (www.cgal.org); you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License as
// published by the Free Software Foundation; either version 3 of the License,
// or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL$
// $Id$
//
//
// Author(s)     : Sylvain Pion

#ifndef CGAL_FPU_H
#define CGAL_FPU_H

#include <CGALmini/basic.h>

#ifndef __INTEL_COMPILER
#include <cmath> // for HUGE_VAL
#endif

// This file specifies some platform dependant functions, regarding the FPU
// directed rounding modes.  There is only support for double precision.
//
// It also contains the definition of the Protect_FPU_rounding<> class,
// which helps to protect blocks of code needing a particular rounding mode.

#if defined __alpha__  && defined __linux__
extern "C" {
#  include <fenv.h>
}
#elif defined __powerpc__ && defined __linux__
#  include <fpu_control.h>
#elif defined __SUNPRO_CC && defined __sun
#  include <ieeefp.h>
#elif defined __osf || defined __osf__
#  ifdef __GNUG__
     // GCC seems to remove (fixincludes) read_rnd/write_rnd...
#    include "/usr/include/float.h"
#  else
#    include <cfloat>
#  endif
#elif defined _MSC_VER || defined __sparc__ || \
     (defined __i386__ && !defined __PGI && !defined __SUNPRO_CC)
   // Nothing to include.
#else
   // By default we use the ISO C99 version.
#  include <fenv.h>
#endif


// Some useful constants

#if defined CGAL_CFG_NO_LIMITS
#  if defined CGAL_CFG_DENORMALS_COMPILE_BUG
     // For compilers crashing when dealing with denormalized values.
     // So we have to generate it at run time instead.
#    define CGAL_IA_MIN_DOUBLE (CGAL::internal::minimin)
#  else
#    define CGAL_IA_MIN_DOUBLE (5e-324)
#  endif
#  define CGAL_IA_MAX_DOUBLE (1.7976931348623157081e+308)
#else
#  include <limits>
#  define CGAL_IA_MIN_DOUBLE std::numeric_limits<double>::denorm_min()
#  define CGAL_IA_MAX_DOUBLE (std::numeric_limits<double>::max)()
#endif


// Pure and safe SSE2 mode (g++ -mfpmath=sse && (-msse2 || -march=pentium4))
// can be detected by :
// TODO : see what Intel and VC++ have to say about this.
#if defined __FLT_EVAL_METHOD__ && defined __SSE2_MATH__ && \
      (__FLT_EVAL_METHOD__ == 0 || __FLT_EVAL_METHOD__ == 1)
#  define CGAL_SAFE_SSE2
#  include <xmmintrin.h>
#endif

// We do not handle -mfpmath=387,sse yet.
#if defined __SSE2_MATH__ && \
    ! (__FLT_EVAL_METHOD__ == 0 || __FLT_EVAL_METHOD__ == 1)
#  warning Unsafe SSE2 mode : not supported yet.
#endif

// The CGAL_FPU_HAS_EXCESS_PRECISION macro is defined if some computations with
// double can use more than the 53bits of precision of IEEE754, and/or if the
// exponent has a wider range.  This can produce double rounding effects and
// other bad things that we need to protect against.
// The typical offender is the traditional FPU of x86 (SSE2-only mode is not affected).
// Are there others?
#if (defined __i386__ && !defined CGAL_SAFE_SSE2) || defined __ia64__ || defined _MSC_VER
#  define CGAL_FPU_HAS_EXCESS_PRECISION
#endif

namespace CGAL {

namespace internal {

#ifdef CGAL_CFG_DENORMALS_COMPILE_BUG
CGAL_EXPORT extern double minimin;
#endif

#ifdef __INTEL_COMPILER
const double infinity = std::numeric_limits<double>::infinity();
#else
const double infinity = HUGE_VAL;
#endif

} // namespace internal


// Inline function to stop compiler optimization.
inline double IA_force_to_double(double x)
{
#if defined __GNUG__ && !defined __INTEL_COMPILER
  // Intel does not emulate GCC perfectly...
  // Is that still true? -- Marc Glisse, 2012-12-17
#  ifdef CGAL_SAFE_SSE2
  // For an explanation of volatile:
  // http://gcc.gnu.org/bugzilla/show_bug.cgi?id=56027
  asm volatile ("" : "+mx"(x) );
#  else
  asm volatile ("" : "=m"(x) : "m"(x));
  // asm("" : "+m"(x) );
#  endif
  return x;
#else
  volatile double e = x;
  return e;
#endif
}

// Interval arithmetic needs to protect against double-rounding effects
// caused by excess FPU precision, even if it forces the 53bit mantissa
// precision, because there is no way to fix the problem for the exponent
// which has the same problem.  This affects underflow and overflow cases.
// In case one does not care about such "extreme" situations, one can
// set CGAL_IA_NO_X86_OVER_UNDER_FLOW_PROTECT.
// LLVM doesn't have -frounding-math so needs extra protection.
#if (defined CGAL_FPU_HAS_EXCESS_PRECISION && \
   !defined CGAL_IA_NO_X86_OVER_UNDER_FLOW_PROTECT) || defined __llvm__
#  define CGAL_IA_FORCE_TO_DOUBLE(x) CGAL::IA_force_to_double(x)
#else
#  define CGAL_IA_FORCE_TO_DOUBLE(x) (x)
#endif

// We sometimes need to stop constant propagation,
// because operations are done with a wrong rounding mode at compile time.
#ifndef CGAL_IA_DONT_STOP_CONSTANT_PROPAGATION
#  define CGAL_IA_STOP_CPROP(x)    CGAL::IA_force_to_double(x)
#else
#  define CGAL_IA_STOP_CPROP(x)    (x)
#endif

// std::sqrt(double) on VC++ and CygWin is buggy when not optimizing.
#if defined ( _MSC_VER ) && ! defined ( _WIN64 )
inline double IA_bug_sqrt(double d)
{
  _asm
  {
    fld d
    fsqrt
    fstp d
  }
  return d;
}

#  define CGAL_BUG_SQRT(d) IA_bug_sqrt(d)


#elif defined __SSE2_MATH__
// For SSE2, we need to call __builtin_sqrt() instead of libc's sqrt().
#  define CGAL_BUG_SQRT(d) __builtin_sqrt(d)
#elif defined __CYGWIN__
inline double IA_bug_sqrt(double d)
{
  double r;
  asm volatile ("fsqrt" : "=t"(r) : "0"(d));
  return r;
}
#  define CGAL_BUG_SQRT(d) IA_bug_sqrt(d)
#else
#  define CGAL_BUG_SQRT(d) std::sqrt(d)
#endif

// Here are the operator macros that make use of the above.
// With GCC, we can do slightly better : test with __builtin_constant_p()
// that both arguments are constant before stopping one of them.
// Use inline functions instead ?
#define CGAL_IA_ADD(a,b) CGAL_IA_FORCE_TO_DOUBLE((a)+CGAL_IA_STOP_CPROP(b))
#define CGAL_IA_SUB(a,b) CGAL_IA_FORCE_TO_DOUBLE((a)-CGAL_IA_STOP_CPROP(b))
#define CGAL_IA_MUL(a,b) CGAL_IA_FORCE_TO_DOUBLE((a)*CGAL_IA_STOP_CPROP(b))
#define CGAL_IA_DIV(a,b) CGAL_IA_FORCE_TO_DOUBLE((a)/CGAL_IA_STOP_CPROP(b))
#define CGAL_IA_SQUARE(a) CGAL_IA_MUL(a,a)
#define CGAL_IA_SQRT(a) \
        CGAL_IA_FORCE_TO_DOUBLE(CGAL_BUG_SQRT(CGAL_IA_STOP_CPROP(a)))


#if defined __i386__ && !defined __PGI && !defined __SUNPRO_CC

#  if defined CGAL_SAFE_SSE2

#define CGAL_IA_SETFPCW(CW) _MM_SET_ROUNDING_MODE(CW)
#define CGAL_IA_GETFPCW(CW) CW = _MM_GET_ROUNDING_MODE()
typedef unsigned int FPU_CW_t;
#define CGAL_FE_TONEAREST    _MM_ROUND_NEAREST
#define CGAL_FE_TOWARDZERO   _MM_ROUND_TOWARD_ZERO
#define CGAL_FE_UPWARD       _MM_ROUND_UP
#define CGAL_FE_DOWNWARD     _MM_ROUND_DOWN

#  else
// The GNU libc version (cf powerpc) is nicer, but doesn't work on libc 5 :(
// This one also works with CygWin.
// Note that the ISO C99 version may not be enough because of the extended
// mantissa issue on x86 (may be required by some kinds of computation, but
// as far as CGAL::Interval_nt<> is concerned, the double-rounding issues
// are taking care of there).
#define CGAL_IA_SETFPCW(CW) asm volatile ("fldcw %0" : :"m" (CW))
#define CGAL_IA_GETFPCW(CW) asm volatile ("fnstcw %0" : "=m" (CW))
typedef unsigned short FPU_CW_t;
#define CGAL_FE_TONEAREST    (0x000 | 0x127f)
#define CGAL_FE_TOWARDZERO   (0xc00 | 0x127f)
#define CGAL_FE_UPWARD       (0x800 | 0x127f)
#define CGAL_FE_DOWNWARD     (0x400 | 0x127f)

#  endif

#elif defined __powerpc__ && defined __linux__
#define CGAL_IA_SETFPCW(CW) _FPU_SETCW(CW)
#define CGAL_IA_GETFPCW(CW) _FPU_GETCW(CW)
typedef fpu_control_t FPU_CW_t;
#define CGAL_FE_TONEAREST    (_FPU_RC_NEAREST | _FPU_DEFAULT)
#define CGAL_FE_TOWARDZERO   (_FPU_RC_ZERO    | _FPU_DEFAULT)
#define CGAL_FE_UPWARD       (_FPU_RC_UP      | _FPU_DEFAULT)
#define CGAL_FE_DOWNWARD     (_FPU_RC_DOWN    | _FPU_DEFAULT)

#elif defined __SUNPRO_CC && defined __sun
#define CGAL_IA_SETFPCW(CW) fpsetround(fp_rnd(CW))
#define CGAL_IA_GETFPCW(CW) CW = fpgetround()
typedef unsigned int FPU_CW_t;
#define CGAL_FE_TONEAREST    FP_RN
#define CGAL_FE_TOWARDZERO   FP_RZ
#define CGAL_FE_UPWARD       FP_RP
#define CGAL_FE_DOWNWARD     FP_RM

#elif defined __sparc__
#define CGAL_IA_SETFPCW(CW) asm volatile ("ld %0,%%fsr" : :"m" (CW))
#define CGAL_IA_GETFPCW(CW) asm volatile ("st %%fsr,%0" : "=m" (CW))
typedef unsigned int FPU_CW_t;
#define CGAL_FE_TONEAREST    (0x0        | 0x20000000 | 0x1f)
#define CGAL_FE_TOWARDZERO   (0x40000000 | 0x20000000 | 0x1f)
#define CGAL_FE_UPWARD       (0x80000000 | 0x20000000 | 0x1f)
#define CGAL_FE_DOWNWARD     (0xc0000000 | 0x20000000 | 0x1f)

#elif defined __mips__
#define CGAL_IA_SETFPCW(CW) asm volatile ("ctc1 %0,$31" : :"r" (CW))
#define CGAL_IA_GETFPCW(CW) asm volatile ("cfc1 %0,$31" : "=r" (CW))
typedef unsigned int FPU_CW_t;
#define CGAL_FE_TONEAREST    (0x0)
#define CGAL_FE_TOWARDZERO   (0x1)
#define CGAL_FE_UPWARD       (0x2)
#define CGAL_FE_DOWNWARD     (0x3)

#elif defined __osf || defined __osf__  // Not yet supported.
#define CGAL_IA_SETFPCW(CW) write_rnd(CW)
#define CGAL_IA_GETFPCW(CW) CW = read_rnd()
typedef unsigned int FPU_CW_t;
#define CGAL_FE_TONEAREST    FP_RND_RN
#define CGAL_FE_TOWARDZERO   FP_RND_RZ
#define CGAL_FE_UPWARD       FP_RND_RP
#define CGAL_FE_DOWNWARD     FP_RND_RM

#elif defined ( _MSC_VER )
#if ( _MSC_VER < 1400)
#define CGAL_IA_SETFPCW(CW) _controlfp (CW, _MCW_RC )
#define CGAL_IA_GETFPCW(CW) CW = _controlfp (0, 0 ) &  _MCW_RC
typedef unsigned short FPU_CW_t;
#else
#define CGAL_IA_SETFPCW(CW) unsigned int dummy; _controlfp_s (&dummy, CW, _MCW_RC )
#define CGAL_IA_GETFPCW(CW)_controlfp_s (&CW, 0, 0 ); CW  &=  _MCW_RC
typedef unsigned int FPU_CW_t;
#endif

#define CGAL_FE_TONEAREST    _RC_NEAR
#define CGAL_FE_TOWARDZERO   _RC_CHOP
#define CGAL_FE_UPWARD       _RC_UP
#define CGAL_FE_DOWNWARD     _RC_DOWN

#else
// This is a version following the ISO C99 standard, which aims at portability.
// The drawbacks are speed on one hand, and also, on x86, it doesn't fix the
// extended mantissa issue (this is not a problem for IA, but it is one for
// some future modular computations).
#define CGAL_IA_SETFPCW(CW)  fesetround(CW)
#define CGAL_IA_GETFPCW(CW)  CW = fegetround()
typedef int FPU_CW_t;
#define CGAL_FE_TONEAREST    FE_TONEAREST
#define CGAL_FE_TOWARDZERO   FE_TOWARDZERO
#define CGAL_FE_UPWARD       FE_UPWARD
#define CGAL_FE_DOWNWARD     FE_DOWNWARD
#endif

// User interface:

inline
FPU_CW_t
FPU_get_cw (void)
{
    FPU_CW_t cw;
    CGAL_IA_GETFPCW(cw);
    return cw;
}

inline
void
FPU_set_cw (FPU_CW_t cw)
{
    CGAL_IA_SETFPCW(cw);
}

inline
FPU_CW_t
FPU_get_and_set_cw (FPU_CW_t cw)
{
    FPU_CW_t old = FPU_get_cw();
    FPU_set_cw(cw);
    return old;
}


// A class whose constructor sets the FPU mode to +inf, saves a backup of it,
// and whose destructor resets it back to the saved state.

template <bool Protected = true> struct Protect_FPU_rounding;

template <>
struct Protect_FPU_rounding<true>
{
  Protect_FPU_rounding(FPU_CW_t r = CGAL_FE_UPWARD)
    : backup( FPU_get_and_set_cw(r) ) {}

  ~Protect_FPU_rounding()
  {
     FPU_set_cw(backup);
  }

private:
  FPU_CW_t backup;
};

template <>
struct Protect_FPU_rounding<false>
{
  Protect_FPU_rounding() {}
  Protect_FPU_rounding(FPU_CW_t /*= CGAL_FE_UPWARD*/) {}
};


// A wrapper on top of the Protect_FPU_rounding to add "expensive" checks
// of the rounding mode.  It is used internally, to benefit from the
// protector declarations to add checks in non-protected mode.

template <bool Protected = true>
struct Checked_protect_FPU_rounding
  : Protect_FPU_rounding<Protected>
{
  Checked_protect_FPU_rounding()
  {
    CGAL_expensive_assertion(FPU_get_cw() == CGAL_FE_UPWARD);
  }

  Checked_protect_FPU_rounding(FPU_CW_t r)
    : Protect_FPU_rounding<Protected>(r)
  {
    CGAL_expensive_assertion(FPU_get_cw() == CGAL_FE_UPWARD);
  }
};


// The class Set_ieee_double_precision forces the double precision (53bit mantissa),
// to protect from double rounding effects on x86 FPU.
// ( Note that it also sets the rounding mode to nearest. )
// Its destructor restores the FPU state as it was previously.
// Note that this affects "long double" as well, and other potential side effects.
// And note that it does not (cannot) "fix" the same problem for the exponent.

struct Set_ieee_double_precision
#ifdef CGAL_FPU_HAS_EXCESS_PRECISION
  : public Protect_FPU_rounding<>
{
  Set_ieee_double_precision()
    : Protect_FPU_rounding<>(CGAL_FE_TONEAREST) {}
};
#else
{
  Set_ieee_double_precision() {} // only to kill compiler warnings.
};
#endif


// The following function serves the same goal as Set_ieee_double_precision but
// does the change globally (no destructor resets the previous behavior).
inline void force_ieee_double_precision()
{
#ifdef CGAL_FPU_HAS_EXCESS_PRECISION
    FPU_set_cw(CGAL_FE_TONEAREST);
#endif
}

} //namespace CGAL

#endif // CGAL_FPU_H
