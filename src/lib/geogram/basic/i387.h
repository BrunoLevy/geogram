/////////////////////// Put x87 FPU in double-precision mode //////////////////
// taken from:
//        https://gist.github.com/rversteegen/9865ad0dfa734981d3f41ab968d7cf58
//
// For cross-platform portability, force x87 floating-point calculations to be
// done with intermediate results stored in double precision (53 bit mantissa)
// instead of extended double precision (64 bit mantissa) registers.  We change
// the x87 control register to accomplish this.  But it only affects the
// mantissa, not the exponent, so does not remove all inconsistencies.
//
// See http://yosefk.com/blog/consistency-how-to-defeat-the-purpose-of-ieee-floating-point.html
// and http://christian-seiler.de/projekte/fpmath/

#ifndef GEO_I387_H
#define GEO_I387_H

// Indirectly include features.h (for glibc detection), which might not exist
#include <limits.h>

// Check for x86 or amd64, for Visual C++, GCC
#if defined(_M_IX86) || defined(_M_AMD64) || defined(__i386__) || defined(__x86_64__)

#if defined(_MSC_VER) || defined(_WIN32)
// Windows, either Visual C++ or MinGW. Note, Windows defaults to double-precision,
// but MinGW switches on extended precision

#include <float.h>

// Unfortunately at least some MinGW versions (4.8.1) ship with a copy of
// gcc with a float.h which shadows the MinGW float.h header
#ifndef _PC_53
#define	_PC_53		0x00010000
#define	_MCW_PC		0x00030000
_CRTIMP unsigned int __cdecl __MINGW_NOTHROW _controlfp (unsigned int unNew, unsigned int unMask);
#endif

#define GEO_387
inline void geo_disable_387_extended_precision() {
	_controlfp(_PC_53, _MCW_PC);
}

#elif defined(__gnu_linux__) || defined(__GNU_LIBRARY__) || defined(__GLIBC__)
// For glibc

#include <fpu_control.h>

#define GEO_387
inline void geo_disable_387_extended_precision() {
	fpu_control_t cw;
	_FPU_GETCW(cw);
	cw = (fpu_control_t)(cw & ~_FPU_EXTENDED) | _FPU_DOUBLE;
	_FPU_SETCW(cw);
}

#endif
#endif


#endif // GEO_387_H
