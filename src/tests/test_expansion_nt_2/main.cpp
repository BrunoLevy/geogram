#include <geogram/numerics/expansion_nt.h>
#include <geogram/numerics/exact_geometry.h>
#include <geogram/basic/geometry.h>
#include <geogram/basic/string.h>

#ifdef __clang__
#pragma GCC diagnostic ignored "-Wpre-c++17-compat-pedantic"
#endif

// #define WITH_BOOST

#ifdef WITH_BOOST
#include "exact_dyadic.h"
#endif

#ifdef GEOGRAM_WITH_GEOGRAMPLUS
#include <geogram/geogramplus/numerics/exact_nt.h>
inline bool is_zero(const GEO::exact_nt& x) {
    return (x.sign() == GEO::ZERO);
}
inline std::string as_string(const GEO::exact_nt& x) {
    return GEO::String::format("%f",x.estimate());
}
#endif

#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

using namespace GEO;

inline bool is_zero(const expansion_nt& E) {
    return (E.sign() == ZERO);
}

inline std::string as_string(const expansion_nt& E) {
    return String::format("%f",E.estimate());
}

template <class NT> inline NT det(
    const vecng<3,NT>& v1,
    const vecng<3,NT>& v2,
    const vecng<3,NT>& v3
) {
    return dot(cross(v1,v2),v3);
}

template <class NT> inline bool test_arithmetics(const std::string& tag) {
    typedef vecng<3,NT> exact_vec3;

    // Exact binary64 values from the failing case.
    // Hexadecimal floating-point literals make the input bits unambiguous.
    const vec3 a{
        0x1.e2c6881fd8c00p+11,
        0x1.158b9a0c8aa00p+13,
        0x1.bfd5653a0d2b8p+7
    };

    const vec3 b{
        0x1.e3bcbe9e1e900p+11,
        0x1.1585e8af2ee00p+13,
        0x1.bafad3920f9b0p+7
    };

    const vec3 c0{
        0x1.e2d40af9c8a00p+11,
        0x1.1581b8f842200p+13,
        0x1.bea6561bf7620p+7
    };

    // c1 == b exactly.
    const vec3 c1{
        0x1.e3bcbe9e1e900p+11,
        0x1.1585e8af2ee00p+13,
        0x1.bafad3920f9b0p+7
    };

    std::cout << std::boolalpha << std::setprecision(17);

    const exact_vec3 gu = make_vec3<exact_vec3>(a, b);
    const exact_vec3 gh = make_vec3<exact_vec3>(c0, c1);
    const exact_vec3 gr = make_vec3<exact_vec3>(a, c0);
    const exact_vec3 gg{NT(1.0), NT(2.0), NT(3.0)};

    const NT gclosure_x = gu.x - gr.x - gh.x;
    const NT gclosure_y = gu.y - gr.y - gh.y;
    const NT gclosure_z = gu.z - gr.z - gh.z;

    const NT gD  = det(gu, gh, gg);
    const NT gNs = det(gr, gh, gg);
    const NT gNt = det(gr, gu, gg);

    const NT gD_minus_Ns = gD - gNs;
    const NT gD_minus_Nt = gD - gNt;

    std::cout << "=== " << tag << " ===" << std::endl;
    std::cout << "u_equals_r_plus_h = "
              << (is_zero(gclosure_x) &&
                  is_zero(gclosure_y) &&
                  is_zero(gclosure_z))
              << '\n';

    std::cout << "D  estimate = " << as_string(gD)  << '\n';
    std::cout << "Ns estimate = " << as_string(gNs) << '\n';
    std::cout << "Nt estimate = " << as_string(gNt) << '\n';

    std::cout << "D_equals_Ns = " << is_zero(gD_minus_Ns) << '\n';
    std::cout << "D_equals_Nt = " << is_zero(gD_minus_Nt) << '\n';
    std::cout << std::endl;

    return
	is_zero(gclosure_x) &&
	is_zero(gclosure_y) &&
	is_zero(gclosure_z) &&
	is_zero(gD_minus_Ns) &&
	is_zero(gD_minus_Nt) ;
}

int main() {
    //   This function needs to be called before
    // using expansion_nt.
    GEO::expansion::initialize();

    bool OK = true;
    OK = OK && test_arithmetics<expansion_nt>("geogram expansion_nt");
#ifdef WITH_BOOST
    OK = OK && test_arithmetics<ExactDyadic>("ExactDyadic (boost bigint)");
#endif
#ifdef GEOGRAM_WITH_GEOGRAMPLUS
    OK = OK && test_arithmetics<exact_nt>("geogram exact_nt");
#endif
    return OK ? 0 : -1;
}
