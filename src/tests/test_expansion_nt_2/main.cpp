#include <geogram/numerics/expansion_nt.h>
#include <boost/multiprecision/cpp_int.hpp>

#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

// ============================================================================
// Geogram path
// ============================================================================

using E = GEO::expansion_nt;

struct GV3 {
    E x;
    E y;
    E z;
};

GV3 g_between(const double* to, const double* from) {
    return {
        E(to[0]) - E(from[0]),
        E(to[1]) - E(from[1]),
        E(to[2]) - E(from[2])
    };
}

GV3 g_cross(const GV3& a, const GV3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

E g_dot(const GV3& a, const GV3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

E g_det(const GV3& a, const GV3& b, const GV3& c) {
    return g_dot(g_cross(a, b), c);
}

bool g_is_zero(const E& x) {
    return x.sign() == GEO::ZERO;
}

// ============================================================================
// Independent Boost exact-dyadic path
//
// ExactDyadic stores exactly:
//     value = mantissa * 2^exponent
//
// mantissa is an arbitrary-precision integer (boost::multiprecision::cpp_int).
// Every IEEE-754 binary64 input is imported exactly from its bit pattern.
// Only exact integer shifts/add/subtract/multiply are used afterwards.
// ============================================================================

using boost::multiprecision::cpp_int;

struct ExactDyadic {
    cpp_int mantissa = 0;
    int exponent = 0;

    ExactDyadic() = default;

    explicit ExactDyadic(double value) {
        std::uint64_t bits = 0;
        static_assert(sizeof(bits) == sizeof(value), "unexpected double size");
        std::memcpy(&bits, &value, sizeof(bits));

        const bool negative = ((bits >> 63) != 0);
        const std::uint64_t fraction =
            bits & ((std::uint64_t(1) << 52) - 1);
        const unsigned biased_exponent =
            static_cast<unsigned>((bits >> 52) & 0x7ffu);

        if (biased_exponent == 0x7ffu) {
            throw std::runtime_error(
                "NaN/Inf not supported by this diagnostic");
        }

        if (biased_exponent == 0u) {
            // zero or subnormal
            if (fraction == 0u) {
                mantissa = 0;
                exponent = 0;
                return;
            }
            mantissa = cpp_int(fraction);
            exponent = -1074;
        } else {
            // normal:
            // value = sign * significand * 2^(biased-1023-52)
            const std::uint64_t significand =
                (std::uint64_t(1) << 52) | fraction;
            mantissa = cpp_int(significand);
            exponent =
                static_cast<int>(biased_exponent) - 1023 - 52;
        }

        if (negative) {
            mantissa = -mantissa;
        }
    }

    ExactDyadic(cpp_int m, int e)
        : mantissa(std::move(m)), exponent(e) {}

    bool is_zero() const {
        return mantissa == 0;
    }
};

ExactDyadic operator+(const ExactDyadic& a, const ExactDyadic& b) {
    if (a.is_zero()) return b;
    if (b.is_zero()) return a;

    const int e = (a.exponent < b.exponent) ? a.exponent : b.exponent;

    cpp_int am = a.mantissa;
    cpp_int bm = b.mantissa;

    am <<= static_cast<unsigned>(a.exponent - e);
    bm <<= static_cast<unsigned>(b.exponent - e);

    return ExactDyadic(am + bm, e);
}

ExactDyadic operator-(const ExactDyadic& a, const ExactDyadic& b) {
    return ExactDyadic(a.mantissa, a.exponent) +
           ExactDyadic(-b.mantissa, b.exponent);
}

ExactDyadic operator*(const ExactDyadic& a, const ExactDyadic& b) {
    return ExactDyadic(
        a.mantissa * b.mantissa,
        a.exponent + b.exponent);
}

bool exact_equal(const ExactDyadic& a, const ExactDyadic& b) {
    return (a - b).is_zero();
}

struct BV3 {
    ExactDyadic x;
    ExactDyadic y;
    ExactDyadic z;
};

BV3 b_between(const double* to, const double* from) {
    return {
        ExactDyadic(to[0]) - ExactDyadic(from[0]),
        ExactDyadic(to[1]) - ExactDyadic(from[1]),
        ExactDyadic(to[2]) - ExactDyadic(from[2])
    };
}

BV3 b_cross(const BV3& a, const BV3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

ExactDyadic b_dot(const BV3& a, const BV3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

ExactDyadic b_det(const BV3& a, const BV3& b, const BV3& c) {
    return b_dot(b_cross(a, b), c);
}

bool b_vector_zero(const BV3& v) {
    return v.x.is_zero() && v.y.is_zero() && v.z.is_zero();
}

// Convert dyadic to a canonical exact fraction string for inspection.
// This is diagnostic output only.
std::string exact_fraction_string(ExactDyadic x) {
    if (x.mantissa == 0) {
        return "0";
    }

    // Remove powers of two from the integer mantissa so the fraction is reduced.
    while ((x.mantissa % 2) == 0) {
        x.mantissa /= 2;
        ++x.exponent;
    }

    std::ostringstream out;
    if (x.exponent >= 0) {
        cpp_int integer = x.mantissa;
        integer <<= static_cast<unsigned>(x.exponent);
        out << integer;
    } else {
        cpp_int denominator = cpp_int(1);
        denominator <<= static_cast<unsigned>(-x.exponent);
        out << x.mantissa << " / " << denominator;
    }
    return out.str();
}

} // namespace

int main() {
    // Exact binary64 values from the failing case.
    // Hexadecimal floating-point literals make the input bits unambiguous.
    const double a[3] = {
        0x1.e2c6881fd8c00p+11,
        0x1.158b9a0c8aa00p+13,
        0x1.bfd5653a0d2b8p+7
    };

    const double b[3] = {
        0x1.e3bcbe9e1e900p+11,
        0x1.1585e8af2ee00p+13,
        0x1.bafad3920f9b0p+7
    };

    const double c0[3] = {
        0x1.e2d40af9c8a00p+11,
        0x1.1581b8f842200p+13,
        0x1.bea6561bf7620p+7
    };

    // c1 == b exactly.
    const double c1[3] = {
        0x1.e3bcbe9e1e900p+11,
        0x1.1585e8af2ee00p+13,
        0x1.bafad3920f9b0p+7
    };

    std::cout << std::boolalpha << std::setprecision(17);

    // ------------------------------------------------------------------------
    // Geogram expansion_nt
    // ------------------------------------------------------------------------
    const GV3 gu = g_between(b, a);
    const GV3 gh = g_between(c1, c0);
    const GV3 gr = g_between(c0, a);
    const GV3 gg{E(1.0), E(2.0), E(3.0)};

    const E gclosure_x = gu.x - gr.x - gh.x;
    const E gclosure_y = gu.y - gr.y - gh.y;
    const E gclosure_z = gu.z - gr.z - gh.z;

    const E gD  = g_det(gu, gh, gg);
    const E gNs = g_det(gr, gh, gg);
    const E gNt = g_det(gr, gu, gg);

    const E gD_minus_Ns = gD - gNs;
    const E gD_minus_Nt = gD - gNt;

    std::cout << "=== Geogram expansion_nt ===\n";
    std::cout << "u_equals_r_plus_h = "
              << (g_is_zero(gclosure_x) &&
                  g_is_zero(gclosure_y) &&
                  g_is_zero(gclosure_z))
              << '\n';

    std::cout << "D  estimate = " << gD.estimate()  << '\n';
    std::cout << "Ns estimate = " << gNs.estimate() << '\n';
    std::cout << "Nt estimate = " << gNt.estimate() << '\n';

    std::cout << "D_equals_Ns = " << g_is_zero(gD_minus_Ns) << '\n';
    std::cout << "D_equals_Nt = " << g_is_zero(gD_minus_Nt) << '\n';

    std::cout << "sign(D-Ns) = "
              << static_cast<int>(gD_minus_Ns.sign()) << '\n';
    std::cout << "sign(D-Nt) = "
              << static_cast<int>(gD_minus_Nt.sign()) << '\n';

    // ------------------------------------------------------------------------
    // Independent Boost exact dyadic
    // ------------------------------------------------------------------------
    const BV3 bu = b_between(b, a);
    const BV3 bh = b_between(c1, c0);
    const BV3 br = b_between(c0, a);
    const BV3 bg{
        ExactDyadic(1.0),
        ExactDyadic(2.0),
        ExactDyadic(3.0)
    };

    const BV3 bclosure{
        bu.x - br.x - bh.x,
        bu.y - br.y - bh.y,
        bu.z - br.z - bh.z
    };

    const ExactDyadic bD  = b_det(bu, bh, bg);
    const ExactDyadic bNs = b_det(br, bh, bg);
    const ExactDyadic bNt = b_det(br, bu, bg);

    std::cout << "\n=== Boost cpp_int exact dyadic ===\n";
    std::cout << "u_equals_r_plus_h = "
              << b_vector_zero(bclosure) << '\n';

    std::cout << "D_equals_Ns = " << exact_equal(bD, bNs) << '\n';
    std::cout << "D_equals_Nt = " << exact_equal(bD, bNt) << '\n';

    std::cout << "D_exact  = " << exact_fraction_string(bD)  << '\n';
    std::cout << "Ns_exact = " << exact_fraction_string(bNs) << '\n';
    std::cout << "Nt_exact = " << exact_fraction_string(bNt) << '\n';

    return 0;
}
