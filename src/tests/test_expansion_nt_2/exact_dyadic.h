#ifndef EXACT_DYADIC_H
#define EXACT_DYADIC_H

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

#include <boost/multiprecision/cpp_int.hpp>


struct ExactDyadic {
    typedef boost::multiprecision::cpp_int bigint;

    bigint mantissa = 0;
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
            mantissa = bigint(fraction);
            exponent = -1074;
        } else {
            // normal:
            // value = sign * significand * 2^(biased-1023-52)
            const std::uint64_t significand =
                (std::uint64_t(1) << 52) | fraction;
            mantissa = bigint(significand);
            exponent =
                static_cast<int>(biased_exponent) - 1023 - 52;
        }

        if (negative) {
            mantissa = -mantissa;
        }
    }

    ExactDyadic(bigint m, int e)
        : mantissa(std::move(m)), exponent(e) {}

    bool is_zero() const {
        return mantissa == 0;
    }
};

inline ExactDyadic operator+(const ExactDyadic& a, const ExactDyadic& b) {
    if (a.is_zero()) return b;
    if (b.is_zero()) return a;

    const int e = (a.exponent < b.exponent) ? a.exponent : b.exponent;

    ExactDyadic::bigint am = a.mantissa;
    ExactDyadic::bigint bm = b.mantissa;

    am <<= static_cast<unsigned>(a.exponent - e);
    bm <<= static_cast<unsigned>(b.exponent - e);

    return ExactDyadic(am + bm, e);
}

inline ExactDyadic operator-(const ExactDyadic& a, const ExactDyadic& b) {
    return ExactDyadic(a.mantissa, a.exponent) +
           ExactDyadic(-b.mantissa, b.exponent);
}

inline ExactDyadic operator*(const ExactDyadic& a, const ExactDyadic& b) {
    return ExactDyadic(
        a.mantissa * b.mantissa,
        a.exponent + b.exponent);
}

inline bool exact_equal(const ExactDyadic& a, const ExactDyadic& b) {
    return (a - b).is_zero();
}

inline bool is_zero(const ExactDyadic& a) {
    return a.is_zero();
}

// Convert dyadic to a canonical exact fraction string for inspection.
// This is diagnostic output only.
inline std::string as_string(ExactDyadic x) {
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
	ExactDyadic::bigint integer = x.mantissa;
        integer <<= static_cast<unsigned>(x.exponent);
        out << integer;
    } else {
        ExactDyadic::bigint denominator(1);
        denominator <<= static_cast<unsigned>(-x.exponent);
        out << x.mantissa << " / " << denominator;
    }
    return out.str();
}



#endif
