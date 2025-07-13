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

#include <geogram/image/color.h>

namespace {
    using namespace GEO;

    // taken from:
    // https://stackoverflow.com/questions/2353211/hsl-to-rgb-color-conversion

    /**
     * \brief Helper for hsl_to_rgb()
     */
    double hue_to_rgb(double p, double q, double t) {
	if (t < 0.0) t += 1.0;
	if (t > 1.0) t -= 1.0;
	if (t < 1.0 / 6.0) return p + (q - p) * 6.0 * t;
	if (t < 1.0 / 2.0) return q;
	if (t < 2.0 / 3.0) return p + (q - p) * (2.0 / 3.0 - t) * 6.0;
	return p;
    }


    /**
     * \brief Constructs a color from hue,saturation and lightness
     * \param[in] h hue in [0.0,1.0]
     * \param[in] s saturation in [0.0,1.0]
     * \param[in] l lightness in [0.0,1.0]
     */
    Color hsl_to_rgb(double h, double s, double l) {
	double r,g,b;
	if (s == 0.0) {
	    return Color(l,l,l,1.0);
	} else {
	    double q = l < 0.5 ? l * (1.0 + s) : l + s - l * s;
	    double p = 2.0 * l - q;
	    r = hue_to_rgb(p, q, h + 1.0 / 3.0);
	    g = hue_to_rgb(p, q, h);
	    b = hue_to_rgb(p, q, h - 1.0 / 3.0);
	}
	return Color(r,g,b,1.0);
    }
}

namespace GEO {

    Color make_color_from_hsl(double h, double s, double l) {
	return hsl_to_rgb(h,s,l);
    }

    Color make_random_color(
	double min_h, double max_h,
	double min_s, double max_s,
	double min_l, double max_l
    ) {
	double h = Numeric::random_float64() * (max_h - min_h) + min_h;
	double s = Numeric::random_float64() * (max_s - min_s) + min_s;
	double l = Numeric::random_float64() * (max_l - min_l) + min_l;
	return make_color_from_hsl(h,s,l);
    }
}
