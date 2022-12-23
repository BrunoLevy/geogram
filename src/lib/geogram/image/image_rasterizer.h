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
 
#ifndef H_OGF_IMAGE_IO_IMAGE_RASTERIZER_H
#define H_OGF_IMAGE_IO_IMAGE_RASTERIZER_H

#include <geogram/basic/common.h>
#include <geogram/image/image.h>

/**
 * \file geogram/image/image_serializer.h
 * \brief Class to draw triangles in an image.
 */

namespace GEO {

    /**
     * \brief Draws triangles in an image.
     */
    class GEOGRAM_API ImageRasterizer {
      public:

	/**
	 * \brief ImageRasterizer constructor.
	 * \param[in] image a pointer to the target image.
	 * \param x1 , y1 , x2 , y2 viewport coordinates.
	 */
        ImageRasterizer(
	    Image* image,
	    double x1=0.0, double y1=0.0,
	    double x2=1.0, double y2=1.0
	); 

	/**
	 * \brief Clears the target image.
	 */
	void clear();

	/**
	 * \brief Draws a triangle in the target image.
	 * \details Colors are linearly interpolated (Gouraud shading).
	 * \param[in] p1 , p2 , p3 the three vertices of the triangle.
	 * \param[in] c1 , c2 , c3 the three colors of the vertices.
	 */
	void triangle(
	    const vec2& p1, const Color& c1, 
	    const vec2& p2, const Color& c2, 
	    const vec2& p3, const Color& c3
	);

        /**
         * \brief Draws a segment in the target image.
         * \param[in] p1, p2 the two extremities of the segment
         * \param[in] c the color
         */
        void segment(
            const vec2& p1, const vec2& p2,
            const Color& c
        );

        /**
         * \brief Flood-fill from a given pixel
         * \details Fills the connected component of black (zero) pixels
         *   incident to x,y
         */
        void flood_fill(int x, int y, const Color& c);
        
	/**
	 * \brief Sets a pixel of the image.
	 * \details Only BYTE, FLOAT32 and FLOAT64 component encoding
	 *  are supported. If the image has less than 4 channels, the
	 *  extra channels in \p c are ignored.
	 * \param[in] x , y the integer pixel coordinates
	 * \param[in] c the color of the pixel
	 */
	void set_pixel(int x, int y, const Color& c) {
	    geo_debug_assert(x >= 0 && x < int(image_->width()));
	    geo_debug_assert(y >= 0 && y < int(image_->height()));
	    switch(component_encoding_) {
		case Image::BYTE: {
		    Memory::byte* pixel_ptr =
			(Memory::byte*)(
			    image_->pixel_base(index_t(x),index_t(y))
		    );
		    for(index_t comp=0; comp<nb_components_; ++comp) {
			pixel_ptr[comp] = Memory::byte(c[comp] * 255.0);
		    }
		} break;
		case Image::FLOAT32: {
		    Numeric::float32* pixel_ptr =
			(Numeric::float32*)(void*)(
			    image_->pixel_base(index_t(x),index_t(y))
		    );
		    for(index_t comp=0; comp<nb_components_; ++comp) {
			pixel_ptr[comp] = Numeric::float32(c[comp]);
		    }
		} break;
		case Image::FLOAT64: {
		    Numeric::float64* pixel_ptr =
			(Numeric::float64*)(void*)(
			    image_->pixel_base(index_t(x),index_t(y))
		    );
		    for(index_t comp=0; comp<nb_components_; ++comp) {
			pixel_ptr[comp] = Numeric::float64(c[comp]);
		    }
		} break;
		case Image::INT16:
		case Image::INT32: {
		    geo_assert_not_reached;
		} 
	    }
	}

        /**
         * \brief Tests whether a given pixel is black
         * \details Only implemented for BYTE component encoding
         * \param[in] x , y the integer coordinates of the pixel
         * \retval true if the pixel is black
         * \retval false otherwise
         */
        bool pixel_is_black(int x, int y) const {
            Memory::byte* p = image_->pixel_base_byte_ptr(
                index_t(x),index_t(y)
            );
            bool result = true;
            for(size_t c=0; c<image_->components_per_pixel(); ++c) {
                result = result && (*p == 0);
            }
            return result;
        }
	
      protected:

	/**
	 * \brief Transforms a 2d point from world space to pixel coordinates.
	 * \param[in] p the world-space coordinates of the point.
	 * \param[out] transformed the pixel coordinates of the point.
	 */
	void transform(const vec2& p, vec2i& transformed) {
	    transformed.x = Numeric::int32(
		double(image_->width())*(p.x - x1_) / (x2_ - x1_)
	    );
	    transformed.y = Numeric::int32(
		double(image_->height())*(p.y - y1_) / (y2_ - y1_)
	    );
	}

	/**
	 * \brief Computes the linear interpolation between three colors.
	 * \param[in] c1 , c2 , c3 the three colors to be interpolated.
	 * \param[in] l1 , l2 , l3 the coefficients of the interpolation.
	 * \param[out] c the interpolated color.
	 */
	void interpolate_color(
	    const Color& c1, const Color& c2, const Color& c3,
	    double l1, double l2, double l3,
	    Color& c
	) {
	    c[0] = l1*c1[0] + l2*c2[0] + l3*c3[0];
	    c[1] = l1*c1[1] + l2*c2[1] + l3*c3[1];
	    c[2] = l1*c1[2] + l2*c2[2] + l3*c3[2];
	    c[3] = l1*c1[3] + l2*c2[3] + l3*c3[3];	    
	}

      private:
	Image* image_;
	Image::ComponentEncoding component_encoding_;
	index_t nb_components_;
	double x1_;
	double y1_;
	double x2_;
	double y2_;	
    };
}

#endif
