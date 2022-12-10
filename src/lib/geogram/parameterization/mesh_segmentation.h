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

#ifndef GEOGRAM_MESH_MESH_SEGMENTATION
#define GEOGRAM_MESH_MESH_SEGMENTATION

#include <geogram/basic/common.h>
#include <geogram/basic/attributes.h>

/**
 * \file geogram/mesh/mesh_segmentation.h
 * \details Functions to split a surface mesh into multiple parts.
 */

namespace GEO {
    class Mesh;

    enum MeshSegmenter {
        
        /** continuous version of variational shape approximation*/
        SEGMENT_GEOMETRIC_VSA_L2,
        
        /** anisotropic continous version of variational shape approximation */
        SEGMENT_GEOMETRIC_VSA_L12,

        /** spectral segmentation with 8 manifold harmonics */        
        SEGMENT_SPECTRAL_8,

        /** spectral segmentation with 20 manifold harmonics */                
        SEGMENT_SPECTRAL_20,

        /** 
         * spectral segmentation with 100 manifold harmonics 
         * (uses some memory, use with caution !)
         */                
        SEGMENT_SPECTRAL_100
    };

    /**
     * \brief Computes a segmentation of a mesh.
     * \details The segmentation is stored in the "chart" facet attribute.
     * \param[in,out] mesh the mesh to be segmented
     * \param[in] segmenter one of 
     *   SEGMENT_GEOMETRIC_VSA_L2, SEGMENT_GEOMETRIC_VSA_L12, 
     *   SEGMENT_SPECTRAL_8, SEGMENT_SPECTRAL_20, SEGMENT_SPECTRAL_100
     * \param[in] nb_segment desired number of segment
     */
    void GEOGRAM_API mesh_segment(
        Mesh& mesh, MeshSegmenter segmenter, index_t nb_segments
    );
    
    /**
     * \brief A piece of a mesh.
     * \details Stores a list of facet indices. The mesh it belongs
     *  to is supposed to have an Attribute<index_t> attached to the
     *  facets and indicating the id of the chart for each facet.
     */
    struct Chart {

	/**
	 * \brief Chart constructor.
	 * \param[in] mesh_in a reference to a surface mesh.
	 * \param[in] id_in the id of the chart.
	 */
        Chart(Mesh& mesh_in, index_t id_in) :
	    mesh(mesh_in), id(id_in) {
        }

	/**
	 * \brief Chart copy constructor.
	 * \param[in] rhs a const reference to the Chart to be copied.
	 */
        Chart(const Chart& rhs) :
	    mesh(rhs.mesh), facets(rhs.facets), id(rhs.id) {
	}

	/**
	 * \brief Chart affectation.
	 * \param[in] rhs a const reference to the Chart to be copied.
	 * \return a reference to this Chart after copy.
	 * \pre rhs.mesh == mesh
	 */
	Chart& operator=(const Chart& rhs) {
	    if(&rhs != this) {
		geo_debug_assert(&rhs.mesh == &mesh);
		facets = rhs.facets;
		id = rhs.id;
	    }
	    return *this;
	}

	/**
	 * \brief Gets the number of edges on the border of this chart.
	 * \details An edge is on the border of a chart if it is on the
	 *  border of the surface or if the adjacent facet is on a different
	 *  chart.
	 */
	index_t nb_edges_on_border() const;

	/**
	 * \brief Tests whether a chart is shaped like a sock.
	 * \details A chart is shaped like a sock if the area of
	 *  the holes is smaller than a certain threshold relative
	 *  to the surface area.
	 */
	bool is_sock(double min_area_ratio = 1.0/6.0) const;
	
        /**
	 * \brief A reference to the mesh.
	 */
	Mesh& mesh;

	/**
	 * \brief The list of facet indices of this chart.
	 * \details The mesh is supposed to have an Attribute<index_t>
	 *  named "chart" attached to the facets, and the list of facets
	 *  has all the facets f for which chart[f] == id.
	 */
	vector<index_t> facets;

	/**
	 * \brief The id of this chart.
	 * \details The mesh is supposed to have an Attribute<index_t>
	 *  named "chart" attached to the facets, and the list of facets
	 *  has all the facets f for which chart[f] == id.
	 */
	index_t id;
    };

    /**
     * \brief Splits a chart into two parts.
     * \param[in,out] chart the input chart. On exit, its list of
     *  facets is cleared.
     * \param[out] new_chart_1 , new_chart_2 the two generated charts.
     *  Their chart id is used to initialize the "chart" attribute of the
     *  input mesh.
     * \param[in] verbose if true, display messages and statistics.
     */
    void GEOGRAM_API split_chart_along_principal_axis(
	Chart& chart, Chart& new_chart_1, Chart& new_chart_2,
	index_t axis=2,	bool verbose = false
    );


    namespace Geom {

	/**
	 * \brief Computes the 2D bounding box of a parameterized mesh.
	 * \param[in] mesh a const reference to a parameterized surface mesh.
	 * \param[in] tex_coord the texture coordinates as a 2d vector 
	 *  attribute attached to the facet corners.
	 * \param[out] xmin , ymin , xmax , ymax references to the 
	 *  extremum coordinates of the parameterization.
	 */
	void GEOGRAM_API get_mesh_bbox_2d(
	    const Mesh& mesh, Attribute<double>& tex_coord,
	    double& xmin, double& ymin,
	    double& xmax, double& ymax
	);

	/**
	 * \brief Computes the 2D bounding box of a parameterized chart.
	 * \param[in] chart a const reference to a parameterized surface chart.
	 * \param[in] tex_coord the texture coordinates as a 
	 *  2d vector attribute attached to the facet corners.
	 * \param[out] xmin , ymin , xmax , ymax references to the 
	 *  extremum coordinates of the parameterization.
	 */
	void GEOGRAM_API get_chart_bbox_2d(
	    const Chart& chart, Attribute<double>& tex_coord,
	    double& xmin, double& ymin,
	    double& xmax, double& ymax
	);

	/**
	 * \brief Computes the 2D parameter-space area of a parameterized mesh.
	 * \param[in] mesh a const reference to a parameterized surface mesh.
	 * \param[in] tex_coord the texture coordinates as a 
	 *  2d vector attribute attached to the facet corners.
	 * \return the area of the parameter space.
	 */
	double GEOGRAM_API mesh_area_2d(
	    const Mesh& mesh, Attribute<double>& tex_coord
	);

	/**
	 * \brief Computes the 2D parameter-space area of a parameterized chart.
	 * \param[in] chart a const reference to a parameterized surface chart.
	 * \param[in] tex_coord the texture coordinates as a 2d vector 
	 *  attribute attached to the facet corners.
	 * \return the area of the parameter space.
	 */
	double GEOGRAM_API chart_area_2d(
	    const Chart& chart, Attribute<double>& tex_coord
	);

	/**
	 * \brief Computes the area of a chart.
	 * \param[in] chart a const reference to a chart.
	 * \return the area of the chart in 3D.
	 */
	double GEOGRAM_API chart_area(const Chart& chart);


	/**
	 * \brief Computes the 2D parameter-space area of a facet in 
	 *  a parameterized mesh.
	 * \param[in] mesh a const reference to the mesh.
	 * \param[in] f a facet of \p mesh.
	 * \param[in] tex_coord the texture coordinates as a 2d vector 
	 *  attribute attached to the facet corners.
	 * \return the area of the facet in parameter space.
	 */
	double GEOGRAM_API mesh_facet_area_2d(
	    const Mesh& mesh, index_t f, Attribute<double>& tex_coord
	);
    }
}

#endif


