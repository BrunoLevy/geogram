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

#ifndef GEOGRAM_MESH_MESH_HALFEDGES
#define GEOGRAM_MESH_MESH_HALFEDGES

#include <geogram/basic/common.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>
#include <iostream>

/**
 * \file geogram/mesh/mesh_halfedges.h
 * \brief Classes and function for virtually seeing a mesh as a set of halfedges
 * \details
 * MeshHalfedges is the interface itself
 * MeshHalfedges::Halfedges is an oriented edge, which stores a facet index and a facet corner index
 *          o
 *         / \
 *        /   \
 *       /     \
 *      / facet \
 *     /         \
 *    /\corner    \
 *   o ==========> o
 *      halfedge
 * 
 * Around a vertex:
 * - move_to_next_around_vertex() moves clockwise
 * - move_to_prev_around_vertex() moves counterclockwise
 * so counterclockwise, the facet is ahead of the halfedge
 * and clockwise, the halfedge is ahead of the facet
 */

namespace GEO {

    /**
     * \brief Exposes a half-edge like API for
     * traversing a Mesh.
     */
    class GEOGRAM_API MeshHalfedges {
    public:
        /**
         * \brief Stores a reference to a mesh corner and facet, and
         *  provides a halfedge-like API.
         */
        struct Halfedge {

            static const index_t NO_FACET  = index_t(-1);
            static const index_t NO_CORNER = index_t(-1);

            /**
             * \brief Constructs a new uninitialized Halfedge.
             */
            Halfedge() :
                facet(NO_FACET),
                corner(NO_CORNER) {
            }

            /**
             * \brief Constructs a new Halfedge from a facet and corner index.
             * \param[in] f the facet index
             * \param[in] c the corner index
             */
            Halfedge(index_t f, index_t c) :
                facet(f),
                corner(c) {
            }

            /**
             * \brief Clears this Halfedge.
             */
            void clear() {
                facet = NO_FACET;
                corner = NO_CORNER;
            }

            /**
             * \brief Tests whether this Halfedge is initialized.
             * \return true if this Halfedge is uninitialized, false otherwise
             */
            bool is_nil() const {
                return (facet == NO_FACET) && (corner == NO_CORNER);
            }

            /**
             * \brief Tests whether this Halfedge is the same as another one
             * \param[in] rhs the comparand
             * \return true if this Halfedge and \p rhs refer to the same
             *  facet and corner, false otherwise.
             */
            bool operator== (const Halfedge& rhs) const {
                return facet == rhs.facet && corner == rhs.corner;
            }

            /**
             * \brief Tests whether this Halfedge is different from another one
             * \param[in] rhs the comparand
             * \return true if this Halfedge and \p rhs refer to a different
             *  facet or corner, false otherwise.
             */
            bool operator!= (const Halfedge& rhs) const {
                return !(rhs == *this);
            }

            index_t facet;  // the facet at the left of the halfedge
            index_t corner; // the corner of 'facet' along the halfedge origin

        };

        /**
         * \brief Creates a new MeshHalfedges
         * \param[in] mesh the Mesh
         */
        MeshHalfedges(Mesh& mesh) : mesh_(mesh) {
        }

        /**
         * \brief Gets the mesh.
         * \return a reference to the mesh.
         */
        Mesh& mesh() {
            return mesh_;
        }

        /**
         * \brief Gets the mesh.
         * \return a const reference to the mesh.
         */
        const Mesh& mesh() const {
            return mesh_;
        }

        /**
         * \brief Sets whether facet regions determine borders.
         * \param[in] x if set, then an halfedge incident to two facets
         *  with different facet regions is considered to be a
         *  border
         */
        void set_use_facet_region(bool x) {
            if(x) {
                if(!facet_region_.is_bound()) {
                    facet_region_.bind(mesh_.facets.attributes(),"region");
                }
            } else {
                if(facet_region_.is_bound()) {
                    facet_region_.unbind();
                }
            }
        }

	/**
	 * \brief Sets a facet attribute name that determines borders.
	 * \param[in] attribute_name the name of the facet attribute to
	 *  be used to determine borders.
	 */
	void set_use_facet_region(const std::string& attribute_name) {
	    if(facet_region_.is_bound()) {
		facet_region_.unbind();
	    }
	    facet_region_.bind(mesh_.facets.attributes(),attribute_name);
	}

	/**
	 * \brief Sets a facet attribute name that determines borders.
	 * \param[in] attribute_name the name of the facet attribute to
	 *  be used to determine borders.
         * \details Needed to have this overload, because const char*
         *  is implicitly converted to bool instead of std::string.
	 */
        void set_use_facet_region(const char* attribute_name) {
            set_use_facet_region(std::string(attribute_name));
        }
        
	
        /**
         * \brief Tests whether a Halfedge is valid.
         * \param[in] H the Halfedge to be tested
         * \return true if \p H refers to a halfedge that
         *  exists in the mesh, false otherwise
         * \note It only tests whether H.corner and H.facet are valid
         *  indices in the mesh, but does not test whether H.corner exists
         *  in H.facet.
         */
        bool halfedge_is_valid(const Halfedge& H) const {
            return
                H.facet != Halfedge::NO_FACET && 
                H.corner != Halfedge::NO_CORNER &&
                H.facet < mesh_.facets.nb() &&
                H.corner < mesh_.facet_corners.nb()
            ;
        }

        /**
         * \brief Tests whether a Halfedge is on the boder.
         * \details If set_use_facet_region() is set, then
         *  Halfedges incident to two different facet regions are
         *  considered as borders.
         * \param[in] H the Halfedge
         * \return true if \p H is on the border, false otherwise
         */
        bool halfedge_is_border(const Halfedge& H) const {
            geo_debug_assert(halfedge_is_valid(H));
            if(facet_region_.is_bound()) {
                index_t f = H.facet;
                index_t adj_f =
                    mesh_.facet_corners.adjacent_facet(H.corner);
                return
                    adj_f == NO_FACET ||
                    facet_region_[f] != facet_region_[adj_f]
                ;
            } 
            return mesh_.facet_corners.adjacent_facet(H.corner) == NO_FACET;
        }

        /****** Moving around a facet **********/

        /**
         * \brief Replaces a Halfedge with the next one around the facet.
         * \param[in,out] H the Halfedge
         */
        void move_to_next_around_facet(Halfedge& H) const {
            geo_debug_assert(halfedge_is_valid(H));
            H.corner = mesh_.facets.next_corner_around_facet(H.facet, H.corner);
        }

        /**
         * \brief Replaces a Halfedge with the previous one around the facet.
         * \param[in,out] H the Halfedge
         */
        void move_to_prev_around_facet(Halfedge& H) const {
            geo_debug_assert(halfedge_is_valid(H));
            H.corner = mesh_.facets.prev_corner_around_facet(H.facet, H.corner);
        }

        /**
         * \brief Replaces a Halfedge by going clockwise around the facet.
         * \param[in,out] H the Halfedge
         */
        inline void move_clockwise_around_facet(Halfedge& H) const {
            move_to_prev_around_facet(H); // around facets, next is counterclockwise and prev is clockwise
        }

        /**
         * \brief Replaces a Halfedge by going counterclockwise around the facet.
         * \param[in,out] H the Halfedge
         */
        inline void move_counterclockwise_around_facet(Halfedge& H) const {
            move_to_next_around_facet(H); // around facets, next is counterclockwise and prev is clockwise
        }

        /****** Moving around a vertex **********/
        
        /**
         * \brief Replaces a Halfedge with the next one around the vertex.
         * \param[in,out] H the Halfedge
         * \return true if the move was successful, false otherwise. On borders,
         *  the next halfedge around a vertex may not exist.
         */
        bool move_to_next_around_vertex(Halfedge& H) const;

        /**
         * \brief Replaces a Halfedge with the previous one around the vertex.
         * \param[in,out] H the Halfedge
         * \return true if the move was successful, false otherwise. On borders,
         *  the previous halfedge around a vertex may not exist.
         */
        bool move_to_prev_around_vertex(Halfedge& H) const;

        /**
         * \brief Replaces a Halfedge by going clockwise around the vertex.
         * \param[in,out] H the Halfedge
         */
        inline bool move_clockwise_around_vertex(Halfedge& H) const {
            return move_to_next_around_vertex(H); // around vertices, next is clockwise and prev is counterclockwise
        }

        /**
         * \brief Replaces a Halfedge by going counterclockwise around the vertex.
         * \param[in,out] H the Halfedge
         */
        inline bool move_counterclockwise_around_vertex(Halfedge& H) const {
            return move_to_prev_around_vertex(H); // around vertices, next is clockwise and prev is counterclockwise
        }

        /****** Moving around the border **********/

        /**
         * \brief Replaces a Halfedge with the next one around the border.
         * \details If set_use_facet_region() is set, then
         *  Halfedges incident to two different facet regions are
         *  considered as borders.
         * \param[in,out] H the Halfedge
         */
        void move_to_next_around_border(Halfedge& H) const;

        /**
         * \brief Replaces a Halfedge with the previous one around the border.
         * \details If set_use_facet_region() is set, then
         *  Halfedges incident to two different facet regions are
         *  considered as borders.
         * \param[in,out] H the Halfedge
         */
        void move_to_prev_around_border(Halfedge& H) const;

        /****** Flip halfedge **********/
        
        /**
         * \brief Replaces a Halfedge with the opposite one in the
         *  adjacent facet.
         * \param[in,out] H the Halfedge
         * \pre !is_on_border(H)
         */
        void move_to_opposite(Halfedge& H) const;

    protected:
        Mesh& mesh_;
        Attribute<index_t> facet_region_;
    };

    /**
     * \brief Displays a Halfedge.
     * \param[out] out the stream where to print the Halfedge
     * \param[in] H the Halfedge
     * \return a reference to the stream \p out
     */
    inline std::ostream& operator<< (
        std::ostream& out, const MeshHalfedges::Halfedge& H
    ) {
        return out << '(' << H.facet << ',' << H.corner << ')';
    }

    namespace Geom {

        /****** Get origin/extremity vertices **********/

        /**
         * \brief Gets the origin vertex of a Halfedge
         * \param[in] M the mesh
         * \param[in] H the Halfedge
         * \return the vertex index of the origin of \p H
         */
        inline index_t halfedge_vertex_index_from(
            const Mesh& M, const MeshHalfedges::Halfedge& H
        ) {
            return M.facet_corners.vertex(H.corner);
        }

        /**
         * \brief Gets the origin point of a Halfedge
         * \param[in] M the mesh
         * \param[in] H the Halfedge
         * \return a const reference to the origin of \p H
         */
        inline const vec3& halfedge_vertex_from(
            const Mesh& M, const MeshHalfedges::Halfedge& H
        ) {
            return mesh_vertex(M, halfedge_vertex_index_from(M,H));
        }

        /**
         * \brief Gets the arrow extremity vertex of a Halfedge
         * \param[in] M the mesh
         * \param[in] H the Halfedge
         * \return the vertex index of the arrow extremity of \p H
         */
        inline index_t halfedge_vertex_index_to(
            const Mesh& M, const MeshHalfedges::Halfedge& H
        ) {
            index_t c = M.facets.next_corner_around_facet(H.facet, H.corner);
            return M.facet_corners.vertex(c);
        }

        /**
         * \brief Gets the arrow extremity point of a Halfedge
         * \param[in] M the mesh
         * \param[in] H the Halfedge
         * \return a const reference to the arrow extremity of \p H
         */
        inline const vec3& halfedge_vertex_to(
            const Mesh& M, const MeshHalfedges::Halfedge& H
        ) {
            return mesh_vertex(M, halfedge_vertex_index_to(M,H));
        }

        /****** Get halfedge as vector **********/

        /**
         * \brief Gets a 3d vector that connects the origin with the arrow
         *  extremity of a Halfedge.
         * \param[in] M the Mesh
         * \param[in] H the Halfedge
         * \return a 3d vector that connects the origin with the arrow
         *  extremity of \p H
         */
        inline vec3 halfedge_vector(
            const Mesh& M, const MeshHalfedges::Halfedge& H
        ) {
            return halfedge_vertex_to(M, H) - halfedge_vertex_from(M, H);
        }

        /**
         * \brief Gets the length of a Halfedge
         * \param[in] M the Mesh
         * \param[in] H the Halfedge
         * \return the 3d length of \p H
         */
        inline double edge_length(
            const Mesh& M, const MeshHalfedges::Halfedge& H
        ) {
            return length(halfedge_vector(M, H));
        }

        /****** Get left/rigth facets **********/

        /**
         * \brief Gets the facet at the left of a Halfedge
         * \param[in] M the Mesh
         * \param[in] H the Halfedge
         * \return The facet index (can be NO_FACET)
         */
        inline index_t halfedge_facet_left(
            const Mesh& M, const MeshHalfedges::Halfedge& H
        ) {
            geo_argused(M);
            return H.facet;
        }

        /**
         * \brief Gets the facet at the right of a Halfedge
         * \param[in] M the Mesh
         * \param[in] H the Halfedge
         * \return The facet index (can be NO_FACET)
         */
        inline index_t halfedge_facet_right(
            const Mesh& M, const MeshHalfedges::Halfedge& H
        ) {
            return M.facet_corners.adjacent_facet(H.corner); // see H.facet new value in move_to_opposite()
        }
    }
}

#endif

