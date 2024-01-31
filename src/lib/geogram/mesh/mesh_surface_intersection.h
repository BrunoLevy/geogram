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
 
#ifndef GEOGRAM_MESH_MESH_SURFACE_INTERSECTION
#define GEOGRAM_MESH_MESH_SURFACE_INTERSECTION

#include <geogram/basic/common.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/numerics/exact_geometry.h>
#include <geogram/basic/process.h>
#include <geogram/basic/attributes.h>
#include <geogram/basic/debug_stream.h>
#include <functional>

/**
 * \file geogram/mesh/mesh_surface_intersection.h
 * \brief Functions for computing intersections between surfacic meshes and
 *        for boolean operations.
 */

namespace GEO {

    struct IsectInfo;

    /********************************************************************/

    /**
     * \brief Computes surface intersections
     * \details New vertices are stored with exact coordinates
     */
    class GEOGRAM_API MeshSurfaceIntersection {
    public:

        typedef exact::vec3h ExactPoint;

        MeshSurfaceIntersection(Mesh& M);
        ~MeshSurfaceIntersection();

        /**
         * \details A facet attribute of type index_t named "operand_bit" can 
         *  indicate for each facet to which operand of a n-ary boolean 
         *  operation it corresponds to (the same facet might belong to 
         *  several operands). It is taken into account by the two variants of 
         *  mesh_classify_intersections()
         */
        void intersect();

        /**
         * \brief Removes all the facets that are on the outer boundary
         * \pre set_radial_sort(true) was set before calling intersect()
         */
        void remove_external_shell();

        /**
         * \brief Removes all the facets that are not on the outer boundary
         * \pre set_radial_sort(true) was set before calling intersect()
         */
        void remove_internal_shells();


        void remove_fins();
        
        /**
         * \brief Classifies the facets and keep only 
         *   the ones on the boundary of a combination of regions defined
         *   by a boolean expression.
         * \details A facet attribute of type index_t named "operand_bit" 
         *  indicates for each facet to which operand of a n-ary boolean 
         *  operation it corresponds to (the same facet might belong to 
         *  several operands). 
         * \pre set_radial_sort(true) was set before calling intersect()
         * \param[in] expr the boolean function in ASCII. 
         *  One can use the following elements, and parentheses:
         *  - Variables: A..Z or x0..x31, correspond to the bits of the 
         *    "operand_bit" attribute
         *  - the special variable '*' corresponds to the union of everything
         *  - and:        '&' or '*'
         *  - or:         '|' or '+'
         *  - xor:        '^'
         *  - difference: '-'
         *  - not:        '!' or '~'
         *  Special values for expr: 
         *  - "union" (union of everything), synonym of '*'
         *  - "intersection" (intersection of everything).
         */
        void classify(const std::string& expr);


        /**
         * \brief Classifies a connected component
         * \param[in] component a connected component
         * \param[in] v a vertex of the connected component
         * \return the inclusion bits of the connected component relative
         *  to the operands
         */
        index_t classify_component(index_t component, index_t v);

        /**
         * \brief Classifies a vertex of the computed intersection
         * \param[in] component a component
         * \param[in] v a vertex of the component
         * \return the operand inclusion bits, or NO_INDEX if classification
         *  was not successful.
         * \details Uses raytracing along a random direction. The classification
         *  can be not successful if degenerate ray-triangle intersections are
         *  encountered. Then one needs to try again 
         *  using tentatively_classify_component_vertex() (multiple times if
         *  required).
         */
        index_t tentatively_classify_component_vertex_fast(
            index_t component, index_t v
        );

        /**
         * \brief Classifies a vertex of the computed intersection
         * \param[in] component a component
         * \param[in] v a vertex of the component
         * \return the operand inclusion bits, or NO_INDEX if classification
         *  was not successful.
         * \details Uses raytracing along a random direction. The classification
         *  can be not successful if degenerate ray-triangle intersections are
         *  encountered. Then one needs to try again.
         */
        index_t tentatively_classify_component_vertex(
            index_t component, index_t v
        );

        
        /**
         * \brief Merge coplanar facets and retriangulate them using a 
         *  Constrained Delaunay triangulation
         * \param[in] angle_tolerance angle tolerance for detecting coplanar
         *  facets and colinear edges (in degrees)
         */
        void simplify_coplanar_facets(double angle_tolerance = 0.0);
        
        /**
         * \brief Display information while computing the intersection.
         *  Default is unset.
         */
        void set_verbose(bool x) {
            verbose_      = x;
            fine_verbose_ = x;
        }

        /**
         * \brief Sets the threshold from which triangle is considered 
         *  to be a monster.
         * \details Monster triangles are saved to a file for the zoo.
         * \param[in] nb if a triangle has more than \p nb intersections
         *  in it, then it is considered to be a monster. 
         */
        void set_monster_threshold(index_t nb) {
            monster_threshold_ = nb;
        }

        /**
         * \brief In dry run mode, the computed local triangulations
         *  are not inserted in the global mesh. This is for benchmarking.
         *  Default is off.
         */
        void set_dry_run(bool x) {
            dry_run_ = x;
        }

        /**
         * \brief If set, compute constrained Delaunay triangulation
         *  in the intersected triangles. If there are intersections
         *  in coplanar facets, it guarantees uniqueness of their
         *  triangulation. Default is set.
         */
        void set_delaunay(bool x) {
            delaunay_ = x;
        }
        
        /** 
         * \brief detect and compute intersections between facets that share 
         *  a facet or an edge. Set to false if input is a set of conformal
         *  meshes. Default is set.
         */
        void set_detect_intersecting_neighbors(bool x) {
            detect_intersecting_neighbors_ = x;
        }

        /**
         * \brief Specifies whether surfaces should be duplicated and
         *  radial edges sorted in order to create the volumetric
         *  partition yielded by the intersection
         * \param[in] x true if radial edges should be sorted. Default is
         *  set
         */
        void set_radial_sort(bool x) {
            use_radial_sort_ = x;
        }

        /**
         * \brief Specifies whether coordinates should be normalized during
         *  computation. If set, original coordinates are restored at the
         *  end of intersect().
         * \param[in] x true if coordinates should be normalized. Default is
         *  set.
         */
        void set_normalize(bool x) {
            normalize_ = x;
        }


        /**
         * \brief Optionally save the skeleton (that is, the collection of 
         *  non-manifold edges) to a given mesh.
         * \param[in] skeleton a pointer to the mesh that will receive the
         *  skeleton.
         */
        void set_build_skeleton(Mesh* skeleton) {
            skeleton_ = skeleton;
        }

        /**
         * \brief Specifies that attributes should be interpolated
         * \param[in] x true if attributes should be interpolated, 
         *  false otherwise. Default is false.
         */
        void set_interpolate_attributes(bool x) {
            interpolate_attributes_ = x;
        }
        
    protected:
        /**
         * \brief substep of intersect(), prepares the mesh
         * \details Tesselates the facets if they are not triangulated,
         *  creates the operand bit for boolean op classification, removes
         *  the exactly degenerate triangles, colocate the points,
         *  optionally scales the coordinates and sets symbolic perturbation
         *  mode to lexicographic.
         */
        void intersect_prologue();

        /**
         * \brief substep of intersect(), finds all the intersection points
         *   and segments.
         * \param[out] intersections the vector of IsectInfo. Each IsectInfo
         *   is either an intersection vertex or a pair of intersection
         *   vertices. Intersection vertices are represented in symbolic
         *   form, as a couple of triangle indices plus a couple of triangle
         *   subregion id (TriangleRegion).
         * \details First uses a MeshFacetsAABB to detect candidate pairs
         *   of intersecting triangles, then calls triangles_intersection()
         *   in parallel. Finally, mesh facets are shuffled randomly, to
         *   ensure balanced multithreading for the subsequent steps.
         */
        void intersect_get_intersections(vector<IsectInfo>& intersections);

        /**
         * \brief substep of intersect(), inserts the intersection points
         *   and segments into the triangles.
         * \param[in,out] intersections the vector of IsectInfo. Each IsectInfo
         *   is either an intersection vertex or a pair of intersection
         *   vertices. Intersection vertices are represented in symbolic
         *   form, as a couple of triangle indices plus a couple of triangle
         *   subregion id (TriangleRegion).
         * \details Uses MeshInTriangle, a class derived from CDTBase2d, 
         *   that computes a constrained Delaunay triangulation with 
         *   intersection points represented with exact coordinates. 
         *   Operates in parallel. Each thread computes constrained
         *   Delaunay triangulations independently, and commits them in the
         *   resulting mesh (with a lock to protect concurrent accesses).
         *   The initial mesh is copied (and kept in the mesh_copy_ member), 
         *   so that concurrent read access do not need a lock. 
         */
        void intersect_remesh_intersections(vector<IsectInfo>& intersections);

        /**
         * \brief subset of intersect(), cleans the resulting mesh and
         *   undoes optional geometric normalization.
         * \param[in] intersections the vector of IsectInfo. Each IsectInfo
         *   is either an intersection vertex or a pair of intersection
         *   vertices. Intersection vertices are represented in symbolic
         *   form, as a couple of triangle indices plus a couple of triangle
         *   subregion id (TriangleRegion).
         * \details find the intersection that landed exactly onto an
         *   existing mesh vertex and merges them. Removes the initial 
         *   triangles that had intersections (they are replaced with new
         *   triangles). Merges duplicated triangles that come from 
         *   coplanar regions. Undoes geometric normalizations. Restores
         *   initial symbolic perturbation mode.
         */
        void intersect_epilogue(const vector<IsectInfo>& intersections);

    
        /**
         * \brief Acquires a lock on this mesh
         * \details A single thread can have the lock. When multiple threads 
         *  want the lock, the ones that do not have it keep waiting until 
         *  the one that owns the lock calls unlock(). All threads that modify 
         *  the target mesh should call this function
         * \see unlock()
         */
        void lock() {
            Process::acquire_spinlock(lock_);
        }

        /**
         * \brief Releases the lock associated with this mesh
         */
        void unlock() {
            Process::release_spinlock(lock_);
        }

        /**
         * \brief Gets the exact point associated with a vertex
         * \details If the vertex has explicit exact coordinates associated
         *  with it, they are returned, else an exact ExactPoint is constructed
         *  from the double-precision coordinates stored in the mesh
         * \param[in] v a vertex of the mesh
         * \return the exact coordinates of this vertex, as a vector in
         *  homogeneous coordinates stored as expansions
         */
        ExactPoint exact_vertex(index_t v) const;

        /**
         * \brief Finds or creates a vertex in the mesh, by exact coordinates
         * \details If there is already a vertex with coordinates \p p, then
         *  the existing vertex is returned, else a new vertex is constructed.
         *  Note that only the vertices created by find_or_create_vertex() can
         *  be returned as existing vertices. Mesh vertices stored as double-
         *  precision coordinates are not retreived by this function.
         * \param[in] p the exact coordinates of a point
         * \return the index of a mesh vertex with \p p as coordinates
         */
        index_t find_or_create_exact_vertex(const ExactPoint& p);

        /**
         * \brief Gets the target mesh
         * \return a modifiable reference to the mesh that was passed to
         *  the constructor
         */
        Mesh& target_mesh() {
            return mesh_;
        }

        /**
         * \brief Gets the target mesh
         * \return a const reference to the mesh that was passed to
         *  the constructor
         */
        const Mesh& target_mesh() const {
            return mesh_;
        }
        
        /**
         * \brief Gets a copy of the initial mesh passed to the constructor
         * \details It is used by the multithreaded mesh intersection algorithm.
         *   Each thread needs to both access the initial geometry and create
         *   new vertices and triangles in the target mesh. Creating new mesh
         *   elements can reallocate the internal vectors of the mesh, and 
         *   change the address of the elements. This should not occur while 
         *   another thread is reading the mesh. Copying the initial geometry 
         *   in another mesh prevents this type of problems.
         * \return a const reference to the mesh that was copied from the one
         *   passed to the constructor
         */
        const Mesh& readonly_mesh() const {
            return mesh_copy_;
        }

        class RadialSort;
        
        /**
         * \brief Builds the Weiler model
         * \details The Weiler model is a volumetric representation, where each
         *  facet is on the boundary of a closed region. Facets are duplicated,
         *  so that when two regions touch each other, each region has its own 
         *  facet on the boundary. Two facets that touch in this way are 
         *  connected by alpha3 links. Facets on the boundary of the same 
         *  region are connected by alpha2 links. 
         */
        void build_Weiler_model();

        /**
         * \brief Marks all the facets that are on the external shell
         */
        void mark_external_shell(vector<index_t>& on_external_shell);

    protected:

        /**
         * A class for sorting triangles around their common radial edge.
         */
        class GEOGRAM_API RadialSort {
        public:
            /**
             * \brief RadialSort constructor
             * \param[in] mesh the MeshSurfaceIntersection
             */
            RadialSort(
                const MeshSurfaceIntersection& mesh
            ) : mesh_(mesh),
                h_ref_(index_t(-1)),
                degenerate_(false)
            {
            }

            /**
             * \brief Initializes radial sorting around a given halfedge
             * \param[in] h_ref the reference halfedge 
             */
            void init(index_t h_ref);

            /**
             * \brief Compares two halfedges
             * \param[in] h1 , h2 the two halfedges
             * \retval true if \p h1 should be before \p h2 in radial order
             * \retval false otherwise
             */
            bool operator()(index_t h1, index_t h2) const;

            /**
             * \brief Tests if a degeneracy was encountered
             * \retval true if there were two coplanar triangles on the same
             *  side relative to h_ref
             * \retval false otherwise
             */
            bool degenerate() const {
                return degenerate_;
            }

            /**
             * \brief Computes a vector of arbitrary length with its 
             *  direction given by two points 
             * \param[in] p1 , p2 the two points in homogeneous coordinates
             * \return a vector in cartesian coordinates with the same 
             *  direction and orientation as \p p2 - \p p1
             */
            static exact::vec3 exact_direction(
                const ExactPoint& p1, const ExactPoint& p2
            );

            /**
             * \brief Computes an interval vector of arbitrary length with its 
             *  direction given by two points 
             * \param[in] p1 , p2 the two points in homogeneous coordinates
             * \return an interval vector in cartesian coordinates 
             *  with the same direction and orientation as \p p2 - \p p1
             */
            static vec3I exact_direction_I(
                const ExactPoint& p1, const ExactPoint& p2
            );
            
        protected:

            /**
             * \brief Computes the relative orientations of two halfedges
             * \param[in] h1 , h2 the two halfedges
             * \retval POSITIVE if going from \p h1's triangle to
             *  \p h2's triangle is a left turn (with h_ref facing to you)
             * \retval ZERO if \p h1 and \p h2 have co-linear normals
             * \retval NEGATIVE otherwise
             */
            Sign h_orient(index_t h1, index_t h2) const;

            /**
             * \brief Computes the normal orientation of a halfedge 
             *  relative to h_ref
             * \return the sign of the dot product between h_ref's triangle 
             *  normal and \p h2's triangle normal.
             */
            Sign h_refNorient(index_t h2) const;

        public:
            void test(index_t h1, index_t h2) {
                (*this)(h1,h2);
                Sign o_ref1 = h_orient(h_ref_, h1);
                Sign o_ref2 = h_orient(h_ref_, h2);
                Sign oN_ref1 = h_refNorient(h1);
                Sign oN_ref2 = h_refNorient(h2);
                Sign o_12 = h_orient(h1,h2);
                std::cerr
                   << " o_ref1=" << int(o_ref1) << " o_ref2=" << int(o_ref2)
                   << " oN_ref1=" << int(oN_ref1) << " oN_ref2=" << int(oN_ref2)
                   << " o_12=" << int(o_12)
                   << std::endl;
            }

            
        private:
            const MeshSurfaceIntersection& mesh_;
            index_t h_ref_; // ---reference halfedge
            exact::vec3 U_ref_;   // -.
            exact::vec3 V_ref_;   //  +-reference basis
            exact::vec3 N_ref_;   // -'
            vec3I U_ref_I_; // -.
            vec3I V_ref_I_; //  +-reference basis (interval arithmetics)
            vec3I N_ref_I_; // _'
            mutable vector< std::pair<index_t, Sign> > refNorient_cache_;
            mutable bool degenerate_;
        };

        
    protected:
        Process::spinlock lock_;
        Mesh& mesh_;
        Mesh mesh_copy_;
        Attribute<const ExactPoint*> vertex_to_exact_point_;

#ifdef GEOGRAM_USE_EXACT_NT
        // Exact points are canonicalized
        // (by Numeric::optimize_number_representation(vec3HEx)) so
        // we can use this comparator that makes the global vertex map
        // much much faster.
        typedef vec3HExLexicoCompareCanonical ExactPointCompare;
#else    
        // Generic comparator for global vertex map.
        typedef vec3HgLexicoCompare<exact::scalar> ExactPointCompare;
#endif
        std::map<ExactPoint,index_t,ExactPointCompare> exact_point_to_vertex_;
        
        bool verbose_;
        bool fine_verbose_;
        bool delaunay_;
        bool detect_intersecting_neighbors_;
        bool use_radial_sort_;

        PCK::SOSMode SOS_bkp_;
        bool rescale_; 
        bool normalize_;
        vec3 normalize_center_;
        double normalize_radius_;
        
        index_t monster_threshold_;
        bool dry_run_;
        friend class MeshInTriangle;
        friend class CoplanarFacets;

        Mesh* skeleton_;
        bool interpolate_attributes_;

        /***************************************************/
        
        /**
         * \brief Halfedfge-like API wrappers on top of a triangulated mesh
         * \details These are volumetric halfedges, also called 
         *  combinatorial 3-map, with both volumetric links (alpha3) 
         *  and surfacic link (alpha2).
         *  One may refer to this webpage for the definition of a 3-map:
         *   https://doc.cgal.org/latest/Combinatorial_map/
         */
        class Halfedges {
        public:

            /**
             * \brief Halfedges constructor
             * \param[in] I a reference to the MeshSurfaceIntersection
             */
            Halfedges(MeshSurfaceIntersection& I) : mesh_(I.mesh_) {
            }

            /**
             * \brief Halfedges destructor
             */
            ~Halfedges() {
                // TODO: destroy alpha3 attribute (kept now for debugging
            }

            /**
             * \brief Initializes the structure
             * \details Needs to be called before any other function
             */
            void initialize() {
                facet_corner_alpha3_.bind(
                    mesh_.facet_corners.attributes(), "alpha3"
                );
            }

            /**
             * \brief Gets the number of halfedegs in the map
             * \return the number of halfedges, that is, three times 
             *  the number of triangles (halfedges are not stored explicitly).
             */
            index_t nb() const {
                return mesh_.facet_corners.nb();
            }

            /**
             * \brief used by range-based for
             * \return a non-iterator corresponding to the first index.
             */
            index_as_iterator begin() const {
                return index_as_iterator(0);
            }

            /**
             * \brief used by range-based for
             * \return a non-iterator to one position past the last index.
             */
            index_as_iterator end() const {
                return index_as_iterator(nb());
            }

            /**
             * \brief Gets the facet associated to a halfedge
             * \param[in] h a halfedge index
             * \return the facet index, that is, h/3
             */
            index_t facet(index_t h) const {
                return h/3;
            }
            
            /**
             * \brief gets the surfacic neighbor of a halfedge
             * \details see definition of a combinatorial 3-map
             *   here: https://doc.cgal.org/latest/Combinatorial_map/
             * \param[in] h a halfedge index
             * \return another halfedge in the same surface, connecting the same
             *  vertices as \p h, but in opposite order
             * \see sew2()
             */
            index_t alpha2(index_t h) const {
                index_t t1 = facet(h);
                index_t t2 = mesh_.facet_corners.adjacent_facet(h);
                if(t2 == NO_INDEX) {
                    return NO_INDEX;
                }
                for(index_t h2: mesh_.facets.corners(t2)) {
                    if(mesh_.facet_corners.adjacent_facet(h2) == t1) {
                        return h2;
                    }
                }
                geo_assert_not_reached;
            }

            /**
             * \brief gets the volumetric neighbor of a halfedge
             * \details see definition of a combinatorial 3-map
             *   here: https://doc.cgal.org/latest/Combinatorial_map/
             * \param[in] h a halfedge index
             * \return another halfedge in a different volume, connecting 
             *  the same vertices as \p h, but in opposite order
             * \see sew3()
             */
            index_t alpha3(index_t h) const {
                return facet_corner_alpha3_[h];
            }

            /**
             * \brief gets the volumetric neighbor of a facet
             * \param[in] f a facet 
             * \return a facet with the same vertices as \p f but in 
             *  opposite index
             */
            index_t facet_alpha3(index_t f) const {
                return alpha3(3*f)/3;
            }

            /**
             * \brief gets a vertex of an halfedge
             * \param[in] h the halfedge
             * \param[in] dlv the local index of the vertex, in {0,1,2}
             * \return 
             *  - if \p dlv = 0 returns the origin vertex of \p h
             *  - if \p dlv = 1 returns the destination vertex of \p h
             *  - if \p dlv = 2 returns the vertex of the facet adjacent to \p h
             *    that is neither the origin nor the destination of \p h
             */
            index_t vertex(index_t h, index_t dlv) const {
                index_t f  = h/3;
                index_t lv = (h+dlv)%3;
                return mesh_.facets.vertex(f,lv);
            }
            

            /**
             * \brief Creates a surfacic link between two halfedges
             * \param[in] h1 , h2 the two halfedges to be connected
             * \pre \p h1 and \p h2 should have the same origins and
             *  destinations but in reverse order (\p h1 's origin should
             *  be \p h2 's destination and vice-versa).
             * \see alpha2()
             */
            void sew2(index_t h1, index_t h2) {
                geo_debug_assert(vertex(h1,0) == vertex(h2,1));
                geo_debug_assert(vertex(h2,0) == vertex(h1,1));            
                index_t t1 = h1/3;
                index_t t2 = h2/3;
                mesh_.facet_corners.set_adjacent_facet(h1,t2);
                mesh_.facet_corners.set_adjacent_facet(h2,t1);
            }

            /**
             * \brief Creates a volumetric link between two halfedges
             * \param[in] h1 , h2 the two halfedges to be connected
             * \pre \p h1 and \p h2 should have the same origins and
             *  destinations but in reverse order (\p h1 's origin should
             *  be \p h2 's destination and vice-versa).
             * \see alpha3()
             */
            void sew3(index_t h1, index_t h2) {
                geo_debug_assert(vertex(h1,0) == vertex(h2,1));
                geo_debug_assert(vertex(h2,0) == vertex(h1,1));            
                facet_corner_alpha3_[h1] = h2;
                facet_corner_alpha3_[h2] = h1;
            }

        private:
            Mesh& mesh_;
            Attribute<index_t> facet_corner_alpha3_;
        } halfedges_;

        /***************************************************/
        
        /**
         * \brief Represents the set of radial halfedge bundles
         * \details A Radial bundle corresponds to the set of halfedges 
         *   connecting the same pair of vertices (and in the same order).
         */
        class RadialBundles {
        public:

            /**
             * \brief RadialBundles constructor
             * \param[in] I a reference to the MeshSurfaceIntersectionx
             */
            RadialBundles(MeshSurfaceIntersection& I) : I_(I), mesh_(I.mesh_) {
            }

            /**
             * \brief Initializes the structure
             * \details Needs to be called before any other function
             */
            void initialize();

            /**
             * \brief Gets the number of bundles
             */
            index_t nb() const {
                return bndl_start_.size() - 1;
            }

            /**
             * \brief used by range-based for
             * \return a non-iterator corresponding to the first bundle
             */
            index_as_iterator begin() const {
                return index_as_iterator(0);
            }

            /**
             * \brief used by range-based for
             * \return a non-iterator to one position past the last bundle
             */
            index_as_iterator end() const {
                return index_as_iterator(nb());
            }

            /**
             * \brief Gets the number of halfedges in a bundle
             * \param[in] bndl the bundle
             * \return the number of halfedges in \p bndl
             */
            index_t nb_halfedges(index_t bndl) {
                geo_debug_assert(bndl < nb());
                return bndl_start_[bndl+1] - bndl_start_[bndl];
            }

            /**
             * \brief Gets a halfedge in a bundle from local index
             * \param[in] bndl the bundle
             * \param[in] li the local index of the halfedge in the bundle, 
             *    in [0 .. nb_halfedges(bndl)-1]
             * \return the halfedge
             */
            index_t halfedge(index_t bndl, index_t li) {
                geo_debug_assert(bndl < nb());
                geo_debug_assert(li < nb_halfedges(bndl));
                return H_[bndl_start_[bndl] + li];
            }

            /**
             * \brief Sets a halfedge in a bundle
             * \param[in] bndl the bundle
             * \param[in] li the local index of the halfedge in the bunble,
             *    in [0 .. nb_halfedges(bndl)-1]
             * \param[in] h the new halfedge
             */
            void set_halfedge(index_t bndl, index_t li, index_t h) {
                geo_debug_assert(bndl < nb());
                geo_debug_assert(li < nb_halfedges(bndl));
                H_[bndl_start_[bndl] + li] = h;
            }
            
            /**
             * \brief gets the halfedges in a bundle
             * \param[in] bndl bundle index
             * \return a modifiable sequence of halfedge indices
             */
            index_ptr_range halfedges(index_t bndl) {
                return index_ptr_range(
                    H_, bndl_start_[bndl], bndl_start_[bndl+1]
                );
            }

            /**
             * \brief gets the halfedges in a bundle
             * \param[in] bndl bundle index
             * \return a non-modifiable sequence of halfedge indices
             */
            const_index_ptr_range halfedges(index_t bndl) const {
                return const_index_ptr_range(
                    H_, bndl_start_[bndl], bndl_start_[bndl+1]
                );
            } 

            /**
             * \brief gets one of the vertices at the two extremities of a bundle
             * \param[in] bndl bundle index
             * \param[in] lv local vertex index, in {0,1}
             * \return if \p lv = 0 the source vertex, if \p lv = 1 the
             *  destination vertex
             */
            index_t vertex(index_t bndl, index_t lv) const {
                geo_debug_assert(bndl_start_[bndl+1] - bndl_start_[bndl] > 0);
                index_t h = H_[bndl_start_[bndl]];
                return I_.halfedges_.vertex(h,lv);
            }
            
            /**
             * \brief gets the first bundle starting from a vertex
             * \param[in] v the vertex
             * \details bundles starting from the same vertex are chained
             * \return the index of the first bundle starting from \p v, or
             *   NO_INDEX if there is no such bundle
             */
            index_t vertex_first_bundle(index_t v) const {
                return v_first_bndl_[v];
            }

            /**
             * \brief gets the next bundle around a vertex
             * \param[in] bndl the bundle
             * \details bundles starting from the same vertex are chained
             * \return the index of the next bundle that has the same origin
             *  vertex as \p bndl, or NO_INDEX if there is no such bundle
             */
            index_t next_around_vertex(index_t bndl) const {
                return bndl_next_around_v_[bndl];
            }

            /**
             * \brief gets the bumber of bundles around a vertex
             * \param[in] v the vertex
             * \return the number of bundles starting from \p v
             */
            index_t nb_bundles_around_vertex(index_t v) const {
                index_t result = 0;
                for(
                    index_t bndl = vertex_first_bundle(v);
                    bndl != NO_INDEX;
                    bndl = next_around_vertex(bndl)
                ) {
                    ++result;
                }
                return result;
            }
            
            /**
             * \brief gets the opposite bundle
             * \param[in] bndl a bundle index
             * \return the bundle connecting the same vertices as a given
             *  bundle but in the reverse order
             */
            index_t opposite(index_t bndl) {
                geo_debug_assert(bndl < nb());
                return (bndl >= nb()/2) ? (bndl-nb()/2) : (bndl+nb()/2);
            }

            /**
             * \brief gets the predecessor of a bundle along its polyline
             * \return the bundle arriving at the source vertex if it exists and
             *  is unique, NO_INDEX otherwise
             */
            index_t prev_along_polyline(index_t bndl) {
                index_t v = vertex(bndl,0);
                if(nb_bundles_around_vertex(v) > 2) {
                    return NO_INDEX;
                }
                for(
                    index_t bndl2 = vertex_first_bundle(v);
                    bndl2 != NO_INDEX; bndl2 = next_around_vertex(bndl2)
                ) {
                    if(bndl2 != bndl) {
                        return opposite(bndl2);
                    }
                }

                std::cerr << "Nb bundles around vertex = "
                          << nb_bundles_around_vertex(v) << std::endl;
                
                mesh_save(mesh_, "blackbox.geogram");
                DebugStream dbg("dbg");
                index_t v2 = vertex(bndl,1);
                vec3 p1(mesh_.vertices.point_ptr(v));
                vec3 p2(mesh_.vertices.point_ptr(v2));
                dbg.add_segment(p1,p2);
                geo_assert_not_reached;
            }

            /**
             * \brief gets the successor of a bundle along its polyline
             * \return the bundle originated at the destination vertex 
             *  if it exists and is unique, NO_INDEX otherwise
             */
            index_t next_along_polyline(index_t bndl) {
                index_t v = vertex(bndl,1);
                if(nb_bundles_around_vertex(v) > 2) {
                    return NO_INDEX;
                }
                for(
                    index_t bndl2 = vertex_first_bundle(v);
                    bndl2 != NO_INDEX; bndl2 = next_around_vertex(bndl2)
                ) {
                    if(opposite(bndl2) != bndl) {
                        return bndl2;
                    }
                }

                std::cerr << "Nb bundles around vertex = "
                          << nb_bundles_around_vertex(v) << std::endl;
                
                mesh_save(mesh_, "blackbox.geogram");
                DebugStream dbg("dbg");
                index_t v2 = vertex(bndl,0);
                vec3 p1(mesh_.vertices.point_ptr(v));
                vec3 p2(mesh_.vertices.point_ptr(v2));
                dbg.add_segment(p1,p2);
                
                geo_assert_not_reached;
            }

            /**
             * \brief Sorts the halfedges of the bundle in-place
             * \param[in] bndl the bundle
             * \param[in] RS a RadialSort structure (that caches 
             *  some information)
             * \retval true if radial sort was successful
             * \retval false otherwise (may happen with expansion_nt)
             */
            bool radial_sort(index_t bndl, RadialSort& RS) {
                geo_debug_assert(bndl < nb());
                if(nb_halfedges(bndl) <= 2) {
                    return true;
                }
                auto b = H_.begin() + std::ptrdiff_t(bndl_start_[bndl]);
                auto e = H_.begin() + std::ptrdiff_t(bndl_start_[bndl+1]);
                RS.init(*b);
                std::sort(
                    b, e,
                    [&](index_t h1, index_t h2)->bool {
                        return RS(h1,h2);
                    }
                );
                bool OK = !RS.degenerate();
                bndl_is_sorted_[bndl] = OK;
                return OK;
            }

            /**
             * \brief Sets the halfedges of a bundle
             * \details Used when radial sorting can be replaced with 
             *  combinatorial propagation. 
             * \param[in] bndl a bundle
             * \param[in] halfedges the sorted list of the halfedges 
             *  in the bundle
             */
            void set_sorted_halfedges(
                index_t bndl, const vector<index_t>& halfedges
            ) {
                geo_debug_assert(halfedges.size() == nb_halfedges(bndl));
                for(index_t i=0; i<halfedges.size(); ++i) {
                    set_halfedge(bndl, i, halfedges[i]);
                }
                bndl_is_sorted_[bndl] = true;
            }
            
            /**
             * \brief Indicates where to find a chart in a bundle
             * \details the first index is a chart index, and the second index
             *  indicates which halfedge in a bundle is incident to that chart.
             */
            typedef std::pair<index_t, index_t> ChartPos;
            
            /**
             * \brief Gets the sorted list of charts around bundle
             * \param[in] bndl a bundle
             * \param[out] chart_pos a list of (chart id, halfedge index) 
             *  couples, sorted by chart id, and where the halfedge index 
             *  is the original index in the bundle before sorting
             */
            void get_sorted_incident_charts(
                index_t bndl, vector<ChartPos>& chart_pos
            );

            bool is_sorted(index_t bndl) const {
                geo_assert(bndl < nb());
                return bndl_is_sorted_[bndl];
            }
            
        // private:
            MeshSurfaceIntersection& I_;
            Mesh& mesh_;
            Attribute<index_t> facet_chart_;
            vector<index_t> H_;
            vector<index_t> bndl_start_;
            vector<index_t> v_first_bndl_;
            vector<index_t> bndl_next_around_v_;
            vector<bool> bndl_is_sorted_;
        } radial_bundles_;

        /***************************************************/
        
        class RadialPolylines {
        public:
            /**
             * \brief RadialPolylines constructor
             * \param[in] I a reference to the MeshSurfaceIntersectionx
             */
            RadialPolylines(MeshSurfaceIntersection& I) : I_(I), mesh_(I.mesh_) {
            }

            /**
             * \brief Initializes the structure
             * \details Needs to be called before any other function
             */
            void initialize();

            /**
             * \brief Sorts all the bundles of all polylines
             * \details The "chart" facet attribute needs to be initialized with
             *  all surface connected components before calling this function.
             */
            void radial_sort();
            
            /**
             * \brief Gets the number of polylines
             */
            index_t nb() const {
                return polyline_start_.size() - 1;
            }

            /**
             * \brief used by range-based for
             * \return a non-iterator corresponding to the first polyline
             */
            index_as_iterator begin() const {
                return index_as_iterator(0);
            }

            /**
             * \brief used by range-based for
             * \return a non-iterator to one position past the last polyline
             */
            index_as_iterator end() const {
                return index_as_iterator(nb());
            }

            /**
             * \brief gets the bundles in a polyline
             * \param[in] polyline index of the polyline
             * \return a non-modifiable sequence of bundle indices
             */
            const_index_ptr_range bundles(index_t polyline) const {
                geo_debug_assert(polyline < nb());
                return const_index_ptr_range(
                    B_, polyline_start_[polyline], polyline_start_[polyline+1]
                );
            }            

            index_t nb_bundles(index_t polyline) const {
                geo_debug_assert(polyline < nb());
                return polyline_start_[polyline+1] - polyline_start_[polyline];
            }

            index_t bundle(index_t polyline, index_t li) const {
                geo_debug_assert(polyline < nb());
                geo_debug_assert(li < nb_bundles(polyline));
                return B_[polyline_start_[polyline] + li];
            }
            
            /**
             * \brief Copies the set of polylines to a mesh
             * \details Used for visualization purposes
             * \param[out] to a mesh that will contain all the polygonal lines
             */
            void get_skeleton(Mesh& to);
            
        private:
            MeshSurfaceIntersection& I_;
            Mesh& mesh_;
            vector<index_t> B_;
            vector<index_t> polyline_start_;
        } radial_polylines_;
    };
    
    /********************************************************************/    

    /**
     * \brief Computes a boolean operation with two surface meshes.
     * \details A and B need to be two closed surface
     *  mesh without intersections.
     * \param[in] A , B the two operands.
     * \param[out] result the computed mesh.
     * \param[in] operation one of "A+B", "A*B", "A-B", "B-A"
     * \param[in] verbose if set, display additional information 
     *   during computation
     */
    void GEOGRAM_API mesh_boolean_operation(
        Mesh& result, Mesh& A, Mesh& B, const std::string& operation,
        bool verbose=false
    );
    
    /**
     * \brief Computes the union of two surface meshes.
     * \details A and B need to be two closed surface
     *  mesh without intersections.
     * \param[in] A , B the two operands.
     * \param[out] result the computed mesh.
     * \param[in] verbose if set, display additional 
     *  information during computation
     */
    void GEOGRAM_API mesh_union(
        Mesh& result, Mesh& A, Mesh& B, bool verbose=false
    );

    /**
     * \brief Computes the intersection of two surface meshes.
     * \details A and B need to be two closed surface
     *  mesh without intersections.
     * \param[in] A , B the two operands.
     * \param[out] result the computed mesh.
     * \param[in] verbose if set, display additional information 
     *  during computation
     */
    void GEOGRAM_API mesh_intersection(
        Mesh& result, Mesh& A, Mesh& B, bool verbose=false
    );

    /**
     * \brief Computes the difference of two surface meshes.
     * \details A and B need to be two closed surface
     *  mesh without intersections.
     * \param[in] A , B the two operands.
     * \param[out] result the computed mesh.
     * \param[in] verbose if set, display additional information 
     *  during computation
     */
    void GEOGRAM_API mesh_difference(
        Mesh& result, Mesh& A, Mesh& B, bool verbose=false
    );
    
    /**
     * \brief Attempts to make a surface mesh conformal by
     *  removing intersecting facets and re-triangulating the holes.
     * \param[in] verbose if set, display additional information 
     *  during computation
     */
    void GEOGRAM_API mesh_remove_intersections(
        Mesh& M, index_t max_iter = 3, bool verbose=false
    );

    /**
     * \brief Tests whether two mesh facets have a non-degenerate intersection.
     * \details If the facets are polygonal, they are triangulated from the
     *  first vertex, and intersections between each pair of triangles is
     *  tested.
     * \retval true if the two facets have an intersection. If they share a
     *  vertex, it does not count as an intersection. 
     * \retval false otherwise.
     */
    bool GEOGRAM_API mesh_facets_have_intersection(
        Mesh& M, index_t f1, index_t f2
    );

    /**************************************************************************/
}

#endif

