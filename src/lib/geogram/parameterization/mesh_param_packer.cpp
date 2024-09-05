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

#include <geogram/parameterization/mesh_param_packer.h>
#include <geogram/parameterization/mesh_atlas_maker.h>
#include <geogram/parameterization/mesh_segmentation.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/points/principal_axes.h>
#include <geogram/basic/logger.h>
#include <geogram/numerics/matrix_util.h>
#include <geogram/third_party/xatlas/xatlas.h>
#include <algorithm>
#include <stack>
#include <math.h>

/**************************************************************
 **** XATLAS                                               ****
 **************************************************************/

namespace {
    using namespace GEO;

    /**
     * \brief Interface between geogram and xatlas to represent
     *  charts.
     */
    class XAtlasChart {
    public:
        // TODO: create all charts simultaneously (because here
        // we are O(m*n), not good, there can be many charts...)
        // ... (seems to be OK though, even on large meshes)
        void initialize(const Mesh& M, index_t chart_id) {
            Attribute<index_t> chart;
            chart.bind_if_is_defined(M.facets.attributes(), "chart");
            geo_assert(chart.is_bound());
            Attribute<double> tex_coord;
            tex_coord.bind_if_is_defined(
                M.facet_corners.attributes(), "tex_coord"
            );
            geo_assert(tex_coord.is_bound());
            Attribute<index_t> xatlas_vertex_index(
                M.vertices.attributes(), "xatlas_index"
            );
            Attribute<index_t> facet_corner_xatlas_vertex_index(
                M.facet_corners.attributes(), "xatlas_index"
            );


            for(index_t v: M.vertices) {
                xatlas_vertex_index[v] = index_t(-1);
            }
            index_t cur_xatlas_vertex = 0;
            for(index_t f: M.facets) {
                if(chart[f] == chart_id) {
                    for(index_t c: M.facets.corners(f)) {
                        index_t v = M.facet_corners.vertex(c);
                        if(xatlas_vertex_index[v] == index_t(-1)) {
                            xatlas_vertex_index[v] = cur_xatlas_vertex;
                            const double* xyz = M.vertices.point_ptr(v);
                            xyz_.push_back(float(xyz[0]));
                            xyz_.push_back(float(xyz[1]));
                            xyz_.push_back(float(xyz[2]));
                            const double* uv = &tex_coord[2*c];
                            uv_.push_back(float(uv[0]));
                            uv_.push_back(float(uv[1]));
                            ++cur_xatlas_vertex;
                        }
                    }
                }
            }
            for(index_t f: M.facets) {
                if(chart[f] != chart_id) {
                    continue;
                }

                index_t c1 = M.facets.corners_begin(f);
                for(index_t c2 = c1+1;
                    c2+1 < M.facets.corners_end(f); ++c2
                   ) {
                    index_t v1 = M.facet_corners.vertex(c1);
                    index_t v2 = M.facet_corners.vertex(c2);
                    index_t v3 = M.facet_corners.vertex(c2+1);

                    triangles_.push_back(xatlas_vertex_index[v1]);
                    triangles_.push_back(xatlas_vertex_index[v2]);
                    triangles_.push_back(xatlas_vertex_index[v3]);
                }
            }

            for(index_t f: M.facets) {
                if(chart[f] != chart_id) {
                    continue;
                }
                for(index_t c: M.facets.corners(f)) {
                    index_t v = M.facet_corners.vertex(c);
                    facet_corner_xatlas_vertex_index[c] =
                        xatlas_vertex_index[v];
                }
            }

            mesh_decl_.vertexCount = uint32_t(cur_xatlas_vertex);
            mesh_decl_.vertexPositionData = xyz_.data();
            mesh_decl_.vertexPositionStride = sizeof(float)*3;
            mesh_decl_.vertexUvData = uv_.data();
            mesh_decl_.vertexUvStride = sizeof(float)*2;
            mesh_decl_.indexCount = uint32_t(triangles_.size());
            mesh_decl_.indexData = triangles_.data();
            mesh_decl_.indexOffset = 0;
            mesh_decl_.indexFormat = xatlas::IndexFormat::UInt32;

            uv_mesh_decl_.vertexCount = uint32_t(cur_xatlas_vertex);
            uv_mesh_decl_.vertexStride = sizeof(float)*2;
            uv_mesh_decl_.vertexUvData = uv_.data();
            uv_mesh_decl_.indexCount = uint32_t(triangles_.size());
            uv_mesh_decl_.indexData = triangles_.data();
            uv_mesh_decl_.indexOffset = 0;
            uv_mesh_decl_.indexFormat = xatlas::IndexFormat::UInt32;
            uv_mesh_decl_.faceMaterialData = nullptr;
            uv_mesh_decl_.rotateCharts = false;
        }

        xatlas::MeshDecl mesh_decl_;
        xatlas::UvMeshDecl uv_mesh_decl_;
        vector<float> xyz_;
        vector<float> uv_;
        vector<index_t> triangles_;
    };


    /**
     * \brief Allocator function for geogram
     * \details If we call realloc() with size == 0 to free memory
     *  (as xatlas does), then valgrind thinks that there is a memory leak,
     *  so I re-interpret the initial intent of the caller here,
     *  to make the memory debugger happy.
     */
    void* my_realloc(void* ptr, size_t size) {
        if(size == 0) {
            if(ptr != nullptr) {
                free(ptr);
            }
            return nullptr;
        }
        if(ptr == nullptr) {
            return malloc(size);
        }
        return realloc(ptr, size);
    }
}

namespace GEO {

    void pack_atlas_using_xatlas(Mesh& mesh) {

        Attribute<double> tex_coord;

        tex_coord.bind_if_is_defined(
            mesh.facet_corners.attributes(), "tex_coord"
        );

        if(!tex_coord.is_bound()) {
            Logger::err("Atlas") << "Mesh has no texture coordinates"
                                 << std::endl;
            return;
        }


        // Step 1: compute chart indices
        Attribute<index_t> chart(mesh.facets.attributes(), "chart");
        index_t nb_charts = mesh_get_charts(mesh);

        xatlas::Atlas* atlas = nullptr;

        // Step 2: copy to xatlas and pack
        {
            std::vector<XAtlasChart> charts;
            charts.resize(nb_charts);
            for(index_t c=0; c<nb_charts; ++c) {
                charts[c].initialize(mesh, c);
            }

            atlas = xatlas::Create();
            for(index_t c=0; c<nb_charts; ++c) {
                xatlas::AddMeshError::Enum error = xatlas::AddUvMesh(
                    atlas, charts[c].uv_mesh_decl_
                );
                if (error != xatlas::AddMeshError::Success) {
                    Logger::err("Packer")
                        << "XAtlas error: "
                        << xatlas::StringForEnum(error)
                        << std::endl;
                }
            }
            Attribute<index_t> xatlas_vertex_index(
                mesh.vertices.attributes(), "xatlas_index"
            );
            xatlas_vertex_index.destroy();
        }

        // Step 3: pack
        {
            xatlas::SetAlloc(my_realloc);
            xatlas::PackOptions packerOptions;
            packerOptions.padding = 1;
            xatlas::PackCharts(atlas, packerOptions);

            double u_min = Numeric::max_float64();
            double v_min = Numeric::max_float64();
            double u_max = Numeric::min_float64();
            double v_max = Numeric::min_float64();

            // Get uv bbox
            {
                Attribute<index_t> facet_corner_xatlas_vertex_index(
                    mesh.facet_corners.attributes(), "xatlas_index"
                );
                for(index_t f: mesh.facets) {
                    index_t chart_id = chart[f];
                    for(index_t c: mesh.facets.corners(f)) {
                        index_t v = facet_corner_xatlas_vertex_index[c];
                        const xatlas::Vertex& vertex =
                            atlas->meshes[chart_id].vertexArray[v];
                        double U = double(vertex.uv[0]);
                        double V = double(vertex.uv[1]);
                        u_min = std::min(u_min,U);
                        v_min = std::min(v_min,V);
                        u_max = std::max(u_max,U);
                        v_max = std::max(v_max,V);
                    }
                }
            }

            // Scale texture coordinates to [0,1]
            {
                double scale = 1.0 / std::max(u_max-u_min,v_max-v_min);
                Attribute<index_t> facet_corner_xatlas_vertex_index(
                    mesh.facet_corners.attributes(), "xatlas_index"
                );
                for(index_t f: mesh.facets) {
                    index_t chart_id = chart[f];
                    for(index_t c: mesh.facets.corners(f)) {
                        index_t v = facet_corner_xatlas_vertex_index[c];
                        const xatlas::Vertex& vertex =
                            atlas->meshes[chart_id].vertexArray[v];
                        tex_coord[2*c]   = scale*(double(vertex.uv[0])-u_min);
                        tex_coord[2*c+1] = scale*(double(vertex.uv[1])-v_min);
                    }
                }
                facet_corner_xatlas_vertex_index.destroy();
                chart.destroy();
            }
            xatlas::Destroy(atlas);
        }
    }

}

/**************************************************************
 **** TETRIS PACKER                                        ****
 **************************************************************/

namespace GEO {
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
        index_t nb_edges_on_border() const {
            index_t result = 0;
            Attribute<index_t> chart(mesh.facets.attributes(),"chart");
            for(index_t f1: facets) {
                for(index_t le = 0; le < mesh.facets.nb_vertices(f1); ++le) {
                    index_t f2 = mesh.facets.adjacent(f1,le);
                    if(f2 == index_t(-1) || chart[f2] != id) {
                        ++result;
                    }
                }
            }
            return result;
        }

        /**
         * \brief Tests whether a chart is shaped like a sock.
         * \details A chart is shaped like a sock if the area of
         *  the holes is smaller than a certain threshold relative
         *  to the surface area.
         */
        bool is_sock(double min_area_ratio = 1.0/6.0) const {
            Attribute<index_t> chart(mesh.facets.attributes(),"chart");

            vec3 border_bary(0.0, 0.0, 0.0);
            index_t bary_N = 0;

            for(index_t f1: facets) {
                for(index_t le = 0; le < mesh.facets.nb_vertices(f1); ++le) {
                    index_t f2 = mesh.facets.adjacent(f1,le);
                    if(f2 == index_t(-1) || chart[f2] != id) {
                        index_t v = mesh.facets.vertex(f1,le);
                        border_bary += vec3(mesh.vertices.point_ptr(v));
                        ++bary_N;
                    }
                }
            }

            border_bary *= (1.0 / double(bary_N));

            double total_area  = 0.0;
            double border_area = 0.0;

            for(index_t f1: facets) {
                total_area += Geom::mesh_facet_area(mesh,f1);

                index_t N = mesh.facets.nb_vertices(f1);
                for(index_t le = 0; le < N; ++le) {
                    index_t f2 = mesh.facets.adjacent(f1,le);
                    if(f2 == index_t(-1) || chart[f2] != id) {
                        index_t v1 = mesh.facets.vertex(f1,le);
                        index_t v2 = mesh.facets.vertex(f1,(le+1) % N);
                        vec3 p1(mesh.vertices.point_ptr(v1));
                        vec3 p2(mesh.vertices.point_ptr(v2));
                        border_area += Geom::triangle_area(border_bary, p1, p2);
                    }
                }
            }
            return ((border_area / total_area) < min_area_ratio);
        }

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

    namespace Geom {

        /**
         * \brief Computes the 2D bounding box of a parameterized mesh.
         * \param[in] mesh a const reference to a parameterized surface mesh.
         * \param[in] tex_coord the texture coordinates as a 2d vector
         *  attribute attached to the facet corners.
         * \param[out] xmin , ymin , xmax , ymax references to the
         *  extremum coordinates of the parameterization.
         */
        static void get_mesh_bbox_2d(
            const Mesh& mesh, Attribute<double>& tex_coord,
            double& xmin, double& ymin,
            double& xmax, double& ymax
        ) {
            xmin = Numeric::max_float64();
            ymin = Numeric::max_float64();
            xmax = Numeric::min_float64();
            ymax = Numeric::min_float64();
            for(index_t c: mesh.facet_corners) {
                xmin = std::min(xmin, tex_coord[2*c]);
                ymin = std::min(ymin, tex_coord[2*c+1]);
                xmax = std::max(xmax, tex_coord[2*c]);
                ymax = std::max(ymax, tex_coord[2*c+1]);
            }
        }

        /**
         * \brief Computes the 2D bounding box of a parameterized chart.
         * \param[in] chart a const reference to a parameterized surface chart.
         * \param[in] tex_coord the texture coordinates as a
         *  2d vector attribute attached to the facet corners.
         * \param[out] xmin , ymin , xmax , ymax references to the
         *  extremum coordinates of the parameterization.
         */
        static void get_chart_bbox_2d(
            const Chart& chart, Attribute<double>& tex_coord,
            double& xmin, double& ymin,
            double& xmax, double& ymax
        ) {
            xmin = Numeric::max_float64();
            ymin = Numeric::max_float64();
            xmax = Numeric::min_float64();
            ymax = Numeric::min_float64();
            for(index_t ff=0; ff<chart.facets.size(); ++ff) {
                index_t f = chart.facets[ff];
                for(index_t c: chart.mesh.facets.corners(f)) {
                    xmin = std::min(xmin, tex_coord[2*c]);
                    ymin = std::min(ymin, tex_coord[2*c+1]);
                    xmax = std::max(xmax, tex_coord[2*c]);
                    ymax = std::max(ymax, tex_coord[2*c+1]);
                }
            }
        }

        /**
         * \brief Computes the 2D parameter-space area of a facet in
         *  a parameterized mesh.
         * \param[in] mesh a const reference to the mesh.
         * \param[in] f a facet of \p mesh.
         * \param[in] tex_coord the texture coordinates as a 2d vector
         *  attribute attached to the facet corners.
         * \return the area of the facet in parameter space.
         */
        static double mesh_facet_area_2d(
            const Mesh& mesh, index_t f, Attribute<double>& tex_coord
        ) {
            double result = 0.0;
            index_t c1 = mesh.facets.corners_begin(f);
            vec2 p1(tex_coord[2*c1], tex_coord[2*c1+1]);
            for(
                index_t c2 = c1+1;
                c2+1 < mesh.facets.corners_end(f); ++c2
            ) {
                index_t c3 = c2+1;
                vec2 p2(tex_coord[2*c2], tex_coord[2*c2+1]);
                vec2 p3(tex_coord[2*c3], tex_coord[2*c3+1]);
                result += Geom::triangle_area(p1,p2,p3);
            }
            return result;
        }

        /**
         * \brief Computes the 2D parameter-space area of a parameterized mesh.
         * \param[in] mesh a const reference to a parameterized surface mesh.
         * \param[in] tex_coord the texture coordinates as a
         *  2d vector attribute attached to the facet corners.
         * \return the area of the parameter space.
         */
        static double mesh_area_2d(
            const Mesh& mesh, Attribute<double>& tex_coord
        ) {
            double result = 0.0;
            for(index_t f: mesh.facets) {
                result += mesh_facet_area_2d(mesh, f, tex_coord);
            }
            return result;
        }


        /**
         * \brief Computes the 2D parameter-space area of a parameterized chart.
         * \param[in] chart a const reference to a parameterized surface chart.
         * \param[in] tex_coord the texture coordinates as a 2d vector
         *  attribute attached to the facet corners.
         * \return the area of the parameter space.
         */
        static double chart_area_2d(
            const Chart& chart, Attribute<double>& tex_coord
        ) {
            double result = 0.0;
            for(index_t ff=0; ff<chart.facets.size(); ++ff) {
                index_t f = chart.facets[ff];
                result += mesh_facet_area_2d(chart.mesh, f, tex_coord);
            }
            return result;
        }

        /**
         * \brief Computes the area of a chart.
         * \param[in] chart a const reference to a chart.
         * \return the area of the chart in 3D.
         */
        static double chart_area(const Chart& chart) {
            double result = 0.0;
            for(index_t ff=0; ff<chart.facets.size(); ++ff) {
                index_t f = chart.facets[ff];
                result += Geom::mesh_facet_area(chart.mesh, f);
            }
            return result;
        }


    }
}

namespace {
    using namespace GEO;

    class AverageDirection2d {
    public:

        AverageDirection2d()  {
            result_is_valid_ = false;
        }

        void begin() {
            for(int i=0; i<3; i++) {
                M_[i] = 0.0 ;
            }
            result_is_valid_ = false;
        }

        void add_vector(const vec2& e) {
            M_[0] += e.x * e.x ;
            M_[1] += e.x * e.y ;
            M_[2] += e.y * e.y ;
        }

        void end() {
            double eigen_vectors[4] ;
            double eigen_values[2] ;
            MatrixUtil::semi_definite_symmetric_eigen(
                M_, 2, eigen_vectors, eigen_values
            );
            int k = 0 ;
            result_ = vec2(
                eigen_vectors[2*k],
                eigen_vectors[2*k+1]
            );
            result_is_valid_ = true ;
        }

        const vec2& average_direction() const {
            geo_assert(result_is_valid_) ;
            return result_ ;
        }

    private:
        double M_[3] ;
        vec2 result_ ;
        bool result_is_valid_ ;
    } ;
}


namespace GEO {

    /**
     * \brief Stores a reference to a chart, its bounding box
     *  in parameter space, and its upper and lower horizons.
     */
    class ChartBBox {
    public:

        /**
         * \brief ChartBBox constructor.
         * \param[in] chart a pointer to the chart.
         * \param[in] min , max lower-left and upper-right
         *  corners of the parameter-space bounding box of
         *  the chart.
         */
        ChartBBox(
            Chart* chart, const vec2& min, const vec2& max
        ) :
            min_(min), max_(max), chart_(chart),
            min_func_(nullptr), max_func_(nullptr),
            nb_steps_(0) {
        }

        /**
         * \brief ChartBBox destructor.
         */
        ~ChartBBox(){
            free();
            chart_ = nullptr;
        }

        /**
         * \brief ChartBBox copy-constructor.
         * \param[in] rhs a const reference to the ChartBBox to be copied.
         */
        ChartBBox(const ChartBBox& rhs) {
            copy(rhs);
        }

        /**
         * \brief ChartBBox affectation.
         * \param[in] rhs a const reference to the ChartBBox to be copied.
         * \return A reference to this ChartBBox after affectation.
         */
        ChartBBox& operator=(const ChartBBox& rhs) {
            if(&rhs != this) {
                free();
                copy(rhs);
            }
            return *this;
        }

        /**
         * \brief Initializes the upper and lower horizons.
         * \param[in] step pixel-size in parameter space
         * \param[in] margin margin size in parameter space
         * \param[in] margin_width_in_pixels margin size in pixels
         * \param[in] tex_coord a 2d vector attribute attached to
         *  the facet corners of the mesh with the texture coordinates.
         * \param[in] chart_attr an attribute attached to the facets of
         *  the mesh with the id of the chart the facet belongs to.
         */
        void init_max_and_min_func(
            double step, double margin, index_t margin_width_in_pixels,
            Attribute<double>& tex_coord, Attribute<index_t>& chart_attr
        ) {

            if(min_func_ != nullptr) {
                delete[] min_func_;
            }
            if(max_func_ != nullptr) {
                delete[] max_func_;
            }

            nb_steps_ = int(1.0 + (max_.x - min_.x) / step );
            min_func_ = new double[size_t(nb_steps_)];
            max_func_ = new double[size_t(nb_steps_)];
            for (int i=0;i<nb_steps_;i++){
                min_func(i) = max_.y - min_.y;
                max_func(i) = 0;
            }

            for(index_t ff=0; ff<chart_->facets.size(); ++ff) {
                index_t f = chart_->facets[ff];
                for(
                    index_t c1: chart_->mesh.facets.corners(f)) {
                    index_t neighf =
                        chart_->mesh.facet_corners.adjacent_facet(c1);

                    if(neighf != NO_FACET && chart_attr[neighf] == chart_->id) {
                        continue;
                    }

                    index_t c2 =
                        chart_->mesh.facets.next_corner_around_facet(f,c1);
                    vec2 p1(tex_coord[2*c1], tex_coord[2*c1+1]);
                    vec2 p2(tex_coord[2*c2], tex_coord[2*c2+1]);

                    if(p2.x == p1.x) {
                        continue;
                    }

                    if(p2.x < p1.x) {
                        std::swap(p1,p2);
                    }

                    p1.x -= double(margin_width_in_pixels) * step;
                    p2.x += double(margin_width_in_pixels) * step;

                    double a = (p2.y - p1.y) / (p2.x - p1.x);
                    for(double x = p1.x; x<p2.x; x+=step) {
                        double y = p1.y + a * (x - p1.x);
                        int cX = int((x - min_.x) / step);
                        if (cX>=0 && cX<nb_steps_) {
                            min_func(cX) = std::min(min_func(cX), y - margin);
                            max_func(cX) = std::max(max_func(cX), y + margin);
                        }
                    }
                }
            }
        }

        double& max_func(int i) {
            geo_assert(i>=0 && i<nb_steps_);
            return max_func_[i];
        }

        double& min_func(int i) {
            geo_assert(i>=0 && i<nb_steps_);
            return min_func_[i];
        }

        vec2 size() const {
            return max_-min_;
        }

        /**
         * \brief Gets the area of the bounding box.
         * \return the area of the bounding box in parameter space.
         */
        double area() const {
            vec2 s = size();
            return s.x*s.y;
        }

        /**
         * \brief Applies a translation vector to the texture coordinates.
         * \param[in] v the translation vector.
         * \param[in,out] tex_coord a dimension 2 vector attribute attached to
         *  the facet corners of the mesh with the texture coordinates.
         */
        void translate(const vec2& v, Attribute<double>& tex_coord){
            min_=min_+v;
            max_=max_+v;
            for(index_t ff=0; ff<chart_->facets.size(); ++ff) {
                index_t f = chart_->facets[ff];
                for(index_t c: chart_->mesh.facets.corners(f)) {
                    tex_coord[2*c] += v.x;
                    tex_coord[2*c+1] += v.y;
                }
            }
        }

        /**
         * \brief Gets the lower-left corner of the bounding box.
         * \return A const reference to the parametric-space
         *  coordinates of the lower-left corner of the bounding box.
         */
        const vec2& min() const {
            return min_;
        }

        /**
         * \brief Gets the upper-right corner of the bounding box.
         * \return A const reference to the parametric-space
         *  coordinates of the upper-right corner of the bounding box.
         */
        const vec2& max() const {
            return max_;
        }

        /**
         * \brief Gets the lower-left corner of the bounding box.
         * \return A modifiable reference to the parametric-space
         *  coordinates of the lower-left corner of the bounding box.
         */
        vec2& min() {
            return min_;
        }

        /**
         * \brief Gets the upper-right corner of the bounding box.
         * \return A modifiable reference to the parametric-space
         *  coordinates of the upper-right corner of the bounding box.
         */
        vec2& max() {
            return max_;
        }

        /**
         * \brief Gets the chart.
         * \return a const pointer to the chart.
         */
        const Chart* chart() const {
            return chart_;
        }


    protected:

        /**
         * \brief Copies a ChartBBox.
         * \param[in] rhs a const reference to the ChartBBox
         *  to be copied.
         */
        void copy(const ChartBBox& rhs) {
            chart_ = rhs.chart_;
            nb_steps_ = rhs.nb_steps_;

            if(rhs.min_func_ != nullptr) {
                min_func_ = new double[size_t(nb_steps_)];
                max_func_ = new double[size_t(nb_steps_)];
                for(int i=0; i<nb_steps_; i++) {
                    min_func_[i] = rhs.min_func_[i];
                    max_func_[i] = rhs.max_func_[i];
                }
            } else {
                geo_assert(rhs.max_func_ == nullptr);
                min_func_ = nullptr;
                max_func_ = nullptr;
            }
            min_ = rhs.min_;
            max_ = rhs.max_;
        }

        /**
         * \brief Releases the memory allocated by
         *  this ChartBBox.
         */
        void free() {
            delete[] min_func_;
            delete[] max_func_;
            min_func_ = nullptr;
            max_func_ = nullptr;
            nb_steps_ = 0;
        }

    private:
        vec2 min_, max_;
        Chart* chart_;
        double* min_func_;
        double* max_func_;
        int nb_steps_;
    };

    /***********************************************************/

    /**
     * \brief Internal implementation of the packing algorithm.
     * \details By Nicolas Ray. The algorithm is inspired by
     *  the Tetris game, and iteratively places the charts from
     *  bottom to top.
     */
    class TetrisPacker {
    public :

        /**
         * \brief TetrisPacker constructor.
         * \param[in] mesh a reference the surface mesh to be packed.
         *  It needs to have a vector attribute of dimension 2 attached
         *  to the facet corners and named "tex_coord", as well as a
         *  facet attribute named "chart".
         */
        TetrisPacker(Mesh& mesh) {
            nb_xpos_ = 1024;
            height_ = new double[size_t(nb_xpos_)];
            image_size_in_pixels_  = 1024;
            margin_width_in_pixels_ = 4;
            tex_coord_.bind_if_is_defined(
                mesh.facet_corners.attributes(), "tex_coord"
            );
            geo_assert(tex_coord_.is_bound() && tex_coord_.dimension() == 2);
            chart_attr_.bind_if_is_defined(
                mesh.facets.attributes(), "chart"
            );
            if(!chart_attr_.is_bound()) {
                mesh_get_charts(mesh);
                chart_attr_.bind_if_is_defined(
                    mesh.facets.attributes(), "chart"
                );
                geo_assert(chart_attr_.is_bound());
            }
        }

        /**
         * \brief TetrisPacker destructor.
         */
        ~TetrisPacker() {
            delete[] height_;
            height_ = nullptr;
        }

        void set_image_size_in_pixels(index_t size) {
            image_size_in_pixels_ = size;
        }

        index_t margin_width_in_pixels() const {
            return margin_width_in_pixels_;
        }

        void set_margin_width_in_pixels(index_t width) {
            margin_width_in_pixels_ = width;
        }

        void add(const ChartBBox& r){
            data_.push_back(r);
        }

        /**
         * \brief Compares two ChartBBox objects.
         * \details Used by the packing algorithm to sort
         *  the boxes.
         * \param[in] b0 , b1 const references to the
         *  two ChartBBox objects to be compared.
         * \retval true if \p b0 is taller than \p b1.
         * \retval false otherwise.
         */
        static bool compare(
            const ChartBBox& b0, const ChartBBox& b1
        ) {
            return (b0.size().y > b1.size().y);
        }

        void add_margin(double margin_size) {
            for (unsigned int i=0;i<data_.size();i++){
                data_[i].translate(
                    vec2(
                        -data_[i].min().x + margin_size,
                        -data_[i].min().y + margin_size
                    ),
                    tex_coord_
                );
                data_[i].max() = data_[i].max() +
                    vec2( 2.0 * margin_size, 2.0 * margin_size);
                geo_assert(data_[i].max().x>0);
                geo_assert(data_[i].max().y>0);
                data_[i].min() = vec2(0,0);
            }
        }

        double max_height() {
            double result = 0;
            for (unsigned int i=0;i<data_.size();i++) {
                result = std::max(result, data_[i].max().y);
            }
            return result;
        }

        void recursive_apply() {
            // compute margin
            double area = 0;
            for (unsigned int numrect = 0; numrect <data_.size();numrect++ ) {
                area += data_[numrect].area();
            }
            double margin =
                (::sqrt(area) / double(image_size_in_pixels_)) *
                double(margin_width_in_pixels_);
            add_margin(margin);

            // find a first solution
            apply(margin);
            double scoreSup = max_height();
            double borneSup = width_;
            double decalborne = 0.5 * ::sqrt(scoreSup * width_);

            // dichotomy
            for  (int i=0;i<10;i++ ){
                double new_borne = borneSup - decalborne;
                apply(margin,new_borne);
                double max = max_height();
                if (max < new_borne){
                    borneSup = borneSup - decalborne;
                }
                decalborne/=2;
            }
            apply(margin, borneSup);
        }


        void apply(double margin, double width =-1) {
            width_ = width;
            for (unsigned int i=0; i<data_.size(); i++) {
                data_[i].translate(
                    vec2(
                        -data_[i].min().x,
                        -data_[i].min().y
                    ),
                    tex_coord_
                );
            }

            // sort ChartBBoxes by their heights ...
            std::sort(data_.begin(),data_.end(),compare);

            { //find the best width
                double max_bbox_width = 0;
                for (unsigned int i=0; i<data_.size(); i++) {
                    max_bbox_width= std::max(
                        max_bbox_width, data_[i].size().x
                    );
                }

                if ( width == -1){
                    // try to have a square : width_ = sqrt(area);
                    double area =0;

                    for (unsigned int i=0; i<data_.size(); i++){
                        area += data_[i].area();
                    }

                    width_ = ::sqrt(area) *1.1;


                    // resize if a square seems to be too bad
                    // (the first piece is too high)

                    if (data_[0].size().y > width_) {
                        width_ = area / data_[0].size().y;
                    }
                }

                // be sure all surface can fit in the width
                width_ = std::max(
                    max_bbox_width * (double(nb_xpos_ + 2) / double(nb_xpos_)),
                    width_
                );
            }
            // set the step depending on the width and the discretisation
            step_ = width_ / nb_xpos_;


            // init local min and max height functions
            for (unsigned int numrect = 0; numrect <data_.size(); numrect++) {
                data_[numrect].init_max_and_min_func(
                    step_, margin, margin_width_in_pixels_,
                    tex_coord_, chart_attr_
                );
            }

            // init global height function
            for (int x=0; x<nb_xpos_; x++) {
                height(x)=0;
            }

            for (unsigned int i=0;i<data_.size();i++) {
                place(data_[i]);
            }
        }


    private:

        double& height(int x) {
            geo_assert(x >= 0 && x < nb_xpos_);
            return height_[x];
        }

        const double& height(int x) const {
            geo_assert(x >= 0 && x < nb_xpos_);
            return height_[x];
        }


        /**
         * \brief Inserts a new chart.
         * \details Follows the "tetris" strategy.
         * \param[in] rect the chart to be inserted.
         */
        void place(ChartBBox& rect) {

            const int width_in_pas = int ( (rect.size().x / step_) + 1 );

            // find the best position
            int bestXPos=0;
            double bestHeight = Numeric::max_float64();
            double bestFreeArea = Numeric::max_float64();

            for (int x=0;x<nb_xpos_ - width_in_pas;x++) {

                double localHeight = max_height(x,width_in_pas,rect);
                double localFreeArea =
                    free_area(x,width_in_pas,localHeight)
                    + localHeight * rect.size().x;

                // the best position criterion is the position that
                // minimize area lost
                // area could be lost under and upper the bbox
                if (localFreeArea < bestFreeArea){
                    bestXPos = x;
                    bestHeight = localHeight;
                    bestFreeArea = localFreeArea;
                }
            }

            // be sure we have a solution
            geo_assert(bestHeight != Numeric::max_float64());

            // place the rectangle
            rect.translate(vec2(bestXPos*step_,bestHeight),tex_coord_);
            for (int i=bestXPos;i<bestXPos + width_in_pas;i++){
                height(i) =  rect.min().y + rect.max_func(i-bestXPos);
            }
        }

        // find the max height in the range  [xpos,  xpos + width]
        double max_height(int xpos, int width, ChartBBox& rect){
            double result = 0;
            for (int i=xpos; i<xpos+width; i++) {
                result = std::max( result, height(i)-rect.min_func(i-xpos) );
            }
            return result;
        }

        // find the free area in the range under height_max in range
        // [xpos,  xpos + width]

        double free_area(int xpos, int width, double height_max) {
            double result =0;
            for (int i=xpos; i<xpos+width; i++) {
                result += step_ * (height_max - height(i));
            }
            return result;
        }

    private:
        index_t image_size_in_pixels_;
        index_t margin_width_in_pixels_;

        int nb_xpos_;
        double width_;
        double step_;
        double *height_;

        std::vector<ChartBBox> data_;

        Attribute<double> tex_coord_;
        Attribute<index_t> chart_attr_;
    };

}

/***************************************************************
 **** Tetris Packer                                         ****
 ***************************************************************/

namespace {
    using namespace GEO;

    // There where some problems with nan (Not a number),
    // this code is used to track such problems (seems to
    // be ok now)
    static bool chart_is_ok(Chart& chart, Attribute<double>& tex_coord) {
        for(index_t ff=0; ff<chart.facets.size(); ++ff) {
            index_t f = chart.facets[ff];
            for(index_t c: chart.mesh.facets.corners(f)) {
                if(Numeric::is_nan(tex_coord[2*c])) {
                    return false;
                }
                if(Numeric::is_nan(tex_coord[2*c+1])) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * \brief Packs a set of surfaces in parameter (texture) space.
     * \details This organizes the charts of a parameterization in
     *  a way that tries to minimize unused texture space.
     *  The algorithm, by Nicolas Ray, is described in the following
     *  reference:
     *  - least squares mode: Least Squares Conformal Maps,
     *    Levy, Petitjean, Ray, Maillot, ACM SIGGRAPH, 2002
     */
    class GEOGRAM_API Packer {
    public:

    /**
     * \brief Packer constructor.
     */
    Packer() {
        image_size_in_pixels_ = 1024;
        margin_width_in_pixels_ = 4;
    }

    /**
     * \brief Forbids copy.
     */
    Packer(const Packer& rhs) = delete;

    /**
     * \brief Forbids copy.
     */
    Packer& operator=(const Packer& rhs) = delete;

    /**
     * \brief Packs a texture atlas.
     * \param[in,out] mesh a surface mesh.
     * \details The mesh is supposed to have texture coordinates
     *  stored in a 2d vector attribute attached to the facet
     *  corners and named "tex_coord".
     */
    void pack_surface(Mesh& mesh, bool normalize_only=false) {
        tex_coord_.bind_if_is_defined(
            mesh.facet_corners.attributes(), "tex_coord"
        );
        geo_assert(tex_coord_.is_bound() && tex_coord_.dimension() == 2);
        chart_attr_.bind_if_is_defined(
            mesh.facets.attributes(), "chart"
        );
        if(!chart_attr_.is_bound()) {
            mesh_get_charts(mesh);
            chart_attr_.bind_if_is_defined(
                mesh.facets.attributes(), "chart"
            );
            geo_assert(chart_attr_.is_bound());
        }

        // Get the charts
        vector<Chart> charts;
        index_t nb_charts=0;
        for(index_t f: mesh.facets) {
            nb_charts = std::max(nb_charts, chart_attr_[f]);
        }
        ++nb_charts;

        for(index_t i=0; i<nb_charts; ++i) {
            charts.push_back(Chart(mesh, i));
        }

        for(index_t f: mesh.facets) {
            charts[chart_attr_[f]].facets.push_back(f);
        }

        Logger::out("Packer") << "Packing "
                              << charts.size() << " charts" << std::endl;

        // Sanity check
        for(index_t i=0; i<nb_charts; ++i) {
            if(!chart_is_ok(charts[i], tex_coord_)) {
                for(index_t ff=0; ff<charts[i].facets.size(); ++ff) {
                    index_t f = charts[i].facets[ff];
                    for(index_t c: mesh.facets.corners(f)) {
                        tex_coord_[2*c] = 0.0;
                        tex_coord_[2*c+1] = 0.0;
                    }
                }
            }
        }

        pack_charts(charts, normalize_only);

        // Normalize tex coords in [0,1] x [0,1]
        {
            double u_min, v_min, u_max, v_max;
            Geom::get_mesh_bbox_2d(
                mesh, tex_coord_, u_min, v_min, u_max, v_max
            );
            double l = std::max(u_max - u_min, v_max - v_min);
            if(l > 1e-6) {
                for(index_t c: mesh.facet_corners) {
                    tex_coord_[2*c]   = (tex_coord_[2*c] - u_min) / l;
                    tex_coord_[2*c+1] = (tex_coord_[2*c+1] - v_min) / l;
                }
            }
        }
        tex_coord_.unbind();
        chart_attr_.unbind();
    }

    /**
     * \brief Gets the size of the target texture image in
     *  pixels.
     * \return the size of the target texture image.
     */
    index_t image_size_in_pixels() const {
        return image_size_in_pixels_;
    }

    /**
     * \brief Gets the size of the margin (or "gutter") around the charts.
     * \details This may be required to avoid undesirable blends due to
     *  mip-mapping.
     * \return the number of empty pixels to be preserved around each chart.
     */
    index_t margin_width_in_pixels() const {
        return margin_width_in_pixels_;
    }

    protected:
    /**
     * \brief Packs a set of charts.
     * \param[in,out] charts a const reference to a vector with the
     *  charts to be packed.
     * \param[in] normalize_only if set, just normalize texture coordinates
     *  and do not pack the charts.
     * \details All the charts are supposed to be attached to the same mesh.
     *  Texture coordinates are stored in a 2d vector attribute attached to
     *  the facet corners of the mesh and called "tex_coord".
     */
    void pack_charts(vector<Chart>& charts, bool normalize_only = false) {

        if(charts.size() == 0) {
            return;
        }

        Mesh& mesh = charts[0].mesh;

        for(index_t i=0; i<charts.size(); ++i) {
            geo_assert(chart_is_ok(charts[i], tex_coord_));
            normalize_chart(charts[i]);
            geo_assert(chart_is_ok(charts[i], tex_coord_));
        }

        if(normalize_only) {
            return;
        }

        // use the tetris packer (more efficient for large dataset)
        // set some application dependent const
        TetrisPacker pack(mesh);
        pack.set_image_size_in_pixels(image_size_in_pixels());
        pack.set_margin_width_in_pixels(margin_width_in_pixels());

        for(index_t i=0; i<charts.size(); ++i) {
            geo_assert(chart_is_ok(charts[i], tex_coord_));
            double u_min, v_min, u_max, v_max;
            Geom::get_chart_bbox_2d(
                charts[i], tex_coord_, u_min, v_min, u_max, v_max
            );

            geo_assert(!Numeric::is_nan(u_min));
            geo_assert(!Numeric::is_nan(v_min));
            geo_assert(!Numeric::is_nan(u_max));
            geo_assert(!Numeric::is_nan(v_max));
            geo_assert(u_max >= u_min);
            geo_assert(v_max >= v_min);

            ChartBBox r(
                &charts[i],
                vec2(u_min,v_min) ,
                vec2(u_max,v_max)
            );
            pack.add(r);
        }


        pack.recursive_apply();

        {
            double total_area = Geom::mesh_area_2d(mesh, tex_coord_);
            double u_min, v_min, u_max, v_max;
            Geom::get_mesh_bbox_2d(
                mesh, tex_coord_, u_min, v_min, u_max, v_max
            );
            double bbox_area = (u_max - u_min)*(v_max-v_min);
            double filling_ratio = total_area / bbox_area;
            Logger::out("Packer") << "BBox area:"  << bbox_area << std::endl;
            Logger::out("Packer") << "Filling ratio:"
                                  << filling_ratio << std::endl;
        }
    }

    /**
     * \brief Normalizes the parameterization of a chart.
     * \param[in,out] chart a reference to the chart to be normalized.
     * \details This rescales texture coordinates in such a way that the
     *  chart has the same area in 3D and in texture space.
     *  This also applies a rotation to the texture coordinates such that
     *  the area of the bounding rectangle is minimized.
     *  Texture coordinates are stored in a 2d vector attribute attached
     *  to the facet corners of the mesh and called "tex_coord".
     */
    void normalize_chart(Chart& chart) {
        // TODO: choose best axes, rotate chart.
        vec2 U(1.0, 0.0);
        vec2 V(0.0, 1.0);
        AverageDirection2d dir;
        dir.begin();
        for(index_t ff=0; ff<chart.facets.size(); ++ff) {
            index_t f = chart.facets[ff];
            for(index_t c1: chart.mesh.facets.corners(f)) {
                index_t adj_f = chart.mesh.facet_corners.adjacent_facet(c1);
                if(adj_f == NO_FACET || chart_attr_[adj_f] != chart.id) {
                    index_t c2 =
                        chart.mesh.facets.next_corner_around_facet(f,c1);
                    vec2 uv1(tex_coord_[2*c1], tex_coord_[2*c1+1]);
                    vec2 uv2(tex_coord_[2*c2], tex_coord_[2*c2+1]);
                    dir.add_vector(uv2-uv1);
                }
            }
        }
        dir.end();
        vec2 W=dir.average_direction();
        if(!Geom::has_nan(W)) {
            W = normalize(W);
            V = W;
            U = vec2(-V.y, V.x);
        }

        for(index_t ff=0; ff<chart.facets.size(); ++ff) {
            index_t f = chart.facets[ff];
            for(index_t c: chart.mesh.facets.corners(f)) {
                vec2 uv(tex_coord_[2*c], tex_coord_[2*c+1]);
                tex_coord_[2*c] = dot(uv,U);
                tex_coord_[2*c+1] = dot(uv,V);
            }
        }

        double area3d = Geom::chart_area(chart);
        double area2d = Geom::chart_area_2d(chart, tex_coord_);
        double factor = 1.0;
        if(::fabs(area2d) > 1e-30) {
            factor = ::sqrt(area3d/area2d);
        } else {
            factor = 0.0;
        }

        geo_assert(!Numeric::is_nan(area2d));
        geo_assert(!Numeric::is_nan(area3d));
        geo_assert(!Numeric::is_nan(factor));

        for(index_t ff=0; ff<chart.facets.size(); ++ff) {
            index_t f = chart.facets[ff];
            for(index_t c: chart.mesh.facets.corners(f)) {
                tex_coord_[2*c] *= factor;
                tex_coord_[2*c+1] *= factor;
            }
        }
    }

    private:
    index_t image_size_in_pixels_ ;
    index_t margin_width_in_pixels_ ;
    Attribute<double> tex_coord_;
    Attribute<index_t> chart_attr_;
    } ;
}

namespace GEO {

    void pack_atlas_only_normalize_charts(Mesh& mesh) {
        Packer tetris;
        bool normalize_tex_coords_only = true;
        tetris.pack_surface(mesh, normalize_tex_coords_only);
    }

    void pack_atlas_using_tetris_packer(Mesh& mesh) {
        Packer tetris;
        tetris.pack_surface(mesh);
    }
}
