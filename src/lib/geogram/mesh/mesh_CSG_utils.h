/*
 *  Copyright (c) 2000-2025 Inria
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

#ifndef H_GEOGRAM_MESH_MESH_CSG_UTILS_H
#define H_GEOGRAM_MESH_MESH_CSG_UTILS_H

#include <geogram/basic/common.h>
#include <geogram/mesh/mesh.h>
#include <string>
#include <filesystem>
#include <memory>

namespace GEOCSG {

    using namespace GEO;

    /**********************************************************************/
    /**** Values, ArgList                                              ****/
    /**********************************************************************/

    /**
     * \brief A parsed value in a .csg file
     * \details Can be a number, a boolean, a 1d array or a 2d array
     */
    struct Value {
        enum Type {NONE, NUMBER, BOOLEAN, ARRAY1D, ARRAY2D, STRING, PATH};

        Value();
        Value(double x);
        Value(int x);
        Value(bool x);
        Value(const std::string& x);
	Value(const std::filesystem::path& x);
        std::string to_string() const;

        Type type;
        bool boolean_val;
        double number_val;
        vector<vector<double> > array_val;
        std::string string_val;
    };

    /**
     * \brief A parsed argument list in a .csg file.
     * \details Stores name-value pairs.
     */
    class ArgList {
    public:
        typedef std::pair<std::string, Value> Arg;

        index_t size() const {
            return args_.size();
        }

        const std::string& ith_arg_name(index_t i) const {
            geo_debug_assert(i < size());
            return args_[i].first;
        }

        const Value& ith_arg_val(index_t i) const {
            geo_debug_assert(i < size());
            return args_[i].second;
        }

        void add_arg(const std::string& name, const Value& value);
        bool has_arg(const std::string& name) const;
        const Value& get_arg(const std::string& name) const;
        double get_arg(const std::string& name,double default_val) const;
        int get_arg(const std::string& name, int default_val) const;
        bool get_arg(const std::string& name, bool default_val) const;
        vec2 get_arg(const std::string& name, vec2 default_val) const;
        vec3 get_arg(const std::string& name, vec3 default_val) const;
        vec4 get_arg(const std::string& name, vec4 default_val) const;
        mat4 get_arg(const std::string& name, const mat4& default_val) const;
        std::string get_arg(
	    const std::string& name, const std::string& default_val
        ) const;
        std::string get_arg(
            const std::string& name, const char* default_val
        ) const {
	    return get_arg(name, std::string(default_val));
	}
    private:
        vector<Arg> args_;
    };

    /**********************************************************************/
    /**** General sweeping function                                    ****/
    /**********************************************************************/

    /**
     * \brief Symbolic constants for sweep()
     */
    enum SweepCapping {
	SWEEP_CAP,
	SWEEP_POLE,
	SWEEP_PERIODIC
    };

    /**
     * \brief The generalized sweeping operation
     * \details Used to implement sphere(), cylinder(), linear_extrude() and
     *  rotate_extrude()
     * \param[in,out] M on entry, a 2D mesh. On exit, a 3D mesh. The triangles
     *  present in the mesh are used to generate the caps. They are copied to
     *  generate the second cap if \p capping is set to SWEEP_CAP (default).
     * \param[in] nv number of sweeping steps. Minimum is 2.
     * \param[in] sweep_path a function that maps u,v indices to 3D
     *  points, where u is the index of a initial 2D vertex and v
     *  in [0..nv-1] the sweeping step. One can use the point at vertex
     *  u to evaluate the path (it will not be overwritten before calling
     *  sweep_path()). Note that u vertices are not necessarily ordered.
     * \param[in] capping one of:
     *   - SWEEP_CAP standard sweeping, generate second capping by
     *     copying first one
     *   - SWEEP_POLE if last sweeping step degenerates to a
     *     single point
     *   - SWEEP_PERIODIC if no cappings should be generated and last
     *     sweeping step corresponds to first one
     */
     void GEOGRAM_API sweep(
	std::shared_ptr<Mesh>& M, index_t nv,
	std::function<vec3(index_t, index_t)> sweep_path,
	SweepCapping capping = SWEEP_CAP
    );

    /**
     * \brief keeps only triangles and vertices embedded in the z=0 plane, and
     *  makes the mesh 2D.
     * \param[in,out] M a shared pointer to the mesh
     */
    void GEOGRAM_API keep_z0_only(std::shared_ptr<Mesh>& M);

    /**********************************************************************/
    /**** Call OpenSCAD for help (and cache result)                    ****/
    /**********************************************************************/

    /**
     * \brief Specifies that starting from now all cached OpenSCAD files
     *  are considered to be out of date and will be re-generated.
     */
    void GEOGRAM_API OpenSCAD_cache_invalidate();

    /**
     * \brief Specifies that last modification time should be ignored when
     *  considering the OpenSCAD cache.
     * \details It is interesting to do so for testsuites that embark the
     *  OpenSCached directory for users who cannot install OpenSCAD.
     */
    void GEOGRAM_API OpenSCAD_cache_ignore_time();

    std::shared_ptr<Mesh> GEOGRAM_API call_OpenSCAD(
	const std::filesystem::path& path, const std::string& command,
	const ArgList& args, bool TWO_D=false
    );

    std::string GEOGRAM_API load_OpenSCAD(const std::filesystem::path& filename);

    /**********************************************************************/
    /**** Functions to estimate number of fragments and slices,        ****/
    /****  taken from OpenSCAD                                         ****/
    /**********************************************************************/

    /**
     * \brief Taken from OpenSCAD
     */
    int GEOGRAM_API get_fragments_from_r_and_twist(
	double r, double twist, double fn, double fs, double fa
    );

    /**
     * \brief Taken from OpenSCAD
     */
    inline int get_fragments_from_r(
	double r, double fn, double fs, double fa
    ) {
	return get_fragments_from_r_and_twist(r, 360.0, fn, fs, fa);
    }

    /**
     * \brief Taken from OpenSCAD
     */
    int GEOGRAM_API get_linear_extrusion_slices(
	std::shared_ptr<Mesh> M, double height, vec2 scale, double twist,
	double fn, double fs, double fa
    );

}

#endif
