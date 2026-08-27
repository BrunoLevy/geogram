/*
 *  Copyright (c) 2000-2023 Inria
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

#ifndef H_GEO_MESH_CSG_BUILDER_H
#define H_GEO_MESH_CSG_BUILDER_H

#include <geogram/basic/common.h>
#include <geogram/mesh/mesh_CSG_utils.h>
#include <memory>
#include <functional>
#include <stack>

namespace GEO {

    /**
     * \brief Base class for implementing CSG objects and instructions.
     * \details AbstractCSGBuilder implement the basic mechanism for
     *  calling generic objects and CSG instructions that take an ArgList
     *  (name value pairs) as arguments. It dispatches objects/instructions
     *  from their name (as a string) and unpacks the arguments from the
     *  ArgList. It is interesting to have an abstract base class for that,
     *  because one can have a subclass that records the CSG
     *  abstract syntax tree, for subsequently optimizing it before playing
     *  it back.
     */
    class GEOGRAM_API AbstractCSGBuilder {
    public:
    typedef GEOCSG::ArgList ArgList;
    typedef GEOCSG::Value Value;

    /** \see set_fa() */
    static constexpr double DEFAULT_FA = 12.0;

    /** \see set_fs() */
    static constexpr double DEFAULT_FS = 2.0;

    /** \see set_fn() */
    static constexpr double DEFAULT_FN = 0.0;

    /**
     * \brief AbstractCSGBuilder constructor
     */
    AbstractCSGBuilder();

    /**
     * \brief AbstractCSGBuilder destructor
     */
    virtual ~AbstractCSGBuilder();

    /****** Parameters ******/

    /**
     * \brief Resets defaults value for fn, fs, fa
     * \see set_fn(), set_fs(), set_fa()
     */
    void reset_defaults();

    /**
     * \brief Sets the number of fragments.
     * \details This corresponds to the number of edges in a polygonal
     *  approximation of a circle. If left to 0, it is automatically
     *  computed from fs and fa
     * \param[in] fn the number of fragments.
     * \see set_fs(), set_fa()
     */
    void set_fn(double fn) {
        fn_ = std::max(fn, 0.0);
    }

    /**
     * \brief Sets the minimum size for a fragment.
     * \param[in] fs minimum size for a fragment.
     * \details This determines the number of edges in a polygonal
     *  approximation of a circle.
     */
    void set_fs(double fs) {
        fs_ = std::max(fs,0.01);
    }

    /**
     * \brief Sets the minimum angle for a fragment.
     * \param[in] fa minimum angle for a fragment, in degrees.
     * \details This determines the number of edges in a polygonal
     *  approximation of a circle.
     */
    void set_fa(double fa) {
        fa_ = std::max(fa,0.01);
    }

    /**
     * \brief Displays (lots of) additional information
     * \param[in] x whether additional information should be displayed.
     *  Default is off
     */
    void set_verbose(bool x) {
        verbose_ = x;
	if(verbose_) {
	    warnings_ = true;
	}
    }

    /**
     * \brief Displays (even more) additional information
     * \param[in] x whether even more information should be displayed.
     *  Default is off
     */
    void set_detailed_verbose(bool x) {
        detailed_verbose_ = x;
	if(detailed_verbose_ && !verbose_) {
	    verbose_ = true;
	}
    }

    /**
     * \brief Tests wheter verbose mode is set.
     * \retval true if additional information will be displayed.
     * \retval false otherwise.
     */
    bool verbose() const {
        return verbose_;
    }


    /**
     * \brief Adds a path to the file path
     * \details The file path is where import() searches files. The default
     *  file path contains the current directory "."
     * \param[in] path the file path to be added, without trailing '/'
     */
    void add_file_path(const std::filesystem::path& path) {
        file_path_.push_back(path);
    }

    /**
     * \brief Resets the file path to its default value, with only the
     *  current directory "."
     */
    void reset_file_path() {
        file_path_.clear();
        file_path_.push_back(std::filesystem::current_path());
    }

    /**
     * \brief Adds a path to the file path
     * \details The file path is where import() searches files. The default
     *  file path contains the current directory "."
     * \param[in] path the file path to be added, without trailing '/'
     */
    void push_file_path(const std::filesystem::path& path) {
	file_path_.push_back(path);
    }

    /**
     * \brief Removes the latest pushed file path
     */
    void pop_file_path() {
	geo_assert(file_path_.size() != 0);
	file_path_.pop_back();
    }

    /**** Abstract API - objects ******/

    virtual void add_object(
	const std::string& object, const ArgList& args
    );

    virtual void begin_instruction();

    virtual void end_instruction(
	const std::string& instruction, const ArgList& args
    );


    bool is_object(const std::string& id) const {
        return (object_funcs_.find(id) != object_funcs_.end());
    }

    bool is_instruction(const std::string& id) const {
        return (instruction_funcs_.find(id) != instruction_funcs_.end());
    }

    protected:

    /****** Objects ******************************************/

    virtual void add_square(const ArgList& args);
    virtual void add_circle(const ArgList& args);
    virtual void add_cube(const ArgList& args);
    virtual void add_sphere(const ArgList& args);
    virtual void add_cylinder(const ArgList& args);
    virtual void add_polyhedron(const ArgList& args);
    virtual void add_polygon(const ArgList& args);
    virtual void add_import(const ArgList& args);
    virtual void add_surface(const ArgList& args);
    virtual void add_text(const ArgList& args);

    /****** Instructions ************************************/

    virtual void eval_multmatrix(const ArgList& args);
    virtual void eval_translate(const ArgList& args);
    virtual void eval_rotate(const ArgList& args);
    virtual void eval_scale(const ArgList& args);
    virtual void eval_resize(const ArgList& args);
    virtual void eval_union(const ArgList& args);
    virtual void eval_intersection(const ArgList& args);
    virtual void eval_difference(const ArgList& args);
    virtual void eval_group(const ArgList& args);
    virtual void eval_color(const ArgList& args);
    virtual void eval_hull(const ArgList& args);
    virtual void eval_linear_extrude(const ArgList& args);
    virtual void eval_rotate_extrude(const ArgList& args);
    virtual void eval_projection(const ArgList& args);
    virtual void eval_minkowski(const ArgList& args);
    virtual void eval_render(const ArgList& args);

    /**************************/

    [[noreturn]] void error(const char* str) {
	throw(std::logic_error(str));
    }

    [[noreturn]] void error(const std::string& str) {
	throw(std::logic_error(str.c_str()));
    }

    double fn_;
    double fs_;
    double fa_;

    std::vector<std::filesystem::path> file_path_;
    bool warnings_;
    bool verbose_;
    bool detailed_verbose_;

    typedef std::function<void(const ArgList& args)> csg_builder_func;
    std::map<std::string, csg_builder_func> object_funcs_;
    std::map<std::string, csg_builder_func> instruction_funcs_;
    };

    /***********************************************************************/

    /**
     * \brief A Scope corresponds to a set of primitive between curly braces
     *  in OpenSCAD, arguments of an operation. It is implemented as a vector
     *  of meshes.
     */
    class CSGScope {
    public:
	/**
	 * \brief Constructs an empty CSGScope
	 */
	CSGScope() {
	}

	/**
	 * \brief Constructs a CSGScope that contains a single mesh
	 * \param[in] M a shared_ptr to the mesh
	 */
	CSGScope(std::shared_ptr<Mesh> M) {
	    emplace_back(M);
	}

	/**
	 * \brief Constructs a CSGScope from a list of meshes
	 * \param[in] Ms the list of meshes (specified between curly braces)
	 */
        CSGScope(
	    const std::initializer_list<std::shared_ptr<Mesh>>& Ms
	) : meshes_(Ms) {
	}

	void push_back(std::shared_ptr<Mesh> M) {
	    meshes_.push_back(M);
	}

	void emplace_back(std::shared_ptr<Mesh> M) {
	    meshes_.emplace_back(M);
	}

	void pop_back() {
	    meshes_.pop_back();
	}

	index_t size() const {
	    return index_t(meshes_.size());
	}

	const std::shared_ptr<Mesh>& operator[](index_t i) const {
	    geo_debug_assert(i < size());
	    return meshes_[i];
	}

	auto begin() { return meshes_.begin(); }
	auto end() { return meshes_.end(); }
	auto begin() const { return meshes_.begin(); }
	auto end() const { return meshes_.end(); }
	auto rbegin() { return meshes_.rbegin(); }
	auto rend() { return meshes_.rend(); }
	auto rbegin() const { return meshes_.rbegin(); }
	auto rend() const { return meshes_.rend(); }

	void append(std::shared_ptr<Mesh> M) {
	    emplace_back(M);
	}

	template <typename...Types> void append(
	    std::shared_ptr<Mesh> head, Types... tail
	) {
	    append(head);
	    append(tail...);
	}

    private:
	std::vector<std::shared_ptr<Mesh>> meshes_;
    };

    /**
     * \brief Implements CSG objects and instructions.
     * \details Can be used to construct volumes in C++ with a syntax
     *  very similar to OpenSCAD .csg files.
     */
    class GEOGRAM_API CSGBuilder : public AbstractCSGBuilder {
    public:
    CSGBuilder();
    ~CSGBuilder() override;

    /****** Objects **********/

    virtual std::shared_ptr<Mesh> square(
	vec2 size = vec2(1.0,1.0), bool center=false
    );

    /**
     * \param[in] nu number of fragments. If left unspecified, then it
     *   is deduced from radius and/or $fn variable.
     */
    virtual std::shared_ptr<Mesh> circle(double r=1.0, index_t nu=0);

    virtual std::shared_ptr<Mesh> cube(
	vec3 size = vec3(1.0, 1.0, 1.0), bool center=false
    );

    virtual std::shared_ptr<Mesh> sphere(double r=1.0);

    virtual std::shared_ptr<Mesh> cylinder(
        double h=1.0, double r1=1.0, double r2=1.0, bool center=false
    );

    virtual std::shared_ptr<Mesh> import(
        const std::filesystem::path& filename, const std::string& layer="",
        index_t timestamp=0,
        vec2 origin = vec2(0.0, 0.0), vec2 scale = vec2(1.0,1.0)
    );

    virtual std::shared_ptr<Mesh> surface(
        const std::filesystem::path& filename, bool center, bool invert
    );

    virtual std::shared_ptr<Mesh> text(
	const std::string& text,
	double size = 10.0,
	const std::string& font = "",
	const std::string& halign = "left",
	const std::string& valign = "baseline",
	double spacing = 1.0,
	const std::string& direction = "ltr",
	const std::string& language = "en",
	const std::string& script = "latin"
    );


    /****** Instructions ****/

    /**
     * \brief Groups several meshes into a single one and transforms
     *   them.
     * \param[in] M the transformation matrix. It follows the same
     *   convention as OpenSCAD, that is, not the OpenGL convention.
     *   For instance, a translation matrix has the translation vector
     *   as its third column.
     * \param[in] scope one or several meshes to be merged.
     */
    virtual std::shared_ptr<Mesh> multmatrix(
	const mat4& M, const CSGScope& scope
    );

    /**
     * \brief Computes the union of two or more meshes.
     * \param[in] scope the meshes. One can use a CSGScope object, or a
     *   curly-braced list of meshes (list of shared_ptr to meshes in fact).
     * \return a mesh with the union of the meshes in \p scope.
     */
    virtual std::shared_ptr<Mesh> union_instr(const CSGScope& scope);

    /**
     * \brief Computes the union of two or more meshes.
     * \param[in] args the list of meshes to be unioned,
     *  as the function parameters (this function takes an arbitrary number
     *  of parameters).
     * \return a mesh with the union of the meshes in \p args.
     */
    template <typename...Types> std::shared_ptr<Mesh> union_instr(Types...args) {
	CSGScope scope;
	scope.append(args...);
	return union_instr(scope);
    }

    /**
     * \brief Computes the intersection between two or more meshes.
     * \param[in] scope the meshes
     * \return a mesh with the intersection of the meshes in \p scope
     */
    virtual std::shared_ptr<Mesh> intersection(const CSGScope& scope);

    /**
     * \brief Computes the mutual intersection between meshes
     * \param[in] args the list of meshes to be intersected
     *  as the function parameters (this function takes an arbitrary number
     *  of parameters).
     * \return a mesh with the mutual intersection of the meshes in \p args
     */
    template <typename...Types> std::shared_ptr<Mesh>
    intersection(Types...args) {
	CSGScope scope;
	scope.append(args...);
	return intersection(scope);
    }

    /**
     * \brief Computes the intersection between two meshes.
     * \details If \p scope contains more than two meshes, it computes
     *   the difference between the first mesh and the union of the rest.
     * \param[in] scope the meshes
     */
    virtual std::shared_ptr<Mesh> difference(const CSGScope& scope);

    /**
     * \brief Computes the difference between meshes
     * \details If \p args contains more than two meshes, it computes
     *   the difference between the first mesh and the union of the rest.
     * \param[in] args the list of meshes
     *  as the function parameters (this function takes an arbitrary number
     *  of parameters).
     * \return a mesh with the difference between the meshes in \p args
     */
    template <typename...Types> std::shared_ptr<Mesh>
    difference(Types...args) {
	CSGScope scope;
	scope.append(args...);
	return difference(scope);
    }

    /**
     * \brief synonym for union.
     * \details Maybe there's something I did not understand in
     *  OpenSCAD, but I do not see the difference between group
     *  and union.
     */
    virtual std::shared_ptr<Mesh> group(const CSGScope& scope);

    /**
     * \brief Groups several meshes into a single one and sets their
     *   color.
     * \param[in] color the color, as r,g,b,a.
     * \details ignored for now, just behaves as group().
     */
    virtual std::shared_ptr<Mesh> color(vec4 color, const CSGScope& scope);

    /**
     * \brief Computes the convex hull of several meshes.
     * \param[in] scope the meshes
     */
    virtual std::shared_ptr<Mesh> hull(const CSGScope& scope);

    /**
     * \brief Computes a 3D extrusion from a 2D shape
     * \param[in] scope one or more 2D shapes
     * \param[in] height total height of the extrusion
     * \param[in] center if set, z will go from -height/2 to height/2,
     *   else from 0 to height
     * \param[in] scale scaling factor to be applied to x and y coordinates
     *   when reaching \p height
     * \param[in] slices number of slices along the z axis
     * \param[in] twist rotation to be applied when sweeping, in degrees
     */
    virtual std::shared_ptr<Mesh> linear_extrude(
        const CSGScope& scope,
        double height = 1.0,
        bool center = false,
        vec2 scale = vec2(1.0,1.0),
        index_t slices = 0,
        double twist = 0.0
    );

    /**
     * \brief Computes a 3D extrusion from a 2D shape
     * \param[in] scope one or more 2D shapes. Everything should be on the
     *   same side of the Y axis, preferably the positive side.
     * \param[in] angle optional angle
     */
    virtual std::shared_ptr<Mesh> rotate_extrude(
	const CSGScope& scope, double angle = 360.0
    );

    /**
     * \brief Creates a 2D mesh from 3D mesh.
     * \param[in] cut if set, computes the boundary of the intersection
     *   between the object and the X,Y plane, else computes the boundary
     *   of the projection.
     */
    virtual std::shared_ptr<Mesh> projection(const CSGScope& scope, bool cut);

    /**
     * \brief Computes the Minkowski sum of meshes.
     */
    virtual std::shared_ptr<Mesh> minkowski(const CSGScope& scope);

    /**
     * \brief Appends all meshes in scope into a unique mesh,
     *  without testing for intersections.
     * \details Prepares operand bits for a subsequent CSG operation.
     */
    virtual std::shared_ptr<Mesh> append(const CSGScope& scope);

    /****** AbstractCSGBuilder API ********************************/

    void add_object(const std::string& object, const ArgList& args) override;
    void begin_instruction() override;
    void end_instruction(
	const std::string& instruction, const ArgList& args
    ) override;

    /****** Objects (AbstractCSGBuilder API) *********************/

    void add_square(const ArgList& args) override;
    void add_circle(const ArgList& args) override;
    void add_cube(const ArgList& args) override;
    void add_sphere(const ArgList& args) override;
    void add_cylinder(const ArgList& args) override;
    void add_polyhedron(const ArgList& args) override;
    void add_polygon(const ArgList& args) override;
    void add_import(const ArgList& args) override;
    void add_surface(const ArgList& args) override;
    void add_text(const ArgList& args) override;

    /****** Instructions (AbstractCSGBuilder API) ****************/

    void eval_multmatrix(const ArgList& args) override;
    void eval_translate(const ArgList& args) override;
    void eval_rotate(const ArgList& args) override;
    void eval_scale(const ArgList& args) override;
    void eval_resize(const ArgList& args) override;
    void eval_union(const ArgList& args) override;
    void eval_intersection(const ArgList& args) override;
    void eval_difference(const ArgList& args) override;
    void eval_group(const ArgList& args) override;
    void eval_color(const ArgList& args) override;
    void eval_hull(const ArgList& args) override;
    void eval_linear_extrude(const ArgList& args) override;
    void eval_rotate_extrude(const ArgList& args) override;
    void eval_projection(const ArgList& args) override;
    void eval_minkowski(const ArgList& args) override;
    void eval_render(const ArgList& args) override;

    /****** Commodity functions **********************************/

    /**
     * \brief Builds a translation matrix
     * \param[in] T the translation matrix
     * \return a 4x4 homogeneous coordinate translation matrix
     */
    static mat4 translation_matrix(const vec3& T) {
	return mat4{{1, 0, 0, T.x},
		    {0, 1, 0, T.y},
		    {0, 0, 1, T.z},
		    {0, 0, 0, 1  }};
    }

    /**
     * \brief Builds a scaling matrix
     * \param[in] sx , sy , sz scaling along each axis
     * \return a 4x4 homogeneous coordinate scaling matrix
     */
    static mat4 scaling_matrix(double sx, double sy, double sz) {
	return mat4{{sx, 0,  0,  0},
		    {0,  sy, 0,  0},
		    {0,  0,  sz, 0},
		    {0,  0,  0,  1}};
    }

    /**
     * \brief Builds a rotation matrix
     * \param[in] axis rotation axis
     * \param[in] angle rotation angle in degrees
     * \return a 4x4 homogeneous coordinate rotation matrix
     */
    static mat4 rotation_matrix(double angle, vec3 axis = {0.0, 0.0, 0.0}) {
	if(axis.x == 0.0 && axis.y == 0.0 && axis.z == 0.0) {
	    axis.z = 1.0; // special case, OpenSCAD convention
	}
	vec3 N = normalize(axis);
	double x = N.x;
	double y = N.y;
	double z = N.z;
	angle = angle * M_PI / 180;
	double s = sin(angle);
	double c = cos(angle);
	double t = 1.0-c;
	return mat4{{t*x*x + c,   t*x*y - s*z, t*x*z + s*y, 0.0},
		    {t*x*y + s*z, t*y*y + c,   t*y*z - s*x, 0.0},
		    {t*x*z - s*y, t*y*z + s*x, t*z*z + c,   0.0},
		    {0.0,         0.0,         0.0,         1.0}};
    }

    /**
     * \brief Builds a rotation matrix
     * \param[in] angles rotation angles around each axis in degrees
     * \return a 4x4 homogeneous coordinate rotation matrix
     */
    static mat4 rotation_matrix(const vec3& angles) {
	double a = angles[0] * M_PI / 180.0;
	double b = angles[1] * M_PI / 180.0;
	double c = angles[2] * M_PI / 180.0;
	double sa = sin(a); double ca = cos(a);
	double sb = sin(b); double cb = cos(b);
	double sc = sin(c); double cc = cos(c);
	return mat4{{cc*cb,  cc*sb*sa - sc*ca,  cc*sb*ca + sc*sa,  0.0},
		    {sc*cb,  sc*sb*sa + cc*ca,  sc*sb*ca - cc*sa,  0.0},
		    {  -sb,             cb*sa,             cb*ca,  0.0},
		    {  0.0,               0.0,               0.0,  1.0}};
    }

    /**
     * \brief Commodity function for translating a CSGScope using OpenSCAD syntax
     * \param[in] T translation vector
     * \param[in] scope a scope with the list of meshes to be transformed
     * \return a mesh with the union of the transformed meshes in \p scope
     * \see multmatrix
     */
    std::shared_ptr<Mesh> translate(const vec3& T, const CSGScope& scope) {
	return multmatrix(translation_matrix(T),scope);
    }

    /**
     * \brief Commodity function for 2D-rotating a CSGScope using OpenSCAD syntax
     * \param[in] axis rotation axis
     * \param[in] angle rotation angle in degrees
     * \param[in] scope a scope with the list of meshes to be transformed
     * \return a mesh with the union of the transformed meshes in \p scope
     * \see multmatrix
     */
    std::shared_ptr<Mesh> rotate(double angle, const CSGScope& scope) {
	return multmatrix(rotation_matrix(angle),scope);
    }

    /**
     * \brief Commodity function for rotating a CSGScope using OpenSCAD syntax
     * \param[in] axis rotation axis
     * \param[in] angle rotation angle in degrees
     * \param[in] scope a scope with the list of meshes to be transformed
     * \return a mesh with the union of the transformed meshes in \p scope
     * \see multmatrix
     */
    std::shared_ptr<Mesh> rotate(
	double angle, const vec3& axis, const CSGScope& scope
    ) {
	return multmatrix(rotation_matrix(angle,axis),scope);
    }

    /**
     * \brief Commodity function for rotating a CSGScope using OpenSCAD syntax
     * \param[in] angles rotation angles along the three axes in degrees
     * \param[in] scope a scope with the list of meshes to be transformed
     * \return a mesh with the union of the transformed meshes in \p scope
     * \see multmatrix
     */
    std::shared_ptr<Mesh> rotate(const vec3& angles, const CSGScope& scope) {
	return multmatrix(rotation_matrix(angles),scope);
    }

    /**
     * \brief Commodity function for scaling a CSGScope using OpenSCAD syntax
     * \param[in] sx , sy , sz scaling factors along each axis
     * \param[in] scope a scope with the list of meshes to be transformed
     * \return a mesh with the union of the transformed meshes in \p scope
     * \see multmatrix
     */
    std::shared_ptr<Mesh> scale(
	double sx, double sy, double sz, const CSGScope& scope
    ) {
	return multmatrix(scaling_matrix(sx,sy,sz),scope);
    }

    /**************************/

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
     * \brief Specifies whether coplanar facets should be simplified
     * \param[in] x if set, coplanar facets are simplified, else they
     *  are kept as is (faster but generates many triangles). Default
     *  is set.
     * \param[in] angle_tolerance (in degree) the pairs of
     *  adjacent facets with normals that make an angle smaller than
     *  this threshold as considered to be coplanar.
     */
    void set_simplify_coplanar_facets(bool x, double angle_tolerance=0.0) {
        simplify_coplanar_facets_ = x;
        coplanar_angle_tolerance_ = angle_tolerance;
    }

    /**
     * \brief Sets fast union mode
     * \details In fast union mode, all intersections are computed and the
     *  external shell is kept. It may give incorrect result if an object
     *  is floating inside another one (completely included).
     * \param[in] x true if fast union mode should be used, false otherwise.
     */
    void set_fast_union(bool x) {
        fast_union_ = x;
    }


    /***** misc ********/

    /**
     * \brief Sets noop mode
     * \details In noop mode, all CSG operations (union, intersection,
     *  difference) are replaced with append. Useful for debugging CSG trees.
     * \param[in] x whether noop mode should be set
     */
    void set_noop(bool x) {
	noop_ = x;
    }


    /**
     * \brief Computes the bounding box of a mesh
     * \param[in] mesh the mesh
     * \return the bounding box. If the mesh is 2D, then the z components
     *  of the bounding box are set to 0
     */
    static Box3d get_bbox(const std::shared_ptr<Mesh>& mesh);

    /**
     * \brief Computes the bounding box of a mesh
     * \param[in] mesh the mesh
     * \return the bounding box as a pair of minimum, maximum bounds.
     *  If the mesh is 2D, then the z components of the bounding box
     *  are set to 0
     */
    static std::pair<vec3, vec3> get_bbox_bounds(
	const std::shared_ptr<Mesh>& mesh
    ) {
	Box3d result = get_bbox(mesh);
	return std::make_pair(vec3(result.xyz_min), vec3(result.xyz_max));
    }

    protected:

    std::shared_ptr<Mesh> surface_with_OpenSCAD(
        const std::filesystem::path& filename, bool center, bool invert
    );

    std::shared_ptr<Mesh> text_with_OpenSCAD(
	const std::string& text,
	double size = 10.0,
	const std::string& font = "",
	const std::string& halign = "left",
	const std::string& valign = "baseline",
	double spacing = 1.0,
	const std::string& direction = "ltr",
	const std::string& language = "en",
	const std::string& script = "latin"
    );

    /**** Lower-level functions ****/

    /**
     * \brief Finds a file in the path
     * \param[in,out] filename the file to be found. On exit, the complete
     *   path to the file if found
     * \retval true if the file could be found
     * \retval false otherwise
     */
    bool find_file(std::filesystem::path& filename);

    /**
     * \brief Gets the current path
     * \return the latest directory pushed onto the file path
     */
    const std::filesystem::path& current_path() {
	geo_assert(file_path_.size() != 0);
	return *(file_path_.rbegin());
    }

    /**
     * \brief For the file formats that are not supported by geogram,
     *  get help from OpenSCAD to convert them.
     * \details Converts STEP files.
     */
    std::shared_ptr<Mesh> import_with_openSCAD(
        const std::filesystem::path& filename, const std::string& layer="",
        index_t timestamp=0
    );

    /**
     * \brief Apply a CSG operation to a mesh
     * \details Default implementation works in 2D (based on triangulate())
     *   and does nothing in 3D (to be overriden by user).
     *  Each triangle has an "operand bit" indicating to which input operand
     *  it belongs to, set to (1u << operand_id).
     * \param[in,out] mesh the mesh
     * \param[in] boolean_expr the operation to be applied
     *   - "union"
     *   - "intersection"
     *   - a general boolean expression, where:
     *      - variables are x0 ... x31 (they correspond to input operands)
     *      - operators are +,-,*
     *      - there can be parentheses
     */
    virtual void do_CSG(
	std::shared_ptr<Mesh>& mesh, const std::string& boolean_expr
    );

    /**
     * \brief Triangulates a 2D mesh.
     * \details Computes a constrained Delaunay triangulation from the edges
     *  of the mesh, then classifies the triangles using a boolean expression.
     *  Each edge has an "operand bit" indicating to which input operand
     *  it belongs to, set to (1u << operand_id). Used to implement
     *  CSG operations in 2D.
     * \param[in,out] mesh the input is a set of vertices and edges.
     *   The output has a set of triangles inside the polygons defined by
     *   the edges.
     * \param[in] boolean_expr optional operation to be applied, can be one of
     *   - "union" (default)
     *   - "intersection"
     *   - a general boolean expression, where:
     *      - variables are x0 ... x31 (they correspond to input operands)
     *      - operators are +,-,*
     *      - there can be parentheses
     *   - "union_cnstr_operand_bits_is_operand_id", same as "union"
     *     but edges operand bits are set to
     *     operand_id (instead of 1u << operand_id). This allows for an
     *     unlimited number of operands (as opposed to operand bits where
     *     it is limited to 32). It is used to implement projection(cut=false).
     */
    virtual void triangulate(
        std::shared_ptr<Mesh>& mesh, const std::string& boolean_expr
    );

    /**
     * \brief Triangulates a 2D mesh
     * \details This sets all edge operand bits to 1 and then computes a
     *   union using the other flavor of triangulate()
     */
    virtual void triangulate(std::shared_ptr<Mesh>& mesh);

    /**
     * \brief keeps only triangles and vertices embedded in the z=0 plane, and
     *  makes the mesh 2D.
     * \details This also computes the border and re-triangulates it.
     * \param[in,out] M a shared pointer to the mesh
     */
    void keep_z0_only(std::shared_ptr<Mesh>& M);

    /**
     * \brief Derived classes may override this function and compute
     *  some cached information, e.g. bounding boxes, stored in the
     *  mesh.
     */
    virtual void finalize_mesh(std::shared_ptr<Mesh>& mesh);

    CSGScope& top_scope() {
	geo_debug_assert(!scope_stack_.empty());
	return scope_stack_.top();
    }

    void push_scope() {
	scope_stack_.emplace();
    }

    void pop_scope() {
	geo_debug_assert(!scope_stack_.empty());
	scope_stack_.pop();
    }

    protected:
    double STL_epsilon_;
    index_t max_arity_;
    bool detect_intersecting_neighbors_;
    bool delaunay_;
    bool simplify_coplanar_facets_;
    double coplanar_angle_tolerance_;
    bool fast_union_;
    bool noop_;
    std::shared_ptr<Mesh> empty_mesh_;
    std::shared_ptr<Mesh> result_;
    std::stack<CSGScope> scope_stack_;

    friend class CSGCompiler;
    };

}

#endif
