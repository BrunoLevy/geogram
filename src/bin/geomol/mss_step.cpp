#include "mss_step.h"

#include <geogram/voronoi/convex_cell.h>

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#endif

namespace {
    using namespace MSS_Claude;
}

/*****************************************************************************/

namespace MSS_Claude {

    // My version, based on ConvexCell
    Polytope build_polytope(const std::vector<HalfSpace>& H, double tol) {
	geo_argused(tol);
	ConvexCell C;
	C.use_exact_predicates(true);
	const double L = 1e4;
	C.init_with_box(-L,-L,-L,L,L,L);
	for(const HalfSpace& h: H) {
	    C.clip_by_plane({-h.n,h.d}); // Damnit, Halfspace is not x,y,z,w
	}
	C.compute_geometry();

	Polytope P;
	vector<index_t> v2t(C.nb_v(),VBW::index_t(-1));
	vector<index_t> t_index(C.nb_t(),VBW::index_t(-1));

	// get vertices and initialize t_index and v2t
	for(
	    VBW::ushort t = C.first_triangle(); t != VBW::END_OF_LIST;
	    t = C.next_triangle(t)
	) {
	    t_index[t] = index_t(P.verts.size());
	    P.verts.push_back(C.triangle_point(t));
	    VBW::Triangle T = C.get_triangle(t);
	    v2t[T.i] = t;
	    v2t[T.j] = t;
	    v2t[T.k] = t;
	}

	// get facets
	for(index_t v=0; v<C.nb_v(); ++v) {
            if(v2t[v] != VBW::index_t(-1)) {
                index_t t = v2t[v];
		P.faces.emplace_back();
                do {
		    P.faces.rbegin()->push_back(t_index[t]);
		    index_t lv = C.triangle_find_vertex(t,v);
                    t = C.triangle_adjacent(t, (lv + 1)%3);
                } while(t != v2t[v]);
            }
	}
	return P;
    }
}

/*****************************************************************************/
/***** Public API: StepWriter                                            *****/
/*****************************************************************************/

namespace MSS_Claude {

    int StepWriter::add(const std::string& body) {
	entities_.push_back(body);
	return int(entities_.size());
    }

    void StepWriter::add_face(int face_id) {
	faces_.push_back(face_id);
    }

    std::string StepWriter::real(double v) const {
	char buf[64];
	std::snprintf(buf, sizeof buf, "%.15g", v);
	std::string s(buf);
	for (char& ch : s) if (ch == 'e') ch = 'E';
	size_t e = s.find('E');
	std::string mant = e == std::string::npos ? s : s.substr(0, e);
	if (mant.find('.') == std::string::npos) {
	    mant += ".";
	    s = e == std::string::npos ? mant : mant + s.substr(e);
	}
	return s;
    }

    std::string StepWriter::ref(int id) {
	return "#" + std::to_string(id);
    }

    std::string StepWriter::refs(const std::vector<int>& ids) {
	std::string s;
	for (size_t i = 0; i < ids.size(); ++i) {
	    s += (i ? "," : "") + ref(ids[i]);
	}
	return s;
    }

    int StepWriter::point(const vec3& p) {
	return add(
	    "CARTESIAN_POINT('',(" +
	    real(p.x) + "," + real(p.y) + "," + real(p.z) + "))"
	);
    }

    int StepWriter::direction(const vec3& d) {
	vec3 u = normalize(d);
	return add(
	    "DIRECTION('',(" +
	    real(u.x) + "," + real(u.y) + "," + real(u.z) + "))"
	);
    }

    int StepWriter::axis2(const vec3& o, const vec3& z, const vec3& x) {
	int p = point(o), dz = direction(z), dx = direction(x);
	return add(
	    "AXIS2_PLACEMENT_3D(''," + ref(p) + "," +
	    ref(dz) + "," + ref(dx) + ")"
	);
    }

    int StepWriter::axis1(const vec3& o, const vec3& z) {
	int p = point(o), dz = direction(z);
	return add("AXIS1_PLACEMENT(''," + ref(p) + "," + ref(dz) + ")");
    }

    // Writes all collected faces as one open shell. Call once.
    bool StepWriter::save(const std::string& path, const std::string& name) {
	int app = add("APPLICATION_CONTEXT('automotive design')");
	add("APPLICATION_PROTOCOL_DEFINITION('international standard',"
	    "'automotive_design',2000," + ref(app) + ")");
	int pc = add("PRODUCT_CONTEXT(''," + ref(app) + ",'mechanical')");
	int prod = add("PRODUCT('" + name + "','" + name + "','',(" +
		       ref(pc) + "))");
	int pdf = add("PRODUCT_DEFINITION_FORMATION('',''," + ref(prod) + ")");
	int pdc = add("PRODUCT_DEFINITION_CONTEXT('part definition'," +
		       ref(app) + ",'design')");
	int pd = add("PRODUCT_DEFINITION('design',''," + ref(pdf) + "," +
		     ref(pdc) + ")");
	int pds = add("PRODUCT_DEFINITION_SHAPE('',''," + ref(pd) + ")");
	int lu = add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )");
	int au = add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )");
	int su = add(
	    "( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )"
	);
	int unc  = add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(" +
		       real(std::max(tol_, 1e-6)) +
		       ")," + ref(lu) + ",'distance_accuracy_value','')");
	int ctx  = add(
	    "( GEOMETRIC_REPRESENTATION_CONTEXT(3) "
	    "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((" +
	    ref(unc) + ")) GLOBAL_UNIT_ASSIGNED_CONTEXT((" +
	    ref(lu) + "," + ref(au) + "," +
	    ref(su) + ")) REPRESENTATION_CONTEXT('','') )"
	);
	int shell = add("OPEN_SHELL('',(" + refs(faces_) + "))");
	int sbsm  = add("SHELL_BASED_SURFACE_MODEL('',(" + ref(shell) + "))");
	int place = axis2({0, 0, 0}, {0, 0, 1}, {1, 0, 0});
	int rep   = add("MANIFOLD_SURFACE_SHAPE_REPRESENTATION('',(" +
			ref(place) + "," + ref(sbsm) + ")," + ref(ctx) + ")");
	add("SHAPE_DEFINITION_REPRESENTATION("+ref(pds)+"," + ref(rep) + ")");
	add("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(" + ref(prod) + "))");

	std::ofstream f(path);
	if (!f) return false;
	f << "ISO-10303-21;\nHEADER;\n"
	  << "FILE_DESCRIPTION(('molecular skin surface: "
	  << "trimmed quadric patches'),'2;1');\n"
	  << "FILE_NAME('" << path
	  << "','',(''),(''),'skin_patches_step','','');\n"
	  << "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n"
	  << "ENDSEC;\nDATA;\n";
	for (size_t i = 0; i < entities_.size(); ++i) {
	    f << "#" << i + 1 << " = " << entities_[i] << ";\n";
	}
	f << "ENDSEC;\nEND-ISO-10303-21;\n";
	return true;
    }
}

/*****************************************************************************/
/***** Public API: patches conversion                                    *****/
/*****************************************************************************/

namespace MSS_Claude {

    // Sphere ∩ convex cell -> trimmed SPHERICAL_SURFACE faces
    // (one per octant met).
    bool sphere_patch_to_step(const SpherePatch& P, StepWriter& w) {
	if (P.radius <= 0) return false;
	Quadric q;
	q.sphere = true;
	q.c = P.center;
	q.A = P.radius * P.radius;
	q.B = -1;
	std::vector<HalfSpace> cell = normalised(P.cell);
	int n = 0;
	for (int sx : {-1, 1}) for (int sy : {-1, 1}) for (int sz : {-1, 1}) {
		    std::vector<HalfSpace> sub = cell;
		    sub.push_back(above({double(sx), 0, 0}, q.c, 0));
		    sub.push_back(above({0, double(sy), 0}, q.c, 0));
		    sub.push_back(above({0, 0, double(sz)}, q.c, 0));
		    TrimResult R;
		    trim_quadric(q, sub, P.solid_is_inside, w, R);
		    if (R.loops.empty()) continue;
		    Surface S = sphere_surface(
			q, normalize(vec3{double(sx), double(sy), double(sz)}),
			P.solid_is_inside, w
		    );
		    n += emit_faces(q, P.solid_is_inside, w, R, S);
		}
	return n > 0;
    }

    // Hyperboloid of revolution ∩ convex cell -> trimmed rational Bezier faces.
    bool hyperboloid_patch_to_step(const HyperboloidPatch& P, StepWriter& w) {
	if (P.B <= 0 || P.A == 0 || length(P.axis) == 0) {
	    return false;
	}
	Quadric q;
	q.sphere = false;
	q.c = P.center;
	q.a = normalize(P.axis);
	q.A = P.A;
	q.B = P.B;
	const std::vector<HalfSpace> cell = normalised(P.cell);
	const vec3 e1 = any_perp(q.a), e2 = cross(q.a, e1);
	const double tol = w.tol(), phi = 55.0 * M_PI / 180.0;
           // sector half-angle (90 deg cut + margin)
	int n = 0;

	auto axial_range = [&](
	    const std::vector<HalfSpace>& sub, const vec3& axis,
	    double& zmin, double& zmax
	) {
	    Polytope pt = build_polytope(sub, tol);
	    if (pt.verts.size() < 4) return false;
	    zmin = 1e300; zmax = -1e300;
	    for (const vec3& v : pt.verts) {
		double z = dot(axis, v - q.c);
		zmin = std::min(zmin, z);
		zmax = std::max(zmax, z);
	    }
	    return zmax > zmin;
	};

	// Revolution pieces: sectors x axial segments
	// of the generatrix (rho(t), z(t)).
	auto revolution_pieces = [&](
	    const std::vector<HalfSpace>& base, const vec3& axis,
	    const std::function<vec2(double)>& G,
	    const std::function<vec2(double)>& dG,
	    double T0, double T1
	) {
	    int nseg = std::max(1, (int)std::ceil((T1 - T0) / 1.5));
	    for (int k = 0; k < nseg; ++k) {
		double Ta = T0 + (T1 - T0) * k / nseg;
		double Tb = T0 + (T1 - T0) * (k + 1) / nseg;
		double delta = 0.05 * (Tb - Ta);
		vec2 Gp[3]; double w1;
		if (!conic_arc_bezier(G, dG, Ta - delta, Tb + delta, Gp, w1)) {
		    continue;
		}
		for (int s1 : {-1, 1}) {
		    for (int s2 : {-1, 1}) {
			std::vector<HalfSpace> sub = base;
			sub.push_back(above(e1 * double(s1), q.c, 0));
			sub.push_back(above(e2 * double(s2), q.c, 0));
			sub.push_back(above(axis, q.c, G(Ta).y));
			sub.push_back(below(axis, q.c, G(Tb).y));
			TrimResult R;
			trim_quadric(q, sub, P.solid_is_inside, w, R);
			if (R.loops.empty()) continue;
			Surface S = revolution_surface(
			    q, axis, normalize(e1*double(s1) + e2*double(s2)),
			    phi, Gp, w1, P.solid_is_inside, w
			);
			n += emit_faces(q, P.solid_is_inside, w, R, S);
		    }
		}
	    }
	};

	if (q.A > 0) { // one sheet: rho = sa cosh t, z = sb sinh t
	    double zmin, zmax;
	    if (!axial_range(cell, q.a, zmin, zmax)) return false;
	    double m = 0.05 * (zmax - zmin); zmin -= m; zmax += m;
	    double sa = std::sqrt(q.A), sb = std::sqrt(q.A / q.B);
	    auto G  = [=](double t) {
		return vec2(sa * std::cosh(t), sb * std::sinh(t));
	    };
	    auto dG = [=](double t) {
		return vec2(sa * std::sinh(t), sb * std::cosh(t));
	    };
	    revolution_pieces(
		cell, q.a, G, dG, std::asinh(zmin / sb), std::asinh(zmax / sb)
	    );
	} else { // two sheets, one at a time
	    double z0 = std::sqrt(-q.A / q.B), rc = std::sqrt(-q.A);
	    const double zsplit = 2.5 * z0; // near cap / far part boundary
	    for (int sigma : {1, -1}) {
		vec3 axis = q.a * double(sigma);
		std::vector<HalfSpace> near_hs = cell;
		near_hs.push_back(above(axis, q.c, 0));
		near_hs.push_back(below(axis, q.c, zsplit));
		TrimResult R;
		trim_quadric(q, near_hs, P.solid_is_inside, w, R);
		if (!R.loops.empty()) {
		    // domain [-r,r]^2 covers zeta <= 2.7 z0 and
		    // stays inside the disk s^2+t^2 < B
		    Surface S = stereographic_surface(
			q, axis, z0, std::sqrt(q.B * 1.7 / 3.7),
			P.solid_is_inside, w
		    );
		    n += emit_faces(q, P.solid_is_inside, w, R, S);
		}
		std::vector<HalfSpace> far_hs = cell;
		far_hs.push_back(above(axis, q.c, zsplit));
		double zmin, zmax;
		if (!axial_range(far_hs, axis, zmin, zmax)) continue;
		auto G  = [=](double t) {
		    return vec2(rc * std::sinh(t), z0 * std::cosh(t));
		};
		auto dG = [=](double t) {
		    return vec2(rc * std::cosh(t), z0 * std::sinh(t));
		};
		revolution_pieces(
		    far_hs, axis, G, dG, std::acosh(zsplit / z0),
		    std::acosh(std::max(2.6, 1.05 * zmax / z0))
		);
	    }
	}
	return n > 0;
    }
}

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
