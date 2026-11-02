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

    void show_polytope(const Polytope& P) {
	std::cerr << "Polytope" << std::endl;
	std::cerr << "  Vertices" << std::endl;
	for(const vec3 p: P.verts) {
	    std::cerr << p << ", ";
	}
	std::cerr << std::endl;
	std::cerr << "  Facets" << std::endl;
	for(auto F: P.faces) {
	    for(index_t v: F) {
		std::cerr << v << " ";
	    }
	    std::cerr << std::endl;
	}
    }

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
	for(index_t v=7; v<C.nb_v(); ++v) {
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

    // Initial version by Claude
    Polytope build_polytope_0(const std::vector<HalfSpace>& H, double tol) {
	Polytope P;
	const int n = (int)H.size();
	auto inside = [&](const vec3& p) {
	    for (const auto& h : H) {
		if (dot(h.n, p) > h.d + tol) {
		    return false;
		}
	    }
	    return true;
	};
	for (int i = 0; i < n; ++i) {
	    for (int j = i + 1; j < n; ++j) {
		for (int k = j + 1; k < n; ++k) {
		    vec3 njk = cross(H[j].n, H[k].n);
		    double det = dot(H[i].n, njk);
		    if (std::fabs(det) < 1e-12) {
			continue;
		    }
		    vec3 p = (njk * H[i].d +
			      cross(H[k].n, H[i].n) * H[j].d +
			      cross(H[i].n, H[j].n) * H[k].d) * (1.0 / det);
		    if (!inside(p)) {
			continue;
		    }
		    bool dup = false;
		    for (const auto& v : P.verts) {
			if (length(v - p) <= tol) {
			    dup = true;
			    break;
			}
		    }
		    if (!dup) {
			P.verts.push_back(p);
		    }
		}
	    }
	}
	P.faces.resize(n);
	for (int i = 0; i < n; ++i) {
	    std::vector<int> on;
	    for (int v = 0; v < (int)P.verts.size(); ++v) {
		if (std::fabs(dot(H[i].n, P.verts[v]) - H[i].d) <= tol) {
		    on.push_back(v);
		}
	    }
	    if (on.size() < 3) {
		continue;
	    }
	    vec3 e1 = any_perp(H[i].n);
	    vec3 e2 = cross(H[i].n, e1);
	    vec3 ctr{0.0, 0.0, 0.0};
	    for (int v : on) {
		ctr = ctr + P.verts[v];
	    }
	    ctr = ctr * (1.0 / double(on.size()));
	    std::sort(on.begin(), on.end(), [&](int p, int q) {
		vec3 dp = P.verts[p] - ctr;
		vec3 dq = P.verts[q] - ctr;
		return (std::atan2(dot(dp, e2), dot(dp, e1)) <
			std::atan2(dot(dq, e2), dot(dq, e1))) ;
	    });
	    P.faces[i] = on;
	}
	return P;
    }

    // Writes all collected faces as one open shell. Call once.
    bool StepWriter::save(const std::string& path, const std::string& name) {
	int app = add("APPLICATION_CONTEXT('automotive design')");
	add("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000," + ref(app) + ")");
	int pc   = add("PRODUCT_CONTEXT(''," + ref(app) + ",'mechanical')");
	int prod = add("PRODUCT('" + name + "','" + name + "','',(" + ref(pc) + "))");
	int pdf  = add("PRODUCT_DEFINITION_FORMATION('',''," + ref(prod) + ")");
	int pdc  = add("PRODUCT_DEFINITION_CONTEXT('part definition'," + ref(app) + ",'design')");
	int pd   = add("PRODUCT_DEFINITION('design',''," + ref(pdf) + "," + ref(pdc) + ")");
	int pds  = add("PRODUCT_DEFINITION_SHAPE('',''," + ref(pd) + ")");
	int lu   = add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )");
	int au   = add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )");
	int su   = add("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )");
	int unc  = add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(" + real(std::max(tol_, 1e-6)) +
		       ")," + ref(lu) + ",'distance_accuracy_value','')");
	int ctx  = add("( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((" +
		       ref(unc) + ")) GLOBAL_UNIT_ASSIGNED_CONTEXT((" + ref(lu) + "," + ref(au) + "," +
		       ref(su) + ")) REPRESENTATION_CONTEXT('','') )");
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

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
