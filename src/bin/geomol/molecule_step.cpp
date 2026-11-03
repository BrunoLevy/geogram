// molecule_step.cpp  -  see molecule_step.h
#include "molecule_step.h"

namespace MSS_Claude {

    static vec3 xyz(const GEO::vec4f& v) {
	return {double(v.x), double(v.y), double(v.z)};
    }

    // Claude's version
    /*
    std::vector<HalfSpace> cell_halfspaces_0(
	const Cells& cells, GEO::index_t c, int& sign, double tol
    ) {
	std::vector<HalfSpace> raw;
	for (GEO::vec4f P : cells.cell_planes(c)) {
	    raw.push_back({xyz(P), double(P.w)});
	}
	auto with_sign = [&](int s) {
	    std::vector<HalfSpace> out;
	    for (const auto& h : raw) {
		out.push_back(
		    s > 0 ? HalfSpace{h.n, -h.d} : HalfSpace{-h.n, h.d}
		);
	    }
	    return out;
	};
	if (sign == 0) {
	    for (int s : {1, -1}) {
		if (
		    build_polytope(
			normalised(with_sign(s)), tol
		    ).verts.size() >= 4
		) {
		    sign = s;
		    break;
		}
	    }
	    if (sign == 0) return {};
	}
	return with_sign(sign);
    }
    */

    // My version (ignores sign)
    std::vector<HalfSpace> cell_halfspaces(
	const Cells& cells, GEO::index_t c, int& sign, double tol
    ) {
	geo_assert(sign == 0 || sign == 1);
	geo_argused(tol);
	sign = 1;
	std::vector<HalfSpace> result;
	for (GEO::vec4f P : cells.cell_planes(c)) {
	    result.push_back({xyz(P), -double(P.w)});
	}
	return result;
    }

    HyperboloidPatch quadric_from_cell_eqn(
	const GEO::vec4f& eq1, const GEO::vec4f& eq2,
	bool is_H2, double shrink_factor
    ) {
	// geomol conventions (verified on its output):
	// the balls carry weight 2 r^2 (so that the shrunk skin spheres have
	// the tabulated radius r), f = {fx,fy,fz} is the radical point of
	// the edge (H1) or the orthocenter of the triangle (H2) for those balls,
	// and R2 is the power of f w.r.t. them (positive when the balls do not
	// reach f).
	// Skin surface in the mixed cell, shrink factor s, zeta along the axis,
	// rho the distance to it:
	//   H1:  rho^2 - (s/(1-s)) zeta^2 = -s R2,      solid = {<=}
	//   H2:  rho^2 - ((1-s)/s) zeta^2 = (1-s) R2,   solid = {>=}
	// One sheet if the right-hand side is positive, two sheets otherwise.
	const double s = shrink_factor, R2 = double(eq1.w);
	HyperboloidPatch P;
	P.center = xyz(eq1);
	P.axis = xyz(eq2);
	P.A = is_H2 ? (1.0 - s) * R2 : -s * R2;
	P.B = is_H2 ? (1.0 - s) / s : s / (1.0 - s);
	P.solid_is_inside = !is_H2;
	return P;
    }

    bool sphere_cell_to_step(
	const Cells& cells, GEO::index_t c,
	bool is_S3, StepWriter& w, int plane_sign
    ) {
	std::pair<GEO::vec4f, GEO::vec4f> eq = cells.cell_eqn(c);
	SpherePatch P;
	P.center = xyz(eq.first);
	P.radius = double(eq.first.w);
	P.solid_is_inside = !is_S3;
	P.cell = cell_halfspaces(cells, c, plane_sign, w.tol());
	if (P.cell.empty()) return false;
	return sphere_patch_to_step(P, w);
    }

    bool hyperboloid_cell_to_step(
	const Cells& cells, GEO::index_t c, bool is_H2, double shrink_factor,
	StepWriter& w, int plane_sign
    ) {
	std::pair<GEO::vec4f, GEO::vec4f> eq = cells.cell_eqn(c);
	HyperboloidPatch P = quadric_from_cell_eqn(
	    eq.first, eq.second, is_H2, shrink_factor
	);
	P.cell = cell_halfspaces(cells, c, plane_sign, w.tol());
	if (P.cell.empty()) return false;
	return hyperboloid_patch_to_step(P, w);
    }

    GEO::index_t mixed_cells_to_step(
	const Cells& cells, GEO::Molecule::MixedCellType type,
	double shrink_factor, StepWriter& w, int plane_sign
    ) {
	using GEO::Molecule;
	GEO::index_t done = 0;
	for (GEO::index_t c = 0; c < cells.nb_cells(); ++c) {
	    bool ok;
	    switch (type) {
            case Molecule::CELL_TYPE_S0:
		ok = sphere_cell_to_step(cells, c, false, w, plane_sign);
		break;
            case Molecule::CELL_TYPE_S3:
		ok = sphere_cell_to_step(cells, c, true, w, plane_sign);
		break;
            case Molecule::CELL_TYPE_H1:
		ok = hyperboloid_cell_to_step(
		    cells, c, false, shrink_factor, w, plane_sign
		);
		break;
            case Molecule::CELL_TYPE_H2:
		ok = hyperboloid_cell_to_step(
		    cells, c, true, shrink_factor, w, plane_sign
		);
		break;
            default: ok = false;
	    }
	    done += ok ? 1 : 0;
	}
	return done;
    }

    bool save_molecule_STEP(
	GEO::Molecule& mol, const std::string& filename, double tol
    ) {
	StepWriter w(tol);
	double s = mol.shrink_factor();
	for (GEO::index_t t = 0; t < GEO::Molecule::CELL_TYPE_NB; ++t) {
	    mixed_cells_to_step(
		mol.cells(t), GEO::Molecule::MixedCellType(t), s, w
	    );
	}
	return w.save(filename, "molecular_skin");
    }

} // namespace skin
