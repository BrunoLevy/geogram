// written by Claude, slightly edited

#include "molecule_step.h"
#include "mss_step.h"

// Data read from Molecule::MixedComplexCells (public):
//   cells.nb_cells()
//   cells.cell_eqn(c)-> {vec4f, vec4f}
//                    sphere      : {cx,cy,cz,R},  {unused,unused,unused,cell_id}
//                    hyperboloid : {fx,fy,fz,R2}, {axisx,axisy,axisz,cell_id}
//   cells.cell_planes(c)  -> range of vec4f (nx,ny,nz,d) with n.p + d = 0
//                            (from end_facet() or add_facet_plane()).
//
// Cell families (Molecule::MixedCellType):
//   CELL_TYPE_S0 (atom)         sphere,      solid = inside the sphere
//   CELL_TYPE_H1 (edge)         hyperboloid  rho^2 - (s/(1-s)) z^2 = -s R2,
//                                            solid = {<=}
//   CELL_TYPE_H2 (triangle)     hyperboloid
//                               rho^2 - ((1-s)/s) z^2 = (1-s) R2,  solid = {>=}
//   CELL_TYPE_S3 (tetrahedron)  sphere,      solid = complement
// (s = shrink factor; either hyperboloid has one or two sheets
//  depending on the sign).
//
// Two conventions are not visible in molecule.h (they live in the shaders of
// molecule_shaders.h) and are isolated so they are a one-line change:
//   * which side of a clipping plane is inside: auto-detected per cell (the sign
//     for which the planes bound a non-empty polytope),
//     or forced with plane_sign;
//   * the meaning of {f,R2}: verified on geomol output to be the radical point /
//     orthocenter and its power for the balls with doubled weights (2 r^2); the
//     hyperboloid constant is -s R2 (H1) or (1-s) R2 (H2),
//     see quadric_from_cell_eqn().

namespace {
    using namespace MSS_Claude;

    typedef GEO::Molecule::MixedComplexCells Cells;

    // Clipping planes of cell c as half-spaces n.p <= d.
    std::vector<HalfSpace> cell_halfspaces(const Cells& cells, GEO::index_t c) {
	std::vector<HalfSpace> result;
	for (GEO::vec4f P : cells.cell_planes(c)) {
	    result.push_back(
		{vec3{double(P.x), double(P.y), double(P.z)}, -double(P.w)}
	    );
	}
	return result;
    }

    // Hyperboloid parameters from a cell equation
    // (the place to adapt the R2 convention).
    HyperboloidPatch quadric_from_cell_eqn(
	const GEO::vec4f& eq1, const GEO::vec4f& eq2, bool is_H2,
	double shrink_factor
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
	P.center = {double(eq1.x), double(eq1.y), double(eq1.z)};
	P.axis = {double(eq2.x), double(eq2.y), double(eq2.z)};
	P.A = is_H2 ? (1.0 - s) * R2 : -s * R2;
	P.B = is_H2 ? (1.0 - s) / s : s / (1.0 - s);
	P.solid_is_inside = !is_H2;
	return P;
    }

    // S0 / S3 cell -> trimmed SPHERICAL_SURFACE face.
    bool sphere_cell_to_step(
	const Cells& cells, GEO::index_t c, bool is_S3, StepWriter& w
    ) {
	std::pair<GEO::vec4f, GEO::vec4f> eq = cells.cell_eqn(c);
	SpherePatch P;
	P.center = {double(eq.first.x), double(eq.first.y), double(eq.first.z)};
	P.radius = double(eq.first.w);
	P.solid_is_inside = !is_S3;
	P.cell = cell_halfspaces(cells, c);
	if (P.cell.empty()) {
	    return false;
	}
	return sphere_patch_to_step(P, w);
    }

    // H1 / H2 cell -> trimmed SURFACE_OF_REVOLUTION(HYPERBOLA) face(s).
    bool hyperboloid_cell_to_step(
	const Cells& cells, GEO::index_t c, bool is_H2, double shrink_factor,
	StepWriter& w
    ) {
	std::pair<GEO::vec4f, GEO::vec4f> eq = cells.cell_eqn(c);
	HyperboloidPatch P = quadric_from_cell_eqn(
	    eq.first, eq.second, is_H2, shrink_factor
	);
	P.cell = cell_halfspaces(cells, c);
	if (P.cell.empty()) {
	    return false;
	}
	return hyperboloid_patch_to_step(P, w);
    }

    // Converts a whole family. Returns the number of cells that produced faces
    // (empty cells, where the quadric misses the polytope, produce none).
    GEO::index_t mixed_cells_to_step(
	const Cells& cells, GEO::Molecule::MixedCellType type,
	double shrink_factor, StepWriter& w
    ) {
	using GEO::Molecule;
	GEO::index_t done = 0;
	for (GEO::index_t c = 0; c < cells.nb_cells(); ++c) {
	    bool ok;
	    switch (type) {
            case Molecule::CELL_TYPE_S0:
		ok = sphere_cell_to_step(cells, c, false, w);
		break;
            case Molecule::CELL_TYPE_S3:
		ok = sphere_cell_to_step(cells, c, true, w);
		break;
            case Molecule::CELL_TYPE_H1:
		ok = hyperboloid_cell_to_step(
		    cells, c, false, shrink_factor, w
		);
		break;
            case Molecule::CELL_TYPE_H2:
		ok = hyperboloid_cell_to_step(
		    cells, c, true, shrink_factor, w
		);
		break;
            default: ok = false;
	    }
	    done += ok ? 1 : 0;
	}
	return done;
    }
}

/*****************************************************************************/

namespace MSS_Claude {

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

} // namespace MSS_Claude
