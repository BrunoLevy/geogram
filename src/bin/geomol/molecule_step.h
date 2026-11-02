// molecule_step.h
//
// Export of geomol's molecular skin surface (geogram, src/bin/geomol/molecule.h)
// as trimmed parametric surfaces in STEP, using the writer in mss_step.h.
//
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
//
// To export a whole molecule, Molecule needs read access to its cells_[] array,
// e.g. add to the public section of Molecule:
//   const MixedComplexCells& cells(index_t type) const { return cells_[type]; }
// then call  skin::save_molecule_STEP(molecule, "skin.step");

#pragma once
#ifndef SKIN_STEP_STUB_GEO
#include <geogram/basic/geometry.h>
#include "molecule.h"
#endif
#include "mss_step.h"

namespace MSS_Claude {

    typedef GEO::Molecule::MixedComplexCells Cells;

    // Clipping planes of cell c as half-spaces n.p <= d.
    // sign: +1 => inside is n.p + d <= 0,
    //       -1 => inside is n.p + d >= 0,
    //        0 => auto-detect
    // (on return, sign holds the detected value).
    std::vector<HalfSpace> cell_halfspaces(
	const Cells& cells, GEO::index_t c, int& sign, double tol
    );

    // Hyperboloid parameters from a cell equation
    // (the place to adapt the R2 convention).
    HyperboloidPatch quadric_from_cell_eqn(
	const GEO::vec4f& eq1, const GEO::vec4f& eq2,
	bool is_H2, double shrink_factor
    );

    // S0 / S3 cell -> trimmed SPHERICAL_SURFACE face.
    bool sphere_cell_to_step(
	const Cells& cells, GEO::index_t c, bool is_S3,
	StepWriter& w, int plane_sign = 0
    );

    // H1 / H2 cell -> trimmed SURFACE_OF_REVOLUTION(HYPERBOLA) face(s).
    bool hyperboloid_cell_to_step(
	const Cells& cells, GEO::index_t c, bool is_H2, double shrink_factor,
	StepWriter& w, int plane_sign = 0
    );

    // Converts a whole family. Returns the number of cells that produced faces
    // (empty cells, where the quadric misses the polytope, produce none).
    GEO::index_t mixed_cells_to_step(
	const Cells& cells, GEO::Molecule::MixedCellType type,
	double shrink_factor, StepWriter& w, int plane_sign = 0
    );

    // Converts all four families of a molecule and writes the STEP file.
    // Requires Molecule::cells(type) (see above); shrink_factor() is used for B.
    bool save_molecule_STEP(
	GEO::Molecule& mol, const std::string& filename, double tol = 3e-5
    );

} // namespace MSS_Claude
