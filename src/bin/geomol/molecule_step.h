// written by Claude, slightly edited
//
// Export of geomol's molecular skin surface as trimmed parametric surfaces
// in STEP, using the writer in mss_step.h.

#pragma once
#include "molecule.h"

namespace MSS_Claude {
    // Converts all four families of a molecule and writes the STEP file.
    bool save_molecule_STEP(
	GEO::Molecule& mol, const std::string& filename, double tol = 3e-5
    );
}
