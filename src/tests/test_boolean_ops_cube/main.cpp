// Minimal reproducer: boolean of two unit cubes that share exactly one vertex,
//  (1,1,1).
// Expected: union volume 2, intersection volume 0, difference (A-B) volume 1.

#include <geogram/basic/common.h>
#include <geogram/basic/logger.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_surface_intersection.h>
#include <cstdio>
using namespace GEO;

static void cube(Mesh& M, double o) {
    M.clear();
    M.vertices.set_dimension(3);
    M.vertices.create_vertices(8);
    for (index_t i = 0; i < 8; ++i)
        M.vertices.point(i) = vec3(
	    o + (i & 1u), o + ((i >> 1) & 1u), o + ((i >> 2) & 1u)
	);
    static const index_t t[12][3] = {
	{0,2,1},{1,2,3},{4,5,6},{5,7,6},{0,1,4},{1,5,4},
	{2,6,3},{3,6,7},{0,4,2},{2,4,6},{1,3,5},{3,7,5}
    };
    M.facets.create_triangles(12);
    for (index_t f = 0; f < 12; ++f) {
	for (index_t k = 0; k < 3; ++k) {
	    M.facets.set_vertex(f, k, t[f][k]);
	}
    }
    M.facets.connect();
}

static bool check(
    const Mesh& A, const Mesh& B, const char* op, double expected_volume
) {
    Mesh R;
    mesh_boolean_operation(R, A, B, op);
    R.facets.connect();
    double vol = ::GEO::Geom::mesh_enclosed_volume(R);

    bool OK = ::fabs(expected_volume - vol) < 1e-6;

    Logger::out("BooleanOp")
	<< op << ": " << R.facets.nb() << " facets, volume = "
	<< vol << (OK ? "  OK" : "") << std::endl;

    if(!OK) {
	Logger::err("BooleanOp") << "KO (expected "
				 << expected_volume
				 << ", got "
				 << vol
				 << ")"
				 << std::endl;
    }
    return OK;
}


int main() {
    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
    Logger::instance()->set_quiet(false);
    Mesh A;
    Mesh B;
    cube(A, 0.0); // A = [0,1]^3
    cube(B, 1.0); // B = [1,2]^3
    Logger::out("BooleanOp")
	<< "volume A  = " << ::GEO::Geom::mesh_enclosed_volume(A)
	<< std::endl;

    Logger::out("BooleanOp")
	<< "volume B  = " << ::GEO::Geom::mesh_enclosed_volume(B)
	<< std::endl;
    bool OK =
	check(A,B,"A+B",2.0) &&
	check(A,B,"A*B",0.0) &&
	check(A,B,"A-B",1.0) ;
    return OK ? 0 : 1;
}
