#include <geogram/basic/common.h>
#include <geogram/basic/logger.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_surface_intersection.h>

#include <cmath>
#include <iostream>

// Boolean operations between solids that touch only at vertices. No
// intersection is recorded between such solids, so the inside/outside test of
// each connected component must not be decided by a ray that starts at, or
// passes through, the contact vertex. Unit cubes on the integer lattice give
// exact expected volumes: two cubes touch at exactly one vertex iff their
// lower corners differ by +-1 along every axis.

namespace {
    using namespace GEO;

    void append_cube(Mesh& M, int ox, int oy, int oz, bool vertex_order_flip) {
        const index_t first = M.vertices.nb();
        for(index_t i = 0; i < 8; ++i) {
            index_t j = vertex_order_flip ? 7 - i : i;
            M.vertices.create_vertex(
                vec3(
                    double(ox + int(j & 1)),
                    double(oy + int((j >> 1) & 1)),
                    double(oz + int((j >> 2) & 1))
                ).data()
            );
        }
        // Triangles of the cube with corner i at (i&1, (i>>1)&1, (i>>2)&1),
        // outward oriented; vertex_order_flip relabels the corners (7-i).
        static const index_t t[12][3] = {
            {0,2,1},{1,2,3},{4,5,6},{5,7,6},{0,1,4},{1,5,4},
            {2,6,3},{3,6,7},{0,4,2},{2,4,6},{1,3,5},{3,7,5}
        };
        for(index_t f = 0; f < 12; ++f) {
            M.facets.create_triangle(
                first + t[f][0], first + t[f][1], first + t[f][2]
            );
        }
    }

    struct Cube { int x, y, z; };

    void build(Mesh& M, const std::vector<Cube>& cubes, bool flip) {
        M.clear();
        M.vertices.set_dimension(3);
        for(const Cube& c : cubes) {
            append_cube(M, c.x, c.y, c.z, flip);
        }
        M.facets.connect();
    }

    // Returns the volume, or NaN if the operation threw.
    double boolean_volume(Mesh& A, Mesh& B, const char* op) {
        Mesh R;
        try {
            mesh_boolean_operation(R, A, B, op);
        } catch(...) {
            return std::nan("");
        }
        R.facets.connect();
        return Geom::mesh_enclosed_volume(R);
    }

    int failures = 0;
    int runs = 0;

    void expect(
        const char* what, const char* op, double got, double expected
    ) {
        ++runs;
        if(!(std::abs(got - expected) < 1e-9)) {
            ++failures;
            std::cout << "FAIL " << what << " " << op << ": volume "
                      << got << ", expected " << expected << std::endl;
        }
    }
}

int main() {
    GEO::initialize();
    Logger::instance()->set_quiet(true);

    for(int flip = 0; flip < 2; ++flip) {
        // A = [0,1]^3, B = A + d with d in {-1,1}^3: one shared vertex.
        for(int d = 0; d < 8; ++d) {
            Cube a{0, 0, 0};
            Cube b{(d & 1) ? 1 : -1, (d & 2) ? 1 : -1, (d & 4) ? 1 : -1};
            Mesh A, B;
            build(A, {a}, flip != 0);
            build(B, {b}, flip != 0);
            expect("two cubes", "A+B", boolean_volume(A,B,"A+B"), 2.0);
            expect("two cubes", "A*B", boolean_volume(A,B,"A*B"), 0.0);
            expect("two cubes", "A-B", boolean_volume(A,B,"A-B"), 1.0);
            expect("two cubes", "B-A", boolean_volume(A,B,"B-A"), 1.0);
        }

        // A is one cube; B is a chain of two cubes, each touching the
        // previous one at a vertex. Rays going up from a component's start
        // vertex pass through the contact vertices of the other bodies.
        for(int d1 = 0; d1 < 8; ++d1) {
            for(int d2 = 0; d2 < 8; ++d2) {
                const int s1[3] = {
                    (d1 & 1) ? 1 : -1, (d1 & 2) ? 1 : -1, (d1 & 4) ? 1 : -1
                };
                const int s2[3] = {
                    (d2 & 1) ? 1 : -1, (d2 & 2) ? 1 : -1, (d2 & 4) ? 1 : -1
                };
                // The third cube must be disjoint from the first (no contact
                // at all): every coordinate of s1+s2 is 0 or +-2, and it must
                // not be 0 on all axes at once with contact to A.
                int dx = s1[0]+s2[0], dy = s1[1]+s2[1], dz = s1[2]+s2[2];
                if(
                    std::abs(dx) < 2 && std::abs(dy) < 2 && std::abs(dz) < 2
                ) {
                    continue; // overlaps or touches A by an edge or a face
                }
                Cube a{0, 0, 0};
                Cube b1{s1[0], s1[1], s1[2]};
                Cube b2{dx, dy, dz};
                Mesh A, B;
                build(A, {a}, flip != 0);
                build(B, {b1, b2}, flip != 0);
                expect("chain", "A+B", boolean_volume(A,B,"A+B"), 3.0);
                expect("chain", "A*B", boolean_volume(A,B,"A*B"), 0.0);
                expect("chain", "A-B", boolean_volume(A,B,"A-B"), 1.0);
                expect("chain", "B-A", boolean_volume(A,B,"B-A"), 2.0);
            }
        }
    }

    std::cout << (failures == 0 ? "PASS " : "FAIL ")
              << runs - failures << " of " << runs << " checks" << std::endl;
    return failures == 0 ? 0 : 1;
}
