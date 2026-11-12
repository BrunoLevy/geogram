#include <geogram/basic/common.h>
#include <geogram/mesh/mesh_surface_intersection.h>

#include <cmath>
#include <iomanip>
#include <iostream>

namespace {
    using namespace GEO;

    // Each box is a closed, outward-oriented triangular surface. Reverse the
    // small box to represent a cavity boundary instead of a separate solid.
    void append_box(
        Mesh& mesh, const vec3& lo, const vec3& hi, index_t operand,
        double scale, const vec3& translation, bool reverse = false
    ) {
        const vec3 vertices[] = {
            {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z},
            {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z},
            {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z},
            {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}
        };
        const index_t triangles[][3] = {
            {0,3,2}, {0,2,1}, {4,5,6}, {4,6,7},
            {0,1,5}, {0,5,4}, {1,2,6}, {1,6,5},
            {2,3,7}, {2,7,6}, {3,0,4}, {3,4,7}
        };
        const index_t first = mesh.vertices.nb();
        for(const vec3& vertex : vertices) {
            const vec3 p = scale * vertex + translation;
            mesh.vertices.create_vertex(p.data());
        }
        Attribute<index_t> bits(mesh.facets.attributes(), "operand_bit");
        for(const auto& t : triangles) {
            const index_t f = mesh.facets.create_triangle(
                first + t[0], first + t[reverse ? 2 : 1],
                first + t[reverse ? 1 : 2]
            );
            bits[f] = operand;
        }
    }

    bool check(bool cavity, double scale, bool translated) {
        Mesh mesh;
        mesh.vertices.set_dimension(3);
        const vec3 translation = translated ?
            vec3(4096, -8192, 16384) : vec3(0, 0, 0);
        append_box(
            mesh, vec3(.1, .1, .1),
            cavity ? vec3(.9, .9, .9) : vec3(.5, .5, .9),
            1, scale, translation
        );
        append_box(
            mesh, vec3(.7, .7, .5), vec3(.7001, .7001, .5008),
            1, scale, translation, cavity
        );
        // Operand B contains all of A, so A*B must preserve A's boundaries.
        append_box(mesh, vec3(0, 0, 0), vec3(1, 1, 1), 2, scale, translation);
        MeshSurfaceIntersection intersection(mesh);
        intersection.set_radial_sort(true);
        intersection.intersect();
        intersection.classify("A*B");

        // Inspect classification directly, before any repair can reverse a
        // shell again. Measure the small boundary separately: its sign error
        // would be hidden by the much larger solid in a total-volume check.
        double volumes[2] = {0.0, 0.0};
        index_t counts[2] = {0, 0};
        const vec3 origins[] = {vec3(.1, .1, .1), vec3(.7, .7, .5)};
        for(index_t f : mesh.facets) {
            if(mesh.facets.nb_vertices(f) != 3) {
                return false;
            }
            vec3 p[3];
            for(index_t c = 0; c < 3; ++c) {
                p[c] = (mesh.facets.point(f, c) - translation) / scale;
            }
            const index_t part = (p[0].x > .69 && p[0].x < .71) ? 1 : 0;
            ++counts[part];
            volumes[part] += dot(
                p[0] - origins[part],
                cross(p[1] - origins[part], p[2] - origins[part])
            ) / 6.0;
        }
        // Expected values come from box dimensions, not Geogram's volume
        // routine. A cavity must remain negative; forcing all shells positive
        // would pass the solid case but fail this one.
        const double expected_small = cavity ? -8e-12 : 8e-12;
        const double expected_large = cavity ? .512 : .128;
        const bool passed =
            counts[0] == 12 && counts[1] == 12 &&
            std::isfinite(volumes[0]) && std::isfinite(volumes[1]) &&
            std::abs(volumes[0] - expected_large) <= 1e-10 &&
            std::abs(volumes[1] - expected_small) <= 1e-18;
        std::cout << (passed ? "PASS" : "FAIL")
                  << " cavity=" << cavity << " scale=" << scale
                  << " translated=" << translated << std::setprecision(17)
                  << " small_volume=" << volumes[1]
                  << " expected=" << expected_small << std::endl;
        return passed;
    }
}

int main() {
    GEO::initialize();
    int failures = 0;
    for(bool cavity : {false, true}) {
        for(double scale : {1.0, 100.0}) {
            for(bool translated : {false, true}) {
                if(!check(cavity, scale, translated)) {
                    ++failures;
                }
            }
        }
    }
    return failures == 0 ? 0 : 1;
}
