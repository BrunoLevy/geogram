#include <geogram/basic/common.h>
#include <geogram/basic/numeric.h>
#include <geogram/basic/process.h>
#include <geogram/delaunay/delaunay.h>

#include <iostream>
#include <random>
#include <vector>

int main() {
    GEO::initialize();
    GEO::Process::set_max_threads(2);

    const GEO::index_t nb_vertices = 10000;
    std::vector<double> points(3 * nb_vertices);
    std::mt19937_64 rng(11001);
    for(GEO::index_t v = 0; v < nb_vertices; ++v) {
        for(GEO::index_t coord = 0; coord < 3; ++coord) {
            points[3 * v + coord] =
                static_cast<double>(rng() >> 11) * (1.0 / 9007199254740992.0);
        }
        rng();
    }

    // Query PDEL directly: Delaunay::create() can fall back to NN.
    GEO::Delaunay_var delaunay =
        GEO::DelaunayFactory::create_object("PDEL", 3);
    if(delaunay.is_null()) {
        std::cerr << "PDEL factory unavailable\n";
        return 1;
    }
    delaunay->set_reorder(false);
    delaunay->set_keeps_infinite(false);
    // A large sequential bootstrap grows storage before parallel BRIO rounds.
    GEO::vector<GEO::index_t> levels(4);
    levels[0] = 0;
    levels[1] = 9000;
    levels[2] = 9500;
    levels[3] = 10000;
    delaunay->set_BRIO_levels(levels);
    GEO::Numeric::random_reset();
    delaunay->set_vertices(nb_vertices, points.data());

    std::vector<unsigned char> seen(nb_vertices, 0);
    // Infinite cells were removed, so all nb_cells() cells are finite.
    for(GEO::index_t c = 0; c < delaunay->nb_cells(); ++c) {
        for(GEO::index_t lv = 0; lv < 4; ++lv) {
            const GEO::index_t v = delaunay->cell_vertex(c, lv);
            if(v >= nb_vertices) {
                std::cerr << "Invalid finite-cell vertex ID " << v
                          << " in cell " << c << ", local vertex " << lv
                          << " (input vertex count " << nb_vertices << ")\n";
                return 1;
            }
            seen[v] = 1;
        }
    }

    GEO::index_t missing = 0;
    GEO::index_t first_missing = nb_vertices;
    for(GEO::index_t v = 0; v < nb_vertices; ++v) {
        if(!seen[v]) {
            if(missing == 0) {
                first_missing = v;
            }
            ++missing;
        }
    }
    if(missing != 0) {
        std::cerr << "PDEL output is missing " << missing << " of "
                  << nb_vertices << " input vertices; first missing ID: "
                  << first_missing << '\n';
        return 1;
    }
    return 0;
}
