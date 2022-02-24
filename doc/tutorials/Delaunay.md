# Delaunay triangulations and Voronoi diagrams

Delaunay triangulation and Voronoi diagrams are fundamental structures
for geometric computing. Geogram has memory efficient implementations in
2D and in 3D.

2D Delaunay triangulation
=========================

The [example program](https://github.com/BrunoLevy/geogram/blob/main/src/examples/graphics/demo_Delaunay2d/main.cpp) 
computes a 2D Delaunay triangulation and the dual Voronoi diagram. The
computed structures are displayed and can be interactively modified.
The pointset can be optimized by Lloyd relaxation.

The Delaunay class
------------------

```
Delaunay_var delaunay = new Delaunay2d()
```

where `Delaunay_var` is a typedef corresponding to `SmartPointer<Delaunay>`.
The delaunay triangulation is computed by the following function:
```
delaunay->set_vertices(n, points);
```
where `n` denotes the number of points and where `points` is a pointer
to a contiguous array of 2n double precision numbers with the
coordinates of the points. If the point coordinates are stored in a
`std::vector<double>`, one may use 
`delaunay->set_vertices(v.size()/2, v.data())`. If the point coordinates
are stored in a `std::vector<vec2>`, one may use
`delaunay->set_vertices(v.size(), &v.data()->x)` (as in the demo program).
Using `double*` as an interface may seem low-level / outdated, but it is
the smallest common denominator, and makes it easier to use with your
own data structures. 

Once the triangulation is created, it can be accessed through the
following functions:

- `Delaunay::nb_cells()` returns the number of cells (in our case triangles) in the triangulation.
- `Delaunay::cell_vertex(c,lv)` where `0 <= c < nb_cells()` and `lv` in `{0,1,2}`, returns a vertex of a triangle
- `Delaunay::cell_adjacent(c,lf)` where `0 <= c < nb_cells()` and `lf` in `{0,1,2}`, returns the triangle adjacent to `c` accross edge `lf`

Alternatively, one may use the (lower-level) functions:

- `Delaunay::cell_to_v()` returns a pointer to the cell-vertices incidence array.
- `Delaunay::cell_to_cell()` returns a pointer to the cell-cell adjacency array.

_WIP_