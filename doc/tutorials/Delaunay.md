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

We shall now see how to visualize the Delaunay triangulation and its
dual, the Voronoi diagram. Geogram provides a set of classes 

To draw things on the screen, Geogram has a
set of functions called `GLUP`, declared in
[this header file](https://github.com/BrunoLevy/geogram/blob/main/src/lib/geogram_gfx/GLUP/GLUP.h).
The vertices are drawn as follows:

```
  glupBegin(GLUP_POINTS);
  for(index_t i=0; i<points_.size(); ++i) {
    glupVertex(points_[i]);
  }
  glupEnd();
```
Each call to `glupVertex()` between `glupBegin(GLUP_POINTS)` and `glupEnd()`
results in a little ball drawn on the screen. In addition, one can specify
the size (`glupSetPointSize()`) and the color (`glupSetColor3f()`) of the
points. Programmers who know the (old) OpenGL 2.x API will feel "at home":
GLUP takes exactly the same convention. Under the hood, GLUP "translates"
its call into modern OpenGL (with vertex buffers and shaders). 


The triangles are drawn as follows:
```
  glupBegin(GLUP_LINES);
  for(index_t c=0; c<delaunay_->nb_cells(); ++c) {
    const signed_index_t* cell = delaunay_->cell_to_v() + 3*c;
    for(index_t e=0; e<3; ++e) {
      signed_index_t v1 = cell[e];
      signed_index_t v2 = cell[(e+1)%3];
      glupVertex2dv(delaunay_->vertex_ptr(index_t(v1)));
      glupVertex2dv(delaunay_->vertex_ptr(index_t(v2)));
    }
  }
  glupEnd();
```
Each pair of calls to `glupVertex()` between `glupBegin(GLUP_LINES)`
and `glupEnd()` results in a segment drawn on the screen.

Let us now see how to draw the Voronoi cells. It is slightly more complicated, because
they are not represented explicitly. What we need to do is deducing them "on the fly"
from the Delaunay triangulation.


_WIP_