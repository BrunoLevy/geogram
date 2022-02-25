# 3D Delaunay triangulations and Voronoi diagram

![](Delaunay3D.png)

The [example program](https://github.com/BrunoLevy/geogram/blob/main/src/examples/graphics/demo_Delaunay3d/main.cpp) 
computes a 2D Delaunay triangulation and the dual Voronoi diagram. Try
it [online](https://members.loria.fr/Bruno.Levy/GEOGRAM/geogram_demo_Delaunay3d.html).
The pointset can be optimized by Lloyd relaxation.

The Delaunay class
------------------

The same base class is shared by Delaunay in 2d and 3d. The base class is presented in
[this tutorial](https://github.com/BrunoLevy/geogram/blob/main/doc/tutorials/Delaunay2D.md).

There are several implementations of 3d Delaunay triangulations in Geogram:
- [Delaunay3d](https://github.com/BrunoLevy/geogram/blob/main/src/lib/geogram/delaunay/delaunay_3d.h):
    the simplest implementation, heavily documented, kept here for reference.
- [ParallelDelaunay3d](https://github.com/BrunoLevy/geogram/blob/main/src/lib/geogram/delaunay/parallel_delaunay_3d.h):
    a highly optimized multithreaded implementation.
- [PeriodicDelaunay3d](https://github.com/BrunoLevy/geogram/blob/main/src/lib/geogram/delaunay/periodic_delaunay_3d.h):
    a highly optimized multithreaded implementation that has in addition the possibility of handling periodic
    boundary conditions. This one is useful for research in astrophysics and material science that often use this type
    of boundary condition. 

Cell incidence (cell to vertex) and cell adjacency (cell to cell) queries work as with Delaunay2d (with the difference that
each cell is a tetrahedron, with 4 vertices and 4 neighbors, instead of 3 vertices and 3 neighbors in 2D).

Let us now focus on `PeriodicDelaunay3d` (that can do everything, and that has functions to select which feature to use).
The constructor has an option to specify whether periodic boundary conditions are used, and what the period is (default
is the unit cube):

```c++
  PeriodicDelaunay3d(bool periodic, double period=1.0)
```

It can be constructed as follows:
```c++
  SmartPointer<PeriodicDelaunay3d> delaunay = new PeriodicDelaunay3d(periodic_, 1.0);
```

The triangulation can be computed as follows:
```c++
  delaunay_->set_vertices(nb_points, points);
  delaunay_->compute();
```
where `points` is a pointer to the vertices coordinates, in a contiguous array (same thing as `Delaunay2d`,
but this time three coordinates per vertex). Once the vertices specified, one needs to call the `compute()`
function (it is so because one can specify additional information before calling `compute()`, more on this
below).

When in non-periodic mode, Delaunay uses a special vertex "at infinity" and virtual inifinite tetrahedra that
connect all facets of the convex hull. By default, these tetrahedra are discarted at the end of the construction
of the triangulation. One may want to keep them (for instance, to get the convex hull from the tetrahedra
incident to the inifinite vertex). It can be done using `delaunay_->set_keeps_infinite(true);`.

Drawing the Voronoi diagram
---------------------------

As with `Delaunay2d`, the Voronoi diagram can be deduced "on the fly" from the Delaunay triangulation. To
get the Voronoi cell associated with a given vertex, one needs to iterate on the tetrahedra incident to
a given vertex, compute the circumcenter of each tetrahedron, and connect it to the circumcenters of the
neighborhing tetrahedra. To make this easier, there is a helper `ConvexCell` class, that represents a
convex polytope in 3D, that is, the intersection of half-spaces (delimited by oriented planes).
`ConvexCell` represents a polytope in dual form, that is, as a triangulation where
each vertex corresponds to a plane equation, and where each triangle corresponds to a vertex. In our
context, we want to represent the Voronoi cell of a vertex `v`. The triangles of the `ConvexCell`
corresponds to the boundary of the set of tetrahedra incident to vertex `v`. There is a helper function
`copy_Laguerre_cell_from_Delaunay()` to directly copy a Voronoi cell of a 3d Delaunay triangulation
to a `ConvexCell`. Since this function needs to traverse the tetrahedra incident to the vertex, it takes
as an argument a working variable to store the list of tetrahedra (client code does not have to worry
about that, it is just a workspace passed to the function, having it makes multithreading more efficient
when one extracts Voronoi cells in concurrent threads):

```c++
   ConvexCell C;
   PeriodicDelaunay3d::IncidentTetrahedra W;
   for(index_t i=0; i<delaunay_->nb_vertices(); ++i) {
      delaunay_->copy_Laguerre_cell_from_Delaunay(v, C, W);
      // do something with C
   }
```

Now let us see how to display a given Voronoi cell. Remember, `ConvexCell` stores a convex polytope in dual
form. Iterating over the faces and vertices of each face can be done as follows:

```c++
for(index_t v=1; v<C.nb_v(); ++v) {
    index_t t = C.vertex_triangle(v);
    if(t == VBW::END_OF_LIST) {
	continue;
    }
    do {
	vec3 p = C.triangle_point(VBW::ushort(t));
	// Do something with p ...
	index_t lv = C.triangle_find_vertex(t,v);		   
	t = C.triangle_adjacent(t, (lv + 1)%3);
    } while(t != C.vertex_triangle(v));
}
```
- The loop starts as index 1 (instead of 0), because in `ConvexCell`,
  vertex 0 is a virtual vertex at infinity;
- Each vertex `v` of `ConvexCell` knows one of the triangles incident to it,
   through `C.vertex_triangle(v)`. It is set to `VBW::END_OF_LIST` if there
   is no such triangle;
- If you look at the function `draw_cell()` in the
   [example program](https://github.com/BrunoLevy/geogram/blob/main/src/examples/graphics/demo_Delaunay3d/main.cpp), you
   will see how to triangulate each face and send the triangles to GLUP to
   draw them.

Convex clipping
---------------

As in the 2D example, if we are not using periodic mode, the vertices on the border of the convex hull have infinite
Voronoi cells (with triangles incident to the infinite vertex 0 of the `ConvexCell`). We can compute the intersection
between the `ConvexCell` and a convex polytope (for instance unit cube), as follows:

```c++
   ConvexCell C = ...;
   C.clip_by_plane(vec4( 1.0, 0.0, 0.0, 0.0));
   C.clip_by_plane(vec4(-1.0, 0.0, 0.0, 1.0));
   C.clip_by_plane(vec4( 0.0, 1.0, 0.0, 0.0));
   C.clip_by_plane(vec4( 0.0,-1.0, 0.0, 1.0));	
   C.clip_by_plane(vec4( 0.0, 0.0, 1.0, 0.0));
   C.clip_by_plane(vec4( 0.0, 0.0,-1.0, 1.0));
   C.compute_geometry();
```

The code above first clips the cell with the six planes that define the unit cube. `ConvexCell` has a member function
`clip_by_plane()` that implements a 3D version of the Sutherland-Hogdman reentrant clipping. Under the hood, it uses
exact predicates to make sure that all combinatorial decisions remain coherent. One needs to call `C.compute_geometry()`
in the end to compute the coordinates of the vertices.

In the [example program](https://github.com/BrunoLevy/geogram/blob/main/src/examples/graphics/demo_Delaunay3d/main.cpp),
copying the cell from the Delaunay triangulation and clipping it is implemented in a `get_cell()` function.

Lloyd relaxation
----------------

![](Delaunay3D_Lloyd.gif)

Now we have all the components to implement Lloyd relaxation (that relocates each vertex at the
centroid of its Voronoi cell), as follows:

```c++
void Lloyd_iteration() {
    vector<double> new_points(points_.size());
    ConvexCell C;
    for(index_t v=0; v<points_.size()/3; ++v) {
	get_cell(v, C);
	vec3 g = C.barycenter();
	new_points[3*v]   = g.x;
	new_points[3*v+1] = g.y;
	new_points[3*v+2] = g.z;		
    }
    // In periodic mode, points may escape out of
    // the domain. Relocate them in [0,1]^3
    for(index_t i=0; i<new_points.size(); ++i) {
	if(new_points[i] < 0.0) {
	    new_points[i] += 1.0;
	}
	if(new_points[i] > 1.0) {
	    new_points[i] -= 1.0;
	}
    }
    points_.swap(new_points);
    delaunay_->set_vertices(points_.size()/3, points_.data());
    delaunay_->compute();
}
```

