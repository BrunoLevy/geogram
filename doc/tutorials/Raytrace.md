# Raytracing and Axis-Aligned Bounding Box Tree

![](Raytrace.gif)

_Back to the 90's ! Let us draw some shiny spheres and checkerboards_

Geogram includes efficient primitives for accelerated geometric
queries on meshes (AABB: Axis-Aligned Bounding Box Tree). The included
example program demonstrates it by including a mesh in a raytracing
scene, displayed at interactive speed with a purely software
implementation (no GPU). The sources are here:

- [example program (GUI)](https://github.com/BrunoLevy/geogram/blob/main/src/examples/graphics/demo_Raytrace/main.cpp)
- [example program (command line)](https://github.com/BrunoLevy/geogram/blob/main/src/examples/geogram/simple_raytrace/main.cpp)
- [raytracing core (used by both)](https://github.com/BrunoLevy/geogram/blob/main/src/examples/geogram/simple_raytrace/raytracing.h)

Raytracing basics
-----------------
About raytracing basics, the following sources and tutorials are recommended:
- [Dmitry Sokolov's tinyraytracer](https://github.com/ssloy/tinyraytracer/wiki/Part-1:-understandable-raytracing)
- [Peter Sherley's raytracing in one weekend](https://raytracing.github.io/books/RayTracingInOneWeekend.html)

The minimalistic [raytracing core](https://github.com/BrunoLevy/geogram/blob/main/src/examples/geogram/simple_raytrace/raytracing.h)
is very classical. It is not meant to be a part of the library, it just has a minimalistic set of classes and interface
for demo purposes. It has the following classes:

- `Camera`: launches primary rays (`launch_ray()`), stores the computed image, and saves the image in PPM format (`save_image()`).
- `Object`: has two virtual functions that test ray-object intersections. The first one, `find_nearest_intersection()` does what
   it says. The second one, `in_shadow()`, only tests for the presence of an intersection (used to determine if a point is in the
   shadow with respect to a light source). It also stores a `Material`, and has functions to edit it in the GUI (when the macro
   `RAYTRACE_GUI` is defined).
- `Sphere`, `Light`, and `HorizontalCheckerboardPlane` (because it is the tradition !)
- `MeshObject`, detailed below.


Raytracing meshes
-----------------

To raytrace a mesh, the basic primitive is a function to compute the
intersection between a ray and a triangle. It is explained in this
[stackoverflow answer](https://stackoverflow.com/questions/42740765/intersection-between-line-and-triangle-in-3d). Now if your mesh has a large number of
triangles, calling the function for each triangle will be very unefficient. 
To accelerate the computations, the idea is to embed the mesh into a hierarchy of boxes: start with a ray-box intersection, if there is an intersection,
then open the box, that can contain either other boxes or a list of triangles. To grasp a "visual" intuition of the idea, take a look at this
[ShaderToy](https://www.shadertoy.com/view/tl3XRN).

The idea of the algorithm is as follows:
```
    Intersect(ray, box):
       if there is an intersection between the ray and the box
          if the box contains a list of sub-boxes
             for each sub-box b
   	        Intersect(ray, b)
          else // box is a leaf with a list of triangle
             for each triangle t in the list
	        Intersect(ray, t)
          end
       end
    end
```

Now the question is how can we represent a tree of boxes. One may think of
using a `Node` struct, with pointers to children, but it is not a very good
idea for two reasons: first, all these pointers are going to take a considerable
amount of memory, and second, allocating all these structures in memory is going
to cause memory fragmentation. It is much more efficient to use a compact data
structure, with everything stored in contiguous arrays in memory. In addition,
we can also reorder the facets in the mesh, in such a way that facets present in
the same box are always contiguous in memory. Using this idea, the only thing
we need to store is the array of bounding boxes (the combinatorics is implicitly
encoded in the order of the facets in the mesh). It is even possible to avoid storing
the boxes, using [Pierre Terdiman's zero-memory AABB](http://www.codercorner.com/ZeroByteBVH.pdf)
(Geogram does not support them yet and stores the boxes). 

In Geogram, these acceleration structures are implements in
[MeshAABB.h](https://github.com/BrunoLevy/geogram/blob/main/src/lib/geogram/mesh/mesh_AABB.h).There
is one for surfacic meshes (`MeshFacetAABB`) and one for volumetric meshes (`MeshCellsAABB`). Besides
accelerating the ray-triangle intersection function, they can be used to compute all the facets or
cells intersections between two meshes (or within the same mesh).

We are now equipped to explain how `MeshObject` class works. It has a `Mesh` and
a `MeshFacetsAABB` member:

```c++
class MeshObject: public Object {
   ...
 private:
   Mesh mesh_;
   MeshFacetsAABB AABB_;
};
```

The constructor works as follows:
```
   MeshObject::MeshObject(const std::string& filename) {
      mesh_load(filename, mesh_);
      AABB_.initialize(mesh_);
   }
```
It first loads the specified filename into the `Mesh`, then constructs the `MeshFacetsAABB`.
Some details worth to mention:
- `MeshFacetsAABB` keeps a pointer to the `Mesh` (this is why we need to keep the mesh !)
- `MeshFacetsAABB` reorders the facets of the `Mesh`. It has an optional parameter to
   tell it not to do so, but then performance will drop dramatically if the facets
   are not spatially ordered in the mesh (but it can be interesting to use the option
   in the case you know you have already sorted the facets of the mesh).
- in the demo program, the constructor of `MeshObject` normalizes the coordinates of the
   mesh after loading (to make sure you see something in the demo whatever the coordinates
   in the file). It has an optional parameter to deactivate this behavior.

Now we can see how we can override `Object::get_nearest_intersection()`:
```c++
   void MeshObject::get_nearest_intersection(
   	const Ray& R, Intersection& I
   ) const override {
        MeshFacetsAABB::Intersection cur_I;
        if(AABB_.ray_nearest_intersection(R, cur_I)) {
   	   if(cur_I.t > epsilon_t && cur_I.t < I.t) {
	        I.t = cur_I.t;
	        I.object = this;
	        I.material = material_;
	        I.position = cur_I.p; 
	        I.normal = normalize(cur_I.N); 
	   }
	}
   }
```
The `ray_nearest_intersection()` function returns `true` of there was an
intersection and returns the intersection details in a `MeshFacetsAABB::Intersection`
struct, with the parameter `cur_I.t` along the ray, the intersection point `cur_I.p` and the
(not normalized !) normal vector `cur_I.N`. It also has the barycentric coordinates of the
intersection point in the triangle. Note that the (not normalized !) normal vector and barycentric coordinates
are given "for free" by the Moller and Trumbore algorithm that we are using (this is why
they are systematically returned in the intersection structure (doing so does not cost anything).

The function that tests for shadow rays intersections determines whether a given point
is in the shadow relative to a light source. It works as follows:

```c++
   bool MeshObject::in_shadow(const Ray& R) const override {
      vec3 p2 = R.origin + R.direction;
      return AABB_.segment_intersection(R.origin, p2);
   }
```

Now we have an object that can be used in any raytracing framework, including more
subtle lighting effect, such as refraction, as demonstrated in
this [shadertoy](https://www.shadertoy.com/view/3ltSzM) and
also [that one](https://www.shadertoy.com/view/WlcXRS) with smooth shading.

Multithreading
--------------

Geogram has a construct for running a loop in parallel, that can be used as
follows:

```c++
   parallel_for(0, N,
     [this, &](index_t i) {
        // ... do something with i
     }
   )
```
_Technically, the `parallel_for()` construct in Geogram is a function that takes
 as an argument a C++ unnamed function, also called a "lambda". The list of items
 inside the square brackets `[this,&]` indicates here that one can access all
 member functions and variables in the scope in the function. One can be more
 specific and indicate here the very list of variables meant to be accessed in
 the body of the loop__

It is equivalent to:

```c++
   for(index_t i=0; i<N; ++i) {
      // ... do something with i
   }
```

with the difference that several iterations can run in parallel.
Hence, the rendering loop can be written as follows:

```c++
  parallel_for(
    0, camera_.image_height(),
    [this](index_t Y) {
	for(index_t X=0; X<camera_.image_width(); ++X) {
	    Ray R = primary_ray(X,Y);
	    vec3 K = scene_.raytrace(R);
	    camera_.set_pixel(X,Y,K);
    }
  }

```

There are other topics to be covered about multithreading, namely sliced parallel loops,
spinlocks and synchronization primitives. Since we do not need them for raytracing, this
will be the topic of another tutorial.

Other functionalities of Geogram AABB classes
---------------------------------------------

The (surfacic) class `MeshFacetsAABB` has the following member functions:

- `compute_facet_bbox_intersections()`: it computes all intersections between
  the pair of facet bounding boxes in a mesh. It is used as follows:
```c++
   aabb_.compute_facet_bbox_intersections(
      [this,&](index_t f1, index_t f2) {
         // do something with f1 and f2
      }
   )
```
where `(f1,f2)` will iterate on all pair of facets which bounding box intersect.

- `nearest_facet()`: finds the facet nearest to a query point
- `ray_intersection()`: tests whether there exists an intersection with a `Ray`
- `ray_nearest_intersection()`: gets the nearest intersection along a `Ray`
- `segment_nearest_intersection()`: gets the nearest intersection along a segment

There is also a (volumetric) class `MeshCellsAABB`, that optimizes geometric queries
for volumetric meshes (made of tetrahedra or made of polyhedral cells). It has the
following functions:
- `containing_tet()`: find the tetrahedron that contains a given point
- `compute_bbox_cell_bbox_intersections()`: compute all the intersections between a
   given box and the bounding boxes of all the cells
- `containing_boxes()`: find all the cells which bounding boxes contain a given point
- `compute_cell_bbox_intersections()`: find all the pairs of cell bounding boxes that
   have an intersection within a given mesh
- `compute_other_cell_bbox_intersections()`: find all the pairs of cell bounding boxes
   that have an intersection in two different meshes

