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

Now the question is how can we represent lists of boxes. One may think of
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

_WIP_
