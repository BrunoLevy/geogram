# geogram

Geogram is a programming library with geometric algorithms.
It contains the main results in Geometry Processing from the former
ALICE Inria project, that is, more than 30 research articles published
in ACM SIGGRAPH, ACM Transactions on Graphics, Symposium on Geometry 
Processing and Eurographics. It was supported by two grants from the
European Research Council (ERC): GOODSHAPE and VORPALINE.

Geogram includes a simple yet efficient Mesh data structure (for surfacic
and volumetric meshes), exact computer arithmetics (a-la Shewchuck,
implemented in GEO::expansion ), a predicate code generator (PCK: the
Predicate Construction Kit), standard geometric predicates
(orient/insphere), Delaunay triangulation, Voronoi diagram, spatial
search data structures, spatial sorting) and less standard ones (more
general geometric predicates, intersection between a Voronoi diagram
and a triangular or tetrahedral mesh embedded in n dimensions), and 
semi-discrete optimal transport.

Compiling
---------
  - [Instructions for Linux](doc/tutorials/compiling_Linux.md)
  - [Instructions for Windows](doc/tutorials/compiling_Windows.md)
  - [Instructions for OS/X](doc/tutorials/compiling_MacOS.md)

Design principles
-----------------

- Make it as <b> simple as possible </b> _(but not simpler)_
- Make it as <b> easy to use as possible </b>
- Make it as <b> easy to compile as possible </b>
- Maximize <b> speed </b>
- Minimize <b> memory consumption </b>
- <b> Minimize </b> number of <b> lines of code </b>
- <b> Minimize </b> number of <b> C++ classes </b>

<p>
<it>Simplicity is the ultimate sophistication [Leonardo da Vinci] </it> 

<p>
 More on this in [this keynote Eurographics presentation](https://fr.slideshare.net/BrunoLevy4/the-joy-of-computer-graphics-programming).

