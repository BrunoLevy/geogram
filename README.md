# geogram

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Release](https://github.com/BrunoLevy/geogram/actions/workflows/make_release.yml/badge.svg)](https://github.com/BrunoLevy/geogram/actions/workflows/make_release.yml)
[![Emscripten](https://github.com/BrunoLevy/geogram/actions/workflows/emscripten.yml/badge.svg)](https://github.com/BrunoLevy/geogram/actions/workflows/emscripten.yml)
[![Doxygen](https://github.com/BrunoLevy/geogram/actions/workflows/doxygen.yml/badge.svg)](https://github.com/BrunoLevy/geogram/actions/workflows/doxygen.yml)

[![Continuous](https://github.com/BrunoLevy/geogram/actions/workflows/continuous.yml/badge.svg)](https://github.com/BrunoLevy/geogram/actions/workflows/continuous.yml)
[![Continuous](https://custom-icon-badges.demolab.com/badge/CI-Continuous-lightblue?logo=tasklist&logoColor=white)](https://brunolevy.github.io/geogram.CI/reports/smoke/)

[![Nightly](https://github.com/BrunoLevy/geogram/actions/workflows/nightly.yml/badge.svg)](https://github.com/BrunoLevy/geogram/actions/workflows/nightly.yml)
[![Nightly](https://custom-icon-badges.demolab.com/badge/CI-Nightly-lightblue?logo=tasklist&logoColor=white)](https://brunolevy.github.io/geogram.CI/reports/nightly/)



![](https://github.com/BrunoLevy/geogram/wiki/geogram_banner_2026.png)


Geogram is a programming library with geometric algorithms. It has
geometry-processing functionalities:
- [surface reconstruction](https://github.com/BrunoLevy/geogram/wiki/Reconstruction)
- [remeshing](https://github.com/BrunoLevy/geogram/wiki/Remeshing)
- [parameterization and texturing](https://github.com/BrunoLevy/geogram/wiki/Texturing)
- [Intersections and Boolean operations](https://github.com/BrunoLevy/geogram/wiki/BooleanOps)
- [Constructive Solid Geometry](https://github.com/BrunoLevy/geogram/wiki/CSG)

It also has lower-level algorithm:
- [Exact numbers / exact predicates](https://github.com/BrunoLevy/geogram/wiki/Exact)
- [Delaunay triangulations in 2D](https://github.com/BrunoLevy/geogram/wiki/Delaunay2D)
  and highly efficient parallel [Delaunay triangulations in 3D](https://github.com/BrunoLevy/geogram/wiki/Delaunay3D)
- A multithread-friendly 2D constrained Delaunay triangulation, that supports intersecting constraints, in arbitrary precision.
- Memory efficient surfacic/volumetric/hybrid [mesh data structure](https://github.com/BrunoLevy/geogram/wiki/Mesh)
- Efficient [geometric search data structures](https://github.com/BrunoLevy/geogram/wiki/Raytrace) for
  intersection and raytracing (AABBs, KdTrees, ...)
- [Spectral mesh processing](https://github.com/BrunoLevy/geogram/wiki/ManifoldHarmonics)
- [Linear solver on CPU and GPU](https://github.com/BrunoLevy/geogram/wiki/OpenNL)

Geogram received the [Symposium on Geometry Processing Software Award](http://awards.geometryprocessing.org/)
in 2023.

*History* Geogram is the result of a sustained development effort that started
in Y2K (Bruno's post-doc in Stanford). It contains the main results in
Geometry Processing from the former ALICE Inria project (2004-2018),
that is, more than 30 research articles published in ACM SIGGRAPH,
ACM Transactions on Graphics, Symposium on Geometry
Processing and Eurographics, all these results maintained and reproducible, and
some of them used in industrial products. Geogram was supported by two grants from the
European Research Council (ERC): GOODSHAPE and VORPALINE, and two
Exploratory Action grants from Inria (AeX): EXPLORAGRAM and COSMOGRAM
(the latter still active), pushing the limits of Geogram to large-scale
computational physics and Optimal Transport, in the frame of the
PARMA Inria project.


Links
-----
  - [Documentation, how to compile, tutorials....](https://github.com/BrunoLevy/geogram/wiki)
  - [Programmer's reference manuals...](https://brunolevy.github.io/geogram/)
  - [Releases](https://github.com/BrunoLevy/geogram/releases)
  - [Projects with geogram](https://github.com/BrunoLevy/geogram/wiki/Publications)
  - [Graphite](https://github.com/BrunoLevy/GraphiteThree), an experimental 3D modeler built around geogram.
  - [Geogram in-browser demos](https://github.com/BrunoLevy/geogram/wiki/compiling_Emscripten#examples)
    (How is it possible ? _more on this [here](https://github.com/BrunoLevy/geogram/wiki/compiling_Emscripten)_)
  - [Data](https://github.com/BrunoLevy/GraphiteThree/wiki/Data)

How does it compare to other geometry-processing libraries ?
------------------------------------------------------------

See [FAQ](https://github.com/BrunoLevy/geogram/wiki/FAQ)
