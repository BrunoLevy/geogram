A simple CGAL program that computes a 3D Delaunay triangulation,
with GEOGRAM's expansion_nt class plugged into it.

Note: it is meant to be used with the Pluggable Software Module.
You got the Pluggable Software Module if the file from which you
extracted this program is called something like MultiPrecision_psm_ver.tar.gz
or MultiPrecision_psm_ver.zip, where ver is the version number.
If you got the Geogram distribution, then you can generate the Pluggable
Software Module by using tools/extract_psm.sh

To compile it:
cmake . -D CMAKE_BUILD_TYPE=Release
make

To test:
cat ocean_r.xyz ./benchmark 10

Where 10 is the number of times the Delaunay triangulation is
constructed (in order to have sufficient long running times to measure
timings)

To compare with CGAL's arbitrary precision type:
Comment-out the line with "#define CGAL_USE_GEO_EXPANSION" in benchmark.cpp
and recompile it (make). Note: CGAL's arbitrary precision type is a bit faster
(I need to work more !!)
