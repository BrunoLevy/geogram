# Geogram compilation on MacOS

Prerequisites
-------------
- CMake
- C++ compiler (clang)

Quick compilation guide
-----------------------
```
$ git clone https://github.com/BrunoLevy/geogram.git
$ cd geogram
$ ./configure.sh
$ cd Build/Darwin-clang-dynamic-Release
$ make -j 8
```

This will compile the geogram library as well as demo programs. Demo
programs are generated in the `bin/` subdirectory.

