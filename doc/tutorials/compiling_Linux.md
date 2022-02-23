# Geogram compilation on Linux

Prerequisites
-------------
- CMake
- g++ compiler
- X11 development libraries

On a Debian system, dependancies can be installed as follows:
```
$ sudo apt-get install cmake build-essential libX11-dev libxcb1-dev libxau-dev libxdmcp-dev 
```

Quick compilation guide
-----------------------
```
$ git clone https://github.com/BrunoLevy/geogram.git
$ cd geogram
$ ./configure.sh
$ cd Build/Linux64-gcc-dynamic-Release
$ make -j 8
```

This will compile the geogram library as well as demo programs. Demo
programs are generated in the `bin/` subdirectory.

