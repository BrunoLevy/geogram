

# extract_psm: extracts a PSM (Pluggable Software Module)
# from geogram source tree.
#
# a PSM is a functionality of Geogram made available through a
# standalone .h/.cpp pair for users interested in that
# functionality who do not want to use the whole geogram package.

# remove_copyright()
#
# Arguments: none (stream function)
# Variables: none
#
# Removes the first comment from a source file
# (the one that contains the copyright)

remove_copyright() {
    awk '
    BEGIN {
      state = 0;
    } {
      if(state == 0) {
        if($0 ~ /\/\*.*/) {
          state = 1;
        }
      } else if(state == 1) {
        if($0 ~ / \*\/.*/) {
          state = 2;
        }
      } else {
        print;
      }
    }'
}

# extract_copyright()
#
# Arguments: $1: path to a sourcefile relative
#   to $PSM_DIR
# Variables: $PSM_DIR: the directory that
#   contains the .psm file
#
#   Extracts the copyright from $1 and sends it to the
# standard output.

extract_copyright() {
    cat $PSM_DIR/$1 | awk '
    BEGIN {
      state = 0;
    } {
      if(state == 0) {
        print;
        if($0 ~ /\/\*.*/) {
          state = 1;
        }
      } else if(state == 1) {
        print;
        if($0 ~ / \*\/.*/) {
          state = 2;
        }
      }
    }'
}

# remove_includes()
#
# Arguments: none (stream function)
# Variables: none
#
# Removes all include statements referring to Geogram files

remove_includes() {
    egrep -v '#include "' | egrep -v '#include <geogram/'
}

# remove_doxygen()
#
# Arguments: none (stream function)
# Variables: none
#
# Removes all Doxygen comments
# (note: removes also /**/ comments)

remove_doxygen() {
    sed -e 's|/\*\*/||g' -e 's|/\*\*.*\*/||g' | awk '
    BEGIN {
      state = 0;
    } {
      if(state == 0) {
        if($0 ~ /\/\*\*.*/) {
          state = 1;
        } else {
          print;
        }
      } else if(state == 1) {
        if($0 ~ / \*\/.*/) {
          state = 0;
        }
      }
    }'
}

# extract_file()
#
# Arguments: $1: path to a sourcefile relative
#   to $PSM_DIR
# Variables: $PSM_DIR: the directory that
#   contains the .psm file
#
#   Extracts a sourcefile, processes it to remove
# unnecessary information and sends it to the standard
# output.

extract_file() {
    echo
    echo "/******* extracted from" $1 "*******/"
    cat $PSM_DIR/$1 | remove_copyright | remove_includes | remove_doxygen
}

# extract_files()
#
# Arguments: $@: list of files to be extracted, all paths
#   relative to $PSM_DIR
# Variables: $PSM_DIR: the directory that
#   contains the .psm file
#
#   Extracts a list of sourcefiles, processes them to remove
# unnecessary information and sends them to the standard
# output.

extract_files() {
    extract_copyright $1
    cat <<EOF


/*
 *  This file is a PSM (pluggable software module)
 *   generated from the distribution of Geogram.
 *
 *  See Geogram documentation on:
 *   http://alice.loria.fr/software/geogram/doc/html/index.html
 *
 *  See documentation of the functions bundled in this PSM on:
 *   $PSM_DOC
 */


EOF
    for i in $@
    do
        extract_file $i
    done
}

# extract_header()
#
# Arguments: none
# Variables: $PSM_HEADER the absolute path of the generated header
#            $HEADERS the list of header files to be assembled, each
#             of them relative to $PSM_DIR
#            $PSM_DIR: the directory that contains the .psm file
#
# Assembles the header ($PSM_HEADER) from the source headers ($HEADERS)

extract_header() {
    echo "   generating" $PSM_HEADER ...
    cat <<EOF >$PSM_HEADER
#define GEOGRAM_PSM
#ifndef GEO_STATIC_LIBS
#define GEO_DYNAMIC_LIBS
#endif
EOF
    extract_files $HEADERS >> $PSM_HEADER
}

# extract_source()
#
# Arguments: none
# Variables: $PSM_SOURCE the absolute path of the generated header
#            $SOURCES the list of header files to be assembled, each
#             of them relative to $PSM_DIR
#            $PSM_DIR: the directory that contains the .psm file
#
# Assembles the source ($PSM_SOURCE) from the sources ($SOURCES)

extract_source() {
    echo "   generating" $PSM_SOURCE ...
    echo '#include "'$PSM_HEADER'"' > $PSM_SOURCE
    echo >> $PSM_SOURCE
    extract_files $SOURCES >> $PSM_SOURCE
}

# extract_example()
#
# Arguments: none
# Variables: $EXAMPLE: path to the example source, relative to $PSM_DIR
#            $PSM_DIR: the directory that contains the .psm file
#
# If $EXAMPLE is a non-empty string, extracts it.

extract_example() {
    if [ -z $EXAMPLE ]
    then
        echo "   No example file"
        return;
    fi
    PSM_EXAMPLE_BIN=$PSM_NAME"_example"
    PSM_EXAMPLE=$PSM_NAME"_example.c"
    COMPILER=gcc
    if [[ $EXAMPLE =~ ^.*\.cpp$ ]]
    then
        PSM_EXAMPLE=$PSM_NAME"_example.cpp"
	COMPILER=g++
    fi
    echo "   generating $PSM_EXAMPLE ..."
    cat <<EOF > $PSM_EXAMPLE
/*
 * To compile under Linux:
 *   $COMPILER -O3 -fopenmp -frounding-math -ffp-contract=off --std=c++20 $PSM_EXAMPLE $PSM_SOURCE -o $PSM_EXAMPLE_BIN -ldl -lm
 */
EOF
    echo >> $PSM_EXAMPLE
    extract_copyright $EXAMPLE >> $PSM_EXAMPLE
    echo '#include "'$PSM_HEADER'"' >> $PSM_EXAMPLE
    cat $PSM_DIR/$EXAMPLE | remove_copyright | remove_includes >> $PSM_EXAMPLE
}


# extract_copydir()
#
# Arguments: none
# Variables: $COPYDIR: path to the subdirectory to copy, relative to $PSM_DIR
#            $PSM_DIR: the directory that contains the .psm file
#
# If $COPYDIR is a non-empty string, copy the subdirectory into the PSM distrib.

extract_copydir() {
   if [ -z $COPYDIR ]
   then
       return;
   fi
   cp -r $PSM_DIR/$COPYDIR .
}

# extract_cmake()
#
# Arguments: none
# Variables: $PSM_NAME, $PSM_HEADER, $PSM_SOURCE
#            $EXAMPLE, $PSM_EXAMPLE: the example, if any
#
# Generates a CMakeLists.txt that builds the PSM as a static library,
# and its example program if any.

extract_cmake() {
    echo "   generating CMakeLists.txt ..."
    PSM_LOWER=`echo $PSM_NAME | tr '[:upper:]' '[:lower:]'`
    PSM_TARGET=geogram_$PSM_LOWER
    PSM_ALIAS=geogram::$PSM_LOWER
    PSM_LANGUAGE=CXX
    PSM_STANDARD=cxx_std_17
    if [[ $PSM_SOURCE =~ ^.*\.c$ ]]
    then
        PSM_LANGUAGE=C
        PSM_STANDARD=c_std_99
    fi
    cat <<EOF > CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

project($PSM_TARGET LANGUAGES $PSM_LANGUAGE)

find_package(Threads REQUIRED)

# Public inline arithmetic uses globals that are not exported by Windows DLLs.
add_library($PSM_TARGET STATIC $PSM_SOURCE $PSM_HEADER)
add_library($PSM_ALIAS ALIAS $PSM_TARGET)

target_include_directories($PSM_TARGET PUBLIC "\${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries($PSM_TARGET PUBLIC Threads::Threads)
target_compile_features($PSM_TARGET PUBLIC $PSM_STANDARD)
target_compile_definitions($PSM_TARGET PUBLIC GEO_STATIC_LIBS)

# Exact predicates, including inline code in the public header, must not
# contract multiply-add operations into FMAs.
if(MSVC)
    target_compile_options($PSM_TARGET PUBLIC /fp:strict)
elseif(CMAKE_${PSM_LANGUAGE}_COMPILER_ID MATCHES "^(GNU|Clang|AppleClang)\$")
    target_compile_options($PSM_TARGET PUBLIC -ffp-contract=off)
endif()
EOF
    if [ -n "$EXAMPLE" ]
    then
        cat <<EOF >> CMakeLists.txt

add_executable(${PSM_TARGET}_example $PSM_EXAMPLE)
target_link_libraries(${PSM_TARGET}_example PRIVATE $PSM_ALIAS)
EOF
    fi
}

# extract_cmake_readme()
#
# Arguments: none
# Variables: $PSM_ALIAS: the CMake target generated by extract_cmake()
#
# Documents the generated CMakeLists.txt. It is appended after
# extract_readme(), which .psm files may redefine.

extract_cmake_readme() {
    cat <<EOF

The included CMakeLists.txt builds the PSM as a static library (CMake target
$PSM_ALIAS), and the example program if any:
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
  cmake --build build --config Release
EOF
}

extract_readme() {
    cat <<EOF
This is $PSM_NAME Pluggable Software Module, extracted
from GEOGRAM source tree. It contains a standalone .cpp/.h
pair that can be used in any program and that does not have
any dependency.

It may also contain an example program that can be compiled by using:
  g++ --std=c++20 ${PSM_NAME}_psm.cpp ${PSM_NAME}_example.cpp -o ${PSM_NAME}_example
(or gcc if it is plain C, as in OpenNL)

Some examples may require additional compilation flags (see comments at the beginning
of the example source, e.g. Delaunay_example.cpp).
EOF
}

# Get directory and PSM name from PSM file name
PSM_DIR=`dirname $1`
PSM_NAME=`basename $1 .psm`

# Read the PSM file, and get the value of
#  $SOURCES, $HEADERS, $DOC and $EXAMPLE
. $1
echo Extracting $PSM_NAME Pluggable Software Module...

PSM_HEADER=$PSM_NAME"_psm.h"
PSM_SOURCE=$PSM_NAME"_psm.c"

# If sources contain C++ files, then generated source is
# a C++ file
if [[ $SOURCES =~ ^.*\.cpp.*$ ]]
then
    PSM_SOURCE=$PSM_NAME"_psm.cpp"
fi

PSM_DOC=$DOC

extract_header
extract_source
extract_example
extract_copydir
extract_cmake
extract_readme > README.txt
extract_cmake_readme >> README.txt
