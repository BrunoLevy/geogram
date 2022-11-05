# make_geogram_dist.sh
# Generates Geogram distribution from Vorpaline sourcetree

# copy_dist()
#
# Arguments: none
# Variables: $DIST_DIR: where to copy the distribution
#
#  Copies the current distribution into $DIST_DIR
# and removes Vorpaline-specific files

copy_dist() {
    rm -fr $DIST_DIR
    mkdir -p $DIST_DIR
    cp -r * $DIST_DIR/
    find $DIST_DIR -name ".git*" -prune -exec rm -fr {} \;
}

# create_archives()
#
# Arguments: $1 full path to the directory to be archived
# Variables: none
#
#   Creates $1.tar.gz and $1.zip in the directory
# that contains $1, then deletes $1

create_archives() {
    echo Archiving $1 ...
    (cd `dirname $1`; tar cvfz `basename $1`.tar.gz `basename $1` &> /dev/null)
    (cd `dirname $1`; zip -r `basename $1`.zip `basename $1` &> /dev/null)
    echo Cleaning-up $1
    rm -fr $1
    echo
}

# create_PSM_archives()
#
# Arguments:
#  $@: the list of .psm files, paths relative to $DIST_DIR
# Variables:
#  $DIST_BASE: where all the archives are generated
#  $DIST_DIR: where the distribution was copied
#
# Creates a pair of .tar.gz/.zip archives for each PSM.

create_PSM_archives() {
    echo "***Extracting Pluggable Software Modules***"
    echo
    for PSMFILE in $@
    do
        PSM=`basename $PSMFILE .psm`
        PSMDIR=$PSM'_psm_'$VERSION
        (cd $DIST_BASE; mkdir $PSMDIR; cd $PSMDIR; \
         $DIST_DIR/tools/extract_psm.sh $DIST_DIR/$PSMFILE)
        create_archives $DIST_BASE/$PSMDIR
    done
    echo "*******************************************"
    echo
}

# create_doc_archive()
#
# Arguments: none
# Variables:
#  $DIST_BASE: where all the archives are generated
#  $DIST_DIR: where the distribution was copied
#   Generates the Doxygen documentation and creates a
# .tar.gz archive of it.

create_doc_archive() {
    echo "Generating documentation archive ...."
    echo "   Configuring build tree"
    (cd $DIST_DIR; ./configure.sh Linux64-gcc &> /dev/null)
    echo "   Generating documentation"
    (cd $DIST_DIR/build/Linux64-gcc-Release; \
     make doc-devkit-full &> /dev/null; \
     cd doc/devkit-full; \
     tar cvfz $DIST_BASE/geogram_$VERSION-doc.tar.gz html &> /dev/null)
    echo "   Cleaning-up build tree"
    rm -fr $DIST_DIR/build
    echo 
}


# create_debian_package()
#
# Arguments: none
# Variables:
#  $DIST_BASE: where all the archives are generated
#  $DIST_DIR: where the distribution was copied
#   Generates the Debian package

create_debian_package() {
    echo "Generating Debian package ...."
    echo "   Configuring build tree"
    (cd $DIST_DIR; echo "set(CPACK_GENERATOR DEB)" > CMakeOptions.txt)
    (cd $DIST_DIR; ./configure.sh Linux64-gcc-dynamic &> /dev/null)
    echo "   Generating Debian package"
    (cd $DIST_DIR/build/Linux64-gcc-dynamic-Release; \
     make -j 8 package &> /dev/null; \
     cp *.deb $DIST_BASE/ &> /dev/null)
    echo "   Cleaning-up build tree"
    rm -fr $DIST_DIR/build
    echo 
}


# create_RPM_package()
#
# Arguments: none
# Variables:
#  $DIST_BASE: where all the archives are generated
#  $DIST_DIR: where the distribution was copied
#   Generates the Debian package

create_RPM_package() {
    echo "Generating RPM package ...."
    echo "   Configuring build tree"
    (cd $DIST_DIR; echo "set(CPACK_GENERATOR RPM)" > CMakeOptions.txt)
    (cd $DIST_DIR; ./configure.sh Linux64-gcc-dynamic &> /dev/null)
    echo "   Generating RPM package"
    (cd $DIST_DIR/build/Linux64-gcc-dynamic-Release; \
     make -j 8 package &> /dev/null; \
     cp *.rpm $DIST_BASE/ &> /dev/null)
    echo "   Cleaning-up build tree"
    rm -fr $DIST_DIR/build
    echo 
}

usage() {
   cat <<EOF
      make_geogram_dist.sh (-q|--quick) (-no-doc) (-no-tar-zip) (-no-deb) (-no-rpm)
EOF
}

while [[ $# -gt 0 ]]
do
   arg="$1"
   case $arg in
      -q|-quick)
         QUICK=1
         ;;
      -no-doc)
	  NO_DOC=1
	  ;;
      -no-deb)
	  NO_DEB=1
	  ;;
      -no-rpm)
	  NO_RPM=1
	  ;;
      -no-tar-zip)
	  NO_TAR_ZIP=1
	  ;;
      *|-h|/?)
         usage
         exit
      ;;
   esac
   shift
done


VERSION_MAJOR=`cat CMakeLists.txt | grep 'set(VORPALINE_VERSION_MAJOR' | sed -e 's|[^0-9]||g'`
VERSION_MINOR=`cat CMakeLists.txt | grep 'set(VORPALINE_VERSION_MINOR' | sed -e 's|[^0-9]||g'`
VERSION_PATCH=`cat CMakeLists.txt | grep 'set(VORPALINE_VERSION_PATCH' | sed -e 's|[^0-9]||g'`
VERSION=$VERSION_MAJOR.$VERSION_MINOR.$VERSION_PATCH
DIST_BASE=/tmp/GEOGRAM
DIST_NAME=geogram_$VERSION
DIST_DIR=$DIST_BASE/$DIST_NAME
rm -fr $DIST_BASE
mkdir -p $DIST_BASE

cat <<EOF

******
******
***
*** Generating Geogram $VERSION distribution
***
******
******

EOF

copy_dist

PSMFILES="\
src/lib/geogram/NL/OpenNL.psm \
src/lib/geogram/numerics/MultiPrecision.psm \
src/lib/geogram/numerics/Predicates.psm \
src/lib/geogram/delaunay/Delaunay.psm
"

create_PSM_archives $PSMFILES

if [ $QUICK ]; then
   echo
   echo "**** Skipping package creation (-quick)"
   echo
else
   echo
   echo "**** Creating packages"
   echo
   if [ ! $NO_DOC ]; then
      create_doc_archive
   fi
   if [ ! $NO_DEB ]; then
      create_debian_package
   fi
   if [ ! $NO_RPM ]; then
      create_RPM_package
   fi
fi   
if [ ! $NO_TAR_ZIP ]; then
   create_archives $DIST_DIR
else
   echo Cleaning-up $DIST_DIR
   rm -fr $DIST_DIR
fi

echo "Archive files generated in " $DIST_BASE ":"
echo "*******************************************"
echo
ls -al $DIST_BASE

cat <<EOF

******
******
***                                    
*** Geogram $VERSION distribution generated 
***
******
******

EOF
