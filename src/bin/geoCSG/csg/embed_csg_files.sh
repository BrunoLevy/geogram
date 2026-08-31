#!/bin/sh

csg_files="
example001.csg
example002.csg
example003.csg
example004.csg
example005.csg
"

cat <<EOF
/*
 * This file was automatically generated, do not edit.
 */

#include <geogram/basic/file_system.h>

void register_embedded_csg_files(
   GEO::FileSystem::MemoryNode* n
);

void register_embedded_csg_files(
   GEO::FileSystem::MemoryNode* n
) {
EOF

for f in $csg_files
do
    echo "     n->create_file(\"$f\","
    cat $f | sed -e 's|\\|\\\\|' \
                 -e 's|"|\\\"|g' \
	         -e 's|^|        \"|' \
	         -e 's| *$|\\n\"|' 
    echo "     );"
    echo
done

echo "}"
