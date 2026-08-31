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
     n->create_file("example001.csg",
        "// example001 adapted from OpenSCAD\n"
        "\n"
        "difference() {\n"
        "  sphere(25.0);\n"
        "  rotate(90.0, [0,0,1]) cylinder(62.5, 12.5, 12.5, center=true);\n"
        "  rotate(90.0, [1,0,0]) cylinder(62.5, 12.5, 12.5, center=true);\n"
        "  rotate(90.0, [0,1,0]) cylinder(62.5, 12.5, 12.5, center=true);\n"
        "}\n"
     );

     n->create_file("example002.csg",
        "// example002 adapted from OpenSCAD\n"
        "\n"
        "intersection() {\n"
        "   difference() {\n"
        "     union() {\n"
        "         cube([30, 30, 30], center=true);\n"
        "         translate([0, 0, -25]) cube([15, 15, 50], center=true);\n"
        "     }\n"
        "     union() {\n"
        "        cube([50, 10, 10], center=true);\n"
        "        cube([10, 50, 10], center=true);\n"
        "        cube([10, 10, 50], center=true);\n"
        "     }\n"
        "  }\n"
        "  translate([0, 0, 5]) cylinder(50,20,5, center=true);\n"
        "}\n"
     );

     n->create_file("example003.csg",
        "// example003 adapted from OpenSCAD\n"
        "\n"
        "difference() {\n"
        "   union() {\n"
        "     cube([30, 30, 30], center=true);\n"
        "     cube([40, 15, 15], center=true);\n"
        "     cube([15, 40, 15], center=true);\n"
        "     cube([15, 15, 40], center=true);\n"
        "   }\n"
        "   union() {\n"
        "     cube([50, 10, 10], center=true);\n"
        "     cube([10, 50, 10], center=true);\n"
        "     cube([10, 10, 50], center=true);\n"
        "   }\n"
        "}\n"
     );

     n->create_file("example004.csg",
        "// example004 adapted from OpenSCAD\n"
        "difference() {\n"
        "   cube([30,30,30],true);\n"
        "   sphere(20);\n"
        "}\n"
        "\n"
     );

     n->create_file("example005.csg",
        "// example005 adapted from OpenSCAD\n"
        "translate([0, 0, -120]) {\n"
        "   difference() {\n"
        "      cylinder(50, 100, 100);\n"
        "      translate([0, 0, 10]) cylinder(50, 80, 80);\n"
        "      translate([100, 0, 35]) cube([50, 50, 50], center=true);\n"
        "   }\n"
        "   rotate(36) translate([80, 0, 0]) cylinder(200, 10, 10);\n"
        "   rotate(108) translate([80, 0, 0]) cylinder(200, 10, 10);\n"
        "   rotate(180) translate([80, 0, 0]) cylinder(200, 10, 10);\n"
        "   rotate(252) translate([80, 0, 0]) cylinder(200, 10, 10);\n"
        "   rotate(324) translate([80, 0, 0]) cylinder(200, 10, 10);\n"
        "   translate([0, 0, 200]) cylinder(80, 120, 0);\n"
        "}\n"
     );

}
