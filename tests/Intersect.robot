*** Settings ***
Test Setup        Prepare Test
Test Teardown     Cleanup Test
Force Tags        Intersect    smoke    daily    daily_valgrind
Library           OperatingSystem
Library           String
Library           lib/VorpatestLibrary.py

*** Variables **
${DATADIR}        %{VORPATEST_ROOT_DIR}${/}data${/}Intersections

*** Test Cases ***
classif_1.obj
    Run Test

classif_2.obj
    Run Test

cube_with_8_spheres.obj
    Run Test

cucubes.obj
    Run Test

rot_seven_cubes.obj
    Run Test

sphere_icosa_2.obj
    Run Test

sphere_icosa.obj
    Run Test

three_cubes_3.obj
    Run Test

three_cubes.obj
    Run Test

three_cylinders.obj
    Run Test

tmp_wing.obj
    Run Test

two_cubes_1.obj
    Run Test

two_cylinders.obj
    Run Test

*** Keywords ***
Run Test
    [Arguments]    ${input_name}=${TEST NAME}    @{options}
    [Documentation]    Computes surface intersection, removes internal boundaries, checks that topology is a sphere
    ...    The name of the input file is taken from the test name.
    run command    intersect    remove_internal_shells=true    @{options}    ${DATADIR}${/}${input_name}
    run command    vorpastat    out.meshb    SPHERE

