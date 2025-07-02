*** Settings ***
Test Setup        Prepare Test
Test Teardown     Cleanup Test
Force Tags        CSG    smoke    daily    daily_valgrind
Library           OperatingSystem
Library           String
Library           lib/VorpatestLibrary.py

*** Variables **
${DATADIR}        %{VORPATEST_ROOT_DIR}${/}data${/}CSG

*** Test Cases ***
example001.scad
    Run Test

example002.scad
    Run Test

example003.scad
    Run Test

example004.scad
    Run Test

example005.scad
    Run Test

example006.scad
    Run Test

example007.scad
    Run Test

example008.scad
    Run Test

example009.scad
    Run Test

example010.scad
    Run Test

example011.scad
    Run Test

example012.scad
    Run Test

example013.scad
    Run Test

example014.scad
    Run Test

example015.scad
    Run Test

example016.scad
    Run Test

example017.scad
    Run Test

example018.scad
    Run Test

example019.scad
    Run Test

example020.scad
    Run Test

*** Keywords ***
Run Test
    [Arguments]    ${input_name}=${TEST NAME}    @{options}
    [Documentation]    Computes a CSG operation
    ...    The name of the input file is taken from the test name.
    run command    compute_CSG  @{options}  ignore_cache_time=true  ${DATADIR}${/}${input_name}
