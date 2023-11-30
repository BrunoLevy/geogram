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
example001.csg
    Run Test

example002.csg
    Run Test

example003.csg
    Run Test

example004.csg
    Run Test

example005.csg
    Run Test

example006.csg
    Run Test

example007.csg
    Run Test

example008.csg
    Run Test

example009.csg
    Run Test

example010.csg
    Run Test

example011.csg
    Run Test

example012.csg
    Run Test

example013.csg
    Run Test

example014.csg
    Run Test

example015.csg
    Run Test

example016.csg
    Run Test

example017.csg
    Run Test

example018.csg
    Run Test

example019.csg
    Run Test

example020.csg
    Run Test

example021.csg
    Run Test

example022.csg
    Run Test

example023.csg
    Run Test

example024.csg
    Run Test

*** Keywords ***
Run Test
    [Arguments]    ${input_name}=${TEST NAME}    @{options}
    [Documentation]    Computes a CSG operation
    ...    The name of the input file is taken from the test name.
    run command    compute_CSG  @{options}    ${DATADIR}${/}${input_name}
