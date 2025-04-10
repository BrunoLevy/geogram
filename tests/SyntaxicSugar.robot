*** Settings ***
Test Setup        Prepare Test
Test Teardown     Cleanup Test
Force Tags        smoke    daily    daily_valgrind
Library           OperatingSystem
Library           lib/VorpatestLibrary.py

*** Test Cases ***
triangles
    Run Test    nb_subdivisions=11   quads=false

quads
    Run Test    nb_subdivisions=11   quads=true


*** Keywords ***
Run Test
    [Arguments]    @{options}
    [Documentation]    Runs test of mesh traversal with syntaxic sugar
    run command    test_mesh_syntaxic_sugar    @{options}
