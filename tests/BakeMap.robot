*** Settings ***
Test Setup        Prepare Test
Test Teardown     Cleanup Test
Force Tags        Remesh    smoke    daily
Library           OperatingSystem
Library           lib/VorpatestLibrary.py

*** Variables ***
${DATADIR}        %{VORPATEST_ROOT_DIR}${/}data${/}Chimeres

*** Test Cases ***
armabunny.obj (no remesh)
    [Tags]    daily_valgrind
    Run Test

armabunny.obj (remesh)
    [Tags]    daily_valgrind
    Run Test   nb_pts=300


*** Keywords ***
Run Test
    [Arguments]    ${input_name}=${TEST NAME}    @{options}
    [Documentation]    Runs remesh (optionnal), parameterization and baking
    ...    The name of the input file is taken from the test name.
    run bake_map    ${DATADIR}${/}${input_name}  @{options}
