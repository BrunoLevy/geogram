*** Settings ***
Test Setup        Prepare Test
Test Teardown     Cleanup Test
Force Tags        Remesh    smoke_Standard    daily_valgrind_Standard
Library           OperatingSystem
Library           lib/VorpatestLibrary.py

*** Variables ***
${DATADIR}        %{VORPATEST_ROOT_DIR}${/}data${/}Small

*** Test Cases ***

three_holes.obj (profile=tet)
    Run Test    three_holes.obj     profile=tet


*** Keywords ***
Run Test
    [Arguments]    ${input_name}=${TEST NAME}    @{options}
    [Documentation]    Runs a vorpaline test
    ...    The name of the input file is taken from the test name.
    run vorpaline    ${DATADIR}${/}${input_name}    pts=5000    @{options}
    run vorpastat

Run Fail Test
    [Arguments]    @{args}
    [Documentation]    Runs a vorpaline test that is expected to fail
    Run Keyword And Expect Error    CalledProcessError: Command*returned non-zero exit status*    run vorpaline    @{args}
