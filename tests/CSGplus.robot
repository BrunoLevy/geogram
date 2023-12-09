*** Settings ***
Test Setup        Prepare Test
Test Teardown     Cleanup Test
Force Tags        CSGplus
Library           OperatingSystem
Library           String
Library           lib/VorpatestLibrary.py

*** Variables **
${DATADIR}        %{VORPATEST_ROOT_DIR}${/}data${/}CSG

*** Test Cases ***
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
