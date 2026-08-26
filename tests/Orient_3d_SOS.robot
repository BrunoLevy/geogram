*** Settings ***
Test Setup        Prepare Test
Test Teardown     Cleanup Test
Force Tags        smoke    daily
Library           OperatingSystem
Library           lib/VorpatestLibrary.py

*** Variables ***
${DATADIR}        %{VORPATEST_ROOT_DIR}${/}data

*** Test Cases ***
orient_3d_SOS.1
    [Tags]    daily_valgrind
    Run Test    shape=1   h=0.125   dh=0.25   lo=-0.5   hi=1.5

orient_3d_SOS.2
    [Tags]    daily_valgrind
    Run Test    shape=2   h=0.125   dh=0.25   lo=-0.5   hi=1.5

*** Keywords ***
Run Test
    [Arguments]    ${input_name}=${TEST NAME}    @{options}
    [Documentation]    Tests the Logger, with multiple threads
    run command    test_orient_3d_SOS    @{options}
