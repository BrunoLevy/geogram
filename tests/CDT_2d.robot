*** Settings ***
Test Setup        Prepare Test
Test Teardown     Cleanup Test
Force Tags        smoke    daily
Library           OperatingSystem
Library           lib/VorpatestLibrary.py

*** Variables ***
${DATADIR}        %{VORPATEST_ROOT_DIR}${/}data${/}CDT2d

*** Test Cases ***
constraints_100_1.geogram 
    [Tags]    daily_valgrind
    Run Test   

constraints_100_3.geogram 
    [Tags]    daily_valgrind
    Run Test   

constraints_100_4.geogram 
    [Tags]    daily_valgrind
    Run Test   

constraints_100_5.geogram 
    [Tags]    daily_valgrind
    Run Test   

constraints_100_6.geogram 
    [Tags]    daily_valgrind
    Run Test   

constraints_100_7.geogram 
    [Tags]    daily_valgrind
    Run Test   

constraints_100_8.geogram 
    [Tags]    daily_valgrind
    Run Test   

constraints_100_9.geogram 
    [Tags]    daily_valgrind
    Run Test   

constraints_100_10.geogram 
    [Tags]    daily_valgrind
    Run Test   

constraints_100_11.geogram 
    [Tags]    daily_valgrind
    Run Test   

*** Keywords ***
Run Test
    [Arguments]    ${input_name}=${TEST NAME}    @{options}
    [Documentation]    Runs a vorpaline constrained Delaunay 2d test
    ...    The name of the input file is taken from the test name.
    ...    The name of the input file is taken from the test name.
    run command    test_CDT_2d    @{options}    ${DATADIR}${/}${input_name}
