*** Settings ***
Test Setup        Prepare Test
Test Teardown     Cleanup Test
Force Tags        smoke    daily    daily_valgrind
Library           OperatingSystem
Library           lib/VorpatestLibrary.py

*** Test Cases ***
with_string_attrib
    Run Test    nb_subdivisions=10   with_string_attribute=true

without_string_attrib
    Run Test    nb_subdivisions=10   with_string_attribute=false


*** Keywords ***
Run Test
    [Arguments]    @{options}
    [Documentation]    Runs test of attribute with life cycle
    run command    test_life_cycled_attribute    @{options}
