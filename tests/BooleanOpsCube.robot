*** Settings ***
Test Setup        Prepare Test
Test Teardown     Cleanup Test
Force Tags        Intersect    smoke    daily    daily_valgrind
Library           OperatingSystem
Library           lib/VorpatestLibrary.py

*** Test Cases ***
Boolean ops between two cubes sharing a vertex
    run command    test_boolean_ops_cube
