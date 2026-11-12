*** Settings ***
Test Setup        Prepare Test
Test Teardown     Cleanup Test
Force Tags        Intersect    smoke    daily    daily_valgrind
Library           OperatingSystem
Library           lib/VorpatestLibrary.py

*** Test Cases ***
Translated small solid and cavity retain their winding
    run command    test_intersection_volume
