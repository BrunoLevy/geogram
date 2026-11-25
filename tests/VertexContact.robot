*** Settings ***
Test Setup        Prepare Test
Test Teardown     Cleanup Test
Force Tags        Intersect    smoke    daily    daily_valgrind
Library           OperatingSystem
Library           lib/VorpatestLibrary.py

*** Test Cases ***
Solids touching at vertices are classified by geometry
    run command    test_vertex_contact
