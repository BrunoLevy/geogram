@ECHO OFF
SETLOCAL ENABLEDELAYEDEXPANSION

REM ---------------------------------------------------------------- 
REM Check for options: [ --build_name_suffix suffix ]
REM ----------------------------------------------------------------
set buildNameSuffix=""
if "%1" ==  "--build_name_suffix" (
   set buildNameSuffix=%2
   SHIFT & SHIFT
)

ECHO Configuring Geogram build

REM ----------------------------------------------------------------
REM Create build directory and run cmake
REM ----------------------------------------------------------------

ECHO Starting CMake...
ECHO  (NOTE: it may complain about missing VULKAN, you can safely ignore)

if not exist "build\Windows%buildNameSuffix%" (
   mkdir "build\Windows%buildNameSuffix%"
)

cd build\Windows%buildNameSuffix%

"%ProgramFiles%\cmake\bin\cmake.exe" ..\.. ^
 -DVORPALINE_PLATFORM:STRING=Win-vs-generic 


REM -----------------------------------------------------------------
REM Wait for user keypress to keep DOS box open
REM -----------------------------------------------------------------

ECHO ----------------------------------------------------------------

if exist "Geogram.sln" (
   ECHO Geogram build is configured
   ECHO Visual Studio solution is in GraphiteThree\build\Windows%buildNameSuffix%\Geogram.sln
) else (
   ECHO ERROR: could not generate Visual Studio solution
)

ECHO ----------------------------------------------------------------

REM set /p DUMMY=Hit ENTER to continue...

