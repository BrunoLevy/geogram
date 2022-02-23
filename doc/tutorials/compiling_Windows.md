# Geogram compilation on Windows

Prerequisites
-------------
- git [here](https://git-scm.com/download/win) _use 64-bit Git for Windows Setup._
- Tortoise git [here](https://tortoisegit.org/) _optionnal, adds context menus to file browser, convenient_
- CMake [here](https://cmake.org/download/) _use cmake-xxxx-windows-x86_64.msi  IMPORTANT: select "Add CMake to system path for all users"__
- Visual C++ [here](https://visualstudio.microsoft.com/) _use community version_

Quick compilation guide
-----------------------
- click on `configure.bat`
- open `Build/Windows/Geogram.sln`
- set `Release` mode (default is `Debug`)
- build solution

This will compile the geogram library as well as demo programs. Demo
programs are generated in the `Build\Release\bin` subdirectory.

