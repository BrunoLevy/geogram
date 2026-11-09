# Source integration CI

This Linux check uses unmodified `majioa/libimgui` sources from `sis_in`, the
standalone editor fork, and this Geogram branch. The separate ImGui test adapter
builds only its four core source files and exports a CMake target into a temporary
prefix. It does not patch the maintainer repository or generate RPMs/version
provides; the editor's ALT gear-hsh job tests packaging separately.

The editor's regression tests run with both optional feature flags enabled.
Geogram builds with the standalone editor and system GLFW, with optional Lua
disabled. A separate project compiles against the installed Geogram devkit-full
and exercises the text-editor wrapper without starting a graphical application.
JUnit reports, exact input commits and logs are saved as Actions artifacts.

Only trusted pushes/manual runs trigger this workflow. Once this CI commit is
included in `feature/external-text-editor`, source changes there run it directly.
