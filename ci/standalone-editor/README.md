# Source integration CI

This Linux check uses unmodified `majioa/libimgui` sources from `sis_in`, the
standalone editor fork, and this Geogram branch. The separate ImGui test adapter
builds its core and demo source files (the Lua bindings expose demo functions)
and exports a CMake target into a temporary
prefix. It does not patch the maintainer repository or generate RPMs/version
provides; the editor's ALT gear-hsh job tests packaging separately.

The editor's regression tests run with both optional feature flags enabled.
Geogram builds with the standalone editor, system GLFW and builtin Lua enabled.
A separate project compiles against the installed Geogram devkit-full and
exercises the text-editor wrapper without starting a graphical application.
It also registers the actual Lua/ImGui bindings, checks retained constant names
and calls their style wrappers against the external ImGui context.

System-editor mode adapts the binding runtime and generated sources in the
build tree so builtin Lua can coexist with external ImGui. The source submodule
is unchanged; no extra fork or modifications to the ImGui dependency are needed.
JUnit reports, exact input commits and logs are saved as Actions artifacts.

Only trusted pushes/manual runs trigger this workflow. Once this CI commit is
included in `feature/external-text-editor`, source changes there run it directly.
