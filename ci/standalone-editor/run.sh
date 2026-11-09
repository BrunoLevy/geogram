#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
prefix="$root/.ci-prefix"
records="$root/artifacts/standalone-editor"
mkdir -p "$records"

git clone --single-branch --branch sis_in \
    https://git.altlinux.org/people/majioa/packages/libimgui.git "$root/.ci-imgui"
{
    git -C "$root" log -1 --format='geogram: %H %s'
    git -C "$root/.ci-editor" log -1 --format='editor: %H %s'
    git -C "$root/.ci-imgui" log -1 --format='imgui: %H %s'
} | tee "$records/input-revisions.txt"
git submodule update --init --depth 1 \
    src/lib/geogram/third_party/OpenNL src/lib/geogram/third_party/amgcl \
    src/lib/geogram/third_party/libMeshb src/lib/geogram/third_party/rply \
    src/lib/geogram_gfx/third_party/imgui src/lib/geogram_gfx/third_party/imoguizmo

cmake -S "$root/ci/standalone-editor/imgui" -B "$root/.ci-imgui-build" \
    -DIMGUI_SOURCE_DIR="$root/.ci-imgui" -DCMAKE_INSTALL_PREFIX="$prefix" \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "$root/.ci-imgui-build" --parallel 2
cmake --install "$root/.ci-imgui-build"

cmake -S "$root/.ci-editor" -B "$root/.ci-editor-build" \
    -DCMAKE_PREFIX_PATH="$prefix" -DCMAKE_INSTALL_PREFIX="$prefix" \
    -DENABLE_GEOGRAM=ON -DENABLE_BALAZS=ON -DBUILD_TESTING=ON \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "$root/.ci-editor-build" --parallel 2
ctest --test-dir "$root/.ci-editor-build" --output-on-failure \
    --output-junit "$records/editor-tests.xml"
cmake --install "$root/.ci-editor-build"

cmake -S "$root" -B "$root/.ci-geogram-build" \
    -DCMAKE_PREFIX_PATH="$prefix" -DCMAKE_INSTALL_PREFIX="$prefix" \
    -DCMAKE_BUILD_TYPE=Release -DGEOGRAM_USE_SYSTEM_TEXT_EDITOR=ON \
    -DGEOGRAM_USE_SYSTEM_GLFW3=ON -DGEOGRAM_WITH_LUA=OFF \
    -DGEOGRAM_LIB_ONLY=ON -DVORPALINE_BUILD_DYNAMIC=ON
cmake --build "$root/.ci-geogram-build" --parallel 2
cmake --install "$root/.ci-geogram-build" --component devkit
cmake --install "$root/.ci-geogram-build" --component devkit-full

cmake -S "$root/tests/standalone-text-editor" -B "$root/.ci-consumer-build" \
    -DGEOGRAM_INSTALL_PREFIX="$prefix" -DCMAKE_PREFIX_PATH="$prefix" \
    -DCMAKE_MODULE_PATH="$prefix/lib/cmake/modules"
cmake --build "$root/.ci-consumer-build" --parallel 2
LD_LIBRARY_PATH="$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    ctest --test-dir "$root/.ci-consumer-build" --output-on-failure \
        --output-junit "$records/consumer-tests.xml"
git -C "$root/.ci-imgui" diff --exit-code
git -C "$root/.ci-imgui" diff --cached --exit-code
printf '%s\n' 'Source integration and installed consumer checks passed; ImGui unchanged.' \
    > "$records/integration.ok"
