#include <geogram_gfx/gui/text_editor.h>
#include <geogram_gfx/gui/simple_application.h>
#include <iostream>

#ifndef GEOGRAM_USE_SYSTEM_TEXT_EDITOR
#error This check requires the installed standalone-editor mode
#endif

namespace {
    class TestApplication : public GEO::SimpleApplication {
    public:
        TestApplication() : SimpleApplication("editor integration") {
            add_key_func("F5", [this]() { ++runs; });
            ImGui_load_fonts();
        }
        int runs = 0;
    };

    void frame(GEO::TextEditor& editor, bool focus_editor) {
        ImGui::NewFrame();
        ImGui::SetNextWindowSize(ImVec2(800, 600));
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        editor.draw();
        ImGui::SetNextWindowPos(ImVec2(820, 0));
        ImGui::Begin("Other window");
        if(!focus_editor) {
            ImGui::SetWindowFocus();
        }
        ImGui::End();
        ImGui::Render();
    }
}

int main() {
    // Loading must also work before ImGui or an application is initialized.
    bool visible = true;
    GEO::TextEditor editor(&visible);
    editor.load_data("print('hello')");
    if(editor.text() != "print('hello')\n") { return 1; }
    editor.clear();
    if(!editor.text().empty()) { return 2; }

    if(!IMGUI_CHECKVERSION()) {
        std::cerr << "ImGui headers and library have incompatible layouts\n";
        return 5;
    }
    ImGui::CreateContext();
    auto& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(1024, 768);
    io.DeltaTime = 1.0f / 60.0f;
    // Exercise CPU-side frames without a renderer or obsolete atlas APIs.
    io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
    {
        TestApplication application;
        editor.set_fixed_layout(false);
        frame(editor, true);
        ImGui::SetWindowFocus("Text Editor");
        frame(editor, true);
        // Click into the editable child, not just its parent window.
        io.AddMousePosEvent(120, 120);
        io.AddMouseButtonEvent(0, true);
        frame(editor, true);
        io.AddMouseButtonEvent(0, false);
        frame(editor, true);
        io.AddKeyEvent(ImGuiKey_F5, true);
        frame(editor, true);
        if(application.runs != 1) { return 3; }
        io.AddKeyEvent(ImGuiKey_F5, false);
        frame(editor, false);
        frame(editor, false);
        io.AddKeyEvent(ImGuiKey_F5, true);
        frame(editor, false);
        if(application.runs != 1) { return 4; }
        io.AddKeyEvent(ImGuiKey_F5, false);
        frame(editor, false);
    }
    ImGui::DestroyContext();
    std::cout << "Geogram standalone editor load/clear and focused F5 passed\n";
}
