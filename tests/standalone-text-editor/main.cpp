#include <geogram_gfx/gui/text_editor.h>
#include <iostream>

#ifndef GEOGRAM_USE_SYSTEM_TEXT_EDITOR
#error This check requires the installed standalone-editor mode
#endif
#ifndef __GEOGRAM__
#error The editor compatibility flag must propagate through FindGeogram
#endif

int main() {
    // Text loading must work before any graphical application is started.
    bool visible = true;
    GEO::TextEditor editor(&visible);
    editor.load_data("print('hello')");
    if (editor.text() != "print('hello')\n") { return 1; }
    editor.clear();
    if (!editor.text().empty()) { return 2; }
    std::cout << "Geogram standalone editor load/clear passed\n";
}
