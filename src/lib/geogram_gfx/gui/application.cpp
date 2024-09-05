/*
 *  Copyright (c) 2000-2023 Inria
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *  * Neither the name of the ALICE Project-Team nor the names of its
 *  contributors may be used to endorse or promote products derived from this
 *  software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Contact: Bruno Levy
 *
 *     https://www.inria.fr/fr/bruno-levy
 *
 *     Inria,
 *     Domaine de Voluceau,
 *     78150 Le Chesnay - Rocquencourt
 *     FRANCE
 *
 */

#include <geogram_gfx/gui/application.h>
#include <geogram_gfx/gui/user_callback_android.h>

#include <geogram_gfx/imgui_ext/imgui_ext.h>
#include <geogram_gfx/third_party/imgui/backends/imgui_impl_opengl3.h>
#include <geogram_gfx/imgui_ext/icon_font.h>

#include <geogram_gfx/lua/lua_glup.h>
#include <geogram_gfx/lua/lua_imgui.h>

#include <geogram_gfx/third_party/imgui_fonts/cousine_regular.h>
#include <geogram_gfx/third_party/imgui_fonts/roboto_medium.h>
#include <geogram_gfx/third_party/imgui_fonts/fa_solid.h>

#include <geogram/image/image.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/file_system.h>


#if defined(GEO_GLFW)
#  include <geogram_gfx/third_party/imgui/backends/imgui_impl_glfw.h>
// Too many documentation warnings in glfw
// (glfw uses tags that clang does not understand).
#  ifdef __clang__
#    pragma GCC diagnostic ignored "-Wdocumentation"
#  endif

#  if defined(GEO_USE_SYSTEM_GLFW3) || defined(GEO_OS_EMSCRIPTEN)
#    include <GLFW/glfw3.h>
#  else
#    include <third_party/glfw/include/GLFW/glfw3.h>
#  endif

#ifdef GEO_OS_EMSCRIPTEN
#  include <emscripten.h>
#endif

#elif defined(GEO_OS_ANDROID)

#  include <geogram_gfx/imgui_ext/imgui_impl_android_ext.h>
#  include <geogram/basic/android_utils.h>

#  include <EGL/egl.h>
#  include <EGL/eglext.h>
#  include <GLES/gl.h>
#  include <android_native_app_glue.h>

#endif

namespace ImGui {
    void UpdateHoveredWindowAndCaptureFlags();
}

namespace GEO {

    /**
     * \brief Computes the pixel ratio for hidpi devices.
     * \details Uses the current GLFW window.
     */
    double compute_pixel_ratio();

    /**
     * \brief Computes the scaling factor for hidpi devices.
     * \details Uses the current GLFW window.
     */
    double compute_hidpi_scaling();


    /**
     * \brief If nothing happens during 100 frames, then
     * we (micro)-sleep instead of redrawing the
     * window.
     */
    const index_t NB_FRAMES_UPDATE_INIT = 100;

#if defined(GEO_GLFW)
    class ApplicationData {
    public:
        ApplicationData() {
            window_ = nullptr;
            GLFW_callbacks_initialized_ = false;
        }
        GLFWwindow* window_;
        bool GLFW_callbacks_initialized_;
    };
#elif defined(GEO_OS_ANDROID)
    class ApplicationData {
    public:
        ApplicationData() {
            app = nullptr;
            display = EGL_NO_DISPLAY;
            surface = EGL_NO_SURFACE;
            context = EGL_NO_CONTEXT;
            GLES_version = 3;
            has_focus = false;
            has_window = false;
            is_visible = false;
            GL_initialized = false;
        }
        bool should_draw() const {
            return has_focus && has_window && is_visible;
        }
        android_app* app;
        EGLDisplay display;
        EGLSurface surface;
        EGLConfig  config;
        EGLContext context;
        EGLint     GLES_version;
        bool       has_focus;
        bool       has_window;
        bool       is_visible;
        LoggerClient_var logger_client;
        bool       GL_initialized;
    };
#else
#  error "No windowing system"
#endif

}

namespace GEO {

    Application* Application::instance_ = nullptr;

    Application::Application(const std::string& name) {
        geo_assert(instance_ == nullptr);
        GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
        instance_ = this;
        name_ = name;
        data_ = new ApplicationData();
        ImGui_restart_ = false;
        ImGui_reload_font_ = false;
        ImGui_initialized_ = false;
        ImGui_firsttime_init_ = false;
        width_ = 800;
        height_ = 800;
        frame_buffer_width_ = 800;
        frame_buffer_height_ = 800;
        in_main_loop_ = false;
        accept_drops_ = true;
        scaling_ = 1.0;
        font_size_ = 18;
        nb_update_locks_ = 0;
        nb_frames_update_ = NB_FRAMES_UPDATE_INIT;
        hidpi_scaling_ = 1.0;
        pixel_ratio_ = 1.0;
        currently_drawing_gui_ = false;
        animate_ = false;
        menubar_visible_ = true;
        phone_screen_ = false;
    }

    Application::~Application() {
        delete_window();
        geo_assert(instance_ == this);
        delete data_;
        data_ = nullptr;
        instance_ = nullptr;
    }

    void Application::start(int argc, char** argv) {
        if(argc != 0 && argv != nullptr) {
            geogram_initialize(argc, argv);
        }
        if(CmdLine::arg_is_declared("gui:font_size")) {
            set_font_size(CmdLine::get_arg_uint("gui:font_size"));
        }
        create_window();
        main_loop();
    }

    void Application::stop() {
        in_main_loop_ = false;
    }

    std::string Application::get_styles() {
        return "Light;Dark";
    }

    void Application::set_style(const std::string& style_name) {
        style_ = style_name;

        if(style_name == "Light") {
            ImGui::StyleColorsLight();
            ImGuiStyle& style = ImGui::GetStyle();
            style.WindowRounding = 10.0f;
            style.FrameRounding = 5.0f;
            style.GrabRounding = 10.0f;
            style.WindowBorderSize = 1.5f;
            style.FrameBorderSize = 1.0f;
            style.PopupBorderSize = 1.0f;
            ImVec4* colors = style.Colors;
            colors[ImGuiCol_Text]         = ImVec4(0.0f,  0.0f,  0.25f, 1.00f);
            colors[ImGuiCol_TextDisabled] = ImVec4(0.25f, 0.25f, 0.75f, 1.00f);
            colors[ImGuiCol_Separator]    = ImVec4(0.75f, 0.75f, 0.75f, 1.00f);
        } else if(style_name == "Dark") {
            ImGuiStyle& style = ImGui::GetStyle();
            style.WindowRounding = 10.0f;
            style.FrameRounding = 5.0f;
            style.GrabRounding = 10.0f;
            style.WindowBorderSize = 1.5f;
            style.FrameBorderSize = 0.0f;
            style.PopupBorderSize = 1.0f;
            ImGui::StyleColorsDark();
            // Tron vibes...
            ImVec4* colors = style.Colors;
            colors[ImGuiCol_Text]         = ImVec4(0.50f, 1.00f, 1.00f, 1.00f);
            colors[ImGuiCol_TextDisabled] = ImVec4(0.30f, 0.50f, 0.50f, 1.00f);
        } else {
            set_style("Light");
            Logger::err("Skin") << style_name << ": no such style"
                                << std::endl;
        }

        ImGuiStyle& style = ImGui::GetStyle();
        if(CmdLine::get_arg_bool("gfx:transparent")) {
            // Make ImGui windows opaque if background is
            // transparent (else it becomes difficult to
            // distinguish anything...)
            style.Alpha = 1.0f;
        } else {
            style.Alpha = 0.90f;
        }

        if(phone_screen_) {
            style.ScrollbarSize = 10.0f * float(scaling_);
            style.GrabMinSize   = 15.0f * float(scaling_);
        }
    }

    void Application::set_font_size(index_t value) {
        font_size_ = index_t(value);
        scaling_ = double(font_size_)/16.0;
        if(phone_screen_) {
            scaling_ *= double(std::max(get_width(), get_height()))/600.0;
        }
        if(CmdLine::arg_is_declared("gui:font_size")) {
            CmdLine::set_arg("gui:font_size", String::to_string(value));
        }
        if(ImGui_initialized_) {
            ImGui_reload_font_ = true;
        }
    }

    double Application::scaling() const {
        return scaling_ * hidpi_scaling_ / pixel_ratio_;
    }

    void Application::resize(
        index_t w, index_t h, index_t fb_w, index_t fb_h
    ) {
        width_ = w;
        height_ = h;
        frame_buffer_width_ = fb_w;
        frame_buffer_height_ = fb_h;
        scaling_ = double(font_size_)/16.0;
        if(phone_screen_) {
            scaling_ *= double(std::max(get_width(), get_height()))/600.0;
        }
        update();
    }

    void Application::draw_gui() {
    }

    void Application::draw_graphics() {
    }

    void Application::mouse_button_callback(
        int button, int action, int mods, int source
    ) {
        geo_argused(button);
        geo_argused(action);
        geo_argused(mods);
        geo_argused(source);
    }

    void Application::scroll_callback(double xoffset, double yoffset) {
        geo_argused(xoffset);
        geo_argused(yoffset);
    }

    void Application::cursor_pos_callback(double x, double y, int source) {
        geo_argused(x);
        geo_argused(y);
        geo_argused(source);
    }

    void Application::drop_callback(int nb, const char** f) {
        geo_argused(nb);
        geo_argused(f);
    }

    void Application::char_callback(unsigned int c) {
        geo_argused(c);
    }

    void Application::key_callback(
        int key, int scancode, int action, int mods
    ) {
        geo_argused(key);
        geo_argused(scancode);
        geo_argused(action);
        geo_argused(mods);
    }

    void Application::draw_dock_space() {
        ImGuiIO& io = ImGui::GetIO();
        // Create window and dockspace for docking.
        if((io.ConfigFlags & ImGuiConfigFlags_DockingEnable) != 0) {
            ImGuiViewport* viewport = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(viewport->Pos);
            ImGui::SetNextWindowSize(viewport->Size);
            ImGui::SetNextWindowViewport(viewport->ID);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
            ImGui::PushStyleVar(
                ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f)
            );
            static bool open = true;
            ImGui::Begin(
                "DockSpace", &open,
                ImGuiWindowFlags_NoDocking |
                ImGuiWindowFlags_NoTitleBar |
                ImGuiWindowFlags_NoCollapse |
                ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoBringToFrontOnFocus |
                ImGuiWindowFlags_NoNavFocus |
                ImGuiWindowFlags_NoBackground |
                (menubar_visible_ ? ImGuiWindowFlags_MenuBar : 0)
            );
            ImGui::PopStyleVar(3);
            ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
            ImGui::DockSpace(
                dockspace_id, ImVec2(0.0f, 0.0f),
                ImGuiDockNodeFlags_None |
                ImGuiDockNodeFlags_PassthruCentralNode |
                ImGuiDockNodeFlags_AutoHideTabBar
            );
            ImGui::End();
        }
    }

    void Application::draw() {
        if(!ImGui_initialized_) {
            return;
        }
        update();
        if(nb_update_locks_ == 0 && !Process::is_running_threads()) {
            one_frame();
        }
    }

    void Application::GL_initialize() {
        GEO::Graphics::initialize();
        geo_assert(glupCurrentContext() == nullptr);
        glupMakeCurrent(glupCreateContext());
        if(glupCurrentContext() == nullptr) {
            Logger::err("Skin") << "Could not create GLUP context"
                                << std::endl;
            exit(-1);
        }
    }

    void Application::GL_terminate() {
        glupDeleteContext(glupCurrentContext());
        glupMakeCurrent(nullptr);
        GEO::Graphics::terminate();
#ifdef GEO_OS_ANDROID
        data_->GL_initialized = false;
#endif
    }

    void Application::ImGui_initialize() {
        geo_assert(!ImGui_initialized_ );
        ImGui::CreateContext();
        {
            std::string state = CmdLine::get_arg("gui:state");
            for(size_t i=0; i<state.length(); ++i) {
                if(state[i] == '\t') {
                    state[i] = '\n';
                }
            }
            ImGui::LoadIniSettingsFromMemory(state.c_str());
        }
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

        // Viewports allow to drag ImGui windows outside the app's window,
        // but it is still a bit unstable, so deactivated it for now.
        if(
            CmdLine::arg_is_declared("gui:viewports") &&
            CmdLine::get_arg_bool("gui:viewports")
        ) {
            io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
        }

        // Note: NavKeyboard sets WantsCaptureKeyboard all the time and
        // thus prevents from nanosleeping !
        if(
            CmdLine::arg_is_declared("gui:keyboard_nav") &&
            CmdLine::get_arg_bool("gui:keyboard_nav")
        ) {
            io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        }

#if defined(GEO_GLFW)

        // Second argument to true = install callbacks
        // (note that this function can be called multiple times,
        //  e.g. when ImGui is restarted after re-loading a saved
        //  window state, this mechanism prevents the callbacks to
        //  be installed multiple times, which would be an error since
        //  they are chained).
        ImGui_ImplGlfw_InitForOpenGL(
            data_->window_, !data_->GLFW_callbacks_initialized_
        );
#elif defined(GEO_OS_ANDROID)
        ImGui_ImplAndroidExt_Init(data_->app);
#endif

#if defined(GEO_OS_APPLE)
        ImGui_ImplOpenGL3_Init("#version 330");
#else
        ImGui_ImplOpenGL3_Init("#version 100");
#endif
        callbacks_initialize();

        if(CmdLine::get_arg_bool("gui:phone_screen")) {
            set_style("Dark");
        } else if(style_ != "") {
            set_style(style_);
        } else if(Environment::instance()->has_value("gui:style")) {
            std::string style = Environment::instance()->get_value("gui:style");
            set_style(style);
        } else {
            set_style("Light");
        }

        ImGui_load_fonts();
        ImGui_initialized_ = true;
        ImGui_firsttime_init_ = true;
    }

    void Application::ImGui_load_fonts() {
        ImGuiIO& io = ImGui::GetIO();
        io.IniFilename = nullptr;

        float s = 1.0f;
        if(phone_screen_) {
            s = float(std::max(get_width(), get_height())) / 600.0f;
        }

        float font_size = s * float(double(font_size_) * hidpi_scaling_);

        // Default font
        io.FontDefault = io.Fonts->AddFontFromMemoryCompressedTTF(
            roboto_medium_compressed_data,
            roboto_medium_compressed_size, font_size
        );

        // Add icons to default font.
        {
#define ICON_MIN_FA 0xf000
#define ICON_MAX_FA 0xf63c

            ImFontConfig config;
            config.MergeMode = true;

            // Make the icon monospaced
            config.GlyphMinAdvanceX = 1.5f*font_size;
            config.GlyphOffset.y += 2.0f;

            static const ImWchar icon_ranges[] = {
                ICON_MIN_FA, ICON_MAX_FA, 0
            };

            io.Fonts->AddFontFromMemoryCompressedTTF(
                fa_solid_compressed_data,
                fa_solid_compressed_size, font_size,
                &config, icon_ranges
            );

            init_icon_table();
        }

        // Fixed font for console and editor
        io.Fonts->AddFontFromMemoryCompressedTTF(
            cousine_regular_compressed_data,
            cousine_regular_compressed_size, font_size
        );

        // Larger font
        io.Fonts->AddFontFromMemoryCompressedTTF(
            roboto_medium_compressed_data,
            roboto_medium_compressed_size, font_size*1.5f
        );

        if(phone_screen_) {
            // Smaller fixed font for console
            io.Fonts->AddFontFromMemoryCompressedTTF(
                cousine_regular_compressed_data,
                cousine_regular_compressed_size, font_size*0.5f
            );
        }

        io.FontGlobalScale = float(1.0 / pixel_ratio_);
    }

    void Application::ImGui_terminate() {
        geo_assert(ImGui_initialized_);
        ImGui_ImplOpenGL3_Shutdown();
#if defined(GEO_GLFW)
        ImGui_ImplGlfw_Shutdown();
        data_->GLFW_callbacks_initialized_ = false;
#elif defined(GEO_OS_ANDROID)
        ImGui_ImplAndroidExt_Shutdown();
#endif
        ImGui::DestroyContext();
        ImGui_initialized_ = false;
    }

    void Application::ImGui_new_frame() {
        ImGui_ImplOpenGL3_NewFrame();
#if defined(GEO_GLFW)
        ImGui_ImplGlfw_NewFrame();
#elif defined(GEO_OS_ANDROID)
        ImGui_ImplAndroidExt_NewFrame();
#endif
        ImGui::NewFrame();
    }

    void Application::geogram_initialize(int argc, char** argv) {
        GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
        CmdLine::import_arg_group("standard");
        CmdLine::import_arg_group("algo");
        CmdLine::import_arg_group("gfx");
        CmdLine::import_arg_group("gui");
        CmdLine::parse(argc, argv, filenames_);
        phone_screen_ = CmdLine::get_arg_bool("gui:phone_screen");
#ifndef GEO_OS_ANDROID
        if(phone_screen_ && CmdLine::get_arg("gfx:geometry") == "1024x1024") {
            CmdLine::set_arg("gfx:geometry", "768x1024");
        }
#endif
    }

    bool Application::needs_to_redraw() const {
        return
            animate_ ||
            //ImGui::GetIO().WantCaptureMouse ||     // HERE: do not think
            //ImGui::GetIO().WantCaptureKeyboard ||  // it is needed anymore !
            (nb_frames_update_ > 0);
    }

    void Application::update() {
        // We redraw several frames, in order to make
        // sure all events are properly processed.
        nb_frames_update_ = NB_FRAMES_UPDATE_INIT;
    }

    void Application::set_gui_state(std::string x) {
        CmdLine::set_arg("gui:state", x);
        if(!ImGui_initialized_) {
            return;
        }
        ImGui_restart_ = true;
    }

    std::string Application::get_gui_state() const {
        std::string state;
        if(ImGui_initialized_) {
            state = std::string(ImGui::SaveIniSettingsToMemory());
            for(size_t i=0; i<state.length(); ++i) {
                if(state[i] == '\n') {
                    state[i] = '\t';
                }
            }
        }
        return state;
    }

    /**************************** GLFW-specific code *********************/
#if defined(GEO_GLFW)


    void Application::pre_draw() {
    }

    void Application::post_draw() {
    }

    void Application::create_window() {
        if(!glfwInit()) {
            Logger::err("Skin")
                <<  "Could not initialize GLFW" << std::endl;
            exit(-1);
        }

        if(CmdLine::get_arg("gfx:GL_profile") == "core") {
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
            glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
            glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        }

        if(CmdLine::get_arg_bool("gfx:GL_debug")) {
            glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
        }

        const char* title = name_.c_str();

        {
            std::string geometry = CmdLine::get_arg("gfx:geometry");
            std::vector<std::string> words;
            String::split_string(geometry, 'x', words);
            if(words.size() != 2) {
                Logger::err("Skin")
                    << "Invalid gfx:geometry:" << geometry << std::endl;
                exit(-1);
            }
            if(
                !String::string_to_unsigned_integer(words[0].c_str(), width_) ||
                !String::string_to_unsigned_integer(words[1].c_str(), height_)
            ) {
                Logger::err("Skin")
                    << "Invalid gfx:geometry:" << geometry << std::endl;
                exit(-1);
            }
        }

        if(CmdLine::get_arg_bool("gfx:transparent")) {
#ifdef GLFW_TRANSPARENT_FRAMEBUFFER
            glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER, GLFW_TRUE);
#else
            Logger::warn("Skin")
                << "Transparent not supported by this version of GLFW"
                << std::endl;
#endif
        }

        if(CmdLine::get_arg_bool("gfx:full_screen")) {
            GLFWmonitor* monitor = glfwGetPrimaryMonitor();
            const GLFWvidmode* vidmode = glfwGetVideoMode(monitor);
            width_ = index_t(vidmode->width);
            height_ = index_t(vidmode->height);

            bool no_decoration = CmdLine::get_arg_bool("gfx:no_decoration");

            if(no_decoration) {
                glfwWindowHint(GLFW_FOCUSED,GL_TRUE);
                glfwWindowHint(GLFW_DECORATED,GL_FALSE);
                glfwWindowHint(GLFW_RESIZABLE,GL_FALSE);
                glfwWindowHint(GLFW_AUTO_ICONIFY,GL_FALSE);
                glfwWindowHint(GLFW_FLOATING,GL_FALSE);
                glfwWindowHint(GLFW_MAXIMIZED,GL_TRUE);
            }

            data_->window_ = glfwCreateWindow(
                int(width_), int(height_), title,
                no_decoration ? glfwGetPrimaryMonitor() : nullptr,
                nullptr
            );

        } else {
            data_->window_ = glfwCreateWindow(
                int(width_), int(height_), title, nullptr, nullptr
            );
        }

        if(data_->window_ == nullptr) {
            Logger::err("Skin")
                << "Could not create GLFW window" << std::endl;
            exit(-1);
        }

        glfwSetWindowUserPointer(data_->window_, this);

        glfwMakeContextCurrent(data_->window_);
        glfwSwapInterval(1);

        hidpi_scaling_ = compute_hidpi_scaling();
        pixel_ratio_ = compute_pixel_ratio();

        Logger::out("Skin")
            << "hidpi_scaling=" << hidpi_scaling_ << std::endl;
        Logger::out("Skin")
            << "pixel_ratio=" << pixel_ratio_ << std::endl;
    }

    void Application::delete_window() {
        glfwDestroyWindow(data_->window_);
        data_->window_ = nullptr;
        glfwTerminate();
        in_main_loop_ = false;
    }


    void Application::one_frame() {
        // Avoid nested ImGui calls
        // (due to calling draw())
        if(currently_drawing_gui_) {
            return;
        }

        // Can happen when ImGui Graphite application
        // triggers update too soon.
        if(data_->window_ == nullptr) {
            return;
        }

        if(glfwWindowShouldClose(data_->window_) || !in_main_loop_) {
            return;
        }


        {
            int cur_width, cur_height;
            int cur_fb_width,  cur_fb_height;

            glfwGetWindowSize(data_->window_, &cur_width, &cur_height);
            glfwGetFramebufferSize(
                data_->window_, &cur_fb_width, &cur_fb_height
            );

            if(
                int(width_) != cur_width ||
                int(height_) != cur_height ||
                int(frame_buffer_width_) != cur_fb_width ||
                int(frame_buffer_height_) != cur_fb_height
            ) {
                resize(
                    index_t(cur_width), index_t(cur_height),
                    index_t(cur_fb_width), index_t(cur_fb_height)
                );
            }
        }

        {
            // Detect if hidpi scaling changed. This can happen when
            // dragging the window from the laptop screen to an external
            // monitor.
            if(
                glfwGetCurrentContext() != nullptr &&
                compute_hidpi_scaling() != hidpi_scaling_
            ) {
                hidpi_scaling_ = compute_hidpi_scaling();
                pixel_ratio_ = compute_pixel_ratio();
                set_font_size(font_size_); // This reloads the font.
            }
        }


        glfwPollEvents();

        if(needs_to_redraw()) {
            pre_draw();
            currently_drawing_gui_ = true;
            ImGui_new_frame();
            draw_graphics();
            draw_gui();
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glUseProgram(0); // RenderDrawData() leaves a bound program

#ifndef GEO_OS_EMSCRIPTEN
            // Update and Render additional Platform Windows
            // (see ImGui demo app).
            if(ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
                GLFWwindow* backup_current_context = glfwGetCurrentContext();
                ImGui::UpdatePlatformWindows();
                ImGui::RenderPlatformWindowsDefault();
                glfwMakeContextCurrent(backup_current_context);
            }
#endif

            currently_drawing_gui_ = false;

#ifdef GEO_OS_EMSCRIPTEN
            // Set alpha channel to 1 else the image is composited
            // with the background
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_TRUE);
            glClear(GL_COLOR_BUFFER_BIT);
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
#endif

            glfwSwapBuffers(data_->window_);
            post_draw();
            if(
                nb_frames_update_ > 0 && !animate_ &&
                !(ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
            ) {
                --nb_frames_update_;
            }
#ifndef GEO_OS_EMSCRIPTEN
            // Be nice, sleep for 0.01 seconds, always
            Process::sleep(10000);
#endif

        } else {
            // If there was no activity in the last few frames,
            // sleep for 0.1 seconds, to let the processor cool-down
            // instead of actively waiting (be a good citizen for the
            // other processes.
            Process::sleep(100000);
        }

        // ImGui needs to be restarted whenever docking state is reloaded.
        if(ImGui_restart_ || ImGui_reload_font_) {
            ImGui_restart_ = false;
            ImGui_terminate();
            if(CmdLine::arg_is_declared("gui:font_size")) {
                set_font_size(CmdLine::get_arg_uint("gui:font_size"));
            }
            ImGui_reload_font_ = false;
            ImGui_initialize();
        }
    }

#ifdef GEO_OS_EMSCRIPTEN

    void emscripten_load_latest_file();

    /**
     * \brief This function is called by the HTML shell each
     *  time a file is loaded.
     */
    void emscripten_load_latest_file() {
        if(Application::instance() == nullptr) {
            return;
        }
        std::vector<std::string> all_files;
        GEO::FileSystem::get_directory_entries("/",all_files);
        std::string filename = "";
        Numeric::uint64 timestamp = 0;
        for(auto f: all_files) {
            if(GEO::FileSystem::is_file(f)) {
                Numeric::uint64 f_timestamp =
                    GEO::FileSystem::get_time_stamp(f);
                if(f_timestamp > timestamp) {
                    timestamp = f_timestamp;
                    filename = f;
                }
            }
        }
        if(filename != "") {
            const char* filename_str = filename.c_str();
            Application::instance()->drop_callback(1, &filename_str);
        }
    }


    /**
     * \brief The job to be done for each frame when running
     *  in Emscripten.
     * \details The browser keeps control of the main loop
     *  in Emscripten. This function is a callback passed to
     *  the Emscripten runtime.
     */
    void emscripten_one_frame() {
        if(Application::instance() == nullptr) {
            return;
        }
        Application::instance()->one_frame();
    }
#endif

    void Application::main_loop() {
        in_main_loop_ = true;
#ifdef GEO_OS_EMSCRIPTEN
        FileSystem::set_file_system_changed_callback(
            emscripten_load_latest_file
        );
        GL_initialize();
        ImGui_initialize();
        emscripten_load_latest_file();
        emscripten_set_main_loop(emscripten_one_frame, 0, 1);
#else
        bool initialized = false;
        while (!glfwWindowShouldClose(data_->window_) && in_main_loop_) {
            if(!initialized) {
                GL_initialize();
                ImGui_initialize();
                initialized = true;
            }
            one_frame();
        }
        if(initialized) {
            ImGui_terminate();
            GL_terminate();
        }
#endif
    }

    namespace {
        void GLFW_callback_mouse_button(
            GLFWwindow* w, int button, int action, int mods
        ) {
            Application* app = static_cast<Application*>(
                glfwGetWindowUserPointer(w)
            );
            app->update();
            if(!ImGui::GetIO().WantCaptureMouse) {
                app->mouse_button_callback(button,action,mods);
            }
            // Note: when a menu is open and you click elsewhere, the
            // WantCaptureMouse flag is still set, and the framework
            // misses the "mouse button up" event. If a translation is
            // active, it remains active later ("sticky translation" bug).
            // The following code always generates a "mouse button up" event
            // to solve this problem.
            if(ImGui::GetIO().WantCaptureMouse && action==EVENT_ACTION_UP) {
                ImVec2 mouse_pos = ImGui::GetIO().MousePos;
                app->cursor_pos_callback(
                    double(mouse_pos.x), double(mouse_pos.y)
                );
                app->mouse_button_callback(button,action,mods);
            }
            ImGui_ImplGlfw_MouseButtonCallback(w, button, action, mods);
        }

        void GLFW_callback_cursor_pos(
            GLFWwindow* w, double xf, double yf
        ) {
            Application* app = static_cast<Application*>(
                glfwGetWindowUserPointer(w)
            );
            app->update();
            ImGui_ImplGlfw_CursorPosCallback(w,xf,yf);
            if(!ImGui::GetIO().WantCaptureMouse) {
                app->cursor_pos_callback(xf, yf);
            }
        }

        void GLFW_callback_scroll(
            GLFWwindow* w, double xoffset, double yoffset
        ) {
            Application* app = static_cast<Application*>(
                glfwGetWindowUserPointer(w)
            );
            app->update();
            if(!ImGui::GetIO().WantCaptureMouse) {
#ifdef GEO_OS_EMSCRIPTEN
                // Emscripten sometimes returns fantaisist
                // values for yoffset (100, -100).
                if(yoffset > 0) { yoffset =  1; }
                if(yoffset < 0) { yoffset = -1; }
#endif
#if defined(GEO_OS_EMSCRIPTEN) || defined(GEO_OS_APPLE)
                app->scroll_callback(xoffset,-yoffset);
#else
                app->scroll_callback(xoffset, yoffset);
#endif
            }
            ImGui_ImplGlfw_ScrollCallback(w,xoffset,yoffset);
        }

        void GLFW_callback_drop(
            GLFWwindow* w, int nb, const char** p
        ) {
            Application* app = static_cast<Application*>(
                glfwGetWindowUserPointer(w)
            );
            app->update();
            app->drop_callback(nb, p);
        }

        void GLFW_callback_char(GLFWwindow* w, unsigned int c) {
            Application* app = static_cast<Application*>(
                glfwGetWindowUserPointer(w)
            );
            app->update();
            ImGui_ImplGlfw_CharCallback(w, c);
            if(!ImGui::GetIO().WantCaptureKeyboard) {
                app->char_callback(c);
            }
        }

        void GLFW_callback_key(
            GLFWwindow* w, int key, int scancode, int action, int mods
        ) {
            Application* app = static_cast<Application*>(
                glfwGetWindowUserPointer(w)
            );
            app->update();
            ImGui_ImplGlfw_KeyCallback(w, key, scancode, action, mods);
            if(!ImGui::GetIO().WantCaptureKeyboard) {
                app->key_callback(key,scancode,action,mods);
            }
        }

        void GLFW_callback_refresh(GLFWwindow* w) {
            Application* app = static_cast<Application*>(
                glfwGetWindowUserPointer(w)
            );
            app->update();
        }
    }

    void Application::callbacks_initialize() {
        if(!data_->GLFW_callbacks_initialized_) {
            GEO::Logger::out("ImGui") << "Viewer GUI init (GL3)"
                                      << std::endl;
        }

        if (!data_->GLFW_callbacks_initialized_) {
            glfwSetMouseButtonCallback(
                data_->window_, GLFW_callback_mouse_button
            );
            glfwSetCursorPosCallback(
                data_->window_, GLFW_callback_cursor_pos
            );
            glfwSetScrollCallback(
                data_->window_, GLFW_callback_scroll
            );
            glfwSetCharCallback(
                data_->window_, GLFW_callback_char
            );
            glfwSetKeyCallback(
                data_->window_, GLFW_callback_key
            );
            glfwSetDropCallback(
                data_->window_, GLFW_callback_drop
            );
            glfwSetWindowRefreshCallback(
                data_->window_, GLFW_callback_refresh
            );
            data_->GLFW_callbacks_initialized_ = true;
        }
    }

    void Application::set_window_icon(Image* icon_image) {
        GLFWimage glfw_image;
        glfw_image.width = int(icon_image->width());
        glfw_image.height = int(icon_image->height());
        glfw_image.pixels = icon_image->base_mem();
        glfwSetWindowIcon(data_->window_, 1, &glfw_image);
    }

    void Application::set_full_screen_mode(
        index_t w, index_t h, index_t Hz, index_t monitor
    ) {
        if(data_->window_ == nullptr) {
            return;
        }
        int count;
        GLFWmonitor** monitors = glfwGetMonitors(&count);
        if(int(monitor) >= count) {
            Logger::err("Application") << monitor << ": no such monitor"
                                       << std::endl;
        }
        if((w == 0) || (h == 0) || (Hz == 0)) {
            Logger::out("Application")
                << "Using default video mode" << std::endl;
            const GLFWvidmode* mode = glfwGetVideoMode(monitors[monitor]);
            w = index_t(mode->width);
            h = index_t(mode->height);
            Hz = index_t(mode->refreshRate);
        }
        glfwSetWindowMonitor(
            data_->window_, monitors[monitor], 0, 0, int(w), int(h), int(Hz)
        );
        update();
    }

    void Application::set_windowed_mode(index_t w, index_t h) {
        if(w != 0 && h != 0) {
            width_ = w;
            height_ = w;
        }
        glfwSetWindowMonitor(
            data_->window_, nullptr, 0, 0, int(width_), int(height_), 50
        );
        update();
    }


    void Application::list_video_modes() {
        int nb_monitors = 0;
        GLFWmonitor** monitors = glfwGetMonitors(&nb_monitors);
        Logger::out("Application") << "Detected " << nb_monitors
                                   << " monitor(s)"
                                   << std::endl;
        for(int m=0; m<nb_monitors; ++m) {
            GLFWmonitor* monitor = monitors[m];
            Logger::out("Application") << "Monitor " << m << ":"
                                       << glfwGetMonitorName(monitor)
                                       << std::endl;
            int nb_modes = 0;
            const GLFWvidmode* modes = glfwGetVideoModes(monitor, &nb_modes);
            for(int mm=0; mm<nb_modes; ++mm) {
                const GLFWvidmode& mode = modes[mm];
                Logger::out("Application") << "   mode " << mm << ":"
                                           << mode.width << "x" << mode.height
                                           << " " << mode.refreshRate << "Hz "
                                           << "R" << mode.redBits
                                           << "G" << mode.greenBits
                                           << "B" << mode.blueBits
                                           << std::endl;
            }
        }
    }


    void Application::iconify() {
        if(data_->window_ == nullptr ){
            return ;
        }
        glfwIconifyWindow(data_->window_);
    }

    void Application::restore() {
        if(data_->window_ == nullptr ){
            return ;
        }

        // In full screen mode, glfwRestoreWindow()
        // does not seem to work, so we switch to
        // windowed mode, deiconify, then switch back
        // to full screen mode.
        if(get_full_screen()) {
            set_full_screen(false);
            glfwRestoreWindow(data_->window_);
            set_full_screen(true);
        } else {
            glfwRestoreWindow(data_->window_);
        }
    }

    bool Application::get_full_screen() const {
        return (data_->window_ != nullptr &&
                glfwGetWindowMonitor(data_->window_) != nullptr);
    }

    void Application::set_full_screen(bool x) {
        if(x != get_full_screen()) {
            if(x) {
                set_full_screen_mode();
            } else {
                set_windowed_mode();
            }
        }
    }

    void* Application::impl_window() {
        return data_->window_;
    }

    /**************************** Android-specific code *********************/
#elif defined(GEO_OS_ANDROID)

    /**
     * \brief Redirects Geogram messages both to the console
     * and to Android log (adb logcat | grep GEOGRAM)
     */
    class AndroidLoggerClient : public LoggerClient {
    public:
        void div(const std::string& value) override {
            geo_argused(value);
        }
        void out(const std::string& value) override {
            __android_log_print(
                ANDROID_LOG_VERBOSE, "GEOGRAM", "%s", value.c_str()
            );
        }
        void warn(const std::string& value) override {
            __android_log_print(
                ANDROID_LOG_WARN, "GEOGRAM", "%s", value.c_str()
            );
        }
        void err(const std::string& value) override {
            __android_log_print(
                ANDROID_LOG_ERROR, "GEOGRAM", "%s", value.c_str()
            );
        }
        void status(const std::string& value) override {
            // Do not display error messages twice.
            if(GEO::String::string_starts_with(value, "Error:")) {
                return;
            }
        }
    };

    void Application::pre_draw() {
        // We initialize graphic resources in pre_draw() rather
        // than create_window() because Android apps can be restarted
        // or loose graphic resources. This function, called at each frame,
        // re-creates everything that needs to be created.

        // Get display and initialize GLES
        if(data_->display == EGL_NO_DISPLAY) {
            data_->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
            eglInitialize(data_->display, 0, 0);
        }

        // Choose best matching config and create surface
        if(data_->surface == EGL_NO_SURFACE) {
            const EGLint attribs[] = {
                EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT_KHR, // OpenGL ES 3.0
                EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
                EGL_BLUE_SIZE,  8,
                EGL_GREEN_SIZE, 8,
                EGL_RED_SIZE,   8,
                EGL_DEPTH_SIZE, 16,
                EGL_NONE
            };

            EGLint numConfigs;

            // Get the number of matching configs.
            eglChooseConfig(data_->display, attribs, nullptr, 0, &numConfigs);
            EGLConfig* supportedConfigs = new EGLConfig[numConfigs];
            assert(supportedConfigs != nullptr);
            // Do that again now that we know the number of configs.
            eglChooseConfig(
                data_->display, attribs, supportedConfigs,
                numConfigs, &numConfigs
            );
            assert(numConfigs != 0);

            // Find the best matching config. among them.
            int i = 0;
            for (; i < numConfigs; i++) {
                auto& cfg = supportedConfigs[i];
                EGLint r, g, b, d;
                if (
                    eglGetConfigAttrib(data_->display,cfg,EGL_RED_SIZE  ,&r) &&
                    eglGetConfigAttrib(data_->display,cfg,EGL_GREEN_SIZE,&g) &&
                    eglGetConfigAttrib(data_->display,cfg,EGL_BLUE_SIZE, &b) &&
                    eglGetConfigAttrib(data_->display,cfg,EGL_DEPTH_SIZE,&d) &&
                    r == 8 && g == 8 && b == 8 && d == 16
                ) {
                    data_->config = supportedConfigs[i];
                    break;
                }
            }
            // In the worst case, use the first one.
            if (i == numConfigs) {
                data_->config = supportedConfigs[0];
            }
            delete[] supportedConfigs;
            data_->surface = eglCreateWindowSurface(
                data_->display, data_->config, data_->app->window, nullptr
            );
        }

        // Create context. Try first OpenGL ES 3.0 then 2.0.
        if(data_->context == EGL_NO_CONTEXT) {

            // Important: create an OpenGL es 3.0 context. Without this,
            // it defaults to an OpenGL es 1.0 context,
            // and glCreateProgram() / glCreateBuffer() silently
            // fail and return 0 (banged my head to the wall before
            // figuring out !!)
            data_->GLES_version = 3;
            const EGLint context_attribs[] = {
                EGL_CONTEXT_CLIENT_VERSION, 3,
                EGL_NONE
            };

            data_->context = eglCreateContext(
                data_->display, data_->config, nullptr, context_attribs
            );

            // If we do not succeed, we create an OpenGL es 2.0 context.
            if(data_->context == EGL_NO_CONTEXT) {
                const EGLint context_attribs_2[] = {
                    EGL_CONTEXT_CLIENT_VERSION, 2,
                    EGL_NONE
                };
                data_->GLES_version = 2;
                data_->context = eglCreateContext(
                    data_->display, data_->config, nullptr, context_attribs_2
                );
            }
        }

        eglMakeCurrent(
            data_->display, data_->surface, data_->surface, data_->context
        );

        if(!data_->GL_initialized) {
            GL_initialize();
            data_->GL_initialized = true;
        }
    }

    void Application::post_draw() {
    }

    void Application::create_window() {
        data_->app = CmdLine::get_android_app();
        data_->app->userData = this;

        // Create a logger client for debugging.
        // (use adb logcat | grep GEOGRAM to see the messages)
        data_->logger_client = new AndroidLoggerClient();
        GEO::Logger::instance()->register_client(data_->logger_client);
        // Note: the display/context/surface are created as needed in pre_draw()
    }

    void Application::delete_window() {
        if (data_->display != EGL_NO_DISPLAY) {
            eglMakeCurrent(
                data_->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT
            );
            if (data_->context != EGL_NO_CONTEXT) {
                eglDestroyContext(data_->display, data_->context);
            }
            if (data_->surface != EGL_NO_SURFACE) {
                eglDestroySurface(data_->display, data_->surface);
            }
            eglTerminate(data_->display);
        }
        data_->display = EGL_NO_DISPLAY;
        data_->context = EGL_NO_CONTEXT;
        data_->surface = EGL_NO_SURFACE;
    }

    void Application::one_frame() {
        // Avoid nested ImGui calls
        // (due to calling draw())
        if(currently_drawing_gui_) {
            return;
        }
        pre_draw();
        if(data_->display == EGL_NO_DISPLAY) {
            return;
        }

        {
            EGLint new_width;
            EGLint new_height;
            eglQuerySurface(
                data_->display, data_->surface, EGL_WIDTH, &new_width
            );
            eglQuerySurface(
                data_->display, data_->surface, EGL_HEIGHT, &new_height
            );
            if(index_t(new_width) != width_ || index_t(new_height) != height_) {
                resize(
                    index_t(new_width), index_t(new_height),
                    index_t(new_width), index_t(new_height)
                );
            }
        }

        // Initialize ImGui if not already initialized.
        if(ImGui::GetCurrentContext() == nullptr) {
            ImGui_initialize();
        }

        if(needs_to_redraw()) {
            currently_drawing_gui_ = true;
            ImGui_new_frame();
            draw_graphics();
            draw_gui();
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glUseProgram(0); // RenderDrawData() leaves a bound program
            currently_drawing_gui_ = false;
            eglSwapBuffers(data_->display, data_->surface);
            post_draw();
            currently_drawing_gui_ = false;
            if(nb_frames_update_ > 0 && !animate_) {
                --nb_frames_update_;
            }
        } else {
            // Sleep for 0.2 seconds, to let the processor cool-down
            // instead of actively waiting (be a good citizen for the
            // other processes.
            Process::sleep(20000);
        }

        // ImGui needs to be restarted whenever docking state is reloaded.
        if(ImGui_restart_) {
            ImGui_restart_ = false;
            ImGui_terminate();
            if(CmdLine::arg_is_declared("gui:font_size")) {
                set_font_size(CmdLine::get_arg_uint("gui:font_size"));
            }
            ImGui_initialize();
        } else if(ImGui_reload_font_) {
            ImGuiIO& io = ImGui::GetIO();
            io.Fonts->Clear();
            ImGui_load_fonts();
            ImGui_ImplOpenGL3_DestroyDeviceObjects();
            ImGui_reload_font_ = false;
        }
    }

    void Application::main_loop() {
        callbacks_initialize();
        in_main_loop_ = true;
        while(in_main_loop_) {
            int ident;
            int events;
            android_poll_source* source;
            while (
                (ident = ALooper_pollAll(
                    // 0 = non-blocking, -1 = blocking
                    data_->should_draw() ? 0 : -1,
                    nullptr,
                    &events,
                    (void**)&source)
                ) >= 0
            ) {
                // process event
                if (source != nullptr) {
                    source->process(data_->app, source);
                }

                // Check if we are exiting.
                if (data_->app->destroyRequested != 0) {
                    ::GEO::AndroidUtils::debug_log(
                        "Destroy requested, freeing resources"
                    );
                    ImGui_terminate();
                    GL_terminate();
                    return;
                }
            }

            if(data_->should_draw()) {
                one_frame();
            }
        }
        ::GEO::AndroidUtils::debug_log("End of main loop");
        ::GEO::AndroidUtils::debug_log("calling std::terminate()");
        // Not very clean, but for now I do not know another
        // solution to exit an Android app.
        std::terminate();
    }

    namespace {

        int32_t android_input_event_handler(
            struct android_app* app, AInputEvent* event
        ) {
            int32_t result = ImGui_ImplAndroidExt_HandleInputEvent(event);
            Application* geoapp = static_cast<Application*>(
                CmdLine::get_android_app()->userData
            );
            ImGui_ImplAndroidExt_HandleEventUserCallback(app, event);
            geoapp->update();
            return result;
        }

        void android_command_handler(struct android_app* app, int32_t cmd) {
            Application* app_impl =
                static_cast<GEO::Application*>(app->userData);
            ApplicationData* data = app_impl->impl_data();
            app_impl->update();

            switch (cmd) {
            case APP_CMD_INIT_WINDOW:
                ::GEO::AndroidUtils::debug_log("command: APP_CMD_INIT_WINDOW");
                data->has_window = (app->window != nullptr);
                break;

            case APP_CMD_TERM_WINDOW:
                // This event may be triggered when the app is re-started.
                // Like in the endless-tunnel demo of the NDK,
                // we just destroy the surface. If the application is
                // restarted, the surface will be re-created by pre_draw()
                // that is called at the beginning of one_frame().
                // Note: AndroidManifest.xml needs to have:
                // android:configChanges=
                // "orientation|screenSize|keyboardHidden|keyboard|navigation"
                // else this event will be triggered  when a physical
                // keyboard is connected/disconnected while the app is
                // running. Note "navigation"
                ::GEO::AndroidUtils::debug_log("command: APP_CMD_TERM_WINDOW");
                app_impl->ImGui_terminate();
                app_impl->GL_terminate();
                app_impl->delete_window();
                break;

            case APP_CMD_GAINED_FOCUS:
                ::GEO::AndroidUtils::debug_log("command: APP_CMD_GAINED_FOCUS");
                data->has_focus = true;
                break;

            case APP_CMD_LOST_FOCUS:
                ::GEO::AndroidUtils::debug_log("command: APP_CMD_LOST_FOCUS");
                data->has_focus = false;
                break;

            case APP_CMD_START:
                ::GEO::AndroidUtils::debug_log("command: APP_CMD_START");
                data->is_visible = true;
                break;

            case APP_CMD_STOP:
                ::GEO::AndroidUtils::debug_log("command: APP_CMD_STOP");
                data->is_visible = false;
                break;

            case APP_CMD_SAVE_STATE:
                ::GEO::AndroidUtils::debug_log("command: APP_CMD_SAVE_STATE");
                break;

            case APP_CMD_PAUSE:
                ::GEO::AndroidUtils::debug_log("command: APP_CMD_PAUSE");
                break;

            case APP_CMD_RESUME:
                ::GEO::AndroidUtils::debug_log("command: APP_CMD_RESUME");
                break;

            case APP_CMD_WINDOW_RESIZED:
                ::GEO::AndroidUtils::debug_log("command: APP_CMD_WINDOW_RESIZED");
                break;

            case APP_CMD_CONFIG_CHANGED:
                ::GEO::AndroidUtils::debug_log("command: APP_CMD_CONFIG_CHANGED");
                break;

            case APP_CMD_LOW_MEMORY:
                ::GEO::AndroidUtils::debug_log("command: APP_CMD_LOW_MEMORY");
                break;
            }
        }

        /*
         * \brief The callback to handle Android mouse events in the rendering
         *  area
         * \details See ImGui_ImplAndroidExt_SetMouseUserCallback()
         * \param[in] x , y window coordinates of the event
         * \param[in] button the button
         * \param[in] action the action (one of
         *  EVENT_ACTION_UP, EVENT_ACTION_DOWN, EVENT_ACTION_DRAG)
         * \param[in] source the event source (one of EVENT_SOURCE_MOUSE,
         *   EVENT_SOURCE_FINGER, EVENT_SOURCE_STYLUS)
         */
        void android_mouse_callback(
            float x, float y, int button, int action, int source
        ) {
            Application* app = static_cast<Application*>(
                CmdLine::get_android_app()->userData
            );

            static const char* action_names[] = {
                "up", "down", "drag", "unknown"
            };

            static const char* source_names[] = {
                "keyboard", "mouse", "finger", "stylus", "unknown"
            };

            ::GEO::AndroidUtils::debug_log(
                std::string("mouse CB  ") +
                " (" + String::to_string(x) + "," + String::to_string(y) + ")" +
                " btn=" + String::to_string(button) +
                " action=" + action_names[std::min(action,3)] +
                " source=" + source_names[std::min(source,4)]
            );

            // For touch devices, hovering does not generate
            // events, and we need to update ImGui flags that
            // indicate whether we are hovering ImGui or another
            // zone of the window.
            if(
                action == EVENT_ACTION_DOWN &&
                (source == EVENT_SOURCE_FINGER || source == EVENT_SOURCE_MOUSE)
            ) {
                ImGui::GetIO().MousePos = ImVec2(x,y);
                ImGui::UpdateHoveredWindowAndCaptureFlags();
            }

            if(action != EVENT_ACTION_UNKNOWN) {
                if(!ImGui::GetIO().WantCaptureMouse) {
                    if(action != EVENT_ACTION_UP) {
                        app->cursor_pos_callback(double(x), double(y), source);
                    }
                    app->mouse_button_callback(button, action, 0, source);
                }
            }
            app->update();
        }

    }

    void Application::callbacks_initialize() {
        data_->app->onAppCmd     = android_command_handler;
        data_->app->onInputEvent = android_input_event_handler;
        ImGui_ImplAndroidExt_SetMouseUserCallback(android_mouse_callback);
    }

    void Application::set_window_icon(Image* icon_image) {
        geo_argused(icon_image);
    }

    void Application::set_full_screen_mode(
        index_t w, index_t h, index_t Hz, index_t monitor
    ) {
        geo_argused(w);
        geo_argused(h);
        geo_argused(Hz);
        geo_argused(monitor);
    }

    void Application::set_windowed_mode(index_t w, index_t h) {
        geo_argused(w);
        geo_argused(h);
    }

    void Application::list_video_modes() {
    }

    void Application::iconify() {
    }

    void Application::restore() {
    }

    bool Application::get_full_screen() const {
        return true;
    }

    void Application::set_full_screen(bool x) {
        geo_argused(x);
    }

    void* Application::impl_window() {
        return nullptr;
    }
#else
# error "No windowing system"
#endif


#ifdef GEO_GLFW
    const char* Application::key_to_string(int key) {
        if(key == GLFW_KEY_LEFT) {
            return "left";
        }
        if(key == GLFW_KEY_RIGHT) {
            return "right";
        }
        if(key == GLFW_KEY_UP) {
            return "up";
        }
        if(key == GLFW_KEY_DOWN) {
            return "down";
        }
        if(key == GLFW_KEY_PAGE_UP) {
            return "page_up";
        }
        if(key == GLFW_KEY_PAGE_DOWN) {
            return "page_down";
        }
        if(key == GLFW_KEY_HOME) {
            return "home";
        }
        if(key == GLFW_KEY_END) {
            return "end";
        }
        if(key == GLFW_KEY_F1) {
            return "F1";
        }
        if(key == GLFW_KEY_F2) {
            return "F2";
        }
        if(key == GLFW_KEY_F3) {
            return "F3";
        }
        if(key == GLFW_KEY_F4) {
            return "F4";
        }
        if(key == GLFW_KEY_F5) {
            return "F5";
        }
        if(key == GLFW_KEY_F6) {
            return "F6";
        }
        if(key == GLFW_KEY_F7) {
            return "F7";
        }
        if(key == GLFW_KEY_F8) {
            return "F8";
        }
        if(key == GLFW_KEY_F9) {
            return "F9";
        }
        if(key == GLFW_KEY_F10) {
            return "F10";
        }
        if(key == GLFW_KEY_F11) {
            return "F11";
        }
        if(key == GLFW_KEY_F12) {
            return "F12";
        }
        if(key == GLFW_KEY_LEFT_CONTROL) {
            return "left_control";
        }
        if(key == GLFW_KEY_RIGHT_CONTROL) {
            return "right_control";
        }
        if(key == GLFW_KEY_LEFT_ALT) {
            return "left_alt";
        }
        if(key == GLFW_KEY_RIGHT_ALT) {
            return "right_alt";
        }
        if(key == GLFW_KEY_LEFT_SHIFT) {
            return "left_shift";
        }
        if(key == GLFW_KEY_RIGHT_SHIFT) {
            return "right_shift";
        }
        if(key == GLFW_KEY_ESCAPE) {
            return "escape";
        }
        if(key == GLFW_KEY_TAB) {
            return "tab";
        }
        if(key == GLFW_KEY_BACKSPACE) {
            return "backspace";
        }
        return "";
    }
#else
    const char* Application::key_to_string(int key) {
        return "";
    }
#endif

}

/************************ Utilities *************************************/

namespace GEO {

#if defined(GEO_GLFW) && !defined(GEO_OS_EMSCRIPTEN)

    double compute_pixel_ratio() {
        int buf_size[2];
        int win_size[2];
        GLFWwindow* window = glfwGetCurrentContext();
        glfwGetFramebufferSize(window, &buf_size[0], &buf_size[1]);
        glfwGetWindowSize(window, &win_size[0], &win_size[1]);
        // The window may be iconified.
        if(win_size[0] == 0) {
            return 1.0;
        }
        return double(buf_size[0]) / double(win_size[0]);
    }

    double compute_hidpi_scaling() {
        float xscale, yscale;
        GLFWwindow* window = glfwGetCurrentContext();
        glfwGetWindowContentScale(window, &xscale, &yscale);
        return 0.5 * double(xscale + yscale);
    }

#else

    double compute_pixel_ratio() {
        return 1.0;
    }

    double compute_hidpi_scaling() {
        return 1.0;
    }

#endif

}
