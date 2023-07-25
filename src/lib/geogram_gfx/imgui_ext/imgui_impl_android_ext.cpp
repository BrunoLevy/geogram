// ImGui Platform Binding for: Android

#ifdef __ANDROID__

#include "imgui.h"
#include "imgui_impl_android_ext.h"

#include <EGL/egl.h>
#include <GLES/gl.h>
#include <android/log.h>

#include <time.h>
#include <cassert>
#include <stdexcept>
#include <cctype>
#include <cmath>

#include <string>

#include <geogram/basic/string.h>
#include <geogram/basic/android_utils.h>
#include <geogram_gfx/gui/application.h>

#include <geogram_gfx/third_party/imgui/backends/imgui_impl_android.h>

#include <android/log.h>

using namespace GEO;


// move mouse handler code to Application
// fingers
// stylus
// mouse
// soft keyboard show/hide
// keys translation
// blue tooth keyboard

/********************************************************************/

inline void android_debug(const std::string& str) {
    __android_log_print(
        ANDROID_LOG_VERBOSE, "GEOGRAM", "DBG: %s", str.c_str()
   );
}

/********************************************************************/

bool ImGui_ImplAndroidExt_Init(struct ANativeWindow* window) {
    return ImGui_ImplAndroid_Init(window);
}

void ImGui_ImplAndroidExt_Shutdown() {
    ImGui_ImplAndroid_Shutdown();
}

void ImGui_ImplAndroidExt_NewFrame() {
    ImGui_ImplAndroid_NewFrame();
}


void ImGui_ImplAndroidExt_EndFrame() {
    // TODO: We may need to reset keys when key event was triggered
    // by the soft keyboard 
}


static const char* event_type_to_str(int32_t event_type) {
    switch(event_type) {
    case AINPUT_EVENT_TYPE_KEY:
        return "key";
    case AINPUT_EVENT_TYPE_MOTION:
        return "motion";
    default:
        return "unknown";
    }
}

static const char* event_action_to_str(int32_t event_action) {
    switch(event_action) {
    case AKEY_EVENT_ACTION_DOWN:
        return "key/motion_down";
    case AKEY_EVENT_ACTION_UP:
        return "key/motion_up";
    case AMOTION_EVENT_ACTION_BUTTON_PRESS:
        return "motion_button_press";
    case AMOTION_EVENT_ACTION_BUTTON_RELEASE:
        return "motion_button_release";
    case AMOTION_EVENT_ACTION_HOVER_MOVE:
        return "motion_hover_move";
    case AMOTION_EVENT_ACTION_MOVE:
        return "motion_move";
    default:
        return "unknown";
    }
}

static const char* event_tool_type_to_str(int32_t event_tool_type) {
    switch(event_tool_type) {
        case AMOTION_EVENT_TOOL_TYPE_MOUSE:
            return "mouse";
        case AMOTION_EVENT_TOOL_TYPE_STYLUS:
            return "stylus";
        case AMOTION_EVENT_TOOL_TYPE_ERASER:
            return "eraser";
        case AMOTION_EVENT_TOOL_TYPE_FINGER:
            return "finger";
        default:
            return "unknown";
    }
}

void debug_show_event(AInputEvent* event) {
    std::string msg = std::string("Event=") +
        " type:"   + std::string(event_type_to_str(AInputEvent_getType(event)));
    
    if(AInputEvent_getType(event) == AINPUT_EVENT_TYPE_MOTION) {
        msg +=
         " action:"     +
         std::string(event_action_to_str(AMotionEvent_getAction(event))) +
         " tool:"       +
         std::string(event_tool_type_to_str(AMotionEvent_getToolType(event,0)))+
         " nb_fingers:" +
         ::GEO::String::to_string(int(AMotionEvent_getPointerCount(event))) ;
    }
    
    android_debug(msg);
}

int32_t ImGui_ImplAndroidExt_HandleInputEvent(
    struct android_app* app, AInputEvent* event
) {
    debug_show_event(event);
    return ImGui_ImplAndroid_HandleInputEvent(event);
}

/********************************************************************/

#endif

