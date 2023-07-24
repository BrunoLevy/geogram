
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
#include <geogram/basic/command_line.h>
#include <geogram_gfx/gui/application.h>


using namespace GEO;

namespace {

    struct android_app* g_app = nullptr;
    double g_Time = 0.0;
    float g_mouseX = 0.0f;
    float g_mouseY = 0.0f;
    bool  g_mousePressed[5]     = {false, false, false, false, false};
    int   g_mouseJustPressed[5] = {0, 0, 0, 0, 0};    
    bool  g_resetKeys = false;


}

bool ImGui_ImplAndroidExt_Init(struct ANativeWindow* window) {
    geo_argused(window);
    g_app = GEO::CmdLine::get_android_app();
    g_Time = 0.0;
    g_mouseX = 0.0f;
    g_mouseY = 0.0f;
    for (int i = 0; i < IM_ARRAYSIZE(g_mousePressed); i++) {
	g_mousePressed[i]     = false;
	g_mouseJustPressed[i] = 0;	
    }
    return true;
}

void ImGui_ImplAndroidExt_Shutdown() {
}

static void ImGui_ImplAndroidExt_UpdateMousePosAndButtons()
{
    ImGuiIO& io = ImGui::GetIO();

    for (int i = 0; i < IM_ARRAYSIZE(io.MouseDown); i++) {
	// We do the same thing as in imgui_impl_glfw.cpp:
        // If a mouse press event came, always pass it as
	// "mouse held this frame", so we don't miss click-release
	// events that are shorter than 1 frame.
	// (unlike in imgui_impl_glfw.cpp, we do that during several
	//  frames instead of a single one).
        io.MouseDown[i] = (g_mouseJustPressed[i] != 0) || g_mousePressed[i];
	if(g_mouseJustPressed[i] != 0) {
	    --g_mouseJustPressed[i];
	}
    }
    io.MousePos = ImVec2(g_mouseX, g_mouseY);
}

static void ImGui_ImplAndroidExt_UpdateMouseCursor() {
}

void ImGui_ImplAndroidExt_NewFrame() {
    ImGuiIO& io = ImGui::GetIO();
    // Font atlas needs to be built, call renderer _NewFrame() function
    // e.g. ImGui_ImplOpenGL3_NewFrame()     
    IM_ASSERT(io.Fonts->IsBuilt());     

    // Get current display size
    EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    EGLSurface surface = eglGetCurrentSurface(EGL_DRAW);
    int w=0;
    int h=0;
    eglQuerySurface(display, surface, EGL_WIDTH, &w);
    eglQuerySurface(display, surface, EGL_HEIGHT, &h);
    int display_w = w;
    int display_h = h;

    io.DisplaySize = ImVec2((float)w, (float)h);
    io.DisplayFramebufferScale =
	ImVec2(
	    w > 0 ?((float)display_w / w) : 0,
	    h > 0 ? ((float)display_h / h) : 0
    );

    // Setup time step
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double current_time = double(now.tv_sec) + double(now.tv_nsec) * 1e-9;
    
    io.DeltaTime = g_Time > 0.0 ? float(current_time - g_Time) : 1.0f/60.0f;
    g_Time = current_time;

    ImGui_ImplAndroidExt_UpdateMousePosAndButtons();
    ImGui_ImplAndroidExt_UpdateMouseCursor();
}


void ImGui_ImplAndroidExt_EndFrame() {
    // g_resetKeys is set when the latest key event came from the soft keyboard,
    // then we need to reset the keys.
    if(g_resetKeys) {
	ImGuiIO& io = ImGui::GetIO();
        /*
	for(int key = 0; key < IM_ARRAYSIZE(io.KeysDown); ++key) {
	    io.KeysDown[key] = false;	    
	}
        */
	io.KeyShift = false;
	io.KeyCtrl = false;
	io.KeyAlt = false;
	io.KeySuper = false;
	g_resetKeys = false;
    }
}

// Emulates mouse buttons using multiple fingers:
//   emulated mouse button is determined by number of fingers
//   coordinates are defined by last finger
int32_t  ImGui_ImplAndroidExt_FingerEvent(
    struct android_app* app, AInputEvent* event
) {
    int32_t action = AMotionEvent_getAction(event);
    bool down_or_move = (action == AMOTION_EVENT_ACTION_DOWN ||
			 action == AMOTION_EVENT_ACTION_MOVE );

    int nb_fingers = int(AMotionEvent_getPointerCount(event));

    int btn = nb_fingers-1;
    for(int i=0; i<IM_ARRAYSIZE(g_mousePressed); ++i) {
	if(i == btn) {
	    g_mousePressed[i] = down_or_move;
	    if(action == AMOTION_EVENT_ACTION_DOWN) {
		// Notify that the mouse was just pressed during two
		// frames. This is needed else ImGui does not notice
		// that the button was pressed (I think it needs one
		// frame to notice that the cursor moved to the button,
		// and another frame to notice that the button was
		// pressed). This is because fingers do not generate
		// events when hovering the touch screen.
		g_mouseJustPressed[i] = 2;
	    }
	} else {
	    g_mousePressed[i] = false;
	}
    }

    g_mouseX = AMotionEvent_getX(event, nb_fingers-1);
    g_mouseY = AMotionEvent_getY(event, nb_fingers-1);
    return 1;
}


// Handles stylus input, like Galaxy SPen. Uses the tiny button on
// the pen to emulate second mouse button.
int32_t ImGui_ImplAndroidExt_StylusEvent(
    struct android_app* app, AInputEvent* event
) {
    int32_t action = AMotionEvent_getAction(event);
    bool down_or_move = (action == AMOTION_EVENT_ACTION_DOWN ||
			 action == AMOTION_EVENT_ACTION_MOVE );

    int btn = (
	(AMotionEvent_getButtonState(event) &
	    AMOTION_EVENT_BUTTON_STYLUS_PRIMARY) != 0
    ) ? 1 : 0;

    for(int i=0; i<IM_ARRAYSIZE(g_mousePressed); ++i) {
	if(i == btn) {
	    g_mousePressed[i] = down_or_move;
	    if(action == AMOTION_EVENT_ACTION_DOWN) {
		g_mouseJustPressed[i] = 1;
	    }
	} else {
	    g_mousePressed[i] = false;
	}
    }
    g_mouseX = AMotionEvent_getX(event, 0);
    g_mouseY = AMotionEvent_getY(event, 0);

    return 1;    
}


// Handles a standard USB or bluetooth mouse connected to the phone.
int32_t  ImGui_ImplAndroidExt_MouseEvent(
    struct android_app* app, AInputEvent* event
) {
    int32_t buttons = AMotionEvent_getButtonState(event);
    for(int i=0; i<IM_ARRAYSIZE(g_mousePressed); ++i) {
	g_mousePressed[i] = false;
    }
    g_mousePressed[0] = (buttons &  AMOTION_EVENT_BUTTON_PRIMARY) != 0;
    g_mousePressed[1] = (buttons &  AMOTION_EVENT_BUTTON_SECONDARY) != 0;
    g_mousePressed[2] = (buttons &  AMOTION_EVENT_BUTTON_TERTIARY) != 0;
    // TODO: g_mouseJustPressed
    g_mouseX = AMotionEvent_getX(event, 0);
    g_mouseY = AMotionEvent_getY(event, 0);

    // Mouse wheel
    int32_t action = AMotionEvent_getAction(event);

    if(action == AMOTION_EVENT_ACTION_SCROLL) {
	float hscroll = AMotionEvent_getAxisValue(
	    event, AMOTION_EVENT_AXIS_HSCROLL, 0
        );
	float vscroll = AMotionEvent_getAxisValue(
	    event, AMOTION_EVENT_AXIS_VSCROLL, 0
        );
	ImGuiIO& io = ImGui::GetIO();
	io.MouseWheelH += hscroll;
	io.MouseWheel  += vscroll;
    }
    return 1;    
}

int32_t ImGui_ImplAndroidExt_MotionEvent(
    struct android_app* app, AInputEvent* event
) {
    int32_t result = 0;
    switch(AMotionEvent_getToolType(event,0)) {
	case AMOTION_EVENT_TOOL_TYPE_FINGER:
	    result = ImGui_ImplAndroidExt_FingerEvent(app, event);
	    break;
	case AMOTION_EVENT_TOOL_TYPE_STYLUS:
	    result = ImGui_ImplAndroidExt_StylusEvent(app, event);	    
	    break;
	case AMOTION_EVENT_TOOL_TYPE_MOUSE:
	    result = ImGui_ImplAndroidExt_MouseEvent(app, event);	    	    
	    break;
	default:
	    break;
    }
    return result;
}

int32_t ImGui_ImplAndroidExt_KeyEvent(
    struct android_app* app, AInputEvent* event
) {
    // Note: important to return 1 on BACK key pressed,
    // else this triggers default behavior that stops the
    // application brutally.

    ImGuiIO& io = ImGui::GetIO();
    
    int32_t action = AKeyEvent_getAction(event);
    int32_t key = AKeyEvent_getKeyCode(event);
    int32_t modifiers = AKeyEvent_getMetaState(event);
    int32_t device = AInputEvent_getDeviceId(event);    

    /*
    // TODO
    if(key >= 0 && key < IM_ARRAYSIZE(io.KeysDown)) {
	if((AKeyEvent_getFlags(event) & AKEY_EVENT_FLAG_SOFT_KEYBOARD)) {
	    // The soft keyboard generates Push/Release events when the
	    // key is released. Thus we mark the key as pushed, and
	    // set g_resetKeys so that ImGui_ImplAndroidExt_EndFrame()
	    // will mark the key as released after ImGui could do what
	    // it has to do with the key.
	    io.KeysDown[key] = true;
	    g_resetKeys = true;
	} else {
	    io.KeysDown[key] = (action == AKEY_EVENT_ACTION_DOWN);
	    g_resetKeys = false;	    
	}
	io.KeyShift = ((modifiers & AMETA_SHIFT_ON) != 0);
	io.KeyCtrl = ((modifiers & AMETA_CTRL_ON) != 0);
	io.KeyAlt = ((modifiers & AMETA_ALT_ON) != 0);
	io.KeySuper = ((modifiers & AMETA_META_ON) != 0);
    }
    */
    
    if(action == AKEY_EVENT_ACTION_DOWN) {
	if(key != AKEYCODE_BACK) {
	    jint unicode = AndroidUtils::keycode_to_unicode(
		app, device, key, modifiers
	    );
	    // TODO: use AddInputCharactersUTF8()
	    char c = char(unicode);
	    if(isprint(c)) {
		io.AddInputCharacter(c);
	    }
	}
    }
    
    return 1;
}

int32_t ImGui_ImplAndroidExt_HandleInputEvent(
    struct android_app* app, AInputEvent* event
) {
    int32_t result = 0;
    switch(AInputEvent_getType(event)) {
	case AINPUT_EVENT_TYPE_MOTION:
	    result = ImGui_ImplAndroidExt_MotionEvent(app, event);
	    break;
	case AINPUT_EVENT_TYPE_KEY:
	    result = ImGui_ImplAndroidExt_KeyEvent(app, event);
	    break;
	default:
	    break;
    }
    return result;
}

/********************************************************************/
/********************************************************************/

#endif



