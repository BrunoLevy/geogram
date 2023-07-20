/*
 * ImGui Platform Binding for: Android
 * Author: Bruno Levy   Sun Aug 19 08:01:39 CEST 2018
 * Note: not part (yet) of the official ImGui distribution
 */ 

#ifndef IMGUI_IMPL_ANDROID_EXT
#define IMGUI_IMPL_ANDROID_EXT

#ifdef __ANDROID__
#include <android_native_app_glue.h>

IMGUI_IMPL_API bool     ImGui_ImplAndroidExt_Init(
    struct ANativeWindow* window 
);

// Needs to be called at the end, deallocates resources
IMGUI_IMPL_API void     ImGui_ImplAndroidExt_Shutdown();

// Needs to be called at the beginning of each frame,
// before ImGui::NewFrame().
IMGUI_IMPL_API void     ImGui_ImplAndroidExt_NewFrame();

// Needs to be called at the end of each frame,
// after all other ImGui functions.
IMGUI_IMPL_API void     ImGui_ImplAndroidExt_EndFrame();

// The event handler (make app->onInputEvent point to it)
IMGUI_IMPL_API int32_t  ImGui_ImplAndroidExt_HandleInputEvent(
    struct android_app* app, AInputEvent* event
);

// x,y window coordinats (0..width-1 x 0..height-1)
// button: one of 0:left, 1:right, 2:middle
// action: one of 0:UP, 1:DOWN, 2:DRAG
// source: one of 0:KEYBOARD, 1:MOUSE, 2:FINGER, 3:STYLUS
typedef void (*ImGui_ImplAndroidExt_MouseUserCallback)(
    float x, float y, int button, int action, int source
);

//   Registers a user mouse event handler.
// Note: the mouse handler needs to test the ImGui::GetIO().WantCaptureMouse
// flag to determine whether the event should be processed. The reason why
// it is not tested before the handler is because when a menu is open and
// the user clicks outside the menu, the flag is still set (this situation
// needs special code to be handled properly).
IMGUI_IMPL_API void ImGui_ImplAndroidExt_SetMouseUserCallback(
    ImGui_ImplAndroidExt_MouseUserCallback CB
);

#endif // __ANDROID__

#endif // IMGUI_IMPL_ANDROID_EXT
