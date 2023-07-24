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

#endif // __ANDROID__

#endif // IMGUI_IMPL_ANDROID_EXT
