/*
 *  Copyright (c) 2000-2022 Inria
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

#ifndef H_USER_CALLBACK_ANDROID_H
#define H_USER_CALLBACK_ANDROID_H

/**
 * \file geogram_gfx/gui/user_callback_android.h
 * \brief functions to handle user input in the rendering area of
 *   a geogram application
 * \details translates fingers,stylus,mouse android events into a unified 
 *   callback.
 */

#ifdef __ANDROID__

#include <geogram_gfx/gui/events.h>
#include <android_native_app_glue.h>

/**
 * x,y window coordinats (0..width-1 x 0..height-1)
 * button: one of 0:left, 1:right, 2:middle
 * action: one of 0:UP, 1:DOWN, 2:DRAG
 * source: one of 0:KEYBOARD, 1:MOUSE, 2:FINGER, 3:STYLUS
 */
typedef void (*ImGui_ImplAndroidExt_MouseUserCallback)(
    float x, float y, int button, int action, int source
);

/**
 * \brief Registers a user mouse event handler.
 * \details the mouse handler needs to test the ImGui::GetIO().WantCaptureMouse
 * flag to determine whether the event should be processed. The reason why
 * it is not tested before the handler is because when a menu is open and
 * the user clicks outside the menu, the flag is still set (this situation
 * needs special code to be handled properly).
 */
 void ImGui_ImplAndroidExt_SetMouseUserCallback(
    ImGui_ImplAndroidExt_MouseUserCallback CB
);

/**
 * \brief Processes an event and calls the registered user callback accordingly
 * \details can be called right after the ImGui event handler
 */
int32_t ImGui_ImplAndroidExt_HandleEventUserCallback(
    struct android_app* app, AInputEvent* event
);

#endif
#endif
