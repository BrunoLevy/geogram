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

#ifdef __ANDROID__

#include <geogram_gfx/gui/user_callback_android.h>
#include <geogram_gfx/gui/application.h>
#include <geogram_gfx/imgui_ext/imgui_impl_android_ext.h>
#include <geogram/basic/android_utils.h>
#include "imgui.h"

using namespace GEO;

namespace {
    ImGui_ImplAndroidExt_MouseUserCallback g_mouse_CB = nullptr;

    /**
     * \brief Converts an Android action code into a Geogram action code
     * \param[in] int action the android action code
     * \return the Geogram action code, as defined in geogram_gfx/gui/events.h
     */
    inline int decode_action(int action) {
	switch(action) {
	    case AMOTION_EVENT_ACTION_BUTTON_PRESS:
	    case AMOTION_EVENT_ACTION_DOWN:
		return EVENT_ACTION_DOWN;
	    case AMOTION_EVENT_ACTION_BUTTON_RELEASE:	    
	    case AMOTION_EVENT_ACTION_UP:
		return EVENT_ACTION_UP;
	    case AMOTION_EVENT_ACTION_MOVE:
		return EVENT_ACTION_DRAG;
	}
	return EVENT_ACTION_UNKNOWN;
    }

    /**
     * \brief Gets from an android event the information in the form taken
     *  by the unified callback
     * \param[in] event the android event
     * \parma[out] x , y window coordinates, in 0..width-1, 0..height-1
     * \param[out] button one of 0 (left), 1 (right), 2 (middle)
     * \param[out] action one of 
     *    EVENT_ACTION_DOWN, EVENT_ACTION_UP, EVENT_ACTION_DRAG
     * \param[out] source one of EVENT_SOURCE_FINGER, EVENT_SOURCE_STYLUS,
     *    EVENT_SOURCE_MOUSE
     */
    void decode_android_event(
        const AInputEvent* event,
        double& x, double& y,
        int& button, int& action, int& source
    ) {
        action = EVENT_ACTION_UNKNOWN;
        
        if(AInputEvent_getType(event) != AINPUT_EVENT_TYPE_MOTION) {
            return;
        }

        action = decode_action(AMotionEvent_getAction(event));
        
        x = double(AMotionEvent_getX(event,0));
        y = double(AMotionEvent_getY(event,0));
        
        switch(AMotionEvent_getToolType(event,0)) {
        case AMOTION_EVENT_TOOL_TYPE_FINGER:
            source = EVENT_SOURCE_FINGER;
            button = 0;
	    break;
        case AMOTION_EVENT_TOOL_TYPE_STYLUS:
            source = EVENT_SOURCE_STYLUS;
            button = int((AMotionEvent_getButtonState(event) &
                          AMOTION_EVENT_BUTTON_STYLUS_PRIMARY) != 0);
	    break;
        case AMOTION_EVENT_TOOL_TYPE_MOUSE:
            source = EVENT_SOURCE_MOUSE;
            action = EVENT_ACTION_UNKNOWN; 
            break;
        default:
            action = EVENT_ACTION_UNKNOWN; 
            break;
        }
    }

    /**
     * \brief Computes the barycenter of two points
     * \param[in] p1 , p2 the two points
     * \return the barycenter of \p p1 and \p p2
     */
    inline ImVec2 barycenter(const ImVec2& p1, const ImVec2& p2) {
	return ImVec2(0.5f*(p1.x+p1.x), 0.5f*(p1.y+p2.y));
    }

    /**
     * \brief Computes the distance between two points
     * \param[in] p1 , p2 the two points
     * \return the distance between \p p1 and \p p2
     */
    inline float distance(const ImVec2& p1, const ImVec2& p2) {
	return ::sqrtf((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y));
    }


    /**
     * \brief Event translation for finger events
     * \details used by ImGui_ImplAndroidExt_HandleEventUserCallback()
     * \param[in] app the android app
     * \param[in] event the android event
     */
    int32_t HandleEventUserCallback_fingers(
        struct android_app* app, AInputEvent* event
    ) {
        int nb_fingers = int(AMotionEvent_getPointerCount(event));
        int32_t action = AMotionEvent_getAction(event);
        float mouseX = AMotionEvent_getX(event, nb_fingers-1);
        float mouseY = AMotionEvent_getY(event, nb_fingers-1);
        
	static int last_button_ = -1;
	
	if(nb_fingers == 1) {
	    if(last_button_ != -1 && last_button_ != 0) {
		g_mouse_CB(
		    mouseX, mouseY, last_button_,
		    EVENT_ACTION_UP, EVENT_SOURCE_FINGER
		);
	    }
	    last_button_ = 0;
	    g_mouse_CB(
		mouseX, mouseY, 0,
		decode_action(action), EVENT_SOURCE_FINGER
	    );
	} else if(nb_fingers == 2) {
	    // Two-fingers interactions: does both zoom (button 1) and
	    // translation (button 2). The chosen action depends on the
	    // variation of the distance between the two fingers and the
	    // displacement of the centroid of the two fingers:
	    // if distance varies most -> zoom
	    // if centroid moves most  -> translation
	    
	    if(last_button_ != -1 && last_button_ != 2) {
		g_mouse_CB(
		    mouseX, mouseY, last_button_,
		    EVENT_ACTION_UP, EVENT_SOURCE_FINGER
		);
	    }
	    ImVec2 finger1(
		AMotionEvent_getX(event, 0),
		AMotionEvent_getY(event, 0)
	    );
	    ImVec2 finger2(
		AMotionEvent_getX(event, 1),
		AMotionEvent_getY(event, 1)
	    );
	    float length = distance(finger1, finger2);
	    ImVec2 center = barycenter(finger1, finger2);
	    
	    static float last_length = 0.0f;
	    static ImVec2 last_center;

	    if(action == AMOTION_EVENT_ACTION_MOVE) {
		if(distance(center, last_center) > ::fabs(length-last_length)) {
		    // Translation: synthetise press btn 1, move, release btn 1
		    g_mouse_CB(
			last_center.x, last_center.y, 1,
			EVENT_ACTION_DOWN, EVENT_SOURCE_FINGER
		    );
		    g_mouse_CB(
			center.x, center.y, 1,
			EVENT_ACTION_DRAG, EVENT_SOURCE_FINGER
		    );
		    g_mouse_CB(
			center.x, center.y, 1,
			EVENT_ACTION_UP, EVENT_SOURCE_FINGER
		    );
		} else {
		    // Zoom: synthetise press btn 2, move, release btn 2
		    g_mouse_CB(
			0.0f, last_length, 2,
			EVENT_ACTION_DOWN, EVENT_SOURCE_FINGER
		    );
		    g_mouse_CB(
			0.0f, length, 2,
			EVENT_ACTION_DRAG, EVENT_SOURCE_FINGER
		    );
		    g_mouse_CB(
			0.0f, length, 2,
			EVENT_ACTION_UP, EVENT_SOURCE_FINGER
		    );
		}
	    }
	    last_length = length;
	    last_center = center;
	    last_button_ = 2;
	} else if(nb_fingers == 3) {
	    if(last_button_ != -1 && last_button_ != 1) {
		g_mouse_CB(
		    mouseX, mouseY, last_button_,
		    EVENT_ACTION_UP, EVENT_SOURCE_FINGER
		);
	    }
	    if(last_button_ != 1) {
		last_button_ = 1;
		g_mouse_CB(
		    mouseX, mouseY, 1,
		    EVENT_ACTION_DOWN, EVENT_SOURCE_FINGER
		);
	    } else {
		g_mouse_CB(
		    mouseX, mouseY, 1,
		    decode_action(action), EVENT_SOURCE_FINGER
		);
	    }
	}

        // When a menu is open and you click elsewhere, the
        // WantCaptureMouse flag is still set, and the framework
        // misses the "mouse button up" event. If a translation is
        // active, it remains active later ("sticky translation" bug).
        // The following code always generates a "mouse button up" event
        // to solve this problem.
        if(
            ImGui::GetIO().WantCaptureMouse &&
            decode_action(action) == EVENT_ACTION_DOWN
        ) {
            ImVec2 mouse_pos = ImGui::GetIO().MousePos;
            double x;
            double y;
            int button;
            int action;
            int source;
            // TODO (clean): remove decode_android_evente(), do the job here.
            // TODO: generates a mouse action down in fact ? to be checked
            decode_android_event(event, x, y, button, action, source);
            g_mouse_CB(mouse_pos.x, mouse_pos.y, button, action, source);
        }
        return 1;
    }

    /**
     * \brief Event translation for stylus events
     * \details used by ImGui_ImplAndroidExt_HandleEventUserCallback()
     * \param[in] app the android app
     * \param[in] event the android event
     */
    int32_t HandleEventUserCallback_stylus(
        struct android_app* app, AInputEvent* event
    ) {
        float x = AMotionEvent_getX(event, 0);
        float y = AMotionEvent_getY(event, 0);
        int32_t action = AMotionEvent_getAction(event);
        // No button mapped to button 0, all other buttons mapped to button 1
        int btn = int(
            (AMotionEvent_getButtonState(event) &
             AMOTION_EVENT_BUTTON_STYLUS_PRIMARY) != 0
        );
	g_mouse_CB(x, y, btn, decode_action(action), EVENT_SOURCE_STYLUS);
        return 1;
    }

    /**
     * \brief Event translation for mouse events
     * \details used by ImGui_ImplAndroidExt_HandleEventUserCallback().
     *   Needs to be called for both MOTION and KEY events, because right
     *   mouse click is a KEY event.
     * \param[in] app the android app
     * \param[in] event the android event
     */
    int32_t HandleEventUserCallback_mouse(
        struct android_app* app, AInputEvent* event
    ) {
        // Initially declared as static global so that key handler
        // can 'push' button 1 when the back key event is synthetized
        // by a right mouse click (but in fact does not work like that)
        // TODO (clean): remove it, not needed in fact.
        static int mouse_handler_btn = -1;
        
        // Right mouse button is a KEY rather than a MOUSE BUTTON,
        // hence, to properly handle events, we need to keep track
        // of its state, in order to be able to generate DRAG events
        // (because the mouse only sees a HOVER event).
        static bool right_mouse_btn_pressed = false;

        // TODO: cleaner version, with something like:
        // static bool mouse_button_pressed[3] = { false, false, false };
        // - mouse button events and right mouse button key event update it
        //   (and call callback whenever it changes)
        // - hover/move events call DRAG callback according to mouse button status
        
        if(
            AInputEvent_getType(event) == AINPUT_EVENT_TYPE_MOTION &&
            AMotionEvent_getToolType(event,0) == AMOTION_EVENT_TOOL_TYPE_MOUSE
        ) {
            float x = AMotionEvent_getX(event, 0);
            float y = AMotionEvent_getY(event, 0);

            int32_t action = AMotionEvent_getAction(event);
        
            if(action == AMOTION_EVENT_ACTION_SCROLL) {
                
                float hscroll = AMotionEvent_getAxisValue(
                    event, AMOTION_EVENT_AXIS_HSCROLL, 0
                );
            
                float vscroll = AMotionEvent_getAxisValue(
                    event, AMOTION_EVENT_AXIS_VSCROLL, 0
                );
            
                // Synthetize btn 2 push, move, btn 2 release
                g_mouse_CB(
                    x, y, 2,
                    EVENT_ACTION_DOWN, EVENT_SOURCE_MOUSE
                );
                g_mouse_CB(
                    x + hscroll, y - 10.0f * vscroll, 2,
                    EVENT_ACTION_DRAG, EVENT_SOURCE_MOUSE
                );	    
                g_mouse_CB(
                    x + hscroll, y - 10.0f * vscroll, 2,
                    EVENT_ACTION_UP, EVENT_SOURCE_MOUSE
                );	    
            } if(action == AMOTION_EVENT_ACTION_HOVER_MOVE) {
                if(right_mouse_btn_pressed) {
                    g_mouse_CB(
                        x, y, 1,
                        EVENT_ACTION_DRAG, EVENT_SOURCE_MOUSE
                    );
                }
            } else {
                if(
                    action == AMOTION_EVENT_ACTION_BUTTON_PRESS ||
                    action ==  AMOTION_EVENT_ACTION_DOWN
                ) {
                    int32_t buttons = AMotionEvent_getButtonState(event);
                    if((buttons&AMOTION_EVENT_BUTTON_PRIMARY) != 0) {
                        mouse_handler_btn = 0;
                    } else if(((buttons & AMOTION_EVENT_BUTTON_SECONDARY)!=0)) {
                        mouse_handler_btn = 1;
                    } else if(((buttons & AMOTION_EVENT_BUTTON_TERTIARY) !=0)) {
                        mouse_handler_btn = 2;
                    }
                }
                if(decode_action(action) != EVENT_ACTION_UNKNOWN) {
                    g_mouse_CB(
                        x, y, mouse_handler_btn,
                        decode_action(action), EVENT_SOURCE_MOUSE
                    );
                }
            }
        }

        // Right mouse button handler (yes, it is a KEY !)
        // It is because in Android, right mouse button is supposed
        // to behave like the BACK key.
        if(AInputEvent_getType(event) == AINPUT_EVENT_TYPE_KEY) {
            ImGuiIO& io = ImGui::GetIO();
            int32_t action = AKeyEvent_getAction(event);
            int32_t key = AKeyEvent_getKeyCode(event);
            
            if(
                action == AKEY_EVENT_ACTION_UP &&
                key == AKEYCODE_BACK &&
                AInputEvent_getSource(event) == AINPUT_SOURCE_MOUSE 
            ) {
                mouse_handler_btn = -1;
                g_mouse_CB(
                    io.MousePos.x, io.MousePos.y, 1,
                    EVENT_ACTION_UP, EVENT_SOURCE_MOUSE
                );
                right_mouse_btn_pressed = false;
            }
            
            if(action == AKEY_EVENT_ACTION_DOWN && key == AKEYCODE_BACK) {
                if(AInputEvent_getSource(event) != AINPUT_SOURCE_MOUSE) {
                    ::GEO::AndroidUtils::debug_log("Back softkey pushed");
                    // If real back button, quit application
                    // (normally, returning 0 should do the same, but
                    //  it does seem to work, to be understood...).
                    if(Application::instance() != nullptr) {
                        ::GEO::AndroidUtils::debug_log("Exiting application");
                        Application::instance()->stop();
                    }
                } else {
                    mouse_handler_btn = 1;
                    // Since right mouse button is a KEY, when it is
                    // pressed, it repeatedly generate key pressed
                    // events, so we just capture the first one here.
                    if(!right_mouse_btn_pressed) {
                        g_mouse_CB(
                            io.MousePos.x, io.MousePos.y, 1,
                            EVENT_ACTION_DOWN, EVENT_SOURCE_MOUSE
                        );
                        right_mouse_btn_pressed = true;
                    }
                }
            }
        }
        return 1;
    }
}

/**************************************************************************/

int32_t ImGui_ImplAndroidExt_HandleEventUserCallback(
    struct android_app* app, AInputEvent* event
) {

    if(g_mouse_CB == nullptr) {
        return 0;
    }

    int32_t type = AInputEvent_getType(event);
    int32_t tool = AMotionEvent_getToolType(event,0);
    
    if(type==AINPUT_EVENT_TYPE_MOTION && tool==AMOTION_EVENT_TOOL_TYPE_FINGER) {
        return HandleEventUserCallback_fingers(app, event);
    }

    if(type==AINPUT_EVENT_TYPE_MOTION && tool==AMOTION_EVENT_TOOL_TYPE_STYLUS) {
        return HandleEventUserCallback_stylus(app, event);
    }

    if(
        (type==AINPUT_EVENT_TYPE_MOTION&&tool==AMOTION_EVENT_TOOL_TYPE_MOUSE) ||
        AInputEvent_getType(event) == AINPUT_EVENT_TYPE_KEY
    ) {
        return HandleEventUserCallback_mouse(app, event);
    }
    
    return 1;
}

void ImGui_ImplAndroidExt_SetMouseUserCallback(
    ImGui_ImplAndroidExt_MouseUserCallback CB
) {
    g_mouse_CB = CB;
}

#endif
