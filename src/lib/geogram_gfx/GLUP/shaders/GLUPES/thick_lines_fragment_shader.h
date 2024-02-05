//import <GLUP/current_profile/fragment_shader_preamble.h>
//import <GLUPES/fragment_shader_state.h>
//import <GLUP/stdglup.h>
//import <GLUP/current_profile/toggles.h>
//import <GLUP/current_profile/primitive.h>
//import <GLUPES/fragment_shader_utils.h>

   glup_in float clip_dist;                                 
   glup_in vec4 color;                                     
   glup_in vec4 tex_coord;
   glup_flat glup_in glup_id primitive_id;
   glup_in float R;
   glup_in vec2 p1_ndc;
   glup_in vec2 p2_ndc;

void main() {

    if(glupIsEnabled(GLUP_CLIPPING)) {
        if(clip_dist < 0.0) {                                       
             discard;                               
        }                                         
    }

    // Create nicer joints between overlapping thick lines by creating a
    // small disk over the joints
    
    vec2 p_ndc = vec2(
        2.0 * ( (gl_FragCoord.x - GLUP.viewport[0]) / GLUP.viewport[2] - 0.5),
        2.0 * ( (gl_FragCoord.y - GLUP.viewport[1]) / GLUP.viewport[3] - 0.5)
    );

    vec2 U = p2_ndc - p1_ndc;
    vec2 V1 = p_ndc - p1_ndc;
    vec2 V2 = p_ndc - p2_ndc;

    if(dot(V1,U) < 0.0 && dot(V1,V1) > R*R) {
        discard;
    }
    
    if(dot(V2,U) > 0.0 && dot(V2,V2) > R*R) {
        discard;
    }
    
    if(glupIsEnabled(GLUP_PICKING)) {
        glup_FragColor = glup_picking(int(primitive_id));        
        return;
    }

    vec4 result;
    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
        result = color;
    } else {
        result = GLUP.mesh_color;
    }

    if(glupIsEnabled(GLUP_TEXTURING) && !glupIsEnabled(GLUP_NORMAL_MAPPING)) {
        result = glup_texturing(result, tex_coord);
    }
    glup_FragColor = result;
    glup_alpha_discard();
}                                                             
