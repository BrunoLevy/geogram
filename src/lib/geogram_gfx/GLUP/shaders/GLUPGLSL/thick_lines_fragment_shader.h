//import <GLUP/current_profile/fragment_shader_preamble.h>
//import <GLUPGLSL/state.h>
//import <GLUP/stdglup.h>
//import <GLUP/current_profile/toggles.h>
//import <GLUP/current_profile/primitive.h>
//import <GLUP/fragment_shader_utils.h>

#ifndef GLUP_NO_GL_CLIPPING
in float gl_ClipDistance[];
#endif

in VertexData {
    vec4 vertex_clip_space;
    vec4 color;
    vec4 tex_coord;
    vec4 mesh_tex_coord;
} FragmentIn;

in vec2  p1_ndc;
in vec2  p2_ndc;
in float R;


void main() {

#ifdef GLUP_GL_ES
#ifndef GLUP_NO_GL_CLIPPING
    if(glupIsEnabled(GLUP_CLIPPING) && (gl_ClipDistance[0] < 0.0)) {
        discard;
    }
#endif
#endif

    // Create nicer joints between overlapping thick lines by creating a
    // small disk over the joints

    vec2 p_ndc = vec2(
        2.0 * ( (gl_FragCoord.x - GLUP.viewport[0]) / GLUP.viewport[2] - 0.5),
        2.0 * ( (gl_FragCoord.y - GLUP.viewport[1]) / GLUP.viewport[3] - 0.5)
    );

    vec2 U = p2_ndc - p1_ndc;
    vec2 V1 = p_ndc - p1_ndc;
    vec2 V2 = p_ndc - p2_ndc;

    if(dot(V1,U) < 0 && dot(V1,V1) > R*R) {
        discard;
    }

    if(dot(V2,U) > 0 && dot(V2,V2) > R*R) {
        discard;
    }

    if(glupIsEnabled(GLUP_PRIMITIVE_FILTERING)) {
        glup_primitive_filter(gl_PrimitiveID);
    }

    if(glupIsEnabled(GLUP_PICKING)) {
        glup_FragColor = glup_picking(gl_PrimitiveID);
        return;
    }

    vec4 result;
    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
        result = FragmentIn.color;
    } else {
        result = GLUP.mesh_color;
    }

    if(glupIsEnabled(GLUP_TEXTURING) && !glupIsEnabled(GLUP_NORMAL_MAPPING)) {
        result = glup_texturing(result, FragmentIn.tex_coord);
    }
    glup_FragColor = result;
    glup_alpha_discard();
}
