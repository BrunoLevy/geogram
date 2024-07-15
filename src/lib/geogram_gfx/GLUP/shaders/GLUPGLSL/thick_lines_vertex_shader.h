//import <GLUP/current_profile/vertex_shader_preamble.h>
//import <GLUPGLSL/state.h>
//import <GLUP/stdglup.h>
//import <GLUP/current_profile/toggles.h>
//import <GLUP/current_profile/primitive.h>

in vec4 vertex_in;
in vec4 color_in;
in vec4 tex_coord_in;

out VertexData {
#ifdef GLUP_GL_ES
      vec4 vertex_clip_space;
#endif
    vec4 color;
    vec4 tex_coord;
} VertexOut;

void main(void) {
    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
        VertexOut.color = color_in;
    }
    if(glupIsEnabled(GLUP_TEXTURING)) {
        if(glupIsEnabled(GLUP_INDIRECT_TEXTURING)) {
            VertexOut.tex_coord = tex_coord_in;
        } else {
            VertexOut.tex_coord =
                GLUP.texture_matrix * tex_coord_in;
        }
    }
    gl_Position = GLUP.modelviewprojection_matrix * vertex_in;
}
