//import <GLUP/current_profile/vertex_shader_preamble.h>
//import <GLUPGLSL/state.h>
//import <GLUP/stdglup.h>
//import <GLUP/current_profile/toggles.h>
//import <GLUP/current_profile/primitive.h>

in vec4 vertex_in;
in vec4 color_in;
in vec4 tex_coord_in;

out VertexData {
    vec4 color;
    vec4 tex_coord;
    vec3 center_world_space;
    float radius;
} VertexOut;

void main(void) {

    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
        VertexOut.color = color_in;
    }

    if(glupIsEnabled(GLUP_TEXTURING)) {
        if(glupIsEnabled(GLUP_INDIRECT_TEXTURING)) {
            VertexOut.tex_coord = tex_coord_in;
        } else {
            VertexOut.tex_coord = GLUP.texture_matrix * tex_coord_in;
        }
    }

    gl_Position = GLUP.modelviewprojection_matrix*vertex_in;

    VertexOut.center_world_space = vertex_in.xyz / vertex_in.w;
    vec4 P1 = GLUP.inverse_modelviewprojection_matrix*vec4(0.0, 0.0, 0.0, 1.0);
    vec4 P2 = GLUP.inverse_modelviewprojection_matrix*vec4(
	GLUP.point_size/GLUP.viewport[2],0.0,0.0,1.0
    );
    VertexOut.radius = length(P1.xyz/P1.w-P2.xyz/P2.w);
}
