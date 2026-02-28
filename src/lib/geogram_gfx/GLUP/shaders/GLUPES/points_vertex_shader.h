//import <GLUP/current_profile/vertex_shader_preamble.h>
//import <GLUPES/vertex_shader_state.h>
//import <GLUP/stdglup.h>
//import <GLUP/current_profile/toggles.h>
//import <GLUP/current_profile/primitive.h>

glup_in vec4 vertex_in;
glup_in vec4 color_in;
glup_in vec4 tex_coord_in;
glup_in highp float vertex_id_in;

glup_flat glup_out vec4 color;
glup_flat glup_out vec4 tex_coord;
glup_flat glup_out vec3 center_world_space;
glup_flat glup_out float radius;
glup_flat glup_out glup_id primitive_id;


void main(void) {

    if(glupIsEnabled(GLUP_PICKING)) {
#ifdef GLUP_ES_100
        primitive_id = float(int(vertex_id_in + 0.5)) + 0.5;
#else
        primitive_id = int(vertex_id_in + 0.5);
#endif
    }

    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
        color = color_in;
    }

    if(glupIsEnabled(GLUP_TEXTURING)) {
        if(glupIsEnabled(GLUP_INDIRECT_TEXTURING)) {
            tex_coord = tex_coord_in;
        } else {
            tex_coord = GLUP_VS.texture_matrix * tex_coord_in;
        }
    }

    gl_Position = GLUP_VS.modelviewprojection_matrix*vertex_in;

    center_world_space = vertex_in.xyz / vertex_in.w;

    vec4 P1 = GLUP_VS.inverse_modelviewprojection_matrix*vec4(
	0.0, 0.0, 0.0, 1.0
    );
    vec4 P2 = GLUP_VS.inverse_modelviewprojection_matrix*vec4(
	GLUP_VS.point_size/GLUP_VS.viewport[2],0.0,0.0,1.0
    );
    radius = length(P1.xyz/P1.w-P2.xyz/P2.w);
}
