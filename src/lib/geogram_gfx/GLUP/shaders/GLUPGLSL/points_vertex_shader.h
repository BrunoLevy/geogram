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
    float depth_radius;
} VertexOut;

void main() {
#ifndef GLUP_NO_GL_CLIPPING
    if(glupIsEnabled(GLUP_CLIPPING)) {
        gl_ClipDistance[0] = dot(
            vertex_in, GLUP.world_clip_plane
        );
    } else {
        gl_ClipDistance[0] = 0.0;
    }
#endif
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
    gl_PointSize = GLUP.point_size;
    gl_Position = GLUP.modelviewprojection_matrix*vertex_in;



    // Compute depth radius, that is, maximum difference of depth,
    // at the center of the displayed GL_POINT
    // Note: GLUP.viewport = [x0, y0, width, height]

    // I still do not understand what's going on:
    //   - I do not multiply with GLUP.point_size (but I think we should)
    //   - factor is 0.1, should be 0.5 (but I still get artifacts with 0.5)
    float pointsize_clip_space = 1.0 / GLUP.viewport[2];
    VertexOut.depth_radius =
	0.1 * pointsize_clip_space * (gl_DepthRange.far - gl_DepthRange.near);
}
