//import <GLUP/current_profile/geometry_shader_preamble.h> 
//import <GLUPGLSL/state.h>
//import <GLUP/stdglup.h>
//import <GLUP/current_profile/toggles.h>
//import <GLUP/current_profile/primitive.h>

// Send the two extremities of the segment and the radius to
// the fragment shader (used to create nicer joints)
out vec2  p1_ndc;
out vec2  p2_ndc;
out float R;

void emit_vertex_2(in int i, in vec2 offset, in bool do_clip) {
#ifndef GLUP_NO_GL_CLIPPING        
    if(glupIsEnabled(GLUP_CLIPPING)) {
        gl_ClipDistance[0] =
            clip_distance(vertex_clip_space_in(i),do_clip);
    }
#endif    
    gl_Position = vertex_clip_space_in(i) / vertex_clip_space_in(i).w ;
    gl_Position.x += offset.x;
    gl_Position.y += offset.y;
    gl_Position.z -= 0.001; // TODO: polygon offset, do something smarter
    VertexOut.vertex_clip_space = gl_Position;
    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
        VertexOut.color = color_in(i);                             
    }                                                               
    if(glupIsEnabled(GLUP_TEXTURING)) {
        VertexOut.tex_coord = tex_coord_in(i);                     
    }
    EmitVertex();                                                   
}

void main() {
    gl_PrimitiveID = gl_PrimitiveIDIn;

    R = 2.0*(GLUP.mesh_width/(GLUP.viewport[2]+GLUP.viewport[3]));
    p1_ndc = vertex_clip_space_in(0).xy / vertex_clip_space_in(0).w;
    p2_ndc = vertex_clip_space_in(1).xy / vertex_clip_space_in(1).w;
    vec2 U = R*normalize(p2_ndc-p1_ndc);
    vec2 V = vec2(U.y,-U.x);
    emit_vertex_2(0,-U-V,true);
    emit_vertex_2(0,-U+V,true);
    emit_vertex_2(1, U-V,true);    
    emit_vertex_2(1, U+V,true);
    EndPrimitive();
}

