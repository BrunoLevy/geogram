//import <GLUP/current_profile/vertex_shader_preamble.h>
//import <GLUP/stdglup.h>
//import <GLUP/current_profile/toggles.h>
//import <GLUP/current_profile/primitive.h>
//import <GLUPES/vertex_shader_state.h>

   glup_in vec4 vertex_in;                                 
   glup_in vec4 color_in;                                  
   glup_in vec4 tex_coord_in;
   glup_in vec4 normal_in;
   glup_in highp float vertex_id_in;                       
   glup_out float clip_dist;                               
   glup_out vec4 color;                                    
   glup_out vec4 tex_coord;
   glup_flat glup_out glup_id primitive_id;
   glup_out float R;
   glup_out vec2 p1_ndc;
   glup_out vec2 p2_ndc;


void emit_vertex_2(in vec4 p_world, in vec4 p_clip_space, in vec2 offset) {
    if(glupIsEnabled(GLUP_CLIPPING)) {
        clip_dist = dot(                                
            p_world, GLUP_VS.world_clip_plane         
        );          
    }
    gl_Position = p_clip_space / p_clip_space.w ;
    gl_Position.x += offset.x;
    gl_Position.y += offset.y;
    gl_Position.z -= 0.001; // TODO: polygon offset, do something smarter
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
}


void main() {                                         

    R = 2.0*(GLUP_VS.mesh_width/(GLUP_VS.viewport[2]+GLUP_VS.viewport[3]));

    vec4 p1_clipspace = GLUP_VS.modelviewprojection_matrix * vertex_in;
    vec4 p2_clipspace = GLUP_VS.modelviewprojection_matrix * normal_in;
    
    p1_ndc = p1_clipspace.xy / p1_clipspace.w;
    p2_ndc = p2_clipspace.xy / p2_clipspace.w;
    
    if(glupIsEnabled(GLUP_PICKING)) {
#ifdef GLUP_ES_100
        // Note: we need to add 0.5, else there are some precision
        // issues, and the integer mod() operation creates random
        // values...
        primitive_id = float(
            int(vertex_id_in+0.5)/glup_primitive_nb_vertices
        )+0.5;
#else                                                   
        primitive_id = int(vertex_id_in + 0.5)/glup_primitive_nb_vertices; 
#endif                                                  
    }                                                    
        
    vec2 U = R*normalize(p2_ndc-p1_ndc);
    vec2 V = vec2(U.y,-U.x);

    int v_local_id = glup_mod(int(vertex_id_in+0.5),4);

    if(v_local_id == 0) {
        emit_vertex_2(vertex_in, p1_clipspace,-U-V);
    } else if(v_local_id == 1) {
        emit_vertex_2(vertex_in, p1_clipspace,-U+V);
    } else if(v_local_id == 2) {
        emit_vertex_2(normal_in, p2_clipspace, U-V);
    } else {
        emit_vertex_2(normal_in, p2_clipspace, U+V);
    }

}                                                     
