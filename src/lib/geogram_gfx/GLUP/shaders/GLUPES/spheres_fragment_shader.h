//import <GLUP/current_profile/fragment_shader_preamble.h>
//import <GLUPES/fragment_shader_state.h>
//import <GLUP/stdglup.h>
//import <GLUP/current_profile/toggles.h>
//import <GLUP/current_profile/primitive.h>
//import <GLUP/fragment_shader_utils.h>
//import <GLUP/fragment_ray_tracing.h>

glup_flat glup_in vec4 color;
glup_flat glup_in vec4 tex_coord;
glup_flat glup_in vec3 center_world_space;
glup_flat glup_in float radius;
glup_flat glup_in glup_id primitive_id;

void main(void) {
    vec3 C = center_world_space;
    float r = radius;

    if(glupIsEnabled(GLUP_CLIPPING)) {
        if(GLUP.clipping_mode == GLUP_CLIP_WHOLE_CELLS) {
            if(dot(vec4(center_world_space,1.0),GLUP.world_clip_plane) < 0.0) {
                discard;
            }
        } else if(GLUP.clipping_mode == GLUP_CLIP_STRADDLING_CELLS) {
            float dist =
                abs(dot(vec4(center_world_space,1.0),GLUP.world_clip_plane)) /
                length(GLUP.world_clip_plane.xyz) ;
            if(dist > r) {
                discard;
            }
        }
    }

    Ray R = glup_primary_ray();
    vec3 M,N;

    if(
        glupIsEnabled(GLUP_CLIPPING) &&
        GLUP.clipping_mode == GLUP_CLIP_SLICE_CELLS
    ) {
        N = GLUP.world_clip_plane.xyz;
        float w = GLUP.world_clip_plane.w;
        float t = -(w + dot(N,R.O)) / dot(N,R.V);
        M = R.O + t*R.V;
        if(dot(M-C,M-C) > r*r) {
            discard;
        }
    } else {
        vec3 D = R.O-C;
        float a = dot(R.V,R.V);
        float b = 2.0*dot(R.V,D);
        float c = dot(D,D)-r*r;
        float delta = b*b-4.0*a*c;

        if(delta < 0.0) {
            discard;
        }
        float t = (-b-sqrt(delta))/(2.0*a);
        M = R.O + t*R.V;
        N = M-C;
    }

    if(
        glupIsEnabled(GLUP_CLIPPING) &&
        GLUP.clipping_mode == GLUP_CLIP_STANDARD
    ) {
        if(dot(vec4(M,1.0),GLUP.world_clip_plane) < 0.0) {
            discard;
        }
    }

    glup_update_depth(M);

    if(glupIsEnabled(GLUP_PICKING)) {
        glup_FragColor = glup_picking(int(primitive_id));
        return;
    }

    vec4 result;
    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
        result = color;
    } else {
        result = GLUP.front_color;
    }
    if(glupIsEnabled(GLUP_TEXTURING)) {
        result = glup_texturing(result, tex_coord);
    }
    if(glupIsEnabled(GLUP_LIGHTING)) {
        N = normalize(GLUP.normal_matrix*N);
        if(
            glupIsEnabled(GLUP_CLIPPING) &&
            GLUP.clipping_mode == GLUP_CLIP_SLICE_CELLS &&
            N.z < 0.0
        ) {
            N = -N;
        }
        result = glup_lighting(result, N);
    }
    glup_FragColor = result;
    glup_alpha_discard();
}
