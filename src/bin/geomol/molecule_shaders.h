
#include "TBO.h"
#include <geogram_gfx/basic/GLSL.h>


namespace {

    void register_geomol_shader_utilities() {

#ifdef GEO_TBO_SUPPORTED
	GEO::GLSL::register_GLSL_include_file("geomol/buffers.h",
          R"(
          #define glup_Buffer  samplerBuffer
          #define glup_iBuffer isamplerBuffer
          #define glup_uBuffer usamplerBuffer
          #define glup_fetch(B,idx) texelFetch(B,int(idx))
          )"
        );
#else
	GEO::GLSL::register_GLSL_include_file("geomol/buffers.h",
          R"(
          #define glup_Buffer  sampler2D
          #define glup_iBuffer isampler2D
          #define glup_uBuffer usampler2D
          #define glup_fetch(B,idx) \
             texelFetch(B,ivec2( \
                int(idx)%textureSize(B,0).x, \
                int(idx)/textureSize(B,0).x  \
             ),0)
          )"
        );
#endif

	GEO::GLSL::register_GLSL_include_file("geomol/clipping.h",
        R"(
        uniform glup_uBuffer facet_ptr_TBO;
        uniform glup_Buffer facet_plane_TBO;
        uniform bool show_imposters;
        uniform int max_clip_planes;

        // \brief Tests whether a point is inside a cell of the mixed complex
        // \details max_clip_planes (a uniform) determines the maximum number
        //   of tested clipping planes. For optimization purposes, clipping
        //   can be complitely deactivated for S0 cells, and limited to the
        //   first two clipping planes for H1 cells.
        // \param[in] p the world-space coordinates of the point
        // \param[in] cell_id the index of the cell to be displayed (typically
        //   gl_vertexID divided by number of vertices per imposter)
        // \retval true if the point is inside the convex cell
        // \retval false otherwise
        bool in_cell(in vec3 p, in int cell_id) {
           if(max_clip_planes == 0) {
              return true;
           }
           int c = 0;
           uint b = glup_fetch(facet_ptr_TBO, cell_id).x;
           uint e = glup_fetch(facet_ptr_TBO, cell_id+1).x;
           for(uint k=b; k<e; ++k,++c) {
              if(c >= max_clip_planes) {
                 return true;
              }
              vec4 P = glup_fetch(facet_plane_TBO, k);
              if(dot(vec4(p,1.0),P) > 0.0) {
                 return false;
              }
           }
           return true;
        }

        // \brief Tests whether a point is clipped-out by the user clipping plane
        // \param[in] M world-space coordinates of the point
        // \retval true if clipping is active and the point is clipped-out
        // \retval false otherwise
        bool clipped(in vec3 M) {
           if(glupIsEnabled(GLUP_CLIPPING)) {
              if(dot(vec4(M,1.0),GLUP.world_clip_plane) < 0.0) {
                 return true;
              }
           }
           return false;
        }

        // constant-width edges, using fwidth()
        float voro_edge_factor_adaptive(in float bary) {
           float d = fwidth(bary);
           // special fix for NVidia boards that do not
           // evaluate fwidth() well near patch boundaries!
           if(d == 0.0) {
              return 0.0;
           }
           return smoothstep(0.0, d*GLUP.mesh_width, bary);
        }

        // Smoothed minimum using cubic polynomial
        // See https://iquilezles.org/articles/smin/
        float smin( float a, float b, float k) {
            k *= 6.0;
            float h = max( k-abs(a-b), 0.0 )/k;
            return min( a, b ) - h*h*h*k*(1.0/6.0);
        }

        // Compute minimum distance to all clipping planes
        // \param[in] cell_id index of the cell (to fetch clipping planes)
        // \param[in] M world-space coordinates of intersection point
        // \return smoothed minimum distance to cell's clipping planes
        float min_dist(in int cell_id, in vec3 M) {
           float result = 1e6;
           uint b = glup_fetch(facet_ptr_TBO, cell_id).x;
           uint e = glup_fetch(facet_ptr_TBO, cell_id+1).x;
           for(uint k=b; k<e; ++k) {
              vec4 P = glup_fetch(facet_plane_TBO, k);
              float this_dist = abs(dot(M,P.xyz)+P.w)/length(P.xyz);
              result = smin(result, this_dist, GLUP.mesh_width/1000.0);
           }
           if(glupIsEnabled(GLUP_CLIPPING)) {
              vec4 P = GLUP.world_clip_plane;
              float this_dist = abs(dot(M,P.xyz)+P.w)/length(P.xyz);
              result = smin(result, this_dist, GLUP.mesh_width/1000.0);
           }
           return result;
        }

        // super-imposes the mesh if GLUP_DRAW_MESH is active
        // \param[in] color input color (from shaded surface)
        // \param[in] cell_id index of the cell (to fetch clipping planes)
        // \param[in] M world-space coordinates of intersection point
        // \return input color modified by mesh color if near to clip planes
        vec4 show_mesh(in vec4 color, in int cell_id, in vec3 M) {
           if(!glupIsEnabled(GLUP_DRAW_MESH)) {
              return color;
           }
           float d = min_dist(cell_id, M);
           return mix(GLUP.mesh_color, color, voro_edge_factor_adaptive(d));
        }

        // draw imposter instead of raytracing quadric
        // \param[in] C center of the quadric
        void draw_imposter(in vec3 C) {
           if(glupIsEnabled(GLUP_CLIPPING)) {
              // x,y coordinates of current fragment
              vec2 frag_xy_c = 2.0*gl_FragCoord.xy / vec2(
                   GLUP.viewport[2], GLUP.viewport[3]
              ) + vec2(-1.0,-1.0);
              // copy z from quadric's center, transformed in clip space
              vec4 C_c = GLUP.modelviewprojection_matrix*vec4(C,1.0);
              vec4 frag_c = vec4(frag_xy_c, C_c.z/C_c.w, 1.0);
              if(dot(frag_c,GLUP.clip_clip_plane) < 0.0) {
                 discard;
              }
           }
           glup_FragColor = GLUP.front_color;
           glup_update_depth(C);
        }

        )"
	);

	GEO::GLSL::register_GLSL_include_file("geomol/imposter.h",
	R"(

        vec4 row(in mat4 M, in int i) {
           return vec4(M[0][i], M[1][i], M[2][i], M[3][i]);
        }

        // Correction factor on both axes for non-square windows
        // Obtained by computing how a normal vector is transformed
        // from window space to normalized clip space, that is, by the
        // *transpose* normalized clip space to window space matrix.
        vec2 viewport_scaling() {
            float W = GLUP.viewport[2];
            float H = GLUP.viewport[3];
            return vec2(2.0*H/(W+H),2.0*W/(W+H));
        }

        // clip-space radius of a circle covering the projection
        // of a world-space sphere.
        // Dmitry Sokolov tutorial: https://github.com/ssloy/glsltutorial
        // ref: http://reality.cs.ucl.ac.uk/projects/quadrics/pbg06.pdf
        //
        // center_w: center of the sphere in world coordinates
        // R_w: radius of the sphere in world coordinates
        // center_c: center of the projected sphere in clip coordinates
        // R_c: radius of a disk centered on center_c that covers the projection
        //      of the sphere.
        void project_sphere(
            in vec3 center_w, in float R_w, out vec2 center_c, out float R_c
        ) {
            // TODO: optimize: directly compute r1,r2,r4
            // (but the GLSL compiler probably does optimize already)
            mat4 T = mat4(
               1.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0,
               center_w.x/R_w, center_w.y/R_w, center_w.z/R_w, 1.0/R_w
            );

            mat4 PMT = GLUP.modelviewprojection_matrix * T;
            vec4 r1 = row(PMT,0);
            vec4 r2 = row(PMT,1);
            vec4 r4 = row(PMT,3);

            float r1Dr4T = dot(r1.xyz,r4.xyz)-r1.w*r4.w;
            float r1Dr1T = dot(r1.xyz,r1.xyz)-r1.w*r1.w;
            float r4Dr4T = dot(r4.xyz,r4.xyz)-r4.w*r4.w;
            float r2Dr2T = dot(r2.xyz,r2.xyz)-r2.w*r2.w;
            float r2Dr4T = dot(r2.xyz,r4.xyz)-r2.w*r4.w;

            center_c = (1.0/r4Dr4T)*vec2(r1Dr4T, r2Dr4T);

            float discriminant_x = abs(r1Dr4T*r1Dr4T-r4Dr4T*r1Dr1T);
            float discriminant_y = abs(r2Dr4T*r2Dr4T-r4Dr4T*r2Dr2T);

            R_c = -sqrt(max(discriminant_x,discriminant_y)) / r4Dr4T;
            //(note: r4Dr4T is (probably) negative hence the minus sign)
        }

        uniform bool full_imposters;

        // imposter offset in function of (vertex id modulo 6)
        const vec2 imposter_offset[6] = vec2[](
            vec2(-1.0, -1.0),
            vec2( 1.0, -1.0),
            vec2( 1.0,  1.0),
            vec2(-1.0, -1.0),
            vec2( 1.0,  1.0),
            vec2(-1.0,  1.0)
        );

        )"
	);
    }

   /********************************************************************/

    const char* spheres_source =
        R"(
        //primitive GLUP_TRIANGLES
	)"

        R"(
        //stage GL_VERTEX_SHADER
        //import <GLUP/current_profile/vertex_shader_preamble.h>
	//import <GLUP/stdglup.h>
	//import <GLUPGLSL/state.h>
	//import <GLUP/current_profile/toggles.h>
        //import <geomol/buffers.h>
        //import <geomol/imposter.h>

        uniform glup_Buffer cell_eqn_1_TBO;
        uniform glup_Buffer cell_imposter_1_TBO;
        uniform bool replace_imposter;

        glup_flat glup_out vec4 focus_R;
        glup_flat glup_out int cell_id;

	void main() {
            cell_id = gl_VertexID / 6;
            focus_R = glup_fetch(cell_eqn_1_TBO,cell_id);
            vec4 imp_focus_R = replace_imposter ?
                               glup_fetch(cell_imposter_1_TBO,cell_id) : focus_R;
            vec2 center_c;
            float R_c;
            project_sphere(imp_focus_R.xyz, imp_focus_R.w, center_c, R_c);

            vec2 offset = imposter_offset[gl_VertexID%6];
            if(full_imposters) {
               gl_Position=vec4(offset,0.0,1.0);
            } else {
               gl_Position=vec4(center_c+viewport_scaling()*R_c*offset,0.0,1.0);
            }
	}
	)"

        R"(
        //stage GL_FRAGMENT_SHADER
        //import <GLUP/current_profile/fragment_shader_preamble.h>
        //import <GLUP/stdglup.h>
        //import <GLUPGLSL/state.h>
        //import <GLUP/current_profile/toggles.h>
        //import <GLUP/current_profile/primitive.h>
        //import <GLUP/fragment_shader_utils.h>
        //import <GLUP/fragment_ray_tracing.h>
        //import <geomol/buffers.h>
        //import <geomol/clipping.h>

        glup_flat glup_in vec4 focus_R;
        glup_flat glup_in int cell_id;

	void main() {
           if(show_imposters) {
              draw_imposter(focus_R.xyz);
              return;
           }
           vec3 C = focus_R.xyz;
           float r = focus_R.w;
           float sign = 1.0;

           Ray R = glup_primary_ray();

	   // High-precision ray-sphere intersection
	   // See Ray Tracing Gems, Chapter 7,
	   // Precision Improvements for Ray-Sphere Intersection,
	   // E. Haines, J. Gunther, T. Akenine-Moller
           vec3  D = R.O-C;
           float a = dot(R.V,R.V);

	   float b_prime = -dot(D,R.V);
	   vec3  H = D + (b_prime/a) * R.V;
	   float delta = r*r - dot(H,H);
           if(delta < 0.0) {
               discard;
           }
	   // Original article: q = b_prime + sign(b_prime)*sqrt(a*delta)
	   // Don't know why they do that, here we know we want t1
           float sqrt_a_delta = sqrt(a*delta);
	   float q = b_prime - sqrt_a_delta;
	   float t = q/a;

           vec3 M = R.O + t*R.V;

           if(clipped(M) || !in_cell(M,cell_id)) {
              // Get the other root, still using the high-precision method
              q = b_prime + sqrt_a_delta;
              t = q/a;
              M = R.O + t*R.V;
              sign = sign * -1.0;
              if(clipped(M) || !in_cell(M,cell_id)) {
                  discard;
              }
           }

           glup_update_depth(M);
           vec4 result = GLUP.front_color;
           if(glupIsEnabled(GLUP_LIGHTING)) {
               vec3 N = sign*normalize(GLUP.normal_matrix*(M-C));
               result = glup_lighting(result, N);
           }
           result = show_mesh(result, cell_id, M);
           glup_FragColor = result;
	}
	)";

   /********************************************************************/

    const char* hyperboloids_source =
        R"(
        //primitive GLUP_TRIANGLES
	)"

        R"(
        //stage GL_VERTEX_SHADER
        //import <GLUP/current_profile/vertex_shader_preamble.h>
	//import <GLUP/stdglup.h>
	//import <GLUPGLSL/state.h>
	//import <GLUP/current_profile/toggles.h>
        //import <geomol/buffers.h>
        //import <geomol/imposter.h>

        uniform glup_Buffer cell_eqn_1_TBO;
        uniform glup_Buffer cell_eqn_2_TBO;
        uniform glup_Buffer cell_imposter_1_TBO;
        uniform glup_Buffer cell_imposter_2_TBO;

        glup_flat glup_out vec3  C;  // focus
        glup_flat glup_out vec3  n;  // axis
        glup_flat glup_out float R2;
        glup_flat glup_out int cell_id;

	void main() {
            cell_id = gl_VertexID / 6;
            vec4 focus_R = glup_fetch(cell_eqn_1_TBO,cell_id);
            C = focus_R.xyz;
            R2 = focus_R.w;
            n = glup_fetch(cell_eqn_2_TBO,cell_id).xyz;

            vec4 imp1 = glup_fetch(cell_imposter_1_TBO,cell_id);
            vec4 imp2 = glup_fetch(cell_imposter_2_TBO,cell_id);

            vec2 scaling = viewport_scaling();
            vec2 UV1; float R1_c;
            vec2 UV2; float R2_c;
            project_sphere(imp1.xyz, imp1.w, UV1, R1_c);
            project_sphere(imp2.xyz, imp2.w, UV2, R2_c);
            float qsize = max(R1_c,R2_c);

            vec2 H = normalize(UV2-UV1);
            vec2 Hperp = vec2(-H.y,H.x);
            vec2 offset = imposter_offset[gl_VertexID % 6];

            if(full_imposters) {
               gl_Position = vec4(offset,0.0,1.0);
            } else {
               vec2 origin = mix(UV1,UV2,0.5*(offset.x+1));
               offset = qsize*scaling*(mat2(H,Hperp)*offset);
               gl_Position = vec4(origin+offset,0.0,1.0);
            }
	}
	)"

        R"(
        //stage GL_FRAGMENT_SHADER
        //import <GLUP/current_profile/fragment_shader_preamble.h>
        //import <GLUP/stdglup.h>
        //import <GLUPGLSL/state.h>
        //import <GLUP/current_profile/toggles.h>
        //import <GLUP/current_profile/primitive.h>
        //import <GLUP/fragment_shader_utils.h>
        //import <GLUP/fragment_ray_tracing.h>
        //import <geomol/buffers.h>
        //import <geomol/clipping.h>

        uniform vec2 cAxisPerp;
        glup_flat glup_in vec3  C;  // focus
        glup_flat glup_in vec3  n;  // axis
        glup_flat glup_in float R2;
        glup_flat glup_in int cell_id;

	void main() {

            if(show_imposters) {
               draw_imposter(C);
               return;
            }

            float alpha = cAxisPerp.x;
            float beta  = cAxisPerp.y;

            Ray R = glup_primary_ray();

            // Advance R's origin to projection of C onto R, to avoid
            // numeric precision errors (a bit like in sphere shader)
            R.V = normalize(R.V);
            R.O += dot(C-R.O,R.V)*R.V;

            vec3 co = R.O-C;
            float coDn = dot(co,n);
            float vDn  = dot(R.V,n);

            // my old version using length of cross product for YZ axis
            /*
            vec3 coXn = cross(co,n);
            vec3 vXn  = cross(R.V,n);
            float a =      alpha*vDn*vDn   + beta*dot(vXn,vXn)  ;
            float b = 2.0*(alpha*coDn*vDn  + beta*dot(coXn,vXn));
            float c =      alpha*coDn*coDn + beta*dot(coXn,coXn) + R2;
            */

            // This version from Matthieu's shader, more efficient I think
            // Probably computes YZ components by subtracting X component
            // (to be understood)
            // Initially there was dot(R.V,R.V) here but R.V is unit now
                                                   //       |
            float gamma = alpha - beta;            //       v
            float a =      gamma*vDn*vDn   + beta; // *dot(R.V,R.V);
            float b = 2.0*(gamma*coDn*vDn  + beta*dot(R.V, co));
            float c =      gamma*coDn*coDn + beta*dot(co,co) + R2;

            float delta = b*b - 4.0*a*c;
            if(delta < 0.0) {
               discard;
            }
            float sq = sqrt(delta);
            float t1 = (-b - sq) / (2.0*a);
            float t2 = (-b + sq) / (2.0*a);
            float sign = 1.0;
            float t = min(t1,t2);
            vec3 M = R.O + t * R.V;
            if(t1 > t2) { sign = -1.0; }

            if(clipped(M) || !in_cell(M,cell_id)) {
                t = max(t1,t2);
                M = R.O + t * R.V;
                sign = sign * -1.0;
                if(clipped(M) || !in_cell(M,cell_id)) {
                   discard;
                }
            }

            glup_update_depth(M);
            vec4 result = GLUP.front_color;
            if(glupIsEnabled(GLUP_LIGHTING)) {
               vec3 CM = M-C;
               vec3 CMperp = dot(CM,n)*n;
               vec3 CMpar  = CM - CMperp;
               vec3 N = alpha * CMperp + beta * CMpar;
               N = sign*normalize(GLUP.normal_matrix*N);
               result = glup_lighting(result, N);
            }

            result = show_mesh(result, cell_id, M);
            glup_FragColor = result;
        }
	)";
}
