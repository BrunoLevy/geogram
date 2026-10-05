
#include <geogram_gfx/basic/GLSL.h>


namespace {

    void register_geomol_shader_utilities() {
	GEO::GLSL::register_GLSL_include_file("geomol/clipping.h",
        R"(
        uniform isamplerBuffer facet_ptr_TBO;
        uniform samplerBuffer facet_plane_TBO;

        bool in_cell(in vec3 p, in int cell_id) {
           int b = texelFetch(facet_ptr_TBO, cell_id).x;
           int e = texelFetch(facet_ptr_TBO, cell_id+1).x;
           for(int k=b; k<e; ++k) {
              vec4 P = texelFetch(facet_plane_TBO, k);
              if(dot(vec4(p,1.0),P) > 0.0) {
                 return false;
              }
           }
           return true;
        }
        )"
	);
    }

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

	glup_in vec4 vertex_in;
        glup_in vec4 color_in;
        glup_in vec4 tex_coord_in;
        glup_in vec4 normal_in;
        glup_flat glup_out vec4 color;
        glup_flat glup_out vec4 focus_R;
        glup_flat glup_out int cell_id;

	void main() {
	    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
	       color = color_in;
	    }
	    focus_R = tex_coord_in;
            cell_id = int(normal_in.w);
            gl_Position = GLUP.modelviewprojection_matrix * vertex_in;
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
        //import <geomol/clipping.h>

        glup_flat glup_in vec4 color;
        glup_flat glup_in vec4 focus_R;
        glup_flat glup_in int cell_id;

	void main() {
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

           if(!in_cell(M,cell_id)) {
              // Get the other root, still using the high-precision method
              q = b_prime + sqrt_a_delta;
              t = q/a;
              M = R.O + t*R.V;
              sign = sign * -1.0;
              if(!in_cell(M,cell_id)) {
                  discard;
              }
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
           vec4 result = GLUP.front_color;
           if(glupIsEnabled(GLUP_LIGHTING)) {
               vec3 N = sign*normalize(GLUP.normal_matrix*(M-C));
               result = glup_lighting(result, N);
           }
           glup_FragColor = result;
	}
	)";

   /***************************************************************************/

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

	glup_in vec4 vertex_in;
        glup_in vec4 color_in;
        glup_in vec4 tex_coord_in;
        glup_in vec4 normal_in;
        glup_flat glup_out vec4  color;
        glup_flat glup_out vec3  C;  // focus
        glup_flat glup_out vec3  n;  // axis
        glup_flat glup_out float R2;
        glup_flat glup_out int cell_id;
	void main() {
	    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
	       color = color_in;
	    }
            C = tex_coord_in.xyz;
            R2 = -tex_coord_in.w;
            n = normal_in.xyz;
            cell_id = int(normal_in.w);
            gl_Position = GLUP.modelviewprojection_matrix * vertex_in;
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
        //import <geomol/clipping.h>

        uniform vec2 cAxisPerp;
        glup_flat glup_in vec4 color;
        glup_flat glup_in vec3  C;  // focus
        glup_flat glup_in vec3  n;  // axis
        glup_flat glup_in float R2;
        glup_flat glup_in int cell_id;

	void main() {
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
            float c =      alpha*coDn*coDn + beta*dot(coXn,coXn) - R2;
            */

            // This version from Matthieu's shader, more efficient I think
            // Probably computes YZ components by subtracting X component
            // (to be understood)
            // Initially there was dot(R.V,R.V) here but R.V is unit now
                                                   //       |
            float gamma = alpha - beta;            //       v
            float a =      gamma*vDn*vDn   + beta; // *dot(R.V,R.V);
            float b = 2.0*(gamma*coDn*vDn  + beta*dot(R.V, co));
            float c =      gamma*coDn*coDn + beta*dot(co,co) - R2;

            float delta = b*b - 4.0*a*c;
            if(delta < 0.0) {
               discard;
            }
            float sq = sqrt(delta);
            float t1 = (-b - sq) / (2.0*a);
            float t2 = (-b + sq) / (2.0*a);
            float sign = 1;
            float t = min(t1,t2);
            vec3 M = R.O + t * R.V;
            if(t1 > t2) { sign = -1.0; }

            if(!in_cell(M,cell_id)) {
                t = max(t1,t2);
                M = R.O + t * R.V;
                sign = sign * -1.0;
                if(!in_cell(M,cell_id)) {
                   discard;
                }
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
            vec4 result = GLUP.front_color;
            if(glupIsEnabled(GLUP_LIGHTING)) {
               vec3 CM = M-C;
               vec3 CMperp = dot(CM,n)*n;
               vec3 CMpar  = CM - CMperp;
               vec3 N = alpha * CMperp + beta * CMpar;
               N = sign*normalize(GLUP.normal_matrix*N);
               result = glup_lighting(result, N);
            }
            glup_FragColor = result;
        }
	)";


   /********************************************************************/
}
