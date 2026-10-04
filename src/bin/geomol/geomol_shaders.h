
namespace {

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
        glup_flat glup_out float cell_id;

	void main() {
	    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
	       color = color_in;
	    }
	    focus_R = tex_coord_in;
            cell_id = normal_in.w;
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
        glup_flat glup_in vec4 color;
        glup_flat glup_in vec4 focus_R;
        glup_flat glup_in float cell_id;

        uniform isamplerBuffer facet_ptr_TBO;
        uniform samplerBuffer facet_plane_TBO;

        bool in_cell(in vec3 p) {
           int i_cell_id = int(cell_id);
           int b = texelFetch(facet_ptr_TBO, i_cell_id).x;
           int e = texelFetch(facet_ptr_TBO, i_cell_id+1).x;
           for(int k=b; k<e; ++k) {
              vec4 P = texelFetch(facet_plane_TBO, k);
              if(dot(vec4(p,1.0),P) > 0.0) {
                 return false;
              }
           }
           return true;
        }

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
	   float q = b_prime - sqrt(a*delta);
	   float t = q/a;

           vec3 M = R.O + t*R.V;

           if(!in_cell(M)) {
              float c = dot(D,D)-r*r;
              t = c/q;
              M = R.O + t*R.V;
              sign = sign * -1;
              if(!in_cell(M)) {
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
        glup_flat glup_out float cell_id;
	void main() {
	    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
	       color = color_in;
	    }
            C = tex_coord_in.xyz;
            R2 = -tex_coord_in.w;
            n = normal_in.xyz;
            cell_id = normal_in.w;
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

        uniform vec2 cAxisPerp;
        glup_flat glup_in vec4 color;
        glup_flat glup_in vec3  C;  // focus
        glup_flat glup_in vec3  n;  // axis
        glup_flat glup_in float R2;
        glup_flat glup_in float cell_id;

        uniform isamplerBuffer facet_ptr_TBO;
        uniform samplerBuffer facet_plane_TBO;

        bool in_cell(in vec3 p) {
           int i_cell_id = int(cell_id);
           int b = texelFetch(facet_ptr_TBO, i_cell_id).x;
           int e = texelFetch(facet_ptr_TBO, i_cell_id+1).x;
           for(int k=b; k<e; ++k) {
              vec4 P = texelFetch(facet_plane_TBO, k);
              if(dot(vec4(p,1),P) > 0.0) {
                 return false;;
              }
           }
           return true;
        }

	void main() {
            float alpha = cAxisPerp.x;
            float beta  = cAxisPerp.y;

            Ray R = glup_primary_ray();
            vec3 co = R.O-C;

            vec3 coXn = cross(co,n);
            vec3 vXn  = cross(R.V,n);

            float coDn = dot(co,n);
            float vDn  = dot(R.V,n);

            float a =      beta*dot(vXn,vXn)   + alpha*vDn*vDn;
            float b = 2.0*(beta*dot(coXn,vXn)  + alpha*coDn*vDn);
            float c =      beta*dot(coXn,coXn) + alpha*coDn*coDn - R2;

/*
    // From Matthieu's shader, gives same result
    float u_cAxis = cAxisPerp.x;
    float u_cPerp = cAxisPerp.y;
    vec3  A = n;
    vec3  dv = R.O - C;
    float u0 = dot(dv, A);
    float da = dot(R.V, A);
    float cdiff = u_cAxis - u_cPerp;
    float a = cdiff*da*da + u_cPerp*dot(R.V, R.V);
    float b = 2.0*(cdiff*u0*da + u_cPerp*dot(dv, R.V));
    float c = cdiff*u0*u0 + u_cPerp*dot(dv, dv) - R2;
 */

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
            if(t1 > t2) { sign = -1; }

            if(!in_cell(M)) {
                t = max(t1,t2);
                M = R.O + t * R.V;
                sign = sign * -1.0;
                if(!in_cell(M)) {
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
