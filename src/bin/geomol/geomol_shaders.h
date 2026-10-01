
namespace {

    const char* GLUPES_spheres_source =
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
	void main() {
	    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
	       color = color_in;
	    }
	    focus_R = tex_coord_in;
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
	void main() {
           vec3 C = focus_R.xyz;
           float r = focus_R.w;
           float sign = 1.0;
           if(r < 0.0) {
              r = -r;
              sign = -1.0;
           }

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

           glup_update_depth(M);
           vec4 result = GLUP.front_color;
           if(glupIsEnabled(GLUP_LIGHTING)) {
               vec3 N = sign*normalize(GLUP.normal_matrix*(M-C));
               result = glup_lighting(result, N);
           }
           glup_FragColor = result;
	}
	)";

    const char* GLUPES_hyperboloids_source =
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
        glup_flat glup_out vec4  Pi1;
        glup_flat glup_out vec4  Pi2;
	void main() {
	    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
	       color = color_in;
	    }
            C = tex_coord_in.xyz;
            R2 = -tex_coord_in.w;
            n = normalize(normal_in.xyz);
            Pi1 = -normal_in;
            Pi2 = vec4(n,-dot(n,C+normal_in.xyz));
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
        glup_flat glup_in vec4  Pi1;
        glup_flat glup_in vec4  Pi2;

	void main() {
            float u = cAxisPerp.x;
            float v = cAxisPerp.y;
            float r = 0.25;
            float sign = 1.0;

            u = 0.0;
            v = 1.0;


            Ray R = glup_primary_ray();

           /*
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

           glup_update_depth(M);
           vec4 result = GLUP.front_color;
           if(glupIsEnabled(GLUP_LIGHTING)) {
               vec3 N = sign*normalize(GLUP.normal_matrix*(M-C));
               result = glup_lighting(result, N);
           }
           glup_FragColor = result;
           */

            vec3 co = R.O-C;

            vec3 coXn = cross(co,n);
            vec3 vXn  = cross(R.V,n);

            float coDn = dot(co,n);
            float vDn  = dot(R.V,n);

            float a = u*dot(vXn,vXn) + v*vDn;
            float b = 2.0*(u*dot(coXn,vXn)+v*coDn*vDn);
            float c = u*dot(coXn,coXn) + v*coDn*coDn - 4.0; // HERE R2
            float delta = b*b - 4.0*a*c;
            if(delta < 0.0) {
               discard;
            }
            float sq = sqrt(delta);
            float t1 = (-b - sq) / (2.0*a);
            float t2 = (-b + sq) / (2.0*a);
            float t = min(t1,t2);
            vec3 M = R.O + t * R.V;
            float s = 1;

            if(
               dot(M,Pi1.xyz)+Pi1.w > 0 ||
               dot(M,Pi2.xyz)+Pi2.w > 0
            ) {
                t = max(t1,t2);
                M = R.O + t * R.V;
                s = -1;
                if(
                  dot(M,Pi1.xyz)+Pi1.w > 0 ||
                  dot(M,Pi2.xyz)+Pi2.w > 0
                ) {
                   discard;
                }
            }

            glup_update_depth(M);
            vec4 result = GLUP.front_color;
            if(glupIsEnabled(GLUP_LIGHTING)) {
               vec3 H = C+dot(M-C,n)*n;
               vec3 N = s*(M-H);
               N = normalize(GLUP.normal_matrix*N);
               result = glup_lighting(result, N);
            }
            glup_FragColor = result;
        }
	)";


   /********************************************************************/
}
