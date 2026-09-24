
namespace {

    const char* GLUPES_spheres_source =
        R"(
        //primitive GLUP_TRIANGLES
	)"

        R"(
        //stage GL_VERTEX_SHADER
        //import <GLUP/current_profile/vertex_shader_preamble.h>
	//import <GLUP/stdglup.h>
	//import <GLUP/current_profile/toggles.h>
	//import <GLUPES/vertex_shader_state.h>
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
            gl_Position = GLUP_VS.modelviewprojection_matrix * vertex_in;
	}
	)"

        R"(
        //stage GL_FRAGMENT_SHADER
        //import <GLUP/current_profile/fragment_shader_preamble.h>
        //import <GLUPES/fragment_shader_state.h>
        //import <GLUP/stdglup.h>
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
	//import <GLUP/current_profile/toggles.h>
	//import <GLUPES/vertex_shader_state.h>
	glup_in vec4 vertex_in;
        glup_in vec4 color_in;
        glup_in vec4 tex_coord_in;
        glup_in vec4 normal_in;
        glup_flat glup_out vec4 color;
        glup_flat glup_out vec4 focus_R;
        glup_flat glup_out vec3 axis;
	void main() {
	    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
	       color = color_in;
	    }
	    focus_R = tex_coord_in;
            axis = normal_in.xyz;
            gl_Position = GLUP_VS.modelviewprojection_matrix * vertex_in;
	}
	)"

        R"(
        //stage GL_FRAGMENT_SHADER
        //import <GLUP/current_profile/fragment_shader_preamble.h>
        //import <GLUPES/fragment_shader_state.h>
        //import <GLUP/stdglup.h>
        //import <GLUP/current_profile/toggles.h>
        //import <GLUP/current_profile/primitive.h>
        //import <GLUP/fragment_shader_utils.h>
        //import <GLUP/fragment_ray_tracing.h>

        uniform vec2 cAxisPerp;
        glup_flat glup_in vec4 color;
        glup_flat glup_in vec4 focus_R;
        glup_flat glup_in vec3 axis;

	void main() {

            vec3 F = focus_R.xyz;
            float R2 = -focus_R.w;
            vec3 A = axis;
            float u_cAxis = cAxisPerp.x;
            float u_cPerp = cAxisPerp.y;
            Ray R = glup_primary_ray();

            //  F(O + tV) = a.t^2 + b.t + c, obtained by expanding
            //  cdiff.u(t)^2 + cPerp.|d(t)|^2 - R2
            // with d = (O-F) + tV and u = d.A.
            //  |V| is NOT one here -- the ray comes from two
            // clip-space points -- so
            //  dot(V,V) is carried explicitly rather than assumed away.
            vec3  dv = R.O - F;
            float u0 = dot(dv, A);
            float da = dot(R.V, A);
            float cdiff = u_cAxis - u_cPerp;
            float a = cdiff*da*da + u_cPerp*dot(R.V, R.V);
            float b = 2.0*(cdiff*u0*da + u_cPerp*dot(dv, R.V));
            float c = cdiff*u0*u0 + u_cPerp*dot(dv, dv) - R2;

            float t0, t1;
            if(abs(a) < 1e-12) {
                //  The ray runs along an asymptotic direction: one root only.
                if(abs(b) < 1e-20) {
                    discard;
                }
                t0 = -c / b;
                t1 = t0;
            } else {
                float delta = b*b - 4.0*a*c;
                if(delta < 0.0) {
                    discard;
                }
                float sq = sqrt(delta);
                float ra = (-b - sq) / (2.0*a);
                float rb = (-b + sq) / (2.0*a);
                //  a may be negative, which swaps them --
                // hence the explicit sort
                //  instead of relying on the sign.
                t0 = min(ra, rb);
                t1 = max(ra, rb);
            }

            float t = (t0 > 0.0) ? t0 : t1;
            if(t < 0.0) {
               discard;
            }
            vec3 M = R.O + t * R.V;

            glup_update_depth(M);
            vec4 result = GLUP.front_color;
            if(glupIsEnabled(GLUP_LIGHTING)) {
               //  grad F = 2 . ( (cAxis - cPerp) . u . A  +  cPerp . d )
               vec3 d = M - F;
               vec3 N = cdiff * dot(d, A) * A + u_cPerp * d;
               N = normalize(GLUP.normal_matrix*N);
               result = glup_lighting(result, N);
            }
            glup_FragColor = result;
        }
	)";


   /********************************************************************/
}
