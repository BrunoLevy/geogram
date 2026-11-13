
#include "GLUP_buffer.h"
#include <geogram_gfx/basic/GLSL.h>


namespace {

    void register_geomol_shader_utilities() {
	// Portable abstraction of "buffers", that can be implemented as:
	// - a TBO (Texture Buffer Object) if GEO_TBO_SUPPORTED is defined
	// - or an integer or floating-point texture otherwise
	GLUP::BufferObject::Type bufftype = GLUP::BufferObject::default_type();

	if(bufftype == GLUP::BufferObject::TEXTURE_2D) {
	    GEO::GLSL::register_GLSL_include_file("geomol/buffers.h",
            R"(
            #define glup_Buffer  sampler2D
            #define glup_iBuffer isampler2D
            #define glup_uBuffer usampler2D
            #define glup_fetch(B,idx) texelFetch(B,ivec2( \
                       int(idx)%textureSize(B,0).x, \
                       int(idx)/textureSize(B,0).x  \
                    ),0)
            )"
            );
	} else if(bufftype == GLUP::BufferObject::TBO) {
	  GEO::GLSL::register_GLSL_include_file("geomol/buffers.h",
          R"(
          #define glup_Buffer  samplerBuffer
          #define glup_iBuffer isamplerBuffer
          #define glup_uBuffer usamplerBuffer
          #define glup_fetch(B,idx) texelFetch(B,int(idx))
          )"
          );
	}

        // Utilities for fragment shaders:
        // - clipping by cells and clipping by user clip plane
        // - drawing mesh (exploiting distance to planes and its derivative)
	// - drawing imposter (for debugging)
	GEO::GLSL::register_GLSL_include_file("geomol/fragment_shader_utils.h",
        R"(
        uniform glup_uBuffer facet_ptr_TBO;
        uniform glup_Buffer facet_plane_TBO;
        uniform bool show_imposters;
        uniform int max_clip_planes; // max number of clipping planes to test

        // \brief Tests whether a point is inside a cell of the mixed complex
        // \details max_clip_planes (a uniform) determines the maximum number
        //   of tested clipping planes. For optimization purposes, clipping
        //   can be complitely deactivated for S0 cells, and limited to the
        //   first two clipping planes for H1 cells.
        // \param[in] p the world-space coordinates of the point
        // \param[in] cell_id the index of the cell to be displayed (typically
        //   gl_vertexID divided by number of vertices per imposter, that is, 6)
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

        // draw imposter instead of raytracing quadric, for debugging purposes
        // \param[in] C center of the quadric
        void draw_imposter(in vec3 C) {
           if(glupIsEnabled(GLUP_CLIPPING)) {
              // x,y coordinates of current fragment
              vec2 frag_xy_c = 2.0*gl_FragCoord.xy / vec2(
                   GLUP.viewport[2], GLUP.viewport[3]
              ) + vec2(-1.0,-1.0);
              // copy z from quadric's center, transformed in clip space
              // (so that SSAO lets see more or less where the imposters are)
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

	GEO::GLSL::register_GLSL_include_file("geomol/vertex_shader_utils.h",
	R"(
        // For debugging purpose: if set, draw imposters that fill the
        // entire screen
        uniform bool full_imposters;

        // imposter offset in function of (vertex id modulo 6)
        const vec2 imposter_offset[6] = vec2[](
            vec2(-1.0, -1.0), vec2( 1.0, -1.0), vec2( 1.0,  1.0),
            vec2(-1.0, -1.0), vec2( 1.0,  1.0), vec2(-1.0,  1.0)
        );
        )"
	);
    }

   /********************************************************************/

    const char* spheres_source =
        R"(
        //stage GL_VERTEX_SHADER
        //import <GLUP/current_profile/vertex_shader_preamble.h>
	//import <GLUP/stdglup.h>
	//import <GLUPGLSL/state.h>
	//import <GLUP/current_profile/toggles.h>
        //import <geomol/buffers.h>
        //import <geomol/vertex_shader_utils.h>

        uniform glup_Buffer cell_eqn_1_TBO;
        uniform glup_Buffer cell_imposter_1_TBO;
        uniform bool replace_imposter; // if set, use cell_imposter_1_TBO

        glup_flat glup_out vec4 focus_R;
        glup_flat glup_out int cell_id;


        // Correction factor on both axes for non-square windows
        // Obtained by computing how a normal vector is transformed
        // from window space to normalized clip space, that is, by the
        // *transpose* normalized clip space to window space matrix.
        vec2 viewport_scaling() {
            float W = GLUP.viewport[2];
            float H = GLUP.viewport[3];
            return vec2(2.0*H/(W+H),2.0*W/(W+H));
        }

        // Gets the ith row of a 4x4 matrix
        vec4 row4x4(in mat4 M, in int i) {
           return vec4(M[0][i], M[1][i], M[2][i], M[3][i]);
        }

        // clip-space radius of a circle covering the projection
        // of a world-space sphere.
        // Dmitry Sokolov tutorial: https://github.com/ssloy/glsltutorial
        // initial ref: http://reality.cs.ucl.ac.uk/projects/quadrics/pbg06.pdf
        //
        // center_w: center of the sphere in world coordinates
        // R_w: radius of the sphere in world coordinates
        // center_c: center of the projected sphere in clip coordinates
        // R_c: radius of a disk centered on center_c that covers the projection
        //      of the sphere.
        void project_sphere(
            in vec3 center_w, in float R_w, out vec2 center_c, out float R_c
        ) {
            // This looks suboptimal, but all these zeroes and ones product
            // are (probably) optimized-out by the GLSL compiler (at least
            // I hope so, to be verified...)
            // Important reminder: GLSL constructs matrices in column major
            // order ! (each time I'm confused by that !)
            mat4 T = mat4(
               1.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0,
               center_w.x/R_w, center_w.y/R_w, center_w.z/R_w, 1.0/R_w
            );

            mat4 PMT = GLUP.modelviewprojection_matrix * T;
            vec4 r1 = row4x4(PMT,0);
            vec4 r2 = row4x4(PMT,1);
            vec4 r4 = row4x4(PMT,3);

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
        //import <geomol/fragment_shader_utils.h>

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
	   // Don't know why they do that, here we know we want t1 first
           float sqrt_a_delta = sqrt(a*delta);
	   float q = b_prime - sqrt_a_delta;
	   float t = q/a;

           vec3 M = R.O + t*R.V;

           // If intersection is outside cell or clipped out, get second
           // intersection.
           if(clipped(M) || !in_cell(M,cell_id)) {
              // Get second intersection
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
        //stage GL_VERTEX_SHADER
        //import <GLUP/current_profile/vertex_shader_preamble.h>
	//import <GLUP/stdglup.h>
	//import <GLUPGLSL/state.h>
	//import <GLUP/current_profile/toggles.h>
        //import <geomol/buffers.h>
        //import <geomol/vertex_shader_utils.h>

        uniform glup_Buffer cell_eqn_1_TBO;
        uniform glup_Buffer cell_eqn_2_TBO;
        uniform glup_Buffer cell_imposter_1_TBO;
        uniform glup_Buffer cell_imposter_2_TBO;

        glup_flat glup_out vec3  C;  // focus
        glup_flat glup_out vec3  n;  // axis
        glup_flat glup_out float R2;
        glup_flat glup_out int cell_id;

        // Gets the ith row of a 4x3 matrix (called mat3x4, because
        // GLSL is column-major, even for types;  mat3x4 has 3 columns
        // and 4 rows).
        vec3 row3x4(in mat3x4 M, in int i) {
           return vec3(M[0][i], M[1][i], M[2][i]);
        }

        // Orthonormal basis (U, V, n) from a *unit* vector n.
        // Duff et al., "Building an Orthonormal Basis, Revisited", JCGT 2017.
        void orthonormal_basis(in vec3 n, out vec3 U, out vec3 V) {
           float s = (n.z >= 0.0) ? 1.0 : -1.0;
           float a = -1.0 / (s + n.z);
           float b = n.x * n.y * a;
           U = vec3(1.0 + s * n.x * n.x * a, s * b, -s * n.x);
           V = vec3(b, s + n.y * n.y * a, -n.y);
        }

        // Computes the tangent to the projection of a disk in clip space
        // \param[in] center_w , axis_w , R_w center, normal vector and radius
        //            of the disk in world space
        // \param[in] N1_c , N2_c two 2D vectors in clip space
        // \param[out] alpha1, alpha2, beta1, beta2
        //    the maximal and minimal value of alpha and beta such that the 2D
        //    straight lines of equations  N1_c.x*x + N1_c.y*y + alpha and
        //                                 N2_c.y*x + N2_c.x*y + beta
        //    are tangent to the projection of the disk.
        void project_disk(
            in vec3 center_w, in vec3 axis_w, in float R_w,
            in vec2 N1_c, in vec2 N2_c,
            out float alpha1, out float alpha2, out float beta1, out float beta2
        ) {

           // U, V MUST be unit and orthogonal: T maps (u,v,1) with u^2+v^2=1
           // to center_w + R_w*(u*U + v*V).
           vec3 U, V;
           orthonormal_basis(axis_w, U, V);

           // Remember, matrices are column-major order in GLSL!
           mat3x4 T = mat3x4(
              U.x, U.y, U.z, 0.0,
              V.x, V.y, V.z, 0.0,
              center_w.x/R_w, center_w.y/R_w, center_w.z/R_w, 1.0/R_w
           );
           mat3x4 PMT = GLUP.modelviewprojection_matrix * T;
           vec3 r1 = row3x4(PMT,0);
           vec3 r2 = row3x4(PMT,1);
           vec3 r4 = row3x4(PMT,3);

           vec3 rn1 = N1_c.x*r1+N1_c.y*r2;
           vec3 rn2 = N2_c.x*r1+N2_c.y*r2;

           float r4Dr4T   = dot(r4.xy,r4.xy)   -r4.z*r4.z;   // a
           float rn1Dr4T  = dot(rn1.xy,r4.xy)  -rn1.z*r4.z;  // b'
           float rn1Drn1T = dot(rn1.xy,rn1.xy) -rn1.z*rn1.z; // c

           float rn2Dr4T  = dot(rn2.xy,r4.xy)  -rn2.z*r4.z;  // b'
           float rn2Drn2T = dot(rn2.xy,rn2.xy) -rn2.z*rn2.z; // c

           float sqrtdelta1 = sqrt(abs(rn1Dr4T*rn1Dr4T - r4Dr4T*rn1Drn1T));
           float sqrtdelta2 = sqrt(abs(rn2Dr4T*rn2Dr4T - r4Dr4T*rn2Drn2T));

           if(r4Dr4T > 0.0) {
              alpha1 = (-rn1Dr4T-sqrtdelta1)/r4Dr4T;
              alpha2 = (-rn1Dr4T+sqrtdelta1)/r4Dr4T;
              beta1  = (-rn2Dr4T-sqrtdelta2)/r4Dr4T;
              beta2  = (-rn2Dr4T+sqrtdelta2)/r4Dr4T;
           } else {
              alpha2 = (-rn1Dr4T-sqrtdelta1)/r4Dr4T;
              alpha1 = (-rn1Dr4T+sqrtdelta1)/r4Dr4T;
              beta2  = (-rn2Dr4T-sqrtdelta2)/r4Dr4T;
              beta1  = (-rn2Dr4T+sqrtdelta2)/r4Dr4T;
           }
        }

        // intersection of two lines of equations
        //  D1.x * x + D1.y * y + D1.z = 0 and
        //  D2.x * x + D2.y * y + D2.z = 0 and
        vec2 intersect(vec3 D1, vec3 D2) {
           return -inverse(mat2(D1.x,D2.x,D1.y,D2.y))*vec2(D1.z,D2.z);
        }

	void main() {
            cell_id = gl_VertexID / 6;
            vec4 focus_R = glup_fetch(cell_eqn_1_TBO,cell_id);
            C = focus_R.xyz;
            R2 = focus_R.w;
            n = glup_fetch(cell_eqn_2_TBO,cell_id).xyz;

            vec2 offset = imposter_offset[gl_VertexID%6];
            if(full_imposters) {
               gl_Position = vec4(offset,0.0,1.0);
               return;
            }

            vec4 imp1 = glup_fetch(cell_imposter_1_TBO,cell_id);
            vec4 imp2 = glup_fetch(cell_imposter_2_TBO,cell_id);

            vec4 UV1 = GLUP.modelviewprojection_matrix * vec4(imp1.xyz,1.0);
            vec4 UV2 = GLUP.modelviewprojection_matrix * vec4(imp2.xyz,1.0);

            // (N1_c,N2_c): projected axis of cyl and ortho direction, clip space
            vec2 N1_c = vec2(UV2.xy/UV2.w-UV1.xy/UV1.w);
            vec2 N2_c = normalize(vec2(-N1_c.y, N1_c.x));
            // fix N1_c to ensure orthog w.r.t. N2_c *in window coords*
            // (W*W and H*H: pullback dot product from win to clip space)
            float W = GLUP.viewport[2]; float H = GLUP.viewport[3];
            N1_c = normalize(vec2(N1_c.x * W * W, N1_c.y * H * H));

            float a11, a12, b11, b12; // bounds of first cap
            float a21, a22, b21, b22; // bounds of second cap
            project_disk(imp1.xyz, n, imp1.w, N1_c, N2_c, a11, a12, b11, b12);
            project_disk(imp2.xyz, n, imp2.w, N1_c, N2_c, a21, a22, b21, b22);

            vec2 uv = 0.5*(offset+vec2(1.0,1.0)); // remap offset to [0,1]
            // bounding line equations. Note: line eqn: dot(N1_c,UV)+alpha = 0->
            // large alpha means small U (same thing for beta,V), hence
            // max alphas, max betas first -. then mins -.
            //                              v            v
            vec3 A = vec3(N1_c, mix(max(a12,a22), min(a11,a21), uv.x));
            vec3 B = vec3(N2_c, mix(max(b12,b22), min(b11,b21), uv.y));
            gl_Position=vec4(intersect(A,B),0.0,1.0);
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
        //import <geomol/fragment_shader_utils.h>

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

            // If intersection is outside cell or clipped out, get second
            // intersection.
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
