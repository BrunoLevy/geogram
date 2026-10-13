//import <fullscreen/current_profile/fragment_shader_preamble.h>
//import <GLUP/stdglup.h>

glup_in vec2 tex_coord;
uniform sampler2D tex;
uniform int downscale_factor;

// downscaling for depth buffer, takes the max value

void main() {
   float g = 0.0;
   int x = int(gl_FragCoord.x)*downscale_factor;
   int y = int(gl_FragCoord.y)*downscale_factor;
   for(int dx = 0; dx<downscale_factor; ++dx) {
       for(int dy = 0; dy<downscale_factor; ++dy) {
	   g = max(g,texelFetch(tex, ivec2(x+dx,y+dy),0).x);
       }
   }
   glup_FragColor = vec4(g,g,g,1.0);
}
