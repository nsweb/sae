
//const float c_normal_distance = 0.0001;
//include "common/dftools.h"
//uniform float collision_dist = 1.0;

in vec3 vs_fs_normal;

layout (location = 0) out vec4 frag_color;


void main(void)
{
	float dotNL = max( 0., dot( normalize(vs_fs_normal), normalize(vec3(0.5,0.7,1.0)) ) );
	frag_color.rgb = vec3(0.8,0.8,0.8) * dotNL;
	frag_color.a = 1.0;
    
    // output in gamma space
    frag_color.rgb = pow( frag_color.rgb, vec3( 1.0/2.2 ) );
}
