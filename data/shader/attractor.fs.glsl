
//const float c_normal_distance = 0.0001;
//include "common/dftools.h"
//uniform float collision_dist = 1.0;

in vec3 vs_fs_normal;
in vec4 vs_fs_color;

layout (location = 0) out vec4 frag_color;


void main(void)
{
    vec3 light_dir = normalize(vec3(0.5,0.7,1.0));
    vec3 normal = normalize(vs_fs_normal);
	float dotNL = max( 0., 0.1 + dot( normal, light_dir ) );
    float dotNL_back = max( 0., dot( normal, vec3(0.0, 0.0, -1.0) ) );
	frag_color.rgb = vec3(0.8,0.8,0.8) * dotNL + vec3(0.01,0.01,0.1) * dotNL_back;
	frag_color.rgb *= vs_fs_color.rgb;
	frag_color.a = vs_fs_color.a;
    
    // output in gamma space
    frag_color.rgb = pow( frag_color.rgb, vec3( 1.0/2.2 ) );
}
