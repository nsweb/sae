

in vec4 vs_fs_color;

layout (location = 0) out vec4 frag_color;

void main(void)
{
	frag_color.rgb = vec3(0.8,0.8,0.8);
	frag_color.rgb *= vs_fs_color.rgb;
	frag_color.a = vs_fs_color.a;
    
    // output in gamma space
    frag_color.rgb = pow( frag_color.rgb, vec3( 1.0/2.2 ) );
}
