
layout (location = 0) out vec4 color;
 
uniform sampler2D textureUnit0;

smooth in vec2 vs_fs_texcoord;
smooth in vec4 vs_fs_color;

 
void main(void)
{
    vec4 rgba = texture( textureUnit0, vs_fs_texcoord );
	rgba *= vs_fs_color;
	//rgba.a = 0.5;

	color = rgba;//vec4( rgba.r, 0,0,1);
}