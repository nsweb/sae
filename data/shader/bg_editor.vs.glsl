
layout (location = 0) in vec4 position;
layout (location = 1) in vec2 texcoord;

out vec2 vs_fs_texcoord;
out vec4 vs_fs_color;

void main(void)
{
    gl_Position = position;
	//gl_Position.xy *= 0.5;
	vs_fs_texcoord = texcoord;
}
