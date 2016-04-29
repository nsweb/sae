
//layout (std140, binding = 0) uniform WVP
//{
	//uniform mat4 world_mat;
	uniform mat4 view_mat;
	uniform mat4 proj_mat;
//};


layout (location = 0) in vec4 position;
layout (location = 1) in vec4 color;

out vec4 vs_fs_color;

void main(void)
{
    gl_Position = proj_mat * (view_mat * /*world_mat **/ position);

    vs_fs_color = color;
}
