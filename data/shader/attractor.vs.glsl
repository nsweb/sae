
//layout (std140, binding = 0) uniform WVP
//{
	uniform mat4 world_mat;
	uniform mat4 view_mat;
	uniform mat4 proj_mat;
	//uniform mat4 viewtobox_mat;
	//uniform mat4 viewinv_mat;
//};


layout (location = 0) in vec4 position;
layout (location = 1) in vec3 normal;
//layout (location = 2) in mat4 world_mat;

//out vec3 vs_fs_pos;
out vec3 vs_fs_normal;

void main(void)
{
	vec4 view_pos = (view_mat * (world_mat * position));
    gl_Position = proj_mat * view_pos;
	
	vs_fs_normal = (world_mat * vec4(normal, 0.)).xyz;
}
