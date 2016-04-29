
//layout (std140, binding = 0) uniform WVP
//{
	//uniform mat4 world_mat;
	uniform mat4 view_mat;
	uniform mat4 proj_mat;
//};


layout (location = 0) in vec4 position;
layout (location = 1) in mat4 world_mat;
layout (location = 5) in vec4 color;
layout (location = 6) in vec3 params;
layout (location = 7) in vec3 eye_to_box;

out vec4 vs_fs_color;
out vec3 vs_fs_pos;
out vec3 vs_fs_eye_pos;
out vec3 vs_fs_params;

void main(void)
{
    gl_Position = proj_mat * (view_mat * (world_mat * position));
    
    vs_fs_pos = position.xyz;
    vs_fs_eye_pos = eye_to_box;
    vs_fs_color = color;
    vs_fs_params = params;
}
