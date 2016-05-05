
//layout (std140, binding = 0) uniform WVP
//{
	//uniform mat4 world_mat;
	uniform mat4 view_mat;
	uniform mat4 proj_mat;
//};


layout (location = 0) in vec4 position;
layout (location = 1) in mat4 world_mat;
layout (location = 5) in vec4 color;
layout (location = 6) in vec4 params;
layout (location = 7) in vec3 eye_to_box;

out vec4 vs_fs_color;
out vec3 vs_fs_view_pos;
out vec3 vs_fs_box_pos;
out vec3 vs_fs_box_eye_pos;
out vec4 vs_fs_params;

void main(void)
{
    vec4 view_pos = (view_mat * (world_mat * position));
    vs_fs_view_pos = view_pos.xyz;
    gl_Position = proj_mat * view_pos;
    
    vs_fs_box_pos = position.xyz;
    vs_fs_box_eye_pos = eye_to_box;
    vs_fs_color = color;
    vs_fs_params = params;
}
