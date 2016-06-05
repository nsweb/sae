
//layout (std140, binding = 0) uniform WVP
//{
	uniform mat4 world_mat;
	uniform mat4 view_mat;
	uniform mat4 proj_mat;
	uniform vec3 color_info;	// highlight ID + highlight range + mesh alpha
//};


layout (location = 0) in vec4 position;
layout (location = 1) in vec3 normal;
//layout (location = 2) in mat4 world_mat;

out vec3 vs_fs_normal;
out vec4 vs_fs_color;

void main(void)
{
	vec4 view_pos = (view_mat * (world_mat * position));
    gl_Position = proj_mat * view_pos;
	
	vs_fs_normal = (world_mat * vec4(normal, 0.)).xyz;

	if( color_info.x >= 0 )
	{
		float color_ratio = clamp( 0.5 + 0.5 * ((float(gl_VertexID) - color_info.x) / color_info.y), 0.0, 1.0 );
		float alpha = color_ratio == 1. || color_ratio == 0. ? 0.2 : 1.0;  
		vs_fs_color = vec4( mix( vec3(1.0,0.9,0.5), vec3(0.5,0.2,1.0), color_ratio ), color_info.z * alpha );
	}
	else
	{
		vs_fs_color = vec4(1., 1., 1., color_info.z);
	}
}
