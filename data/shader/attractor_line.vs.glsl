
uniform mat4 world_mat;
uniform mat4 view_mat;
uniform mat4 proj_mat;
uniform vec3 color_info;	// highlight ID + highlight range + mesh alpha

layout (location = 0) in vec4 position;

out vec4 vs_fs_color;

void main(void)
{
	vec4 view_pos = (view_mat * (world_mat * position));
    gl_Position = proj_mat * view_pos;

	if( color_info.x >= 0 )
	{
		/*float color_ratio = clamp( 0.5 + 0.5 * ((float(gl_VertexID) - color_info.x) / color_info.y), 0.0, 1.0 );
		if( color_ratio == 1. || color_ratio == 0. )
			vs_fs_color = vec4( vec3(0.0,0.0,0.0), color_info.z * 0.7 );
		else*/
			vs_fs_color = vec4( vec3(1.0,0.0,0.0), color_info.z );

		//float alpha = color_ratio == 1. || color_ratio == 0. ? 0.2 : 1.0;  
		//vs_fs_color = vec4( mix( vec3(1.0,0.0,0.0), vec3(1.0,0.0,0.0), color_ratio ), color_info.z * alpha );
	}
	else
	{
		vs_fs_color = vec4(0.7, 0.7, 0.7, 0.8);
	}
}
