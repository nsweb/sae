
//layout (std140, binding = 0) uniform WVP
//{
	uniform mat4 proj_mat;
//	uniform mat4 model2cam_mat;
//	uniform mat4 proj_mat;
//	uniform vec3 sun_dir;
//	uniform vec3 back_dir;
//};


layout (location = 0) in vec4 position;
layout (location = 1) in vec2 texcoord;
layout (location = 2) in vec4 color;

out vec2 vs_fs_texcoord;
out vec4 vs_fs_color;

void main(void)
{
    gl_Position = proj_mat * (position);

	//vec2 vertexPosition_homoneneousspace = position.xy - vec2(400,300); // [0..800][0..600] -> [-400..400][-300..300]
 //   vertexPosition_homoneneousspace /= vec2(400,300);
 //   gl_Position =  vec4(vertexPosition_homoneneousspace,0,1);

	vs_fs_color = color;
	vs_fs_texcoord = texcoord;
}
