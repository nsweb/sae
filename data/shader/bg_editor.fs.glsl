

#ifdef SHADER_SECTION
	layout (location = 0) out vec4 frag_color;
#endif

const float c_normal_distance = 0.002f;
const float c_minimun_distance = 0.001f;
const int c_max_march_steps = 150;
 
#include "common/dftools.h"

#ifdef SHADER_SECTION
	smooth in vec2 vs_fs_texcoord;
	smooth in vec4 vs_fs_color;
#endif

#ifdef SHADER_SECTION
    uniform mat4 viewinv_mat;
    uniform mat4 proj_mat;
    uniform vec3 camera_pos;
    uniform float global_time;
    // z_var.x = (far+near)/(far-near);
    // z_var.y = 2.0*far*near/(far-near);
    uniform vec2 z_var;
    uniform vec2 screen_res;
    uniform vec2 scene_center = vec2( 0., 0. );
#endif



vec2 opU( vec2 d1, vec2 d2 )
{
    return (d1.x<d2.x) ? d1 : d2;
}
float opS( float d1, float d2 )
{
    return max(-d2,d1);
}
float fPlane( vec3 p )
{
    return p.z;
}

vec2 map( vec3 pos )
{
    return vec2( -1., -1. );
}

vec2 map_ext(vec3 pos, vec3 dir)
{
    // direct plane intersection
    if( dir.z > -1.e-4 )
        return vec2( -1., -1. );   // bg
        
    float l = -pos.z / dir.z;
    vec2 inter = pos.xy + dir.xy * l;
    vec2 v = inter - scene_center;
    float v2 = dot(v, v);
    if( v2 > 40.*40. )
        return vec2( -1., -1. );   // bg
    
    return vec2( l, 1. );
}

/*
vec3 rayMarch( vec3 from, vec3 dir, float MaxDistance )
{
 	vec3 pos;
    vec2 dist_mat = vec2( c_minimun_distance*2.0f, 0.0f );
    float sum_dist = 0.0;
    int sum_steps = 0;
    for( int i=0; i<c_max_march_steps; i++ )
    {
        pos = from + sum_dist * dir;
        dist_mat = map(pos, dir);
        if( dist_mat.x < c_minimun_distance ) 
            break;
        sum_dist += dist_mat.x * 1.0f;
        sum_steps++;
    }

    if( sum_dist >= MaxDistance ) 
        sum_dist = -1.0;

    float ao = float(sum_steps) / float(c_max_march_steps);
    return vec3( sum_dist, dist_mat.y, ao );   
}*/

#ifdef SHADER_SECTION
	void main(void)
	{
		vec3 camPos     = viewinv_mat[3].xyz;
		vec3 camRight   = viewinv_mat[0].xyz;
		vec3 camUp      = viewinv_mat[1].xyz;
		vec3 camDir     = -viewinv_mat[2].xyz;
	
		vec2 screen_coord = vs_fs_texcoord.xy * screen_res * 1.0;
	
		// Get direction for this pixel
		vec3 rayDir = normalize(camDir + screen_coord.x*camRight + screen_coord.y*camUp);
    
		//
		vec3 l0 = vec3(1.0, 0.75, 0.05);
		vec3 d0 = normalize( vec3(1.0, 1.0, 1.0) );
		//vec3 l1 = vec3(0.4, 0.8, 0.9);//vec3(0.8, 0.2, 0.1);//
		//vec3 d1 = normalize( vec3(1.0, -0.5, -1.0) );
		vec4 bg_color = vec4( vec3(0.05, 0.14, 0.27)*0.5, 1.0 );
    
   		vec2 res = map_ext( camPos, rayDir );
		float zn = 0.999;
		if( res.x == -1.0 )
		{
			frag_color = bg_color;
		}
		else
		{
    		vec3 pos = camPos + res.x * rayDir;
    		//col = material(inters, dir);
			//vec3 nor = calcNormal( pos );
			//vec3 ref = reflect( rd, nor );
        
			vec3 normal = getNormal(pos);//-dir*normalDistance*3.0);
			vec3 col = l0 * max( 0.0, dot( d0, normal) );// + l1 * max( 0.0, dot( d1, normal) ); 
			
			if( res.y < 1.5 )
			{
				float f = mod( floor(5.0*pos.y) + floor(5.0*pos.x), 2.0);
				col = 0.2 + 0.1*f*vec3(1.0);
                col = mix( col, vec3(1., 0., 0.), smoothstep(0.03, 0., abs(pos.x)) );
                col = mix( col, vec3(0., 1., 0.), smoothstep(0.03, 0., abs(pos.y)) );
			}
			
			//col = mix( col, vec3( 1.0, 0.5, 0.25 ), res.y );
        
			frag_color = mix( vec4( col, 1.0 ), bg_color, 0. );
        
			float zeye = dot( camDir, rayDir ) * -res.x;
			zn = z_var.x + z_var.y / zeye;
			//frag_color = vec4( vec3( clamp( 0.5 + 0.5*zn , 0.0, 1.0) ), 1 );
		}
        
        // need to convert to window coord [0;1], *not* in normalized device coordinate [-1;1]
        gl_FragDepth = (zn + 1.0) / 2.0;
		//gl_FragDepth = zn;
    
		// output in gamma space
		frag_color.rgb = pow( frag_color.rgb, vec3( 1.0/2.2 ) );
	}
#endif	// SHADER_SECTION