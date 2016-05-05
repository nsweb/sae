
in vec4 vs_fs_color;
in vec3 vs_fs_view_pos;
in vec3 vs_fs_box_pos;
in vec3 vs_fs_box_eye_pos;
in vec4 vs_fs_params;

layout (location = 0) out vec4 frag_color;

uniform vec2 z_var;


float RaySphereIntersection( vec3 ray_orig, vec3 ray_dir, /*float max_dist,*/ vec3 center, float radius )
{
    // get the offset vector
    vec3 offset = center - ray_orig;
    
    // get the distance along the ray to the center point of the sphere
    float ray_dist = dot( ray_dir, offset);
    if(ray_dist <= 0.)
    {
        // moving away from object
        return -1.;
    }
    
    // get the squared distances
    float off_2 = dot( offset, offset);
    float rad_2 = radius * radius;
    if(off_2 <= rad_2)
    {
        // we're in the sphere
        return 0.;
    }
    
    // find hit distance squared
    float d_2 = rad_2 - (off_2 - ray_dist * ray_dist);
    if( d_2 < 0.0f)
    {
        // ray passes by sphere without hitting
        return -1.;
    }
    
    // get the distance along the ray
    float dist = ray_dist - sqrt(d_2);
    
    return dist;
}

float RayBoxIntersection(vec3 ray_orig, vec3 ray_dir, float ratio_y, float ratio_z)
{
    float tx1 = (-1. - ray_orig.x) / ray_dir.x;
    float tx2 = (1. - ray_orig.x) / ray_dir.x;
    
    float tmin = min(tx1, tx2);
    float tmax = max(tx1, tx2);
    
    float tz1 = (-ratio_z - ray_orig.z) / ray_dir.z;
    float tz2 = (ratio_z - ray_orig.z) / ray_dir.z;
    
    tmin = max(tmin, min(tz1, tz2));
    tmax = min(tmax, max(tz1, tz2));
    
    float ty1 = (-ratio_y - ray_orig.y) / ray_dir.y;
    float ty2 = (ratio_y - ray_orig.y) / ray_dir.y;
    
    tmin = max(tmin, min(ty1, ty2));
    tmax = min(tmax, max(ty1, ty2));
    
    if( tmax < tmin )
        return -1.;
    
    return tmin;
}

void main(void)
{
    vec3 ray_dir = normalize( vs_fs_box_pos - vs_fs_box_eye_pos );
    vec3 ray_dir_view = normalize( vs_fs_view_pos );
    
    float dist = -1.;
    float zn = 0.999;

    if( vs_fs_params.x < 0.5 )          // BOX
    {
        dist = RayBoxIntersection( vs_fs_box_eye_pos, ray_dir, vs_fs_params.y, vs_fs_params.z );
    }
    else if( vs_fs_params.x < 1.5 )     // CYLINDER
    {
        dist = 0.;
    }
    else if( vs_fs_params.x < 2.5 )     // SPHERE
    {
        dist = RaySphereIntersection( vs_fs_box_eye_pos, ray_dir, vec3(0., 0., 0.), vs_fs_params.y );
    }
    
    if( dist >= 0. )
    {
        float scale = vs_fs_params.w;
        //float zeye = dot( camDir, rayDir ) * -res.x;
        float zeye = ray_dir_view.z * dist * scale;
        zn = z_var.x + z_var.y / zeye;
    
        frag_color = vs_fs_color;
    }
    else
    {
        frag_color = vec4(0., 0., 0., 0.);
    }
    
    // output in gamma space
    frag_color.rgb = pow( frag_color.rgb, vec3( 1.0/2.2 ) );
    
    // need to convert to window coord [0;1], *not* in normalized device coordinate [-1;1]
    gl_FragDepth = (zn + 1.0) / 2.0;
}
