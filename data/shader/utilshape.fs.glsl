
in vec4 vs_fs_color;
in vec3 vs_fs_pos;
in vec3 vs_fs_eye_pos;
in vec3 vs_fs_params;

//layout (location = 6) in vec3 params;

layout (location = 0) out vec4 frag_color;


bool RaySphereIntersection( vec3 origin, vec3 dir, /*float max_dist,*/ vec3 center, float radius )
{
    // get the offset vector
    vec3 offset = center - origin;
    
    // get the distance along the ray to the center point of the sphere
    float ray_dist = dot( dir, offset);
    
    // get the squared distances
    float off2 = dot( offset, offset);
    float rad_2 = radius * radius;
    if(off2 <= rad_2)
    {
        // we're in the sphere
        return true;
    }
    
    if(ray_dist <= 0 /*|| (ray_dist - max_dist) > radius*/)
    {
        // moving away from object or too far away
        return false;
    }
    
    // find hit distance squared
    float d2 = rad_2 - (off2 - ray_dist * ray_dist);
    if( d2 < 0.0f)
    {
        // ray passes by sphere without hitting
        return false;
    }
    
    // get the distance along the ray
    /*dist = ray_dist - sqrt(d2);
    if(dist > max_dist)
    {
        // hit point beyond length
        return false;
    }*/
    
    return true;
}

bool RayBoxIntersection(vec3 origin, vec3 dir, float ratio_y, float ratio_z)
{
    float tx1 = (-1. - origin.x) / dir.x;
    float tx2 = (1. - origin.x) / dir.x;
    
    float tmin = min(tx1, tx2);
    float tmax = max(tx1, tx2);
    
    float tz1 = (-ratio_z - origin.z) / dir.z;
    float tz2 = (ratio_z - origin.z) / dir.z;
    
    tmin = max(tmin, min(tz1, tz2));
    tmax = min(tmax, max(tz1, tz2));
    
    float ty1 = (-ratio_y - origin.y) / dir.y;
    float ty2 = (ratio_y - origin.y) / dir.y;
    
    tmin = max(tmin, min(ty1, ty2));
    tmax = min(tmax, max(ty1, ty2));
    
    return tmax >= tmin;
}

void main(void)
{
    vec3 rayDir = normalize( vs_fs_pos - vs_fs_eye_pos );
    
/*    vec3 t0 = (vec3( 1.f, 1.f, 1.f) - vs_fs_pos) / rayDir;
    vec3 t1 = (vec3(-1.f,-1.f,-1.f) - vs_fs_pos) / rayDir;
    vec3 tmin = min( t0, t1 );
    float dist = -max( max( tmin.x, tmin.y ), tmin.z );
*/
    
    bool intersection = false;
    if( vs_fs_params.x < 0.5 )          // BOX
    {
        intersection = RayBoxIntersection( vs_fs_eye_pos, rayDir, vs_fs_params.y, vs_fs_params.z );
    }
    else if( vs_fs_params.x < 1.5 )     // CYLINDER
    {
        intersection = true;
    }
    else if( vs_fs_params.x < 2.5 )     // SPHERE
    {
        intersection = RaySphereIntersection( vs_fs_eye_pos, rayDir, vec3(0., 0., 0.), vs_fs_params.y );
    }
    
    if( intersection )
    {
        frag_color = vs_fs_color;
    }
    else
    {
        frag_color = vec4(0., 0., 0., 0.);
    }
}
