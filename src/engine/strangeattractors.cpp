
#include "../sae.h"
#include "strangeattractors.h"
#include "system/file.h"
#include "math/intersections.h"
#include "math/splines.h"
#include "core/sort.h"


void SAUtils::ComputeStrangeAttractorPoints(StrangeAttractor* attractor, AttractorLineParams const& params, Array<vec3>& line_points)
{
    vec3 seed = params.seed;
    attractor->m_init_point = seed;
    
    float init_dt = attractor->m_dt;
    float init_ad = attractor->m_adaptative_dist;
    
    attractor->m_dt = init_dt * params.step_factor;
    attractor->m_adaptative_dist = init_ad * params.step_factor;
    
    if (params.warmup_iter > 0)
    {
        attractor->LoopAdaptative(line_points, params.warmup_iter);
        seed = line_points.Last();
        attractor->m_init_point = line_points.Last();
        line_points.clear();
    }
    
    if (params.rev_iter > 0)
    {
        attractor->m_dt = -attractor->m_dt;
        attractor->LoopAdaptative(line_points, params.rev_iter);
        
        const int32 nb_point = line_points.size();
        for (int32 i = 0; i < nb_point / 2; i++)
            line_points.Swap(i, nb_point - 1 - i);
        line_points.push_back(seed);
        
        attractor->m_dt = -attractor->m_dt;
    }
    
    attractor->LoopAdaptative(line_points, params.iter);
    
    if (params.shearing_scale_x != 1.f || params.shearing_scale_y != 1.f || params.shearing_angle != 0.f)
    {
        const float cf = bigball::cos(params.shearing_angle);
        const float sf = bigball::sin(params.shearing_angle);
        
        for (int32 i = 0; i < line_points.size(); ++i)
        {
            vec3 p = line_points[i];
            float rot_x = params.shearing_scale_x * (p.x * cf - p.y * sf);
            float rot_y = params.shearing_scale_y * (p.x * sf + p.y * cf);
            line_points[i].x = rot_x * cf + rot_y * sf;
            line_points[i].y = rot_x * -sf + rot_y * cf;
        }
    }
    
    // Adapt size
    if (params.target_dim > 0.0)
    {
        vec3 min_pos(FLT_MAX, FLT_MAX, FLT_MAX), max_pos(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        for (int32 i = 0; i < line_points.size(); ++i)
        {
            min_pos.x = bigball::min(min_pos.x, line_points[i].x);
            min_pos.y = bigball::min(min_pos.y, line_points[i].y);
            min_pos.z = bigball::min(min_pos.z, line_points[i].z);
            max_pos.x = bigball::max(max_pos.x, line_points[i].x);
            max_pos.y = bigball::max(max_pos.y, line_points[i].y);
            max_pos.z = bigball::max(max_pos.z, line_points[i].z);
        }
        vec3 V = max_pos - min_pos;
        float dim_max = max(max(V.x, V.y), V.z);
        float rescale = params.target_dim / dim_max;
        for (int32 i = 0; i < line_points.size(); ++i)
            line_points[i] *= rescale;
    }
    
    attractor->m_dt = init_dt;
    attractor->m_adaptative_dist = init_ad;
}

static float SquaredDistancePointSegment_Unclamped(vec3 const& point, vec3 const& seg0, vec3 const& seg1, float& t_on_segment_unclamped)
{
    vec3 V_seg0 = point - seg0;
    vec3 dir_seg = seg1 - seg0;
    float t_unclamped = dot(dir_seg, V_seg0);
    
    float dot_dd = dot(dir_seg, dir_seg);
    t_unclamped /= dot_dd;
    
    float t = bigball::clamp(t_unclamped, 0.f, 1.f);
    V_seg0 += dir_seg*(-t);
    
    t_on_segment_unclamped = t_unclamped;
    return dot(V_seg0, V_seg0);
}

static void PrintBary(SABarycenterRef const* bary, int32 bary_idx, float dist)
{
    String str_debug;
    str_debug = String::Printf("bary [%d][%.2f]", bary_idx, dist);
    for( int32 s_idx = 0; s_idx < bary->seg_refs.size(); s_idx++)
    {
        int32 seg_ref = bary->seg_refs[s_idx];
        str_debug += String::Printf(" %d", seg_ref);
    }
    str_debug += "\n";
    BB_LOG(SAUtils, Log, str_debug.c_str());
}

void SAUtils::MergeLinePoints5(AttractorLineFramed const& line_framed, const Array<AttractorHandle>& attr_handles, AttractorShapeParams const& shape_params, Array<AttractorLineFramed>& snapped_lines)
{
    BB_LOG(SAUtils, Log, "MergeLinePoints5\n");
    
    Array<AttractorSnapRange> snap_ranges;
    const int32 interp_spacing = 100;
    const bool blend_position = true;
    const Array<vec3>& line_points = line_framed.points;
    const int32 nb_points = line_points.size();
    if(nb_points > interp_spacing*2 + 2)
    {
        vec3 p_0 = line_points[0];
        vec3 p_1 = line_points.Last();
        
        const float merge_dist = shape_params.merge_dist;
        const float sq_merge_dist = merge_dist * merge_dist;
        float t, sq_dist, min_sq_dist_0 = sq_merge_dist, min_sq_dist_1 = sq_merge_dist;
        int32 min_seg_0 = INDEX_NONE, min_seg_1 = INDEX_NONE;
        for (int32 p_idx = interp_spacing; p_idx < nb_points - interp_spacing - 1; p_idx++)
        {
            sq_dist = intersect::SquaredDistancePointSegment(p_0, line_points[p_idx], line_points[p_idx + 1], t);
            if (sq_dist < min_sq_dist_0)
            {
                min_sq_dist_0 = sq_dist;
                min_seg_0 = p_idx;
               
            }
            sq_dist = intersect::SquaredDistancePointSegment(p_1, line_points[p_idx], line_points[p_idx + 1], t);
            if (sq_dist < min_sq_dist_1)
            {
                min_sq_dist_0 = sq_dist;
                min_seg_1 = p_idx;
                
            }
        }
        
        if( min_seg_0 != INDEX_NONE && min_seg_1 != INDEX_NONE)
        {
            AttractorSnapRange sr_0 = { ivec2(0,0), ivec2(min_seg_0, min_seg_0) };
            AttractorSnapRange sr_1 = { ivec2(nb_points-1,nb_points-1), ivec2(min_seg_1, min_seg_1) };
            snap_ranges.push_back(sr_0);
            snap_ranges.push_back(sr_1);
        }
    }
    
    GenerateSnappedLinesWithFrames(line_framed.points, line_framed.frames, line_framed.follow_angles, snap_ranges, shape_params, snapped_lines, interp_spacing, blend_position);
}

void SAUtils::MergeLinePoints4(AttractorLineFramed const& line_framed, const Array<AttractorHandle>& attr_handles, AttractorShapeParams const& shape_params, Array<AttractorLineFramed>& snapped_lines)
{
    BB_LOG(SAUtils, Log, "MergeLinePoints4\n");
    
    // New algo (force based)
    Array<vec3> line_points = line_framed.points;
    float avg_pt_separation = 0.f;
    {
        int max_pt = max(100, line_points.size() - 1);
        for (int i = 0; i < max_pt; i++)
        {
            float d = distance(line_points[i], line_points[i  + 1]);
            avg_pt_separation += d;
        }
        avg_pt_separation /= (float)max_pt;
    }
    
    // 1- sort every line point into a grid
    SAGrid grid;
    grid.InitGrid(line_framed.points, 1000);
    
    // 2- parse every point and find out if it has neighbours <= merge_dist range
    //    move_factor : mark 0.0 if no neighbours, 1.0 otherwise
    Array<float> move_factors;
    const int nb_points = line_points.size();
    move_factors.resize(nb_points, 0.f);
    
    const int interp_range = 40;
    
if(!shape_params.show_bary)
{
    const int max_pass = shape_params.max_iter_count;
    for(int pass_idx = 0; pass_idx < max_pass; pass_idx++)
    {
        bool first_pass = (pass_idx == 0 ? true : false);
        for (int pt_idx = 0; pt_idx < nb_points; pt_idx++)
        {
            if(!first_pass && move_factors[pt_idx] < 1.0f)
                continue;
            
            SASegResult found_seg = grid.FindNearestSeg(pt_idx, line_points, shape_params.merge_dist, interp_range);
            if (found_seg.cell_id != INDEX_NONE)
            {
                SACell const& seg_cell = grid.cells[found_seg.cell_id];
                int seg_idx = seg_cell.segs[found_seg.seg_in_array_idx];
                vec3 seg_0 = line_points[seg_idx];
                vec3 seg_1 = line_points[seg_idx + 1];
                float dist = bigball::sqrt(found_seg.sq_dist);
                if( dist < 1e-4f )
                    continue;
                vec3 seg_p = seg_0 * (1.f - found_seg.t_seg) + seg_1 * found_seg.t_seg;
                vec3 p_0 = line_points[pt_idx];
                vec3 dir_seg = (seg_p - p_0) / dist;
                float push_dist = min( dist, avg_pt_separation );
                vec3 new_pos = p_0 + dir_seg * push_dist;
                line_points[pt_idx] = new_pos;
                
                grid.MovePoint(p_0, new_pos, pt_idx);
                
                move_factors[pt_idx] = 1.0f;
            }
        }

        // 3- interpolate and propagate move_factor 1.0 1.0 0.9 0.8 0.7 0.6 0.5 0.4 0.5 0.6 0.7 0.8 0.9 1.0
        if (first_pass)
        {
            int last_idx = INDEX_NONE;
            for (int pt_idx = 0; pt_idx < nb_points; pt_idx++)
            {
                if(move_factors[pt_idx] == 1.f)
                    last_idx = pt_idx;
                else
                {
                    if(pt_idx - last_idx < interp_range)
                        move_factors[pt_idx] = 1.f - (pt_idx - last_idx) / interp_range;
                }
            }
            last_idx = INDEX_NONE;
            for (int pt_idx = nb_points - 1; pt_idx >= 0 ; pt_idx--)
            {
                if(move_factors[pt_idx] == 1.f)
                    last_idx = pt_idx;
                else
                {
                    if(last_idx - pt_idx < interp_range)
                        move_factors[pt_idx] = max(move_factors[pt_idx], 1.f - (last_idx - pt_idx) / interp_range);
                }
            }
        }
        
        // relaxation step: move midway from immediate neighbors (-1 +1), curve fitting from (-2 +2)
        CubicSpline spline;
        for (int pt_idx = 6; pt_idx < nb_points - 6; pt_idx++)
        {
            if(move_factors[pt_idx] < 1.f)
                continue;
            
            vec3 p_0 = line_points[pt_idx - 6];
            vec3 p_1 = line_points[pt_idx - 3];
            vec3 p_2 = line_points[pt_idx + 3];
            vec3 p_3 = line_points[pt_idx + 6];
            
            float dt_0 = bigball::pow(bigball::sqlength(p_0 - p_1), 0.25f);
            float dt_1 = bigball::pow(bigball::sqlength(p_1 - p_2), 0.25f);
            float dt_2 = bigball::pow(bigball::sqlength(p_2 - p_3), 0.25f);
            
            // safety check for repeated points
            if (dt_1 < 1e-4f)    dt_1 = 1.0f;
            if (dt_0 < 1e-4f)    dt_0 = dt_1;
            if (dt_2 < 1e-4f)    dt_2 = dt_1;
            
            spline.InitNonuniformCatmullRom(p_0, p_1, p_2, p_3, dt_0, dt_1, dt_2);
            vec3 s_pos, s_tan;
            spline.Eval(0.5f, s_pos, s_tan);
            
            // move halfway toward spline pos
            line_points[pt_idx] = (s_pos + line_points[pt_idx]) * 0.5f;
        }
        
    }
}
    
    AttractorLineFramed new_framed;
    snapped_lines.push_back(new_framed);
    AttractorLineFramed& ref_framed = snapped_lines.Last();
    
    ref_framed.points = line_points;
    ref_framed.colors.resize(nb_points, 0.f);

    GenerateFrames(ref_framed);
}

void SAUtils::MergeLinePoints3(AttractorLineFramed const& line_framed, const Array<AttractorHandle>& attr_handles, AttractorShapeParams const& shape_params, Array<AttractorLineFramed>& snapped_lines)
{
    BB_LOG(SAUtils, Log, "MergeLinePoints3\n");
    
    // New algo with barycenters
    SAGrid grid;
    
    // 1- sort every line segment into a grid
    grid.InitGrid(line_framed.points, 1000);
    
    // 2- Parse segment in order, find a barycenter seg nearby <= merge_dist
    const Array<vec3>& line_points = line_framed.points;
    const int nb_points = line_points.size();
    for (int seg_idx = 0; seg_idx < nb_points - 1; seg_idx++)
    {
        //if( seg_idx == 261 || seg_idx == 262 || seg_idx == 523 || seg_idx == 524 || seg_idx == 634 || seg_idx == 635 )
        //    int Break=0;
        vec3 p_0 = line_points[seg_idx];
        vec3 p_1 = line_points[seg_idx + 1];
        int cell_id = grid.GetCellIdx(p_0);
        SABaryResult found_bary = grid.FindBaryCenterSeg(seg_idx, p_0, /*line_points[seg_idx + 1],*/ shape_params.merge_dist);
        
        //  if it exists, point to it (seg_bary_array) and update barycenter point info with weighted average(pos + w)
        if (found_bary.bary_in_array_idx != INDEX_NONE)
        {
            SACell const& bary_cell = grid.cells[found_bary.cell_id];
            int bary_idx = bary_cell.barys[found_bary.bary_in_array_idx];
            
            grid.seg_bary_array[seg_idx] = bary_idx;
            SABarycenterRef& bary_0 = grid.bary_points[bary_idx];
            SABarycenterRef& bary_1 = grid.bary_points[bary_idx + 1];
            
            // project back bary_0 pos onto segment
            float t_seg;
            float sq_dist = SquaredDistancePointSegment_Unclamped(bary_0.pos, p_0, p_1, t_seg);
            vec3 v_seg = p_0 * (1.f - t_seg) + p_1 * t_seg;
            vec3 vec = v_seg - bary_0.pos;
            
            // moving average
            
            bool move_bary = true;
            for( int32 s_idx = 0; s_idx < bary_0.seg_refs.size(); s_idx++)
            {
                int32 seg_ref = bary_0.seg_refs[s_idx];
                if( seg_ref >= seg_idx - 5 && seg_ref <= seg_idx + 5 )
                {
                    move_bary = false;
                    break;
                }
            }
            if (!move_bary)
            {
                bary_0.seg_refs.push_back(seg_idx);
                continue;
            }
            
            bary_0.pos = bary_0.pos + vec / (float)(bary_0.weight + 1);
            bary_0.weight++;
            bary_0.seg_refs.push_back(seg_idx);
            
            grid.MoveBary(found_bary, bary_0.pos, bary_idx);
            
            //TODO: backtrack to see if seg-1 / seg-2 is not snapped on same bary line, but seg-2 / seg-3 is, in which case we force contuinity of snapping
            if( bary_idx > 1)
            {
                //int32 p_b_m1 = grid.seg_bary_array[seg_idx - 1];
                //int32 p_b_m2 = grid.seg_bary_array[seg_idx - 2];
                int32 seg_idx_m2;
                if( INDEX_NONE == grid.FindSegInRange(bary_idx - 1, seg_idx - 1, 2) && INDEX_NONE != (seg_idx_m2 = grid.FindSegInRange(bary_idx - 2 , seg_idx - 2, 2)))
                {
                    int seg_mid = (seg_idx_m2 + seg_idx) / 2;
                    SABarycenterRef& bary_m1 = grid.bary_points[bary_idx - 1];
                    
                    vec3 p_m0 = line_points[seg_mid];
                    vec3 p_m1 = line_points[seg_mid + 1];

                    // project back bary_m1 pos onto segment
                    float t_seg_m1;
                    SquaredDistancePointSegment_Unclamped(bary_m1.pos, p_m0, p_m1, t_seg_m1);
                    vec3 v_seg_m = p_m0 * (1.f - t_seg_m1) + p_m1 * t_seg_m1;
                    vec3 vec_m = v_seg_m - bary_m1.pos;
                    
                    vec3 prev_bary_m1_pos = bary_m1.pos;
                    bary_m1.pos = bary_m1.pos + vec / (float)(bary_m1.weight + 1);
                    bary_m1.weight++;
                    bary_m1.seg_refs.push_back(seg_mid);
                    grid.MoveBary(prev_bary_m1_pos, bary_m1.pos, bary_idx - 1);
                }
            }
            
            //grid.MoveBary(found_bary, bary_0.pos, bary_idx);
            
            if (bary_1.is_last_in_chain)
            {
                // move next bary too if end of chain
                vec3 prev_bary_1_pos = bary_1.pos;
                bary_1.pos = bary_1.pos + vec / (float)(bary_1.weight + 1);
                bary_1.weight++;
                //bary_1.seg_refs.push_back(seg_idx);
                
                grid.MoveBary(prev_bary_1_pos, bary_1.pos, bary_idx + 1);
            }
        }
        //  otherwise, create a new barycenter point / seg with current seg info (first_leading_seg = me, bary pos = seg pos, w = 1)
        else
        {
            SABarycenterRef* prev_bary = nullptr;
            int nb_bary = grid.bary_points.size();
            if (nb_bary)
            {
                prev_bary = &grid.bary_points.Last();
            }
            
            if (prev_bary && prev_bary->pos == p_0)
            {
                // continuing previous chain
                prev_bary->is_last_in_chain = 0;
                prev_bary->seg_refs.push_back(seg_idx);
                grid.seg_bary_array[seg_idx] = nb_bary - 1;
                
                SABarycenterRef new_bary_1 = { seg_idx + 1, p_1, 1 /*weight*/, 1 /*is_last_in_chain*/ };
                grid.bary_points.push_back(new_bary_1);
                int new_cell_id = grid.GetCellIdx(p_1);
                grid.cells[new_cell_id].barys.push_back(nb_bary);
            }
            else
            {
                // start new chain
                SABarycenterRef new_bary_0 = { seg_idx, p_0, 1 /*weight*/, 0 /*is_last_in_chain*/ };
                new_bary_0.seg_refs.push_back(seg_idx);
                grid.bary_points.push_back(new_bary_0);
                grid.bary_chains.push_back(nb_bary);
                grid.cells[cell_id].barys.push_back(nb_bary);
                grid.seg_bary_array[seg_idx] = nb_bary;
                
                SABarycenterRef new_bary_1 = { seg_idx + 1, p_1, 1 /*weight*/, 1 /*is_last_in_chain*/ };
                grid.bary_points.push_back(new_bary_1);
                int new_cell_id = grid.GetCellIdx(p_1);
                grid.cells[new_cell_id].barys.push_back(nb_bary + 1);
            }
        }
    }
    
    // TEMP
    {
        for (int chain_idx = 0; chain_idx < grid.bary_chains.size(); chain_idx++)
        {
            int bary_idx = grid.bary_chains[chain_idx];
            vec3 prev_pos = grid.bary_points[bary_idx].pos;
            float prev2_dist = 0.f;
            float prev_dist = 0.f;
            int print_next = 0;
            
            SABarycenterRef const* prev2_bary = nullptr;
            SABarycenterRef const* prev_bary = nullptr;
            SABarycenterRef const* bary;
            do
            {
                bary = &grid.bary_points[bary_idx++];
                vec3 pos = bary->pos;
                float dist = distance(prev_pos, pos);
                if( prev_dist > 0.f && dist > 5.f * prev_dist)
                {
                    if(prev2_bary)
                        PrintBary(prev2_bary, bary_idx - 2, prev2_dist);
                    if(prev_bary)
                        PrintBary(prev_bary, bary_idx - 1, prev_dist);
                    PrintBary(bary, bary_idx, dist);
                    print_next = 2;
                }
                else if(print_next)
                {
                    PrintBary(bary, bary_idx, dist);
                    print_next--;
                    if(!print_next)
                        BB_LOG(SAUtils, Log, "\n");
                }
                
                prev2_dist = prev_dist;
                prev_dist = dist;
                prev_pos = pos;
                prev2_bary = prev_bary;
                prev_bary = bary;
            }
            while (!bary->is_last_in_chain);
            
            BB_LOG(SAUtils, Log, "Chain %d stops with bary %d\n", chain_idx, bary_idx);
        }
    }
    
    // TEMP debug view
    if( shape_params.show_bary )
    {
        for (int chain_idx = 0; chain_idx < grid.bary_chains.size(); chain_idx++)
        {
            AttractorLineFramed new_framed;
            snapped_lines.push_back(new_framed);
            AttractorLineFramed& ref_framed = snapped_lines.Last();
            
            int bary_idx = grid.bary_chains[chain_idx];
            SABarycenterRef const* bary;
            do
            {
                bary = &grid.bary_points[bary_idx++];
                ref_framed.points.push_back(bary->pos);
                ref_framed.colors.push_back((float)bary->weight);
            }
            while (!bary->is_last_in_chain);
            
            if (ref_framed.points.size() < 4)
            {
                snapped_lines.pop_back();
            }
            else
            {
                GenerateFrames(ref_framed);
            }
        }
    }
    else
    {
        // interpolate weights
        // 3- Parse segment in order, smoothly lerp current blend weight of seg to next barycenter, and compute new position
        //AttractorLineFramed* new_framed = nullptr;
        AttractorLineFramed new_framed;
        snapped_lines.push_back(new_framed);
        AttractorLineFramed& ref_framed = snapped_lines.Last();
        if (nb_points < 2)
        {
            return;
        }
        
        int bary_counter = 0;
        static int max_bary_counter = 30;
        vec3 previous_interp_pos = line_points[0];
        vec3 previous_interp_dir = normalize(line_points[1] - line_points[0]);
        vec3 previous_dir_to_bary = orthogonal(previous_interp_dir);
        vec3 previous_seg_dir = previous_interp_dir;
        quat previous_seg_quat = quat(1.f);
        ref_framed.points.push_back(previous_interp_pos);
        
        for (int seg_idx = 1; seg_idx < nb_points - 1; seg_idx++)
        {
            vec3 pt = line_points[seg_idx];
            vec3 next_pt = line_points[seg_idx + 1];
            vec3 seg_dir = next_pt - pt;
            float seg_len = length(seg_dir);
            seg_dir /= seg_len;
            
            quat seg_quat = quat::rotate(previous_seg_dir, seg_dir);
            
            int bary_idx = grid.seg_bary_array[seg_idx];
            SABarycenterRef& bary_0 = grid.bary_points[bary_idx];
            SABarycenterRef& bary_1 = grid.bary_points[bary_idx + 1];
            
            int target_seg_idx = bigball::min( seg_idx + shape_params.target_bary_offset, nb_points - 2 );
            int target_bary_idx = grid.seg_bary_array[target_seg_idx];
            SABarycenterRef& bary_t = grid.bary_points[target_bary_idx];
            
            float t_bary;
            float sq_dist = SquaredDistancePointSegment_Unclamped(pt, bary_0.pos, bary_1.pos, t_bary);
            float len_to_bary = bigball::sqrt(sq_dist);
            vec3 bary_proj = bary_0.pos * (1.f - t_bary) + bary_1.pos * t_bary;
            vec3 dir_to_bary;
            if (len_to_bary < 1e-3f)
            {
                dir_to_bary = previous_dir_to_bary;
            }
            else
            {
                dir_to_bary = bary_proj - pt;
                dir_to_bary /= len_to_bary;
            }
            
            vec3 left_dir = normalize(cross(dir_to_bary, seg_dir));
            vec3 plane_dir = cross(left_dir, dir_to_bary);
            
            // extrapolate pos from previous_interp_dir
            // fix dir by following seg curve rotation
            vec3 ray_dir = /*shape_params.show_bary ? previous_interp_dir :*/ previous_seg_quat.transform(previous_interp_dir);
            float project_dist = intersect::RayPlaneIntersection(previous_interp_pos, ray_dir, pt, plane_dir);
            
            // compare against avg dist between points, clamp dist as we can move -50% <-> +50%
            float move_dist = bigball::clamp(project_dist, seg_len * 0.5f, seg_len * 1.5f);
            
            // compute project_pos (clamped)
            vec3 project_pos = previous_interp_pos + previous_interp_dir * move_dist;
            
            // compute interp_pos by drifting project_pos towards bary_proj by small amount
            vec3 interp_pos = project_pos;
            
            vec3 drift_dir;
//if(shape_params.show_bary)
//            drift_dir = bary_proj - project_pos;
//else
{
            vec3 target_dir = bary_t.pos - project_pos;
            float dist_target = dot( ray_dir, target_dir );
            drift_dir = target_dir - (ray_dir * dist_target);
}
            float drift_len = length(drift_dir);
            if( drift_len > 1e-3f)
            {
                drift_dir /= drift_len;
                
                const float max_drift = shape_params.max_drift;// 0.01f;
                drift_len = bigball::clamp(drift_len, -max_drift, max_drift);
                interp_pos += drift_dir * drift_len;
            }
            
            ref_framed.points.push_back(interp_pos);
            ref_framed.colors.push_back((float)0.f);
            
            previous_dir_to_bary = dir_to_bary;
            previous_interp_dir = normalize(interp_pos - previous_interp_pos);
            previous_interp_pos = interp_pos;
            previous_seg_dir = seg_dir;
            previous_seg_quat = seg_quat;
        }
        
#if OLD_CODE
        vec3 interp_pos = nb_points ? line_points[0] : vec3(0.f, 0.f, 0.f);
        vec3 project_pos, dir_to_bary(0.f,0.f,1.f);
        vec3 previous_seg_dir = nb_points > 1 ? line_points[1] - line_points[0] : vec3(0.f, 0.f, 0.f);
        float inc = 1.f;
        //float last_sstep = 0.f;
        
        for (int seg_idx = 0; seg_idx < nb_points - 1; seg_idx++)
        {
            vec3 pos = line_points[seg_idx];
            int bary_idx = grid.seg_bary_array[seg_idx];
            SABarycenterRef& bary_0 = grid.bary_points[bary_idx];
            SABarycenterRef& bary_1 = grid.bary_points[bary_idx + 1];
            
            float t_bary;
            float sq_dist = SquaredDistancePointSegment_Unclamped(pos, bary_0.pos, bary_1.pos, t_bary);
            float len_to_bary = bigball::sqrt(sq_dist);
            vec3 bary_proj = bary_0.pos * (1.f - t_bary) + bary_1.pos * t_bary;
            if (len_to_bary < 1e-3f)
            {
                //
                vec3 dir_seg = vec3(0.f,0.f,0.f);
                {
                    if (seg_idx > 0)
                        dir_seg = pos - line_points[seg_idx - 1];
                    dir_seg += line_points[seg_idx + 1] - pos;
                    dir_seg = normalize(dir_seg);
                    vec3 vec_ortho = normalize(cross(dir_seg, dir_to_bary));
                    dir_to_bary = cross(vec_ortho, dir_seg);
                }
            }
            else
            {
                dir_to_bary = bary_proj - pos;
                dir_to_bary /= len_to_bary;
            }
            
            vec3 right_dir = normalize( cross(dir_to_bary, previous_seg_dir) );
            vec3 plane_dir = cross(right_dir, dir_to_bary);
            
            //float sstep = smoothstep((float)bary_counter / (float)max_bary_counter);
            
            // project last interp pos onto dir
            float interp_len = dot(interp_pos - pos, dir_to_bary);
            project_pos = pos + dir_to_bary * interp_len;
            float remaining_dist = len_to_bary - interp_len;
            float current_t = (float)bary_counter / (float)max_bary_counter;
            const float max_inc = 2.f / (float)max_bary_counter;
            float inc = bigball::clamp(remaining_dist, -max_inc, max_inc);
            //if (current_t == 0.f)
            //{
            //
            //}
            //if (current_t < 1.f)
            {
                //float dsstep = 6.f*current_t - 6.f*current_t*current_t;
                interp_pos = project_pos + dir_to_bary * inc;//(remaining_dist * dsstep / (float)max_bary_counter);
            }
            //else
            //{
            //	interp_pos = bary_proj;
            //}
            
            //float dist_H = (len_to_bary - interp_len) / (1.f - last_sstep);
            //interp_pos = pos + dir_to_bary * (dist_H * sstep);
            //float dist_H = (len_to_bary - interp_len) / (1.f - last_sstep);
            //interp_pos = pos + dir_to_bary * (interp_len + dist_H * sstep);
            
            
            ref_framed.points.push_back(interp_pos);
            
            previous_seg_dir = line_points[seg_idx + 1] - pos;
            
            if (bary_1.is_last_in_chain || bary_0.weight != bary_1.weight)
            {
                bary_counter = 0;
                //last_sstep = 0.f;
            }
            else
            {
                bary_counter++;
                //last_sstep = sstep;
            }
        }
#endif // OLD_CODE
        GenerateFrames(ref_framed);
    }
    // compute positions and frames
    //		- if first_leading_seg = me, push pos
}

// Second iteration of MergeLinePoints
void SAUtils::MergeLinePoints2(AttractorLineFramed const& line_framed, const Array<AttractorHandle>& attr_handles, AttractorShapeParams const& shape_params, Array<AttractorLineFramed>& snapped_lines)
{
    BB_LOG(SAUtils, Log, "MergeLinePoints2\n");
    
    // compute current bounds
    Array<AABB> bounds;
    ComputeBounds(line_framed.points, shape_params.merge_dist, bounds);
    int nb_bounds = bounds.size();
    
    const int min_spacing = 9 * SAUtils::points_in_bound;
    
    snapped_lines.push_back(line_framed); // temporary
    
    AttractorSnapRangeEx snap_range;
    ivec2 last_src_points = { -1, -1 };
    
    int b_idx1 = 0;
    while (b_idx1 < nb_bounds)
    {
        BB_LOG(SAUtils, Log, "\rMergeLinePoints2 %d / %d", b_idx1, nb_bounds);
        
        AABB const& b1 = bounds[b_idx1];
        int next_idx = b_idx1 + 1;
        
        // compare with all previous bounds, since we want only to merge with previous lines
        for (int b_idx0 = 0; b_idx0 < b_idx1 - 1; b_idx0++)
        {
            AABB const& b0 = bounds[b_idx0];
            if (AABB::BoundsIntersect(b0, b1))
            {
                // potential merge, check whether at least one segment in b1 is near one in b0
                snap_range.Clear();
                if (FindSnapRangeEx(line_framed.points, b_idx0, b_idx1, shape_params.merge_dist, snap_range))
                {
                    if (snap_range.src_points.y - snap_range.src_points.x >= min_spacing ||	// ignore snap_ranges if they are too short
                        (/*)snap_range.src_points.y >= last_src_points.x && */snap_range.src_points.x > last_src_points.y))
                    {
                        // compute relative weights of snap and compute reverse infos
                        ComputeReverseSnapRangeExInfo(line_framed.points, shape_params.merge_dist, snap_range);
                        
                        AttractorLineFramed& ref_framed = snapped_lines.Last();
                        
                        for (int d_idx = 0; d_idx < snap_range.dst_seg_array.size(); d_idx++)
                        {
                            SnapSegInfo& snap_info = snap_range.dst_seg_array[d_idx];
                            int src_idx = snap_range.src_points.x + d_idx;
                            vec3 src_pos = line_framed.points[src_idx];
                            vec3 dst_pos = line_framed.points[snap_info.seg_idx] * (1.f - snap_info.t_seg) + line_framed.points[snap_info.seg_idx + 1] * snap_info.t_seg;
                            vec3 delta_pos = dst_pos - src_pos;
                            vec3 p = src_pos + delta_pos * snap_info.weight * 0.5f;
                            ref_framed.points[src_idx] = p;
                        }
                        
                        for (int d_idx = 0; d_idx < snap_range.src_seg_array.size(); d_idx++)
                        {
                            SnapSegInfo& snap_info = snap_range.src_seg_array[d_idx];
                            int src_idx = snap_range.dst_segs.x + d_idx;
                            vec3 src_pos = line_framed.points[src_idx];
                            vec3 dst_pos = line_framed.points[snap_info.seg_idx] * (1.f - snap_info.t_seg) + line_framed.points[snap_info.seg_idx + 1] * snap_info.t_seg;
                            vec3 delta_pos = dst_pos - src_pos;
                            vec3 p = src_pos + delta_pos * snap_info.weight * 0.5f;
                            ref_framed.points[src_idx] = p;
                        }
                        
                        
                        
                        // don't snap on an already snapped segment
                        /*int snap_idx = 0;
                         for (; snap_idx < snap_ranges.size(); snap_idx++)
                         {
                         if (!(snap_range.dst_segs.x > snap_ranges[snap_idx].src_points.y || snap_range.dst_segs.y < snap_ranges[snap_idx].src_points.x))
                         break;
                         }
                         
                         if (snap_idx >= snap_ranges.size())
                         {
                         // check that it does not collide with previous range
                         bool skip_insert = false;
                         int snap_idx_comp = snap_ranges.size() - 1;
                         if (snap_idx_comp >= 0)
                         {
                         if (snap_range.src_points.y >= snap_ranges[snap_idx_comp].src_points.x && snap_range.src_points.x <= snap_ranges[snap_idx_comp].src_points.y)
                         {
                         // collision detected
                         int snap_dist = snap_range.src_points.y - snap_range.src_points.x;
                         int snap_dist_comp = snap_ranges[snap_idx_comp].src_points.y - snap_ranges[snap_idx_comp].src_points.x;
                         if (snap_dist_comp > snap_dist)
                         {
                         BB_LOG(SAUtils, Log, "MergeLinePoints ignoring %d snap range\n", snap_ranges.size());
                         skip_insert = true;
                         }
                         else
                         {
                         BB_LOG(SAUtils, Log, "MergeLinePoints replacing %d snap range\n", snap_idx_comp);
                         snap_ranges.erase(snap_idx_comp);
                         }
                         }
                         }
                         
                         if (!skip_insert)
                         snap_ranges.push_back(snap_range);*/
                        
                        last_src_points.x = snap_range.src_points.x;
                        last_src_points.y = snap_range.src_points.y;
                        
                        next_idx = 1 + snap_range.src_points.y / SAUtils::points_in_bound;
                        break;
                        //}
                    }
                }
            }
        }
        
        b_idx1 = next_idx;
    }
}

void SAUtils::MergeLinePoints(AttractorLineFramed const& line_framed, const Array<AttractorHandle>& attr_handles, AttractorShapeParams const& shape_params, Array<AttractorLineFramed>& snapped_lines)
{
    BB_LOG(SAUtils, Log, "MergeLinePoints\n");
    
    // compute bounds
    Array<AABB> bounds;
    ComputeBounds(line_framed.points, shape_params.merge_dist, bounds);
    int nb_bounds = bounds.size();
    
    Array<vec3> modified_points;
    Array<AttractorSnapRange> snap_ranges;
    const int min_spacing = 9 * SAUtils::points_in_bound;
    
    int b_idx1 = 0;
    while (b_idx1 < nb_bounds)
    {
        BB_LOG(SAUtils, Log, "\rMergeLinePoints2 %d / %d", b_idx1, nb_bounds);
        
        AABB const& b1 = bounds[b_idx1];
        int next_idx = b_idx1 + 1;
        
        // compare with all previous bounds, since we want only to merge with previous lines
        for (int b_idx0 = 0; b_idx0 < b_idx1 - 1; b_idx0++)
        {
            AABB const& b0 = bounds[b_idx0];
            if (AABB::BoundsIntersect(b0, b1))
            {
                // potential merge, check whether at least one segment in b1 is near one in b0
                AttractorSnapRange snap_range;
                if (FindSnapRange(line_framed.points, b_idx0, b_idx1, shape_params.merge_dist, snap_range))
                {
                    if (snap_range.src_points.y - snap_range.src_points.x >= min_spacing)	// ignore snap_ranges if they are too short
                    {
                        // don't snap on an already snapped segment
                        int snap_idx = 0;
                        for (; snap_idx < snap_ranges.size(); snap_idx++)
                        {
                            if (!(snap_range.dst_segs.x > snap_ranges[snap_idx].src_points.y || snap_range.dst_segs.y < snap_ranges[snap_idx].src_points.x))
                                break;
                        }
                        
                        if (snap_idx >= snap_ranges.size())
                        {
                            // check that it does not collide with previous range
                            bool skip_insert = false;
                            int snap_idx_comp = snap_ranges.size() - 1;
                            if (snap_idx_comp >= 0)
                            {
                                if (snap_range.src_points.y >= snap_ranges[snap_idx_comp].src_points.x && snap_range.src_points.x <= snap_ranges[snap_idx_comp].src_points.y)
                                {
                                    // collision detected
                                    int snap_dist = snap_range.src_points.y - snap_range.src_points.x;
                                    int snap_dist_comp = snap_ranges[snap_idx_comp].src_points.y - snap_ranges[snap_idx_comp].src_points.x;
                                    if (snap_dist_comp > snap_dist)
                                    {
                                        BB_LOG(SAUtils, Log, "MergeLinePoints ignoring %d snap range\n", snap_ranges.size());
                                        skip_insert = true;
                                    }
                                    else
                                    {
                                        BB_LOG(SAUtils, Log, "MergeLinePoints replacing %d snap range\n", snap_idx_comp);
                                        snap_ranges.erase(snap_idx_comp);
                                    }
                                }
                            }
                            
                            if (!skip_insert)
                                snap_ranges.push_back(snap_range);
                            
                            next_idx = 1 + snap_range.src_points.y / SAUtils::points_in_bound;
                            break;
                        }
                    }
                }
            }
        }
        
        b_idx1 = next_idx;
    }
    
    // check ending snap and whether we need to cut the line endings
    const int nb_range = snap_ranges.size();
    int first_index = -1;
    int last_index = nb_range - 1;
    if (nb_range > 1 && shape_params.remove_line_ends)
    {
        // check if one range snaps onto beginning of curve
        int snap_idx = 0;
        for (snap_idx = 0; snap_idx < nb_range; snap_idx++)
        {
            if (snap_ranges[snap_idx].dst_segs.x == 0)
                break;
        }
        if (snap_idx == nb_range)
            first_index = 0;
        
        // check if ending of curve snaps onto sthg
        if (snap_ranges.Last().src_points.y < line_framed.points.size() - 1)
            last_index = nb_range - 2;
    }
    
    // build snapped_lines array
    if (nb_range > 0)
    {
        snapped_lines.reserve(nb_range + 2);
        
        for (int snap_idx = first_index; snap_idx <= last_index; snap_idx++)
        {
            int start_idx = (snap_idx >= 0 ? snap_ranges[snap_idx].src_points.y : 0);
            int end_idx = (snap_idx < nb_range - 1 ? snap_ranges[snap_idx + 1].src_points.x : line_framed.points.size() - 1);
            
            if (end_idx - start_idx < 3)
                continue;
            
            AttractorLineFramed new_framed;
            snapped_lines.push_back(new_framed);
            AttractorLineFramed& ref_framed = snapped_lines.Last();
            
            int cur_seg_0 = INDEX_NONE;
            const int lerp_spacing = SAUtils::points_in_bound * 7 / 2;
            if (snap_idx >= 0 && shape_params.snap_interp)
            {
                // from snapped to unsnapped (i.e. from unplugged to plugged)
                cur_seg_0 = snap_ranges[snap_idx].dst_segs.y;
                for (int d_idx = 1; d_idx <= lerp_spacing; d_idx++)
                {
                    int src_idx = start_idx - d_idx;
                    float best_t, ratio_blend = smoothstep(float(d_idx) / float(lerp_spacing));
                    cur_seg_0 = FindNextBestSnapSeg(line_framed.points, src_idx, cur_seg_0, -1 /*inc*/, 1e8f, best_t);
                    if (cur_seg_0 == INDEX_NONE)
                        break;	// oups, should not happen
                    
                    vec3 src_pos = line_framed.points[src_idx];
                    vec3 dst_pos = line_framed.points[cur_seg_0] * (1.f - best_t) + line_framed.points[cur_seg_0 + 1] * best_t;
                    vec3 delta_pos = dst_pos - src_pos;
                    vec3 p = src_pos + delta_pos * ratio_blend;
                    
                    ref_framed.points.insert(p, 0);
                    ref_framed.frames.insert(line_framed.frames[src_idx], 0);
                    ref_framed.follow_angles.insert(line_framed.follow_angles[src_idx], 0);
                }
            }
            
            ref_framed.snap_ranges.x = 0;
            ref_framed.snap_ranges.y = ref_framed.points.size();
            
            // copy
            for (int idx = start_idx; idx <= end_idx; idx++)
            {
                ref_framed.points.push_back(line_framed.points[idx]);
                ref_framed.frames.push_back(line_framed.frames[idx]);
                ref_framed.follow_angles.push_back(line_framed.follow_angles[idx]);
            }
            ref_framed.snap_ranges.z = ref_framed.points.size();
            
            if (cur_seg_0 != INDEX_NONE)
            {
                quat src_frame = line_framed.frames[cur_seg_0];
                float src_follow_angle = line_framed.follow_angles[cur_seg_0];
                quat dst_frame = line_framed.frames[start_idx];
                float dst_follow_angle = line_framed.follow_angles[start_idx];
                vec3 src_follow, dst_follow;
                FindNearestFollowVector(dst_frame, dst_follow_angle, src_frame, src_follow_angle, shape_params.local_edge_count, dst_follow, src_follow);
                
                GenerateFrames(ref_framed, 0, lerp_spacing - 1, false, true /*use continuity from last copy*/, &src_follow, &dst_follow);
            }
            
            if (snap_idx < nb_range - 1 && shape_params.snap_interp)
            {
                // from unsnapped to snapped (i.e. from plugged to unplugged)
                //int prev_size = ref_framed.points.size();
                int cur_seg_0 = snap_ranges[snap_idx + 1].dst_segs.x;
                for (int d_idx = 1; d_idx <= lerp_spacing; d_idx++)
                {
                    int src_idx = end_idx + d_idx;
                    float best_t, ratio_blend = smoothstep(float(d_idx) / float(lerp_spacing));
                    cur_seg_0 = FindNextBestSnapSeg(line_framed.points, src_idx, cur_seg_0, 1 /*inc*/, 1e8f, best_t);
                    if (cur_seg_0 == INDEX_NONE)
                        break;	// oups, should not happen
                    
                    vec3 src_pos = line_framed.points[src_idx];
                    vec3 dst_pos = line_framed.points[cur_seg_0] * (1.f - best_t) + line_framed.points[cur_seg_0 + 1] * best_t;
                    vec3 delta_pos = dst_pos - src_pos;
                    vec3 p = src_pos + delta_pos * ratio_blend;
                    ref_framed.points.push_back(p);
                    ref_framed.frames.push_back(line_framed.frames[src_idx]);
                    ref_framed.follow_angles.push_back(line_framed.follow_angles[src_idx]);
                }
                
                if (cur_seg_0 != INDEX_NONE)
                {
                    quat src_frame = line_framed.frames[end_idx];
                    float src_follow_angle = line_framed.follow_angles[end_idx];
                    quat dst_frame = line_framed.frames[cur_seg_0];
                    float dst_follow_angle = line_framed.follow_angles[cur_seg_0];
                    vec3 src_follow, dst_follow;
                    FindNearestFollowVector(src_frame, src_follow_angle, dst_frame, dst_follow_angle, shape_params.local_edge_count, src_follow, dst_follow);
                    
                    GenerateFrames(ref_framed, ref_framed.points.size() - 1 - lerp_spacing, ref_framed.points.size() - 1, true /*use continuity from last copy*/, false, &src_follow, &dst_follow);
                }
            }
            
            ref_framed.snap_ranges.w = ref_framed.points.size();
        }
    }
    else
    {
        snapped_lines.push_back(line_framed);
    }
}

// takes a continuous line and a bunch of snap ranges, output an array of framed lines
// note that: 
//	- it does not generate frames where lines are snapped
//	- handles the case where multiple lines would snap on the same area / segments
//  - suppose line_points was already modified / interpolated; not line_frames nor follow_angles
//  - if blend_positions is true, resulting points in framed_lines will be smoothed out otherwise line_points positions will be copied as is
void SAUtils::GenerateSnappedLinesWithFrames(const Array<vec3>& line_points, const Array<quat>& line_frames, const Array<float>& follow_angles, const Array<AttractorSnapRange>& snap_ranges, AttractorShapeParams const& shape_params, Array<AttractorLineFramed>& framed_lines /*out*/, const int32 interp_spacing, const bool blend_positions)
{
	// sort snap_ranges by src_points
	Array<AttractorSnapRange> snap_ranges_sorted = snap_ranges;
	sort(&snap_ranges_sorted[0], snap_ranges_sorted.size(),
			[](const AttractorSnapRange & a, const AttractorSnapRange & b) -> bool
			{
				return a.src_points.x < b.src_points.x;
			});

	// check ending snap and whether we need to cut the line endings
	const int nb_range = snap_ranges.size();
	int first_index = -1;
	int last_index = nb_range - 1;
	if (nb_range > 0)
	{
		// check if beginning of curve snaps onto sthg
		first_index = snap_ranges[0].src_points.x == 0 ? 0 : -1;

		// check if ending of curve snaps onto sthg
		last_index = snap_ranges.Last().src_points.y == line_points.size() - 1 ? nb_range - 2 : nb_range - 1;
	}

	// build snapped_lines array
	if (nb_range > 0)
	{
		framed_lines.reserve(nb_range + 2);

		for (int snap_idx = first_index; snap_idx <= last_index; snap_idx++)
		{
			int start_idx = (snap_idx >= 0 ? snap_ranges[snap_idx].src_points.y : 0);
			int end_idx = (snap_idx < nb_range - 1 ? snap_ranges[snap_idx + 1].src_points.x : line_points.size() - 1);

			if (end_idx - start_idx < 3)
				continue;

			AttractorLineFramed new_framed;
			framed_lines.push_back(new_framed);
			AttractorLineFramed& cur_framed = framed_lines.Last();

			int cur_seg_0 = INDEX_NONE;
			//const int lerp_spacing = SAUtils::points_in_bound * 7 / 2;
			if (snap_idx >= 0)
			{
				// from snapped to unsnapped [S -> U] (i.e. from unplugged to plugged)
				cur_seg_0 = snap_ranges[snap_idx].dst_segs.y;
				for (int d_idx = 1; d_idx <= interp_spacing; d_idx++)
				{
					int src_idx = start_idx - d_idx;
					float best_t;
					cur_seg_0 = FindNextBestSnapSeg(line_points, src_idx, cur_seg_0, -1 /*inc*/, 1e8f, best_t);
					if (cur_seg_0 == INDEX_NONE)
						break;	// oups, should not happen

					vec3 src_pos = line_points[src_idx];
					vec3 p = src_pos;
					if (blend_positions)
					{
						float ratio_blend = smoothstep(float(d_idx) / float(interp_spacing));
						vec3 dst_pos = line_points[cur_seg_0] * (1.f - best_t) + line_points[cur_seg_0 + 1] * best_t;
						vec3 delta_pos = dst_pos - src_pos;
						p += delta_pos * ratio_blend;
					}

					cur_framed.points.insert(p, 0);
					cur_framed.frames.insert(line_frames[src_idx], 0);
					cur_framed.follow_angles.insert(follow_angles[src_idx], 0);
				}
			}

			cur_framed.snap_ranges.x = 0;
			cur_framed.snap_ranges.y = cur_framed.points.size();

			// copy for points in-between snap / smooth sections [U -> U]
			for (int idx = start_idx; idx <= end_idx; idx++)
			{
				cur_framed.points.push_back(line_points[idx]);
				cur_framed.frames.push_back(line_frames[idx]);
				cur_framed.follow_angles.push_back(follow_angles[idx]);
			}
			cur_framed.snap_ranges.z = cur_framed.points.size();

			if (cur_seg_0 != INDEX_NONE)
			{
				quat src_frame = line_frames[cur_seg_0];
				float src_follow_angle = follow_angles[cur_seg_0];
				quat dst_frame = line_frames[start_idx];
				float dst_follow_angle = follow_angles[start_idx];
				vec3 src_follow, dst_follow;
				FindNearestFollowVector(dst_frame, dst_follow_angle, src_frame, src_follow_angle, shape_params.local_edge_count, dst_follow, src_follow);

				// generate frames and follow_angles for the first section [SU]
				// we needed to wait for the in-between copy because of the continuity flag below
				GenerateFrames(cur_framed, 0, interp_spacing - 1, false, true /*use continuity from last copy*/, &src_follow, &dst_follow);
			}

			if (snap_idx < nb_range - 1)
			{
				// from unsnapped to snapped [U -> S] (i.e. from plugged to unplugged)
				//int prev_size = cur_framed.points.size();
				int cur_seg_0 = snap_ranges[snap_idx + 1].dst_segs.x;
				for (int d_idx = 1; d_idx <= interp_spacing; d_idx++)
				{
					int src_idx = end_idx + d_idx;
					float best_t;
					cur_seg_0 = FindNextBestSnapSeg(line_points, src_idx, cur_seg_0, 1 /*inc*/, 1e8f, best_t);
					if (cur_seg_0 == INDEX_NONE)
						break;	// oups, should not happen

					vec3 src_pos = line_points[src_idx];
					vec3 p = src_pos;
					if (blend_positions)
					{
						float ratio_blend = smoothstep(float(d_idx) / float(interp_spacing));
						vec3 dst_pos = line_points[cur_seg_0] * (1.f - best_t) + line_points[cur_seg_0 + 1] * best_t;
						vec3 delta_pos = dst_pos - src_pos;
						p += delta_pos * ratio_blend;
					}
					cur_framed.points.push_back(p);
					cur_framed.frames.push_back(line_frames[src_idx]);
					cur_framed.follow_angles.push_back(follow_angles[src_idx]);
				}

				if (cur_seg_0 != INDEX_NONE)
				{
					quat src_frame = line_frames[end_idx];
					float src_follow_angle = follow_angles[end_idx];
					quat dst_frame = line_frames[cur_seg_0];
					float dst_follow_angle = follow_angles[cur_seg_0];
					vec3 src_follow, dst_follow;
					FindNearestFollowVector(src_frame, src_follow_angle, dst_frame, dst_follow_angle, shape_params.local_edge_count, src_follow, dst_follow);

					GenerateFrames(cur_framed, cur_framed.points.size() - 1 - interp_spacing, cur_framed.points.size() - 1, true /*use continuity from last copy*/, false, &src_follow, &dst_follow);
				}
			}

            cur_framed.colors.resize(line_points.size(), 0.f);
			cur_framed.snap_ranges.w = cur_framed.points.size();
		}
	}
	else
	{
		AttractorLineFramed new_framed;
		framed_lines.push_back(new_framed);
		AttractorLineFramed& cur_framed = framed_lines.Last();
		cur_framed.points = line_points;
        cur_framed.colors.resize(line_points.size(), 0.f);

		GenerateFrames(cur_framed);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

int32 FindSnapFromSrc(int32 src_idx, Array<AttractorSnapRange> const& snap_ranges)
{
    for (int32 snap_idx = 0; snap_idx < snap_ranges.size(); ++snap_idx)
    {
        if (src_idx >= snap_ranges[snap_idx].src_points.x && src_idx <= snap_ranges[snap_idx].src_points.y)
            return snap_idx;
    }
    return INDEX_NONE;
}

void FindRefSnaps(int32 dst_idx, Array<AttractorSnapRange> const& snap_ranges, Array<RefSnap>& snaps)
{
    for (int32 snap_idx = 0; snap_idx < snap_ranges.size(); ++snap_idx)
    {
        if (dst_idx >= snap_ranges[snap_idx].dst_segs.x && dst_idx <= snap_ranges[snap_idx].dst_segs.y)
        {
            
        }
    }
}

void FindAllRefs(int32 p_idx, Array<AttractorSnapRange> const& snap_ranges, Array<vec3> const& points, Array<int32> const& snap_segs)
{
    Array<RefSnap> snapref_array;
    if (snap_segs[p_idx] != INDEX_NONE)
    {
        int32 snap_idx = FindSnapFromSrc(p_idx, snap_ranges);
        BB_ASSERT(snap_idx != INDEX_NONE);
        float t_seg;
        intersect::SquaredDistancePointSegment(points[p_idx], points[snap_segs[p_idx]], points[snap_segs[p_idx] + 1], t_seg);
        RefSnap new_ref = { snap_idx, p_idx, snap_segs[p_idx], t_seg };
        snapref_array.push_back(new_ref);
    }
    
    int32 loop_idx = 0;
    while (loop_idx < snapref_array.size())
    {
        RefSnap const& snap_ref = snapref_array[loop_idx];
        
        // down search
        {
            int32 p_dn[2] = { snap_ref.seg_idx, snap_ref.seg_idx + 1 };
            vec3 seg_pos = points[p_dn[0]] * (1.f - snap_ref.t_seg) + points[p_dn[1]] * snap_ref.t_seg;
            float t[2], d[2] = { FLT_MAX, FLT_MAX };
            if (snap_segs[p_dn[0]] != INDEX_NONE)
                d[0] = intersect::SquaredDistancePointSegment(seg_pos, points[snap_segs[p_dn[0]]], points[snap_segs[p_dn[0]] + 1], t[0]);
            
            if (snap_segs[p_dn[1]] != INDEX_NONE)
                d[1] = intersect::SquaredDistancePointSegment(seg_pos, points[snap_segs[p_dn[1]]], points[snap_segs[p_dn[1]] + 1], t[1]);
            
            if (d[0] != FLT_MAX && d[1] != FLT_MAX)
            {
                int32 best_idx = (d[0] < d[1] ? 0 : 1);
                int32 snap_idx = FindSnapFromSrc(p_dn[best_idx], snap_ranges);
                BB_ASSERT(snap_idx != INDEX_NONE);
                RefSnap new_ref = { snap_idx, p_dn[best_idx], snap_segs[p_dn[best_idx]], t[best_idx] };
                // TODO: push only if not already present
                snapref_array.push_back(new_ref);
            }
        }
        
        // top search: find all snap referencing our point
        {
            
        }
    }
    
    // find all segments to where this point should snap
    //struct { p8idx + t}
}

bool SAUtils::FindSnapRangeEx(const Array<vec3>& line_points, int b_idx0, int b_idx1, float merge_dist, AttractorSnapRangeEx& snap_range)
{
    int start_0 = bigball::max(0, b_idx0 * SAUtils::points_in_bound - 1);
    int end_0 = bigball::min((b_idx0 + 1) * SAUtils::points_in_bound + 1, line_points.size());
    
    int start_1 = b_idx1 * SAUtils::points_in_bound;
    int end_1 = bigball::min((b_idx1 + 1) * SAUtils::points_in_bound, line_points.size());
    
    // find one segment in b1 that is closest to b0 chain than merge_dist
    const float sq_merge_dist = merge_dist * merge_dist;
    float t, sq_dist, min_sq_dist;
    int min_seg_0 = INDEX_NONE, min_seg_1 = INDEX_NONE, prev_seg_0 = INDEX_NONE;
    for (int p_1 = start_1; p_1 < end_1; p_1++)
    {
        min_sq_dist = FLT_MAX;
        for (int seg_0 = start_0; seg_0 < end_0 - 1; seg_0++)
        {
            sq_dist = intersect::SquaredDistancePointSegment(line_points[p_1], line_points[seg_0], line_points[seg_0 + 1], t);
            if (sq_dist < sq_merge_dist)
            {
                min_sq_dist = sq_dist;
                min_seg_0 = seg_0;
                break;
            }
        }
        if (min_sq_dist < sq_merge_dist)
        {
            if (prev_seg_0 != INDEX_NONE)
            {
                // found two successive points < merge_dist
                min_seg_1 = p_1 - 1;
                break;
            }
            else
                prev_seg_0 = min_seg_0;
        }
        else
            prev_seg_0 = INDEX_NONE;
    }
    
    if (min_seg_1 == INDEX_NONE)
        return false;
    
    // find out if lines are reversed or not
    vec3 dir_0 = line_points[min_seg_0 + 1] - line_points[min_seg_0];
    vec3 dir_1 = line_points[min_seg_1 + 1] - line_points[min_seg_1];
    int inc = bigball::dot(dir_0, dir_1) > 0.f ? 1 : -1;
    if (inc < 0)
        return false;	// don't want to merge lines in opposite directions
    
    snap_range.dst_segs.x = prev_seg_0;
    snap_range.dst_segs.y = min_seg_0;
    snap_range.src_points.x = min_seg_1;
    snap_range.src_points.y = min_seg_1 + 1;
    // extend lines on both sides so as to get the full chain
    const int nb_points = line_points.size();
    
    // left (r_1.x)
    {
        int cur_seg_0 = snap_range.dst_segs.x;
        int c_1_next = snap_range.src_points.x;
        while (c_1_next >= 0)
        {
            float best_t;
            cur_seg_0 = FindNextBestSnapSeg(line_points, c_1_next, cur_seg_0, -1 /*inc*/, sq_merge_dist, best_t);
            if (cur_seg_0 != INDEX_NONE)
            {
                snap_range.src_points.x = c_1_next--;
                snap_range.dst_segs.x = cur_seg_0;
                snap_range.dst_seg_array.insert(SnapSegInfo{ cur_seg_0, best_t, 0.f }, 0);
            }
            else
            {
                break;
            }
        }
    }
    
    // right (r_1.y)
    {
        int cur_seg_0 = snap_range.dst_segs.y;
        int c_1_next = snap_range.src_points.y;
        while (c_1_next < nb_points)
        {
            float best_t;
            cur_seg_0 = FindNextBestSnapSeg(line_points, c_1_next, cur_seg_0, 1 /*inc*/, sq_merge_dist, best_t);
            if (cur_seg_0 != INDEX_NONE)
            {
                snap_range.src_points.y = c_1_next++;
                snap_range.dst_segs.y = cur_seg_0;
                snap_range.dst_seg_array.push_back(SnapSegInfo{ cur_seg_0, best_t, 0.f });
            }
            else
            {
                break;
            }
        }
    }
    
    return true;
}

bool SAUtils::ComputeReverseSnapRangeExInfo(const Array<vec3>& points, float merge_dist, AttractorSnapRangeEx& snap_range)
{
    // we want to find reverse snapping info from the snap_range.dst_segs.x to snap_range.dst_segs.y + 1
    float t[2], d[2] = { FLT_MAX, FLT_MAX };
    if (snap_range.src_points.x > 0)
        d[0] = intersect::SquaredDistancePointSegment(points[snap_range.dst_segs.x], points[snap_range.src_points.x - 1], points[snap_range.src_points.x], t[0]);
    
    if (snap_range.src_points.x < points.size() - 1)
        d[1] = intersect::SquaredDistancePointSegment(points[snap_range.dst_segs.x], points[snap_range.src_points.x], points[snap_range.src_points.x + 1], t[1]);
    
    if (d[0] == FLT_MAX && d[1] == FLT_MAX)
        return false;
    
    int32 cur_seg_0 = (d[0] < d[1] ? snap_range.src_points.x - 1 : snap_range.src_points.x);
    
    for (int dst_idx = snap_range.dst_segs.x; dst_idx <= snap_range.dst_segs.y + 1; dst_idx++)
    {
        float best_t;
        cur_seg_0 = FindNextBestSnapSeg(points, dst_idx, cur_seg_0, 1 /*inc*/, 1e8f, best_t);
        if (cur_seg_0 == INDEX_NONE)
        {
            return false;	// oups, should not happen
        }
        
        snap_range.src_seg_array.push_back(SnapSegInfo{ cur_seg_0, best_t, 0.f });
    }
    
    // compute weighting info
    static float test = 2.f;
    float res = smoothstep(test);
    const int lerp_spacing = SAUtils::points_in_bound * 7 / 2;
    {
        const int dst_seg_count = snap_range.dst_seg_array.size();
        const int dst_half_count = dst_seg_count / 2;
        for (int d_idx = 0; d_idx < dst_half_count; d_idx++)
        {
            SnapSegInfo& snap_info = snap_range.dst_seg_array[d_idx];
            float ratio_blend = bigball::min(smoothstep(float(d_idx) / float(lerp_spacing)), 1.f);
            snap_info.weight = ratio_blend;
        }
        for (int d_idx = dst_half_count; d_idx < dst_seg_count; d_idx++)
        {
            SnapSegInfo& snap_info = snap_range.dst_seg_array[d_idx];
            float ratio_blend = bigball::min(smoothstep(float(dst_seg_count - d_idx - 1) / float(lerp_spacing)), 1.f);
            snap_info.weight = ratio_blend;
        }
    }
    
    // compute reverse weighting
    {
        const int src_seg_count = snap_range.src_seg_array.size();
        for (int d_idx = 0; d_idx < src_seg_count; d_idx++)
        {
            SnapSegInfo& rev_snap_info = snap_range.src_seg_array[d_idx];
            if (rev_snap_info.seg_idx < snap_range.src_points.x || rev_snap_info.seg_idx >= snap_range.src_points.y)
                rev_snap_info.weight = 0.f;
            else
            {
                int d_idx = rev_snap_info.seg_idx - snap_range.src_points.x;
                SnapSegInfo& snap_info_0 = snap_range.dst_seg_array[d_idx];
                SnapSegInfo& snap_info_1 = snap_range.dst_seg_array[d_idx + 1];
                rev_snap_info.weight = snap_info_0.weight + rev_snap_info.t_seg * (snap_info_1.weight - snap_info_0.weight);
            }
        }
    }
    
    return true;
}

bool SAUtils::FindSnapRange(const Array<vec3>& line_points, int b_idx0, int b_idx1, float merge_dist, AttractorSnapRange& snap_range)
{
    int start_0 = bigball::max(0, b_idx0 * SAUtils::points_in_bound - 1);
    int end_0 = bigball::min((b_idx0 + 1) * SAUtils::points_in_bound + 1, line_points.size());
    
    int start_1 = b_idx1 * SAUtils::points_in_bound;
    int end_1 = bigball::min((b_idx1 + 1) * SAUtils::points_in_bound, line_points.size());
    
    // find one segment in b1 that is closest to b0 chain than merge_dist
    const float sq_merge_dist = merge_dist * merge_dist;
    float t, sq_dist, min_sq_dist;
    int min_seg_0 = INDEX_NONE, min_seg_1 = INDEX_NONE, prev_seg_0 = INDEX_NONE;
    for (int p_1 = start_1; p_1 < end_1; p_1++)
    {
        min_sq_dist = FLT_MAX;
        for (int seg_0 = start_0; seg_0 < end_0 - 1; seg_0++)
        {
            sq_dist = intersect::SquaredDistancePointSegment(line_points[p_1], line_points[seg_0], line_points[seg_0 + 1], t);
            if (sq_dist < sq_merge_dist)
            {
                min_sq_dist = sq_dist;
                min_seg_0 = seg_0;
                break;
            }
        }
        if (min_sq_dist < sq_merge_dist)
        {
            if (prev_seg_0 != INDEX_NONE)
            {
                // found two successive points < merge_dist
                min_seg_1 = p_1 - 1;
                break;
            }
            else
                prev_seg_0 = min_seg_0;
        }
        else
            prev_seg_0 = INDEX_NONE;
    }
    
    if (min_seg_1 == INDEX_NONE)
        return false;
    
    // find out if lines are reversed or not
    vec3 dir_0 = line_points[min_seg_0 + 1] - line_points[min_seg_0];
    vec3 dir_1 = line_points[min_seg_1 + 1] - line_points[min_seg_1];
    int inc = bigball::dot(dir_0, dir_1) > 0.f ? 1 : -1;
    snap_range.dst_segs.x = prev_seg_0;
    snap_range.dst_segs.y = min_seg_0;
    snap_range.src_points.x = min_seg_1;
    snap_range.src_points.y = min_seg_1 + 1;
    
    if (inc < 0)
        return false;	// don't want to merge lines in opposite directions
    
    // extend lines on both sides so as to get the full chain
    const int nb_points = line_points.size();
    
    // left (r_1.x)
    {
        int cur_seg_0 = snap_range.dst_segs.x;
        int& c_1 = snap_range.src_points.x;
        int c_1_next = c_1;
        while (c_1_next >= 0)
        {
            float best_t;
            cur_seg_0 = FindNextBestSnapSeg(line_points, c_1_next, cur_seg_0, -1 /*inc*/, sq_merge_dist, best_t);
            if (cur_seg_0 != INDEX_NONE &&
                (cur_seg_0 < snap_range.src_points.x || cur_seg_0 > snap_range.src_points.y))	// prevent line from wrapping and snapping on itself
            {
                c_1 = c_1_next--;
                snap_range.dst_segs.x = cur_seg_0;
            }
            else
            {
                break;
            }
        }
    }
    
    // right (r_1.y)
    {
        int cur_seg_0 = snap_range.dst_segs.y;
        int& c_1 = snap_range.src_points.y;
        int c_1_next = c_1;
        while (c_1_next < nb_points)
        {
            float best_t;
            cur_seg_0 = FindNextBestSnapSeg(line_points, c_1_next, cur_seg_0, 1 /*inc*/, sq_merge_dist, best_t);
            if (cur_seg_0 != INDEX_NONE &&
                (cur_seg_0 < snap_range.src_points.x || cur_seg_0 > snap_range.src_points.y))	// prevent line from wrapping and snapping on itself
            {
                c_1 = c_1_next++;
                snap_range.dst_segs.y = cur_seg_0;
            }
            else
            {
                break;
            }
        }
    }
    
    return true;
}

int SAUtils::FindNextBestSnapSeg(const Array<vec3>& line_points, int c_1_next, int cur_seg_0, int inc, float sq_merge_dist, float& best_t)
{
    int best_seg = INDEX_NONE;
    const int max_iter = 6;
    for (int iter = 0; iter < max_iter; iter++)
    {
        float t, sq_dist = intersect::SquaredDistancePointSegment(line_points[c_1_next], line_points[cur_seg_0], line_points[cur_seg_0 + 1], t);
        if (sq_dist < sq_merge_dist)
        {
            // move on to next seg
            sq_merge_dist = sq_dist;
            best_t = t;
            best_seg = cur_seg_0;
            
            cur_seg_0 = bigball::clamp(cur_seg_0 + inc, 0, line_points.size() - 2);
        }
        else
            break;
    }
    
    return best_seg;
}

void SAUtils::GenerateSolidMesh(Array<AttractorLineFramed> const& snapped_lines, const AttractorShapeParams& params, Array<vec3>& tri_vertices /*out*/, Array<vec3>* tri_normals /*out*/, Array<float>* tri_colors /*out*/, Array<int32>& tri_indices /*out*/)
{
    Array<vec3> local_shape;
    GenerateLocalShape(local_shape, params);
    const int32 num_local_points = local_shape.size();
    
    for (int line_idx = 0; line_idx < snapped_lines.size(); line_idx++)
    {
        AttractorLineFramed const & line_framed = snapped_lines[line_idx];
        
        int32 base_vertex = tri_vertices.size();
        GenerateTriVertices(tri_vertices, tri_normals, tri_colors, local_shape, line_framed, params);
        
        GenerateTriIndices(tri_vertices, num_local_points, tri_indices, params.weld_vertex, base_vertex);
    }
}



//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void SAUtils::GenerateColors(AttractorLineFramed& line_framed, float color)
{
    int32 nb_points = line_framed.points.size();
    line_framed.colors.resize(nb_points);
    for (int32 i = 0; i < nb_points; i++)
    {
        line_framed.colors[i] = color;
    }
}

void SAUtils::GenerateFrames(AttractorLineFramed& line_framed)
{
    vec3 P_prev, P_current, P_next, vz_follow;
    vec3 vy_prev(1.f, 0.f, 0.f);
    
    Array<vec3> const& line_points = line_framed.points;
    Array<quat>& frames = line_framed.frames;
    Array<float>& follow_angles = line_framed.follow_angles;
    
    line_framed.frames.resize(line_points.size());
    line_framed.follow_angles.resize(line_points.size());
    
    for (int32 i = 1; i < line_points.size() - 1; i++)
    {
        P_prev = line_points[i - 1];
        P_current = line_points[i];
        P_next = line_points[i + 1];
        vec3 V0 = P_current - P_prev;
        vec3 V1 = P_next - P_current;
        float V0_len = length(V0);
        vec3 V0N = V0 / V0_len;
        vec3 V1N = normalize(V1);
        vec3 vy = cross(V1N, V0N);    // up vector
        float vy_len = length(vy);
        if (vy_len > 1e-4)
            vy /= vy_len;
        else
            vy = vy_prev;
        
        vec3 V0RightN = cross(V0N, vy);
        vec3 V1RightN = cross(V1N, vy);
        vec3 vz = normalize(V0RightN + V1RightN); // right vector
        vec3 vx = cross(vy, vz);      // front vector
        frames[i] = mat3(vx, vy, vz);
        
        // Replace vz_follow inside vy, vz frame and remove vx component
        if (i == 1)
            vz_follow = vz;
        
        float dot_x = dot(vx, vz_follow);
        if (bigball::abs(dot_x) > 0.99f)
            int32 Break = 0;
        vz_follow -= vx * dot_x;
        vz_follow = normalize(vz_follow);
        
        float dot_y = dot(vy, vz_follow);
        float dot_z = dot(vz, vz_follow);
        float delta_angle = bigball::atan2(dot_y, dot_z);
        follow_angles[i] = delta_angle;
        
        vy_prev = vy;
    }
    frames[0] = frames[1];
    frames[line_points.size() - 1] = frames[line_points.size() - 2];
    follow_angles[0] = follow_angles[1];
    follow_angles[line_points.size() - 1] = follow_angles[line_points.size() - 2];
}

void SAUtils::GenerateFrames(AttractorLineFramed& line_framed, int from_idx, int to_idx, bool start_continuity, bool end_continuity, vec3* start_vector, vec3* end_vector)
{
    vec3 P_prev, P_current, P_next, vz_follow;
    vec3 vy_prev(1.f, 0.f, 0.f);
    
    Array<vec3> const& line_points = line_framed.points;
    Array<quat>& frames = line_framed.frames;
    Array<float>& follow_angles = line_framed.follow_angles;
    
    int start_idx = (start_continuity ? from_idx : from_idx + 1);
    int end_idx = (end_continuity ? to_idx : to_idx - 1);
    quat delta_rot(1.f);
    bool constrained_vectors = (start_vector != nullptr && end_vector != nullptr) ? true : false;
    if (constrained_vectors)
        delta_rot = quat::rotate(*start_vector, *end_vector);
    
    for (int32 i = start_idx; i <= end_idx; i++)
    {
        P_prev = line_points[i - 1];
        P_current = line_points[i];
        P_next = line_points[i + 1];
        vec3 V0 = P_current - P_prev;
        vec3 V1 = P_next - P_current;
        float V0_len = length(V0);
        vec3 V0N = V0 / V0_len;
        vec3 V1N = normalize(V1);
        vec3 vy = cross(V1N, V0N);    // up vector
        float vy_len = length(vy);
        if (vy_len > 1e-4)
            vy /= vy_len;
        else
            vy = vy_prev;
        
        vec3 V0RightN = cross(V0N, vy);
        vec3 V1RightN = cross(V1N, vy);
        vec3 vz = normalize(V0RightN + V1RightN); // right vector
        vec3 vx = cross(vy, vz);      // front vector
        frames[i] = mat3(vx, vy, vz);
        
        if (constrained_vectors)
        {
            // Simply interpolate between requested vectors
            float ratio = smoothstep(float(i - start_idx) / float(end_idx - start_idx));
            quat ratio_quat = slerp(quat(1.f), delta_rot, ratio);
            ratio_quat = normalize(ratio_quat);
            vz_follow = ratio_quat.transform(*start_vector);
        }
        else
        {
            if (i == start_idx)
                vz_follow = vz;
        }
        
        // Replace vz_follow inside vy, vz frame and remove vx component
        float dot_x = dot(vx, vz_follow);
        if (bigball::abs(dot_x) > 0.99f)
            int32 Break = 0;
        vz_follow -= vx * dot_x;
        vz_follow = normalize(vz_follow);
        
        float dot_y = dot(vy, vz_follow);
        float dot_z = dot(vz, vz_follow);
        float delta_angle = bigball::atan2(dot_y, dot_z);
        follow_angles[i] = delta_angle;
        
        vy_prev = vy;
    }
    if (!start_continuity)
        frames[from_idx] = frames[from_idx + 1];
    if (!end_continuity)
        frames[to_idx] = frames[to_idx - 1];
}

void SAUtils::GenerateTriVertices(Array<vec3>& tri_vertices, Array<vec3>* tri_normals, Array<float>* tri_colors, const Array<vec3>& local_shape, AttractorLineFramed const & line_framed, const AttractorShapeParams& params)
{
    const Array<vec3>& line_points = line_framed.points;
    const Array<float>& colors = line_framed.colors;
    const Array<quat>& frames = line_framed.frames;
    const Array<float>& follow_angles = line_framed.follow_angles;
    
    const int32 base_vertex = tri_vertices.size();
    vec3 P_prev, P_current, P_next, VX_follow;
    
    const int32 num_local_points = local_shape.size();
    const int32 line_inc = (params.simplify_level > 1 ? params.simplify_level : 1);
    Array<vec3> rotated_shape;
    rotated_shape.resize(num_local_points);
    for (int32 i = 1; i < line_points.size() - 1; i += line_inc)
    {
        mat3 mat(frames[i]);
        vec3 vx = mat.v0;   // FRONT
        vec3 vy = mat.v1;   // UP
        vec3 vz = mat.v2;   // RIGHT
        
        float color = colors[i];
        //if (i < line_framed.snap_ranges.y) // snap out
        //    color = lerp(-1.f, 0.f, float(i - line_framed.snap_ranges.x) / float(line_framed.snap_ranges.y - line_framed.snap_ranges.x));
        //else if (i >= line_framed.snap_ranges.z) // snap in
        //    color = lerp(0.f, 1.f, float(i - line_framed.snap_ranges.z) / float(line_framed.snap_ranges.w - line_framed.snap_ranges.z));
        
        P_prev = line_points[i - 1];
        P_current = line_points[i];
        //P_next = merge_line_points[i+1];
        vec3 V0 = P_current - P_prev;
        //vec3 V1 = P_next - P_current;
        float V0_len = length(V0);
        vec3 V0N = V0 / V0_len;
        vec3 V0RightN = cross(V0N, vy);
        
        // Twist local shape with follow vec inside current frame VX, VY, VZ
        float delta_angle = follow_angles[i];
        float cf = bigball::cos(delta_angle);
        float sf = bigball::sin(delta_angle);
        
        float cos_alpha = dot(V0RightN, vz);
        float scale = 1.0f / cos_alpha;
        for (int32 j = 0; j < num_local_points; ++j)
        {
            float local_z = local_shape[j].x * cf - local_shape[j].z * sf;
            float local_y = local_shape[j].x * sf + local_shape[j].z * cf;
            rotated_shape[j] = P_current + vz * (local_z * (scale * params.fatness_scale)) + vx * (local_shape[j].y * params.fatness_scale) + vy * (local_y * params.fatness_scale);
        }
        
        for (int32 j = 0; j < num_local_points; ++j)
        {
            vec3 P_shape = rotated_shape[j];
            
            if (!params.weld_vertex)
            {
                tri_vertices.push_back(P_shape);
                tri_vertices.push_back(P_shape);
                if (tri_normals)
                {
                    vec3 P_shape_prev = rotated_shape[(j + num_local_points - 1) % num_local_points];
                    vec3 P_shape_next = rotated_shape[(j + 1) % num_local_points];
                    vec3 temp_V = P_shape - P_current*2.f;
                    vec3 normal_prev = normalize(temp_V + P_shape_prev);
                    vec3 normal_next = normalize(temp_V + P_shape_next);
                    tri_normals->push_back(normal_prev);
                    tri_normals->push_back(normal_next);
                }
                if (tri_colors)
                {
                    tri_colors->push_back(color);
                    tri_colors->push_back(color);
                }
            }
            else
            {
                tri_vertices.push_back(P_shape);
                if (tri_normals)
                {
                    vec3 normal_P = normalize(P_shape - P_current);
                    tri_normals->push_back(normal_P);
                }
                if (tri_colors)
                {
                    tri_colors->push_back(color);
                }
            }
        }
        
        //VLastZ = VZ;
    }
    
    // For capping...
    int32 ShapeWOCapCount = tri_vertices.size() - base_vertex;
    tri_vertices.push_back(line_points[1]);
    vec3 start_normal = normalize(line_points[0] - line_points[1]);
    if (tri_normals)
        tri_normals->push_back(start_normal);
    if (tri_colors)
        tri_colors->push_back(-1.f);
    
    if (!params.weld_vertex)
    {
        // Push start vertices
        for (int32 j = 0; j < num_local_points; ++j)
            tri_vertices.push_back(tri_vertices[base_vertex + 2 * j]);
        
        if (tri_normals)
        {
            for (int32 j = 0; j < num_local_points; ++j)
                tri_normals->push_back(start_normal);
        }
        if (tri_colors)
        {
            for (int32 j = 0; j < num_local_points; ++j)
                tri_colors->push_back(-1.f);
        }
    }
    
    tri_vertices.push_back(line_points[line_points.size() - 2]);
    vec3 end_normal = normalize(line_points[line_points.size() - 1] - line_points[line_points.size() - 2]);
    if (tri_normals)
        tri_normals->push_back(end_normal);
    if (tri_colors)
        tri_colors->push_back(1.f);
    if (!params.weld_vertex)
    {
        // Push end vertices
        for (int32 j = 0; j < num_local_points; ++j)
            tri_vertices.push_back(tri_vertices[base_vertex + ShapeWOCapCount - 2 * num_local_points + 2 * j]);
        
        if (tri_normals)
        {
            for (int32 j = 0; j < num_local_points; ++j)
                tri_normals->push_back(end_normal);
        }
        if (tri_colors)
        {
            for (int32 j = 0; j < num_local_points; ++j)
                tri_colors->push_back(1.f);
        }
    }
}

void SAUtils::GenerateTriIndices(const Array<vec3>& tri_vertices, int32 num_local_points, Array<int32>& tri_indices, const bool bWeldVertex, int32 base_vertex)
{
    // Generate tri index
    if (bWeldVertex)
    {
        const int32 nShape = (tri_vertices.size() - base_vertex) / num_local_points;
        for (int32 i = 0; i<nShape - 1; i++)
        {
            for (int32 j = 0; j < num_local_points; ++j)
            {
                tri_indices.push_back(base_vertex + i * num_local_points + j);
                tri_indices.push_back(base_vertex + (i + 1) * num_local_points + j);
                tri_indices.push_back(base_vertex + i * num_local_points + (1 + j) % num_local_points);
                
                tri_indices.push_back(base_vertex + i * num_local_points + (1 + j) % num_local_points);
                tri_indices.push_back(base_vertex + (i + 1) * num_local_points + j);
                tri_indices.push_back(base_vertex + (i + 1) * num_local_points + (1 + j) % num_local_points);
                
            }
        }
        
        // First cap
        int32 nShapeSize = tri_vertices.size() - base_vertex;
        int32 Cap0 = nShapeSize - 2;
        int32 Cap1 = nShapeSize - 1;
        
        for (int32 j = 0; j < num_local_points; ++j)
        {
            tri_indices.push_back(base_vertex + Cap0);
            tri_indices.push_back(base_vertex + 0 * num_local_points + j);
            tri_indices.push_back(base_vertex + 0 * num_local_points + (1 + j) % num_local_points);
        }
        
        // Second cap
        for (int32 j = 0; j < num_local_points; ++j)
        {
            tri_indices.push_back(base_vertex + Cap1);
            tri_indices.push_back(base_vertex + (nShape - 1) * num_local_points + (1 + j) % num_local_points);
            tri_indices.push_back(base_vertex + (nShape - 1) * num_local_points + j);
        }
    }
    else
    {
        const int32 nShape = (tri_vertices.size() - base_vertex) / (2 * num_local_points) - 1;
        for (int32 i = 0; i<nShape - 1; i++)
        {
            for (int32 j = 0; j < num_local_points; ++j)
            {
                tri_indices.push_back(base_vertex + i * 2 * num_local_points + (2 * j + 1));
                tri_indices.push_back(base_vertex + (i + 1) * 2 * num_local_points + (2 * j + 1));
                tri_indices.push_back(base_vertex + i * 2 * num_local_points + (2 * j + 2) % (2 * num_local_points));
                
                tri_indices.push_back(base_vertex + i * 2 * num_local_points + (2 * j + 2) % (2 * num_local_points));
                tri_indices.push_back(base_vertex + (i + 1) * 2 * num_local_points + (2 * j + 1));
                tri_indices.push_back(base_vertex + (i + 1) * 2 * num_local_points + (2 * j + 2) % (2 * num_local_points));
                
            }
        }
        
        // First cap
        int32 nShapeSize = tri_vertices.size() - base_vertex;
        int32 Cap0 = nShapeSize - 2 - 2 * num_local_points;
        int32 Cap1 = nShapeSize - 1 - num_local_points;
        
        for (int32 j = 0; j < num_local_points; ++j)
        {
            tri_indices.push_back(base_vertex + Cap0);
            tri_indices.push_back(base_vertex + Cap0 + 1 + j);
            tri_indices.push_back(base_vertex + Cap0 + 1 + (j + 1) % num_local_points);
            //tri_indices.push_back( 0 * 2*num_local_points + (2*j+1) );
            //tri_indices.push_back( 0 * 2*num_local_points + (2*j+2) % (2*num_local_points) );
        }
        
        // Second cap
        for (int32 j = 0; j < num_local_points; ++j)
        {
            tri_indices.push_back(base_vertex + Cap1);
            tri_indices.push_back(base_vertex + Cap1 + 1 + (j + 1) % num_local_points);
            tri_indices.push_back(base_vertex + Cap1 + 1 + j);
            //tri_indices.push_back( (nShape-1) * 2*num_local_points + (2*j+2) % (2*num_local_points) );
            //tri_indices.push_back( (nShape-1) * 2*num_local_points + (2*j+1) );
        }
    }
}

void SAUtils::WriteObjFile( Archive& file, const Array<vec3>& tri_vertices, const Array<int32>& tri_indices )
{
    const char* filename = "strange attractor";
    String tmp_str;
    tmp_str = String::Printf("#\n# %s\n#\n\n", filename);
    file.SerializeString(tmp_str);
    //fopen_s( &fp, filename, "wt" );
    //fprintf( fp, "#\n# %s\n#\n\n", filename );
    
    int32 i, num_points = tri_vertices.size(), num_tri = tri_indices.size() / 3;
    for (i = 0; i < num_points; ++i)
    {
        tmp_str = String::Printf("v %f %f %f\n", tri_vertices[i].x, tri_vertices[i].y, tri_vertices[i].z);
        file.SerializeString(tmp_str);
        //fprintf( fp, "v %f %f %f\n", vPos[i].x, vPos[i].y, vPos[i].z );
    }
    
    for (i = 0; i < num_tri; ++i)
    {
        tmp_str = String::Printf("f %d %d %d\n", 1 + tri_indices[3*i], 1 + tri_indices[3*i+1], 1 + tri_indices[3*i+2] );
        file.SerializeString(tmp_str);
        //fprintf( fp, "f %d %d %d\n", 1 + tri_indices[3*i], 1 + tri_indices[3*i+1], 1 + tri_indices[3*i+2] );
    }
    
    //fclose( fp );
}

#if 0
void SAUtils::WriteObjFile(const char* filename, const Array<AttractorShape>& vAllShapes)
{
    File fp;
    if (!fp.Open(filename, true))
        return;
    //FILE* fp = NULL;
    //fopen_s( &fp, filename, "wt" );
    String tmp_str;
    tmp_str = String::Printf("#\n# %s\n#\n\n", filename);
    fp.SerializeString(tmp_str);
    //fprintf( fp, "#\n# %s\n#\n\n", filename );
    
    // Write vertices
    for (int32 ShapeIdx = 0; ShapeIdx < vAllShapes.size(); ShapeIdx++)
    {
        const AttractorShape& S = vAllShapes[ShapeIdx];
        int32 i, nPart = S.tri_vertices.size();
        for (i = 0; i < nPart; ++i)
        {
            tmp_str = String::Printf("v %f %f %f\n", S.tri_vertices[i].x, S.tri_vertices[i].y, S.tri_vertices[i].z);
            fp.SerializeString(tmp_str);
            //fprintf( fp, "v %f %f %f\n", S.vShape[i].x, S.vShape[i].y, S.vShape[i].z );
        }
    }
    
    // Write faces
    int32 nVertexOffset = 0;
    for (int32 ShapeIdx = 0; ShapeIdx < vAllShapes.size(); ShapeIdx++)
    {
        const AttractorShape& S = vAllShapes[ShapeIdx];
        int32 i, nTri = S.tri_indices.size() / 3;
        for (i = 0; i < nTri; ++i)
        {
            tmp_str = String::Printf("f %d %d %d\n", nVertexOffset + 1 + S.tri_indices[3 * i], nVertexOffset + 1 + S.tri_indices[3 * i + 1], nVertexOffset + 1 + S.tri_indices[3 * i + 2]);
            fp.SerializeString(tmp_str);
            //fprintf( fp, "f %d %d %d\n", nVertexOffset + 1 + S.tri_indices[3*i], nVertexOffset + 1 + S.tri_indices[3*i+1], nVertexOffset + 1 + S.tri_indices[3*i+2] );
        }
        
        nVertexOffset += S.tri_vertices.size();
    }
    
    //fclose( fp );
}
#endif // 0

void SAUtils::GenerateLocalShape(Array<vec3>& local_shapes, const AttractorShapeParams& params)
{
    bool is_simple_mesh = (params.crease_width <= 0.0 || params.crease_depth <= 0.0 ? true : false);
    const int32 nLocalPoint = (is_simple_mesh ? 1 : 5) * params.local_edge_count;
    local_shapes.resize(nLocalPoint);
    const float fatness = 1.0f;
    
    if (is_simple_mesh)
    {
        for (int32 edge_idx = 0; edge_idx < params.local_edge_count; ++edge_idx)
        {
            float Angle = 2.0f * F_PI * (float)edge_idx / (float)(params.local_edge_count);
            local_shapes[edge_idx] = vec3(fatness*bigball::cos(Angle), 0.0f, fatness*bigball::sin(Angle));
        }
    }
    else
    {
        //const float min_wall = 0.7f;
        //float MinCreaseWidth = min_wall*0.5f / bigball::sin( F_PI / params.local_edge_count );
        
        
        for (int32 edge_idx = 0; edge_idx < params.local_edge_count; ++edge_idx)
        {
            float Angle = 2.0f * F_PI * (float)edge_idx / (float)(params.local_edge_count);
            local_shapes[5 * edge_idx] = vec3(fatness*bigball::cos(Angle), 0.0, fatness*bigball::sin(Angle));
        }
        
        vec3 V;
        for (int32 edge_idx = 0; edge_idx < params.local_edge_count; ++edge_idx)
        {
            const vec3& Prev = local_shapes[5 * edge_idx];
            const vec3& Next = local_shapes[5 * (edge_idx + 1) % nLocalPoint];
            V = Next - Prev;
            
            local_shapes[5 * edge_idx + 1] = Prev + V * params.crease_width;
            local_shapes[5 * edge_idx + 2] = (Prev + V * (params.crease_width + params.crease_bevel)) * (1.0f - params.crease_depth);
            local_shapes[5 * edge_idx + 3] = (Prev + V * (1.0f - (params.crease_width + params.crease_bevel))) * (1.0f - params.crease_depth);
            local_shapes[5 * edge_idx + 4] = Prev + V * (1.0f - params.crease_width);
        }
    }
}

int32 SAUtils::FindNearestPoint(const Array<vec3>& line_points, int32 PointIdx, int32 IgnoreStart, int32 IgnoreEnd)
{
    vec3 V;
    vec3 PointToFind = line_points[PointIdx];
    float NearestDist = FLT_MAX;
    float Len;
    int32 Neighbour = PointIdx;
    for (int32 i = 0; i<line_points.size(); ++i)
    {
        V = PointToFind - line_points[i];
        Len = length(V);
        if (Len < NearestDist)
        {
            if (i < IgnoreStart || i > IgnoreEnd)
            {
                NearestDist = Len;
                Neighbour = i;
            }
        }
    }
    return Neighbour;
}


void SAUtils::FindNearestFollowVector(quat const& src_frame, float src_follow_angle, quat const& dst_frame, float dst_follow_angle, int32 local_edge_count, vec3& src_follow, vec3& dst_follow)
{
    mat3 src_mat(src_frame);
    //vec3 src_vx = src_mat.v0;   // FRONT
    vec3 src_vy = src_mat.v1;   // UP
    vec3 src_vz = src_mat.v2;   // RIGHT
    float src_cf = bigball::cos(src_follow_angle);
    float src_sf = bigball::sin(src_follow_angle);
    src_follow = src_vz * src_cf + src_vy * src_sf;
    
    mat3 dst_mat(dst_frame);
    //vec3 dst_vx = dst_mat.v0;   // FRONT
    vec3 dst_vy = dst_mat.v1;   // UP
    vec3 dst_vz = dst_mat.v2;   // RIGHT
    
    dst_follow = vec3(0.f, 0.f, 0.f);
    float best_dot = -FLT_MAX;
    for (int32 i = 0; i < local_edge_count; ++i)
    {
        float dst_angle = dst_follow_angle + 2.0f * F_PI * (float)(i) / (float)(local_edge_count);
        float cf = bigball::cos(dst_angle);
        float sf = bigball::sin(dst_angle);
        vec3 new_follow = dst_vz * cf + dst_vy * sf;
        float dot_follow = dot(new_follow, src_follow);
        if (dot_follow > best_dot)
        {
            best_dot = dot_follow;
            dst_follow = new_follow;
        }
    }
}

#if 0
void SAUtils::MergeLinePointsOld(const Array<vec3>& line_points, const Array<vec3>& VX_follow_array, const Array<vec3>& VX_array, const Array<vec3>& VZ_array, Array<vec3>& vMergePoints, Array<vec3>& vMergeFollow, const AttractorShapeParams& params)
{
    vMergePoints = line_points;
    vMergeFollow = VX_follow_array;
    
    // Manage start
    if (params.merge_start >= 1)
    {
        // Find point nearest to us
        int32 StartIdx = 0;
        int32 Neighbour = FindNearestPoint(line_points, StartIdx, StartIdx, StartIdx + params.merge_start);
        
        // Check by which side we should go
        int32 Inc = 1;
        if (Neighbour >= line_points.size() - params.merge_start)
        {
            Inc = -1;
        }
        else if (Neighbour <= params.merge_start)
        {
            Inc = 1;
        }
        else
        {
            vec3 V0, V1;
            V0 = line_points[StartIdx + 1] - line_points[Neighbour + 1];
            V1 = line_points[StartIdx + 1] - line_points[Neighbour - 1];
            if (length(V0) < length(V1))
                Inc = 1;
            else
                Inc = -1;
        }
        
        // Merge start
        const int32 Margin = 2;
        for (int32 Offset = 0; Offset < params.merge_start; Offset++)
        {
            float MergeRatio = (Offset < Margin) ? 1.0f : (1.0f - (float)(Offset - Margin) / (float)(params.merge_start - Margin));
            vMergePoints[StartIdx + Offset] = line_points[StartIdx + Offset] * (1.0f - MergeRatio) + line_points[Neighbour + Offset*Inc] * MergeRatio;
            
            vec3 NeighbourFollow = FindNearestFollowVector(VX_follow_array[StartIdx + Offset], VX_follow_array[Neighbour + Offset*Inc], VX_array[Neighbour + Offset*Inc], VZ_array[Neighbour + Offset*Inc], params.local_edge_count);
            vMergeFollow[StartIdx + Offset] = VX_follow_array[StartIdx + Offset] * (1.0f - MergeRatio) + NeighbourFollow * MergeRatio;
            vMergeFollow[StartIdx + Offset] = normalize(vMergeFollow[StartIdx + Offset]);
        }
    }
    
    // Manage end
    if (params.merge_end >= 1)
    {
        // Find point nearest to us
        int32 StartIdx = line_points.size() - 1 - params.merge_end;
        int32 Neighbour = FindNearestPoint(line_points, StartIdx, StartIdx - params.merge_end, StartIdx + params.merge_end);
        
        // Check by which side we should go
        int32 Inc = 1;
        if (Neighbour >= line_points.size() - params.merge_end)
        {
            Inc = -1;
        }
        else if (Neighbour <= params.merge_end)
        {
            Inc = 1;
        }
        else
        {
            vec3 V0, V1;
            V0 = line_points[StartIdx + 1] - line_points[Neighbour + 1];
            V1 = line_points[StartIdx + 1] - line_points[Neighbour - 1];
            if (length(V0) < length(V1))
                Inc = 1;
            else
                Inc = -1;
        }
        
        // Merge end
        const int32 Margin = 2;
        for (int32 Offset = 0; Offset < params.merge_end; Offset++)
        {
            float MergeRatio = (Offset >= params.merge_end - Margin) ? 1.0f : ((float)(Offset) / (float)(params.merge_end - Margin));
            vMergePoints[StartIdx + Offset] = line_points[StartIdx + Offset] * (1.0f - MergeRatio) + line_points[Neighbour + Offset*Inc] * MergeRatio;
            
            vec3 NeighbourFollow = FindNearestFollowVector(VX_follow_array[StartIdx + Offset], VX_follow_array[Neighbour + Offset*Inc], VX_array[Neighbour + Offset*Inc], VZ_array[Neighbour + Offset*Inc], params.local_edge_count);
            vMergeFollow[StartIdx + Offset] = VX_follow_array[StartIdx + Offset] * (1.0f - MergeRatio) + NeighbourFollow * MergeRatio;
            vMergeFollow[StartIdx + Offset] = normalize(vMergeFollow[StartIdx + Offset]);
        }
    }
}
#endif // 0

StrangeAttractor* SAUtils::CreateAttractorType(String const& attractor_name)
{
    if (attractor_name == "Lorentz")
        return CreateAttractorType(eAttractor_Lorentz);
    if (attractor_name == "Aizawa")
        return CreateAttractorType(eAttractor_Aizawa);
    if (attractor_name == "TSUCS2")
        return CreateAttractorType(eAttractor_TSUCS2);
    if (attractor_name == "Arnoedo")
        return CreateAttractorType(eAttractor_Arneodo);
    if (attractor_name == "ChenLee")
        return CreateAttractorType(eAttractor_ChenLee);
    if (attractor_name == "DequanLi")
        return CreateAttractorType(eAttractor_DequanLi);
    if (attractor_name == "LorentzMod2")
        return CreateAttractorType(eAttractor_LorentzMod2);
    if (attractor_name == "SpiralTest")
        return CreateAttractorType(eAttractor_SpiralTest);
    
    return nullptr;
}

StrangeAttractor* SAUtils::CreateAttractorType(eAttractorType attractor_type)
{
    if (attractor_type == eAttractor_Lorentz)
        return new LorenzAttractor;
    if (attractor_type == eAttractor_Aizawa)
        return new AizawaAttractor;
    if (attractor_type == eAttractor_TSUCS2)
        return new TSUCS2Attractor;
    if (attractor_type == eAttractor_Arneodo)
        return new ArneodoAttractor;
    if (attractor_type == eAttractor_ChenLee)
        return new ChenLeeAttractor;
    if (attractor_type == eAttractor_DequanLi)
        return new DequanLiAttractor;
    if (attractor_type == eAttractor_LorentzMod2)
        return new LorentzMod2Attractor;
    if (attractor_type == eAttractor_SpiralTest)
        return new SpiralTestAttractor;
    
    return nullptr;
}

void SAUtils::GetAttractorTypeList(Array<String>& attractor_names)
{
    attractor_names.push_back("Lorentz");
    attractor_names.push_back("Aizawa");
    attractor_names.push_back("TSUCS2");
    attractor_names.push_back("Arnoedo");
    attractor_names.push_back("ChenLee");
    attractor_names.push_back("DequanLi");
    attractor_names.push_back("LorentzMod2");
    attractor_names.push_back("SpiralTest");
}

void SAUtils::ComputeBounds(const Array<vec3>& line_points, float margin, Array<AABB>& bounds)
{
    int nb_bounds = (line_points.size() + points_in_bound - 1) / points_in_bound;
    bounds.resize(nb_bounds);
    
    for (int b = 0; b < nb_bounds; b++)
    {
        vec3 min = vec3(FLT_MAX, FLT_MAX, FLT_MAX);
        vec3 max = vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        
        //int start, end;
        //GetBoundPointIndices(b, start, end);
        int start = b * SAUtils::points_in_bound;
        int end = bigball::min(start + SAUtils::points_in_bound, line_points.size());
        
        for (int p_idx = start; p_idx < end; p_idx++)
        {
            min = bigball::min(line_points[p_idx], min);
            max = bigball::max(line_points[p_idx], max);
        }
        
        bounds[b].min = min + vec3(-margin);
        bounds[b].max = max + vec3(margin);
    }
}

bool AABB::BoundsIntersect(AABB const& a, AABB const& b)
{
    if (a.max.x < b.min.x || a.max.y < b.min.y || a.max.z < b.min.z ||
        b.max.x < a.min.x || b.max.y < a.min.y || b.max.z < a.min.z)
        return false;
    
    return true;
}

void SAGrid::InitGrid(const Array<vec3>& line_points, int max_cell)
{
    // compute grid bound
    vec3 min = vec3(FLT_MAX, FLT_MAX, FLT_MAX);
    vec3 max = vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    
    const int nb_points = line_points.size();
    for (int p_idx = 0; p_idx < nb_points; p_idx++)
    {
        min = bigball::min(line_points[p_idx], min);
        max = bigball::max(line_points[p_idx], max);
    }
    
    grid_bound.min = min;
    grid_bound.max = max;
    vec3 diff = max - min;
    
    cell_unit = bigball::pow((diff.x * diff.y * diff.z) / max_cell, 0.33333333f);
    grid_dim.x = (int)(diff.x / cell_unit + 0.99999f);
    grid_dim.y = (int)(diff.y / cell_unit + 0.99999f);
    grid_dim.z = (int)(diff.z / cell_unit + 0.99999f);
    int real_max_cell = grid_dim.x * grid_dim.y * grid_dim.z;
    
    
    // init grid
    cells.resize(real_max_cell);
    seg_bary_array.clear();
    seg_bary_array.resize(nb_points, INDEX_NONE);
    
    // insert segments into grid
    // assume that segments are stored into cells by their first point, even if second point is in another cell
    for (int p_idx = 0; p_idx < nb_points - 1; p_idx++)
    {
        int cell_id = GetCellIdx(line_points[p_idx]);
        cells[cell_id].segs.push_back(p_idx);
    }
}

SABaryResult SAGrid::FindBaryCenterSeg(int seg_idx, vec3 p_0/*, vec3 p_1*/, float max_dist)
{
    //Array<SACell> cells;
    //Array<SABarycenterRef> bary_points;
    //Array<int> bary_chains;
    //Array<int> seg_bary_array;
    //float cell_unit;
    //ivec3 grid_dim;
    //AABB grid_bound;
    
    int i = (int)((p_0.x - grid_bound.min.x) / cell_unit);
    int j = (int)((p_0.y - grid_bound.min.y) / cell_unit);
    int k = (int)((p_0.z - grid_bound.min.z) / cell_unit);
    int i0 = max(0, i - 1); int i1 = min(grid_dim.x - 1, i + 1);
    int j0 = max(0, j - 1); int j1 = min(grid_dim.y - 1, j + 1);
    int k0 = max(0, k - 1); int k1 = min(grid_dim.z - 1, k + 1);
    const float sq_max_dist = max_dist * max_dist;
    
    // search 3x3x3 cells in neighborhood
    SABaryResult found_bary = { INDEX_NONE, INDEX_NONE };
    //int min_bary_idx = INDEX_NONE;
    float sq_min_dist = sq_max_dist;
    
    for (int z = k0; z <= k1; z++)
    {
        for (int y = j0; y <= j1; y++)
        {
            for (int x = i0; x <= i1; x++)
            {
                int cell_id = x + grid_dim.x * y + grid_dim.x * grid_dim.y * z;
                SACell const& cell = cells[cell_id];
                for (int bary_it = 0; bary_it < cell.barys.size(); bary_it++)
                {
                    int bary_idx = cell.barys[bary_it];
                    SABarycenterRef const& b_0 = bary_points[bary_idx];
                    if (b_0.is_last_in_chain)
                        continue;
                    SABarycenterRef const& b_1 = bary_points[bary_idx + 1];
                    //vec3 b_vec = b_1.pos - b_0.pos;
                    
                    float t_seg;
                    float sq_dist = SquaredDistancePointSegment_Unclamped(p_0, b_0.pos, b_1.pos, t_seg);
                    if (sq_dist < sq_min_dist)
                    {
                        if (t_seg >= 0.f && t_seg < 1.f)
                        {
                            sq_min_dist = sq_dist;
                            found_bary.cell_id = cell_id;
                            found_bary.bary_in_array_idx = bary_it;
                        }
                    }
                }
            }
        }
    }
    
    return found_bary;
}

SASegResult SAGrid::FindNearestSeg(int p_idx, const Array<vec3>& line_points, float max_dist, int exclude_range)
{
    vec3 p_0 = line_points[p_idx];
    int i = (int)((p_0.x - grid_bound.min.x) / cell_unit);
    int j = (int)((p_0.y - grid_bound.min.y) / cell_unit);
    int k = (int)((p_0.z - grid_bound.min.z) / cell_unit);
    int i0 = max(0, i - 1); int i1 = min(grid_dim.x - 1, i + 1);
    int j0 = max(0, j - 1); int j1 = min(grid_dim.y - 1, j + 1);
    int k0 = max(0, k - 1); int k1 = min(grid_dim.z - 1, k + 1);
    const float sq_max_dist = max_dist * max_dist;
    
    // search 3x3x3 cells in neighborhood
    SASegResult found_seg = { INDEX_NONE, INDEX_NONE, 0.f, 0.f };
    float sq_min_dist = sq_max_dist;
    
    for (int z = k0; z <= k1; z++)
    {
        for (int y = j0; y <= j1; y++)
        {
            for (int x = i0; x <= i1; x++)
            {
                int cell_id = x + grid_dim.x * y + grid_dim.x * grid_dim.y * z;
                SACell const& cell = cells[cell_id];
                for (int seg_it = 0; seg_it < cell.segs.size(); seg_it++)
                {
                    int seg_idx = cell.segs[seg_it];
                    if(seg_idx > p_idx - exclude_range && seg_idx < p_idx + exclude_range)
                        continue;
                    
                    vec3 s_0 = line_points[seg_idx];
                    vec3 s_1 = line_points[seg_idx + 1];
                    
                    float t_seg;
                    float sq_dist = intersect::SquaredDistancePointSegment(p_0, s_0, s_1, t_seg);
                    if (sq_dist < sq_min_dist)
                    {
                        if (t_seg >= 0.f && t_seg < 1.f)
                        {
                            sq_min_dist = sq_dist;
                            found_seg.cell_id = cell_id;
                            found_seg.seg_in_array_idx = seg_it;
                            found_seg.t_seg = t_seg;
                            found_seg.sq_dist = sq_dist;
                        }
                    }
                }
            }
        }
    }
    
    return found_seg;
}

int32 SAGrid::FindSegInRange(int32 bary_idx, int32 seg_idx, int32 range)
{
    SABarycenterRef const& bary = bary_points[bary_idx];
    
    int32 found_idx = INDEX_NONE;
    for( int32 s_idx = 0; s_idx < bary.seg_refs.size(); s_idx++)
    {
        int32 seg_ref = bary.seg_refs[s_idx];
        if( seg_ref >= seg_idx - range && seg_ref <= seg_idx + range )
        {
            found_idx = max(found_idx, seg_ref);
        }
    }
    return found_idx;
}

void SAGrid::MoveBary(SABaryResult const& bary_ref, vec3 bary_pos, int bary_idx)
{
    int new_bary_cell_id = GetCellIdx(bary_pos);
    if (new_bary_cell_id != bary_ref.cell_id)
    {
        cells[bary_ref.cell_id].barys.erase(bary_ref.bary_in_array_idx);
        cells[new_bary_cell_id].barys.push_back(bary_idx);
    }
}

void SAGrid::MoveBary(vec3 old_bary_pos, vec3 bary_pos, int bary_idx)
{
    int old_bary_cell_id = GetCellIdx(old_bary_pos);
    int new_bary_cell_id = GetCellIdx(bary_pos);
    if (new_bary_cell_id != old_bary_cell_id)
    {
        cells[old_bary_cell_id].barys.remove(bary_idx);
        cells[new_bary_cell_id].barys.push_back(bary_idx);
    }
}

void SAGrid::MovePoint(vec3 old_pos, vec3 new_pos, int pt_idx)
{
    int old_cell_id = GetCellIdx(old_pos);
    int new_cell_id = GetCellIdx(new_pos);
    if (new_cell_id != old_cell_id)
    {
        cells[old_cell_id].segs.remove(pt_idx);
        cells[new_cell_id].segs.push_back(pt_idx);
    }
}

int SAGrid::GetCellIdx(vec3 p) const
{
    int i = (int)((p.x - grid_bound.min.x) / cell_unit);
    int j = (int)((p.y - grid_bound.min.y) / cell_unit);
    int k = (int)((p.z - grid_bound.min.z) / cell_unit);
    int cell_id = i + grid_dim.x * j + grid_dim.x * grid_dim.y * k;
    return cell_id;
}

void AttractorShapeParams::Serialize(Archive& file)
{
    int32 size = (int32)((int8*)&max_drift - (int8*)this);
    file.Serialize( this, size );
}

