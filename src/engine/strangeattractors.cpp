
#include "../sae.h"
#include "strangeattractors.h"
#include "system/file.h"
#include "math/intersections.h"


//void SAUtils::ComputeStrangeAttractorGradient()
//{
//	AttractorShape S0;
//	Array<AttractorShape> init_shapes;
//
//	AizawaAttractor att0;
//	S0.attractor = &att0;
//
//	AttractorShapeParams params;
//	Array<vec3> local_shape;
//	GenerateLocalShape( local_shape, params );
//	const int32 num_local_points = local_shape.size();
//
//	S0.attractor->Loop(S0.line_points, 3000);
//
//	init_shapes.resize( S0.line_points.size() / 10 );
//    
//    Array<quat> frames;
//    Array<float> follow_angles;
//	for( int32 ShapeIdx = 0; ShapeIdx < init_shapes.size(); ShapeIdx++ )
//	{	
//		AttractorShape& S = init_shapes[ShapeIdx];
//		S.attractor = &att0;
//		S.attractor->m_init_point = S0.line_points[10*ShapeIdx];
//		S.attractor->LoopGradient(S.line_points, 200);
//        
//        GenerateFrames(S.line_points, frames, follow_angles);
//
//		// Generate shapes
//		params.fatness_scale = S.attractor->m_fatness_scale;
//		params.weld_vertex = true;
//		GenerateTriVertices(S.tri_vertices, nullptr, local_shape, S.line_points, frames, follow_angles, params);
//	}
//
//	// Generate tri index
//	GenerateTriIndices( init_shapes, num_local_points );
//
//	std::string filename = "StrangeAttractor_";
//	filename += att0.GetClassName();
//	filename += "_gradient.obj";
//	WriteObjFile( filename.c_str(), init_shapes );
//}
//
//void SAUtils::ComputeStrangeAttractorCurl()
//{
//	AttractorShape S0;
//	Array<AttractorShape> init_shapes;
//
//	//AizawaAttractor att0;
//	//att0.m_init_point = vec3( 0.1f, 0, 0 );
//
//	LorenzAttractor att0;
//	S0.attractor = &att0;
//
//	AttractorShapeParams params;
//	Array<vec3> local_shape;
//	GenerateLocalShape( local_shape, params );
//	const int32 num_local_points = local_shape.size();
//
//	S0.attractor->Loop(S0.line_points, 2000);
//
//	init_shapes.resize( S0.line_points.size() / 10 );
//    
//    Array<quat> frames;
//    Array<float> follow_angles;
//	for( int32 ShapeIdx = 0; ShapeIdx < init_shapes.size(); ShapeIdx++ )
//	{	
//		AttractorShape& S = init_shapes[ShapeIdx];
//		S.attractor = &att0;
//		S.attractor->m_init_point = S0.line_points[10*ShapeIdx];
//		S.attractor->LoopCurl( S.line_points, 200 );
//        
//        GenerateFrames(S.line_points, frames, follow_angles);
//
//		// Generate shapes
//		params.fatness_scale = S.attractor->m_fatness_scale;
//		params.weld_vertex = true;
//		GenerateTriVertices(S.tri_vertices, nullptr, local_shape, S.line_points, frames, follow_angles, params);
//	}
//
//	// Generate tri index
//	GenerateTriIndices( init_shapes, num_local_points );
//
//	std::string filename = "StrangeAttractor_";
//	filename += att0.GetClassName();
//	filename += "_curl.obj";
//	WriteObjFile( filename.c_str(), init_shapes );
//}

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
        for( int32 i=0; i < nb_point/2; i++ )
            line_points.Swap( i, nb_point - 1 - i );
		line_points.push_back( seed );

		attractor->m_dt = -attractor->m_dt;
	}

	attractor->LoopAdaptative(line_points, params.iter);

	//if( skip_iter > 0 )
	//{
	//	skip_iter = min( skip_iter, (int32)line_points.size()-2 );
 //       line_points.erase(0,skip_iter);
	//}

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
		vec3 min_pos( FLT_MAX, FLT_MAX, FLT_MAX ), max_pos( -FLT_MAX, -FLT_MAX, -FLT_MAX );
		for( int32 i = 0; i < line_points.size(); ++i )
		{
			min_pos.x = bigball::min( min_pos.x, line_points[i].x );
			min_pos.y = bigball::min( min_pos.y, line_points[i].y );
			min_pos.z = bigball::min( min_pos.z, line_points[i].z );
			max_pos.x = bigball::max( max_pos.x, line_points[i].x );
			max_pos.y = bigball::max( max_pos.y, line_points[i].y );
			max_pos.z = bigball::max( max_pos.z, line_points[i].z );
		}
		vec3 V = max_pos - min_pos;
		float dim_max = max( max( V.x, V.y ), V.z );
		float rescale = params.target_dim / dim_max;
		for( int32 i = 0; i < line_points.size(); ++i )
			 line_points[i] *= rescale;
	}

	attractor->m_dt = init_dt;
	attractor->m_adaptative_dist = init_ad;
}

// Cubic Hermine Curve / SmoothStep()
static float Interpolation_C1(float x) { return x * x * (3.0f - 2.0f * x); }

// Quintic Hermite Curve
static float Interpolation_C2(float x) { return x * x * x * (x * (x * 6.0f - 15.0f) + 10.0f); }

// 7x^3-7x^4+x^7
static  float Interpolation_C2_Fast(float x) { float x3 = x*x*x; return (7.0f + (x3 - 7.0f) * x) * x3; }


void SAUtils::TwistLinePoints(const Array<vec3>& line_points, const Array<quat>& frames, const Array<float>& follow_angles, const Array<AttractorHandle>& attr_handles, Array<vec3>& twist_line_points, Array<quat>& twist_frames, Array<float>& twist_follow_angles)
{
	const int handle_count = attr_handles.size();
	if (handle_count < 2)
	{
		twist_line_points = line_points;
		twist_frames = frames;
		twist_follow_angles = follow_angles;
		return;
	}

	twist_line_points.reserve(line_points.size());
	twist_frames.reserve(frames.size());
	twist_follow_angles.reserve(follow_angles.size());

	// Assumes line_idx are ordered

	for (int h_idx = 0; h_idx < handle_count - 1; h_idx++)
	{
		AttractorHandle const& h0 = attr_handles[h_idx];
		AttractorHandle const& h1 = attr_handles[h_idx + 1];
		int start_line_idx = h0.m_line_idx;
        if (h0.m_type == AttractorHandle::eHT_Cut && h0.m_line_idx < h0.m_mesh_idx && h0.m_mesh_idx < h1.m_line_idx)
			start_line_idx = h0.m_mesh_idx;		// take the shortest path
		vec3 v0 = line_points[start_line_idx];
		vec3 v1 = line_points[h0.m_mesh_idx];
		vec3 start_offset = v1 - v0;

		//int end_line_idx = h1.m_line_idx;
		//if (start_line_idx < h1.m_mesh_idx && h1.m_mesh_idx < h1.m_line_idx)
		//	end_line_idx = h1.m_mesh_idx;		// take the shortest path
		vec3 v2 = line_points[h1.m_line_idx];
		vec3 v3 = line_points[h1.m_mesh_idx];
		vec3 end_offset = v3 - v2;

		int32 delta_idx = h1.m_line_idx - start_line_idx;
		for (int32 d_idx = 0; d_idx < delta_idx; d_idx++)
		{
			float t = (float)d_idx / (float)delta_idx;
			float t0 = clamp(1.0f - t * 2.0f, 0.0f, 1.0f);
			float t1 = clamp(t * 2.0f - 1.0f, 0.0f, 1.0f);
			t0 = Interpolation_C1(t0);
			t1 = Interpolation_C1(t1);

			int32 p_idx = start_line_idx + d_idx;
			vec3 vl = line_points[p_idx];
			vl += start_offset * t0 + end_offset * t1;
			twist_line_points.push_back(vl);
			twist_frames.push_back(frames[p_idx]);
			twist_follow_angles.push_back(follow_angles[p_idx]);
		}
	}
}

void SAUtils::MergeLinePoints(AttractorLineFramed const& line_framed, const Array<AttractorHandle>& attr_handles, AttractorShapeParams const& shape_params, Array<AttractorLineFramed>& snapped_lines)
{
	BB_LOG(SAUtils, Log, "MergeLinePoints\n");

	// compute bounds
	//Array<int> snap_segments;
	const int nb_points = line_framed.points.size();
    Array<AABB> bounds;
	ComputeBounds(line_framed.points, shape_params.merge_dist, bounds);
	int nb_bounds = bounds.size();

	//Array<AttractorMergeInfo> line_merges;
	//line_merges.resize(nb_points);
	Array<AttractorSnapRange> snap_ranges;

	const int min_spacing = 9 * SAUtils::points_in_bound;

    int b_idx1 = 0;
	while ( b_idx1 < nb_bounds )
	{
		BB_LOG(SAUtils, Log, "\rMergeLinePoints %d / %d", b_idx1, nb_bounds);

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

	// build snapped_lines array
	const int nb_range = snap_ranges.size();
	if (nb_range > 0)
	{
		snapped_lines.reserve(nb_range + 2);

		for (int snap_idx = -1; snap_idx < nb_range; snap_idx++)
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
					float best_t, ratio_blend = float(d_idx) / float(lerp_spacing);
					ratio_blend = ratio_blend * ratio_blend * (3.0f - 2.0f * ratio_blend); // smoothstep
					cur_seg_0 = FindNextBestSnapSeg(line_framed.points, src_idx, cur_seg_0, -1 /*inc*/, 1e8f, best_t);
					if (cur_seg_0 == INDEX_NONE)
						break;	// oups, should not happen

					vec3 src_pos = line_framed.points[src_idx];
					vec3 dst_pos = line_framed.points[cur_seg_0] * (1.f - best_t) + line_framed.points[cur_seg_0 + 1] * best_t;
					vec3 delta_pos = dst_pos - src_pos;
					vec3 p = src_pos + delta_pos * ratio_blend;
                    
					ref_framed.points.insert(p,0);
					ref_framed.frames.insert(line_framed.frames[src_idx],0);
					ref_framed.follow_angles.insert(line_framed.follow_angles[src_idx],0);
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
				int prev_size = ref_framed.points.size();
				int cur_seg_0 = snap_ranges[snap_idx + 1].dst_segs.x;
				for (int d_idx = 1; d_idx <= lerp_spacing; d_idx++)
				{
					int src_idx = end_idx + d_idx;
					float best_t, ratio_blend = float(d_idx) / float(lerp_spacing);
					ratio_blend = ratio_blend * ratio_blend * (3.0f - 2.0f * ratio_blend); // smoothstep
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

bool SAUtils::FindSnapRange(const Array<vec3>& line_points, int b_idx0, int b_idx1, float merge_dist, AttractorSnapRange& snap_range/*, ivec2& r_0, ivec2& r_1, Array<int>& snap_segments, Array<AttractorMergeInfo>& line_merges*/)
{
	int start_0 = bigball::max( 0, b_idx0 * SAUtils::points_in_bound - 1 );
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
			
			cur_seg_0 = bigball::clamp(cur_seg_0 + inc, 0, line_points.size() - 1);
		}
		else
			break;
	}

	return best_seg;
}

//
//void SAUtils::SnapRange(Array<vec3>& line_points, float merge_dist, /*ivec2 r_0, ivec2 r_1,*/ Array<int> const& snap_segments, Array<AttractorMergeInfo>& line_merges)
//{
//	BB_ASSERT_LOG(r_1.y - r_1.x + 1 == snap_segments.size(), ("SnapRange error"));
//	const float sq_merge_dist = merge_dist * merge_dist;
//
//	for (int s = 0; s < snap_segments.size(); s++)
//	{
//		int c_1 = r_1.x + s;
//		const vec3& seg_0 = line_points[snap_segments[s]];
//		const vec3& seg_1 = line_points[snap_segments[s] + 1];
//
//		float t, sq_dist = intersect::SquaredDistancePointSegment(line_points[c_1], seg_0, seg_1, t);
//		if (sq_dist < sq_merge_dist)
//		{
//			// overwrite point
//			line_points[c_1] = seg_0 * (1.f - t) + seg_1 * t;
//            line_merges[c_1].merge_parent_idx = snap_segments[s];
//			line_merges[c_1].weight = 1.f;
//		}
//		else
//			int Oups = 0;
//	}
//}

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

//void SAUtils::ComputeStrangeAttractor(StrangeAttractor* attractor, vec3 seed, int32 iter)
//{
//	// Lorentz attractor
//
//	Array<AttractorShape> init_shapes;
//	//LorenzAttractor att0;//, Att1;
//	//att0.m_init_point = vec3( 5.0, 0, 0 );
//	//Att1.m_init_point = vec3( 5.0, 5.0, 5.0 );
//	
//	/*AizawaAttractor att0;
//	att0.m_max_iter = 5000;
//	att0.m_fatness_scale = 0.1f;*/
//
//	//att0.m_init_point = vec3( 0.1f, 0, 0 );
//	//TSUCS2Attractor att0;
//	//att0.m_init_point = vec3( 0.0, -0.05f, 0.01f );
//
////	att0.m_max_iter = 5000;
//////	att0.m_init_point = vec3( -7.85f, -9.94f, 23.01f );
////	//att0.m_init_point = vec3( -2.0, -2.4f, 7.0 );
////	att0.m_init_point = vec3( 4.0, 0.01f, 0.5f );
////	att0.m_fatness_scale = 0.4f;
//
//	AttractorShape AS0;
//	AS0.attractor = attractor;
//	AS0.attractor->m_init_point = seed;
//
//	//AS1.Attractor = &Att1;
//	//AS1.m_max_iter = m_max_iter;
//	init_shapes.push_back( AS0 );
//	//init_shapes.push_back( AS1 );
//
//	AttractorShapeParams params;
//	Array<vec3> local_shape;
//	GenerateLocalShape( local_shape, params );
//	const int32 num_local_points = local_shape.size();
//    
//    Array<quat> frames;
//    Array<float> follow_angles;
//
//	for( int32 ShapeIdx = 0; ShapeIdx < init_shapes.size(); ShapeIdx++ )
//	{	
//		AttractorShape& S = init_shapes[ShapeIdx];
//		S.attractor->LoopAdaptative(S.line_points, iter);
//        
//        GenerateFrames(S.line_points, frames, follow_angles);
//
//		// Generate shapes
//		params.fatness_scale = S.attractor->m_fatness_scale;
//		params.weld_vertex = true;
//		GenerateTriVertices(S.tri_vertices, nullptr, local_shape, S.line_points, frames, follow_angles, params);
//	}
//
//	// Generate tri index
//	GenerateTriIndices( init_shapes, num_local_points );
//
//	std::string filename = "StrangeAttractor_";
//	filename += AS0.attractor->GetClassName();
//	filename += ".obj";
//	WriteObjFile( filename.c_str(), init_shapes );
//}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void SAUtils::GenerateFrames(AttractorLineFramed& line_framed)
{
    vec3 P_prev, P_current, P_next, vz_follow;
    vec3 vx_prev, vy_prev, vz_prev;
    
	Array<vec3> const& line_points = line_framed.points;
	Array<quat>& frames = line_framed.frames;
	Array<float>& follow_angles = line_framed.follow_angles;
    
	line_framed.frames.resize(line_points.size());
	line_framed.follow_angles.resize(line_points.size());
    
    for( int32 i = 1; i < line_points.size()-1; i++ )
    {
        P_prev = line_points[i-1];
        P_current = line_points[i];
        P_next = line_points[i+1];
        vec3 V0 = P_current - P_prev;
        vec3 V1 = P_next - P_current;
        float V0_len = length(V0);
        vec3 V0N = V0 / V0_len;
        vec3 V1N = normalize(V1);
        vec3 vy = cross( V1N, V0N );    // up vector
        float vy_len = length(vy);
        if (vy_len > 1e-4)
            vy /= vy_len;
        else
            vy = vy_prev;
        
        vec3 V0RightN = cross( V0N, vy );
        vec3 V1RightN = cross( V1N, vy );
        vec3 vz = normalize( V0RightN + V1RightN ); // right vector
        vec3 vx = cross( vy, vz );      // front vector
		frames[i] = mat3(vx, vy, vz);
        
        // Replace VX_follow inside VX, VZ frame
        // Remove VY component
        if( i == 1 )
            vz_follow = vz;
        
        float dotX = dot( vx, vz_follow );
        if( bigball::abs(dotX) > 0.99f )
            int32 Break = 0;
        vz_follow -= vx * dotX;
        vz_follow = normalize( vz_follow );
        //VX_follow_array[i] = VX_follow;
        
        float dot_y = dot( vy, vz_follow );
        float dot_z = dot( vz, vz_follow );
        float delta_angle = bigball::atan2(dot_y, dot_z);
        follow_angles[i] = delta_angle;
    }
    frames[0] = frames[1];
    frames[line_points.size()-1] = frames[line_points.size()-2];
    follow_angles[0] = follow_angles[1];
    follow_angles[line_points.size()-1] = follow_angles[line_points.size()-2];
}

/*FQuat FQuat::FastLerp( const FQuat& A, const FQuat& B, const float Alpha )
{
    // To ensure the 'shortest route', we make sure the dot product between the both rotations is positive.
    const float DotResult = (A | B);
    const float Bias = FMath::FloatSelect(DotResult, 1.0f, -1.0f);
    return (B * Alpha) + (A * (Bias * (1.f - Alpha)));
}*/

void SAUtils::GenerateFrames(AttractorLineFramed& line_framed, int from_idx, int to_idx, bool start_continuity, bool end_continuity, vec3* start_vector, vec3* end_vector)
{
    vec3 P_prev, P_current, P_next, vz_follow;
    vec3 vx_prev, vy_prev, vz_prev;
    
    Array<vec3> const& line_points = line_framed.points;
    Array<quat>& frames = line_framed.frames;
    Array<float>& follow_angles = line_framed.follow_angles;
    
    int start_idx = (start_continuity ? from_idx : from_idx + 1);
    int end_idx = (end_continuity ? to_idx : to_idx - 1);
	quat delta_rot(1.f);
    bool constrained_angles = (start_vector != nullptr && end_vector != nullptr) ? true : false;
    if (constrained_angles)
        delta_rot = quat::rotate(*start_vector, *end_vector);

    
    //float delta_angle = end_angle - start_angle;
	//if (constrained_angles)
	//{
	//	if (delta_angle > F_PI)
	//		delta_angle = 2.f * F_PI - delta_angle;
	//	else if (delta_angle < -F_PI)
	//		delta_angle = -delta_angle - 2.f * F_PI;
	//}

    for( int32 i = start_idx; i <= end_idx; i++ )
    {
        P_prev = line_points[i-1];
        P_current = line_points[i];
        P_next = line_points[i+1];
        vec3 V0 = P_current - P_prev;
        vec3 V1 = P_next - P_current;
        float V0_len = length(V0);
        vec3 V0N = V0 / V0_len;
        vec3 V1N = normalize(V1);
        vec3 vy = cross( V1N, V0N );    // up vector
        float vy_len = length(vy);
        if (vy_len > 1e-4)
            vy /= vy_len;
        else
            vy = vy_prev;
        
        vec3 V0RightN = cross( V0N, vy );
        vec3 V1RightN = cross( V1N, vy );
        vec3 vz = normalize( V0RightN + V1RightN ); // right vector
        vec3 vx = cross( vy, vz );      // front vector
		frames[i] = mat3(vx, vy, vz);

		if (constrained_angles)
		{
			// Simply interpolate between requested angles
			float ratio = float(i - start_idx) / float(end_idx - start_idx);
            quat ratio_quat = lerp( quat(1.f), delta_rot, ratio );
            ratio_quat = normalize(ratio_quat);
            vz_follow = ratio_quat.transform(*start_vector);
    
            
			//float angle = start_angle + delta_angle * ratio;
			//follow_angles[i] = angle;
		}
		else
		{
			// Replace VX_follow inside VX, VZ frame
			// Remove VY component
			if (i == start_idx)
				vz_follow = vz;
        }

        float dotX = dot(vx, vz_follow);
        if (bigball::abs(dotX) > 0.99f)
            int32 Break = 0;
        vz_follow -= vx * dotX;
        vz_follow = normalize(vz_follow);

        float dot_y = dot(vy, vz_follow);
        float dot_z = dot(vz, vz_follow);
        float delta_angle = bigball::atan2(dot_y, dot_z);
        follow_angles[i] = delta_angle;
    }
    if (!start_continuity)
        frames[from_idx] = frames[from_idx + 1];
    if (!end_continuity)
        frames[to_idx] = frames[to_idx - 1];
    //follow_angles[0] = follow_angles[1];
    //follow_angles[line_points.size()-1] = follow_angles[line_points.size()-2];
    
}

void SAUtils::GenerateTriVertices(Array<vec3>& tri_vertices, Array<vec3>* tri_normals, Array<float>* tri_colors, const Array<vec3>& local_shape, AttractorLineFramed const & line_framed, const AttractorShapeParams& params)
{
	const Array<vec3>& line_points = line_framed.points;
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
        mat3 mat( frames[i] );
		vec3 vx = mat.v0;   // FRONT
        vec3 vy = mat.v1;   // UP
        vec3 vz = mat.v2;   // RIGHT

		float color = 0.f;
		if (i < line_framed.snap_ranges.y) // snap out
			color = lerp(-1.f, 0.f, float(i - line_framed.snap_ranges.x) / float(line_framed.snap_ranges.y - line_framed.snap_ranges.x));
		else if (i >= line_framed.snap_ranges.z) // snap in
			color = lerp(0.f, 1.f, float(i - line_framed.snap_ranges.z) / float(line_framed.snap_ranges.w - line_framed.snap_ranges.z));

		P_prev = line_points[i-1];
		P_current = line_points[i];
		//P_next = merge_line_points[i+1];
		vec3 V0 = P_current - P_prev;
		//vec3 V1 = P_next - P_current;
		float V0_len = length(V0);
		vec3 V0N = V0 / V0_len;
		vec3 V0RightN = cross( V0N, vy );

		// Twist local shape with follow vec inside current frame VX, VY, VZ
        float delta_angle = follow_angles[i];
		float cf = bigball::cos(delta_angle);
		float sf = bigball::sin(delta_angle);

		float cos_alpha = dot( V0RightN, vz );
		float scale = 1.0f / cos_alpha;
		for (int32 j = 0; j < num_local_points; ++j)
		{ 
			float local_z = local_shape[j].x * cf - local_shape[j].z * sf;
			float local_y = local_shape[j].x * sf + local_shape[j].z * cf;
			rotated_shape[j] = P_current + vz * (local_z * (scale * params.fatness_scale)) + vx * (local_shape[j].y * params.fatness_scale) + vy * (local_y * params.fatness_scale);
		}
		
		for( int32 j = 0; j < num_local_points; ++j )
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
	
	if( !params.weld_vertex )
	{
		// Push start vertices
		for( int32 j = 0; j < num_local_points; ++j )
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
	if( !params.weld_vertex )
	{
		// Push end vertices
		for( int32 j = 0; j < num_local_points; ++j )
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

void SAUtils::GenerateTriIndices(Array<AttractorShape>& vShapes, int32 num_local_points)
{
	// Generate tri index
	for( int32 ShapeIdx = 0; ShapeIdx < vShapes.size(); ShapeIdx++ )
	{	
		AttractorShape& S = vShapes[ShapeIdx];
		GenerateTriIndices(S.tri_vertices, num_local_points, S.tri_indices, true/*bWeldVertex*/, 0/*base_vertex*/);
	}
}

void SAUtils::GenerateTriIndices(const Array<vec3>& tri_vertices, int32 num_local_points, Array<int32>& tri_indices, const bool bWeldVertex, int32 base_vertex)
{
	// Generate tri index
	if( bWeldVertex )
	{
		const int32 nShape = (tri_vertices.size() - base_vertex) / num_local_points;
		for( int32 i=0; i<nShape-1; i++ ) 
		{
			for( int32 j = 0; j < num_local_points; ++j )
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

		for( int32 j = 0; j < num_local_points; ++j )
		{
			tri_indices.push_back(base_vertex + Cap0);
			tri_indices.push_back(base_vertex + 0 * num_local_points + j);
			tri_indices.push_back(base_vertex + 0 * num_local_points + (1 + j) % num_local_points);
		}

		// Second cap
		for( int32 j = 0; j < num_local_points; ++j )
		{
			tri_indices.push_back(base_vertex + Cap1);
			tri_indices.push_back(base_vertex + (nShape - 1) * num_local_points + (1 + j) % num_local_points);
			tri_indices.push_back(base_vertex + (nShape - 1) * num_local_points + j);
		}
	}
	else
	{
		const int32 nShape = (tri_vertices.size() - base_vertex) / (2 * num_local_points) - 1;
		for( int32 i=0; i<nShape-1; i++ ) 
		{
			for( int32 j = 0; j < num_local_points; ++j )
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
		int32 Cap0 = nShapeSize - 2 - 2*num_local_points;
		int32 Cap1 = nShapeSize - 1 - num_local_points;

		for( int32 j = 0; j < num_local_points; ++j )
		{
			tri_indices.push_back(base_vertex + Cap0);
			tri_indices.push_back(base_vertex + Cap0 + 1 + j);
			tri_indices.push_back(base_vertex + Cap0 + 1 + (j + 1) % num_local_points);
			//tri_indices.push_back( 0 * 2*num_local_points + (2*j+1) );
			//tri_indices.push_back( 0 * 2*num_local_points + (2*j+2) % (2*num_local_points) );
		}

		// Second cap
		for( int32 j = 0; j < num_local_points; ++j )
		{
			tri_indices.push_back(base_vertex + Cap1);
			tri_indices.push_back(base_vertex + Cap1 + 1 + (j + 1) % num_local_points);
			tri_indices.push_back(base_vertex + Cap1 + 1 + j);
			//tri_indices.push_back( (nShape-1) * 2*num_local_points + (2*j+2) % (2*num_local_points) );
			//tri_indices.push_back( (nShape-1) * 2*num_local_points + (2*j+1) );
		}
	}
}

void SAUtils::WriteObjFile( const char* filename, Array<vec3>& vPos, Array<int32>& tri_indices )
{
    File fp;
    if( !fp.Open( filename, true ) )
        return;

    String tmp_str;
    tmp_str = String::Printf( "#\n# %s\n#\n\n", filename );
    fp.SerializeString( tmp_str );
	//fopen_s( &fp, filename, "wt" );
	//fprintf( fp, "#\n# %s\n#\n\n", filename );

	int32 i, nPart = vPos.size(), nTri = tri_indices.size() / 3;
	for( i = 0; i < nPart; ++i )
	{
        tmp_str = String::Printf( "v %f %f %f\n", vPos[i].x, vPos[i].y, vPos[i].z );
        fp.SerializeString( tmp_str );
		//fprintf( fp, "v %f %f %f\n", vPos[i].x, vPos[i].y, vPos[i].z );
	}

	for( i = 0; i < nTri; ++i )
	{
        tmp_str = String::Printf("#\n# %s\n#\n\n", filename);
        fp.SerializeString( tmp_str );
		//fprintf( fp, "f %d %d %d\n", 1 + tri_indices[3*i], 1 + tri_indices[3*i+1], 1 + tri_indices[3*i+2] );
	}

	//fclose( fp );
}

void SAUtils::WriteObjFile( const char* filename, const Array<AttractorShape>& vAllShapes )
{
    File fp;
    if( !fp.Open( filename, true ) )
        return;
	//FILE* fp = NULL;
	//fopen_s( &fp, filename, "wt" );
    String tmp_str;
    tmp_str = String::Printf( "#\n# %s\n#\n\n", filename );
    fp.SerializeString( tmp_str );
	//fprintf( fp, "#\n# %s\n#\n\n", filename );

	// Write vertices
	for( int32 ShapeIdx = 0; ShapeIdx < vAllShapes.size(); ShapeIdx++ )
	{	
		const AttractorShape& S = vAllShapes[ShapeIdx];
		int32 i, nPart = S.tri_vertices.size();
		for( i = 0; i < nPart; ++i )
		{
			tmp_str = String::Printf("v %f %f %f\n", S.tri_vertices[i].x, S.tri_vertices[i].y, S.tri_vertices[i].z);
            fp.SerializeString( tmp_str );
			//fprintf( fp, "v %f %f %f\n", S.vShape[i].x, S.vShape[i].y, S.vShape[i].z );
		}
	}

	// Write faces
	int32 nVertexOffset = 0;
	for( int32 ShapeIdx = 0; ShapeIdx < vAllShapes.size(); ShapeIdx++ )
	{	
		const AttractorShape& S = vAllShapes[ShapeIdx];
		int32 i, nTri = S.tri_indices.size() / 3;
		for( i = 0; i < nTri; ++i )
		{
            tmp_str = String::Printf( "f %d %d %d\n", nVertexOffset + 1 + S.tri_indices[3*i], nVertexOffset + 1 + S.tri_indices[3*i+1], nVertexOffset + 1 + S.tri_indices[3*i+2] );
            fp.SerializeString( tmp_str );
			//fprintf( fp, "f %d %d %d\n", nVertexOffset + 1 + S.tri_indices[3*i], nVertexOffset + 1 + S.tri_indices[3*i+1], nVertexOffset + 1 + S.tri_indices[3*i+2] );
		}

		nVertexOffset += S.tri_vertices.size();
	}

	//fclose( fp );
}

void SAUtils::GenerateLocalShape( Array<vec3>& local_shapes, const AttractorShapeParams& params )
{
	bool is_simple_mesh = (params.crease_width <= 0.0 || params.crease_depth <= 0.0 ? true : false);
	const int32 nLocalPoint = (is_simple_mesh ? 1 : 5) * params.local_edge_count;
	local_shapes.resize( nLocalPoint );
	const float fatness = 1.0f;

	if( is_simple_mesh )
	{
		for( int32 edge_idx = 0; edge_idx < params.local_edge_count; ++edge_idx )
		{
			float Angle = 2.0f * F_PI * (float)edge_idx / (float)(params.local_edge_count);
			local_shapes[edge_idx] = vec3( fatness*bigball::cos( Angle ), 0.0f, fatness*bigball::sin( Angle ) );
		}
	}
	else
	{
		//const float min_wall = 0.7f;
		//float MinCreaseWidth = min_wall*0.5f / bigball::sin( F_PI / params.local_edge_count );


		for( int32 edge_idx = 0; edge_idx < params.local_edge_count; ++edge_idx )
		{
			float Angle = 2.0f * F_PI * (float)edge_idx / (float)(params.local_edge_count);
			local_shapes[5*edge_idx] = vec3( fatness*bigball::cos( Angle ), 0.0, fatness*bigball::sin( Angle ) );
		}

		vec3 V;
		float Len;
		for( int32 edge_idx = 0; edge_idx < params.local_edge_count; ++edge_idx )
		{
			const vec3& Prev = local_shapes[5*edge_idx];
			const vec3& Next = local_shapes[5*(edge_idx+1) % nLocalPoint];
			V = Next - Prev;
			Len = length(V);
			//V /= Len;

			local_shapes[5*edge_idx + 1] = Prev + V * params.crease_width;
			local_shapes[5*edge_idx + 2] = (Prev + V * (params.crease_width + params.crease_bevel)) * (1.0f - params.crease_depth);
			local_shapes[5*edge_idx + 3] = (Prev + V * (1.0f - (params.crease_width + params.crease_bevel))) * (1.0f - params.crease_depth);
			local_shapes[5*edge_idx + 4] = Prev + V * (1.0f - params.crease_width);
		}
	}
}

int32 SAUtils::FindNearestPoint( const Array<vec3>& line_points, int32 PointIdx, int32 IgnoreStart, int32 IgnoreEnd )
{
	vec3 V;
	vec3 PointToFind = line_points[PointIdx];
	float NearestDist = FLT_MAX;
	float Len;
	int32 Neighbour = PointIdx;
	for( int32 i=0; i<line_points.size(); ++i )
	{
		V = PointToFind - line_points[i];
		Len = length(V);
		if( Len < NearestDist )
		{
			if( i < IgnoreStart || i > IgnoreEnd )
			{
				NearestDist = Len;
				Neighbour = i;
			}
		}
	}
	return Neighbour;
}

/*vec3 SAUtils::FindNearestFollowVector(const vec3& ref_follow, const vec3& NeighbourFollow, const vec3& NeighbourVX, const vec3& NeighbourVZ, int32 local_edge_count)
{
	vec3 VY = cross( NeighbourVZ, NeighbourVX );
	vec3 LeftFollow = cross( NeighbourFollow, VY );
	LeftFollow = normalize(LeftFollow);

	float best_dot = -FLT_MAX;
	vec3 best_follow = NeighbourFollow;
	for (int32 i = 0; i < local_edge_count; ++i)
	{
		float angle = 2.0f * F_PI * (float)i / (float)(local_edge_count);
		float cf = bigball::cos( angle );
		float sf = bigball::sin( angle );
		vec3 new_follow = NeighbourFollow * cf + LeftFollow * sf;
		float dot_follow = dot( new_follow, ref_follow );
		if (dot_follow > best_dot)
		{
			best_dot = dot_follow;
			best_follow = new_follow;
		}
	}

	return best_follow;
}*/

void SAUtils::FindNearestFollowVector(quat const& src_frame, float src_follow_angle, quat const& dst_frame, float dst_follow_angle, int32 local_edge_count, vec3& src_follow, vec3& dst_follow)
{
	mat3 src_mat(src_frame);
	vec3 src_vx = src_mat.v0;   // FRONT
	vec3 src_vy = src_mat.v1;   // UP
	vec3 src_vz = src_mat.v2;   // RIGHT
	float src_cf = bigball::cos(src_follow_angle);
	float src_sf = bigball::sin(src_follow_angle);
    src_follow = src_vz * src_cf + src_vy * src_sf;

	mat3 dst_mat(dst_frame);
	vec3 dst_vx = dst_mat.v0;   // FRONT
	vec3 dst_vy = dst_mat.v1;   // UP
	vec3 dst_vz = dst_mat.v2;   // RIGHT

	dst_follow = vec3(0.f,0.f,0.f);
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

	// ensure dst_angle is in [-PI, PI] range
	//if (best_angle > F_PI)
	//	best_angle -= F_PI;

	//return best_angle;
}

void SAUtils::MergeLinePoints( const Array<vec3>& line_points, const Array<vec3>& VX_follow_array, const Array<vec3>& VX_array, const Array<vec3>& VZ_array, Array<vec3>& vMergePoints, Array<vec3>& vMergeFollow, const AttractorShapeParams& params )
{
	vMergePoints = line_points;
	vMergeFollow = VX_follow_array;

#if 0
	// Manage start
	if( params.merge_start >= 1 )
	{
		// Find point nearest to us
		int32 StartIdx = 0;
		int32 Neighbour = FindNearestPoint( line_points, StartIdx, StartIdx, StartIdx + params.merge_start );

		// Check by which side we should go
		int32 Inc = 1;
		if( Neighbour >= line_points.size()-params.merge_start)
		{
			Inc = -1;
		}
		else if( Neighbour <= params.merge_start)
		{
			Inc = 1;
		}
		else
		{
			vec3 V0, V1;
			V0 = line_points[StartIdx+1] - line_points[Neighbour+1];
			V1 = line_points[StartIdx+1] - line_points[Neighbour-1];
			if( length(V0) < length(V1) )
				Inc = 1;
			else
				Inc = -1;
		}

		// Merge start
		const int32 Margin = 2;
		for( int32 Offset = 0; Offset < params.merge_start; Offset++ )
		{
			float MergeRatio = (Offset < Margin) ? 1.0f : (1.0f - (float)(Offset - Margin) / (float)(params.merge_start-Margin));
			vMergePoints[StartIdx + Offset] = line_points[StartIdx + Offset] * (1.0f - MergeRatio) + line_points[Neighbour + Offset*Inc] * MergeRatio;

			vec3 NeighbourFollow = FindNearestFollowVector( VX_follow_array[StartIdx + Offset], VX_follow_array[Neighbour + Offset*Inc], VX_array[Neighbour + Offset*Inc], VZ_array[Neighbour + Offset*Inc], params.local_edge_count );
			vMergeFollow[StartIdx + Offset] = VX_follow_array[StartIdx + Offset] * (1.0f - MergeRatio) + NeighbourFollow * MergeRatio;
			vMergeFollow[StartIdx + Offset] = normalize(vMergeFollow[StartIdx + Offset]);
		}
	}

	// Manage end
	if( params.merge_end >= 1 )
	{
		// Find point nearest to us
		int32 StartIdx = line_points.size() - 1 - params.merge_end;
		int32 Neighbour = FindNearestPoint( line_points, StartIdx, StartIdx - params.merge_end, StartIdx + params.merge_end );

		// Check by which side we should go
		int32 Inc = 1;
		if( Neighbour >= line_points.size()-params.merge_end)
		{
			Inc = -1;
		}
		else if( Neighbour <= params.merge_end)
		{
			Inc = 1;
		}
		else
		{
			vec3 V0, V1;
			V0 = line_points[StartIdx+1] - line_points[Neighbour+1];
			V1 = line_points[StartIdx+1] - line_points[Neighbour-1];
			if( length(V0) < length(V1) )
				Inc = 1;
			else
				Inc = -1;
		}

		// Merge end
		const int32 Margin = 2;
		for( int32 Offset = 0; Offset < params.merge_end; Offset++ )
		{
			float MergeRatio = (Offset >= params.merge_end - Margin) ? 1.0f : ((float)(Offset) / (float)(params.merge_end-Margin));
			vMergePoints[StartIdx + Offset] = line_points[StartIdx + Offset] * (1.0f - MergeRatio) + line_points[Neighbour + Offset*Inc] * MergeRatio;

			vec3 NeighbourFollow = FindNearestFollowVector( VX_follow_array[StartIdx + Offset], VX_follow_array[Neighbour + Offset*Inc], VX_array[Neighbour + Offset*Inc], VZ_array[Neighbour + Offset*Inc], params.local_edge_count );
			vMergeFollow[StartIdx + Offset] = VX_follow_array[StartIdx + Offset] * (1.0f - MergeRatio) + NeighbourFollow * MergeRatio;
			vMergeFollow[StartIdx + Offset] = normalize( vMergeFollow[StartIdx + Offset] );
		}
	}
#endif // 0
}

StrangeAttractor* SAUtils::CreateAttractorType(String const& attractor_name)
{
	if( attractor_name == "Lorentz" )
		return CreateAttractorType(eAttractor_Lorentz );
	if (attractor_name == "Aizawa")
		return CreateAttractorType(eAttractor_Aizawa );
	if (attractor_name == "TSUCS2")
		return CreateAttractorType(eAttractor_TSUCS2 );
	if (attractor_name == "Arnoedo")
		return CreateAttractorType(eAttractor_Arneodo );
	if (attractor_name == "ChenLee")
		return CreateAttractorType(eAttractor_ChenLee );
	if (attractor_name == "DequanLi")
		return CreateAttractorType(eAttractor_DequanLi );
	if (attractor_name == "LorentzMod2")
		return CreateAttractorType(eAttractor_LorentzMod2 );
	
	return nullptr;
}

StrangeAttractor* SAUtils::CreateAttractorType(eAttractorType attractor_type)
{
    if( attractor_type == eAttractor_Lorentz )
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

/*void SAUtils::ComputeBoundRange(int b_idx, int& start, int& end)
{
	start = b_idx * points_in_bound;
	end = bigball::min(start + points_in_bound, line_points.size());
}*/