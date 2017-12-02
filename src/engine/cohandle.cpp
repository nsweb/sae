
#include "../sae.h"
#include "cohandle.h"
#include "coattractor.h"
#include "engine/coposition.h"
#include "gfx/rendercontext.h"
#include "math/intersections.h"
#include "system/file.h"


CLASS_EQUIP_CPP(CoHandle);

CoHandle::CoHandle()
{

}

CoHandle::~CoHandle()
{

}

void CoHandle::Create(Entity* owner, class json::Object* proto)
{
	Super::Create( owner, proto );

    // Create a default handle
    if (m_handles.size() == 0)
    {
        m_handles.push_back(AttractorHandle());
    }
}

void CoHandle::Destroy()
{
	Super::Destroy();
    
    m_handles.clear();
    m_cached_handles.clear();
}

void CoHandle::AddToWorld()
{
	Super::AddToWorld();
}

void CoHandle::RemoveFromWorld()
{
	Super::RemoveFromWorld();
}

void CoHandle::Tick(TickContext& tick_ctxt)
{

}

bool CoHandle::OnControllerInput(Camera* pcamera, ControllerInput const& input)
{

    return true;
}

void CoHandle::_Render(RenderContext& render_ctxt)
{
	static float global_time = 0.f;
	global_time += render_ctxt.m_delta_seconds;
	
}

bool CoHandle::HasAnyHandleChanged()
{
	if (m_handles.size() != m_cached_handles.size())
		return true;

	int32 num_handle = m_handles.size();
	for (int32 h_idx = 0; h_idx < num_handle; h_idx++)
	{
		if (!(m_handles[h_idx] == m_cached_handles[h_idx]))
			return true;
	}

	return false;
}

bool CoHandle::HasHandleChanged(int32 h_idx)
{
    int32 num_handle = m_handles.size();
    int32 num_cached_handle = m_cached_handles.size();
    if (h_idx < 0 || h_idx > num_handle - 1 || h_idx > num_cached_handle - 1 )
        return true;
    
    return !(m_handles[h_idx] == m_cached_handles[h_idx]);
}

void CoHandle::SaveHandleArray()
{
	m_cached_handles = m_handles;
}

void CoHandle::InsertHandle(int32 at_idx)
{
    int32 num_handle = m_handles.size();
    if (at_idx <= 0 || at_idx > num_handle )
        return;
    
	AttractorHandle handle;
    if (at_idx < num_handle)
    {
        //handle.m_line_idx = (m_handles[at_idx - 1].m_line_idx + m_handles[at_idx].m_line_idx) / 2;
        //handle.m_mesh_idx = handle.m_line_idx;
        m_handles.insert(handle, at_idx);
    }
    else
    {
        //handle.m_line_idx = (m_handles[at_idx - 1].m_line_idx + 1);
        //handle.m_mesh_idx = handle.m_line_idx;
        m_handles.push_back(handle);
    }
}

void CoHandle::DeleteHandle(int32 at_idx)
{
    int32 num_handle = m_handles.size();
    if ( at_idx >= 0 && at_idx < num_handle && num_handle > 1 )
        m_handles.erase(at_idx);
}

bool CoHandle::RayCast(vec3 const& ray_start, vec3 const& ray_end, PickResult& pick_result)
{
	CoPosition* pos = static_cast<CoPosition*>(GetEntityComponent("CoPosition"));
    CoAttractor* attractor = static_cast<CoAttractor*>(GetEntityComponent("CoAttractor"));
    transform const& attr_transform = pos->GetTransform();
    const float cube_size = 2.f * attr_transform.GetScale() * attractor->m_shape_params.fatness_scale;
    
	//pick_result.m_dist = 1e8f;
	//pick_result.m_handle_idx = INDEX_NONE;
    
    bool is_picked = false;
    
	int32 num_handle = m_handles.size();
	for (int32 h_idx = 0; h_idx < num_handle; h_idx++)
	{
		AttractorHandle const& handle = m_handles[h_idx];
        AttractorOrientedCurve const& curve = attractor->m_curves[h_idx];
        int curve_size = curve.frames.size();
        if (handle.m_idx_on_curve >= 0 && handle.m_idx_on_curve < curve_size)
		{
			vec3 world_line_handle_pos = attr_transform.TransformPosition(handle.m_seed.seed * attractor->m_rescale_factor);
            quat world_line_handle_quat = attr_transform.GetRotation() * curve.frames[handle.m_idx_on_curve];
			transform h_line_transform(world_line_handle_quat, world_line_handle_pos, cube_size);
			vec3 ray_start_box = h_line_transform.TransformPositionInverse(ray_start);
			vec3 ray_end_box = h_line_transform.TransformPositionInverse(ray_end);
			vec3 ray_dir_box = normalize(ray_end_box - ray_start_box);

			float t = intersect::RayBoxIntersection(ray_start_box, ray_dir_box, vec3(1.f, 0.5f, 0.5f));
			if (t >= 0.f && t < pick_result.m_ray_dist)
			{
				pick_result.m_ray_dist = t;
				pick_result.m_handle_idx = h_idx;
                is_picked = true;
				//pick_result.m_is_line_pick = true;
			}
		}
        
		/*{
			vec3 world_mesh_handle_pos = attr_transform.TransformPosition(attractor->m_line_framed.points[handle.m_mesh_idx]);
			transform h_mesh_transform(quat(1.f, 0.f, 0.f, 0.f), world_mesh_handle_pos, cube_size);
			vec3 ray_start_sphere = h_mesh_transform.TransformPositionInverse(ray_start);
			vec3 ray_end_sphere = h_mesh_transform.TransformPositionInverse(ray_end);
			vec3 ray_dir_sphere = normalize(ray_end_sphere - ray_start_sphere);
            
			float dist = intersect::RaySphereIntersection(ray_start_sphere, ray_dir_sphere, vec3(0.f, 0.f, 0.f), 0.85f);
			if (dist >= 0.f && dist < pick_result.m_dist)
			{
				pick_result.m_dist = dist;
				pick_result.m_handle_idx = h_idx;
				pick_result.m_is_line_pick = false;
			}
		}*/
	}

	return is_picked;
}

void CoHandle::Serialize(Archive& file)
{
    int32 count = m_cached_handles.size();
    file.SerializeRaw(count);
    if( file.IsReading() )
        m_cached_handles.resize(count);
    
    for (int32 h_idx = 0; h_idx < m_cached_handles.size(); h_idx++)
    {
        m_cached_handles[h_idx].Serialize(file);
    }
    
    if( file.IsReading() )
        m_handles = m_cached_handles;
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
void AttractorHandle::Serialize(Archive& file)
{
    m_seed.Serialize(file);
    file.SerializeRaw(m_idx_on_curve);
}
    
void AttractorSeedParams::Serialize(Archive& file)
{
    uint32 file_version = file.GetVersion();
    
    file.SerializeRaw(seed);
    file.SerializeRaw(iter);
    file.SerializeRaw(rev_iter);
    
    
    if (file_version >= (uint32)eSaeVersion_SpanPerHandle)
    {
        file.SerializeRaw(merge_span);
    }
    else if (file.IsReading())
    {
        merge_span = -1;
    }
}

