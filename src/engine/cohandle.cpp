

#include "../sae.h"
#include "cohandle.h"
#include "engine/coposition.h"
#include "gfx/rendercontext.h"


CLASS_EQUIP_CPP(CoHandle);

CoHandle::CoHandle() :
    m_selected_handle_idx(INDEX_NONE)
{

}

CoHandle::~CoHandle()
{

}

void CoHandle::Create(Entity* owner, class json::Object* proto)
{
	Super::Create( owner, proto );


}

void CoHandle::Destroy()
{
	Super::Destroy();
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

bool CoHandle::HasHandleArrayChanged()
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

void CoHandle::SaveHandleArray()
{
	m_cached_handles = m_handles;
}

void CoHandle::InsertHandle(int32 at_idx)
{
    int32 num_handle = m_handles.size();
    if (at_idx <= 0 || at_idx > num_handle - 1 )
        return;
    
    MeshHandle handle;
    handle.m_line_idx = (m_handles[at_idx - 1].m_line_idx + m_handles[at_idx].m_line_idx) / 2;
    handle.m_mesh_idx = handle.m_line_idx;
    m_handles.insert(handle, at_idx);
}

void CoHandle::DeleteHandle(int32 at_idx)
{
    int32 num_handle = m_handles.size();
    if (at_idx <= 0 || at_idx >= num_handle - 1 )
        return;
    
    m_handles.erase(at_idx);
}

bool CoHandle::RayCast(vec3 const& ray_start, vec3 const& ray_end)
{
#if 0
	CoPosition* copos = static_cast<CoPosition*>(GetEntityComponent("CoPosition"));

	int32 num_handle = m_handles.size();
	for (int32 h_idx = 0; h_idx < num_handle; h_idx++)
	{
		m_handles[h_idx];

		vec3 world_line_handle_pos = copos->GetTransform().TransformPosition(attractor->m_line_points[handle.m_line_idx]);
		quat world_line_handle_quat = copos->GetRotation() * attractor->m_frames[handle.m_line_idx];
		DrawUtils::GetStaticInstance()->PushOBB(transform(world_line_handle_quat, world_line_handle_pos, cube_size), u8vec4(255, 0, 255, 255), 0.5f, 0.5f);

		vec3 world_mesh_handle_pos = copos->GetTransform().TransformPosition(attractor->m_line_points[handle.m_mesh_idx]);
		DrawUtils::GetStaticInstance()->PushSphere(world_mesh_handle_pos, cube_size * 0.85, u8vec4(0, 255, 255, 255));
	}

	Draw::InstanceParams& params = m_shape_params[shape_idx];
	mat4 const& box_to_world = m_shape_matrices[shape_idx];
	mat4 box_to_view = view_mat * box_to_world;
	params.m_eye_to_box = (inverse(box_to_view) * vec4(0.f, 0.f, 0.f, 1.f)).xyz;
#endif

	return false;
}