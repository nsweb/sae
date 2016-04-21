

#include "../sae.h"
#include "cohandle.h"
#include "gfx/rendercontext.h"


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