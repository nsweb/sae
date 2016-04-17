

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

