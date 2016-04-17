


#ifndef SAECOHANDLE_H
#define SAECOHANDLE_H

#include "engine/component.h"

namespace bigball
{
	class BIGBALL_API Shader;
	struct BIGBALL_API TickContext;
	struct BIGBALL_API RenderContext;
    class BIGBALL_API Camera;
    struct ControllerInput;
};

class MeshHandle
{
public:
	MeshHandle()	{}
	~MeshHandle()	{}

	transform	m_transform;
};

class CoHandle : public Component 
{
	CLASS_EQUIP_H(CoHandle, Component)

public:
						CoHandle();
	virtual				~CoHandle();

	static Component*	NewComponent()		{ return new CoHandle(); }

	virtual void		Create( Entity* owner, class json::Object* proto = nullptr );
	virtual void		Destroy();	
	virtual void		AddToWorld();
	virtual void		RemoveFromWorld();
	virtual void		Tick( TickContext& tick_ctxt );
	void				_Render( RenderContext& render_ctxt );
    bool                OnControllerInput( Camera* pCamera, ControllerInput const& Input );
    

public:
	Array<MeshHandle>	m_handles;

};

#endif // SAECOATTRACTOR_H