


#ifndef ATTRACTORMANAGER_H
#define ATTRACTORMANAGER_H

#include "engine/componentmanager.h"

namespace bigball
{
	class BIGBALL_API Component;
	class BIGBALL_API Shader;
};
class CoAttractor;

class AttractorManager : public ComponentManager
{
	STATIC_MANAGER_H(AttractorManager);

	struct DFVertex
	{
		vec2	pos;
		vec2	tex;
	};

private:
	typedef ComponentManager Super;

public:
						AttractorManager();
	virtual				~AttractorManager();

	virtual void		Create() override;
	virtual void		Destroy() override;
    virtual void		AddComponentToWorld( Component* pComponent ) override;
    virtual void		RemoveComponentFromWorld( Component* pComponent ) override;
	virtual void		Tick( struct TickContext& tick_ctxt ) override;
	virtual void		_Render( struct RenderContext& render_ctxt ) override;
    void				DrawAttractors( struct RenderContext& render_ctxt );
    Array<CoAttractor*> const& GetAttractors()  { return m_attractors;  }

protected:

    Array<CoAttractor*>  m_attractors;
    
    Shader*             m_bg_shader;
    Shader*             m_line_shader;
    Shader*             m_mesh_shader;
};


#endif // ATTRACTORMANAGER_H