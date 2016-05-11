


#ifndef ATTRACTORMANAGER_H
#define ATTRACTORMANAGER_H

#include "engine/componentmanager.h"

namespace bigball
{
	class BIGBALL_API Component;
	class BIGBALL_API Shader;
    struct ControllerMouseState;
};
class CoAttractor;

struct AttractorSelection
{
    AttractorSelection() : m_attractor(nullptr), m_handle_idx(INDEX_NONE) {}
    CoAttractor*    m_attractor;
    int             m_handle_idx;
    bool            m_line_handle;
    
};

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
    void				HandleScenePick(ControllerMouseState const& mouse_state);
    
    Array<CoAttractor*> const& GetAttractors()  { return m_attractors;  }

	void				SetShowHandles(bool show);
	bool				GetShowHandles()		{ return m_show_handles; }

protected:

    Array<CoAttractor*> m_attractors;
    AttractorSelection  m_editor_pick;
    
    Shader*             m_bg_shader;
    Shader*             m_line_shader;
    Shader*             m_mesh_shader;
	bool				m_show_handles;
    bool                m_prev_mouse_left_down;

	void				DrawAttractors(struct RenderContext& render_ctxt);
	void				DrawHandles(struct RenderContext& render_ctxt);
};


#endif // ATTRACTORMANAGER_H