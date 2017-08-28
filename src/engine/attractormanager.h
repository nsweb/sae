


#ifndef ATTRACTORMANAGER_H
#define ATTRACTORMANAGER_H

#include "engine/componentmanager.h"

namespace bigball
{
	class BIGBALL_API Component;
	class BIGBALL_API Shader;
    class Archive;
    struct ControllerMouseState;
};
class CoAttractor;
class AttractorFactory;

struct AttractorSelection
{
    AttractorSelection() : m_attractor(nullptr), m_handle_idx(INDEX_NONE), m_point_idx(INDEX_NONE) {}
    CoAttractor*    m_attractor;
    int32           m_handle_idx;
    int32           m_point_idx;
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
    
    Array<CoAttractor*> const& GetAttractors()      { return m_attractors;  }
    AttractorFactory*   GetFactory()                { return m_attractor_factory; }

	void				SetShowHandles(bool show);
	bool				GetShowHandles()            { return m_show_handles; }
    AttractorSelection& GetEditorSelected()         { return m_editor_selected;    }
    
    void                SerializeAttractor(Archive& file, bool old_format);
    void                ExportAttractorAsObj(Archive& file);

public:

    AttractorFactory*   m_attractor_factory;
    Array<CoAttractor*> m_attractors;
    AttractorSelection  m_editor_selected;
	AttractorSelection  m_editor_hovered;
    float               m_attractor_seed_range;
    float               m_attractor_seed_move_range;
    
    Shader*             m_bg_shader;
    Shader*             m_line_shader;
    Shader*             m_mesh_shader;
	bool				m_show_handles;
    bool                m_show_lines;
    //bool				m_show_seeds;
    bool                m_prev_mouse_left_down;

    //void				UpdateAttractorVertexBuffers(struct RenderContext& render_ctxt);
    void                UpdateAttractorMeshes();
	void				DrawAttractors(struct RenderContext& render_ctxt);
	void				DrawHandles(struct RenderContext& render_ctxt);
    void				DrawAttractorSeeds(struct RenderContext& render_ctxt);
};


#endif // ATTRACTORMANAGER_H
