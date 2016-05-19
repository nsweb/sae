


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
struct AttractorHandle;

class CoHandle : public Component 
{
	CLASS_EQUIP_H(CoHandle, Component)

public:
						CoHandle();
	virtual				~CoHandle();
    
	static Component*	NewComponent()              { return new CoHandle(); }

	virtual void		Create( Entity* owner, class json::Object* proto = nullptr ) override;
	virtual void		Destroy() override;
	virtual void		AddToWorld() override;
	virtual void		RemoveFromWorld() override;
	virtual void		Tick( TickContext& tick_ctxt );
    virtual void        Serialize(Archive& file) override;
    
	void				_Render( RenderContext& render_ctxt );
    bool                OnControllerInput( Camera* pCamera, ControllerInput const& Input );
	bool				HasHandleArrayChanged();
	void				SaveHandleArray();
    void                InsertHandle(int32 at_idx);
    void                DeleteHandle(int32 at_idx);
	AttractorHandle&    GetHandle(int32 at_idx)     { return m_handles[at_idx]; }

	struct PickResult
	{
		int32	m_handle_idx;
		bool	m_is_line_pick;
		float	m_dist;
	};
	bool				RayCast(vec3 const& ray_start, vec3 const& ray_end, PickResult& pick_result);

public:
	Array<AttractorHandle>	m_handles;
	Array<AttractorHandle>	m_cached_handles;

};

#endif // SAECOHANDLE_H