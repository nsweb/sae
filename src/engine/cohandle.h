


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

struct AttractorSeedParams
{
    vec3 seed = vec3(1.f, 1.f, 1.f);
    int32 iter = 2000;
    int32 rev_iter = 0;
    int32 merge_span = -1;
    
    bool operator == (AttractorSeedParams const& oth)
    {
        return seed == oth.seed && iter == oth.iter && rev_iter == oth.rev_iter && merge_span == oth.merge_span;
    }
    
    void Serialize(Archive& file);;
};

struct AttractorHandle
{
public:
    AttractorHandle() : m_idx_on_curve(0)	{}
    ~AttractorHandle()	{}
    
    bool operator == (AttractorHandle const& oth)
    {
        return m_seed == oth.m_seed;
    }
    
    void Serialize(Archive& file);
    
    AttractorSeedParams m_seed;
    int32               m_idx_on_curve;
    //int32             m_mesh_idx;
};

////////////////////////////////////////////////////
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
	bool				HasAnyHandleChanged();
    bool				HasHandleChanged(int32 h_idx);
	void				SaveHandleArray();
    void                InsertHandle(int32 at_idx);
    void                DeleteHandle(int32 at_idx);
	AttractorHandle&    GetHandle(int32 at_idx)     { return m_handles[at_idx]; }

	struct PickResult
	{
		int32	m_handle_idx;
		float	m_ray_dist;
	};
	bool				RayCast(vec3 const& ray_start, vec3 const& ray_end, PickResult& pick_result);

public:
	Array<AttractorHandle>	m_handles;
	Array<AttractorHandle>	m_cached_handles;

};

#endif // SAECOHANDLE_H
