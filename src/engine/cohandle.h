


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

	bool operator == (MeshHandle const& oth)
	{
		return /*m_transform == oth.m_transform
			&& m_type == oth.m_type &&*/ m_line_idx == oth.m_line_idx && m_mesh_idx == oth.m_mesh_idx;
	}

	enum eHandleType
	{
		eHT_PassThrough,
		eHT_Interp,
		eHT_Begin,
		eHT_End,
	};

	//transform	m_transform;
	//eHandleType	m_type;
	int32		m_line_idx;
	int32		m_mesh_idx;
};

class CoHandle : public Component 
{
	CLASS_EQUIP_H(CoHandle, Component)

public:
						CoHandle();
	virtual				~CoHandle();

	static Component*	NewComponent()		{ return new CoHandle(); }

	virtual void		Create( Entity* owner, class json::Object* proto = nullptr ) override;
	virtual void		Destroy() override;
	virtual void		AddToWorld() override;
	virtual void		RemoveFromWorld() override;
	virtual void		Tick( TickContext& tick_ctxt );
	void				_Render( RenderContext& render_ctxt );
    bool                OnControllerInput( Camera* pCamera, ControllerInput const& Input );
	bool				HasHandleArrayChanged();
	void				SaveHandleArray();
    void                InsertHandle(int32 at_idx);
    void                DeleteHandle(int32 at_idx);

	struct PickResult
	{
		int32	m_handle_idx;
		bool	m_is_line_pick;
		float	m_dist;
	};
	bool				RayCast(vec3 const& ray_start, vec3 const& ray_end, PickResult& pick_result);

public:
    int32               m_selected_handle_idx;
	Array<MeshHandle>	m_handles;
	Array<MeshHandle>	m_cached_handles;

};

#endif // SAECOATTRACTOR_H