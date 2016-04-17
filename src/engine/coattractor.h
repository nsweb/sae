


#ifndef SAECOATTRACTOR_H
#define SAECOATTRACTOR_H

#include "engine/component.h"
#include "strangeattractors.h"

namespace bigball
{
	class BIGBALL_API Shader;
	struct BIGBALL_API TickContext;
	struct BIGBALL_API RenderContext;
    class BIGBALL_API Camera;
    struct ControllerInput;
};

struct RayCastParams
{
	vec3 m_start;
	vec3 m_end;
	float m_capsule_width;
};

struct RayCastResults
{
	vec3 m_hit;
};

class CoAttractor : public Component 
{
	CLASS_EQUIP_H(CoAttractor, Component)

public:
						CoAttractor();
	virtual				~CoAttractor();

	static Component*	NewComponent()		{ return new CoAttractor(); }

	virtual void		Create( Entity* owner, class json::Object* proto = nullptr );
	virtual void		Destroy();	
	virtual void		AddToWorld();
	virtual void		RemoveFromWorld();
	virtual void		Tick( TickContext& tick_ctxt );
	void				_Render( RenderContext& render_ctxt );
    bool                OnControllerInput( Camera* pCamera, ControllerInput const& Input );
    
    void                ChangeAttractorType(eAttractorType type);
	void				RebuildAttractorMesh();
    eAttractorType      GetAttractorType()   { return m_attractor ? m_attractor->m_type : eAttractor_None; }

	bool				RayCast(RayCastParams const& params, RayCastResults& results);

public:
	StrangeAttractor*       m_attractor;
    
    Array<vec3>             m_line_points;
    Array<vec3>             m_tri_vertices;
	Array<vec3>             m_tri_normals;
    Array<int32>            m_tri_indices;
    vec3                    m_min_box;
    vec3                    m_max_box;

	// Attractor params
    AttractorShapeParams    m_params;
	vec3                    m_seed;
	int32                   m_iter;
	int32                   m_rev_iter;
	int32                   m_skip_iter;
	float                   m_step_factor;
	float                   m_target_dim;

	enum eVAType
	{
		eVALinePoints = 0,
		eVAMesh,
		eVACount
	};
	enum eVBType
	{
		eVBLinePoints = 0,   
		eVBMesh,
		eVBMeshNormals,
		eVBMeshElt,
		eVBCount
	};

	GLuint			m_varrays[eVACount];
	GLuint			m_vbuffers[eVBCount];

private:
	void				UpdateVertexBuffers();

};

#endif // SAECOATTRACTOR_H