
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


class CoAttractor : public Component 
{
	CLASS_EQUIP_H(CoAttractor, Component)

public:
						CoAttractor();
	virtual				~CoAttractor();

	static Component*	NewComponent()		{ return new CoAttractor(); }

	virtual void		Create( Entity* owner, class json::Object* proto = nullptr ) override;
	virtual void		Destroy() override;
	virtual void		AddToWorld() override;
	virtual void		RemoveFromWorld() override;
	virtual void		Tick( TickContext& tick_ctxt );
    virtual void        Serialize(Archive& file) override;
	virtual void		PostLoad() override;
    void                ExportAsObj(Archive& file);
    
	void				_Render( RenderContext& render_ctxt );
    bool                OnControllerInput( Camera* pCamera, ControllerInput const& Input );
    
    void                ChangeAttractorType(eAttractorType type);
	void				RebuildAttractorMesh(bool force_rebuild = false, bool keep_handle = false);
    eAttractorType      GetAttractorType()   { return m_attractor ? m_attractor->m_type : eAttractor_None; }

	struct PickResult
	{
		vec3	m_hit_pos;
		int32	m_line_idx;
	};

	bool				RayCast(vec3 const& ray_start, vec3 const& ray_end, const float ray_width, PickResult& pick_result);

public:
	StrangeAttractor*       m_attractor;
    
	AttractorLineFramed		m_line_framed;

	Array<AttractorLineFramed>	m_snapped_lines;
    Array<vec3>             m_tri_vertices;
	Array<vec3>             m_tri_normals;
	Array<float>            m_tri_colors;
    Array<int32>            m_tri_indices;
    vec3                    m_min_box;
    vec3                    m_max_box;
    float                   m_rescale_factor;

	// Attractor params
	AttractorLineParams		m_line_params;
	AttractorLineParams		m_cached_line_params;
    AttractorShapeParams    m_shape_params;
	AttractorShapeParams    m_cached_shape_params;

	int32					m_view_handle_range;

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
		eVBMeshColors,
		eVBMeshElt,
		eVBCount
	};

	GLuint			m_varrays[eVACount];
	GLuint			m_vbuffers[eVBCount];

private:
	void				UpdateVertexBuffers();

};

#endif // SAECOATTRACTOR_H
