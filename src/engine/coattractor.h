
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
	void                ExportAsPly(Archive& file);
    
	void				_Render( RenderContext& render_ctxt );
    bool                OnControllerInput( Camera* pCamera, ControllerInput const& Input );
    
    void                ChangeAttractorType(eAttractorType type);
	void				RebuildAttractorMesh(bool force_rebuild = false/*, bool keep_handle = false*/);
    eAttractorType      GetAttractorType()   { return m_attractor ? m_attractor->m_type : eAttractor_None; }
    
    void                InsertCurve(int32 at_idx);
    void                DeleteCurve(int32 at_idx);
    vec3                GuessCurvePos(int32 curve_idx, int32 at_seed_offset);

	struct PickResult
	{
		int32	m_handle_idx;
        int32   m_point_idx;
        float	m_ray_dist;
	};

	bool				RayCast(vec3 const& ray_start, vec3 const& ray_end, const float ray_width, PickResult& pick_result);
    const AttractorOrientedCurve* GetCurvePreview() const;
    void                GetMeshRenderOffsetsWithoutPreview(ivec2& start_range, ivec2& end_range) const;
    
    vec3                GetCurveWorldPos(int32 curve_ix, int32 point_idx);

public:
	StrangeAttractor*       m_attractor;
    
	Array<AttractorOrientedCurve>	m_curves;
    
    Array<vec3>             m_tri_vertices;
	Array<vec3>             m_tri_normals;
    Array<int32>            m_tri_indices;
    
    Array<vec3>             m_tri_vertices_preview;
    Array<vec3>             m_tri_normals_preview;
    Array<int32>            m_tri_indices_preview;
    
    Array<int32>            m_indice_offsets;
    Array<float>            m_show_curve_alphas;
    
    // dimensions of the attractor
    vec3                    m_min_box;
    vec3                    m_max_box;
    float                   m_rescale_factor;

	// Attractor params
	AttractorLineParams		m_line_params;
    AttractorShapeParams    m_shape_params;
    int32                   m_preview_idx;
	
    // Cached parameters
	AttractorLineParams		m_cached_line_params;
    AttractorShapeParams    m_cached_shape_params;
    //Array<AttractorSeedParams>  m_cached_seed_params;
    int32                   m_cached_preview_idx;

	enum eVAType
	{
		//eVALinePoints = 0,
		eVAMesh = 0,
        eVAMeshPreview,
		eVACount
	};
	enum eVBType
	{
		//eVBLinePoints = 0,
		eVBMesh = 0,
		eVBMeshNormals,
		//eVBMeshColors,
		eVBMeshElt,
        eVBMeshPreview,
        eVBMeshNormalsPreview,
        eVBMeshEltPreview,
        eVBCount
	};

	GLuint			m_varrays[eVACount];
	GLuint			m_vbuffers[eVBCount];
    int32           m_vb_preview_size;
    int32           m_vb_mesh_size;

private:
	void				UpdateVertexBuffers(bool update_preview, bool update_mesh);

};

#endif // SAECOATTRACTOR_H
