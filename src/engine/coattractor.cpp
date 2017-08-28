
#include "../sae.h"
#include "coattractor.h"
#include "attractormanager.h"
#include "cohandle.h"
#include "core/json.h"
#include "math/intersections.h"
#include "system/file.h"
#include "engine/coposition.h"
#include "engine/controller.h"
#include "engine/camera.h"
#include "engine/tickcontext.h"
#include "engine/entitymanager.h"
#include "gfx/gfxmanager.h"
#include "gfx/shader.h"
#include "gfx/rendercontext.h"


CLASS_EQUIP_CPP(CoAttractor);

CoAttractor::CoAttractor() :
	m_attractor(nullptr),
    m_min_box(0.f, 0.f, 0.f),
    m_max_box(0.f, 0.f, 0.f),
    m_rescale_factor(1.f),
	m_preview_idx(INDEX_NONE),
    m_cached_preview_idx(INDEX_NONE),
    m_vb_preview_size(0),
    m_vb_mesh_size(0)
{
	Memory::MemZero(m_varrays);
	Memory::MemZero(m_vbuffers);
}

CoAttractor::~CoAttractor()
{

}

void CoAttractor::Create( Entity* owner, class json::Object* proto )
{
	Super::Create( owner, proto );

	json::TokenIdx ent_tok = proto->GetToken("entity", json::OBJECT);
	json::TokenIdx attractor_tok = proto->GetToken("Attractor", json::OBJECT, ent_tok);

	if (attractor_tok != INDEX_NONE)
	{
		json::TokenIdx param_tok = proto->GetToken("type", json::STRING, attractor_tok);
		if (param_tok != INDEX_NONE)
		{
			String attractor_str;
			proto->GetStringValue(param_tok, attractor_str);
			m_attractor = AttractorManager::GetStaticInstance()->GetFactory()->CreateAttractorType(attractor_str);
		}
		if (m_attractor)
		{
			/*param_tok = proto->GetToken("iter", json::PRIMITIVE, attractor_tok);
			if (param_tok != INDEX_NONE)
				m_line_params.iter = proto->GetIntValue(param_tok, m_line_params.iter);
*/
			/*param_tok = proto->GetToken("warmup_iter", json::PRIMITIVE, attractor_tok);
			if (param_tok != INDEX_NONE)
				m_line_params.warmup_iter = proto->GetIntValue(param_tok, m_line_params.warmup_iter);*/

			param_tok = proto->GetToken("merge_dist", json::PRIMITIVE, attractor_tok);
			if (param_tok != INDEX_NONE)
				m_shape_params.merge_dist = proto->GetFloatValue(param_tok, m_shape_params.merge_dist);

			
		}
	}
}

void CoAttractor::Destroy()
{
	BB_DELETE(m_attractor);

	Super::Destroy();
}

void CoAttractor::AddToWorld()
{
	RebuildAttractorMesh(true);
    
	Super::AddToWorld();
}

void CoAttractor::ChangeAttractorType(eAttractorType type)
{
    if( m_attractor && type == m_attractor->m_type )
        return;
    
    BB_DELETE(m_attractor);
    
    m_attractor = AttractorManager::GetStaticInstance()->GetFactory()->CreateAttractorType(type);
    RebuildAttractorMesh(true);
}

void CoAttractor::InsertCurve(int32 at_idx)
{
    int32 num_curve = m_curves.size();
    if (at_idx <= 0 || at_idx > num_curve )
        return;
    
    AttractorOrientedCurve curve;
    if (at_idx < num_curve)
    {
        //handle.m_line_idx = (m_handles[at_idx - 1].m_line_idx + m_handles[at_idx].m_line_idx) / 2;
        //handle.m_mesh_idx = handle.m_line_idx;
        m_curves.insert(curve, at_idx);
    }
    else
    {
        //handle.m_line_idx = (m_handles[at_idx - 1].m_line_idx + 1);
        //handle.m_mesh_idx = handle.m_line_idx;
        m_curves.push_back(curve);
    }
}

void CoAttractor::DeleteCurve(int32 at_idx)
{
    int32 num_curve = m_curves.size();
    if (at_idx <= 0 || at_idx >= num_curve - 1 )
        return;
    
    m_curves.erase(at_idx);
}

void CoAttractor::RebuildAttractorMesh(bool force_rebuild/*, bool keep_handle*/)
{
    CoHandle* cohandle = static_cast<CoHandle*>(GetEntityComponent("CoHandle"));
    if (!m_attractor || !cohandle)
        return;
    
    if (cohandle->m_handles.size() == 0)
    {
        cohandle->m_handles.push_back(AttractorHandle());
    }
    
    //bool line_changed = false;
    bool need_update_preview_buffer = (m_preview_idx != m_cached_preview_idx);
    bool need_update_mesh_buffer = (m_preview_idx != m_cached_preview_idx);
    
    int32 num_handle = cohandle->m_handles.size();
    m_curves.resize(num_handle);
    for (int h_idx = 0; h_idx < num_handle; h_idx++)
    {
        if (force_rebuild || !(m_line_params == m_cached_line_params) || cohandle->HasHandleChanged(h_idx))
        {
            AttractorOrientedCurve& curve = m_curves[h_idx];
            AttractorHandle& handle = cohandle->m_handles[h_idx];
            curve.points.clear();
            SAUtils::ComputeStrangeAttractorPoints(*m_attractor, handle.m_seed, m_line_params, curve.points);
            handle.m_idx_on_curve = handle.m_seed.rev_iter;
            SAUtils::GenerateFrames(curve);
            SAUtils::GenerateColors(curve, 0.f);
            curve.SetDefaultRanges();
            
            if (h_idx == m_preview_idx)
                need_update_preview_buffer = true;
            else
                need_update_mesh_buffer = true;
        }
    }

    if (need_update_mesh_buffer)
    {
        // Merge the end of the curves
        if (m_shape_params.merge_dist > 0.0f)
        {
            SAUtils::MergeCurves(m_curves, m_shape_params);
        }
        
        // Adapt size
        if (m_line_params.target_dim > 0.0f)
        {
            vec3 min_pos(FLT_MAX, FLT_MAX, FLT_MAX), max_pos(-FLT_MAX, -FLT_MAX, -FLT_MAX);
            for (int h_idx = 0; h_idx < num_handle; h_idx++)
            {
                AttractorOrientedCurve& curve = m_curves[h_idx];
                for (int32 i = 0; i < curve.points.size(); ++i)
                {
                    min_pos.x = bigball::min(min_pos.x, curve.points[i].x);
                    min_pos.y = bigball::min(min_pos.y, curve.points[i].y);
                    min_pos.z = bigball::min(min_pos.z, curve.points[i].z);
                    max_pos.x = bigball::max(max_pos.x, curve.points[i].x);
                    max_pos.y = bigball::max(max_pos.y, curve.points[i].y);
                    max_pos.z = bigball::max(max_pos.z, curve.points[i].z);
                }
            }
            vec3 V = max_pos - min_pos;
            float dim_max = max(max(V.x, V.y), V.z);
            float rescale = m_line_params.target_dim / dim_max;
            m_rescale_factor = rescale;
        }
    }
    
    // preview geometry
    if (force_rebuild || need_update_preview_buffer || !(m_shape_params == m_cached_shape_params) )
    {
        const AttractorOrientedCurve* curve_preview = GetCurvePreview();
        if (curve_preview)
        {
            m_tri_vertices_preview.clear();
            m_tri_normals_preview.clear();
            m_tri_indices_preview.clear();

            m_shape_params.weld_vertex = false;
            
            SAUtils::GenerateSolidMesh(*curve_preview, m_shape_params, m_tri_vertices_preview, &m_tri_normals_preview, m_tri_indices_preview, m_rescale_factor);
            
            need_update_preview_buffer = true;
        }
    }
    
    // curves geometry
    if (force_rebuild || need_update_mesh_buffer || !(m_shape_params == m_cached_shape_params) )
    {
        m_tri_vertices.clear();
        m_tri_normals.clear();
        m_tri_indices.clear();
        m_indice_offsets.clear();
        m_show_curve_alphas.resize(m_curves.size(), 1.f);
        
        m_shape_params.weld_vertex = false;
        
        SAUtils::GenerateSolidMesh(m_curves, m_shape_params, m_tri_vertices, &m_tri_normals, nullptr, m_tri_indices, &m_indice_offsets, m_rescale_factor);
        
        need_update_mesh_buffer = true;
    }
    
    if (need_update_mesh_buffer || need_update_preview_buffer)
    {
        vec3 min_box(FLT_MAX, FLT_MAX, FLT_MAX);
        vec3 max_box(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        int32 num_vertices = m_tri_vertices.size();
        for( int32 i = 0; i < num_vertices; i++ )
        {
            vec3 pos = m_tri_vertices[i];
            min_box = min(min_box, pos);
            max_box = max(max_box, pos);
        }
        if(GetCurvePreview())
        {
            num_vertices = m_tri_vertices_preview.size();
            for( int32 i = 0; i < num_vertices; i++ )
            {
                vec3 pos = m_tri_vertices_preview[i];
                min_box = min(min_box, pos);
                max_box = max(max_box, pos);
            }

        }
        m_min_box = min_box;
        m_max_box = max_box;
    }
    
    // Update transform
    const float scale = 0.01f;
    //const float z_offset = -m_min_box.z;
    
    transform t;
    t.Set(quat(1.f), vec3(0.f, 0.f, 0.f/*z_offset*/) * scale, scale);
    CoPosition* copos = static_cast<CoPosition*>(GetEntityComponent("CoPosition"));
    copos->SetTransform( t );
    
    UpdateVertexBuffers(need_update_preview_buffer, need_update_mesh_buffer);
    
    // save current params
    m_cached_line_params = m_line_params;
    m_cached_shape_params = m_shape_params;
    m_cached_preview_idx = m_preview_idx;
    
    cohandle->SaveHandleArray();
}

void CoAttractor::UpdateVertexBuffers(bool update_preview, bool update_mesh)
{
	// Clean previous data
	//glDeleteBuffers(eVBCount, m_vbuffers);
	//glDeleteVertexArrays(eVACount, m_varrays);


	// LINE POINTS
    const AttractorOrientedCurve* curve_preview = GetCurvePreview();
    if (update_preview && curve_preview)
    {
        if (m_vb_preview_size < m_tri_vertices_preview.size())
        {
            glDeleteVertexArrays(1, &m_varrays[eVAMeshPreview]);
            glDeleteBuffers( 3, &m_vbuffers[eVBMeshPreview] );
            
            glGenVertexArrays(1, &m_varrays[eVAMeshPreview]);
            glGenBuffers(3, &m_vbuffers[eVBMeshPreview]);
        }
        
        glBindVertexArray(m_varrays[eVAMeshPreview]);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        //glEnableVertexAttribArray(2);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbuffers[eVBMeshEltPreview]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_tri_indices_preview.size() * sizeof(int32), m_tri_indices_preview.Data(), GL_DYNAMIC_DRAW);
        
        glBindBuffer(GL_ARRAY_BUFFER, m_vbuffers[eVBMeshPreview]);
        glBufferData(GL_ARRAY_BUFFER, m_tri_vertices_preview.size() * sizeof(vec3), m_tri_vertices_preview.Data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3) /*stride*/, (void*)0 /*offset*/);
        
        glBindBuffer(GL_ARRAY_BUFFER, m_vbuffers[eVBMeshNormalsPreview]);
        glBufferData(GL_ARRAY_BUFFER, m_tri_normals_preview.size() * sizeof(vec3), m_tri_normals_preview.Data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vec3) /*stride*/, (void*)0 /*offset*/);
        
        glBindVertexArray(0);
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        
        m_vb_preview_size = bigball::max( m_vb_preview_size, m_tri_vertices_preview.size());
//        if (m_vb_preview_size < curve_preview->points.size())
//        {
//            glDeleteVertexArrays(1, &m_varrays[eVALinePoints]);
//            glDeleteBuffers( 1, &m_vbuffers[eVBLinePoints] );
//            
//            glGenVertexArrays(1, &m_varrays[eVALinePoints]);
//            glGenBuffers(1, &m_vbuffers[eVBLinePoints]);
//        }
//
//        glBindVertexArray(m_varrays[eVALinePoints]);
//        glEnableVertexAttribArray(0);
//        
//        glBindBuffer(GL_ARRAY_BUFFER, m_vbuffers[eVBLinePoints]);
//        glBufferData(GL_ARRAY_BUFFER, curve_preview->points.size() * sizeof(vec3), curve_preview->points.Data(), GL_DYNAMIC_DRAW);
//        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3) /*stride*/, (void*)0 /*offset*/);
//
//        glBindVertexArray(0);
//        glDisableVertexAttribArray(0);
//        
//        m_vb_line_size = bigball::max( m_vb_line_size, curve_preview->points.size());
    }

	// MESH
    if (update_mesh)
    {
        if (m_vb_mesh_size < m_tri_vertices.size())
        {
            glDeleteVertexArrays(1, &m_varrays[eVAMesh]);
            glDeleteBuffers( 3, &m_vbuffers[eVBMesh] );

            glGenVertexArrays(1, &m_varrays[eVAMesh]);
            glGenBuffers(3, &m_vbuffers[eVBMesh]);
        }
        
        glBindVertexArray(m_varrays[eVAMesh]);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        //glEnableVertexAttribArray(2);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbuffers[eVBMeshElt]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_tri_indices.size() * sizeof(int32), m_tri_indices.Data(), GL_STATIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbuffers[eVBMesh]);
        glBufferData(GL_ARRAY_BUFFER, m_tri_vertices.size() * sizeof(vec3), m_tri_vertices.Data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3) /*stride*/, (void*)0 /*offset*/);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbuffers[eVBMeshNormals]);
        glBufferData(GL_ARRAY_BUFFER, m_tri_normals.size() * sizeof(vec3), m_tri_normals.Data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vec3) /*stride*/, (void*)0 /*offset*/);

        //glBindBuffer(GL_ARRAY_BUFFER, m_vbuffers[eVBMeshColors]);
        //glBufferData(GL_ARRAY_BUFFER, m_tri_colors.size() * sizeof(float), m_tri_colors.Data(), GL_STATIC_DRAW);
        //glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float) /*stride*/, (void*)0 /*offset*/);

        glBindVertexArray(0);
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        //glDisableVertexAttribArray(2);
        
        m_vb_mesh_size = bigball::max( m_vb_mesh_size, m_tri_vertices.size());
    }
}

void CoAttractor::RemoveFromWorld()
{
	Super::RemoveFromWorld();
}

void CoAttractor::Tick( TickContext& tick_ctxt )
{

}

bool CoAttractor::OnControllerInput( Camera* pcamera, ControllerInput const& input )
{
    return true;
}

bool CoAttractor::RayCast(vec3 const& ray_start, vec3 const& ray_end, const float ray_width, PickResult& pick_result)
{
    CoPosition* copos = static_cast<CoPosition*>(GetEntityComponent("CoPosition"));
    transform tf = copos->GetTransform();
    
	vec3 seg0 = tf.TransformPositionInverse(ray_start) / m_rescale_factor;
	vec3 seg1 = tf.TransformPositionInverse(ray_end) / m_rescale_factor;
    
	// brute force raycast atm
	const float sq_width = ray_width * ray_width;
    
    //pick_result.m_curve_idx = INDEX_NONE;
	//pick_result.m_line_idx = INDEX_NONE;
    //pick_result.m_dist = 1e8f;
    //pick_result.m_handle_idx = INDEX_NONE;
    
    bool is_picked = false;
    
	for (int32 c_idx = 0; c_idx < m_curves.size(); c_idx++)
	{
        AttractorOrientedCurve& curve = m_curves[c_idx];
        for (int32 i = 0; i < curve.points.size(); i++)
        {
            vec3 point = curve.points[i];
        
            // square dist to segment
            float t;
            float sq_dist = intersect::SquaredDistancePointSegment(point, seg0, seg1, t);
            t *= tf.GetScale();
            if (sq_dist < sq_width && t > 0.f && t < pick_result.m_ray_dist)
            {
                pick_result.m_handle_idx = c_idx;
                pick_result.m_point_idx = i;
                pick_result.m_ray_dist = t;
                is_picked = true;
            }
        }
	}
    
	if (is_picked)
    {
		//pick_result.m_hit_pos = t.TransformPosition(m_curves[pick_result.m_curve_idx].points[pick_result.m_line_idx]);
        return true;
    }
    
    return false;
}

const AttractorOrientedCurve* CoAttractor::GetCurvePreview() const
{
    int32 num_curve = m_curves.size();
    if (m_preview_idx < 0 || m_preview_idx > num_curve - 1 )
        return nullptr;
    
    return &m_curves[m_preview_idx];
}

vec3 CoAttractor::GetCurveWorldPos(int32 curve_ix, int32 point_idx)
{
    CoPosition const* copos = static_cast<CoPosition*>(GetEntityComponent("CoPosition"));
    AttractorOrientedCurve const& curve = m_curves[curve_ix];
    
    vec3 world_curve_pos = copos->GetTransform().TransformPosition(curve.points[point_idx] * m_rescale_factor);
    return world_curve_pos;
}

void CoAttractor::GetMeshRenderOffsetsWithoutPreview(ivec2& start_range, ivec2& end_range) const
{
    start_range.x = 0;
    start_range.y = m_tri_indices.size();
    end_range.x = m_tri_indices.size();;
    end_range.y = m_tri_indices.size();;
    
    int32 num_curve = m_curves.size();
    if (m_preview_idx < 0 || m_preview_idx > num_curve - 1 )
        return;
    
    start_range.y = m_indice_offsets[m_preview_idx];
    if(m_preview_idx < num_curve - 1)
        end_range.x = m_indice_offsets[m_preview_idx + 1];
}

vec3 CoAttractor::GuessCurvePos(int32 h_idx, int32 at_seed_offset)
{
    vec3 out_pos(0.f);
    CoHandle* cohandle = static_cast<CoHandle*>(GetEntityComponent("CoHandle"));
    if (!m_attractor || !cohandle)
        return out_pos;
    
    int32 num_handle = cohandle->m_handles.size();
    if (h_idx >= 0 && h_idx < num_handle)
    {
        AttractorHandle& handle = cohandle->m_handles[h_idx];
        SAUtils::IterateStrangeAttractorPoint(*m_attractor, handle.m_seed, m_line_params, at_seed_offset, out_pos);
    }
    return out_pos;
}

void CoAttractor::_Render( RenderContext& render_ctxt )
{
	static float global_time = 0.f;
	global_time += render_ctxt.m_delta_seconds;
	
}

void CoAttractor::Serialize(Archive& file)
{
    file.SerializeRaw(m_cached_line_params);
    m_cached_shape_params.Serialize(file);
    //file.SerializeRaw(m_cached_shape_params);
 
    eAttractorType type;
    if(file.IsReading())
    {
        file.SerializeRaw(type);
        
		BB_DELETE(m_attractor);
		m_attractor = AttractorManager::GetStaticInstance()->GetFactory()->CreateAttractorType(type);
    }
    else
    {
        type = GetAttractorType();
        file.SerializeRaw(type);
    }
}
void CoAttractor::PostLoad()
{
    m_line_params = m_cached_line_params;
    m_shape_params = m_cached_shape_params;
    
	RebuildAttractorMesh(true/*, true*/);
}

void CoAttractor::ExportAsObj(Archive& file)
{
    // can't use attractor buffers, as obj vertices need to be welded...
    Array<vec3> tri_vertices_export;
    Array<int32> tri_indices_export;
    
    AttractorShapeParams shape_params_export = m_shape_params;
    shape_params_export.weld_vertex = true;
    
    SAUtils::GenerateSolidMesh(m_curves, shape_params_export, tri_vertices_export, nullptr, nullptr, tri_indices_export, nullptr, m_rescale_factor);
    
    SAUtils::WriteObjFile(file, m_tri_vertices, m_tri_indices);
}
