
#include "../sae.h"
#include "coattractor.h"
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
	m_view_handle_range(2000)
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
			m_attractor = SAUtils::CreateAttractorType(attractor_str);
		}
		if (m_attractor)
		{
			param_tok = proto->GetToken("iter", json::PRIMITIVE, attractor_tok);
			if (param_tok != INDEX_NONE)
				m_line_params.iter = proto->GetIntValue(param_tok, m_line_params.iter);

			param_tok = proto->GetToken("warmup_iter", json::PRIMITIVE, attractor_tok);
			if (param_tok != INDEX_NONE)
				m_line_params.warmup_iter = proto->GetIntValue(param_tok, m_line_params.warmup_iter);

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
    
    m_attractor = SAUtils::CreateAttractorType(type);
    RebuildAttractorMesh(true);
}

void CoAttractor::RebuildAttractorMesh(bool force_rebuild, bool keep_handle)
{
	CoHandle* cohandle = static_cast<CoHandle*>(GetEntityComponent("CoHandle"));
	if (!m_attractor || !cohandle)
        return;
	
    bool line_changed = false;
	if (force_rebuild || !(m_line_params == m_cached_line_params))
	{
		m_line_framed.Clear();

		SAUtils::ComputeStrangeAttractorPoints(*m_attractor, m_line_params, m_line_framed.points, m_rescale_factor);
		SAUtils::GenerateFrames(m_line_framed);
        SAUtils::GenerateColors(m_line_framed, 0.f);
		m_line_framed.SetDefaultRanges();
		if (!keep_handle)
			cohandle->m_handles.clear();
        line_changed = true;
	}

	// enforce cohandle ends
	while (cohandle->m_handles.size() < 2)
    {
		cohandle->m_handles.push_back(AttractorHandle());
        if( cohandle->m_handles.size() == 1 )
        {
            //cohandle->m_handles[0].m_type = MeshHandle::eHT_Begin;
            cohandle->m_handles[0].m_line_idx = 1;
            cohandle->m_handles[0].m_mesh_idx = 1;
        }
        else
        {
            //cohandle->m_handles.Last().m_type = MeshHandle::eHT_End;
			cohandle->m_handles.Last().m_line_idx = m_line_framed.points.size() - 2;
			cohandle->m_handles.Last().m_mesh_idx = m_line_framed.points.size() - 2;
        }
    }

	if (force_rebuild || line_changed || !(m_shape_params == m_cached_shape_params) || cohandle->HasHandleArrayChanged())
	{
		m_snapped_lines.clear();

        m_tri_vertices.clear();
        m_tri_normals.clear();
		m_tri_colors.clear();
        m_tri_indices.clear();

        m_shape_params.weld_vertex = false;

		if (m_shape_params.merge_dist > 0.f)
			SAUtils::MergeLinePoints5(m_line_framed, cohandle->m_handles, m_shape_params, m_snapped_lines);
		else
			m_snapped_lines.push_back( m_line_framed );

		SAUtils::GenerateSolidMesh(m_snapped_lines, m_shape_params, m_tri_vertices, &m_tri_normals, &m_tri_colors, m_tri_indices);
	}
    
    //if (m_shape_params.freeze_bbox)
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
        m_min_box = min_box;
        m_max_box = max_box;
    }
    
    // Update transform
    const float scale = 0.01f;
    const float z_offset = -m_min_box.z;
    
    transform t;
    t.Set(quat(1.f), vec3(0.f, 0.f, z_offset) * scale, scale);
    CoPosition* copos = static_cast<CoPosition*>(GetEntityComponent("CoPosition"));
    copos->SetTransform( t );

	UpdateVertexBuffers();

	// save current params
	m_cached_line_params = m_line_params;
	m_cached_shape_params = m_shape_params;
	cohandle->SaveHandleArray();
}

void CoAttractor::UpdateVertexBuffers()
{
	// Clean previous data
	glDeleteBuffers(eVBCount, m_vbuffers);
	glDeleteVertexArrays(eVACount, m_varrays);


	// LINE POINTS
	glGenVertexArrays(eVACount, m_varrays);
	glGenBuffers(eVBCount, m_vbuffers);

	glBindVertexArray(m_varrays[eVALinePoints]);
	glEnableVertexAttribArray(0);
	
	glBindBuffer(GL_ARRAY_BUFFER, m_vbuffers[eVBLinePoints]);
	glBufferData(GL_ARRAY_BUFFER, m_line_framed.points.size() * sizeof(vec3), m_line_framed.points.Data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3) /*stride*/, (void*)0 /*offset*/);

	glBindVertexArray(0);
	glDisableVertexAttribArray(0);

	// MESH
	glBindVertexArray(m_varrays[eVAMesh]);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbuffers[eVBMeshElt]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_tri_indices.size() * sizeof(int32), m_tri_indices.Data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbuffers[eVBMesh]);
	glBufferData(GL_ARRAY_BUFFER, m_tri_vertices.size() * sizeof(vec3), m_tri_vertices.Data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3) /*stride*/, (void*)0 /*offset*/);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbuffers[eVBMeshNormals]);
	glBufferData(GL_ARRAY_BUFFER, m_tri_normals.size() * sizeof(vec3), m_tri_normals.Data(), GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vec3) /*stride*/, (void*)0 /*offset*/);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbuffers[eVBMeshColors]);
	glBufferData(GL_ARRAY_BUFFER, m_tri_colors.size() * sizeof(float), m_tri_colors.Data(), GL_STATIC_DRAW);
	glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float) /*stride*/, (void*)0 /*offset*/);

	glBindVertexArray(0);
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
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
    transform t = copos->GetTransform();
    
	vec3 seg0 = t.TransformPositionInverse(ray_start);
	vec3 seg1 = t.TransformPositionInverse(ray_end);
    
	// brute force raycast atm
	const float sq_width = ray_width * ray_width;
    
	pick_result.m_line_idx = INDEX_NONE;
	for (int32 i = 0; i < m_line_framed.points.size(); i++)
	{
		vec3 point = m_line_framed.points[i];
        
		// square dist to segment
        float t;
        float sq_dist = intersect::SquaredDistancePointSegment(point, seg0, seg1, t);
        if (sq_dist < sq_width)
        {
			pick_result.m_line_idx = i;
        }
	}
    
	if (pick_result.m_line_idx != INDEX_NONE)
    {
		pick_result.m_hit_pos = t.TransformPosition(m_line_framed.points[pick_result.m_line_idx]);
        return true;
    }
    
    return false;
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
		m_attractor = SAUtils::CreateAttractorType(type);
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
    
	RebuildAttractorMesh(true, true);
}

void CoAttractor::ExportAsObj(Archive& file)
{
    // can't use attractor buffers, as obj vertices need to be welded...
    Array<vec3> tri_vertices_export;
    Array<int32> tri_indices_export;
    
    AttractorShapeParams shape_params_export = m_shape_params;
    shape_params_export.weld_vertex = true;
    
    SAUtils::GenerateSolidMesh(m_snapped_lines, shape_params_export, tri_vertices_export, nullptr, nullptr, tri_indices_export);
    
    SAUtils::WriteObjFile(file, m_tri_vertices, m_tri_indices);
}
