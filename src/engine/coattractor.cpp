

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
    m_max_box(0.f, 0.f, 0.f)
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

void CoAttractor::RebuildAttractorMesh(bool force_rebuild)
{
	m_line_points.clear();
	m_tri_vertices.clear();
	m_tri_normals.clear();
	m_tri_indices.clear();
 
	CoHandle* cohandle = static_cast<CoHandle*>(GetEntityComponent("CoHandle"));
	if (!m_attractor || !cohandle)
        return;
	
	if (force_rebuild || !(m_line_params == m_cached_line_params))
	{
		SAUtils::ComputeStrangeAttractorPoints(m_attractor, m_line_params, m_line_points);
	}

	// enforce cohandle ends
	while (cohandle->m_handles.size() < 2)
		cohandle->m_handles.push_back(MeshHandle());

	//cohandle->m_handles[0].m_type = MeshHandle::eHT_Begin;
    cohandle->m_handles[0].m_init_idx = 1;
	cohandle->m_handles[0].m_real_idx = 1;
	//cohandle->m_handles.Last().m_type = MeshHandle::eHT_End;
    cohandle->m_handles.Last().m_init_idx = m_line_points.size() - 2;
	cohandle->m_handles.Last().m_real_idx = m_line_points.size() - 2;

	if (force_rebuild || !(m_shape_params == m_cached_shape_params) || cohandle->HasHandleArrayChanged())
	{
		m_shape_params.weld_vertex = false;
		SAUtils::GenerateSolidMesh(m_line_points, m_shape_params, m_tri_vertices, &m_tri_normals, m_tri_indices);
	}
    
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
    
    // Update transform
    const float scale = 0.01f;
    const float z_offset = -min_box.z;
    
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
	glBufferData(GL_ARRAY_BUFFER, m_line_points.size() * sizeof(vec3), m_line_points.Data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3) /*stride*/, (void*)0 /*offset*/);

	glBindVertexArray(0);
	glDisableVertexAttribArray(0);

	// MESH
	glBindVertexArray(m_varrays[eVAMesh]);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbuffers[eVBMeshElt]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_tri_indices.size() * sizeof(int32), m_tri_indices.Data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbuffers[eVBMesh]);
	glBufferData(GL_ARRAY_BUFFER, m_tri_vertices.size() * sizeof(vec3), m_tri_vertices.Data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3) /*stride*/, (void*)0 /*offset*/);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbuffers[eVBMeshNormals]);
	glBufferData(GL_ARRAY_BUFFER, m_tri_normals.size() * sizeof(vec3), m_tri_normals.Data(), GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vec3) /*stride*/, (void*)0 /*offset*/);

	glBindVertexArray(0);
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
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

bool CoAttractor::RayCast(RayCastParams const& params, RayCastResults& results)
{
    CoPosition* copos = static_cast<CoPosition*>(GetEntityComponent("CoPosition"));
    transform t = copos->GetTransform();
    
    vec3 seg0 = t.TransformPositionInverse(params.m_start);
    vec3 seg1 = t.TransformPositionInverse(params.m_end );
    
	// brute force raycast atm
    const float sq_width = params.m_capsule_width * params.m_capsule_width;
    
    float min_t = FLT_MAX;
    int32 min_idx = INDEX_NONE;
	for (int32 i = 0; i < m_line_points.size(); i++)
	{
        vec3 point = m_line_points[i];
        
		// square dist to segment
        float t;
		float sq_dist = intersect::SquaredDistancePointSegment(point, seg0, seg1, t);
        if (sq_dist < sq_width && t < min_t)
        {
            min_t = t;
            min_idx = i;
        }
	}
    
    if (min_idx != INDEX_NONE)
    {
        results.m_hit = t.TransformPosition(m_line_points[min_idx]);
        return true;
    }
    
    return false;
}

void CoAttractor::_Render( RenderContext& render_ctxt )
{
	static float global_time = 0.f;
	global_time += render_ctxt.m_delta_seconds;
	
}

