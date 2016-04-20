

#include "../sae.h"
#include "attractormanager.h"
#include "coattractor.h"
#include "engine/controller.h"
#include "engine/camera.h"
#include "engine/coposition.h"
#include "gfx/gfxmanager.h"
#include "gfx/shader.h"
#include "gfx/rendercontext.h"
#include "system/profiler.h"



STATIC_MANAGER_CPP(AttractorManager);

AttractorManager::AttractorManager()
{
	m_pStaticInstance = this;
}

AttractorManager::~AttractorManager()
{
	m_pStaticInstance = nullptr;
}

void AttractorManager::Create()
{
	m_bg_shader = GfxManager::GetStaticInstance()->LoadShader( "bg_editor" );
    m_mesh_shader = GfxManager::GetStaticInstance()->LoadShader( "attractor" );
}

void AttractorManager::Destroy()
{
	m_bg_shader = nullptr;
    m_line_shader = nullptr;
    m_mesh_shader = nullptr;
}

void AttractorManager::AddComponentToWorld( Component* component )
{
    if( component->IsA( CoAttractor::StaticClass() ) )
    {
        m_attractors.push_back( reinterpret_cast<CoAttractor*>( component ) );
    }
}
void AttractorManager::RemoveComponentFromWorld( Component* component )
{
    if( component->IsA( CoAttractor::StaticClass() ) )
    {
        m_attractors.remove( reinterpret_cast<CoAttractor*>( component ) );
    }
}

void AttractorManager::Tick( TickContext& tick_ctxt )
{
	PROFILE_SCOPE( __FUNCTION__ );
}

void AttractorManager::_Render( RenderContext& render_ctxt )
{
	PROFILE_SCOPE( __FUNCTION__ );

    DrawAttractors(render_ctxt);
}

void AttractorManager::DrawAttractors( struct RenderContext& render_ctxt )
{
    transform cam2world_transform( render_ctxt.m_view.m_Transform.GetRotation(), render_ctxt.m_view.m_Transform.GetTranslation(), (float)render_ctxt.m_view.m_Transform.GetScale() );
    mat4 view_inv_mat( cam2world_transform.GetRotation(), cam2world_transform.GetTranslation(), cam2world_transform.GetScale() );
    mat4 view_mat = bigball::inverse(view_inv_mat);
    
    m_mesh_shader->Bind();
    
    ShaderUniform uni_world = m_mesh_shader->GetUniformLocation("world_mat");
    ShaderUniform uni_view = m_mesh_shader->GetUniformLocation("view_mat");
    m_mesh_shader->SetUniform( uni_view, view_mat );
    ShaderUniform uni_proj = m_mesh_shader->GetUniformLocation("proj_mat");
    m_mesh_shader->SetUniform( uni_proj, render_ctxt.m_proj_mat );
    
    for( int att_idx = 0; att_idx < m_attractors.size(); att_idx++ )
    {
        CoAttractor* attractor = m_attractors[att_idx];
        CoPosition* copos = static_cast<CoPosition*>(attractor->GetEntityComponent("CoPosition"));

		mat4 world_mat( copos->GetTransform().ToMat4() );
        m_mesh_shader->SetUniform( uni_world, world_mat );
        
        glBindVertexArray( attractor->m_varrays[CoAttractor::eVAMesh] );
        const int index_count = attractor->m_tri_indices.size();
		glDrawElements(GL_TRIANGLES, (GLsizei)index_count, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);


		glBindVertexArray( attractor->m_varrays[CoAttractor::eVALinePoints] );
		const int line_count = attractor->m_line_points.size() - 1;
		glDrawArrays(GL_LINE_STRIP, 0, line_count);
		glBindVertexArray(0);

    }
    
    m_mesh_shader->Unbind();
}

