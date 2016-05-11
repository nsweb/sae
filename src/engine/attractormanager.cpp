

#include "../sae.h"
#include "attractormanager.h"
#include "coattractor.h"
#include "cohandle.h"
#include "engine/controller.h"
#include "engine/camera.h"
#include "engine/coposition.h"
#include "engine/controller.h"
#include "gfx/gfxmanager.h"
#include "gfx/shader.h"
#include "gfx/rendercontext.h"
#include "gfx/drawutils.h"
#include "system/profiler.h"



STATIC_MANAGER_CPP(AttractorManager);

AttractorManager::AttractorManager() :
	m_bg_shader(nullptr),
	m_line_shader(nullptr),
	m_mesh_shader(nullptr),
	m_show_handles(false),
    m_prev_mouse_left_down(false)
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

	if (m_show_handles)
		DrawHandles(render_ctxt);
}

void AttractorManager::DrawAttractors( struct RenderContext& render_ctxt )
{
	transform cam2world_transform(render_ctxt.m_view.m_transform.GetRotation(), render_ctxt.m_view.m_transform.GetTranslation(), (float)render_ctxt.m_view.m_transform.GetScale());
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

void AttractorManager::DrawHandles(struct RenderContext& render_ctxt)
{
	for (int att_idx = 0; att_idx < m_attractors.size(); att_idx++)
	{
		CoAttractor* attractor = m_attractors[att_idx];
		CoHandle* cohandle = static_cast<CoHandle*>(attractor->GetEntityComponent("CoHandle"));
		CoPosition* copos = static_cast<CoPosition*>(attractor->GetEntityComponent("CoPosition"));

		const int num_points = attractor->m_line_points.size();
		const float cube_size = 2.f * copos->GetTransform().GetScale() * attractor->m_shape_params.fatness_scale;

		int32 num_handle = cohandle->m_handles.size();
		for (int32 h_idx = 0; h_idx < num_handle; h_idx++)
		{
			MeshHandle const& handle = cohandle->m_handles[h_idx];
			
			if (handle.m_mesh_idx >= 0 && handle.m_mesh_idx < num_points)
			{
				vec3 world_line_handle_pos = copos->GetTransform().TransformPosition(attractor->m_line_points[handle.m_line_idx]);
                quat world_line_handle_quat = copos->GetRotation() * attractor->m_frames[handle.m_line_idx];
                DrawUtils::GetStaticInstance()->PushOBB(transform(world_line_handle_quat, world_line_handle_pos, cube_size), u8vec4(255, 0, 255, 255), 0.5f, 0.5f);
                
                vec3 world_mesh_handle_pos = copos->GetTransform().TransformPosition(attractor->m_line_points[handle.m_mesh_idx]);
                DrawUtils::GetStaticInstance()->PushSphere(world_mesh_handle_pos, cube_size * 0.85, u8vec4(0, 255, 255, 255) );
			}
		}
	}
}

void AttractorManager::HandleScenePick(ControllerMouseState const& mouse_state)
{
    bool allow_picking = m_show_handles;
    if (!allow_picking || !m_attractors.size())
    {
        return;
    }
    
    // if not *dragging* (!mouse_state.m_left_down)
    // pick amongst handles
    //	if mouse_state.m_left_down -> select
    //						else   -> highlight
    // if *dragging*
    // if handle selected
    //		pick line on attractor
    bool dragging = m_editor_pick.m_attractor != nullptr && mouse_state.m_left_down && m_prev_mouse_left_down;
    
    float screen_width = (float)g_pEngine->GetDisplayMode().w;
    float screen_height = (float)g_pEngine->GetDisplayMode().h;
    float s_x = (2.0f * mouse_state.m_mouse_x) / screen_width - 1.0f;
    float s_y = 1.0f - (2.0f * mouse_state.m_mouse_y) / screen_height;
    mat4 proj_mat = Controller::GetStaticInstance()->GetRenderProjMatrix();
    CameraView const& view = Controller::GetStaticInstance()->GetRenderView();
    
    vec4 ray_clip = vec4(s_x, s_y, -1.f, 1.f);
    vec4 ray_eye_h = inverse(proj_mat) * ray_clip;
    vec3 ray_eye = vec3(ray_eye_h.xy, -1.0);
    vec3 ray_world = normalize(view.m_transform.TransformVector(ray_eye));
    
    vec3 cam_pos = view.m_transform.GetTranslation();
    /*quat cam_rot = render_ctxt.m_view.m_Transform.GetRotation();
     mat3 cam_to_world( cam_rot );
     vec3 cam_front = -cam_to_world.v2.xyz;*/
    
    const float max_ray_dist = 10.f;
    vec3 ray_end = cam_pos + ray_world/*cam_front*/ * max_ray_dist;
    
    for (int att_idx = 0; att_idx < m_attractors.size(); att_idx++)
    {
        CoAttractor* attractor = m_attractors[0];
    
        const float ray_width = attractor->m_shape_params.fatness_scale;
        vec3 hit_result;
        if (attractor->RayCast(cam_pos, ray_end, ray_width, hit_result))
        {
            CoPosition* copos = static_cast<CoPosition*>(attractor->GetEntityComponent("CoPosition"));
            const float cube_size = copos->GetTransform().GetScale() * attractor->m_shape_params.fatness_scale;
            DrawUtils::GetStaticInstance()->PushAABB(hit_result, cube_size, u8vec4(255, 0, 255, 255));
        }
    }
}

void AttractorManager::SetShowHandles(bool show)
{
	m_show_handles = show;
}
