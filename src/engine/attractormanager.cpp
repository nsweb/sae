

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
#include "system/file.h"

STATIC_MANAGER_CPP(AttractorManager);

AttractorManager::AttractorManager() :
    m_attractor_seed_range(1.f),
	m_bg_shader(nullptr),
	m_line_shader(nullptr),
	m_mesh_shader(nullptr),
	m_show_handles(false),
    m_show_lines(true),
    m_show_seeds(false),
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
    m_line_shader = GfxManager::GetStaticInstance()->LoadShader( "attractor_line" );
	m_mesh_shader = GfxManager::GetStaticInstance()->LoadShader("attractor");
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
    
    if (m_show_seeds)
        DrawAttractorSeeds(render_ctxt);
}

void AttractorManager::DrawAttractors( struct RenderContext& render_ctxt )
{
	transform cam2world_transform(render_ctxt.m_view.m_transform.GetRotation(), render_ctxt.m_view.m_transform.GetTranslation(), (float)render_ctxt.m_view.m_transform.GetScale());
    mat4 view_inv_mat( cam2world_transform.GetRotation(), cam2world_transform.GetTranslation(), cam2world_transform.GetScale() );
    mat4 view_mat = bigball::inverse(view_inv_mat);
    
    if( m_show_lines )
	{
        m_line_shader->Bind();
        
		ShaderUniform uni_world = m_line_shader->GetUniformLocation("world_mat");
		ShaderUniform uni_view = m_line_shader->GetUniformLocation("view_mat");
		m_line_shader->SetUniform(uni_view, view_mat);
		ShaderUniform uni_proj = m_line_shader->GetUniformLocation("proj_mat");
		m_line_shader->SetUniform(uni_proj, render_ctxt.m_proj_mat);
		ShaderUniform uni_color = m_line_shader->GetUniformLocation("color_info");

		for (int att_idx = 0; att_idx < m_attractors.size(); att_idx++)
		{
			CoAttractor* attractor = m_attractors[att_idx];
			CoPosition* copos = static_cast<CoPosition*>(attractor->GetEntityComponent("CoPosition"));
			CoHandle* cohandle = static_cast<CoHandle*>(attractor->GetEntityComponent("CoHandle"));

			float sel_idx = -1.f;
			if (m_show_handles && m_editor_selected.m_attractor == attractor && m_editor_selected.m_handle_idx != INDEX_NONE)
			{
				AttractorHandle const& handle = cohandle->GetHandle(m_editor_selected.m_handle_idx);
				sel_idx = (float)handle.m_mesh_idx;
			}

			mat4 world_mat(copos->GetTransform().ToMat4());
			m_line_shader->SetUniform(uni_world, world_mat);
			m_line_shader->SetUniform(uni_color, vec3(sel_idx, (float)attractor->m_view_handle_range, 1.0f));
			glBindVertexArray(attractor->m_varrays[CoAttractor::eVALinePoints]);
			const int line_count = attractor->m_line_framed.points.size() - 1;
			glDrawArrays(GL_LINE_STRIP, 0, line_count);
			glBindVertexArray(0);

		}
        
        m_line_shader->Unbind();
	}

	m_mesh_shader->Bind();
	{
		ShaderUniform uni_world = m_mesh_shader->GetUniformLocation("world_mat");
		ShaderUniform uni_view = m_mesh_shader->GetUniformLocation("view_mat");
		m_mesh_shader->SetUniform(uni_view, view_mat);
		ShaderUniform uni_proj = m_mesh_shader->GetUniformLocation("proj_mat");
		m_mesh_shader->SetUniform(uni_proj, render_ctxt.m_proj_mat);
		ShaderUniform uni_color = m_mesh_shader->GetUniformLocation("color_info");

		for (int att_idx = 0; att_idx < m_attractors.size(); att_idx++)
		{
			CoAttractor* attractor = m_attractors[att_idx];
			CoPosition* copos = static_cast<CoPosition*>(attractor->GetEntityComponent("CoPosition"));
			//CoHandle* cohandle = static_cast<CoHandle*>(attractor->GetEntityComponent("CoHandle"));

			float alpha = 1.f;
			if (m_show_handles && m_editor_selected.m_attractor == attractor && m_editor_selected.m_handle_idx != INDEX_NONE)
			{
				alpha = 0.5f;
			}

			mat4 world_mat(copos->GetTransform().ToMat4());
			m_mesh_shader->SetUniform(uni_world, world_mat);
			m_mesh_shader->SetUniform(uni_color, vec3(-1.f/*sel_idx*/, (float)attractor->m_view_handle_range, alpha));
			glBindVertexArray(attractor->m_varrays[CoAttractor::eVAMesh]);
			const int index_count = attractor->m_tri_indices.size();
			glDrawElements(GL_TRIANGLES, (GLsizei)index_count, GL_UNSIGNED_INT, 0);
			glBindVertexArray(0);

		}
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

		const int num_points = attractor->m_line_framed.points.size();
		const float cube_size = 2.f * copos->GetTransform().GetScale() * attractor->m_shape_params.fatness_scale;

		int32 num_handle = cohandle->m_handles.size();
		for (int32 h_idx = 0; h_idx < num_handle; h_idx++)
		{
			AttractorHandle const& handle = cohandle->GetHandle(h_idx);
			
			if (handle.m_mesh_idx >= 0 && handle.m_mesh_idx < num_points)
			{
				vec3 world_line_handle_pos = copos->GetTransform().TransformPosition(attractor->m_line_framed.points[handle.m_line_idx]);
				quat world_line_handle_quat = copos->GetRotation() * attractor->m_line_framed.frames[handle.m_line_idx];
				bool is_line_selected = m_editor_selected.m_attractor == attractor && m_editor_selected.m_handle_idx == h_idx && m_editor_selected.m_line_handle;
				bool is_line_hovered = m_editor_hovered.m_attractor == attractor && m_editor_hovered.m_handle_idx == h_idx && m_editor_hovered.m_line_handle;
				u8vec4 line_col = is_line_selected ? u8vec4(255, 250, 130, 255) : is_line_hovered ? u8vec4(255, 127, 255, 255) : u8vec4(255, 0, 255, 255);
				DrawUtils::GetStaticInstance()->PushOBB(transform(world_line_handle_quat, world_line_handle_pos, cube_size), line_col, 0.5f, 0.5f);
                
				vec3 world_mesh_handle_pos = copos->GetTransform().TransformPosition(attractor->m_line_framed.points[handle.m_mesh_idx]);
				bool is_mesh_selected = m_editor_selected.m_attractor == attractor && m_editor_selected.m_handle_idx == h_idx && !m_editor_selected.m_line_handle;
				bool is_mesh_hovered = m_editor_hovered.m_attractor == attractor && m_editor_hovered.m_handle_idx == h_idx && !m_editor_hovered.m_line_handle;
				u8vec4 mesh_col = is_mesh_selected ? u8vec4(255, 250, 130, 255) : is_mesh_hovered ? u8vec4(160, 200, 255, 255) : u8vec4(0, 200, 255, 255);
				DrawUtils::GetStaticInstance()->PushSphere(world_mesh_handle_pos, cube_size * 0.85f, mesh_col);
			}
		}
	}
}

void AttractorManager::DrawAttractorSeeds(struct RenderContext& render_ctxt)
{
    for (int att_idx = 0; att_idx < m_attractors.size(); att_idx++)
    {
        CoAttractor* attractor = m_attractors[att_idx];
        CoPosition* copos = static_cast<CoPosition*>(attractor->GetEntityComponent("CoPosition"));
        
        u8vec4 seed_col = u8vec4(255, 250, 130, 255);
        const float cube_size = 1.f * copos->GetTransform().GetScale() * attractor->m_shape_params.fatness_scale;
        vec3 world_seed_pos = copos->GetTransform().TransformPosition(attractor->m_line_params.seed * attractor->m_rescale_factor);
        
        DrawUtils::GetStaticInstance()->PushAABB(world_seed_pos, cube_size, seed_col);
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
    bool dragging = m_editor_selected.m_attractor != nullptr && mouse_state.m_left_down && m_prev_mouse_left_down;
    
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
    
	bool ray_hit = false;
    for (int att_idx = 0; att_idx < m_attractors.size(); att_idx++)
    {
        CoAttractor* attractor = m_attractors[0];
        CoHandle* handle = static_cast<CoHandle*>(attractor->GetEntityComponent("CoHandle"));
    
		if (dragging)
		{
			if (attractor == m_editor_selected.m_attractor)
			{
				AttractorHandle& handle_selected = handle->GetHandle(m_editor_selected.m_handle_idx);
                
                CoHandle::PickResult current_result;
                if (handle->RayCast(cam_pos, ray_end, current_result))
                {
                    if (current_result.m_handle_idx == m_editor_selected.m_handle_idx)
                    {
                        // snap mesh to line
                        if (!m_editor_selected.m_line_handle && current_result.m_is_line_pick)
                            handle_selected.m_mesh_idx = handle_selected.m_line_idx;
                    }
                }
                else
                {
                    const float ray_width = attractor->m_shape_params.fatness_scale;
                    CoAttractor::PickResult pick_result;
                    if (attractor->RayCast(cam_pos, ray_end, ray_width, pick_result))
                    {
                        //CoPosition* copos = static_cast<CoPosition*>(attractor->GetEntityComponent("CoPosition"));
                        //const float cube_size = copos->GetTransform().GetScale() * attractor->m_shape_params.fatness_scale;
                        //DrawUtils::GetStaticInstance()->PushAABB(pick_result.m_hit_pos, cube_size, u8vec4(255, 0, 255, 255));

                        if (m_editor_selected.m_line_handle)
                        {
                            handle_selected.m_line_idx = pick_result.m_line_idx;
                        }
                        else
                        {
                            handle_selected.m_mesh_idx = pick_result.m_line_idx;
                        }
                    }
				}
			}
		}
		else
		{
			CoHandle::PickResult pick_result;
			if (handle->RayCast(cam_pos, ray_end, pick_result))
			{
				if (mouse_state.m_left_down)
				{
					m_editor_selected.m_attractor = attractor;
					m_editor_selected.m_handle_idx = pick_result.m_handle_idx;
					m_editor_selected.m_line_handle = pick_result.m_is_line_pick;
				}
				else
				{
					m_editor_hovered.m_attractor = attractor;
					m_editor_hovered.m_handle_idx = pick_result.m_handle_idx;
					m_editor_hovered.m_line_handle = pick_result.m_is_line_pick;
				}
				ray_hit = true;
				break;
			}
		}
    }

	if (!ray_hit)
	{
		m_editor_hovered.m_attractor = nullptr;
		m_editor_hovered.m_handle_idx = INDEX_NONE;
        
        if (mouse_state.m_left_down && !dragging)
        {
            m_editor_selected.m_attractor = nullptr;
            m_editor_selected.m_handle_idx = INDEX_NONE;
        }
	}

	// save button state to detect dragging next time
	m_prev_mouse_left_down = mouse_state.m_left_down;
}

void AttractorManager::SetShowHandles(bool show)
{
	m_show_handles = show;
}

void AttractorManager::SerializeAttractor(Archive& file)
{
    if (!m_attractors.size())
        return;
    
	Entity* attractor = m_attractors[0]->GetEntity();
	attractor->Serialize(file);
	if (file.IsReading())
	{
		attractor->PostLoad();
	}
}

void AttractorManager::ExportAttractorAsObj(Archive& file)
{
    if (!m_attractors.size())
        return;
    
    CoAttractor* attractor = m_attractors[0];
    attractor->ExportAsObj(file);
}
