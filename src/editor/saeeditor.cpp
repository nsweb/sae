// saeeditor.cpp
//

#include "../sae.h"
#include "saeeditor.h"

//#include "engine/entity.h"
//#include "engine/entitymanager.h"
#include "engine/controller.h"
#include "engine/coposition.h"
#include "ui/uimanager.h"
#include "gfx/rendercontext.h"
#include "gfx/drawutils.h"
#include "../engine/attractormanager.h"
#include "../engine/coattractor.h"
#include "../engine/cohandle.h"
#include "../engine/saecamera.h"


SAEEditor* SAEEditor::ms_peditor = nullptr;

SAEEditor::SAEEditor() :
    m_current_attractor_type(INDEX_NONE)
{
    ms_peditor = this;
}

SAEEditor::~SAEEditor()
{
    ms_peditor = nullptr;
}

static void UIOnToggleEditorCB( bool bshow_editor )
{

}

static void UIDrawEditorCB( bool* bshow_editor, bigball::RenderContext& render_ctxt )
{
    SAEEditor::Get()->UIDrawEditor( bshow_editor, render_ctxt );
}
static void UIDrawEditorMenusCB(bigball::RenderContext& render_ctxt)
{
	SAEEditor::Get()->UIDrawEditorMenus(render_ctxt);
}

bool SAEEditor::GetItemStringArray( void* data, int idx, const char** out_text )
{
	Array<String>* str_array = (Array<String>*)data;
	if( idx >= 0 && idx < str_array->size() )
	{
		*out_text = (*str_array)[idx].c_str();
		return true;
	}

	return false;
}

void SAEEditor::UIDrawEditor( bool* bshow_editor, RenderContext& render_ctxt )
{
    CameraCtrl_Base* cam_ctrl = Controller::GetStaticInstance()->GetActiveCameraCtrl();
    SAECameraCtrl_Editor* cam_edit = nullptr;
    if( cam_ctrl && cam_ctrl->IsA( SAECameraCtrl_Editor::StaticClass() ) )
        cam_edit = static_cast<SAECameraCtrl_Editor*>( cam_ctrl );
    
    ImGui::Begin("Editor", bshow_editor, ImVec2(200,400), -1.f, 0/*ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar*/ );

	ImGui::ShowTestWindow();

    ImGui::End();
}

void SAEEditor::UIDrawEditorMenus(RenderContext& render_ctxt)
{
	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("File"))
		{
			if (ImGui::MenuItem("New attractor"))
			{
				//ImGui::OpenPopup("popup");
			}
			if (ImGui::MenuItem("Save attractor"))
			{
				//ImGui::OpenPopup("popup");
			}
			if (ImGui::MenuItem("Load attractor"))
			{
				//ImGui::OpenPopup("popup");
			}
			if (ImGui::MenuItem("Export Obj"))
			{
				//ImGui::OpenPopup("popup");
			}
			ImGui::EndMenu();
		}
        //if (ImGui::BeginMenu("menu2"))
        //{
        //    if (ImGui::MenuItem("menu item"))
        //        ImGui::OpenPopup("popup");
        //    ImGui::EndMenu();
        //}
		ImGui::EndMainMenuBar();
	}
    
    DrawRightPanel(render_ctxt);
}

void SAEEditor::DrawRightPanel(bigball::RenderContext& render_ctxt)
{
    Array<CoAttractor*> const& attractors = AttractorManager::GetStaticInstance()->GetAttractors();
	if (!attractors.size())
		return;

	CoAttractor* attractor = attractors[0];
	CoHandle* cohandle = static_cast<CoHandle*>(attractor->GetEntityComponent("CoHandle"));
    
    bool show_pane = false;
    SDL_DisplayMode display_mode = g_pEngine->GetDisplayMode();
    const float menu_height = 19.f;
    const float pane_width = 400.f;
    float pane_pos_x = display_mode.w - pane_width;
    float pane_height = display_mode.h - menu_height;
    ImGui::SetNextWindowPos(ImVec2(pane_pos_x, menu_height));
    ImGui::Begin("Editor", &show_pane, ImVec2(pane_width, pane_height), -1.f, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoResize );
    
	if (ImGui::CollapsingHeader("Attractor"))
	{
        if(m_current_attractor_type == INDEX_NONE)
            m_current_attractor_type = attractor->GetAttractorType();
        
		Array<String> attractor_names;
		SAUtils::GetAttractorTypeList(attractor_names);

		ImGui::PushItemWidth(120);
        ImGui::Combo("", &m_current_attractor_type, GetItemStringArray, &attractor_names, attractor_names.size());
		ImGui::PopItemWidth();
        if( m_current_attractor_type != attractor->GetAttractorType() )
        {
            ImGui::SameLine();
            if (ImGui::Button("Confirm change"))
            {
                attractor->ChangeAttractorType((eAttractorType)m_current_attractor_type);
                m_current_attractor_type = attractor->GetAttractorType();
            }
        }
	}

    if( ImGui::CollapsingHeader("Params") )
    {
		ImGui::InputFloat3("Attractor seed", (float*)&attractor->m_line_params.seed);

		ImGui::PushItemWidth(125);
		ImGui::InputInt("Iteration steps", &attractor->m_line_params.iter, 1, 100);// , ImGuiInputTextFlags extra_flags = 0);
		ImGui::InputInt("Reverse iter. steps", &attractor->m_line_params.rev_iter, 1, 100);
		ImGui::InputInt("Warmup iter. steps", &attractor->m_line_params.warmup_iter, 1, 100);
		ImGui::InputFloat("Step size", &attractor->m_line_params.step_factor);
		ImGui::InputInt("Simplify step", &attractor->m_shape_params.simplify_level, 1, 10);

		ImGui::Separator();
        
        ImGui::InputFloat("Line fatness", &attractor->m_shape_params.fatness_scale);
		ImGui::InputInt("Edge count", &attractor->m_shape_params.local_edge_count, 1, 10);
		ImGui::InputFloat("Crease depth", &attractor->m_shape_params.crease_depth);
		ImGui::InputFloat("Crease width", &attractor->m_shape_params.crease_width);
		ImGui::InputFloat("crease bevel", &attractor->m_shape_params.crease_bevel);

		ImGui::Separator();

		ImGui::InputFloat("Target dimension", &attractor->m_line_params.target_dim);
		ImGui::SliderAngle("Shearing angle", &attractor->m_line_params.shearing_angle, -90.f, 90.f);
		ImGui::InputFloat("Shearing scale x", &attractor->m_line_params.shearing_scale_x, 0.01f, 0.05f, 3);
		ImGui::InputFloat("Shearing scale y", &attractor->m_line_params.shearing_scale_y, 0.01f, 0.05f, 3);

		ImGui::PopItemWidth();

		ImGui::Separator();
		
		if (ImGui::Button("Rebuild"))
		{ 
			attractor->RebuildAttractorMesh();
		}
    }

	if (ImGui::CollapsingHeader("Stats"))
	{
		int32 tri_count = attractor->m_tri_indices.size() / 3;
		ImGui::InputInt("Triangle count", &tri_count, -1, ImGuiInputTextFlags_ReadOnly);
        vec3 dimensions = attractor->m_max_box - attractor->m_min_box;
        ImGui::InputFloat3("Dimensions", (float*)&dimensions, 2, ImGuiInputTextFlags_ReadOnly);
	}
    
	bool show_modifications = ImGui::CollapsingHeader("Modifications");
	AttractorManager::GetStaticInstance()->SetShowHandles(show_modifications);

	if (show_modifications)
    {
		int32 num_handles = cohandle->m_handles.size();

		ImGui::PushItemWidth(50);
		Array<String> str_handle_array;
		for (int32 h_idx = 0; h_idx < num_handles; h_idx++)
		{
			str_handle_array.push_back(String::Printf("%d", h_idx));
		}

		ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.f), "Handles");
        ImGui::ListBox("", &cohandle->m_selected_handle_idx, GetItemStringArray, &str_handle_array, num_handles, 6);
		ImGui::PopItemWidth();

		ImGui::SameLine();
		ImGui::BeginGroup();
        
        if (cohandle->m_selected_handle_idx >= 0 && cohandle->m_selected_handle_idx < num_handles)
        {
            bool first = (cohandle->m_selected_handle_idx == 0);
            bool last = (cohandle->m_selected_handle_idx == num_handles - 1);
            if ( (!first && ImGui::Button("Insert before", ImVec2(0,20))) ||
                 (first && ImGui::InvisibleButton("", ImVec2(1,20))))
            {
                cohandle->InsertHandle( cohandle->m_selected_handle_idx );
            }
            if ( (!last && ImGui::Button("Insert after", ImVec2(0,20))) ||
                 (last && ImGui::InvisibleButton("", ImVec2(1,20))))
            {
                cohandle->InsertHandle( cohandle->m_selected_handle_idx + 1 );
            }
            if (!first &&!last)
            if (ImGui::Button("Delete"))
            {
                cohandle->DeleteHandle( cohandle->m_selected_handle_idx );
            }
        }
		ImGui::EndGroup();

		//static float value = 0.5f;
		//if (ImGui::BeginPopupContextItem("item context menu"))
		//{
		//	if (ImGui::Selectable("Set to zero")) value = 0.0f;
		//	if (ImGui::Selectable("Set to PI")) value = 3.1415f;
		//	ImGui::EndPopup();
		//}

    }

    ImGui::End();
}

void SAEEditor::HandleScenePick(ControllerMouseState const& mouse_state)
{
	Array<CoAttractor*> const& attractors = AttractorManager::GetStaticInstance()->GetAttractors();
	bool allow_picking = AttractorManager::GetStaticInstance()->GetShowHandles();
	if (!allow_picking || !attractors.size())
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

	CoAttractor* attractor = attractors[0];

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

	const float ray_width = attractor->m_shape_params.fatness_scale;
	vec3 hit_result;
	if (attractor->RayCast(cam_pos, ray_end, ray_width, hit_result))
	{
		CoPosition* copos = static_cast<CoPosition*>(attractor->GetEntityComponent("CoPosition"));
		const float cube_size = copos->GetTransform().GetScale() * attractor->m_shape_params.fatness_scale;
		DrawUtils::GetStaticInstance()->PushAABB(hit_result, cube_size, u8vec4(255, 0, 255, 255));
	}
}

bool SAEEditor::Init()
{
	UIManager::GetStaticInstance()->SetDrawEditorFn( nullptr /*&UIDrawEditorCB*/ );
	UIManager::GetStaticInstance()->SetToggleEditorFn( &UIOnToggleEditorCB );
	UIManager::GetStaticInstance()->SetDrawCustomMenuFn(&UIDrawEditorMenusCB);

	// allow mouse - menu interactions
	//SDL_SetRelativeMouseMode(SDL_FALSE);
	//SDL_SetWindowGrab(g_pEngine->GetDisplayWindow(), SDL_FALSE);

	return true;
}

void SAEEditor::Shutdown()
{

}
