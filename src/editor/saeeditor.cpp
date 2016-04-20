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
    m_current_attractor_type(INDEX_NONE),
	m_current_handle_idx(INDEX_NONE)
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

#if 1
	ImGui::ShowTestWindow();
#else

    if( ImGui::CollapsingHeader("Level") )
    {
        Array<String> str_level_array;
        Array<CoLevel*> const& levels = SAEWorld::GetStaticInstance()->GetLevelArray();
        for( int lvl_idx = 0; lvl_idx < levels.size(); lvl_idx++ )
            str_level_array.push_back( String::Printf( "%s", levels[lvl_idx]->m_level_name.c_str() ) );
        
        ImGui::PushItemWidth( 50 );
        if( ImGui::ListBox( "Levels", &m_current_lvl_idx, GetItemStringArray, &str_level_array, str_level_array.size(), 6 ) )
        {
            if( m_current_lvl_idx >= 0 && m_current_lvl_idx < levels.size() )
            {
                SAEWorld::GetStaticInstance()->SetCurrentLevel(m_current_lvl_idx);
                level = SAEWorld::GetStaticInstance()->GetCurrentLevel();
                pcopath = level ? static_cast<CoPath*>( level->GetEntityComponent( CoPath::StaticClass() ) ) : nullptr;
                pcoship->SetCurrentLevel( level->GetEntity() );
            }
        }
        ImGui::PopItemWidth();
    }
    if( ImGui::CollapsingHeader("Camera") )
    {
        ImGui::SliderFloat("strafe_speed", &cam_edit->m_strafe_speed, 0.f, 2.f);
        bool left = ImGui::Button( "left" );
        ImGui::SameLine();
        bool right = ImGui::Button( "right" );
        ImGui::SameLine();
        bool front = ImGui::Button( "front" );
        ImGui::SameLine();
        bool back = ImGui::Button( "back" );
        ImGui::SameLine();
        bool up = ImGui::Button( "up" );
        ImGui::SameLine();
        bool down = ImGui::Button( "down" );
        
        uint32 modifiers = 0;
        if( left )
            Controller::GetStaticInstance()->OnInputX( modifiers, -render_ctxt.m_delta_seconds );
        if( right )
            Controller::GetStaticInstance()->OnInputX( modifiers, render_ctxt.m_delta_seconds );
        if( front )
            Controller::GetStaticInstance()->OnInputY( modifiers, render_ctxt.m_delta_seconds );
        if( back )
            Controller::GetStaticInstance()->OnInputY( modifiers, -render_ctxt.m_delta_seconds );
        if( up )
            Controller::GetStaticInstance()->OnInputZ( modifiers, render_ctxt.m_delta_seconds );
        if( down )
            Controller::GetStaticInstance()->OnInputZ( modifiers, -render_ctxt.m_delta_seconds );
    }
	if( ImGui::CollapsingHeader("Path") )
	{
        bool save_path = ImGui::Button( "save" );
        ImGui::SameLine();
        bool load_path = ImGui::Button( "load" );
        if( save_path || load_path )
        {
            bigball::File lvl_path;
            String str_file = String::Printf("../data/level/%s/path.fs", pcopath->m_level_name.ToString().c_str());
            if( lvl_path.Open( str_file.c_str(), save_path ) )
            {
                pcopath->Serialize(lvl_path);
            }
        }
        
        if( ImGui::SliderFloat("knot_dist", &pcoship->m_path_knot_dist_level, 0.f, pcopath->m_sum_knot_distance) )
        {
            pcoship->m_path_knot_dist_level = bigball::clamp( pcoship->m_path_knot_dist_level, 0.f, pcopath->m_sum_knot_distance );
            if( cam_edit )
                m_current_cp_idx = cam_edit->ResetEdit( pcoship->m_path_knot_dist_level );
        }
        ImGui::InputFloat("sum_dist", &pcoship->m_path_dist_level, -1, ImGuiInputTextFlags_ReadOnly);
        
		ImGui::PushStyleVar(ImGuiStyleVar_ChildWindowRounding, 5.0f);
		ImGui::BeginChild("Sub2", ImVec2(0,400), true);
		ImGui::Text("Control points");

		Array<String> str_cp_array;
		for( int cp_idx = 0; cp_idx < pcopath->m_ctrl_points.size(); cp_idx++ )
			str_cp_array.push_back( String::Printf( "%d", cp_idx ) );

        ImGui::PushItemWidth( 50 );
		if( ImGui::ListBox( "", &m_current_cp_idx, GetItemStringArray, &str_cp_array, str_cp_array.size(), 11 ) )
		{
			if( m_current_cp_idx >= 0 && m_current_cp_idx < pcopath->m_ctrl_points.size() )
			{
				pcoship->m_path_knot_dist_level = pcopath->GetSumKnotDistance( m_current_cp_idx );
				if( cam_edit )
					m_current_cp_idx = cam_edit->ResetEdit( pcoship->m_path_knot_dist_level );
			}
		}
        ImGui::PopItemWidth();

		ImGui::SameLine();
        ImGui::BeginChild("Action", ImVec2(0,200), true);
        if( m_current_cp_idx >= 0 && m_current_cp_idx < pcopath->m_ctrl_points.size() )
        {
            bool insert_before = ImGui::Button( "ins. before" );
            ImGui::SameLine();
            bool insert_after = ImGui::Button( "ins. after" );
            if( insert_before || insert_after )
            {
                pcopath->InsertControlPoint( m_current_cp_idx, insert_after );
            }
            
            float delta = cam_edit ? cam_edit->m_edit_slide : 0.f;
            if( pcopath->m_ctrl_points.size() > 1 && (delta > 1e-2f || delta < -1e-2f) )
            {
                ImGui::SameLine();
                if( ImGui::Button( "ins. here" ) )
                {
                    pcopath->InsertControlPoint( pcopath->GetSumKnotDistance( m_current_cp_idx ) + delta );
                }
            }

            // disable if one remaining ctrl point...
            if( pcopath->m_ctrl_points.size() > 1 )
            {
                ImGui::SameLine();
                if( ImGui::Button( "del" ) )
                {
                    pcopath->DeleteControlPoint( m_current_cp_idx );
                    if( cam_edit )
                        m_current_cp_idx = cam_edit->ResetEdit( pcoship->m_path_knot_dist_level );
                }
            }
            if( cam_edit)
            {
				cam_edit->BuildGui();
            }
        }
        ImGui::EndChild();  // Action
		ImGui::EndChild();  // Sub2
		ImGui::PopStyleVar();

	}
    
#endif
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
    
    if (ImGui::CollapsingHeader("Modifications"))
    {
		CoHandle* cohandle = static_cast<CoHandle*>(attractor->GetEntityComponent("CoHandle"));
		int32 num_handles = cohandle->m_handles.size();

		ImGui::PushItemWidth(50);
		Array<String> str_handle_array;
		for (int32 h_idx = 0; h_idx < num_handles; h_idx++)
		{
			str_handle_array.push_back(String::Printf("%d", h_idx));
		}

		if (ImGui::ListBox("Handles", &m_current_handle_idx, GetItemStringArray, &str_handle_array, str_handle_array.size(), 6))
		{
			if (m_current_handle_idx >= 0 && m_current_handle_idx < str_handle_array.size())
			{
				//SAEWorld::GetStaticInstance()->SetCurrentLevel(m_current_lvl_idx);
				//level = SAEWorld::GetStaticInstance()->GetCurrentLevel();
				//pcopath = level ? static_cast<CoPath*>(level->GetEntityComponent(CoPath::StaticClass())) : nullptr;
				//pcoship->SetCurrentLevel(level->GetEntity());
			}
		}
		ImGui::PopItemWidth();

		if (m_current_handle_idx >= 0 && m_current_handle_idx < num_handles)
		{

		}



        int mouse_x, mouse_y;
        SDL_GetMouseState( &mouse_x, &mouse_y );
        float screen_width = (float) g_pEngine->GetDisplayMode().w;
        float screen_height = (float) g_pEngine->GetDisplayMode().h;
        float s_x = (2.0f * mouse_x) / screen_width - 1.0f;
        float s_y = 1.0f - (2.0f * mouse_y) / screen_height;
        vec4 ray_clip = vec4( s_x, s_y, -1.f, 1.f );
        vec4 ray_eye_h = inverse( render_ctxt.m_proj_mat ) * ray_clip;
        vec3 ray_eye = vec3(ray_eye_h.xy, -1.0);
        vec3 ray_world = normalize( render_ctxt.m_view.m_Transform.TransformVector(ray_eye) );

        vec3 cam_pos = render_ctxt.m_view.m_Transform.GetTranslation();
        /*quat cam_rot = render_ctxt.m_view.m_Transform.GetRotation();
        mat3 cam_to_world( cam_rot );
        vec3 cam_front = -cam_to_world.v2.xyz;*/
        
        const float max_ray_dist = 10.f;
        vec3 ray_end = cam_pos + ray_world/*cam_front*/ * max_ray_dist;
        
        RayCastParams params;
        params.m_start  = cam_pos;
        params.m_end    = ray_end;
        params.m_capsule_width = attractor->m_shape_params.fatness_scale;
        
        RayCastResults results;
        if( attractor->RayCast(params, results) )
        {
            CoPosition* copos = static_cast<CoPosition*>(attractor->GetEntityComponent("CoPosition"));
			const float cube_size = copos->GetTransform().GetScale() * attractor->m_shape_params.fatness_scale;
            DrawUtils::GetStaticInstance()->PushAABB(results.m_hit, cube_size, u8vec4(255,0,255,255));
        }
    }

    ImGui::End();
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
