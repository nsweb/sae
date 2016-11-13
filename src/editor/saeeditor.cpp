// saeeditor.cpp
//

#include "../sae.h"
#include "saeeditor.h"
#include "engine/controller.h"
#include "engine/coposition.h"
#include "ui/uimanager.h"
#include "gfx/rendercontext.h"
#include "gfx/drawutils.h"
#include "system/file.h"
#include "../engine/attractormanager.h"
#include "../engine/coattractor.h"
#include "../engine/cohandle.h"
#include "../engine/saecamera.h"


SAEEditor* SAEEditor::ms_peditor = nullptr;

SAEEditor::SAEEditor() :
    m_current_attractor_type(INDEX_NONE)
{
    ms_peditor = this;
	//m_current_file_name.resize(512);
	//m_current_file_name = "";
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
    /*CameraCtrl_Base* cam_ctrl = Controller::GetStaticInstance()->GetActiveCameraCtrl();
    SAECameraCtrl_Editor* cam_edit = nullptr;
    if( cam_ctrl && cam_ctrl->IsA( SAECameraCtrl_Editor::StaticClass() ) )
        cam_edit = static_cast<SAECameraCtrl_Editor*>( cam_ctrl );
    */
    ImGui::Begin("Editor", bshow_editor, ImVec2(200,400), -1.f, 0/*ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar*/ );

	ImGui::ShowTestWindow();

    ImGui::End();
}

void SAEEditor::UIDrawEditorMenus(RenderContext& render_ctxt)
{
	eMenuCommandType menu_cmd_type = eMenuCommandType::None;
	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("File"))
		{
			if (ImGui::MenuItem("New attractor"))
			{
				menu_cmd_type = eMenuCommandType::NewAttractor;
			}
			if (ImGui::MenuItem("Save attractor"))
			{
				menu_cmd_type = eMenuCommandType::SaveAttractor;
			}
			if (ImGui::MenuItem("Load attractor"))
			{
				menu_cmd_type = eMenuCommandType::LoadAttractor;
			}
			if (ImGui::MenuItem("Export Obj"))
			{
				menu_cmd_type = eMenuCommandType::ExportObj;
			}
			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();
	}

	if (menu_cmd_type != eMenuCommandType::None)
	{
		m_current_menu_cmd_type = menu_cmd_type;
		ImGui::OpenPopup("File dialog");
	}

	if (ImGui::BeginPopupModal("File dialog"))
	{
		DrawFileDialog(m_current_menu_cmd_type);

		const char* button_labels[(int)eMenuCommandType::Count] = { "", "Save attractor (.sae)", "Load attractor (.sae)", "Export obj" };
		if (ImGui::Button(button_labels[(int)m_current_menu_cmd_type]))
		{
			// Execute command here
			switch (m_current_menu_cmd_type)
			{
				case eMenuCommandType::SaveAttractor:
				case eMenuCommandType::LoadAttractor:
				{	
					String filename = m_current_file_path + m_current_file_name;
					File file;
					if (file.Open(filename.c_str(), m_current_menu_cmd_type == eMenuCommandType::SaveAttractor ? true : false))
					{
						AttractorManager::GetStaticInstance()->SerializeAttractor(file);
					}
					break;
				}
                default: break;
			}

			m_current_menu_cmd_type = eMenuCommandType::None;
			ImGui::CloseCurrentPopup();
		}
		ImGui::SameLine();
		if (ImGui::Button("Cancel"))
		{
			m_current_menu_cmd_type = eMenuCommandType::None;
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}

    DrawRightPanel(render_ctxt);
}

void SAEEditor::DrawFileDialog(eMenuCommandType cmd_type)
{
	if (m_current_file_path.Len() == 0)
	{
		CommandLine const& cmd_line = g_pEngine->GetCommandLine();

		int slash_idx = cmd_line.cmd_exec.LastIndexOf('\\');
		if (slash_idx == INDEX_NONE)
			slash_idx = cmd_line.cmd_exec.LastIndexOf('/');
		if (slash_idx != INDEX_NONE)
			m_current_file_path = cmd_line.cmd_exec.Sub(0, slash_idx + 1);
		else
			m_current_file_path = cmd_line.cmd_exec;
        
		RefreshListFiles();
	}

	ImGui::Text(m_current_file_path.c_str());

	ImGui::PushItemWidth(350);
    int old_sel = m_current_file_selection;
	if (ImGui::ListBox("", &m_current_file_selection, GetItemStringArray, &m_current_file_array, m_current_file_array.size(), 20))
    {
		if (m_current_file_selection >= 0 && m_current_file_selection < m_current_file_array.size())
		{
#if _WIN32 || _WIN64
			String sep = "\\";
#else
			String sep = "/";
#endif
			if (m_current_file_array[m_current_file_selection].EndsWith(sep))
			{
				if (old_sel == m_current_file_selection)
				{
					// directory
					m_current_file_path += m_current_file_array[m_current_file_selection];
					RefreshListFiles();
					m_current_file_selection = INDEX_NONE;
				}
			}
			else
			{
				// file
				m_current_file_name = m_current_file_array[m_current_file_selection];
			}
		}
    }
	ImGui::PopItemWidth();

	char buffer[512];
	strcpy(buffer, m_current_file_name.c_str());
	if (ImGui::InputText("file", buffer, sizeof(buffer)))
	{
		m_current_file_name = String::Printf("%s", buffer);
	}
}

void SAEEditor::RefreshListFiles()
{
	FileUtils::NormalizePath(m_current_file_path);

	m_current_file_array.clear();

#if _WIN32 || _WIN64
	String file_filter = "*.*";
	String sep = "\\";
#else
	String file_filter = "";
	String sep = "/";
#endif

	FileUtils::ListFiles((m_current_file_path + file_filter).c_str(), m_current_file_array);

	m_current_file_array.remove(".");
	String tmp_str;
	for (int file_idx = 0; file_idx < m_current_file_array.size(); file_idx++)
	{
		tmp_str = m_current_file_path + m_current_file_array[file_idx];
		if (FileUtils::IsDirectory(tmp_str.c_str()))
			m_current_file_array[file_idx] += sep;
	}
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
        if (ImGui::Button("Random seed"))
        {
            attractor->m_line_params.seed.x = bigball::randfloat() * 2.f - 1.f;
            attractor->m_line_params.seed.y = bigball::randfloat() * 2.f - 1.f;
            attractor->m_line_params.seed.z = bigball::randfloat() * 2.f - 1.f;
            attractor->RebuildAttractorMesh();
        }

		ImGui::PushItemWidth(125);
		ImGui::InputInt("Iteration steps", &attractor->m_line_params.iter, 1, 100);// , ImGuiInputTextFlags extra_flags = 0);
		ImGui::InputInt("Reverse iter. steps", &attractor->m_line_params.rev_iter, 1, 100);
		ImGui::InputInt("Warmup iter. steps", &attractor->m_line_params.warmup_iter, 1, 100);
		ImGui::InputFloat("Step size", &attractor->m_line_params.step_factor);
		ImGui::InputInt("Simplify step", &attractor->m_shape_params.simplify_level, 1, 10);
		ImGui::InputFloat("Merge dist", &attractor->m_shape_params.merge_dist);
		ImGui::Checkbox("Snap interp", &attractor->m_shape_params.snap_interp);
		ImGui::Checkbox("Remove line ends", &attractor->m_shape_params.remove_line_ends);

		ImGui::Separator();
        
        ImGui::InputFloat("Line fatness", &attractor->m_shape_params.fatness_scale);
		ImGui::InputInt("Edge count", &attractor->m_shape_params.local_edge_count, 1, 10);
		ImGui::InputFloat("Crease depth", &attractor->m_shape_params.crease_depth);
		ImGui::InputFloat("Crease width", &attractor->m_shape_params.crease_width);
		ImGui::InputFloat("crease bevel", &attractor->m_shape_params.crease_bevel);
        ImGui::InputFloat("max drift", &attractor->m_shape_params.max_drift);
        ImGui::InputInt("target bary offset", &attractor->m_shape_params.target_bary_offset, 1, 20);

		ImGui::Separator();

		//ImGui::Checkbox("Freeze bbox", &attractor->m_shape_params.freeze_bbox);
        ImGui::Checkbox("Show bary", &attractor->m_shape_params.show_bary);
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
		if (ImGui::Button("Rebuild"))
		{
			attractor->RebuildAttractorMesh();
		}
		ImGui::InputInt("View range", &attractor->m_view_handle_range, 1, 100000);
		
		int32 num_handles = cohandle->m_handles.size();

		ImGui::PushItemWidth(50);
		Array<String> str_handle_array;
		for (int32 h_idx = 0; h_idx < num_handles; h_idx++)
		{
			str_handle_array.push_back(String::Printf("%d", h_idx));
		}

        AttractorSelection& current_selection = AttractorManager::GetStaticInstance()->GetEditorSelected();
		ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.f), "Handles");
        ImGui::ListBox("", &current_selection.m_handle_idx, GetItemStringArray, &str_handle_array, num_handles, 6);
		ImGui::PopItemWidth();

		ImGui::SameLine();
		ImGui::BeginGroup();
        
        if (current_selection.m_handle_idx >= 0 && current_selection.m_handle_idx < num_handles)
        {
            current_selection.m_attractor = attractor;
            
            AttractorHandle& handle = cohandle->GetHandle(current_selection.m_handle_idx);

            bool insert_before = (current_selection.m_handle_idx != 0);
			bool insert_after = handle.m_line_idx < attractor->m_line_framed.points.size() - 2; // (current_selection.m_handle_idx != num_handles - 1 );
            if ( (insert_before && ImGui::Button("Insert before", ImVec2(0,20))) ||
                 (!insert_before && ImGui::InvisibleButton("", ImVec2(1,20))))
            {
                cohandle->InsertHandle( current_selection.m_handle_idx );
            }
            if ( (insert_after && ImGui::Button("Insert after", ImVec2(0,20))) ||
                 (!insert_after && ImGui::InvisibleButton("", ImVec2(1,20))))
            {
                cohandle->InsertHandle( current_selection.m_handle_idx + 1 );
            }
            if (num_handles > 2)
            if (ImGui::Button("Delete"))
            {
                cohandle->DeleteHandle( current_selection.m_handle_idx );
            }
        }
		ImGui::EndGroup();
        
        if (current_selection.m_handle_idx >= 0 && current_selection.m_handle_idx < num_handles)
        {
            ImGui::SameLine();
            ImGui::BeginGroup();
            ImGui::PushItemWidth(140);
            AttractorHandle& handle = cohandle->GetHandle(current_selection.m_handle_idx);
            ImGui::InputInt("line idx", &handle.m_line_idx);
            ImGui::InputInt("mesh idx", &handle.m_mesh_idx);
            
			handle.m_line_idx = clamp(handle.m_line_idx, 1, attractor->m_line_framed.points.size() - 2);
			handle.m_mesh_idx = clamp(handle.m_mesh_idx, 1, attractor->m_line_framed.points.size() - 2);
            
            const char* srt_type_array[AttractorHandle::eHT_Count] = {"Move", "Cut"};
            ImGui::Combo("", (int32*)&handle.m_type, srt_type_array, AttractorHandle::eHT_Count);
            
            ImGui::PopItemWidth();
            ImGui::EndGroup();
        }

		//static float value = 0.5f;
		//if (ImGui::BeginPopupContextItem("item context menu"))
		//{
		//	if (ImGui::Selectable("Set to zero")) value = 0.0f;
		//	if (ImGui::Selectable("Set to PI")) value = 3.1415f;
		//	ImGui::EndPopup();
		//}

    }
    
    if (ImGui::CollapsingHeader("Display"))
    {
        ImGui::Checkbox("show lines", &AttractorManager::GetStaticInstance()->m_show_lines);
    }
    
    ImGui::End();
}

void SAEEditor::HandleScenePick(ControllerMouseState const& mouse_state)
{
	AttractorManager::GetStaticInstance()->HandleScenePick(mouse_state);
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
