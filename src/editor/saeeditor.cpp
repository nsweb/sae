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
#include "../engine/dfmanager.h"


SAEEditor* SAEEditor::ms_editor = nullptr;

SAEEditor::SAEEditor() :
    m_current_attractor_type(INDEX_NONE),
    m_seed_offset(0),
    m_seed_copy(0.f)
{
    ms_editor = this;
	//m_current_file_name.resize(512);
	//m_current_file_name = "";
}

SAEEditor::~SAEEditor()
{
    ms_editor = nullptr;
}

static void UIOnToggleEditorCB( bool bshow_editor )
{

}

static void UIDrawEditorCB( bool* show_editor, bigball::RenderContext& render_ctxt )
{
    SAEEditor::Get()->UIDrawEditor( show_editor, render_ctxt );
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

void SAEEditor::UIDrawEditor( bool* show_editor, RenderContext& render_ctxt )
{
    ImGui::Begin("Editor", show_editor, ImVec2(200,400), -1.f, 0/*ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar*/ );

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
			if (ImGui::MenuItem("Load attractor (OLD)"))
			{
				menu_cmd_type = eMenuCommandType::LoadAttractorOld;
			}
			if (ImGui::MenuItem("Export Obj"))
			{
				menu_cmd_type = eMenuCommandType::ExportObj;
			}
			if (ImGui::MenuItem("Export Ply"))
			{
				menu_cmd_type = eMenuCommandType::ExportPly;
			}
			if (ImGui::MenuItem("Export Pbrt scene file"))
			{
				menu_cmd_type = eMenuCommandType::ExportPbrt;
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

		const char* button_labels[(int)eMenuCommandType::Count] = { "", "Save attractor (.sae)", "Load attractor (.sae)", "Load attractor (.sae)", "Export obj", "Export ply", "Export pbrt scene (.pbrt)" };
		if (ImGui::Button(button_labels[(int)m_current_menu_cmd_type]))
		{
			// Execute command here
			switch (m_current_menu_cmd_type)
			{
				case eMenuCommandType::SaveAttractor:
				case eMenuCommandType::LoadAttractor:
				case eMenuCommandType::LoadAttractorOld:
				{	
					String filename = m_current_file_path + m_current_file_name;
					File file;
					if (file.Open(filename.c_str(), m_current_menu_cmd_type == eMenuCommandType::SaveAttractor ? true : false))
					{
						bool old_format = (m_current_menu_cmd_type == eMenuCommandType::LoadAttractorOld ? true : false);
						AttractorManager::GetStaticInstance()->SerializeAttractor(file, old_format);
					}
					break;
				}
                case eMenuCommandType::ExportObj:
                {
                    String filename = m_current_file_path + m_current_file_name;
                    File file;
                    if (file.Open(filename.c_str(), true))
                    {
                        AttractorManager::GetStaticInstance()->ExportAttractorAsObj(file);
                    }
                    break;
                }
				case eMenuCommandType::ExportPly:
				{
					String filename = m_current_file_path + m_current_file_name;
					File file;
					if (file.Open(filename.c_str(), true))
					{
						AttractorManager::GetStaticInstance()->ExportAttractorAsPly(file);
					}
					break;
				}
				case eMenuCommandType::ExportPbrt:
				{
					AttractorManager::GetStaticInstance()->ExportAttractorAsPbrtScene(m_current_file_path, m_current_file_name);
					break;
				}
                default: break;
			}

			m_current_menu_cmd_type = eMenuCommandType::None;
			ImGui::CloseCurrentPopup();
            
            RefreshListFiles();
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
        AttractorManager::GetStaticInstance()->GetFactory()->GetAttractorTypeList(attractor_names);

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
        if (ImGui::Button("Force rebuild"))
        {
            attractor->RebuildAttractorMesh( true );
        }
        
        AttractorSelection& current_selection = AttractorManager::GetStaticInstance()->GetEditorSelected();
        int32 num_handles = cohandle->m_handles.size();
        
        if (current_selection.m_handle_idx >= 0 && current_selection.m_handle_idx < num_handles)
        {
            AttractorHandle& handle = cohandle->GetHandle(current_selection.m_handle_idx);
            ImGui::PushItemWidth(125);
            float& seed_range = AttractorManager::GetStaticInstance()->m_attractor_seed_range;
            ImGui::InputFloat("Range", &seed_range);
            ImGui::PushItemWidth(250.f);
            ImGui::InputFloat3("Seed", (float*)&handle.m_seed.seed, 6/*, ImGuiInputTextFlags_ReadOnly*/);
            ImGui::PopItemWidth();
            ImGui::SameLine();
            if (ImGui::Button("Random seed"))
            {
                handle.m_seed.seed.x = seed_range * (bigball::randfloat() * 2.f - 1.f);
                handle.m_seed.seed.y = seed_range * (bigball::randfloat() * 2.f - 1.f);
                handle.m_seed.seed.z = seed_range * (bigball::randfloat() * 2.f - 1.f);
            }
            if (ImGui::Button("Copy seed"))
            {
                m_seed_copy = handle.m_seed.seed;
            }
            ImGui::SameLine();
            if (ImGui::Button("Paste seed"))
            {
                handle.m_seed.seed = m_seed_copy;
            }
            if( ImGui::InputInt("Iteration steps", &handle.m_seed.iter, 1, 100) )
                handle.m_seed.iter = max( handle.m_seed.iter, 3 );
            if( ImGui::InputInt("Reverse iter. steps", &handle.m_seed.rev_iter, 1, 100) )
               handle.m_seed.rev_iter = max( handle.m_seed.rev_iter, 0 );
            
            ImGui::SliderInt("Slide seed", &m_seed_offset, -10, 10);
            if(m_seed_offset != 0)
            {
                handle.m_seed.seed = attractor->GuessCurvePos(current_selection.m_handle_idx, m_seed_offset);
            }
            
            float& seed_move = AttractorManager::GetStaticInstance()->m_attractor_seed_move_range;
            ImGui::PushItemWidth(250.f);
            ImGui::SliderFloat("Move speed", &seed_move, 0.f, 2.f, "%.5f", 2.f);
            ImGui::PopItemWidth();
            
            {
                
                bool x_0 = ImGui::Button("+x");
                ImGui::SameLine();
                bool x_1 = ImGui::Button("-x");
                ImGui::SameLine();
                bool y_0 = ImGui::Button("+y");
                ImGui::SameLine();
                bool y_1 = ImGui::Button("-y");
                ImGui::SameLine();
                bool z_0 = ImGui::Button("+z");
                ImGui::SameLine();
                bool z_1 = ImGui::Button("-z");
                if( x_0 || x_1 || y_0 || y_1 || z_0 || z_1)
                {
                    vec3 offset(0.f);
                    offset.x += x_0 ? seed_move : (x_1 ? -seed_move : 0.f);
                    offset.y += y_0 ? seed_move : (y_1 ? -seed_move : 0.f);
                    offset.z += z_0 ? seed_move : (z_1 ? -seed_move : 0.f);
                    handle.m_seed.seed += offset;
                }
            }
            
            ImGui::SliderFloat("Curve alpha", &attractor->m_show_curve_alphas[current_selection.m_handle_idx], 0.f, 1.f, "%.1f");
            
            if (ImGui::InputInt("Merge span", &handle.m_seed.merge_span, 1, 100))
                handle.m_seed.merge_span = max( handle.m_seed.merge_span, -1 );
        }
        
        ImGui::PushItemWidth(50);
        Array<String> str_handle_array;
        for (int32 h_idx = 0; h_idx < num_handles; h_idx++)
        {
            str_handle_array.push_back(String::Printf("%d", h_idx));
        }
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.f), "Handles");
        if (ImGui::ListBox("", &current_selection.m_handle_idx, GetItemStringArray, &str_handle_array, num_handles, 6))
            current_selection.m_point_idx = INDEX_NONE;
        ImGui::PopItemWidth();
        
        ImGui::SameLine();
        ImGui::BeginGroup();
        
        if (current_selection.m_handle_idx >= 0 && current_selection.m_handle_idx < num_handles)
        {
            current_selection.m_attractor = attractor;
            
            AttractorHandle& handle = cohandle->GetHandle(current_selection.m_handle_idx);
            
            bool insert_before = (current_selection.m_handle_idx != 0);
            bool insert_after = true;//handle.m_line_idx < attractor->m_line_framed.points.size() - 2; // (current_selection.m_handle_idx != num_handles - 1 );
            if ( (insert_before && ImGui::Button("Insert before", ImVec2(0,20))) ||
                (!insert_before && ImGui::InvisibleButton("", ImVec2(1,20))))
            {
                cohandle->InsertHandle( current_selection.m_handle_idx );
                attractor->InsertCurve( current_selection.m_handle_idx );
            }
            if ( (insert_after && ImGui::Button("Insert after", ImVec2(0,20))) ||
                (!insert_after && ImGui::InvisibleButton("", ImVec2(1,20))))
            {
                cohandle->InsertHandle( current_selection.m_handle_idx + 1 );
                attractor->InsertCurve( current_selection.m_handle_idx + 1 );
            }
            if (num_handles > 1)
                if (ImGui::Button("Delete"))
                {
                    cohandle->DeleteHandle( current_selection.m_handle_idx );
                    attractor->DeleteCurve( current_selection.m_handle_idx );
                }
            
            attractor->m_preview_idx = current_selection.m_handle_idx;
        }
        ImGui::EndGroup();
        
        if (current_selection.m_handle_idx != INDEX_NONE && current_selection.m_point_idx != INDEX_NONE)
        {
            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(255,0,0,100));
            ImGui::SameLine();
            ImGui::BeginChildFrame(234, ImVec2(200,80), ImGuiWindowFlags_ShowBorders);
            //ImGui::LabelText("Point idx", "%d", current_selection.m_point_idx);
            if( ImGui::InputInt("Point idx", &current_selection.m_point_idx, 1, 100) )
            {
                AttractorOrientedCurve const& curve = attractor->m_curves[current_selection.m_handle_idx];
                current_selection.m_point_idx = clamp(current_selection.m_point_idx, 1, curve.points.size() - 1);
            }
            
            static int cut_length = 0;
            if( ImGui::InputInt("Cut length", &cut_length, 1, 100) )
                cut_length = max( cut_length, 0 );
            if (ImGui::Button("Cut line"))
            {
                AttractorOrientedCurve const& old_curve = attractor->m_curves[current_selection.m_handle_idx];
                AttractorHandle& old_handle = cohandle->m_handles[current_selection.m_handle_idx];
                const int32 num_point = old_curve.points.size();
                const int32 old_iter = old_handle.m_seed.iter;
                const int32 old_rev_iter = old_handle.m_seed.rev_iter;
                const int32 pt_idx = current_selection.m_point_idx;
                
                if (pt_idx < old_rev_iter)
                {
                    old_handle.m_seed.seed = old_curve.points[pt_idx];
                    old_handle.m_seed.iter = 0;
                    old_handle.m_seed.rev_iter = pt_idx;
                }
                else
                {
                    old_handle.m_seed.iter = pt_idx - old_rev_iter;
                }
                
                if (cut_length > 0)
                {
                    if (pt_idx + cut_length < num_point)
                    {
                        // cuts at point idx, removes cut_length points after that
                        cohandle->InsertHandle( current_selection.m_handle_idx + 1 );
                        attractor->InsertCurve( current_selection.m_handle_idx + 1 );
                        
                        AttractorOrientedCurve const& prev_curve = attractor->m_curves[current_selection.m_handle_idx];
                        AttractorHandle& new_handle = cohandle->m_handles[current_selection.m_handle_idx + 1];
                        new_handle.m_seed.seed = prev_curve.points[pt_idx + cut_length];
                        new_handle.m_seed.iter = num_point - (pt_idx + cut_length);
                        new_handle.m_seed.rev_iter = 0;
                    }
                }
                // reset
                current_selection.m_point_idx = INDEX_NONE;
            }
            
            //static bool show_demo = false;
            //if (ImGui::Button("Test Window"))
            //    show_demo = true;
            //ImGui::ShowTestWindow(&show_demo);

            ImGui::EndChildFrame();
            ImGui::PopStyleColor();
        }
        
        //static float value = 0.5f;
        //if (ImGui::BeginPopupContextItem("item context menu"))
        //{
        //	if (ImGui::Selectable("Set to zero")) value = 0.0f;
        //	if (ImGui::Selectable("Set to PI")) value = 3.1415f;
        //	ImGui::EndPopup();
        //}
        
        ImGui::Separator();
        
        //ImGui::InputFloat3("Attractor seed", (float*)&attractor->m_line_params.seed);
        //ImGui::Checkbox("Show seed", &AttractorManager::GetStaticInstance()->m_show_seeds);
        //ImGui::SameLine();

		ImGui::PushItemWidth(125);
		ImGui::InputFloat("Step size", &attractor->m_line_params.step_factor);
		ImGui::InputInt("Simplify step", &attractor->m_shape_params.simplify_level, 1, 10);
		ImGui::InputFloat("Merge dist", &attractor->m_shape_params.merge_dist);
        ImGui::InputInt("Merge span", &attractor->m_shape_params.merge_span, 1, 10000);

		ImGui::Separator();
        
        ImGui::InputFloat("Line fatness", &attractor->m_shape_params.fatness_scale);
		ImGui::InputInt("Edge count", &attractor->m_shape_params.local_edge_count, 1, 10);
		ImGui::InputFloat("Crease depth", &attractor->m_shape_params.crease_depth);
		ImGui::InputFloat("Crease width", &attractor->m_shape_params.crease_width);
		ImGui::InputFloat("crease bevel", &attractor->m_shape_params.crease_bevel);

		ImGui::Separator();

		//ImGui::Checkbox("Freeze bbox", &attractor->m_shape_params.freeze_bbox);
        //ImGui::Checkbox("Show bary", &attractor->m_shape_params.show_bary);
        //ImGui::InputInt("Max iteration count", &attractor->m_shape_params.max_iter_count, 1, 10);
        ImGui::InputFloat("Target dimension", &attractor->m_line_params.target_dim);
		ImGui::SliderAngle("Shearing angle", &attractor->m_line_params.shearing_angle, -90.f, 90.f);
		ImGui::InputFloat("Shearing scale x", &attractor->m_line_params.shearing_scale_x, 0.01f, 0.05f, 3);
		ImGui::InputFloat("Shearing scale y", &attractor->m_line_params.shearing_scale_y, 0.01f, 0.05f, 3);

		ImGui::PopItemWidth();

		ImGui::Separator();
    }

	if (ImGui::CollapsingHeader("Stats"))
	{
		int32 tri_count = attractor->m_tri_indices.size() / 3;
		ImGui::InputInt("Triangle count", &tri_count, -1, ImGuiInputTextFlags_ReadOnly);
        vec3 dimensions = attractor->m_max_box - attractor->m_min_box;
        ImGui::InputFloat3("Dimensions", (float*)&dimensions, 2, ImGuiInputTextFlags_ReadOnly);
	}
    
	//bool show_modifications = ImGui::CollapsingHeader("Modifications");
	//AttractorManager::GetStaticInstance()->SetShowHandles(show_modifications);

    
    if (ImGui::CollapsingHeader("Display"))
    {
        ImGui::Checkbox("show lines", &AttractorManager::GetStaticInstance()->m_show_lines);
        ImGui::Checkbox("show meshes", &AttractorManager::GetStaticInstance()->m_show_meshes);
        ImGui::Checkbox("show handles", &AttractorManager::GetStaticInstance()->m_show_handles);
    }
    
    ImGui::End();
}

void SAEEditor::Tick( struct TickContext& tick_ctxt )
{
    vec3 scene_center(0.f, 0.f, 0.f);
    Array<CoAttractor*> const& attractors = AttractorManager::GetStaticInstance()->GetAttractors();
    int num_attractor = attractors.size();
    for( int att_idx = 0; att_idx < num_attractor; att_idx++)
    {
        CoPosition* copos = static_cast<CoPosition*>(attractors[att_idx]->GetEntityComponent("CoPosition"));
        scene_center.z = min( scene_center.z, attractors[att_idx]->m_min_box.z * copos->GetScale() );
    }
    DFManager::GetStaticInstance()->SetSceneCenter(scene_center);
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
