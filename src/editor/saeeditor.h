
// saeeditor.h


#ifndef SAEEDITOR_H
#define SAEEDITOR_H

#include "engine/engine.h"

namespace bigball
{
    struct BIGBALL_API RenderContext;
	struct ControllerMouseState;
};

enum class eMenuCommandType
{
	None = -1,
	NewAttractor = 0,
	SaveAttractor,
	LoadAttractor,
	LoadAttractorOld,
	ExportObj,
	ExportPly,
	ExportPbrt,
	Count,
};

class SAEEditor
{

public:
                        SAEEditor();
	virtual             ~SAEEditor();

	virtual bool        Init();
	virtual void        Shutdown();
    
    static SAEEditor*	Get()		{ return ms_editor;	}
    
    void                UIDrawEditor( bool* bshow_editor, bigball::RenderContext& render_ctxt );
	void				UIDrawEditorMenus(bigball::RenderContext& render_ctxt);
	void				HandleScenePick(ControllerMouseState const& mouse_state);
    void                Tick( struct TickContext& tick_ctxt );
    
    int                 m_current_attractor_type;
	String				m_current_file_path;
	Array<String>		m_current_file_array;
	String				m_current_file_name;
	int					m_current_file_selection;
	eMenuCommandType	m_current_menu_cmd_type;
    
    int                 m_seed_offset;
    vec3                m_seed_copy;
    
private:
    static SAEEditor*    ms_editor;

	static bool			GetItemStringArray( void* data, int idx, const char** out_text );
    
    void                DrawRightPanel(bigball::RenderContext& render_ctxt);
	void				DrawFileDialog(eMenuCommandType cmd_type);
	void				RefreshListFiles();

};


#endif  // SAEEDITOR_H
