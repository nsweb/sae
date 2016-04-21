
// saeeditor.h


#ifndef SAEEDITOR_H
#define SAEEDITOR_H

#include "engine/engine.h"

namespace bigball
{
    struct BIGBALL_API RenderContext;
};

class SAEEditor
{

public:
                        SAEEditor();
	virtual             ~SAEEditor();

	virtual bool        Init();
	virtual void        Shutdown();
    
    static SAEEditor*	Get()		{ return ms_peditor;	}
    
    void                UIDrawEditor( bool* bshow_editor, bigball::RenderContext& render_ctxt );
	void				UIDrawEditorMenus(bigball::RenderContext& render_ctxt);
    
    int                 m_current_attractor_type;
	//int                 m_current_handle_idx;
    
private:
    static SAEEditor*    ms_peditor;

	static bool			GetItemStringArray( void* data, int idx, const char** out_text );
    
    void                DrawRightPanel(bigball::RenderContext& render_ctxt);

};


#endif  // SAEEDITOR_H