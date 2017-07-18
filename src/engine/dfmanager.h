


#ifndef DFMANAGER_H
#define DFMANAGER_H

#include "engine/componentmanager.h"

namespace bigball
{
	class BIGBALL_API Component;
	class BIGBALL_API Shader;
};
class CoBlocks;

class DFManager : public ComponentManager 
{
	STATIC_MANAGER_H(DFManager);

	struct DFVertex
	{
		vec2	pos;
		vec2	tex;
	};

private:
	typedef ComponentManager Super;

public:
						DFManager();
	virtual				~DFManager();

	virtual void		Create();
	virtual void		Destroy();	
	virtual void		AddComponentToWorld( Component* pcomponent );
	virtual void		RemoveComponentFromWorld( Component* pcomponent );
	virtual void		Tick( struct TickContext& tick_ctxt );
	virtual void		_Render( struct RenderContext& render_ctxt );
	void				DrawCube();
    void                SetSceneCenter(vec3 center)     { m_scene_center = center; }

protected:

	enum eVAType
	{
		eVAScene = 0,
		eVACube,
		eVACount
	};
	enum eVBType
	{
		eVBScene = 0,        /** Dynamic VB used to render segments */
		eVBCube,
		eVBCubeElt,
		eVBCount
	};

	GLuint			m_varrays[eVACount];
	GLuint			m_vbuffers[eVBCount];
    Shader*         m_bg_shader;
    vec3            m_scene_center;
};


#endif // DFMANAGER_H
