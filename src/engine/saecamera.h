
#ifndef SAECAMERA_H
#define SAECAMERA_H

#include "engine/camera.h"

//////////////////////////////////////////////////////////////////////////

class BIGBALL_API SAECameraCtrl_Editor : public CameraCtrl_Fly
{
	CLASS_EQUIP_H(SAECameraCtrl_Editor, CameraCtrl_Fly)

public:
	SAECameraCtrl_Editor();

	virtual void		UpdateView( CameraView& cam_view, float delta_seconds ) override;
	virtual bool		OnControllerInput(Camera* pcamera, struct ControllerInput const& input) override;
	virtual void		TickMouseState(ControllerMouseState const& mouse_state) override;
	void				SetTarget( Entity* ptarget )		{ m_ptarget = ptarget; }
	void				BuildGui();

public:
	Entity*			m_ptarget;

	float           m_edit_slide;
	vec3			m_interp_pos;
	vec3			m_interp_tan;
	float			m_dist_before;
	float			m_dist_after;
};

#endif // SAECAMERA_H