
#include "../sae.h"
#include "saecamera.h"
#include "engine/coposition.h"
#include "engine/controller.h"


CLASS_EQUIP_CPP(SAECameraCtrl_Editor);


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
SAECameraCtrl_Editor::SAECameraCtrl_Editor() :
		//m_strafe_speed(0.1f),
		//m_rotation_speed(0.1f),
		m_ptarget(nullptr),
		m_current_cp_edit(0),
		m_edit_slide(0.f)
{
    m_StrafeSpeed = 1.f;
    m_RotationSpeed = 50.f;
}


void SAECameraCtrl_Editor::UpdateView( CameraView& cam_view, float delta_seconds )
{
	Super::UpdateView( cam_view, delta_seconds );
}

bool SAECameraCtrl_Editor::OnControllerInput( Camera* pcamera, ControllerInput const& input )
{
    CameraView& cam_view = pcamera->GetView();
    quat cam_rot = cam_view.m_Transform.GetRotation();
    vec3 cam_pos = cam_view.m_Transform.GetTranslation();
    mat3 cam_to_world( cam_rot );
    vec3 right = cam_to_world.v0.xyz;
    vec3 up = cam_to_world.v1.xyz;
    vec3 front = -cam_to_world.v2.xyz;
    
    // Compute target distance
    // d = max_target_dist
    const float max_target_dist = 3.f;
    float target_dist = max_target_dist;
    
    // d = min( d, ray_ground_intersection )
    if( front.z < -1.e-4 )
    {
        target_dist = bigball::min( target_dist, -cam_pos.z / front.z );
    }
    
    // d = min( d, object_intersection )
    // TODO
    
    if( input.m_type == eCIT_Key )
    {
        cam_pos += (right * input.m_delta.x + up * input.m_delta.z + front * input.m_delta.y) * m_StrafeSpeed;
    }
	else if (input.m_type == eCIT_KeyCtrl || input.m_type == eCIT_KeyAlt)
    {
        vec3 target_pos = cam_pos + front * target_dist;

        quat Yaw( quat::fromeuler_xyz( 0.f, 0.f, input.m_delta.x * m_RotationSpeed ) );
        quat Pitch( quat::fromeuler_xyz( input.m_delta.y * m_RotationSpeed, 0.f, 0.f ) );
        cam_rot = Yaw * cam_rot * Pitch;
        
        mat3 new_cam_to_world( cam_rot );
        front = -new_cam_to_world.v2.xyz;
        cam_pos = target_pos - front * target_dist;
    }
    
    cam_view.m_Transform.SetRotation( cam_rot );
    cam_view.m_Transform.SetTranslation( cam_pos );
    
	//return Super::OnControllerInput( pcamera, input );
    return true;
}


void SAECameraCtrl_Editor::BuildGui()
{

}
