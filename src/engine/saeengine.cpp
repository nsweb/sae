// saeengine.cpp
//

#include "../sae.h"
#include "engine/entity.h"
#include "engine/entitymanager.h"
#include "engine/controller.h"
#include "engine/coposition.h"
#include "ui/uimanager.h"

#include "../engine/dfmanager.h"
#include "../engine/attractormanager.h"
#include "../engine/coattractor.h"
#include "../engine/cohandle.h"
#include "../engine/saecamera.h"
#include "../editor/saeeditor.h"

SAEEngine* SAEEngine::ms_engine = nullptr;

SAEEngine::SAEEngine()
{
	ms_engine = this;
}

SAEEngine::~SAEEngine()
{
	ms_engine = nullptr;
}

bool SAEEngine::Init(EngineInitParams const& init_params)
{
    randinit(__LINE__);
    
	bool bInit = Engine::Init(init_params);
    
    //////////////////////////////////////////////////////////////////////////
	m_editor = new SAEEditor();
	m_editor->Init();


	//////////////////////////////////////////////////////////////////////////
	// Scene description
	// Temp
	Entity* default_attractor = EntityManager::GetStaticInstance()->CreateEntityFromJson( "../data/attractor.json", "Attractor" );
	EntityManager::GetStaticInstance()->AddEntityToWorld(default_attractor);

	Entity* ent_camera = EntityManager::GetStaticInstance()->CreateEntityFromJson( "../data/defaultcamera.json" );
	EntityManager::GetStaticInstance()->AddEntityToWorld(ent_camera);
	if (ent_camera->IsA(Camera::StaticClass()))
	{
		float camera_distance = 3.0;
		Camera* camera = static_cast<Camera*>(ent_camera);
        vec3 cam_pos( -camera_distance, 0.f, camera_distance * 0.5f );
        mat4 look_at = inverse( mat4::lookat( cam_pos, vec3(0.f, 0.f, 0.f), vec3(0.f, 0.f, 1.f) ) );
        quat cam_rot( look_at );
		camera->SetRotation( -cam_rot );
        camera->SetPosition( cam_pos );
	}


	// Link scene objects
	CameraCtrl_Base* cam_ctrl = Controller::GetStaticInstance()->GetCameraCtrl( SAECameraCtrl_Editor::StaticClass() );
	if( cam_ctrl && cam_ctrl->IsA( SAECameraCtrl_Editor::StaticClass() ) )
		((SAECameraCtrl_Editor*)cam_ctrl)->SetTarget( nullptr );

	return bInit;
}

void SAEEngine::Shutdown()
{
	m_editor->Shutdown();
	BB_DELETE( m_editor );

	Engine::Shutdown();
}

void SAEEngine::MainLoop()
{
	Engine::MainLoop();
}

void SAEEngine::ResizeWindow(int w, int h)
{
	Super::ResizeWindow(w, h );
}

void SAEEngine::DeclareComponentsAndEntities()
{
	Super::DeclareComponentsAndEntities();

	DECLARE_COMPONENT_MGR( CoAttractor, AttractorManager );
	DECLARE_COMPONENT(CoHandle);

	DECLARE_ENTITYPATTERN( Attractor, Entity, (3, "CoPosition", "CoAttractor", "CoHandle"), (0) );
}

void SAEEngine::CreateGameCameras()
{
	Controller* controller = Controller::GetStaticInstance();
	controller->RegisterCameraCtrl( new SAECameraCtrl_Editor() );
	controller->SetActiveCameraCtrl( SAECameraCtrl_Editor::StaticClass() );
}

void SAEEngine::InitManagers()
{
	Super::InitManagers();

    DFManager* df_manager = new DFManager();
    df_manager->Create();
    m_managers.push_back( df_manager );
    
    AttractorManager* attractor_manager = new AttractorManager();
    attractor_manager->Create();
    m_managers.push_back( attractor_manager );
}

void SAEEngine::DestroyManagers()
{
	Super::DestroyManagers();
}

void SAEEngine::PreTickManagers( struct TickContext& tick_ctxt )
{
    Super::PreTickManagers(tick_ctxt);
    
    m_editor->Tick(tick_ctxt);
}

bool SAEEngine::RunCommand( String const& cmd_type, Array<String> const& switches, Array<String> const& tokens )
{
#if 0
	if( cmd_type == "builddata" )
	{
		CmdBuildData cmd;
		return cmd.Run( switches, tokens );
	}
#endif

	return Engine::RunCommand( cmd_type, switches, tokens );
}
