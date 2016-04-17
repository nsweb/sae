// sae.cpp : Defines the entry point for the console application.
//

#include "sae.h"
#include "system/profiler.h"
#include "SDL_main.h"


#ifdef __cplusplus
extern "C"
#endif
int main(int argc, char *argv[])
{
	PROFILE_THREAD_SCOPE("MainThread");

	g_pEngine = new SAEEngine();
	CommandLine& CmdLine = g_pEngine->GetCommandLine();
	CmdLine.Parse( argc, argv );

	EngineInitParams init_params;
	init_params.resizable_window = true;
	init_params.mouse_capture = false;
	init_params.default_res_x = 1280;
	init_params.default_res_y = 720;
	g_pEngine->Init(init_params);

	String CmdType;
	if( CmdLine.IsCommand( CmdType ) )
	{
		g_pEngine->RunCommand( CmdType, CmdLine.switches, CmdLine.tokens );
	}
	else
	{
		g_pEngine->MainLoop();
	}

	g_pEngine->Shutdown();
	delete g_pEngine;
	g_pEngine = nullptr;

	return 0;
}
