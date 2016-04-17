
// saeengine.h


#ifndef SAEENGINE_H
#define SAEENGINE_H

#include "engine/engine.h"

class SAEEngine : public Engine
{
private:
	typedef Engine Super;

public:
						SAEEngine();
	virtual				~SAEEngine();

	virtual bool		Init(EngineInitParams const& init_params) override;
	virtual void		Shutdown() override;
	virtual void		MainLoop() override;
	virtual bool		RunCommand( String const& cmd_type, Array<String> const& switches, Array<String> const& tokens ) override;

	static SAEEngine*	Get()		{ return ms_pengine;	}

protected:
	virtual void		DeclareComponentsAndEntities() override;
	virtual void		CreateGameCameras() override;
	virtual void		InitManagers() override;
	virtual void		DestroyManagers() override;
	virtual void		ResizeWindow(int w, int h) override;

	static SAEEngine*	ms_pengine;
	class SAEEditor*	m_peditor;
};


#endif  // SAEENGINE_H