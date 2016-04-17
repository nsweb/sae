

#include "../sae.h"
#include "dfmanager.h"
#include "engine/controller.h"
#include "engine/camera.h"
#include "gfx/gfxmanager.h"
#include "gfx/shader.h"
#include "gfx/rendercontext.h"
#include "system/profiler.h"



STATIC_MANAGER_CPP(DFManager);

DFManager::DFManager()
{
	m_pStaticInstance = this;
}

DFManager::~DFManager()
{
	m_pStaticInstance = nullptr;
}

void DFManager::Create()
{
	m_bg_shader = GfxManager::GetStaticInstance()->LoadShader( "bg_editor" );

	// SCENE
	DFVertex screen_vertices[] = { { vec2(-1.f,-1.f), vec2(-1.f,-1.f) }, { vec2(1.f,1.f), vec2(1.f,1.f) }, { vec2(-1.f,1.f), vec2(-1.f,1.f) },
								  { vec2(-1.f,-1.f), vec2(-1.f,-1.f) },	{ vec2(1.f,-1.f), vec2(1.f,-1.f) }, { vec2(1.f,1.f), vec2(1.f,1.f) } };

	glGenVertexArrays( eVACount, m_varrays );
	glGenBuffers( eVBCount, m_vbuffers );

	glBindVertexArray( m_varrays[eVAScene]);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbuffers[eVAScene]);
	glBufferData( GL_ARRAY_BUFFER, sizeof(screen_vertices), screen_vertices, GL_STATIC_DRAW );

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	glVertexAttribPointer( 0, 2, GL_FLOAT, GL_FALSE, sizeof(DFVertex) /*stride*/, (void*)0 /*offset*/	);
	glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, sizeof(DFVertex) /*stride*/, (void*)8 /*offset*/	);

	glBindVertexArray(0);
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	// CUBE
	const vec3 cube_data[] = { vec3(-1.f, -1.f, -1.f), vec3(-1.f, 1.f, -1.f), vec3(1.f, 1.f, -1.f), vec3(1.f, -1.f, -1.f),
								vec3(-1.f, -1.f, 1.f), vec3(-1.f, 1.f, 1.f), vec3(1.f, 1.f, 1.f), vec3(1.f, -1.f, 1.f) };

	GLuint idx_data[] = {
		0,1,2, 0,2,3,
		1,0,4, 1,4,5,
		2,1,5, 2,5,6,
		3,2,6, 3,6,7,
		0,3,7, 0,7,4,
		5,4,7, 5,7,6
	};

	glBindVertexArray( m_varrays[eVACube] );
	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, m_vbuffers[eVBCubeElt] );
	glBufferData( GL_ELEMENT_ARRAY_BUFFER, COUNT_OF(idx_data) * sizeof(GLuint), idx_data, GL_STATIC_DRAW );

	glBindBuffer( GL_ARRAY_BUFFER, m_vbuffers[eVBCube] );
	glBufferData( GL_ARRAY_BUFFER, COUNT_OF(cube_data) * sizeof(vec3), cube_data, GL_STATIC_DRAW );
	glEnableVertexAttribArray(0);
	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3) /*stride*/, (void*)0 /*offset*/ );

	glBindVertexArray(0);
	glDisableVertexAttribArray(0);
}

void DFManager::Destroy()
{
	m_bg_shader = nullptr;

	glDeleteBuffers(eVBCount, m_vbuffers);
	glDeleteVertexArrays(eVACount, m_varrays);
}


void DFManager::AddComponentToWorld( Component* pComponent )
{

}

void DFManager::RemoveComponentFromWorld( Component* pComponent )
{

}

void DFManager::Tick( TickContext& tick_ctxt )
{
	PROFILE_SCOPE( __FUNCTION__ );


}

void DFManager::_Render( RenderContext& render_ctxt )
{
	PROFILE_SCOPE( __FUNCTION__ );
	//return;
	static float global_time = 0.f;
	global_time += render_ctxt.m_delta_seconds;
    
    vec3 cam_pos = render_ctxt.m_view.m_Transform.GetTranslation();
    mat4 view_inv_mat( render_ctxt.m_view.m_Transform.GetRotation(), render_ctxt.m_view.m_Transform.GetTranslation(), (float)render_ctxt.m_view.m_Transform.GetScale() );
	
	const float fov_y = render_ctxt.m_view.m_fParameters[eCP_FOV] * (F_PI / 180.0f);
	const float z_near = render_ctxt.m_view.m_fParameters[eCP_NEAR];
	const float z_far = render_ctxt.m_view.m_fParameters[eCP_FAR];

	vec2 screen_res;
	screen_res.y = bigball::tan( fov_y * 0.5f );
	screen_res.x = screen_res.y * render_ctxt.m_view.m_fParameters[eCP_ASPECT];

	vec2 z_var;	
	z_var.x = (z_far + z_near) / (z_far - z_near);
	z_var.y = 2.0f*z_far*z_near / (z_far - z_near);
    
	m_bg_shader->Bind();
	ShaderUniform uni_gtime = m_bg_shader->GetUniformLocation("global_time");
	m_bg_shader->SetUniform( uni_gtime, global_time );
    ShaderUniform uni_camera = m_bg_shader->GetUniformLocation("camera_pos");
    m_bg_shader->SetUniform( uni_camera, cam_pos );
    ShaderUniform uni_view_inv = m_bg_shader->GetUniformLocation("viewinv_mat");
    m_bg_shader->SetUniform( uni_view_inv, view_inv_mat );
    ShaderUniform uni_proj = m_bg_shader->GetUniformLocation("proj_mat");
    m_bg_shader->SetUniform( uni_proj, render_ctxt.m_proj_mat );

	ShaderUniform uni_sr = m_bg_shader->GetUniformLocation("screen_res");
	m_bg_shader->SetUniform( uni_sr, screen_res );
	ShaderUniform uni_zvar = m_bg_shader->GetUniformLocation("z_var");
	m_bg_shader->SetUniform( uni_zvar, z_var );

	//level->InterpAndSetUniforms( ship_time );

	glBindVertexArray( m_varrays[eVAScene] );

	glDrawArrays( GL_TRIANGLES, 0, 8 );

	glBindVertexArray(0);

	m_bg_shader->Unbind();
    
}

void DFManager::DrawCube()
{
	glBindVertexArray( m_varrays[eVACube] );

	glDrawElements(GL_TRIANGLES, (GLsizei)36, GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);
}

