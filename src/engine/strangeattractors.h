
#ifndef SAESTRANGEATTRACTORS_H
#define SAESTRANGEATTRACTORS_H

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
enum eAttractorType
{
    eAttractor_None = INDEX_NONE,
	eAttractor_Lorentz,
	eAttractor_Aizawa,
	eAttractor_TSUCS2,
	eAttractor_Arneodo,
	eAttractor_ChenLee,
	eAttractor_DequanLi,
	eAttractor_LorentzMod2,
	eAttractor_MAX
};

class StrangeAttractor
{
public:
	eAttractorType	m_type;
	vec3			m_init_point;
	int32			m_max_iter;
	float			m_fatness_scale;
	float			m_dt;
	float			m_adaptative_dist;

	StrangeAttractor() : m_dt(0.01f) {}
    virtual ~StrangeAttractor() {}
	virtual void Loop( Array<vec3>& out_pos ) const
	{
		vec3 P(m_init_point), DP;

		for( int32 i=0; i<m_max_iter; i++ ) 
		{
			GetDerivatives( P, DP );
			P += DP * m_dt;
			out_pos.push_back( P );
		}
	}
	virtual void LoopAdaptative(Array<vec3>& out_pos) const
	{
		vec3 P( m_init_point ), DP;
		float DRatio, Len;
		float AdDt = bigball::abs(m_adaptative_dist / m_dt);

		for( int32 i=0; i<m_max_iter; i++ ) 
		{
			
			GetDerivatives( P, DP );
			Len = length(DP);
			DRatio = AdDt / Len;

			P += DP * (DRatio * m_dt);
			out_pos.push_back( P );
		}
	}
	virtual void LoopCurl(Array<vec3>& out_pos) const
	{
		const float Eps = m_dt*0.1f;
		vec3 P( m_init_point ), DPx0, DPx1, DPy0, DPy1, DPz0, DPz1, PCurl;

		for( int32 i=0; i<m_max_iter; i++ ) 
		{
			GetDerivatives( vec3( P.x - Eps, P.y, P.z ), DPx0 );
			GetDerivatives( vec3( P.x + Eps, P.y, P.z ), DPx1 );
			GetDerivatives( vec3( P.x, P.y - Eps, P.z ), DPy0 );
			GetDerivatives( vec3( P.x, P.y + Eps, P.z ), DPy1 );
			GetDerivatives( vec3( P.x, P.y, P.z - Eps ), DPz0 );
			GetDerivatives( vec3( P.x, P.y, P.z + Eps ), DPz1 );

			PCurl.x = ( (DPy1.z - DPy0.z) - (DPz1.y - DPz0.y) ) / (2*Eps);
			PCurl.y = ( (DPz1.x - DPz0.x) - (DPx1.z - DPx0.z) ) / (2*Eps);
			PCurl.z = ( (DPx1.y - DPx0.y) - (DPy1.x - DPy0.x) ) / (2*Eps);

			P += PCurl * m_dt;
			out_pos.push_back( P );
		}
	}
	virtual void LoopGradient(Array<vec3>& out_pos) const
	{
		const float Eps = m_dt*0.1f;
		vec3 P( m_init_point ), DPx0, DPx1, DPy0, DPy1, DPz0, DPz1, PN;

		for( int32 i=0; i<m_max_iter; i++ ) 
		{
			GetDerivatives( vec3( P.x - Eps, P.y, P.z ), DPx0 );
			GetDerivatives( vec3( P.x + Eps, P.y, P.z ), DPx1 );
			GetDerivatives( vec3( P.x, P.y - Eps, P.z ), DPy0 );
			GetDerivatives( vec3( P.x, P.y + Eps, P.z ), DPy1 );
			GetDerivatives( vec3( P.x, P.y, P.z - Eps ), DPz0 );
			GetDerivatives( vec3( P.x, P.y, P.z + Eps ), DPz1 );

			PN = vec3(      length(DPx1) - length(DPx0),
							length(DPy1) - length(DPy0),
							length(DPz1) - length(DPz0) );
			PN = normalize(PN);
			P += PN * m_dt;
			out_pos.push_back( P );
		}
	}

	virtual void GetDerivatives(const vec3& P, vec3& DP) const = 0;
	virtual const char* GetClassName() const = 0;
};

//////////////////////////////////////////////////////////////////////////
class LorenzAttractor : public StrangeAttractor
{
public:
	float m_sigma, m_rho, m_beta;

	LorenzAttractor()
	{
		m_type = eAttractor_Lorentz;
		m_sigma = 10.0f;
		m_rho = 28.0f;
		m_beta = 8.0f / 3.0f;

		m_init_point = vec3( 5.0f, 5.0f, 5.0f );
		m_max_iter = 2000;
		// curl :
		//m_init_point = vec3( 0.1f, -0.1f, 0.001f );
		//m_max_iter = 500;
		m_fatness_scale = 0.25f;
		m_dt = 0.0025f;
		m_adaptative_dist = 0.24f;
	}

	virtual void GetDerivatives(const vec3& P, vec3& DP) const override
	{
		DP.x = (m_sigma * (P.y - P.x));
		DP.y = (P.x * (m_rho - P.z) - P.y);
		DP.z = (P.x * P.y - m_beta * P.z);
	}

	virtual const char* GetClassName() const override  { return "Lorenz"; }
};


//////////////////////////////////////////////////////////////////////////
class ArneodoAttractor : public StrangeAttractor
{
public:
	float m_alpha, m_beta, m_gamma;

	ArneodoAttractor()
	{
		m_type = eAttractor_Arneodo;
		m_alpha = -5.5;
		m_beta = 3.5;
		m_gamma = -1;

		m_init_point = vec3( 5.0f, 5.0f, 5.0f );
		m_max_iter = 2000;
		// curl :
		//m_init_point = vec3( 0.1f, -0.1f, 0.001f );
		//m_max_iter = 500;
		m_fatness_scale = 0.25f;
		m_dt = 0.0025f;
		m_adaptative_dist = 0.24f;
	}

	virtual void GetDerivatives(const vec3& P, vec3& DP) const override
	{
		DP.x = P.y;
		DP.y = P.z;
		DP.z = (-m_alpha * P.x - m_beta * P.y - P.z + m_gamma * (P.x*P.x*P.x));
	}

	virtual const char* GetClassName() const override  { return "Arneodo"; }
};

//////////////////////////////////////////////////////////////////////////
class ChenLeeAttractor : public StrangeAttractor
{
public:
	float m_alpha, m_beta, m_gamma;

	ChenLeeAttractor()
	{
		m_type = eAttractor_ChenLee;
		m_alpha = 5.0f;
		m_beta = -10.f;
		m_gamma = -0.38f;

		m_init_point = vec3( 5.0f, 5.0f, 5.0f );
		m_max_iter = 2000;
		// curl :
		//m_init_point = vec3( 0.1f, -0.1f, 0.001f );
		//m_max_iter = 500;
		m_fatness_scale = 0.25f;
		m_dt = 0.0025f;
		m_adaptative_dist = 0.24f;
	}

	virtual void GetDerivatives(const vec3& P, vec3& DP) const override
	{
		DP.x = m_alpha * P.x - P.y * P.z;
		DP.y = m_beta * P.y + P.x * P.z;
		DP.z = m_gamma * P.z + P.x * P.y / 3.0f;
	}

	virtual const char* GetClassName() const override { return "ChenLee"; }
};

//////////////////////////////////////////////////////////////////////////
class AizawaAttractor : public StrangeAttractor
{
public:
	float m_a, m_b, m_c, m_d, m_e, m_f;

	AizawaAttractor()
	{
		m_type = eAttractor_Aizawa;
		m_a = 0.95f;
		m_b = 0.7f;
		m_c = 0.6f;
		m_d = 3.5f;
		m_e = 0.25f;
		m_f = 0.1f;

		m_init_point = vec3( 0.1f, 0.0f, 0.0f );
		m_max_iter = 10000;
		// curl :
		//m_max_iter = 451;
		m_fatness_scale = 0.05f;
		m_dt = 0.01f;
		m_adaptative_dist = 0.24f;
	}

	virtual void GetDerivatives(const vec3& P, vec3& DP) const override
	{
		DP.x = (P.z - m_b) * P.x - m_d * P.y;
		DP.y = m_d * P.x + (P.z - m_b) * P.y;
        DP.z = m_c + m_a * P.z - (bigball::pow(P.z, 3.0f) / 3.0f) - (bigball::pow(P.x, 2.0f) + bigball::pow(P.y, 2.0f)) * (1.0f + m_e*P.z) + m_f * P.z * bigball::pow(P.x, 3.0f);
	}

	virtual const char* GetClassName() const override  { return "Aizawa"; }
};

//////////////////////////////////////////////////////////////////////////
class TSUCS2Attractor : public StrangeAttractor
{
public:
	float m_alpha, m_delta, m_c, m_g, m_beta, m_epsilon;

	TSUCS2Attractor()
	{
		m_type = eAttractor_TSUCS2;
		m_alpha = 40.0f;
		m_c = 55.0f;
		m_beta = 1.833f;
		m_delta = 0.16f;
		m_epsilon = 0.65f;
		m_g = 20.0f;

		m_init_point = vec3( 0.1f, 0.0f, 0.0f );
		m_max_iter = 1000;
		m_fatness_scale = 0.01f;
		m_dt = 0.001f;
		m_adaptative_dist = 0.24f;
	}

	virtual void GetDerivatives(const vec3& P, vec3& DP) const override
	{
		DP.x = m_alpha * (P.y - P.x) + m_delta * P.x * P.z;
		DP.y = m_c * P.x - P.x * P.z + m_g * P.y;
		DP.z = m_beta * P.z + P.x * P.y + m_epsilon * P.x * P.x * P.x;
	}
	virtual const char* GetClassName() const override  { return "TSUCS2"; }
};

//////////////////////////////////////////////////////////////////////////
class DequanLiAttractor : public StrangeAttractor
{
public:
	float m_alpha, m_beta, m_delta, m_epsilon, m_rho, m_xhi;

	DequanLiAttractor()
	{
		m_type = eAttractor_DequanLi;
		m_alpha = 40.0f;
		m_beta = 1.833f;
		m_delta = 0.16f;
		m_epsilon = 0.65f;
		m_rho = 55.0f;
		m_xhi = 20.0f;

		m_init_point = vec3( 5.0f, 5.0f, 5.0f );
		m_max_iter = 2000;

		m_fatness_scale = 0.25f;
		m_dt = 0.0025f;
		m_adaptative_dist = 0.24f;
	}

	virtual void GetDerivatives(const vec3& P, vec3& DP) const override
	{
		DP.x = m_alpha*(P.y - P.x) + m_delta*P.x*P.z; 
		DP.y = m_rho*P.x + m_xhi*P.y - P.x*P.z;
		DP.z = m_beta*P.z + P.x * P.y - m_epsilon*P.x*P.x;
	}

	virtual const char* GetClassName() const override  { return "DequanLi"; }
};

//////////////////////////////////////////////////////////////////////////
class LorentzMod2Attractor : public StrangeAttractor
{
public:
	float m_alpha, m_beta, m_delta, m_gamma;

	LorentzMod2Attractor()
	{
		m_type = eAttractor_LorentzMod2;
		m_alpha = 0.9f;
		m_beta = 5.0f;
		m_delta = 1.0f;
		m_gamma = 9.9f;

		m_init_point = vec3( 5.0f, 5.0f, 5.0f );
		m_max_iter = 2000;

		m_fatness_scale = 0.25f;
		m_dt = 0.0025f;
		m_adaptative_dist = 0.24f;
	}

	virtual void GetDerivatives(const vec3& P, vec3& DP) const override
	{
		DP.x = -m_alpha*P.x + P.y*P.y - P.z*P.z + m_alpha*m_gamma; 
		DP.y = P.x*(P.y - m_beta*P.z) + m_delta;
		DP.z = -P.z + P.x*(m_beta*P.y + P.z);
	}

	virtual const char* GetClassName() const override  { return "LorentzMod2"; }
};

class AttractorShape
{
public:
	StrangeAttractor*	attractor;
	Array<vec3>			line_points;
	Array<vec3>			tri_vertices;
	Array<int32>		tri_indices;

	AttractorShape() : attractor(nullptr) { }
};


struct AttractorShapeParams
{
	float fatness_scale;
	bool weld_vertex;
	int32 merge_start;
	int32 merge_end;
	int32 simplify_level;
	int32 local_edge_count;
	float crease_depth;
	float crease_width;
	float crease_bevel;

	// shape shearing
	float shearing_angle;
	float shearing_scale_x;
	float shearing_scale_y;

	AttractorShapeParams() :
		fatness_scale(1.0f),
		weld_vertex(true),
		merge_start(0),
		merge_end(0),
		simplify_level(1),
		local_edge_count(5),
		crease_depth(0.0f),
		crease_width(0.0f),
		crease_bevel(0.0f),
		shearing_angle(0.0f),
		shearing_scale_x(1.0f),
		shearing_scale_y(1.0f)
	{}
};

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
namespace SAUtils
{

	void		ComputeStrangeAttractorPoints(StrangeAttractor* attractor, vec3 seed, int32 iter, int32 rev_iter, int32 skip_iter, float step_factor, float target_dim, float shearing_angle, float shearing_scale_x, float shearing_scale_y, Array<vec3>& line_points /*out*/);
	void		ComputeStrangeAttractor(StrangeAttractor* attractor, vec3 seed, int32 iter);
	void		ComputeStrangeAttractorCurl();
	void		ComputeStrangeAttractorGradient();
	void		GenerateSolidMesh(const Array<vec3>& line_points, const AttractorShapeParams& Params, Array<vec3>& tri_vertices /*out*/, Array<vec3>* tri_normals /*out*/, Array<int32>& tri_indices /*out*/);

	void		GenerateLocalShape( Array<vec3>& local_shape, const AttractorShapeParams& params );
	void		GenerateTriIndices( Array<AttractorShape>& vShapes, int32 nLocalPoints );
	void		GenerateTriIndices(const Array<vec3>& tri_vertices, int32 nLocalPoints, Array<int32>& tri_indices /*out*/, const bool weld_vertex);
	void		GenerateTriVertices(Array<vec3>& tri_vertices, Array<vec3>* tri_normals, const Array<vec3>& local_shape, const Array<vec3> line_points, const AttractorShapeParams& params);

	void		WriteObjFile( const char* FileName, const Array<AttractorShape>& vAllShapes );
	void		WriteObjFile( const char* FileName, Array<vec3>& vPos, Array<int32>& vTriIdx );

	vec3		FindNearestFollowVector( const vec3& FromV, const vec3& NeighbourFollow, const vec3& NeighbourVX, const vec3& NeighbourVZ, int32 nLocalEdge );
	int32		FindNearestPoint( const Array<vec3>& line_points, int32 PointIdx, int32 IgnoreStart, int32 IgnoreEnd );
	void		MergeLinePoints( const Array<vec3>& line_points, const Array<vec3>& vVXFollow, const Array<vec3>& vVX, const Array<vec3>& vVZ, Array<vec3>& vMergePoints, Array<vec3>& vMergeFollow, const AttractorShapeParams& params );

	StrangeAttractor*	CreateAttractorType(String const& attractor_name);
    StrangeAttractor*	CreateAttractorType(eAttractorType attractor_type);
	void				GetAttractorTypeList(Array<String>& attractor_names);
};

#endif	// SAESTRANGEATTRACTORS_H