
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
    eAttractor_SpiralTest,
	eAttractor_MAX
};

class StrangeAttractor
{
public:
	eAttractorType	m_type;
	vec3			m_init_point;
	//int32			m_max_iter;
	float			m_fatness_scale;
	float			m_dt;
	float			m_adaptative_dist;

	StrangeAttractor() : m_dt(0.01f) {}
    virtual ~StrangeAttractor() {}
	virtual void Loop( Array<vec3>& out_pos, int32 iter_count ) const
	{
		vec3 P(m_init_point), DP;

		for (int32 i = 0; i < iter_count; i++)
		{
			GetDerivatives( P, DP );
			P += DP * m_dt;
			out_pos.push_back( P );
		}
	}
	virtual void LoopAdaptative(Array<vec3>& out_pos, int32 iter_count) const
	{
		vec3 P( m_init_point ), DP;
		float DRatio, Len;
		float AdDt = bigball::abs(m_adaptative_dist / m_dt);

		for (int32 i = 0; i < iter_count; i++)
		{
			
			GetDerivatives( P, DP );
			Len = length(DP);
			DRatio = AdDt / Len;

			P += DP * (DRatio * m_dt);
			out_pos.push_back( P );
		}
	}
	virtual void LoopCurl(Array<vec3>& out_pos, int32 iter_count) const
	{
		const float Eps = m_dt*0.1f;
		vec3 P( m_init_point ), DPx0, DPx1, DPy0, DPy1, DPz0, DPz1, PCurl;

		for (int32 i = 0; i < iter_count; i++)
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
	virtual void LoopGradient(Array<vec3>& out_pos, int32 iter_count) const
	{
		const float Eps = m_dt*0.1f;
		vec3 P( m_init_point ), DPx0, DPx1, DPy0, DPy1, DPz0, DPz1, PN;

		for (int32 i = 0; i < iter_count; i++)
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
		//m_max_iter = 2000;
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
		//m_max_iter = 2000;
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
		//m_max_iter = 2000;
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
		//m_max_iter = 10000;
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
		//m_max_iter = 1000;
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
		//m_max_iter = 2000;

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
		//m_max_iter = 2000;

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

//////////////////////////////////////////////////////////////////////////
class SpiralTestAttractor : public StrangeAttractor
{
public:

    SpiralTestAttractor()
    {
        m_type = eAttractor_SpiralTest;
        
        m_init_point = vec3( 5.f, 0.f, 0.f );

        m_dt = 0.0025f;
        m_adaptative_dist = 0.24f;
    }
    
    virtual void LoopAdaptative(Array<vec3>& out_pos, int32 iter_count) const override
    {
        vec3 P( m_init_point.x, m_init_point.y, 0.f );
        float r = length( P.xy );
        float alpha = bigball::atan2(P.y, P.x);
        //float AdDt = bigball::abs(m_adaptative_dist / m_dt);
        float dalpha = m_adaptative_dist / r;
        float dz = m_init_point.z;
        
        for (int32 i = 0; i < iter_count; i++)
        {
            alpha += dalpha;
            P.xy = vec2( bigball::cos(alpha), bigball::sin(alpha) ) * r;
            P.z += dz;
            
            //P += DP * (DRatio * m_dt);
            out_pos.push_back( P );
        }
    }
    
    virtual void GetDerivatives(const vec3& P, vec3& DP) const override
    {
        vec2 v = normalize( P.xy );
        DP.x = -v.y;
        DP.y = v.x;
        DP.z = m_init_point.z;
    }
    
    virtual const char* GetClassName() const override  { return "SpiralTest"; }
};

//////////////////////////////////////////////////////////////////////////
class AttractorShape
{
public:
	StrangeAttractor*	attractor;
	Array<vec3>			line_points;
	Array<vec3>			tri_vertices;
	Array<int32>		tri_indices;

	AttractorShape() : attractor(nullptr) { }
};

struct AttractorLineParams
{
	vec3 seed;
	int32 iter;
	int32 rev_iter;
	int32 warmup_iter;
	float step_factor;
	float target_dim;

	// shape shearing
	float shearing_angle;
	float shearing_scale_x;
	float shearing_scale_y;

	AttractorLineParams() :
		seed(1.f, 1.f, 1.f),
		iter(2000),
		rev_iter(0),
		warmup_iter(200),
		step_factor(4.f),
		target_dim(0.f),
		shearing_angle(0.f),
		shearing_scale_x(1.f),
		shearing_scale_y(1.f)
	{}

	bool operator == (AttractorLineParams& oth)
	{
		return seed == oth.seed && iter == oth.iter && rev_iter == oth.rev_iter && warmup_iter == oth.warmup_iter && step_factor == oth.step_factor 
			&& target_dim == oth.target_dim && shearing_angle == oth.shearing_angle && shearing_scale_x == oth.shearing_scale_x && shearing_scale_y == oth.shearing_scale_y;
	}
};

struct AttractorShapeParams
{
	float fatness_scale;
	bool weld_vertex;
	bool snap_interp;
	bool remove_line_ends;
	//int32 merge_start;
	//int32 merge_end;
	int32 simplify_level;
	int32 local_edge_count;
	float crease_depth;
	float crease_width;
	float crease_bevel;
	float merge_dist;

	AttractorShapeParams() :
		fatness_scale(1.0f),
		weld_vertex(true),
		snap_interp(false),
		remove_line_ends(false),
		//merge_start(0),
		//merge_end(0),
		simplify_level(1),
		local_edge_count(5),
		crease_depth(0.0f),
		crease_width(0.0f),
		crease_bevel(0.0f),
		merge_dist(0.f)
	{}

	bool operator == (AttractorShapeParams& oth)
	{
		return fatness_scale == oth.fatness_scale && weld_vertex == oth.weld_vertex && snap_interp == oth.snap_interp && remove_line_ends == oth.remove_line_ends && simplify_level == oth.simplify_level && local_edge_count == oth.local_edge_count
			&& crease_depth == oth.crease_depth && crease_width == oth.crease_width && crease_bevel == oth.crease_bevel && merge_dist == oth.merge_dist;
	}
};

struct AttractorHandle
{
public:
    AttractorHandle() : m_type(eHT_Move)	{}
	~AttractorHandle()	{}

	bool operator == (AttractorHandle const& oth)
	{
		return /*m_transform == oth.m_transform
			   &&*/ m_type == oth.m_type && m_line_idx == oth.m_line_idx && m_mesh_idx == oth.m_mesh_idx;
	}

    enum eHandleType : int32
	{
		eHT_Move,
		eHT_Cut,
        eHT_Count
	};

	//transform	m_transform;
	eHandleType	m_type;
	int32		m_line_idx;
	int32		m_mesh_idx;
};

struct AttractorFreeHandle
{
public:
	AttractorFreeHandle() {}
	~AttractorFreeHandle()	{}

	bool operator == (AttractorHandle const& oth)
	{
		return m_line_idx == oth.m_line_idx;
	}

	enum eHandleType : int32
	{
		eHT_Move,
		eHT_Cut,
		eHT_Count
	};

	int32		m_line_idx;
	
};

struct AABB
{
    vec3 min;
    vec3 max;
    static bool BoundsIntersect(AABB const& a, AABB const& b);
};

struct AttractorSnapRange
{
	ivec2 src_points;
	ivec2 dst_segs;
};

struct SnapSegInfo
{
	int32 seg_idx;
	float t_seg;
	float weight;
};

struct RefSnap
{
	int32 snap_range_idx;
	int32 p_idx;
	int32 seg_idx;
	float t_seg;

	bool operator == (RefSnap const& oth)
	{
		return snap_range_idx == oth.snap_range_idx && p_idx == oth.p_idx && seg_idx == oth.seg_idx && t_seg == oth.t_seg;
	}
};

struct AttractorSnapRangeEx : public AttractorSnapRange
{
	Array<SnapSegInfo> dst_seg_array;	// contains (src_points.y - src_points.x + 1) entries
	Array<SnapSegInfo> src_seg_array;	// reversed snap info, built from dst_seg_array, contains (dst_segs.y - dst_segs.x + 2) entries
};

struct AttractorLineFramed
{
	AttractorLineFramed()	{}
	void Clear()
	{
		points.clear();
		frames.clear();
		follow_angles.clear();
		snap_ranges = ivec4(INDEX_NONE, INDEX_NONE, INDEX_NONE, INDEX_NONE);
	}
	void SetDefaultRanges()
	{
		snap_ranges = ivec4(0, 0, points.size(), points.size());
	}

	Array<vec3>		points;
	Array<quat>		frames;
	Array<float>    follow_angles;
	ivec4			snap_ranges;
};

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
namespace SAUtils
{

	void		ComputeStrangeAttractorPoints(StrangeAttractor* attractor, AttractorLineParams const& params, Array<vec3>& line_points /*out*/);
	//void		ComputeStrangeAttractor(StrangeAttractor* attractor, vec3 seed, int32 iter);
	void		GenerateSolidMesh(Array<AttractorLineFramed> const& snapped_lines, const AttractorShapeParams& Params, Array<vec3>& tri_vertices /*out*/, Array<vec3>* tri_normals /*out*/, Array<float>* tri_colors /*out*/, Array<int32>& tri_indices /*out*/);

	void        GenerateFrames(AttractorLineFramed& line_framed);
    void        GenerateFrames(AttractorLineFramed& line_framed, int from_idx, int to_idx, bool start_continuity, bool end_continuity, vec3* start_vector = nullptr, vec3* end_vector = nullptr);
	void		MergeLinePoints(AttractorLineFramed const& line_framed, const Array<AttractorHandle>& attr_handles, AttractorShapeParams const& shape_params, Array<AttractorLineFramed>& snapped_lines);
	void		MergeLinePoints2(AttractorLineFramed const& line_framed, const Array<AttractorHandle>& attr_handles, AttractorShapeParams const& shape_params, Array<AttractorLineFramed>& snapped_lines);
	void		GenerateLocalShape( Array<vec3>& local_shape, const AttractorShapeParams& params );
	void		GenerateTriIndices( Array<AttractorShape>& vShapes, int32 nLocalPoints );
	void		GenerateTriIndices(const Array<vec3>& tri_vertices, int32 nLocalPoints, Array<int32>& tri_indices /*out*/, const bool weld_vertex, int32 base_vertex);
	void		GenerateTriVertices(Array<vec3>& tri_vertices, Array<vec3>* tri_normals, Array<float>* tri_colors, const Array<vec3>& local_shape, AttractorLineFramed const & line_framed, /*const Array<vec3>& line_points, const Array<quat>& frames, const Array<float>& follow_angles,*/ const AttractorShapeParams& params);

	void		WriteObjFile( const char* FileName, const Array<AttractorShape>& vAllShapes );
	void		WriteObjFile( const char* FileName, Array<vec3>& vPos, Array<int32>& vTriIdx );

	void		FindNearestFollowVector(quat const& src_frame, float src_follow_angle, quat const& dst_frame, float dst_follow_angle, int32 local_edge_count, vec3& src_follow, vec3& dst_follow);
	int32		FindNearestPoint( const Array<vec3>& line_points, int32 PointIdx, int32 IgnoreStart, int32 IgnoreEnd );
#if 0
	void		MergeLinePointsOld( const Array<vec3>& line_points, const Array<vec3>& vVXFollow, const Array<vec3>& vVX, const Array<vec3>& vVZ, Array<vec3>& vMergePoints, Array<vec3>& vMergeFollow, const AttractorShapeParams& params );
#endif

	StrangeAttractor*	CreateAttractorType(String const& attractor_name);
    StrangeAttractor*	CreateAttractorType(eAttractorType attractor_type);
	void				GetAttractorTypeList(Array<String>& attractor_names);
    
    const int points_in_bound = 20;
    void ComputeBounds(const Array<vec3>& line_points, float margin, Array<AABB>& bounds);
	bool FindSnapRange(const Array<vec3>& line_points, int b_idx0, int b_idx1, float merge_dist, AttractorSnapRange& snap_range );
	bool FindSnapRangeEx(const Array<vec3>& line_points, int b_idx0, int b_idx1, float merge_dist, AttractorSnapRangeEx& snap_range);
	bool ComputeReverseSnapRangeExInfo(const Array<vec3>& line_points, float merge_dist, AttractorSnapRangeEx& snap_range);
	int	FindNextBestSnapSeg(const Array<vec3>& line_points, int c_1_next, int cur_seg_0, int inc, float sq_merge_dist, float& best_t);
};

#endif	// SAESTRANGEATTRACTORS_H