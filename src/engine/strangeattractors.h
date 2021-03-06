
#ifndef SAESTRANGEATTRACTORS_H
#define SAESTRANGEATTRACTORS_H

namespace bigball
{
    class Archive;
}

struct AttractorSeedParams;
struct AttractorHandle;

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
    eAttractor_Hadley,
    eAttractor_LorentzMod1,
    eAttractor_LotkaVolterra,
    eAttractor_Halvorsen,
    eAttractor_TSUCS1,
    eAttractor_SpiralTest,
	eAttractor_MAX
};

enum eSaeArchiveVersion : uint32
{
    eSaeVersion_WithMergeSpan = 1,
    eSaeVersion_SpanPerHandle,
    
    eSaeVersion_Current = eSaeVersion_SpanPerHandle,
};

class StrangeAttractor
{
public:
	eAttractorType	m_type;
	vec3			m_init_point;
	float			m_dt;
	float			m_adaptative_dist;

	StrangeAttractor() : m_dt(0.01f) {}
    virtual ~StrangeAttractor() {}
	virtual void Loop( Array<vec3>& out_pos, int32 iter_count ) const
	{
		vec3 P(m_init_point), dp;

		for (int32 i = 0; i < iter_count; i++)
		{
			GetDerivatives( P, dp );
			P += dp * m_dt;
			out_pos.push_back( P );
		}
	}
	virtual void LoopAdaptative(Array<vec3>& out_pos, int32 iter_count, float step_factor, bool reverse) const
	{
		vec3 P( m_init_point ), dp;
		float dratio, len;
        float dt = (reverse ? -m_dt : m_dt);
		float ad_dt = bigball::abs(m_adaptative_dist / dt);
        dt *= step_factor;

		for (int32 i = 0; i < iter_count; i++)
		{
			GetDerivatives( P, dp );
			len = length(dp);
			dratio = ad_dt / len;

			P += dp * (dratio * dt);
			out_pos.push_back( P );
		}
	}
	virtual void LoopCurl(Array<vec3>& out_pos, int32 iter_count) const
	{
		const float Eps = m_dt*0.1f;
		vec3 P( m_init_point ), dpx0, dpx1, dpy0, dpy1, dpz0, dpz1, PCurl;

		for (int32 i = 0; i < iter_count; i++)
		{
			GetDerivatives( vec3( P.x - Eps, P.y, P.z ), dpx0 );
			GetDerivatives( vec3( P.x + Eps, P.y, P.z ), dpx1 );
			GetDerivatives( vec3( P.x, P.y - Eps, P.z ), dpy0 );
			GetDerivatives( vec3( P.x, P.y + Eps, P.z ), dpy1 );
			GetDerivatives( vec3( P.x, P.y, P.z - Eps ), dpz0 );
			GetDerivatives( vec3( P.x, P.y, P.z + Eps ), dpz1 );

			PCurl.x = ( (dpy1.z - dpy0.z) - (dpz1.y - dpz0.y) ) / (2*Eps);
			PCurl.y = ( (dpz1.x - dpz0.x) - (dpx1.z - dpx0.z) ) / (2*Eps);
			PCurl.z = ( (dpx1.y - dpx0.y) - (dpy1.x - dpy0.x) ) / (2*Eps);

			P += PCurl * m_dt;
			out_pos.push_back( P );
		}
	}
	virtual void LoopGradient(Array<vec3>& out_pos, int32 iter_count) const
	{
		const float Eps = m_dt*0.1f;
		vec3 P( m_init_point ), dpx0, dpx1, dpy0, dpy1, dpz0, dpz1, PN;

		for (int32 i = 0; i < iter_count; i++)
		{
			GetDerivatives( vec3( P.x - Eps, P.y, P.z ), dpx0 );
			GetDerivatives( vec3( P.x + Eps, P.y, P.z ), dpx1 );
			GetDerivatives( vec3( P.x, P.y - Eps, P.z ), dpy0 );
			GetDerivatives( vec3( P.x, P.y + Eps, P.z ), dpy1 );
			GetDerivatives( vec3( P.x, P.y, P.z - Eps ), dpz0 );
			GetDerivatives( vec3( P.x, P.y, P.z + Eps ), dpz1 );

			PN = vec3(      length(dpx1) - length(dpx0),
							length(dpy1) - length(dpy0),
							length(dpz1) - length(dpz0) );
			PN = normalize(PN);
			P += PN * m_dt;
			out_pos.push_back( P );
		}
	}

	virtual void GetDerivatives(const vec3& P, vec3& dp) const = 0;
	virtual const char* GetClassName() const = 0;
    virtual StrangeAttractor* NewClassObject() const = 0;
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
		//m_fatness_scale = 0.25f;
		m_dt = 0.0025f;
		m_adaptative_dist = 0.24f;
	}

	virtual void GetDerivatives(const vec3& P, vec3& dp) const override
	{
		dp.x = (m_sigma * (P.y - P.x));
		dp.y = (P.x * (m_rho - P.z) - P.y);
		dp.z = (P.x * P.y - m_beta * P.z);
	}

	virtual const char* GetClassName() const override  { return "Lorenz"; }
    virtual StrangeAttractor* NewClassObject() const override { return new LorenzAttractor; }
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
		//m_fatness_scale = 0.25f;
		m_dt = 0.0025f;
		m_adaptative_dist = 0.24f;
	}

	virtual void GetDerivatives(const vec3& P, vec3& dp) const override
	{
		dp.x = P.y;
		dp.y = P.z;
		dp.z = (-m_alpha * P.x - m_beta * P.y - P.z + m_gamma * (P.x*P.x*P.x));
	}

	virtual const char* GetClassName() const override  { return "Arneodo"; }
    virtual StrangeAttractor* NewClassObject() const override { return new ArneodoAttractor; }
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
		//m_fatness_scale = 0.25f;
		m_dt = 0.0025f;
		m_adaptative_dist = 0.24f;
	}

	virtual void GetDerivatives(const vec3& P, vec3& dp) const override
	{
		dp.x = m_alpha * P.x - P.y * P.z;
		dp.y = m_beta * P.y + P.x * P.z;
		dp.z = m_gamma * P.z + P.x * P.y / 3.0f;
	}

	virtual const char* GetClassName() const override { return "ChenLee"; }
    virtual StrangeAttractor* NewClassObject() const override { return new ChenLeeAttractor; }
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
		m_dt = 0.01f;
		m_adaptative_dist = 0.24f;
	}

	virtual void GetDerivatives(const vec3& P, vec3& dp) const override
	{
		dp.x = (P.z - m_b) * P.x - m_d * P.y;
		dp.y = m_d * P.x + (P.z - m_b) * P.y;
        dp.z = m_c + m_a * P.z - (bigball::pow(P.z, 3.0f) / 3.0f) - (bigball::pow(P.x, 2.0f) + bigball::pow(P.y, 2.0f)) * (1.0f + m_e*P.z) + m_f * P.z * bigball::pow(P.x, 3.0f);
	}

	virtual const char* GetClassName() const override  { return "Aizawa"; }
    virtual StrangeAttractor* NewClassObject() const override { return new AizawaAttractor; }
};

//////////////////////////////////////////////////////////////////////////
class TSUCS1Attractor : public StrangeAttractor
{
public:
	float m_alpha, m_delta, m_g, m_beta, m_epsilon;

	TSUCS1Attractor()
	{
		m_type = eAttractor_TSUCS1;
		m_alpha = 40.0f;
		//m_c = 55.0f;
		m_beta = 0.833f;
		m_delta = 0.5f;
		m_epsilon = 0.65f;
		m_g = 20.0f;

		m_init_point = vec3( 0.1f, 0.0f, 0.0f );
		//m_max_iter = 1000;
		//m_fatness_scale = 0.01f;
		m_dt = 0.001f;
		m_adaptative_dist = 0.24f;
	}

	virtual void GetDerivatives(const vec3& P, vec3& dp) const override
	{
		dp.x = m_alpha * (P.y - P.x) + m_delta * P.x * P.z;
		dp.y = /*m_c * P.x*/ - P.x * P.z + m_g * P.y;
		dp.z = m_beta * P.z + P.x * P.y - m_epsilon * P.x * P.x;
	}
	virtual const char* GetClassName() const override  { return "TSUCS1"; }
    virtual StrangeAttractor* NewClassObject() const override { return new TSUCS1Attractor; }
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
        //m_fatness_scale = 0.01f;
        m_dt = 0.001f;
        m_adaptative_dist = 0.24f;
    }
    
    virtual void GetDerivatives(const vec3& P, vec3& dp) const override
    {
        dp.x = m_alpha * (P.y - P.x) + m_delta * P.x * P.z;
        dp.y = m_c * P.x - P.x * P.z + m_g * P.y;
        dp.z = m_beta * P.z + P.x * P.y - m_epsilon * P.x * P.x;
    }
    virtual const char* GetClassName() const override  { return "TSUCS2"; }
    virtual StrangeAttractor* NewClassObject() const override { return new TSUCS2Attractor; }
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

		//m_fatness_scale = 0.25f;
		m_dt = 0.0025f;
		m_adaptative_dist = 0.24f;
	}

	virtual void GetDerivatives(const vec3& P, vec3& dp) const override
	{
		dp.x = m_alpha*(P.y - P.x) + m_delta*P.x*P.z; 
		dp.y = m_rho*P.x + m_xhi*P.y - P.x*P.z;
		dp.z = m_beta*P.z + P.x * P.y - m_epsilon*P.x*P.x;
	}

	virtual const char* GetClassName() const override  { return "DequanLi"; }
    virtual StrangeAttractor* NewClassObject() const override { return new DequanLiAttractor; }
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

		//m_fatness_scale = 0.25f;
		m_dt = 0.0025f;
		m_adaptative_dist = 0.24f;
	}

	virtual void GetDerivatives(const vec3& P, vec3& dp) const override
	{
		dp.x = -m_alpha*P.x + P.y*P.y - P.z*P.z + m_alpha*m_gamma; 
		dp.y = P.x*(P.y - m_beta*P.z) + m_delta;
		dp.z = -P.z + P.x*(m_beta*P.y + P.z);
	}

	virtual const char* GetClassName() const override  { return "LorentzMod2"; }
    virtual StrangeAttractor* NewClassObject() const override { return new LorentzMod2Attractor; }
};

//////////////////////////////////////////////////////////////////////////
class HadleyAttractor : public StrangeAttractor
{
public:
    float m_alpha, m_beta, m_delta, m_gamma;
    
    HadleyAttractor()
    {
        m_type = eAttractor_Hadley;
        m_alpha = 0.2f;
        m_beta = 4.0f;
        m_delta = 1.0f;
        m_gamma = 8.0f;
        
        m_init_point = vec3( 5.0f, 5.0f, 5.0f );

        m_dt = 0.0025f;
        m_adaptative_dist = 0.24f;
    }
    
    virtual void GetDerivatives(const vec3& P, vec3& dp) const override
    {
        dp.x = -P.y * P.y - P.z * P.z + m_alpha * (-P.x + m_gamma);
        dp.y = P.x*(P.y - m_beta*P.z) - P.y + m_delta;
        dp.z = -P.z + P.x*(m_beta*P.y + P.z);
    }
    
    virtual const char* GetClassName() const override  { return "Hadley"; }
    virtual StrangeAttractor* NewClassObject() const override { return new HadleyAttractor; }
};

//////////////////////////////////////////////////////////////////////////
class LorentzMod1Attractor : public StrangeAttractor
{
public:
    float m_alpha, m_beta, m_delta, m_gamma;
    
    LorentzMod1Attractor()
    {
        m_type = eAttractor_LorentzMod1;
        m_alpha = 0.1f;
        m_beta = 4.0f;
        m_delta = 0.08f;
        m_gamma = 14.0f;
        
        m_init_point = vec3( 5.0f, 5.0f, 5.0f );
        //m_max_iter = 2000;
        
        //m_fatness_scale = 0.25f;
        m_dt = 0.0025f;
        m_adaptative_dist = 0.24f;
    }
    
    virtual void GetDerivatives(const vec3& P, vec3& dp) const override
    {
        dp.x = -m_alpha*P.x + P.y*P.y - P.z*P.z + m_alpha*m_gamma;
        dp.y = P.x*(P.y - m_beta*P.z) + m_delta;
        dp.z = -P.z + P.x*(m_beta*P.y + P.z);
    }
    
    virtual const char* GetClassName() const override  { return "LorentzMod1"; }
    virtual StrangeAttractor* NewClassObject() const override { return new LorentzMod1Attractor; }
};

//////////////////////////////////////////////////////////////////////////
class LotkaVolterraAttractor : public StrangeAttractor
{
public:
    float m_a, m_b, m_c;
    
    LotkaVolterraAttractor()
    {
        m_type = eAttractor_LotkaVolterra;
        m_a = 2.9851f;
        m_b = 3.0f;
        m_c = 2.0f;
        
        m_init_point = vec3( 5.0f, 5.0f, 5.0f );
        //m_max_iter = 2000;
        
        //m_fatness_scale = 0.25f;
        m_dt = 0.0025f;
        m_adaptative_dist = 0.24f;
    }
    
    virtual void GetDerivatives(const vec3& P, vec3& dp) const override
    {
        dp.x = P.x * (1.0f - P.y + P.x*(m_c - m_a*P.z));
        dp.y = P.y * (-1.0f + P.x);
        dp.z = P.z * (-m_b + m_a*P.x*P.x);
    }
    
    virtual const char* GetClassName() const override  { return "LotkaVolterra"; }
    virtual StrangeAttractor* NewClassObject() const override { return new LotkaVolterraAttractor; }
};

//////////////////////////////////////////////////////////////////////////
class HalvorsenAttractor : public StrangeAttractor
{
public:
    float m_a;
    
    HalvorsenAttractor()
    {
        m_type = eAttractor_Halvorsen;
        m_a = 1.4f;
        
        m_init_point = vec3( 5.0f, 5.0f, 5.0f );
        m_dt = 0.0025f;
        m_adaptative_dist = 0.24f;
    }
    
    virtual void GetDerivatives(const vec3& P, vec3& dp) const override
    {
        dp.x = -m_a*P.x - 4.f*P.y - 4.f*P.z - P.y*P.y;
        dp.y = -m_a*P.y - 4.f*P.z - 4.f*P.x - P.z*P.z;
        dp.z = -m_a*P.z - 4.f*P.x - 4.f*P.y - P.x*P.x;
    }
    
    virtual const char* GetClassName() const override  { return "Halvorsen"; }
    virtual StrangeAttractor* NewClassObject() const override { return new HalvorsenAttractor; }
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
    
    virtual void LoopAdaptative(Array<vec3>& out_pos, int32 iter_count, float step_factor, bool reverse) const override
    {
        vec3 P( m_init_point.x, m_init_point.y, 0.f );
        float r = length( P.xy );
        float alpha = bigball::atan2(P.y, P.x);
        //float ad_dt = bigball::abs(m_adaptative_dist / m_dt);
        float dalpha = (reverse ? -1.f : 1.f) * step_factor * m_adaptative_dist / r;
        float dz = m_init_point.z;
        
        for (int32 i = 0; i < iter_count; i++)
        {
            alpha += dalpha;
            P.xy = vec2( bigball::cos(alpha), bigball::sin(alpha) ) * r;
            P.z += dz;
            
            //P += dp * (DRatio * m_dt);
            out_pos.push_back( P );
        }
    }
    
    virtual void GetDerivatives(const vec3& P, vec3& dp) const override
    {
        vec2 v = normalize( P.xy );
        dp.x = -v.y;
        dp.y = v.x;
        dp.z = m_init_point.z;
    }
    
    virtual const char* GetClassName() const override  { return "SpiralTest"; }
    virtual StrangeAttractor* NewClassObject() const override { return new SpiralTestAttractor; }
};

//////////////////////////////////////////////////////////////////////////
class AttractorFactory
{
public:
    AttractorFactory();
    ~AttractorFactory();
    void Create();
    void Destroy();
    StrangeAttractor*	CreateAttractorType(String const& attractor_name);
    StrangeAttractor*	CreateAttractorType(eAttractorType attractor_type);
    void				GetAttractorTypeList(Array<String>& attractor_names);
    
    StrangeAttractor* m_all_attractors[eAttractor_MAX];
};

//////////////////////////////////////////////////////////////////////////

struct AttractorLineParams
{
	//vec3 seed;
	//int32 iter;
	//int32 rev_iter;
	//int32 warmup_iter;
	float step_factor;
	float target_dim;

	// shape shearing
	float shearing_angle;
	float shearing_scale_x;
	float shearing_scale_y;

	AttractorLineParams() :
		//seed(1.f, 1.f, 1.f),
		//iter(2000),
		//rev_iter(0),
		//warmup_iter(200),
		step_factor(4.f),
		target_dim(0.f),
		shearing_angle(0.f),
		shearing_scale_x(1.f),
		shearing_scale_y(1.f)
	{}

	bool operator == (AttractorLineParams& oth)
	{
		return /*seed == oth.seed && iter == oth.iter && rev_iter == oth.rev_iter && warmup_iter == oth.warmup_iter &&*/ step_factor == oth.step_factor
			&& target_dim == oth.target_dim && shearing_angle == oth.shearing_angle && shearing_scale_x == oth.shearing_scale_x && shearing_scale_y == oth.shearing_scale_y;
	}
};

struct AttractorShapeParams
{
	float fatness_scale;
	bool weld_vertex;
	bool snap_interp;
	bool remove_line_ends;
    //bool freeze_bbox;
    bool show_bary;
	//int32 merge_start;
	//int32 merge_end;
	int32 simplify_level;
	int32 local_edge_count;
	float crease_depth;
	float crease_width;
	float crease_bevel;
    /** maximum distance where a merge can happen with another segment */
	float merge_dist;
    float max_drift;
    /** number of points involved in a merge */
    int32 merge_span;
    int32 target_bary_offset;
    int32 max_iter_count;

	AttractorShapeParams() :
		fatness_scale(1.0f),
		weld_vertex(true),
		snap_interp(false),
		remove_line_ends(false),
        //freeze_bbox(false),
        show_bary(false),
		//merge_start(0),
		//merge_end(0),
		simplify_level(1),
		local_edge_count(5),
		crease_depth(0.0f),
		crease_width(0.0f),
		crease_bevel(0.0f),
		merge_dist(0.f),
        max_drift(0.02f),
        merge_span(100),
        target_bary_offset(10),
        max_iter_count(5)
	{}

	bool operator == (AttractorShapeParams& oth)
	{
		return fatness_scale == oth.fatness_scale && weld_vertex == oth.weld_vertex && snap_interp == oth.snap_interp && remove_line_ends == oth.remove_line_ends && /*freeze_bbox == oth.freeze_bbox &&*/ show_bary == oth.show_bary && simplify_level == oth.simplify_level && local_edge_count == oth.local_edge_count
			&& crease_depth == oth.crease_depth && crease_width == oth.crease_width && crease_bevel == oth.crease_bevel && merge_dist == oth.merge_dist
            && max_drift == oth.max_drift && target_bary_offset == oth.target_bary_offset && max_iter_count == oth.max_iter_count;
	}
    
    void Serialize(class Archive& file);
};

//////////////////////////////////////////////////////////////////////////

struct AABB
{
    vec3 min;
    vec3 max;
    static bool BoundsIntersect(AABB const& a, AABB const& b);
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

struct AttractorSnapRange
{
    ivec2 src_points;
    ivec2 dst_segs;
};

struct AttractorSnapRangeEx : public AttractorSnapRange
{
	Array<SnapSegInfo> dst_seg_array;	// contains (src_points.y - src_points.x + 1) entries
	Array<SnapSegInfo> src_seg_array;	// reversed snap info, built from dst_seg_array, contains (dst_segs.y - dst_segs.x + 2) entries
    
    void Clear()
    {
        dst_seg_array.clear();
        src_seg_array.clear();
    }
};

//////////////////////////////////////////////////////////////////////////
struct AttractorOrientedCurve
{
	AttractorOrientedCurve()	{}
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
    void Resize(int count)
    {
        points.resize(count);
        frames.resize(count);
        follow_angles.resize(count);
        colors.resize(count);
    }

	Array<vec3>		points;
	Array<quat>		frames;
	Array<float>    follow_angles;
    Array<float>    colors;
	ivec4			snap_ranges;
};

struct AttractorCurveSnap
{
    float sq_dist = 1e8f;
    int32 seg = INDEX_NONE;
    AttractorOrientedCurve const* curve = nullptr;
};

struct SABarycenterRef
{
    int first_leading_seg;
    vec3 pos;
    int16 weight;
	int16 is_last_in_chain;
    Array<int32> seg_refs;
};

struct SACell
{
    Array<int> segs;
    Array<int> barys;
};

struct SABaryResult
{
    int cell_id;
    int bary_in_array_idx;
};

struct SASegResult
{
    int cell_id;
    int seg_in_array_idx;
    float t_seg;
    float sq_dist;
    
};

struct SAGrid
{
    Array<SACell> cells;
    Array<SABarycenterRef> bary_points;
    Array<int> bary_chains; // indices to first elements in chains
    Array<int> seg_bary_array;
    
    //vec3 cell_size;
    float cell_unit;
    ivec3 grid_dim;
    AABB grid_bound;
    
    void InitGrid(const Array<vec3>& line_points, int max_cell);
    int GetCellIdx(vec3 p) const;
    SABaryResult FindBaryCenterSeg(int seg_idx, vec3 p_0/*, vec3 p_1*/, float max_dist);
    SASegResult FindNearestSeg(int p_idx, const Array<vec3>& line_points, float max_dist, int exclude_range);
    int FindSegInRange(int bary_idx, int seg_idx, int range);
    void MoveBary(SABaryResult const& bary_ref, vec3 bary_pos, int bary_idx);
    void MoveBary(vec3 old_bary_pos, vec3 bary_pos, int bary_idx);
    void MovePoint(vec3 old_pos, vec3 new_pos, int pt_idx);
};

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
namespace SAUtils
{

    void        GenerateSnappedCurve(AttractorOrientedCurve& curve /*in-out*/, const AttractorCurveSnap& snap_start, const AttractorCurveSnap& snap_end, AttractorShapeParams const& shape_params, const bool blend_positions);
	void		ComputeStrangeAttractorPoints(StrangeAttractor& attractor, AttractorSeedParams const& seed_params, AttractorLineParams const& params, Array<vec3>& line_points /*out*/);
    void		IterateStrangeAttractorPoint(StrangeAttractor& attractor, AttractorSeedParams const& seed_params, AttractorLineParams const& params, int32 iter_count, vec3& out_pos);
    void        GenerateSolidMesh(Array<AttractorOrientedCurve> const& curves, const AttractorShapeParams& params, Array<vec3>& tri_vertices /*out*/, Array<vec3>* tri_normals /*out*/, Array<float>* tri_colors /*out*/, Array<int32>& tri_indices /*out*/, Array<int32>* indice_offsets, float rescale_factor);
    void        GenerateSolidMesh(AttractorOrientedCurve const& curve, const AttractorShapeParams& params, Array<vec3>& tri_vertices /*out*/, Array<vec3>* tri_normals /*out*/, Array<int32>& tri_indices /*out*/, float rescale_factor);

	void        GenerateFrames(AttractorOrientedCurve& line_framed);
    void        GenerateFrames(AttractorOrientedCurve& line_framed, int from_idx, int to_idx, bool start_continuity, bool end_continuity, vec3* start_vector = nullptr, vec3* end_vector = nullptr);
	void		GenerateSnappedLinesWithFrames(const Array<vec3>& line_points, const Array<quat>& line_frames, const Array<float>& follow_angles, const Array<AttractorSnapRange>& snap_ranges, AttractorShapeParams const& shape_params, Array<AttractorOrientedCurve>& framed_lines /*out*/, const bool blend_positions);
    void        GenerateColors(AttractorOrientedCurve& line_framed, float color);
	void		MergeLinePoints(AttractorOrientedCurve const& line_framed, const Array<AttractorHandle>& attr_handles, AttractorShapeParams const& shape_params, Array<AttractorOrientedCurve>& snapped_lines);
	void		MergeLinePoints2(AttractorOrientedCurve const& line_framed, const Array<AttractorHandle>& attr_handles, AttractorShapeParams const& shape_params, Array<AttractorOrientedCurve>& snapped_lines);
    void		MergeLinePoints3(AttractorOrientedCurve const& line_framed, const Array<AttractorHandle>& attr_handles, AttractorShapeParams const& shape_params, Array<AttractorOrientedCurve>& snapped_lines);
    void		MergeLinePoints4(AttractorOrientedCurve const& line_framed, const Array<AttractorHandle>& attr_handles, AttractorShapeParams const& shape_params, Array<AttractorOrientedCurve>& snapped_lines);
    void		MergeLinePoints5(AttractorOrientedCurve const& line_framed, const Array<AttractorHandle>& attr_handles, AttractorShapeParams const& shape_params, Array<AttractorOrientedCurve>& snapped_lines);
    void		MergeCurves(Array<AttractorOrientedCurve>& curves, AttractorShapeParams const& shape_params, const Array<AttractorHandle>& attr_handles);
    
	void		GenerateLocalShape( Array<vec3>& local_shape, const AttractorShapeParams& params );
	//void		GenerateTriIndices( Array<AttractorShape>& vShapes, int32 nLocalPoints );
	void		GenerateTriIndices(const Array<vec3>& tri_vertices, int32 nLocalPoints, Array<int32>& tri_indices /*out*/, const bool weld_vertex, int32 base_vertex);
	void		GenerateTriVertices(Array<vec3>& tri_vertices, Array<vec3>* tri_normals, Array<float>* tri_colors, const Array<vec3>& local_shape, const AttractorOrientedCurve& curve, const AttractorShapeParams& params, float rescale_factor);

	//void		WriteObjFile( const char* FileName, const Array<AttractorShape>& vAllShapes );
	void		WriteObjFile( Archive& file, const Array<vec3>& tri_vertices, const Array<int32>& tri_indices );
	void		WritePlyFile(Archive& file, const Array<vec3>& tri_vertices, const Array<int32>& tri_indices);

	void		FindNearestFollowVector(quat const& src_frame, float src_follow_angle, quat const& dst_frame, float dst_follow_angle, quat const& dst_cmp_frame, float dst_cmp_follow_angle, int32 local_edge_count, vec3& src_follow, vec3& dst_follow);
	int32		FindNearestPoint( const Array<vec3>& line_points, int32 PointIdx, int32 IgnoreStart, int32 IgnoreEnd );
#if 0
	void		MergeLinePointsOld( const Array<vec3>& line_points, const Array<vec3>& vVXFollow, const Array<vec3>& vVX, const Array<vec3>& vVZ, Array<vec3>& vMergePoints, Array<vec3>& vMergeFollow, const AttractorShapeParams& params );
#endif
    
    const int points_in_bound = 20;
    void ComputeBounds(const Array<vec3>& line_points, float margin, Array<AABB>& bounds);
	bool FindSnapRange(const Array<vec3>& line_points, int b_idx0, int b_idx1, float merge_dist, AttractorSnapRange& snap_range );
	bool FindSnapRangeEx(const Array<vec3>& line_points, int b_idx0, int b_idx1, float merge_dist, AttractorSnapRangeEx& snap_range);
	bool ComputeReverseSnapRangeExInfo(const Array<vec3>& line_points, float merge_dist, AttractorSnapRangeEx& snap_range);
	int	FindNextBestSnapSeg(const Array<vec3>& line_points, int c_1_next, int cur_seg_0, int inc, float sq_merge_dist, float& best_t);
    int	FindNextBestSnapSeg(const Array<vec3>& line_points, vec3 pos, int cur_seg_0, int inc, float sq_merge_dist, float& best_t);
    void FindNearestCurveSegment(AttractorOrientedCurve const& curve, ivec2 seg_range, vec3 p_0, vec3 p_1, float sq_merge_dist, AttractorCurveSnap& min_0, AttractorCurveSnap& min_1);
};

#endif	// SAESTRANGEATTRACTORS_H
