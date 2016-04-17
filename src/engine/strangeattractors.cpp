
#include "../sae.h"
#include "strangeattractors.h"
#include "system/file.h"


void SAUtils::ComputeStrangeAttractorGradient()
{
	AttractorShape S0;
	Array<AttractorShape> init_shapes;

	AizawaAttractor att0;
	S0.attractor = &att0;
	S0.attractor->m_max_iter = 3000;

	AttractorShapeParams params;
	Array<vec3> local_shape;
	GenerateLocalShape( local_shape, params );
	const int32 num_local_points = local_shape.size();

	S0.attractor->Loop( S0.line_points );

	init_shapes.resize( S0.line_points.size() / 10 );
	for( int32 ShapeIdx = 0; ShapeIdx < init_shapes.size(); ShapeIdx++ )
	{	
		AttractorShape& S = init_shapes[ShapeIdx];
		S.attractor = &att0;
		S.attractor->m_init_point = S0.line_points[10*ShapeIdx];
		S.attractor->m_max_iter = 200;
		S.attractor->LoopGradient( S.line_points );

		// Generate shapes
		params.fatness_scale = S.attractor->m_fatness_scale;
		params.weld_vertex = true;
		GenerateTriVertices(S.tri_vertices, nullptr, local_shape, S.line_points, params);
	}

	// Generate tri index
	GenerateTriIndices( init_shapes, num_local_points );

	std::string filename = "StrangeAttractor_";
	filename += att0.GetClassName();
	filename += "_gradient.obj";
	WriteObjFile( filename.c_str(), init_shapes );
}

void SAUtils::ComputeStrangeAttractorCurl()
{
	AttractorShape S0;
	Array<AttractorShape> init_shapes;

	//AizawaAttractor att0;
	//att0.m_init_point = vec3( 0.1f, 0, 0 );

	LorenzAttractor att0;
	S0.attractor = &att0;

	AttractorShapeParams params;
	Array<vec3> local_shape;
	GenerateLocalShape( local_shape, params );
	const int32 num_local_points = local_shape.size();

	S0.attractor->Loop( S0.line_points );

	init_shapes.resize( S0.line_points.size() / 10 );
	for( int32 ShapeIdx = 0; ShapeIdx < init_shapes.size(); ShapeIdx++ )
	{	
		AttractorShape& S = init_shapes[ShapeIdx];
		S.attractor = &att0;
		S.attractor->m_init_point = S0.line_points[10*ShapeIdx];
		S.attractor->m_max_iter = 200;
		S.attractor->LoopCurl( S.line_points );

		// Generate shapes
		params.fatness_scale = S.attractor->m_fatness_scale;
		params.weld_vertex = true;
		GenerateTriVertices(S.tri_vertices, nullptr, local_shape, S.line_points, params);
	}

	// Generate tri index
	GenerateTriIndices( init_shapes, num_local_points );

	std::string filename = "StrangeAttractor_";
	filename += att0.GetClassName();
	filename += "_curl.obj";
	WriteObjFile( filename.c_str(), init_shapes );
}

void SAUtils::ComputeStrangeAttractorPoints(StrangeAttractor* attractor, vec3 seed, int32 iter, int32 rev_iter, int32 skip_iter, float step_factor, float target_dim, float shearing_angle, float shearing_scale_x, float shearing_scale_y, Array<vec3>& line_points)
{
	attractor->m_init_point = seed;

	float init_dt = attractor->m_dt;
	float init_ad = attractor->m_adaptative_dist;

	attractor->m_dt = init_dt * step_factor;
	attractor->m_adaptative_dist = init_ad * step_factor;

	if( rev_iter > 0 )
	{
		attractor->m_max_iter = rev_iter;
		attractor->m_dt = -attractor->m_dt;
		attractor->LoopAdaptative( line_points );
		
        const int32 nb_point = line_points.size();
        for( int32 i=0; i < nb_point/2; i++ )
            line_points.Swap( i, nb_point - 1 - i );
		line_points.push_back( seed );

		attractor->m_dt = -attractor->m_dt;
	}

	attractor->m_max_iter = iter;
	attractor->LoopAdaptative( line_points );

	if( skip_iter > 0 )
	{
		skip_iter = min( skip_iter, (int32)line_points.size()-2 );
        line_points.erase(0,skip_iter);
	}

	if (shearing_scale_x != 1.f || shearing_scale_y != 1.f || shearing_angle != 0.f)
	{ 
		const float cf = bigball::cos(shearing_angle);
		const float sf = bigball::sin(shearing_angle);

		for (int32 i = 0; i < line_points.size(); ++i)
		{
			vec3 p = line_points[i];
			float rot_x = shearing_scale_x * (p.x * cf - p.y * sf);
			float rot_y = shearing_scale_y * (p.x * sf + p.y * cf);
			line_points[i].x = rot_x * cf + rot_y * sf;
			line_points[i].y = rot_x * -sf + rot_y * cf;
		}
	}

	// Adapt size
	if( target_dim > 0.0 )
	{
		vec3 min_pos( FLT_MAX, FLT_MAX, FLT_MAX ), max_pos( -FLT_MAX, -FLT_MAX, -FLT_MAX );
		for( int32 i = 0; i < line_points.size(); ++i )
		{
			min_pos.x = bigball::min( min_pos.x, line_points[i].x );
			min_pos.y = bigball::min( min_pos.y, line_points[i].y );
			min_pos.z = bigball::min( min_pos.z, line_points[i].z );
			max_pos.x = bigball::max( max_pos.x, line_points[i].x );
			max_pos.y = bigball::max( max_pos.y, line_points[i].y );
			max_pos.z = bigball::max( max_pos.z, line_points[i].z );
		}
		vec3 V = max_pos - min_pos;
		float dim_max = max( max( V.x, V.y ), V.z );
		float rescale = target_dim / dim_max;
		for( int32 i = 0; i < line_points.size(); ++i )
			 line_points[i] *= rescale;
	}

	attractor->m_dt = init_dt;
	attractor->m_adaptative_dist = init_ad;
}

void SAUtils::GenerateSolidMesh(const Array<vec3>& line_points, const AttractorShapeParams& params, Array<vec3>& tri_vertices /*out*/, Array<vec3>* tri_normals /*out*/, Array<int32>& tri_indices /*out*/)
{
	Array<vec3> local_shape;
	GenerateLocalShape( local_shape, params );
	const int32 num_local_points = local_shape.size();

	GenerateTriVertices(tri_vertices, tri_normals, local_shape, line_points, params);
	GenerateTriIndices(tri_vertices, num_local_points, tri_indices, params.weld_vertex);
}

void SAUtils::ComputeStrangeAttractor(StrangeAttractor* attractor, vec3 seed, int32 iter)
{
	// Lorentz attractor

	Array<AttractorShape> init_shapes;
	//LorenzAttractor att0;//, Att1;
	//att0.m_init_point = vec3( 5.0, 0, 0 );
	//Att1.m_init_point = vec3( 5.0, 5.0, 5.0 );
	
	/*AizawaAttractor att0;
	att0.m_max_iter = 5000;
	att0.m_fatness_scale = 0.1f;*/

	//att0.m_init_point = vec3( 0.1f, 0, 0 );
	//TSUCS2Attractor att0;
	//att0.m_init_point = vec3( 0.0, -0.05f, 0.01f );

//	att0.m_max_iter = 5000;
////	att0.m_init_point = vec3( -7.85f, -9.94f, 23.01f );
//	//att0.m_init_point = vec3( -2.0, -2.4f, 7.0 );
//	att0.m_init_point = vec3( 4.0, 0.01f, 0.5f );
//	att0.m_fatness_scale = 0.4f;

	AttractorShape AS0;
	AS0.attractor = attractor;
	AS0.attractor->m_init_point = seed;
	AS0.attractor->m_max_iter = iter;

	//AS1.Attractor = &Att1;
	//AS1.m_max_iter = m_max_iter;
	init_shapes.push_back( AS0 );
	//init_shapes.push_back( AS1 );

	AttractorShapeParams params;
	Array<vec3> local_shape;
	GenerateLocalShape( local_shape, params );
	const int32 num_local_points = local_shape.size();

	for( int32 ShapeIdx = 0; ShapeIdx < init_shapes.size(); ShapeIdx++ )
	{	
		AttractorShape& S = init_shapes[ShapeIdx];
		S.attractor->LoopAdaptative( S.line_points );

		// Generate shapes
		params.fatness_scale = S.attractor->m_fatness_scale;
		params.weld_vertex = true;
		GenerateTriVertices(S.tri_vertices, nullptr, local_shape, S.line_points, params);
	}

	// Generate tri index
	GenerateTriIndices( init_shapes, num_local_points );

	std::string filename = "StrangeAttractor_";
	filename += AS0.attractor->GetClassName();
	filename += ".obj";
	WriteObjFile( filename.c_str(), init_shapes );
}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void SAUtils::GenerateTriVertices(Array<vec3>& tri_vertices, Array<vec3>* tri_normals, const Array<vec3>& local_shape, const Array<vec3> line_points, const AttractorShapeParams& params)
{
	vec3 P_prev, P_current, P_next, VLastZ, VX_follow;
	
	// Compute normal to internal curvature
	Array<vec3> VX_array, VY_array, VZ_array, VX_follow_array;
	VX_array.resize( line_points.size() );
	VY_array.resize( line_points.size() );
	VZ_array.resize( line_points.size() );
	VX_follow_array.resize( line_points.size() );
	for( int32 i=1; i<line_points.size()-1; i++ ) 
	{
		P_prev = line_points[i-1];
		P_current = line_points[i];
		P_next = line_points[i+1];
		vec3 V0 = P_current - P_prev;
		vec3 V1 = P_next - P_current;
		float V0_len = length(V0);
		vec3 V0N = V0 / V0_len;
		vec3 V1N = normalize(V1);
		vec3 VZ = cross( V1N, V0N );
		float VZ_len = length(VZ);
		if (VZ_len > 1e-4)
			VZ /= VZ_len;
		else
			VZ = VX_array[i-1];

		vec3 V0RightN = cross( V0N, VZ );
		vec3 V1RightN = cross( V1N, VZ );
		VX_array[i] = V0RightN + V1RightN;
		VX_array[i] = normalize(VX_array[i]);
		VZ_array[i] = VZ;
		VY_array[i] = cross( VZ_array[i], VX_array[i] );

		// Replace VX_follow inside VX, VZ frame
		// Remove VY component
		if( i == 1 )
			VX_follow = VX_array[1];

		float dotY = dot( VY_array[i], VX_follow );
		if( bigball::abs(dotY) > 0.99f )
			int32 Break = 0;
		VX_follow -= VY_array[i] * dotY;
		VX_follow = normalize( VX_follow );
		VX_follow_array[i] = VX_follow;
	}
	VX_array[0] = VX_array[1];
	VX_array[line_points.size()-1] = VX_array[line_points.size()-2];
	VY_array[0] = VY_array[1];
	VY_array[line_points.size()-1] = VY_array[line_points.size()-2];
	VZ_array[0] = VZ_array[1];
	VZ_array[line_points.size()-1] = VZ_array[line_points.size()-2];
	VX_follow_array[0] = VX_follow_array[1];
	VX_follow_array[line_points.size()-1] = VX_follow_array[line_points.size()-2];

	Array<vec3> merge_line_points, merge_VX_follow;
	MergeLinePoints( line_points, VX_follow_array, VX_array, VZ_array, merge_line_points, merge_VX_follow, params );

	const int32 num_local_points = local_shape.size();
	const int32 line_inc = (params.simplify_level > 1 ? params.simplify_level : 1);
	Array<vec3> rotated_shape;
	rotated_shape.resize(num_local_points);
	for (int32 i = 1; i<merge_line_points.size() - 1; i += line_inc)
	{
		vec3 VX = VX_array[i];
		vec3 VY = VY_array[i];
		vec3 VZ = VZ_array[i];
		VX_follow = merge_VX_follow[i];
		P_prev = merge_line_points[i-1];
		P_current = merge_line_points[i];
		P_next = merge_line_points[i+1];
		vec3 V0 = P_current - P_prev;
		//vec3 V1 = P_next - P_current;
		float V0_len = length(V0);
		vec3 V0N = V0 / V0_len;
		vec3 V0RightN = cross( V0N, VZ );

#if 0
			// Compute angle between current VX and VX_follow
			float dotX = VX * VX_follow;
			float dotZ = VZ * VX_follow;
			float DAngle = -atan2( dotZ, dotX );
			// Accumulate angle
			AngularSpeed += DAngle;
			float ClampedAngle = AngularSpeed;
			if( ClampedAngle > MaxAngleSpeed )
				ClampedAngle = MaxAngleSpeed;
			else if( ClampedAngle < -MaxAngleSpeed )
				ClampedAngle = -MaxAngleSpeed;

			// Rotate VX_follow according to speed
			cf = cos( ClampedAngle );
			sf = sin( ClampedAngle );
			VX_follow = VX * cf + VZ * sf;
		
			// Reverse angle for local shape
			dotX = VX * VX_follow;
			dotZ = VZ * VX_follow;
			float FinalAngle = atan2( dotZ, dotX );
			cf = cos( FinalAngle );
			sf = sin( FinalAngle );

			// Reduce speed
			AngularSpeed *= SpeedDamping;
#endif

		// Twist local shape with follow vec inside current frame VX, VY, VZ
		float dot_x = dot( VX, VX_follow );
		float dot_z = dot( VZ, VX_follow );
		float delta_angle = bigball::atan2(dot_z, dot_x);
		float cf = bigball::cos(delta_angle);
		float sf = bigball::sin(delta_angle);

		float cos_alpha = dot( V0RightN, VX_array[i] );
		float scale = 1.0f / cos_alpha;
		for (int32 j = 0; j < num_local_points; ++j)
		{ 
			float local_x = local_shape[j].x * cf - local_shape[j].z * sf;
			float local_z = local_shape[j].x * sf + local_shape[j].z * cf;
			rotated_shape[j] = P_current + VX * (local_x * (scale * params.fatness_scale)) + VY * (local_shape[j].y * params.fatness_scale) + VZ * (local_z * params.fatness_scale);
		}
		
		for( int32 j = 0; j < num_local_points; ++j )
		{
			vec3 P_shape = rotated_shape[j];

			if (!params.weld_vertex)
			{
				tri_vertices.push_back(P_shape);
				tri_vertices.push_back(P_shape);
				if (tri_normals)
				{
					vec3 P_shape_prev = rotated_shape[(j + num_local_points - 1) % num_local_points];
					vec3 P_shape_next = rotated_shape[(j + 1) % num_local_points];
					vec3 temp_V = P_shape - P_current*2.f;
					vec3 normal_prev = normalize(temp_V + P_shape_prev);
					vec3 normal_next = normalize(temp_V + P_shape_next);
					tri_normals->push_back(normal_prev);
					tri_normals->push_back(normal_next);
				}
			}
			else
			{
				tri_vertices.push_back(P_shape);
				if (tri_normals)
				{ 
					vec3 normal_P = normalize(P_shape - P_current);
					tri_normals->push_back(normal_P);
				}
			}
		}

		VLastZ = VZ;
	}

	// For capping...
	int32 ShapeWOCapCount = tri_vertices.size();
	tri_vertices.push_back(merge_line_points[1]);
	vec3 start_normal = normalize(merge_line_points[0] - merge_line_points[1]);
	if (tri_normals)
		tri_normals->push_back(start_normal);
	if( !params.weld_vertex )
	{
		// Push start vertices
		for( int32 j = 0; j < num_local_points; ++j )
			tri_vertices.push_back(tri_vertices[2 * j]);

		if (tri_normals)
		{ 
			for (int32 j = 0; j < num_local_points; ++j)
				tri_normals->push_back(start_normal);
		}	
	}

	tri_vertices.push_back(merge_line_points[merge_line_points.size() - 2]);
	vec3 end_normal = normalize(merge_line_points[merge_line_points.size() - 1] - merge_line_points[merge_line_points.size() - 2]);
	if (tri_normals)
		tri_normals->push_back(end_normal);
	if( !params.weld_vertex )
	{
		// Push end vertices
		for( int32 j = 0; j < num_local_points; ++j )
			tri_vertices.push_back(tri_vertices[ShapeWOCapCount - 2 * num_local_points + 2 * j]);

		if (tri_normals)
		{
			for (int32 j = 0; j < num_local_points; ++j)
				tri_normals->push_back(end_normal);
		}
	}
}

void SAUtils::GenerateTriIndices(Array<AttractorShape>& vShapes, int32 num_local_points)
{
	// Generate tri index
	for( int32 ShapeIdx = 0; ShapeIdx < vShapes.size(); ShapeIdx++ )
	{	
		AttractorShape& S = vShapes[ShapeIdx];
		GenerateTriIndices(S.tri_vertices, num_local_points, S.tri_indices, true/*bWeldVertex*/);
	}
}

void SAUtils::GenerateTriIndices(const Array<vec3>& tri_vertices, int32 num_local_points, Array<int32>& tri_indices, const bool bWeldVertex)
{
	// Generate tri index
	if( bWeldVertex )
	{
		const int32 nShape = tri_vertices.size() / num_local_points;
		for( int32 i=0; i<nShape-1; i++ ) 
		{
			for( int32 j = 0; j < num_local_points; ++j )
			{
				tri_indices.push_back(i * num_local_points + j);
				tri_indices.push_back((i + 1) * num_local_points + j);
				tri_indices.push_back(i * num_local_points + (1 + j) % num_local_points);

				tri_indices.push_back(i * num_local_points + (1 + j) % num_local_points);
				tri_indices.push_back((i + 1) * num_local_points + j);
				tri_indices.push_back((i + 1) * num_local_points + (1 + j) % num_local_points);

			}
		}

		// First cap
		int32 nShapeSize = tri_vertices.size();
		int32 Cap0 = nShapeSize - 2;
		int32 Cap1 = nShapeSize - 1;

		for( int32 j = 0; j < num_local_points; ++j )
		{
			tri_indices.push_back( Cap0 );
			tri_indices.push_back( 0 * num_local_points + j );
			tri_indices.push_back( 0 * num_local_points + (1 + j) % num_local_points );
		}

		// Second cap
		for( int32 j = 0; j < num_local_points; ++j )
		{
			tri_indices.push_back( Cap1 );
			tri_indices.push_back( (nShape-1) * num_local_points + (1 + j) % num_local_points );
			tri_indices.push_back( (nShape-1) * num_local_points + j );
		}
	}
	else
	{
		const int32 nShape = tri_vertices.size() / (2 * num_local_points) - 1;
		for( int32 i=0; i<nShape-1; i++ ) 
		{
			for( int32 j = 0; j < num_local_points; ++j )
			{
				tri_indices.push_back( i * 2*num_local_points + (2*j+1) );
				tri_indices.push_back( (i+1) * 2*num_local_points + (2*j+1) );
				tri_indices.push_back( i * 2*num_local_points + (2*j+2) % (2*num_local_points) );

				tri_indices.push_back( i * 2*num_local_points + (2*j+2) % (2*num_local_points) );
				tri_indices.push_back( (i+1) * 2*num_local_points + (2*j+1) );
				tri_indices.push_back( (i+1) * 2*num_local_points + (2*j+2) % (2*num_local_points) );

			}
		}

		// First cap
		int32 nShapeSize = tri_vertices.size();
		int32 Cap0 = nShapeSize - 2 - 2*num_local_points;
		int32 Cap1 = nShapeSize - 1 - num_local_points;

		for( int32 j = 0; j < num_local_points; ++j )
		{
			tri_indices.push_back( Cap0 );
			tri_indices.push_back( Cap0 + 1 + j );
			tri_indices.push_back( Cap0 + 1 + (j+1)%num_local_points );
			//tri_indices.push_back( 0 * 2*num_local_points + (2*j+1) );
			//tri_indices.push_back( 0 * 2*num_local_points + (2*j+2) % (2*num_local_points) );
		}

		// Second cap
		for( int32 j = 0; j < num_local_points; ++j )
		{
			tri_indices.push_back( Cap1 );
			tri_indices.push_back( Cap1 + 1 + j );
			tri_indices.push_back( Cap1 + 1 + (j+1)%num_local_points );
			//tri_indices.push_back( (nShape-1) * 2*num_local_points + (2*j+2) % (2*num_local_points) );
			//tri_indices.push_back( (nShape-1) * 2*num_local_points + (2*j+1) );
		}
	}
}

void SAUtils::WriteObjFile( const char* filename, Array<vec3>& vPos, Array<int32>& tri_indices )
{
    File fp;
    if( !fp.Open( filename, true ) )
        return;

    String tmp_str;
    tmp_str = String::Printf( "#\n# %s\n#\n\n", filename );
    fp.SerializeString( tmp_str );
	//fopen_s( &fp, filename, "wt" );
	//fprintf( fp, "#\n# %s\n#\n\n", filename );

	int32 i, nPart = vPos.size(), nTri = tri_indices.size() / 3;
	for( i = 0; i < nPart; ++i )
	{
        tmp_str = String::Printf( "v %f %f %f\n", vPos[i].x, vPos[i].y, vPos[i].z );
        fp.SerializeString( tmp_str );
		//fprintf( fp, "v %f %f %f\n", vPos[i].x, vPos[i].y, vPos[i].z );
	}

	for( i = 0; i < nTri; ++i )
	{
        tmp_str = String::Printf("#\n# %s\n#\n\n", filename);
        fp.SerializeString( tmp_str );
		//fprintf( fp, "f %d %d %d\n", 1 + tri_indices[3*i], 1 + tri_indices[3*i+1], 1 + tri_indices[3*i+2] );
	}

	//fclose( fp );
}

void SAUtils::WriteObjFile( const char* filename, const Array<AttractorShape>& vAllShapes )
{
    File fp;
    if( !fp.Open( filename, true ) )
        return;
	//FILE* fp = NULL;
	//fopen_s( &fp, filename, "wt" );
    String tmp_str;
    tmp_str = String::Printf( "#\n# %s\n#\n\n", filename );
    fp.SerializeString( tmp_str );
	//fprintf( fp, "#\n# %s\n#\n\n", filename );

	// Write vertices
	for( int32 ShapeIdx = 0; ShapeIdx < vAllShapes.size(); ShapeIdx++ )
	{	
		const AttractorShape& S = vAllShapes[ShapeIdx];
		int32 i, nPart = S.tri_vertices.size();
		for( i = 0; i < nPart; ++i )
		{
			tmp_str = String::Printf("v %f %f %f\n", S.tri_vertices[i].x, S.tri_vertices[i].y, S.tri_vertices[i].z);
            fp.SerializeString( tmp_str );
			//fprintf( fp, "v %f %f %f\n", S.vShape[i].x, S.vShape[i].y, S.vShape[i].z );
		}
	}

	// Write faces
	int32 nVertexOffset = 0;
	for( int32 ShapeIdx = 0; ShapeIdx < vAllShapes.size(); ShapeIdx++ )
	{	
		const AttractorShape& S = vAllShapes[ShapeIdx];
		int32 i, nTri = S.tri_indices.size() / 3;
		for( i = 0; i < nTri; ++i )
		{
            tmp_str = String::Printf( "f %d %d %d\n", nVertexOffset + 1 + S.tri_indices[3*i], nVertexOffset + 1 + S.tri_indices[3*i+1], nVertexOffset + 1 + S.tri_indices[3*i+2] );
            fp.SerializeString( tmp_str );
			//fprintf( fp, "f %d %d %d\n", nVertexOffset + 1 + S.tri_indices[3*i], nVertexOffset + 1 + S.tri_indices[3*i+1], nVertexOffset + 1 + S.tri_indices[3*i+2] );
		}

		nVertexOffset += S.tri_vertices.size();
	}

	//fclose( fp );
}

void SAUtils::GenerateLocalShape( Array<vec3>& local_shapes, const AttractorShapeParams& params )
{
	bool is_simple_mesh = (params.crease_width <= 0.0 || params.crease_depth <= 0.0 ? true : false);
	const int32 nLocalPoint = (is_simple_mesh ? 1 : 5) * params.local_edge_count;
	local_shapes.resize( nLocalPoint );
	const float fatness = 1.0f;

	if( is_simple_mesh )
	{
		for( int32 edge_idx = 0; edge_idx < params.local_edge_count; ++edge_idx )
		{
			float Angle = 2.0f * F_PI * (float)edge_idx / (float)(params.local_edge_count);
			local_shapes[edge_idx] = vec3( fatness*bigball::cos( Angle ), 0.0f, fatness*bigball::sin( Angle ) );
		}
	}
	else
	{
		//const float min_wall = 0.7f;
		//float MinCreaseWidth = min_wall*0.5f / bigball::sin( F_PI / params.local_edge_count );


		for( int32 edge_idx = 0; edge_idx < params.local_edge_count; ++edge_idx )
		{
			float Angle = 2.0f * F_PI * (float)edge_idx / (float)(params.local_edge_count);
			local_shapes[5*edge_idx] = vec3( fatness*bigball::cos( Angle ), 0.0, fatness*bigball::sin( Angle ) );
		}

		vec3 V;
		float Len;
		for( int32 edge_idx = 0; edge_idx < params.local_edge_count; ++edge_idx )
		{
			const vec3& Prev = local_shapes[5*edge_idx];
			const vec3& Next = local_shapes[5*(edge_idx+1) % nLocalPoint];
			V = Next - Prev;
			Len = length(V);
			//V /= Len;

			local_shapes[5*edge_idx + 1] = Prev + V * params.crease_width;
			local_shapes[5*edge_idx + 2] = (Prev + V * (params.crease_width + params.crease_bevel)) * (1.0f - params.crease_depth);
			local_shapes[5*edge_idx + 3] = (Prev + V * (1.0f - (params.crease_width + params.crease_bevel))) * (1.0f - params.crease_depth);
			local_shapes[5*edge_idx + 4] = Prev + V * (1.0f - params.crease_width);
		}
	}
}

int32 SAUtils::FindNearestPoint( const Array<vec3>& line_points, int32 PointIdx, int32 IgnoreStart, int32 IgnoreEnd )
{
	vec3 V;
	vec3 PointToFind = line_points[PointIdx];
	float NearestDist = FLT_MAX;
	float Len;
	int32 Neighbour = PointIdx;
	for( int32 i=0; i<line_points.size(); ++i )
	{
		V = PointToFind - line_points[i];
		Len = length(V);
		if( Len < NearestDist )
		{
			if( i < IgnoreStart || i > IgnoreEnd )
			{
				NearestDist = Len;
				Neighbour = i;
			}
		}
	}
	return Neighbour;
}

vec3 SAUtils::FindNearestFollowVector( const vec3& FromV, const vec3& NeighbourFollow, const vec3& NeighbourVX, const vec3& NeighbourVZ, int32 nLocalEdge )
{
	vec3 VY = cross( NeighbourVZ, NeighbourVX );
	vec3 LeftFollow = cross( NeighbourFollow, VY );
	LeftFollow = normalize(LeftFollow);

	float BestDot = -FLT_MAX;
	vec3 BestFollow = NeighbourFollow;
    vec3 new_follow;
	for( int32 i = 0; i < nLocalEdge; ++i )
	{
		float Angle = 2.0f * F_PI * (float)i / (float)(nLocalEdge);
		float cf = bigball::cos( Angle );
		float sf = bigball::sin( Angle );
        new_follow = NeighbourFollow * cf + LeftFollow * sf;
		float dot_follow = dot( new_follow, FromV );
		if( dot_follow > BestDot )
		{
			BestDot = dot_follow;
			BestFollow = new_follow;
		}
	}

	return BestFollow;
}

void SAUtils::MergeLinePoints( const Array<vec3>& line_points, const Array<vec3>& VX_follow_array, const Array<vec3>& VX_array, const Array<vec3>& VZ_array, Array<vec3>& vMergePoints, Array<vec3>& vMergeFollow, const AttractorShapeParams& params )
{
	vMergePoints = line_points;
	vMergeFollow = VX_follow_array;

	// Manage start
	if( params.merge_start >= 1 )
	{
		// Find point nearest to us
		int32 StartIdx = 0;
		int32 Neighbour = FindNearestPoint( line_points, StartIdx, StartIdx, StartIdx + params.merge_start );

		// Check by which side we should go
		int32 Inc = 1;
		if( Neighbour >= line_points.size()-params.merge_start)
		{
			Inc = -1;
		}
		else if( Neighbour <= params.merge_start)
		{
			Inc = 1;
		}
		else
		{
			vec3 V0, V1;
			V0 = line_points[StartIdx+1] - line_points[Neighbour+1];
			V1 = line_points[StartIdx+1] - line_points[Neighbour-1];
			if( length(V0) < length(V1) )
				Inc = 1;
			else
				Inc = -1;
		}

		// Merge start
		const int32 Margin = 2;
		for( int32 Offset = 0; Offset < params.merge_start; Offset++ )
		{
			float MergeRatio = (Offset < Margin) ? 1.0f : (1.0f - (float)(Offset - Margin) / (float)(params.merge_start-Margin));
			vMergePoints[StartIdx + Offset] = line_points[StartIdx + Offset] * (1.0f - MergeRatio) + line_points[Neighbour + Offset*Inc] * MergeRatio;

			vec3 NeighbourFollow = FindNearestFollowVector( VX_follow_array[StartIdx + Offset], VX_follow_array[Neighbour + Offset*Inc], VX_array[Neighbour + Offset*Inc], VZ_array[Neighbour + Offset*Inc], params.local_edge_count );
			vMergeFollow[StartIdx + Offset] = VX_follow_array[StartIdx + Offset] * (1.0f - MergeRatio) + NeighbourFollow * MergeRatio;
			vMergeFollow[StartIdx + Offset] = normalize(vMergeFollow[StartIdx + Offset]);
		}
	}

	// Manage end
	if( params.merge_end >= 1 )
	{
		// Find point nearest to us
		int32 StartIdx = line_points.size() - 1 - params.merge_end;
		int32 Neighbour = FindNearestPoint( line_points, StartIdx, StartIdx - params.merge_end, StartIdx + params.merge_end );

		// Check by which side we should go
		int32 Inc = 1;
		if( Neighbour >= line_points.size()-params.merge_end)
		{
			Inc = -1;
		}
		else if( Neighbour <= params.merge_end)
		{
			Inc = 1;
		}
		else
		{
			vec3 V0, V1;
			V0 = line_points[StartIdx+1] - line_points[Neighbour+1];
			V1 = line_points[StartIdx+1] - line_points[Neighbour-1];
			if( length(V0) < length(V1) )
				Inc = 1;
			else
				Inc = -1;
		}

		// Merge end
		const int32 Margin = 2;
		for( int32 Offset = 0; Offset < params.merge_end; Offset++ )
		{
			float MergeRatio = (Offset >= params.merge_end - Margin) ? 1.0f : ((float)(Offset) / (float)(params.merge_end-Margin));
			vMergePoints[StartIdx + Offset] = line_points[StartIdx + Offset] * (1.0f - MergeRatio) + line_points[Neighbour + Offset*Inc] * MergeRatio;

			vec3 NeighbourFollow = FindNearestFollowVector( VX_follow_array[StartIdx + Offset], VX_follow_array[Neighbour + Offset*Inc], VX_array[Neighbour + Offset*Inc], VZ_array[Neighbour + Offset*Inc], params.local_edge_count );
			vMergeFollow[StartIdx + Offset] = VX_follow_array[StartIdx + Offset] * (1.0f - MergeRatio) + NeighbourFollow * MergeRatio;
			vMergeFollow[StartIdx + Offset] = normalize( vMergeFollow[StartIdx + Offset] );
		}
	}

}

StrangeAttractor* SAUtils::CreateAttractorType(String const& attractor_name)
{
	if( attractor_name == "Lorentz" )
		return CreateAttractorType(eAttractor_Lorentz );
	if (attractor_name == "Aizawa")
		return CreateAttractorType(eAttractor_Aizawa );
	if (attractor_name == "TSUCS2")
		return CreateAttractorType(eAttractor_TSUCS2 );
	if (attractor_name == "Arnoedo")
		return CreateAttractorType(eAttractor_Arneodo );
	if (attractor_name == "ChenLee")
		return CreateAttractorType(eAttractor_ChenLee );
	if (attractor_name == "DequanLi")
		return CreateAttractorType(eAttractor_DequanLi );
	if (attractor_name == "LorentzMod2")
		return CreateAttractorType(eAttractor_LorentzMod2 );
	
	return nullptr;
}

StrangeAttractor* SAUtils::CreateAttractorType(eAttractorType attractor_type)
{
    if( attractor_type == eAttractor_Lorentz )
        return new LorenzAttractor;
    if (attractor_type == eAttractor_Aizawa)
        return new AizawaAttractor;
    if (attractor_type == eAttractor_TSUCS2)
        return new TSUCS2Attractor;
    if (attractor_type == eAttractor_Arneodo)
        return new ArneodoAttractor;
    if (attractor_type == eAttractor_ChenLee)
        return new ChenLeeAttractor;
    if (attractor_type == eAttractor_DequanLi)
        return new DequanLiAttractor;
    if (attractor_type == eAttractor_LorentzMod2)
        return new LorentzMod2Attractor;
    
    return nullptr;
}

void SAUtils::GetAttractorTypeList(Array<String>& attractor_names)
{
	attractor_names.push_back("Lorentz");
	attractor_names.push_back("Aizawa");
	attractor_names.push_back("TSUCS2");
	attractor_names.push_back("Arnoedo");
	attractor_names.push_back("ChenLee");
	attractor_names.push_back("DequanLi");
	attractor_names.push_back("LorentzMod2");
}