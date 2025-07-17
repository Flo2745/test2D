#include "audiomanager.h"
#include "benchmarks.h"
#include "draw.h"
#include "human.h"
#include "particules.h"
#include "random.h"
#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/id.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <SFML/Audio.hpp>
#include <algorithm>
#include <array>
#include <filesystem>
#include <imgui.h>
#include <queue>
#include <random>
#include <set>
#include <vector>

constexpr float b2_pi = 3.14159265359f;

inline bool operator<( b2BodyId a, b2BodyId b )
{
	uint64_t ua = b2StoreBodyId( a );
	uint64_t ub = b2StoreBodyId( b );
	return ua < ub;
}

class ZeldaRupee : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new ZeldaRupee( context );
	}

	explicit ZeldaRupee( SampleContext* context )
		: Sample( context )
	{
		// Désactiver la gravité pour observer le rupee en lévitation
		b2World_SetGravity( m_worldId, { 0.0f, 0.0f } );

		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 10.0f;
		}

		// Créer le rupee avec des couleurs personnalisées
		CreateRupeeShapeProperlyJoined();
	}

private:
	// Fonction pour calculer l'intersection de deux segments
	static b2Vec2 IntersectSegments( const b2Vec2& A1, const b2Vec2& A2, const b2Vec2& B1, const b2Vec2& B2 )
	{
		float x1 = A1.x, y1 = A1.y;
		float x2 = A2.x, y2 = A2.y;
		float x3 = B1.x, y3 = B1.y;
		float x4 = B2.x, y4 = B2.y;

		float denom = ( y4 - y3 ) * ( x2 - x1 ) - ( x4 - x3 ) * ( y2 - y1 );
		if ( fabsf( denom ) < 1e-8f )
		{
			return A1;
		}

		float t = ( ( x4 - x3 ) * ( y1 - y3 ) - ( y4 - y3 ) * ( x1 - x3 ) ) / denom;
		float ix = x1 + t * ( x2 - x1 );
		float iy = y1 + t * ( y2 - y1 );

		return { ix, iy };
	}

	// Fonction pour calculer un polygone décalé
	static void ComputeOffsetPolygon( const b2Vec2* p, int n, float offsetOut, b2Vec2* out )
	{
		std::vector<b2Vec2> offsetLineStart( n );
		std::vector<b2Vec2> offsetLineEnd( n );

		for ( int i = 0; i < n; i++ )
		{
			int j = ( i + 1 ) % n;
			b2Vec2 edge = { p[j].x - p[i].x, p[j].y - p[i].y };

			b2Vec2 normal = { -edge.y, edge.x };
			float length = b2Length( normal );
			if ( length > 1e-8f )
			{
				float invLen = offsetOut / length;
				normal = { normal.x * invLen, normal.y * invLen };
			}

			offsetLineStart[i] = { p[i].x + normal.x, p[i].y + normal.y };
			offsetLineEnd[i] = { p[j].x + normal.x, p[j].y + normal.y };
		}

		for ( int i = 0; i < n; i++ )
		{
			int im1 = ( i + n - 1 ) % n;
			out[i] = IntersectSegments( offsetLineStart[im1], offsetLineEnd[im1], offsetLineStart[i], offsetLineEnd[i] );
		}
	}

	// Fonction pour créer le rupee avec des couleurs personnalisées
	void CreateRupeeShapeProperlyJoined()
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = { 0.0f, 0.0f };
		bd.angularDamping = 0.2f;
		b2BodyId bodyId = b2CreateBody( m_worldId, &bd );

		b2Vec2 rupee[6] = { { 0.0f, 1.4f }, { 0.7f, 0.7f }, { 0.7f, -0.7f }, { 0.0f, -1.4f }, { -0.7f, -0.7f }, { -0.7f, 0.7f } };

		// Définir la couleur pour le polygone central
		b2SurfaceMaterial centerMaterial = b2DefaultSurfaceMaterial();
		centerMaterial.friction = 0.3f;
		centerMaterial.restitution = 0.1f;
		centerMaterial.customColor = b2_colorDarkSeaGreen;

		b2ShapeDef centerSd = b2DefaultShapeDef();
		centerSd.density = 1.0f;
		centerSd.material = centerMaterial;

		b2Hull coreHull = b2ComputeHull( rupee, 6 );
		if ( coreHull.count > 2 )
		{
			b2Polygon corePolygon = b2MakePolygon( &coreHull, 0.0f );
			b2CreatePolygonShape( bodyId, &centerSd, &corePolygon );
		}

		float offsetDist = 0.4f;
		b2Vec2 offsetPoly[6];
		ComputeOffsetPolygon( rupee, 6, offsetDist, offsetPoly );

		// Définir la couleur pour les facettes
		b2SurfaceMaterial facetMaterial = b2DefaultSurfaceMaterial();
		facetMaterial.friction = 0.3f;
		facetMaterial.restitution = 0.1f;
		facetMaterial.customColor = b2_colorLightSteelBlue;

		b2ShapeDef facetSd = b2DefaultShapeDef();
		facetSd.material = facetMaterial;

		for ( int i = 0; i < 6; i++ )
		{
			int j = ( i + 1 ) % 6;
			b2Vec2 quad[4] = { rupee[i], rupee[j], offsetPoly[j], offsetPoly[i] };

			b2Hull facetHull = b2ComputeHull( quad, 4 );
			if ( facetHull.count >= 3 )
			{
				b2Polygon facetPolygon = b2MakePolygon( &facetHull, 0.0f );
				b2CreatePolygonShape( bodyId, &facetSd, &facetPolygon );
			}
		}
	}
};
static int ZeldaRupee = RegisterSample( "Pixel Art", "ZELDA Rupee", ZeldaRupee::Create );

class PixelArtHelloSample : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new PixelArtHelloSample( context );
	}

	explicit PixelArtHelloSample( SampleContext* context )
		: Sample( context )
	{
		b2World_SetGravity( m_worldId, { 0.0f, 0.0f } ); // ou 0 pour tester en lévitation

		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 18.0f;
		}

		CreatePixelArtHELLO();
	}

private:
	static constexpr int gridHeight = 5;
	static constexpr int gridWidth = 29; // 5x5 lettres + espaces
	static constexpr bool HELLO[gridHeight][gridWidth] = {
		// H     E     L     L     O
		{ 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1 },
		{ 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
		{ 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
		{ 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
		{ 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1 } };

	void CreatePixelArtHELLO()
	{
		const float pixelSize = 1.0f;
		const float startX = -( gridWidth * pixelSize ) * 0.5f + pixelSize * 0.5f;
		const float startY = +4.0f; // Ajuste pour centrer ou surélever

		for ( int y = 0; y < gridHeight; ++y )
		{
			for ( int x = 0; x < gridWidth; ++x )
			{
				if ( !HELLO[y][x] )
					continue;

				b2BodyDef bd = b2DefaultBodyDef();
				bd.type = b2_dynamicBody;
				bd.position = { startX + x * pixelSize, startY - y * pixelSize };
				bd.linearDamping = 2.0f;
				bd.angularDamping = 2.0f;
				b2BodyId bodyId = b2CreateBody( m_worldId, &bd );

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 0.1f;
				sd.material.friction = 0.6f;
				sd.material.customColor = 0xFFD700; // jaune/or

				b2Polygon box = b2MakeBox( pixelSize * 0.5f, pixelSize * 0.5f );
				b2CreatePolygonShape( bodyId, &sd, &box );
			}
		}
	}
};

static int pixelArtHelloSample = RegisterSample( "Pixel Art", "HELLO (pixel box2D)", PixelArtHelloSample::Create );
