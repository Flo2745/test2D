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
static int ZeldaRupee = RegisterSample( "1-Objects", "ZELDA Rupee", ZeldaRupee::Create );

class SensorPerfTest : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new SensorPerfTest( context );
	}

	explicit SensorPerfTest( SampleContext* context )
		: Sample( context )
	{
		// On désactive totalement la gravité
		b2World_SetGravity( m_worldId, { 0.0f, 0.0f } );
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 20.0f;
		}
		CreateSensors();
	}

	void Step() override
	{
		auto t0 = std::chrono::high_resolution_clock::now();
		Sample::Step();
		auto t1 = std::chrono::high_resolution_clock::now();
		m_stepMs = std::chrono::duration<float, std::milli>( t1 - t0 ).count();
		UpdateGUI();
	}

private:
	float m_stepMs = 0.0f;
	std::vector<b2BodyId> m_bodies;

	void CreateSensors()
	{
		const int target = 50000;
		const int cols = 500;
		const float spacing = 0.4f;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;

		// Le cercle de base
		b2Circle circle;
		circle.center = { 0.0f, 0.0f };
		circle.radius = 0.1f;

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.isSensor = true; // plus de détection de contact
		sd.density = 1.0f;	// masse nécessaire pour l'intégration

		m_bodies.reserve( target );
		int count = 0;

		for ( int i = 0; i < cols && count < target; ++i )
		{
			for ( int j = 0; j < cols && count < target; ++j )
			{
				bd.position = { i * spacing, j * spacing };
				b2BodyId body = b2CreateBody( m_worldId, &bd );
				b2CreateCircleShape( body, &sd, &circle );
				m_bodies.push_back( body );
				++count;
			}
		}
	}

	void UpdateGUI()
	{
		ImGui::Begin( "Sensor Perf" );
		ImGui::Text( "Bodies : %d", (int)m_bodies.size() );
		ImGui::Text( "Step() : %.2f ms", m_stepMs );
		ImGui::End();
	}
};

static int idx = RegisterSample( "Performance", "50k Sensors (no collisions)", SensorPerfTest::Create );

class ColorGuesser : public Sample
{
public:
	// Méthode statique de création pour le framework
	static Sample* Create( SampleContext* context )
	{
		return new ColorGuesser( context );
	}

	// Constructeur
	explicit ColorGuesser( SampleContext* context )
		: Sample( context )
		, m_gridSize( 2 )
		, m_level( 1 )
		, m_explosionRadius( 2.0f )
		, m_explosionMagnitude( 20.0f )
		, m_explosionActive( false )
		, m_explosionTimer( 0 )
		, m_explosionCountdown( INITIAL_EXPLOSION_COUNTDOWN )
		, m_levelCountdown( INITIAL_LEVEL_COUNTDOWN )
		, m_explosionTriggered( false )
	{
		// Réglez la gravité à 0.0f pour un gameplay 2D "à plat"
		b2World_SetGravity( m_worldId, { 0.0f, 0.0f } );

		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 10.0f;
		}

		// Centre d'explosion par défaut (sera ajusté lors du placement du carré spécial)
		m_explosionCenter = { 0.0f, 0.0f };

		// Création initiale de la grille de carrés
		GenerateGrid();
	}

	// Interface utilisateur (ImGui)
	void UpdateGui() override
	{
		ImGui::Begin( "Color Guesser" );

		// Contrôle de la taille de la grille (nombre de carrés par dimension)
		if ( ImGui::SliderInt( "Grid Size", &m_gridSize, 2, 100 ) )
		{
			GenerateGrid();
		}

		// Contrôle du niveau (impacte la difficulté et la différence de couleur)
		if ( ImGui::SliderInt( "Level", &m_level, 1, 25 ) )
		{
			GenerateGrid();
		}

		// Bouton pour déclencher manuellement l'explosion
		if ( ImGui::Button( "Explode Now" ) )
		{
			// Positionner l'explosion sur le carré spécial si valide
			if ( m_specialIndex >= 0 && m_specialIndex < static_cast<int>( m_squareBodies.size() ) )
			{
				m_explosionCenter = b2Body_GetPosition( m_squareBodies[m_specialIndex] );
			}
			Explode();
			m_explosionActive = true;
			m_explosionTimer = 30; // Durée d'affichage de l'explosion
			m_explosionTriggered = true;
		}

		// Contrôle du rayon et de la magnitude de l'explosion
		ImGui::SliderFloat( "Explosion Radius", &m_explosionRadius, 1.0f, 20.0f );
		ImGui::SliderFloat( "Explosion Magnitude", &m_explosionMagnitude, 1.0f, 50.0f );

		// Affichage des timers
		ImGui::Text( "Explosion in: %d frames", m_explosionCountdown );
		ImGui::Text( "Next level in: %d frames", m_levelCountdown );

		ImGui::End();
	}

	// Méthode Step, appelée chaque frame de la simulation
	// Méthode Step, appelée chaque frame de la simulation
	void Step() override
	{
		Sample::Step();

		if ( !m_context->pause )
		{
			// Décrémentation du compte à rebours pour le passage de niveau
			if ( m_levelCountdown > 0 )
			{
				m_levelCountdown--;
			}

			// Décrémentation du compte à rebours pour l'explosion (si elle n'a pas déjà été déclenchée)
			if ( !m_explosionTriggered && m_explosionCountdown > 0 )
			{
				m_explosionCountdown--;
			}

			// Explosion automatique lorsque le compte à rebours arrive à 0 (si elle n'a pas encore eu lieu)
			if ( m_explosionCountdown <= 0 && !m_explosionTriggered )
			{
				if ( m_specialIndex >= 0 && m_specialIndex < static_cast<int>( m_squareBodies.size() ) )
				{
					m_explosionCenter = b2Body_GetPosition( m_squareBodies[m_specialIndex] );
				}
				Explode();
				m_explosionActive = true;
				m_explosionTimer = 100;
				m_explosionTriggered = true;
			}

			// Passage au niveau suivant lorsque le compte à rebours est écoulé
			if ( m_levelCountdown <= 0 )
			{
				m_level++;
				m_gridSize++;
				// On éloigne la caméra pour garder une vision sur la grille qui grandit
				m_context->camera.m_zoom += 2.5f;

				GenerateGrid();

				// Réinitialisation des compteurs
				m_levelCountdown = INITIAL_LEVEL_COUNTDOWN;
				m_explosionCountdown = INITIAL_EXPLOSION_COUNTDOWN;
				m_explosionTriggered = false;
			}
		}

		// Gestion de l'affichage de l'explosion (cercle rouge temporaire)
		if ( m_explosionActive )
		{
			m_context->draw.DrawCircle( m_explosionCenter, m_explosionRadius, b2_colorRed );
			m_explosionTimer--;
			if ( m_explosionTimer <= 0 )
			{
				m_explosionActive = false;
			}
		}
	}

private:
	//--------------------------------------------------------------------------
	// [C] CONSTANTES DE TIMER
	//--------------------------------------------------------------------------
	static constexpr int INITIAL_LEVEL_COUNTDOWN = 200;		// ~200 frames pour le passage de niveau
	static constexpr int INITIAL_EXPLOSION_COUNTDOWN = 100; // ~100 frames avant l'explosion auto

	//--------------------------------------------------------------------------
	// [D] VARIABLES MEMBRES
	//--------------------------------------------------------------------------
	int m_gridSize;						  // Nombre de carrés par ligne/colonne
	int m_level;						  // Niveau courant (impacte notamment la couleur)
	std::vector<b2BodyId> m_squareBodies; // Conteneur des corps (carrés) créés

	int m_specialIndex;		   // Index du carré "spécial" (couleur différente)
	b2HexColor m_baseColor;	   // Couleur de base de la grille
	b2HexColor m_specialColor; // Couleur spécifique du carré spécial

	// Paramètres d'explosion
	bool m_explosionActive;		// Indique si l'explosion est en cours d'affichage
	int m_explosionTimer;		// Durée (en frames) de l'explosion visuelle
	float m_explosionRadius;	// Rayon de l'explosion
	float m_explosionMagnitude; // Force de l'impulsion par unité de distance
	b2Vec2 m_explosionCenter;	// Coordonnées de l'explosion

	// Timers additionnels
	int m_explosionCountdown;  // Temps restant avant l'explosion automatique
	int m_levelCountdown;	   // Temps restant avant le passage au niveau suivant
	bool m_explosionTriggered; // Indique si l'explosion de ce cycle a été déclenchée

	//--------------------------------------------------------------------------
	// [E] MÉTHODES PRIVÉES
	//--------------------------------------------------------------------------

	// Génère ou régénère la grille de carrés
	void GenerateGrid()
	{
		// Nettoyer les corps existants
		for ( b2BodyId body : m_squareBodies )
		{
			if ( B2_IS_NON_NULL( body ) )
			{
				b2DestroyBody( body );
			}
		}
		m_squareBodies.clear();

		const float squareSize = 2.0f;
		const float offset = ( m_gridSize - 1 ) * squareSize * 0.5f;

		// Calcul des couleurs
		m_baseColor = GenerateBaseColor( m_level );
		// La différence de couleur diminue à mesure que le niveau augmente
		const float lightenAmount = std::max( 0.05f, 0.2f - 0.002f * ( m_level - 1 ) );
		m_specialColor = LightenColor( m_baseColor, lightenAmount );

		const int totalSquares = m_gridSize * m_gridSize;
		m_specialIndex = GetRandomInt( 0, totalSquares - 1 );

		// Création de la grille
		m_squareBodies.reserve( static_cast<size_t>( totalSquares ) );
		for ( int i = 0; i < m_gridSize; ++i )
		{
			for ( int j = 0; j < m_gridSize; ++j )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				const int index = i * m_gridSize + j;

				// Désigne le carré spécial comme statique pour qu'il soit plus facilement repérable
				bodyDef.type = ( index == m_specialIndex ) ? b2_staticBody : b2_dynamicBody;

				// Position du carré dans la grille
				bodyDef.position = { j * squareSize - offset, i * squareSize - offset };

				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

				b2ShapeDef shapeDef = b2DefaultShapeDef();
				shapeDef.density = 1.0f;
				shapeDef.material = b2DefaultSurfaceMaterial();
				shapeDef.material.friction = 0.2f;
				shapeDef.material.restitution = 0.0f;
				shapeDef.material.customColor = ( index == m_specialIndex ) ? m_specialColor : m_baseColor;

				// Création d'un carré polygonal
				const float halfSize = squareSize * 0.5f;
				b2Polygon square = b2MakeBox( halfSize, halfSize );
				b2CreatePolygonShape( bodyId, &shapeDef, &square );

				m_squareBodies.push_back( bodyId );
			}
		}

		// Réinitialise la position de l'explosion au centre
		m_explosionCenter = { 0.0f, 0.0f };
	}

	// Déclenche l'explosion "physique" sur les corps
	void Explode()
	{
		b2ExplosionDef def = b2DefaultExplosionDef();
		def.position = m_explosionCenter;
		def.radius = m_explosionRadius;
		def.falloff = 0.1f; // Facteur d'atténuation
		def.impulsePerLength = m_explosionMagnitude;
		b2World_Explode( m_worldId, &def );
	}

	// Génère la couleur de base en fonction du niveau, de façon cyclique
	b2HexColor GenerateBaseColor( int level )
	{
		static const std::vector<std::string> colors = { "#7FFF00", "#ADFF2F", "#00FF7F", "#00FF00",
														 "#32CD32", "#00FA9A", "#8CC924" };

		// On choisit de manière cyclique parmi la liste
		const int index = ( level - 1 ) % static_cast<int>( colors.size() );
		return HexToB2Color( colors[static_cast<size_t>( index )] );
	}

	// Retourne une version éclaircie d'une couleur
	// amount ∈ [0,1]
	b2HexColor LightenColor( b2HexColor color, float amount )
	{
		uint32_t c = static_cast<uint32_t>( color );
		uint8_t r = ( c >> 16 ) & 0xFF;
		uint8_t g = ( c >> 8 ) & 0xFF;
		uint8_t b = c & 0xFF;

		// Calcule les nouvelles composantes (r,g,b) éclaircies
		r = static_cast<uint8_t>( std::min( 255, int( r + 255 * amount ) ) );
		g = static_cast<uint8_t>( std::min( 255, int( g + 255 * amount ) ) );
		b = static_cast<uint8_t>( std::min( 255, int( b + 255 * amount ) ) );

		uint32_t newColor = ( r << 16 ) | ( g << 8 ) | b;
		return static_cast<b2HexColor>( newColor );
	}

	// Convertit une couleur hexadécimale (e.g. "#FF5733") en b2HexColor
	b2HexColor HexToB2Color( const std::string& hex )
	{
		const std::string cleanHex = ( hex[0] == '#' ) ? hex.substr( 1 ) : hex;
		// Conversion hexa -> entier (base 16)
		return static_cast<b2HexColor>( std::stoul( cleanHex, nullptr, 16 ) );
	}

	// Génère un nombre entier dans l'intervalle [minVal, maxVal] de manière plus moderne que rand()
	int GetRandomInt( int minVal, int maxVal )
	{
		static std::random_device rd;
		static std::mt19937 gen( rd() );
		std::uniform_int_distribution<int> dist( minVal, maxVal );
		return dist( gen );
	}
};

static int colorGuesser = RegisterSample( "9:16", "Color Guesser", ColorGuesser::Create );

class Restitution : public Sample
{
public:
	enum ShapeType
	{
		e_circleShape = 0,
		e_boxShape
	};

	explicit Restitution( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 4.0f, 17.0f };
			m_context->camera.m_zoom = 27.5f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			float h = 1.0f * m_count;
			b2Segment segment = { { -h, 0.0f }, { h, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		m_shapeType = e_circleShape;

		CreateBodies();
	}

	void CreateBodies()
	{
		// Détruire proprement les anciens bodies
		for ( int i = 0; i < m_count; ++i )
		{
			if ( B2_IS_NON_NULL( m_bodyIds[i] ) )
			{
				b2DestroyBody( m_bodyIds[i] );
				m_bodyIds[i] = b2_nullBodyId;
			}
		}

		// Paramètres des shapes
		b2Circle circle = {};
		circle.radius = 0.5f;

		b2Polygon box = b2MakeBox( 0.5f, 0.5f );

		float startY = 40.0f; // Altitude de départ
		float dy = 0.1f;	  // Décalage vertical = 0.1 unité (10 px)
		float x = -1.0f * ( m_count - 1 );
		float dx = 2.0f; // Décalage horizontal

		// Restitution progressive si tu veux voir la différence (facultatif)
		float minRest = 0.1f, maxRest = 1.0f;
		float dr = ( m_count > 1 ) ? ( maxRest - minRest ) / ( m_count - 1 ) : 0.0f;

		for ( int i = 0; i < m_count; ++i )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef(); // Réinit à chaque body
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { x, startY + i * dy }; // Décalage Y progressif
			bodyDef.linearVelocity = { 0.0f, 0.0f };
			bodyDef.angularVelocity = 0.0f;
			bodyDef.angularDamping = 0.5f;

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			m_bodyIds[i] = bodyId;

			b2ShapeDef shapeDef = b2DefaultShapeDef(); // Réinit à chaque shape
			shapeDef.density = 1.0f;
			shapeDef.material = b2DefaultSurfaceMaterial();
			shapeDef.material.restitution = minRest + dr * i; // Restitution évolutive
			shapeDef.material.friction = 0.0f;
			shapeDef.enableHitEvents = true;

			if ( m_shapeType == e_circleShape )
			{
				b2CreateCircleShape( bodyId, &shapeDef, &circle );
			}
			else
			{
				b2CreatePolygonShape( bodyId, &shapeDef, &box );
			}

			x += dx;
		}
	}

	void UpdateGui() override
	{
		float height = 100.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Restitution", nullptr, ImGuiWindowFlags_NoResize );

		bool changed = false;
		const char* shapeTypes[] = { "Circle", "Box" };

		int shapeType = int( m_shapeType );
		changed = changed || ImGui::Combo( "Shape", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) );
		m_shapeType = ShapeType( shapeType );

		changed = changed || ImGui::Button( "Reset" );

		if ( changed )
		{
			CreateBodies();
		}

		ImGui::End();
	}

	static Sample* Create( SampleContext* context )
	{
		return new Restitution( context );
	}

	static constexpr int m_count = 40;

	b2BodyId m_bodyIds[m_count] = {};
	ShapeType m_shapeType;
};

static int sampleIndex = RegisterSample( "9:16", "Restitution", Restitution::Create );

class OverlapRecoverySample : public Sample
{
public:
	// Types
	enum class HeavyShapeType
	{
		Triangle = 3,
		Quadrilateral,
		Pentagon,
		Hexagon,
		Heptagon,
		Octagon
	};

	// Création du sample
	static Sample* Create( SampleContext* context )
	{
		return new OverlapRecoverySample( context );
	}

	// Constructeur
	explicit OverlapRecoverySample( SampleContext* context )
		: Sample( context )
		, m_baseCount( 10 )
		, m_overlap( 0.0f )
		, m_extent( 0.5f )
		, m_pushout( 3.0f )
		, m_hertz( 120.0f )
		, m_dampingRatio( 10.0f )
		, m_restitution( 0.0f )
		, m_friction( 0.5f )
		, m_groundId( b2_nullBodyId )
		, m_color{ 1.0f, 1.0f, 1.0f, 1.0f }
		, m_currentShape( HeavyShapeType::Hexagon )
		, m_dynamicShapeType( HeavyShapeType::Quadrilateral )
		, m_heavyDensity( 5.0f )
		, m_heavyBodyId( b2_nullBodyId )
	{
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 15.0f;
		}
		CreateScene();
		CreateHeavyObject();
	}

	// Scène et création des objets
	void CreateScene()
	{
		// Sol
		if ( B2_IS_NON_NULL( m_groundId ) )
		{
			b2DestroyBody( m_groundId );
			m_groundId = b2_nullBodyId;
		}
		for ( auto id : m_bodyIds )
		{
			if ( B2_IS_NON_NULL( id ) )
				b2DestroyBody( id );
		}
		m_bodyIds.clear();

		b2World_SetContactTuning( m_worldId, m_hertz, m_dampingRatio, m_pushout );

		b2BodyDef groundDef = b2DefaultBodyDef();
		m_groundId = b2CreateBody( m_worldId, &groundDef );

		b2ShapeDef groundSd = b2DefaultShapeDef();
		groundSd.material = b2DefaultSurfaceMaterial();
		groundSd.material.friction = m_friction;
		groundSd.material.restitution = m_restitution;
		groundSd.material.customColor = ImVec4ToARGB( m_color );

		b2Polygon ground = b2MakeOffsetBox( 8.4f, 1.0f, { 0.0f, -12.0f }, b2MakeRot( 0 ) );
		b2CreatePolygonShape( m_groundId, &groundSd, &ground );

		// Pyramide
		float dx = 2 * ( 1.0f - m_overlap ) * m_extent;
		float dy = 2 * m_extent;
		float y0 = -12.0f + 1.0f + m_extent;
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		b2ShapeDef sd = b2DefaultShapeDef();
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.friction = m_friction;
		sd.material.restitution = m_restitution;
		sd.density = 1.0f;
		sd.material.customColor = ImVec4ToARGB( m_color );

		for ( int row = 0; row < m_baseCount; ++row )
		{
			int cnt = m_baseCount - row;
			float y = y0 + row * dy;
			float x0 = -dx * ( cnt - 1 ) * 0.5f;
			for ( int i = 0; i < cnt; ++i )
			{
				bd.position = { x0 + i * dx, y };
				auto bodyId = b2CreateBody( m_worldId, &bd );
				b2Polygon poly = CreatePolygonFromShapeType( m_dynamicShapeType, m_extent );
				b2CreatePolygonShape( bodyId, &sd, &poly );
				m_bodyIds.push_back( bodyId );
			}
		}
	}

	void CreateHeavyObject()
	{
		if ( B2_IS_NON_NULL( m_heavyBodyId ) )
		{
			b2DestroyBody( m_heavyBodyId );
			m_heavyBodyId = b2_nullBodyId;
		}
		b2SurfaceMaterial mat = b2DefaultSurfaceMaterial();
		mat.friction = m_friction;
		mat.restitution = m_restitution;
		mat.customColor = ImVec4ToARGB( m_color );

		b2Polygon poly = CreatePolygonFromShapeType( m_currentShape, 2.0f );
		b2ShapeDef sd = b2DefaultShapeDef();
		sd.material = mat;
		sd.density = m_heavyDensity;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = { 0.0f, 20.0f };
		bd.isBullet = false;

		m_heavyBodyId = b2CreateBody( m_worldId, &bd );
		b2CreatePolygonShape( m_heavyBodyId, &sd, &poly );
	}

	// Utilities
	b2Polygon CreatePolygonFromShapeType( HeavyShapeType type, float r )
	{
		int sides = static_cast<int>( type );
		if ( sides == 4 )
			return b2MakeBox( r, r );
		b2Vec2 pts[8];
		float step = 2 * b2_pi / sides;
		for ( int i = 0; i < sides; ++i )
			pts[i] = { r * cosf( i * step ), r * sinf( i * step ) };
		auto hull = b2ComputeHull( pts, sides );
		return b2MakePolygon( &hull, 0.0f );
	}

	uint32_t ImVec4ToARGB( const ImVec4& c )
	{
		uint32_t cr = uint32_t( c.x * 255 ) & 0xFF;
		uint32_t cg = uint32_t( c.y * 255 ) & 0xFF;
		uint32_t cb = uint32_t( c.z * 255 ) & 0xFF;
		uint32_t ca = uint32_t( c.w * 255 ) & 0xFF;
		return ( ca << 24 ) | ( cr << 16 ) | ( cg << 8 ) | cb;
	}

	// GUI
	void UpdateGui() override
	{
		ImGui::Begin( "Overlap Recovery Controls" );
		bool rebuild = false;
		rebuild |= ImGui::SliderFloat( "Extent", &m_extent, 0.1f, 1.0f, "%.2f" );
		rebuild |= ImGui::SliderInt( "Base Count", &m_baseCount, 1, 25 );
		rebuild |= ImGui::SliderFloat( "Overlap", &m_overlap, 0.0f, 1.0f, "%.2f" );
		rebuild |= ImGui::SliderFloat( "Pushout", &m_pushout, 0.0f, 10.0f, "%.1f" );
		rebuild |= ImGui::SliderFloat( "Hertz", &m_hertz, 1.0f, 120.0f, "%.1f" );
		rebuild |= ImGui::SliderFloat( "Damping Ratio", &m_dampingRatio, 0.1f, 20.0f, "%.1f" );

		const char* names[] = { "Triangle", "Quadrilateral", "Pentagon", "Hexagon", "Heptagon", "Octagon" };
		int idx = int( m_dynamicShapeType ) - 3;
		if ( ImGui::Combo( "Dynamic Shape", &idx, names, IM_ARRAYSIZE( names ) ) )
		{
			m_dynamicShapeType = HeavyShapeType( idx + 3 );
			rebuild = true;
		}
		idx = int( m_currentShape ) - 3;
		if ( ImGui::Combo( "Heavy Shape", &idx, names, IM_ARRAYSIZE( names ) ) )
		{
			m_currentShape = HeavyShapeType( idx + 3 );
			CreateHeavyObject();
		}
		rebuild |= ImGui::SliderFloat( "Heavy Density", &m_heavyDensity, 0.1f, 100.0f, "%.1f" );
		if ( rebuild )
		{
			CreateScene();
			CreateHeavyObject();
		}
		ImGui::End();
	}

private:
	std::vector<b2BodyId> m_bodyIds;
	int m_baseCount;
	float m_overlap, m_extent;
	float m_pushout, m_hertz, m_dampingRatio;
	float m_restitution, m_friction;
	b2BodyId m_groundId;
	ImVec4 m_color;
	HeavyShapeType m_currentShape;
	HeavyShapeType m_dynamicShapeType;
	float m_heavyDensity;
	b2BodyId m_heavyBodyId;
};

static int overlapRecoverySampleIndex = RegisterSample( "9:16", "Overlap Recovery", OverlapRecoverySample::Create );

class Sensordestruction : public Sample
{
public:
	struct ShapeUserData
	{
		bool shouldDestroyVisitors;
	};
	explicit Sensordestruction( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 25.0f;
		}

		m_activeSensor.shouldDestroyVisitors = true;

		// Création du body dynamique
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { 0.0f, 0.0f };
		bodyDef.linearVelocity = { 0.0f, -5.0f };
		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

		// Filtrage pour les collisions
		static const uint16_t CATEGORY_DYNAMIC = 0x0002;
		static const uint16_t CATEGORY_SENSOR = 0x0004;
		static const uint16_t CATEGORY_WALL = 0x0001;

		// Création de la forme solide (rebondissante)
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material = b2DefaultSurfaceMaterial();
		shapeDef.material.restitution = 1.0f; // Rebond élevé
		shapeDef.material.friction = 0.0f;
		shapeDef.material.customColor = 0xFF00FF;
		shapeDef.filter.categoryBits = CATEGORY_DYNAMIC;
		shapeDef.filter.maskBits = CATEGORY_WALL | CATEGORY_SENSOR; // Interagit avec les murs et les sensors

		b2Polygon solidBox = b2MakeBox( 2.5f, 2.5f );
		b2CreatePolygonShape( bodyId, &shapeDef, &solidBox );

		// Création de la "peau" sensor
		b2ShapeDef sensorDef = b2DefaultShapeDef();
		sensorDef.isSensor = true;
		sensorDef.enableSensorEvents = true;
		sensorDef.userData = &m_activeSensor;
		sensorDef.filter.categoryBits = CATEGORY_SENSOR;
		sensorDef.filter.maskBits = CATEGORY_DYNAMIC | CATEGORY_WALL; // Peut détecter les dynamiques et les murs
		sensorDef.material.customColor = 0xAAAAAA;					  // Couleur du sensor

		// Définition du sensor avec une légère extension pour éviter les erreurs de collision
		float padding = 0.1f;
		b2Polygon sensorBox = b2MakeBox( 2.5f + padding, 2.5f + padding );
		b2CreatePolygonShape( bodyId, &sensorDef, &sensorBox );

		// Les murs
		{
			// Paramètres de l'arène (9:16)
			float arenaHalfWidth = 14.0f;  // Largeur intermédiaire
			float arenaHalfHeight = 25.0f; // Hauteur intermédiaire

			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material = b2DefaultSurfaceMaterial();
			shapeDef.material.restitution = 1.0f;
			shapeDef.material.friction = 0.0f;
			shapeDef.material.customColor = 0xAAAAAA;

			// Mur du bas
			b2Segment bottomWall = { { -arenaHalfWidth, -arenaHalfHeight }, { arenaHalfWidth, -arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &bottomWall );

			// Mur du haut
			b2Segment topWall = { { -arenaHalfWidth, arenaHalfHeight }, { arenaHalfWidth, arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &topWall );

			// Mur gauche
			b2Segment leftWall = { { -arenaHalfWidth, -arenaHalfHeight }, { -arenaHalfWidth, arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &leftWall );

			// Mur droit
			b2Segment rightWall = { { arenaHalfWidth, -arenaHalfHeight }, { arenaHalfWidth, arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &rightWall );
		}
	}

	void CreateRow( float y )
	{
		float shift = 5.0f;
		float xCenter = 0.5f * shift * m_columnCount;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.gravityScale = 0.0f;
		bodyDef.linearVelocity = { 0.0f, -5.0f };

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.enableSensorEvents = true;

		b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
		for ( int i = 0; i < m_columnCount; ++i )
		{
			float x = -0.5f * ( m_columnCount - 1 ) * shift + i * shift;
			bodyDef.position = { x, 20.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
	}

	void Step() override
	{
		Sample::Step();

		if ( m_stepCount == m_lastStepCount )
		{
			return;
		}

		std::set<b2BodyId> zombies;

		b2SensorEvents events = b2World_GetSensorEvents( m_worldId );
		for ( int i = 0; i < events.beginCount; ++i )
		{
			b2SensorBeginTouchEvent* event = events.beginEvents + i;

			ShapeUserData* userData = static_cast<ShapeUserData*>( b2Shape_GetUserData( event->sensorShapeId ) );
			if ( userData->shouldDestroyVisitors )
			{
				zombies.emplace( b2Shape_GetBody( event->visitorShapeId ) );
			}
			else
			{

				b2SurfaceMaterial surfaceMaterial = b2Shape_GetSurfaceMaterial( event->visitorShapeId );
				surfaceMaterial.customColor = b2_colorLime;
				b2Shape_SetSurfaceMaterial( event->visitorShapeId, surfaceMaterial );
			}
		}

		for ( int i = 0; i < events.endCount; ++i )
		{
			b2SensorEndTouchEvent* event = events.endEvents + i;

			if ( b2Shape_IsValid( event->visitorShapeId ) == false )
			{
				continue;
			}

			b2SurfaceMaterial surfaceMaterial = b2Shape_GetSurfaceMaterial( event->visitorShapeId );
			surfaceMaterial.customColor = 0;
			b2Shape_SetSurfaceMaterial( event->visitorShapeId, surfaceMaterial );
		}

		for ( b2BodyId bodyId : zombies )
		{
			b2DestroyBody( bodyId );
		}

		int delay = 0x0F;

		if ( ( m_stepCount & delay ) == 0 )
		{
			CreateRow( 0.5f * m_rowCount + 5.0f );
		}

		m_lastStepCount = m_stepCount;

		m_maxBeginCount = b2MaxInt( events.beginCount, m_maxBeginCount );
		m_maxEndCount = b2MaxInt( events.endCount, m_maxEndCount );
		DrawTextLine( "max begin touch events = %d", m_maxBeginCount );
		DrawTextLine( "max end touch events = %d", m_maxEndCount );
	}

	static Sample* Create( SampleContext* context )
	{
		return new Sensordestruction( context );
	}

	static constexpr int m_columnCount = 5;
	static constexpr int m_rowCount = 5;
	int m_maxBeginCount;
	int m_maxEndCount;
	ShapeUserData m_passiveSensor;
	ShapeUserData m_activeSensor;
	int m_lastStepCount;
};

static int Sensordestruction = RegisterSample( "9:16", "Sensor destruction", Sensordestruction::Create );

class PolygonGradientDynamicSample : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new PolygonGradientDynamicSample( context );
	}

	explicit PolygonGradientDynamicSample( SampleContext* context )
		: Sample( context )
	{
		b2World_SetGravity( m_worldId, { 0.0f, 0.0f } );

		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 10.0f;
		}

		CreateShapes();
	}

	~PolygonGradientDynamicSample() override
	{
		for ( int i = 0; i < NUM_SHAPES; ++i )
		{
			if ( b2Body_IsValid( m_bodyIds[i] ) )
			{
				b2DestroyBody( m_bodyIds[i] );
			}
		}
	}

private:
	void Step() override
	{
		Sample::Step();

		const float orbitSpeed = 2.0f; // Vitesse orbitale
		const float spinSpeed = 1.5f;  // Vitesse de rotation sur soi-même

		for ( int i = 0; i < NUM_SHAPES; ++i )
		{
			if ( b2Body_IsValid( m_bodyIds[i] ) )
			{
				b2Vec2 pos = b2Body_GetPosition( m_bodyIds[i] );

				if ( b2LengthSquared( pos ) > 0.0001f ) // 👈 seulement si pas collé au centre
				{
					// Calcul du vecteur tangent : rotation 90° (-y, x)
					b2Vec2 tangent = { -pos.y, pos.x };
					tangent = b2Normalize( tangent );

					// Appliquer la vitesse tangentielle (orbite)
					b2Vec2 velocity = orbitSpeed * tangent;
					b2Body_SetLinearVelocity( m_bodyIds[i], velocity );
				}
				else
				{
					// Objet au centre : aucune vitesse linéaire
					b2Body_SetLinearVelocity( m_bodyIds[i], { 0.0f, 0.0f } );
				}

				// Dans tous les cas : rotation sur soi-même
				b2Body_SetAngularVelocity( m_bodyIds[i], spinSpeed );
			}
		}
	}

	void CreateShapes()
	{
		const int numShapes = NUM_SHAPES;
		const float spacing = 4.0f;
		const float startX = -( ( numShapes - 1 ) * spacing ) * 0.5f;

		const uint32_t colors[NUM_SHAPES] = { 0xffff0099, 0xffec48cb, 0xffcc6ff1, 0xffa28cff,
											  0xff73a4ff, 0xff3db4ff, 0xff00c2ff };

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.restitution = 0.6f;
		sd.material.friction = 0.3f;

		for ( int i = 0; i < numShapes; ++i )
		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = { startX + i * spacing, 0.0f };
			bd.linearDamping = 0.0f;  // Pas de frein sur l'orbite
			bd.angularDamping = 0.5f; // Rotation fluide
			m_bodyIds[i] = b2CreateBody( m_worldId, &bd );

			// Couleur personnalisée
			sd.material.customColor = colors[i];

			// Forme : polygone ou cercle
			if ( i < numShapes - 1 )
			{
				int vcount = 3 + i;
				b2Vec2 verts[8];
				float radius = 1.0f;
				float angleOff = b2_pi / 2.0f;
				for ( int j = 0; j < vcount; ++j )
				{
					float a = angleOff + j * 2.0f * b2_pi / vcount;
					verts[j] = { radius * cosf( a ), radius * sinf( a ) };
				}
				b2Hull hull = b2ComputeHull( verts, vcount );
				if ( hull.count > 0 )
				{
					b2Polygon poly = b2MakePolygon( &hull, 0.0f );
					b2CreatePolygonShape( m_bodyIds[i], &sd, &poly );
				}
			}
			else
			{
				b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
				b2CreateCircleShape( m_bodyIds[i], &sd, &circle );
			}
		}
	}

	static constexpr int NUM_SHAPES = 7;
	b2BodyId m_bodyIds[NUM_SHAPES];
};

static int polygonGradientDynamicSample = RegisterSample( "9:16", "Polygon Gradient Dynamic (RevoluteMotor)", PolygonGradientDynamicSample::Create );

class MazeSample : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new MazeSample( context );
	}

	explicit MazeSample( SampleContext* context )
		: Sample( context )
		, m_shapeCount( 1 )
		, m_soundVolume( 30.0f )
	{
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 25.0f;
		}

		InitializeAudio(); // charge et règle le son
		CreateMaze();	   // construit le labyrinthe
		m_shapeType = e_boxShape;
		m_enableHitEvents = true;
		m_hitEvents.fill( HitEvent() );

		Launch(); // crée et lance les shapes dynamiques
	}

	~MazeSample() override
	{
		DestroyAllBodies();
	}

	void Step() override
	{
		Sample::Step();

		if ( m_enableHitEvents )
		{
			b2ContactEvents events = b2World_GetContactEvents( m_worldId );
			for ( int i = 0; i < events.hitCount; ++i )
			{
				auto* e = events.hitEvents + i;
				if ( m_stepCount - m_lastImpactStep < MIN_STEPS_BETWEEN_IMPACTS )
					continue;
				b2Vec2 delta = e->point - m_lastImpactPosition;
				if ( b2LengthSquared( delta ) < MIN_DISTANCE_BETWEEN_IMPACTS * MIN_DISTANCE_BETWEEN_IMPACTS )
					continue;
				HandleHitEffects( e->point, e->approachSpeed );
			}
		}

		for ( auto& h : m_hitEvents )
		{
			if ( h.stepIndex > 0 && m_stepCount <= h.stepIndex + 30 )
			{
				m_context->draw.DrawCircle( h.point, 0.1f, b2_colorOrangeRed );
				m_context->draw.DrawString( h.point, "%.1f", h.speed );
			}
		}

		ApplyShakeEffect();
		m_audioManager.PlayQueued();
	}

private:
	// Types et constantes
	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape
	};
	struct HitEvent
	{
		b2Vec2 point{ 0, 0 };
		float speed{ 0 };
		int stepIndex{ -1 };
	};

	static constexpr int DEFAULT_SHAKE_DURATION = 15;
	static constexpr float SHAKE_DAMPING = 0.85f;
	static constexpr float MIN_SHAKE_INTENSITY = 0.01f;
	static constexpr float RETURN_SPEED = 0.2f;
	static constexpr float MIN_DISTANCE_BETWEEN_IMPACTS = 0.05f;
	static constexpr int MIN_STEPS_BETWEEN_IMPACTS = 2;

	// Membres simulation
	std::vector<b2BodyId> m_bodies;
	std::array<HitEvent, 4> m_hitEvents;
	ShapeType m_shapeType = e_boxShape;
	bool m_enableHitEvents = true;
	bool m_fixedRotation = false;
	bool m_enableScreenShake = true;
	int m_shapeCount;

	// Audio
	AudioManager m_audioManager;
	float m_soundVolume;
	b2Vec2 m_lastImpactPosition{ 0, 0 };
	int m_lastImpactStep{ -MIN_STEPS_BETWEEN_IMPACTS };

	// Shake
	int m_shakeDuration{ 0 };
	float m_shakeIntensity{ 0 };
	b2Vec2 m_cameraBase{ 0, 0 };

	// Utilitaire aléatoire
	static float getRandomFloat( float min, float max )
	{
		static std::mt19937 gen{ std::random_device{}() };
		std::uniform_real_distribution<float> dis( min, max );
		return dis( gen );
	}

	void InitializeAudio()
	{
		m_audioManager.LoadFromDirectory( "D:/Sound & Fx/audio/galvanize" );
		m_audioManager.SetVolume( m_soundVolume );
	}

	void DestroyAllBodies()
	{
		for ( auto id : m_bodies )
			if ( b2Body_IsValid( id ) )
				b2DestroyBody( id );
		m_bodies.clear();
	}

	void Launch()
	{
		DestroyAllBodies();
		for ( int i = 0; i < m_shapeCount; ++i )
		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.isBullet = true;
			bd.position = { getRandomFloat( -5, 5 ), getRandomFloat( -5, 5 ) };
			bd.linearVelocity = { getRandomFloat( -10, 10 ), getRandomFloat( -10, 10 ) };
			bd.gravityScale = 0.0f;
			bd.motionLocks.angularZ = m_fixedRotation;

			b2BodyId body = b2CreateBody( m_worldId, &bd );
			m_bodies.push_back( body );

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.material = b2DefaultSurfaceMaterial();
			sd.density = 1.0f;
			sd.material.restitution = 1.0f;
			sd.material.friction = 0.0f;
			sd.enableHitEvents = m_enableHitEvents;
			static const uint32_t cols[3] = { b2_colorCrimson, b2_colorDodgerBlue, b2_colorLimeGreen };
			sd.material.customColor = cols[i % 3];

			if ( m_shapeType == e_circleShape )
			{
				b2Circle c{ { 0, 0 }, 0.325f };
				b2CreateCircleShape( body, &sd, &c );
			}
			else if ( m_shapeType == e_capsuleShape )
			{
				b2Capsule cap{ { -0.325f, 0 }, { 0.325f, 0 }, 0.1625f };
				b2CreateCapsuleShape( body, &sd, &cap );
			}
			else
			{
				b2Polygon box = b2MakeBox( 0.325f, 0.325f );
				b2CreatePolygonShape( body, &sd, &box );
			}
		}
	}

	void CreateMaze()
	{
		b2BodyDef bd = b2DefaultBodyDef();
		b2BodyId mazeId = b2CreateBody( m_worldId, &bd );

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.restitution = 0.0f;
		sd.material.friction = 0.0f;
		sd.material.customColor = b2_colorLimeGreen;

		float thickness = 0.1f;
		std::vector<std::pair<b2Vec2, b2Vec2>> walls = {
			{ { -8.78f, -15.60f }, { -0.98f, -15.60f } }, { { 0.98f, -15.60f }, { 8.78f, -15.60f } },
			{ { -8.78f, -13.65f }, { -6.83f, -13.65f } }, { { -4.88f, -13.65f }, { -2.93f, -13.65f } },
			{ { -0.98f, -13.65f }, { 0.98f, -13.65f } },  { { 2.93f, -13.65f }, { 6.83f, -13.65f } },
			{ { -6.83f, -11.70f }, { -4.88f, -11.70f } }, { { -2.93f, -11.70f }, { -0.98f, -11.70f } },
			{ { 2.93f, -11.70f }, { 4.88f, -11.70f } },	  { { 6.83f, -11.70f }, { 8.78f, -11.70f } },
			{ { -4.88f, -9.75f }, { -2.93f, -9.75f } },	  { { 4.88f, -9.75f }, { 6.83f, -9.75f } },
			{ { -6.83f, -7.80f }, { -2.93f, -7.80f } },	  { { 0.98f, -7.80f }, { 2.93f, -7.80f } },
			{ { -6.83f, -5.85f }, { -2.93f, -5.85f } },	  { { 4.88f, -5.85f }, { 6.83f, -5.85f } },
			{ { -8.78f, -3.90f }, { -6.83f, -3.90f } },	  { { -2.93f, -3.90f }, { 0.98f, -3.90f } },
			{ { 4.88f, -3.90f }, { 8.78f, -3.90f } },	  { { -6.83f, -1.95f }, { -0.98f, -1.95f } },
			{ { 2.93f, -1.95f }, { 4.88f, -1.95f } },	  { { -6.83f, 0.00f }, { -4.88f, 0.00f } },
			{ { -2.93f, 0.00f }, { 6.83f, 0.00f } },	  { { -8.78f, 1.95f }, { -2.93f, 1.95f } },
			{ { 0.98f, 1.95f }, { 4.88f, 1.95f } },		  { { -4.88f, 3.90f }, { -2.93f, 3.90f } },
			{ { -0.98f, 3.90f }, { 0.98f, 3.90f } },	  { { -2.93f, 5.85f }, { -0.98f, 5.85f } },
			{ { 6.83f, 5.85f }, { 8.78f, 5.85f } },		  { { -2.93f, 7.80f }, { 0.98f, 7.80f } },
			{ { 2.93f, 7.80f }, { 6.83f, 7.80f } },		  { { -2.93f, 9.75f }, { -0.98f, 9.75f } },
			{ { 6.83f, 9.75f }, { 8.78f, 9.75f } },		  { { 4.88f, 11.70f }, { 6.83f, 11.70f } },
			{ { -0.98f, 13.65f }, { 0.98f, 13.65f } },	  { { 4.88f, 13.65f }, { 8.78f, 13.65f } },
			{ { -8.78f, 15.60f }, { -0.98f, 15.60f } },	  { { 0.98f, 15.60f }, { 8.78f, 15.60f } },
			{ { -8.78f, -15.60f }, { -8.78f, 15.60f } },  { { -6.83f, -9.75f }, { -6.83f, -7.80f } },
			{ { -6.83f, -1.95f }, { -6.83f, 0.00f } },	  { { -6.83f, 3.90f }, { -6.83f, 15.60f } },
			{ { -4.88f, -15.60f }, { -4.88f, -9.75f } },  { { -4.88f, -5.85f }, { -4.88f, -1.95f } },
			{ { -4.88f, 3.90f }, { -4.88f, 13.65f } },	  { { -2.93f, -9.75f }, { -2.93f, -7.80f } },
			{ { -2.93f, -1.95f }, { -2.93f, 0.00f } },	  { { -2.93f, 1.95f }, { -2.93f, 3.90f } },
			{ { -2.93f, 7.80f }, { -2.93f, 13.65f } },	  { { -0.98f, -13.65f }, { -0.98f, -5.85f } },
			{ { -0.98f, 0.00f }, { -0.98f, 5.85f } },	  { { -0.98f, 11.70f }, { -0.98f, 15.60f } },
			{ { 0.98f, -13.65f }, { 0.98f, 0.00f } },	  { { 0.98f, 3.90f }, { 0.98f, 13.65f } },
			{ { 2.93f, -13.65f }, { 2.93f, -11.70f } },	  { { 2.93f, -9.75f }, { 2.93f, -7.80f } },
			{ { 2.93f, -5.85f }, { 2.93f, -1.95f } },	  { { 2.93f, 1.95f }, { 2.93f, 15.60f } },
			{ { 4.88f, -11.70f }, { 4.88f, -1.95f } },	  { { 4.88f, 1.95f }, { 4.88f, 5.85f } },
			{ { 4.88f, 7.80f }, { 4.88f, 11.70f } },	  { { 6.83f, -9.75f }, { 6.83f, -7.80f } },
			{ { 6.83f, -1.95f }, { 6.83f, 1.95f } },	  { { 6.83f, 3.90f }, { 6.83f, 5.85f } },
			{ { 8.78f, -15.60f }, { 8.78f, 15.60f } },
		};

		for ( auto& seg : walls )
		{
			auto p1 = seg.first, p2 = seg.second;
			float dx = p2.x - p1.x, dy = p2.y - p1.y;
			float len = std::hypot( dx, dy ), ang = std::atan2( dy, dx );
			b2Polygon wall = b2MakeOffsetBox( len * 0.5f, thickness * 0.5f, { ( p1.x + p2.x ) * 0.5f, ( p1.y + p2.y ) * 0.5f },
											  b2MakeRot( ang ) );
			b2CreatePolygonShape( mazeId, &sd, &wall );
		}
	}

	void HandleHitEffects( const b2Vec2& pt, float speed )
	{
		m_lastImpactPosition = pt;
		m_lastImpactStep = m_stepCount;

		auto* h = &m_hitEvents[0];
		for ( int i = 1; i < 4; ++i )
			if ( m_hitEvents[i].stepIndex < h->stepIndex )
				h = &m_hitEvents[i];
		h->point = pt;
		h->speed = speed;
		h->stepIndex = m_stepCount;

		m_audioManager.QueueSound( static_cast<size_t>( speed ) % m_audioManager.GetSoundCount() );

		if ( m_enableScreenShake )
		{
			m_shakeIntensity = 1.0f;
			m_shakeDuration = DEFAULT_SHAKE_DURATION;
			m_cameraBase = m_context->camera.m_center;
		}
	}

	void ApplyShakeEffect()
	{
		if ( m_shakeDuration > 0 )
		{
			float x = getRandomFloat( -0.5f, 0.5f ) * m_shakeIntensity;
			float y = getRandomFloat( -0.5f, 0.5f ) * m_shakeIntensity;
			m_context->camera.m_center = { m_cameraBase.x + x, m_cameraBase.y + y };
			--m_shakeDuration;
			m_shakeIntensity *= SHAKE_DAMPING;
		}
		else
		{
			m_context->camera.m_center.x += ( m_cameraBase.x - m_context->camera.m_center.x ) * RETURN_SPEED;
			m_context->camera.m_center.y += ( m_cameraBase.y - m_context->camera.m_center.y ) * RETURN_SPEED;
		}
	}

	// GUI Settings
	void UpdateGui() override
	{
		ImGui::Begin( "Maze Settings", nullptr, ImGuiWindowFlags_NoResize );
		// Shape type selection
		const char* shapeTypes[] = { "Circle", "Capsule", "Box" };
		int shapeType = static_cast<int>( m_shapeType );
		if ( ImGui::Combo( "Shape", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) ) )
		{
			m_shapeType = static_cast<ShapeType>( shapeType );
			Launch();
		}
		// Shape count slider
		if ( ImGui::SliderInt( "Shape Count", &m_shapeCount, 1, 20 ) )
		{
			Launch();
		}
		// Fixed rotation toggle
		if ( ImGui::Checkbox( "Fixed Rotation", &m_fixedRotation ) )
		{
			Launch();
		}
		// Hit events toggle
		if ( ImGui::Checkbox( "Hit Events", &m_enableHitEvents ) )
		{
			for ( auto body : m_bodies )
				if ( b2Body_IsValid( body ) )
					b2Body_EnableHitEvents( body, m_enableHitEvents );
		}
		// Screen shake toggle
		if ( ImGui::Checkbox( "Enable Screen Shake", &m_enableScreenShake ) )
		{
			if ( !m_enableScreenShake )
			{
				m_shakeDuration = 0;
				m_shakeIntensity = 0.0f;
				m_context->camera.m_center = m_cameraBase;
			}
		}
		// Sound volume
		if ( ImGui::SliderFloat( "Sound Volume", &m_soundVolume, 0.0f, 100.0f, "%.1f%%" ) )
		{
			m_audioManager.SetVolume( m_soundVolume );
		}
		// Display number of hit events recorded
		int hitCount = static_cast<int>(
			std::count_if( m_hitEvents.begin(), m_hitEvents.end(), []( auto& e ) { return e.stepIndex > 0; } ) );
		ImGui::Text( "Hit Count: %d", hitCount );
		ImGui::End();
	}
};

static int sampleMaze = RegisterSample( "9:16", "Maze Sample", MazeSample::Create );

class ImpactCrash : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new ImpactCrash( context );
	}

	explicit ImpactCrash( SampleContext* context )
		: Sample( context )
		, m_rng( std::random_device{}() )
		, m_impactSpeed( -50.0f )
		, m_impactSize( 1.0f )
	{
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 25.0f;
		}

		b2World_SetGravity( m_worldId, { 0.0f, 0.0f } );

		CreateHumansGrid();
		CreateImpactObject();
	}

	void UpdateGUI()
	{
		ImGui::Begin( "Contrôles Impact Crash" );

		bool anyChange = false;

		// --- Paramètres Impact (velocity, size) ---
		if ( ImGui::SliderFloat( "Vitesse d'Impact", &m_impactSpeed, -200.0f, -10.0f, "%.1f" ) )
		{
			anyChange = true;
		}
		if ( ImGui::SliderFloat( "Taille d'Impact", &m_impactSize, 0.5f, 5.0f, "%.1f" ) )
		{
			anyChange = true;
		}

		ImGui::Separator();

		// --- Paramètres de la grille ---
		if ( ImGui::InputInt( "Colonnes", &m_columns, 1, 5 ) )
		{
			anyChange = true;
			// éviter <1
			m_columns = std::max( 1, m_columns );
		}
		if ( ImGui::InputInt( "Rangées", &m_rows, 1, 5 ) )
		{
			anyChange = true;
			m_rows = std::max( 1, m_rows );
		}
		if ( ImGui::SliderFloat( "Espacement", &m_spacing, 0.5f, 5.0f, "%.2f" ) )
		{
			anyChange = true;
		}
		if ( ImGui::SliderFloat( "Masse Humain", &m_humanMass, 0.1f, 5.0f, "%.2f" ) )
		{
			anyChange = true;
		}
		if ( ImGui::SliderFloat( "Frottement Joint", &m_jointFriction, 0.0f, 2.0f, "%.2f" ) )
		{
			anyChange = true;
		}
		if ( ImGui::SliderFloat( "Fréquence Joint (Hz)", &m_jointHertz, 0.1f, 5.0f, "%.2f" ) )
		{
			anyChange = true;
		}
		if ( ImGui::SliderFloat( "Amortissement Joint", &m_jointDampingRatio, 0.0f, 1.0f, "%.2f" ) )
		{
			anyChange = true;
		}

		ImGui::Text( "Tout changement relance immédiatement la simulation." );

		ImGui::End();

		// Si n'importe quel paramètre a bougé, on full-reset
		if ( anyChange )
		{
			ResetSimulation();
		}
	}

private:
	static constexpr float IMPACT_OBJECT_DENSITY = 0.1f;
	static constexpr float IMPACT_OBJECT_RESTITUTION = 0.1f;
	static constexpr float IMPACT_OBJECT_FRICTION = 0.0f;
	static constexpr float VELOCITY_THRESHOLD = 15.0f;
	static constexpr float IMPULSE_MIN = -0.25f;
	static constexpr float IMPULSE_MAX = 0.25f;

	// Dans la section private :
	int m_columns = 10;
	int m_rows = 1;
	float m_spacing = 1.5f;
	float m_humanMass = 1.0f;
	float m_jointFriction = 0.3f;
	float m_jointHertz = 0.8f;
	float m_jointDampingRatio = 0.2f;

	std::mt19937 m_rng;
	std::vector<Human> m_humans;
	b2BodyId m_impactBody = b2_nullBodyId;
	float m_impactSpeed;
	float m_impactSize;

	float GetRandomFloat( float minVal, float maxVal )
	{
		std::uniform_real_distribution<float> dist( minVal, maxVal );
		return dist( m_rng );
	}

	// Détruit correctement tous les corps/joints associés aux humains
	void DestroyHumansGrid()
	{
		for ( Human& human : m_humans )
		{
			// Si disponible, détruire le corps et les joints associés
			// Exemple (à adapter) :
			DestroyHuman( &human );
		}
		m_humans.clear();
	}

	// Crée la grille des humains en commençant par détruire l'ancienne (si présente)
	void CreateHumansGrid()
	{
		// 1) Détruire l'ancienne grille
		DestroyHumansGrid();

		// 2) Réserver la bonne capacité
		m_humans.reserve( m_columns * m_rows );

		// 3) Calcul des bornes selon espacement
		const float startX = -( m_columns * m_spacing ) / 2.0f;
		const float startY = -( m_rows * m_spacing ) / 2.0f;

		// 4) Boucle en fonction de m_columns / m_rows
		for ( int i = 0; i < m_columns; ++i )
		{
			for ( int j = 0; j < m_rows; ++j )
			{
				b2Vec2 pos = { startX + i * m_spacing, startY + j * m_spacing };
				Human& h = m_humans.emplace_back();
				CreateHuman( &h, m_worldId, pos,
							 m_humanMass,	  // utilisons bien votre mass dynamique
							 m_jointFriction, // (et non 0.3f figé)
							 m_jointHertz, m_jointDampingRatio, i + j, nullptr, true );
			}
		}
	}

	// Crée ou recrée l'objet d'impact (le "gros carré")
	void CreateImpactObject()
	{
		if ( B2_IS_NON_NULL( m_impactBody ) )
		{
			b2DestroyBody( m_impactBody );
			m_impactBody = b2_nullBodyId;
		}

		// Création de la forme de l'impact
		b2Polygon impactBox = b2MakeBox( m_impactSize, m_impactSize );

		// Définition du corps dynamique
		b2BodyDef impactDef = b2DefaultBodyDef();
		impactDef.type = b2_dynamicBody;
		impactDef.position = { 30.0f, 0.0f };
		impactDef.linearVelocity = { m_impactSpeed, 0.0f };

		// Définition de la forme avec matériau
		b2ShapeDef impactShapeDef = b2DefaultShapeDef();
		impactShapeDef.density = IMPACT_OBJECT_DENSITY;
		impactShapeDef.material = b2DefaultSurfaceMaterial();
		impactShapeDef.material.restitution = IMPACT_OBJECT_RESTITUTION;
		impactShapeDef.material.friction = IMPACT_OBJECT_FRICTION;

		// Création du corps et de la forme
		m_impactBody = b2CreateBody( m_worldId, &impactDef );
		b2CreatePolygonShape( m_impactBody, &impactShapeDef, &impactBox );
	}

	void ResetImpactObject()
	{
		CreateImpactObject();
	}
	void ResetSimulation()
	{
		DestroyHumansGrid();
		CreateHumansGrid();
		CreateImpactObject();
	}

	void CheckForJointBreakage()
	{
		for ( Human& human : m_humans )
		{
			for ( int i = 0; i < bone_count; ++i )
			{
				Bone& bone = human.bones[i];

				if ( B2_IS_NULL( bone.jointId ) )
					continue;

				const b2BodyId bodyId = bone.bodyId;
				const b2Vec2 velocity = b2Body_GetLinearVelocity( bodyId );
				const float speed = b2Length( velocity );

				if ( speed > VELOCITY_THRESHOLD )
				{
					b2DestroyJoint( bone.jointId );
					bone.jointId = b2_nullJointId;

					b2Vec2 impulse = { GetRandomFloat( IMPULSE_MIN, IMPULSE_MAX ), GetRandomFloat( IMPULSE_MIN, IMPULSE_MAX ) };
					b2Body_ApplyLinearImpulse( bodyId, impulse, { 0.0f, 0.0f }, true );
				}
			}
		}
	}

	void Step() override
	{
		Sample::Step();
		UpdateGUI();
		CheckForJointBreakage();
	}
};

static int impactCrashSample = RegisterSample( "9:16", "Impact Crash", ImpactCrash::Create );

class FusionBounce : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new FusionBounce( context );
	}

	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape
	};
	enum EffectMode
	{
		e_Growth = 0,
		e_Shrink
	};

	struct HitEvent
	{
		b2Vec2 point = { 0.0f, 0.0f };
		float speed = 0.0f;
		int stepIndex = -1;
	};
	struct DynamicBody
	{
		b2BodyId bodyId;
		ShapeType shapeType;
	};

	explicit FusionBounce( SampleContext* context )
		: Sample( context )
		, m_enableHitEvents( true )
		, m_enableScreenShake( false )
		, m_enableGravity( true )
		, m_effectMode( e_Shrink )
		, m_numBodies( 3 )
		, m_volume( 40.0f )
		, m_lastImpactTime( -5 )
		, m_shakeDuration( 0 )
		, m_shakeIntensity( 0.0f )
	{
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 10.0f };
			m_context->camera.m_zoom = 25.0f;
		}
		m_originalCameraPosition = m_context->camera.m_center;
		for ( auto& e : m_hitEvents )
			e = HitEvent();

		// Initialisation AudioManager (charge tous les sons du dossier)
		m_audioManager.LoadFromDirectory( "D:/Sound & Fx/audio/Ticks" );
		m_audioManager.SetVolume( m_volume );

		CreateArena();
		LaunchDynamicBodies();
	}

	void UpdateGui() override
	{
		ImGuiIO& io = ImGui::GetIO();
		ImGui::SetNextWindowPos( ImVec2( io.DisplaySize.x * 0.2f, io.DisplaySize.y * 0.5f ), ImGuiCond_Always,
								 ImVec2( 0.5f, 0.5f ) );
		ImGui::SetNextWindowSize( ImVec2( 350, 0 ), ImGuiCond_Always );

		ImGui::Begin( "Fusion Bounce" );
		if ( ImGui::Button( "Launch Dynamic Bodies" ) )
			LaunchDynamicBodies();
		ImGui::Checkbox( "Enable Hit Events", &m_enableHitEvents );
		ImGui::Checkbox( "Enable Screen Shake", &m_enableScreenShake );
		ImGui::Checkbox( "Enable Gravity", &m_enableGravity );
		ImGui::Checkbox( "Show Impact Visuals", &m_showImpactVisuals );

		if ( ImGui::RadioButton( "Growth Effect", m_effectMode == e_Growth ) )
			m_effectMode = e_Growth;
		ImGui::SameLine();
		if ( ImGui::RadioButton( "Shrink Effect", m_effectMode == e_Shrink ) )
			m_effectMode = e_Shrink;
		ImGui::SliderInt( "Number of Bodies", &m_numBodies, 1, 10 );

		if ( ImGui::SliderFloat( "Volume", &m_volume, 0.0f, 100.0f, "%.1f%%" ) )
			m_audioManager.SetVolume( m_volume );

		ImGui::End();
	}

private:
	// --- Members ---
	std::vector<DynamicBody> m_dynamicBodies;
	std::array<HitEvent, 4> m_hitEvents;

	bool m_enableHitEvents;
	bool m_enableScreenShake;
	bool m_enableGravity;
	bool m_showImpactVisuals = true;
	EffectMode m_effectMode;
	int m_numBodies;

	// --- Audio ---
	AudioManager m_audioManager;

	// --- Misc ---
	int m_shakeDuration;
	float m_shakeIntensity;
	float m_volume;
	b2Vec2 m_originalCameraPosition;
	b2Vec2 m_lastImpactPosition;
	int m_lastImpactTime;

	// --- Utils ---
	static float getRandomFloat( float min, float max )
	{
		static std::random_device rd;
		static std::mt19937 gen( rd() );
		std::uniform_real_distribution<float> dist( min, max );
		return dist( gen );
	}

	// --- Arena Creation ---
	void CreateArena()
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_kinematicBody;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.customColor = b2_colorAntiqueWhite;
		shapeDef.enableHitEvents = true;
		bodyDef.angularVelocity = 0.5f;

		// Grand cercle gauche
		bodyDef.position = { -26.0f, -15.0f };
		b2BodyId leftCircleBody = b2CreateBody( m_worldId, &bodyDef );
		{
			b2Circle leftCircle = { { 0.0f, 0.0f }, 25.0f };
			b2CreateCircleShape( leftCircleBody, &shapeDef, &leftCircle );
		}
		// Grand cercle droit
		bodyDef.position = { 26.0f, -15.0f };
		b2BodyId rightCircleBody = b2CreateBody( m_worldId, &bodyDef );
		{
			b2Circle rightCircle = { { 0.0f, 0.0f }, 25.0f };
			b2CreateCircleShape( rightCircleBody, &shapeDef, &rightCircle );
		}
		// Segments (murs/toit)
		b2ShapeDef segmentShapeDef = b2DefaultShapeDef();
		segmentShapeDef.material.customColor = b2_colorAntiqueWhite;
		segmentShapeDef.enableHitEvents = true;
		b2BodyDef segmentBodyDef = b2DefaultBodyDef();
		segmentBodyDef.type = b2_staticBody;
		// Segment vertical gauche
		segmentBodyDef.position = { -14.1f, 22.0f };
		b2BodyId leftSegmentBody = b2CreateBody( m_worldId, &segmentBodyDef );
		{
			b2Segment leftSegment = { { 0.0f, -15.0f }, { 0.0f, 30.0f } };
			b2CreateSegmentShape( leftSegmentBody, &segmentShapeDef, &leftSegment );
		}
		// Segment vertical droit
		segmentBodyDef.position = { 14.1f, 22.0f };
		b2BodyId rightSegmentBody = b2CreateBody( m_worldId, &segmentBodyDef );
		{
			b2Segment rightSegment = { { 0.0f, -15.0f }, { 0.0f, 30.0f } };
			b2CreateSegmentShape( rightSegmentBody, &segmentShapeDef, &rightSegment );
		}
		// Segment horizontal (toit)
		segmentBodyDef.position = { 0.0f, 35.0f };
		b2BodyId topSegmentBody = b2CreateBody( m_worldId, &segmentBodyDef );
		{
			b2Segment topSegment = { { -14.1f, 0.0f }, { 14.1f, 0.0f } };
			b2CreateSegmentShape( topSegmentBody, &segmentShapeDef, &topSegment );
		}
	}

	// --- Dynamic Bodies Spawn ---
	void LaunchDynamicBodies()
	{
		for ( const DynamicBody& dynBody : m_dynamicBodies )
		{
			if ( B2_IS_NON_NULL( dynBody.bodyId ) )
				b2DestroyBody( dynBody.bodyId );
		}
		m_dynamicBodies.clear();

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.linearVelocity = { 0.0f, 20.0f };
		bodyDef.allowFastRotation = true;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.restitution = 0.99f;
		shapeDef.material.friction = 0.0f;
		shapeDef.enableHitEvents = true;

		float startX = -10.0f;
		const float spacing = 10.0f;
		for ( int i = 0; i < m_numBodies; ++i )
		{
			bodyDef.position = { startX, 10.0f };
			ShapeType shapeType = static_cast<ShapeType>( i % 3 );
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			if ( B2_IS_NULL( bodyId ) )
				continue;

			shapeDef.material.customColor = b2_colorAntiqueWhite;

			if ( shapeType == e_circleShape )
			{
				b2Circle circle = { { 0.0f, 0.0f }, 5.0f };
				b2CreateCircleShape( bodyId, &shapeDef, &circle );
			}
			else if ( shapeType == e_capsuleShape )
			{
				b2Capsule capsule = { { -2.5f, 0.0f }, { 2.5f, 0.0f }, 2.5f };
				b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			}
			else if ( shapeType == e_boxShape )
			{
				b2Polygon box = b2MakeBox( 5.0f, 5.0f );
				b2CreatePolygonShape( bodyId, &shapeDef, &box );
			}
			m_dynamicBodies.push_back( { bodyId, shapeType } );
			startX += spacing;
		}
	}

	// --- Collision Handling ---
	void HandleCollisions()
	{
		if ( !m_enableHitEvents )
			return;
		b2ContactEvents events = b2World_GetContactEvents( m_worldId );
		for ( int i = 0; i < events.hitCount; ++i )
		{
			b2ContactHitEvent* event = events.hitEvents + i;
			// Filtrage temporel et spatial
			if ( m_stepCount - m_lastImpactTime < 1 )
				continue;
			b2Vec2 delta = event->point - m_lastImpactPosition;
			if ( b2LengthSquared( delta ) < 0.05f * 0.05f )
				continue;
			for ( DynamicBody& dynBody : m_dynamicBodies )
			{
				b2BodyId bodyA = b2Shape_GetBody( event->shapeIdA );
				b2BodyId bodyB = b2Shape_GetBody( event->shapeIdB );
				if ( !B2_ID_EQUALS( dynBody.bodyId, bodyA ) && !B2_ID_EQUALS( dynBody.bodyId, bodyB ) )
					continue;

				// Resize effect
				b2ShapeId shapeIds[10];
				int count = b2Body_GetShapes( dynBody.bodyId, shapeIds, 10 );
				if ( count > 0 )
				{
					b2ShapeId shapeId = shapeIds[0];
					float factor = ( m_effectMode == e_Growth ) ? 1.01f : 0.99f;
					if ( dynBody.shapeType == e_circleShape )
					{
						b2Circle circle = b2Shape_GetCircle( shapeId );
						circle.radius *= factor;
						b2Shape_SetCircle( shapeId, &circle );
					}
					else if ( dynBody.shapeType == e_capsuleShape )
					{
						b2Capsule capsule = b2Shape_GetCapsule( shapeId );
						capsule.radius *= factor;
						capsule.center1.x *= factor;
						capsule.center2.x *= factor;
						b2Shape_SetCapsule( shapeId, &capsule );
					}
					else if ( dynBody.shapeType == e_boxShape )
					{
						b2Polygon box = b2Shape_GetPolygon( shapeId );
						for ( int j = 0; j < box.count; ++j )
						{
							box.vertices[j].x *= factor;
							box.vertices[j].y *= factor;
						}
						b2Shape_SetPolygon( shapeId, &box );
					}
					b2Body_ApplyMassFromShapes( dynBody.bodyId );
				}

				// Screen shake
				if ( m_enableScreenShake )
				{
					m_shakeDuration = std::min( 20, m_shakeDuration + 5 );
					m_shakeIntensity = std::min( 1.2f, m_shakeIntensity + event->approachSpeed * 0.04f );
				}

				// Gestion de l'audio et stockage HitEvent visuel (filtrage côté audio manager)
				m_audioManager.HandleHitEffect( event->point, event->approachSpeed, m_stepCount );

				m_lastImpactPosition = event->point;
				m_lastImpactTime = m_stepCount;

				// Stockage visuel (pour les cercles d'impact à l'écran)
				HitEvent* slot = &m_hitEvents[0];
				for ( int j = 1; j < 4; ++j )
				{
					if ( m_hitEvents[j].stepIndex < slot->stepIndex )
						slot = &m_hitEvents[j];
				}
				slot->point = event->point;
				slot->speed = event->approachSpeed;
				slot->stepIndex = m_stepCount;
			}
		}
	}

	// --- Camera Shake ---
	void UpdateCameraShake()
	{
		if ( !m_enableScreenShake )
			return;
		if ( m_shakeDuration > 0 )
		{
			float shakeX = getRandomFloat( -0.5f, 0.5f ) * m_shakeIntensity;
			float shakeY = getRandomFloat( -0.5f, 0.5f ) * m_shakeIntensity;
			m_context->camera.m_center = { m_originalCameraPosition.x + shakeX, m_originalCameraPosition.y + shakeY };
			m_shakeIntensity *= 0.92f;
			m_shakeDuration--;
		}
		else
		{
			m_context->camera.m_center = m_originalCameraPosition;
			m_shakeIntensity = 0.0f;
		}
	}

	// --- Draw Recent Hit Events ---
	void DrawRecentHitEvents()
	{
		if ( !m_showImpactVisuals )
			return; // <-- Ajout ici

		for ( const HitEvent& e : m_hitEvents )
		{
			if ( e.stepIndex > 0 && m_stepCount <= e.stepIndex + 30 )
			{
				m_context->draw.DrawCircle( e.point, 0.4f, b2_colorOrangeRed );
				m_context->draw.DrawString( e.point, "%.1f", e.speed );
			}
		}
	}

	// --- Step Function ---
	void Step() override
	{
		Sample::Step();
		b2Vec2 currentGravity = b2World_GetGravity( m_worldId );
		b2Vec2 desiredGravity = m_enableGravity ? b2Vec2{ 0.0f, -10.0f } : b2Vec2{ 0.0f, 0.0f };
		if ( currentGravity.x != desiredGravity.x || currentGravity.y != desiredGravity.y )
			b2World_SetGravity( m_worldId, desiredGravity );

		if ( m_enableHitEvents )
			HandleCollisions();
		UpdateCameraShake();
		DrawRecentHitEvents();

		// Appelle la lecture des sons de l'AudioManager à chaque frame
		m_audioManager.PlayQueued();
	}
};
static int sampleFusionBounce = RegisterSample( "9:16", "Fusion Bounce", FusionBounce::Create );

class BouncingMetamorphosis : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new BouncingMetamorphosis( context );
	}

	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape
	};

	enum class AspectRatio
	{
		Ratio_16_9 = 0,
		Ratio_9_16,
		Ratio_1_1
	};

	struct HitEvent
	{
		b2Vec2 point = { 0.0f, 0.0f };
		float speed = 0.0f;
		int stepIndex = -1;
	};

	explicit BouncingMetamorphosis( SampleContext* context )
		: Sample( context )
		, m_shapeType( e_boxShape )
		, m_enableHitEvents( true )
		, m_enableScreenShake( true )
		, m_wallBodyId( b2_nullBodyId )
		, m_bodyId( b2_nullBodyId )
		, m_aspectRatio( AspectRatio::Ratio_16_9 )
		, m_volume( 20.0f )
		, m_lastImpactTime( -5 )
		, m_shakeDuration( 0 )
		, m_shakeIntensity( 0.0f )
	{
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 25.0f;
		}

		for ( auto& e : m_hitEvents )
			e = HitEvent();

		m_audioManager.LoadFromDirectory( "D:/Sound & Fx/audio/zelda TP/rupee" );
		m_audioManager.SetVolume( m_volume );

		CreateGrowthBounce();
		CreateGridOfSmallObjects();

		Launch();
	}

	// --- UI ---
	void UpdateGui() override
	{
		ImGui::Begin( "Blueprint", nullptr, ImGuiWindowFlags_NoResize );

		// Forme
		const char* shapeTypes[] = { "Circle", "Capsule", "Box" };
		int shapeType = static_cast<int>( m_shapeType );
		if ( ImGui::Combo( "Shape", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) ) )
		{
			m_shapeType = static_cast<ShapeType>( shapeType );
			Launch();
		}

		if ( ImGui::Checkbox( "Hit Events", &m_enableHitEvents ) )
			b2Body_EnableHitEvents( m_bodyId, m_enableHitEvents );

		ImGui::Spacing();
		ImGui::Text( "Effects:" );
		ImGui::Checkbox( "Enable Screen Shake", &m_enableScreenShake );

		// Volume
		if ( ImGui::SliderFloat( "Volume", &m_volume, 0.0f, 100.0f, "%.1f%%" ) )
			m_audioManager.SetVolume( m_volume );

		// Aspect Ratio
		const char* aspectRatios[] = { "16:9", "9:16", "1:1" };
		int aspectIndex = static_cast<int>( m_aspectRatio );
		if ( ImGui::Combo( "Aspect Ratio", &aspectIndex, aspectRatios, IM_ARRAYSIZE( aspectRatios ) ) )
		{
			m_aspectRatio = static_cast<AspectRatio>( aspectIndex );
			CreateGrowthBounce();
		}

		ImGui::End();
	}

	void Step() override
	{
		Sample::Step();

		// Gestion des impacts et effets visuels/sonores
		b2ContactEvents events = b2World_GetContactEvents( m_worldId );
		for ( int i = 0; i < events.hitCount; ++i )
		{
			b2ContactHitEvent* event = events.hitEvents + i;

			if ( m_stepCount - m_lastImpactTime < MIN_STEPS_BETWEEN_IMPACTS )
				continue;
			b2Vec2 delta = event->point - m_lastImpactPosition;
			if ( b2LengthSquared( delta ) < MIN_DISTANCE_BETWEEN_IMPACTS * MIN_DISTANCE_BETWEEN_IMPACTS )
				continue;

			HandleHitEffects( event->point, event->approachSpeed );

			// Stockage pour les effets visuels
			HitEvent* e = &m_hitEvents[0];
			for ( int j = 1; j < 4; ++j )
			{
				if ( m_hitEvents[j].stepIndex < e->stepIndex )
					e = &m_hitEvents[j];
			}
			e->point = event->point;
			e->speed = event->approachSpeed;
			e->stepIndex = m_stepCount;
		}

		// Affichage des 4 derniers impacts
		for ( const HitEvent& e : m_hitEvents )
		{
			if ( e.stepIndex > 0 && m_stepCount <= e.stepIndex + 30 )
			{
				m_context->draw.DrawCircle( e.point, 0.1f, b2_colorOrangeRed );
				m_context->draw.DrawString( e.point, "%.1f", e.speed );
			}
		}

		ApplyShakeEffect();

		// Lecture des sons via AudioManager
		m_audioManager.PlayQueued();
	}

private:
	//----------------------------------------------------------------------
	// --- Rupee et objets décoratifs
	// (Identique à ton code d'origine, pas modifié pour audio)
	//----------------------------------------------------------------------
	static b2Vec2 IntersectSegments( const b2Vec2& A1, const b2Vec2& A2, const b2Vec2& B1, const b2Vec2& B2 )
	{
		float x1 = A1.x, y1 = A1.y;
		float x2 = A2.x, y2 = A2.y;
		float x3 = B1.x, y3 = B1.y;
		float x4 = B2.x, y4 = B2.y;

		float denom = ( y4 - y3 ) * ( x2 - x1 ) - ( x4 - x3 ) * ( y2 - y1 );
		if ( fabsf( denom ) < 1e-8f )
			return A1;

		float t = ( ( x4 - x3 ) * ( y1 - y3 ) - ( y4 - y3 ) * ( x1 - x3 ) ) / denom;
		float ix = x1 + t * ( x2 - x1 );
		float iy = y1 + t * ( y2 - y1 );

		return { ix, iy };
	}
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

	const std::vector<std::pair<uint32_t, uint32_t>> rupeeColorPairs = {
		{ 0xff00ff66, 0xff007733 }, { 0xff2277aa, 0xff114466 }, { 0xffffff66, 0xff999900 },
		{ 0xffff9933, 0xffcc5500 }, { 0xffcc3333, 0xff660000 }, { 0xff9955cc, 0xff552288 },
		{ 0xffcccccc, 0xff666666 }, { 0xffffd700, 0xffb8860b }, { 0xff222222, 0xff000000 } };

	b2BodyId CreateSmallRupeeWithColors( b2WorldId worldId, const b2Vec2& position, uint32_t colorCore, uint32_t colorFacet )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = position;
		bd.angularDamping = 0.2f;
		b2BodyId bodyId = b2CreateBody( worldId, &bd );

		b2Vec2 rupee[6] = { { 0.00f, 0.28f },  { 0.14f, 0.14f },   { 0.14f, -0.14f },
							{ 0.00f, -0.28f }, { -0.14f, -0.14f }, { -0.14f, 0.14f } };

		// Core
		{
			b2Hull coreHull = b2ComputeHull( rupee, 6 );
			if ( coreHull.count > 2 )
			{
				b2Polygon corePolygon = b2MakePolygon( &coreHull, 0.0f );
				b2ShapeDef centerSd = b2DefaultShapeDef();
				centerSd.density = 1.0f;
				centerSd.material.friction = 0.3f;
				centerSd.material.restitution = 0.2f;
				centerSd.material.customColor = colorCore;

				b2CreatePolygonShape( bodyId, &centerSd, &corePolygon );
			}
		}
		// Facets
		float offsetDist = 0.1f;
		b2Vec2 offsetPoly[6];
		ComputeOffsetPolygon( rupee, 6, offsetDist, offsetPoly );

		b2ShapeDef facetSd = b2DefaultShapeDef();
		facetSd.density = 1.0f;
		facetSd.material.friction = 0.3f;
		facetSd.material.restitution = 0.2f;
		facetSd.material.customColor = colorFacet;

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
		return bodyId;
	}
	void CreateGridOfSmallObjects()
	{
		const int rows = 25;
		const int cols = 25;
		const float spacing = 1.0f;

		float halfGridWidth = ( ( cols - 1 ) * spacing ) / 2.0f;
		float halfGridHeight = ( ( rows - 1 ) * spacing ) / 2.0f;
		float startX = -halfGridWidth;
		float startY = -halfGridHeight;

		for ( int i = 0; i < cols; ++i )
		{
			for ( int j = 0; j < rows; ++j )
			{
				b2Vec2 pos = { startX + i * spacing, startY + j * spacing };
				int patternIndex = ( i + j ) % rupeeColorPairs.size();
				const auto& colorPair = rupeeColorPairs[patternIndex];
				b2BodyId bodyId = CreateSmallRupeeWithColors( m_worldId, pos, colorPair.first, colorPair.second );
				b2Body_SetGravityScale( bodyId, 0.0f );
			}
		}
	}

	void CreateGrowthBounce()
	{
		if ( B2_IS_NON_NULL( m_wallBodyId ) )
		{
			b2DestroyBody( m_wallBodyId );
			m_wallBodyId = b2_nullBodyId;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		m_wallBodyId = b2CreateBody( m_worldId, &bodyDef );

		float width = 9.0f, height = 9.0f;
		switch ( m_aspectRatio )
		{
			case AspectRatio::Ratio_16_9:
				height = 9.0f;
				width = height * ( 16.0f / 9.0f );
				break;
			case AspectRatio::Ratio_9_16:
				width = 9.0f;
				height = width * ( 16.0f / 9.0f );
				break;
			case AspectRatio::Ratio_1_1:
			default:
				width = 9.0f;
				height = 9.0f;
				break;
		}
	}

	void Launch()
	{
		if ( B2_IS_NON_NULL( m_bodyId ) )
			b2DestroyBody( m_bodyId );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.linearVelocity = { -25.0f, 0.0f };
		bodyDef.position = { 50.0f, 0.0f };
		bodyDef.gravityScale = 0.0f;
		bodyDef.motionLocks = { false, false, true };

		m_bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.restitution = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.enableHitEvents = m_enableHitEvents;
		shapeDef.material.customColor = shapeColors[static_cast<int>( m_shapeType ) % shapeColors.size()];

		if ( m_shapeType == e_circleShape )
		{
			b2Circle circle = { { 0.0f, 0.0f }, 0.325f };
			b2CreateCircleShape( m_bodyId, &shapeDef, &circle );
		}
		else if ( m_shapeType == e_capsuleShape )
		{
			b2Capsule capsule = { { -0.350f, 0.0f }, { 0.350f, 0.0f }, 0.1625f };
			b2CreateCapsuleShape( m_bodyId, &shapeDef, &capsule );
		}
		else
		{
			float size = 0.40f;
			b2Polygon box = b2MakeBox( size, size );
			b2CreatePolygonShape( m_bodyId, &shapeDef, &box );
		}
	}

	void GrowBody()
	{
		if ( B2_IS_NON_NULL( m_bodyId ) )
		{
			b2ShapeId shapes[10];
			int count = b2Body_GetShapes( m_bodyId, shapes, 10 );
			if ( count > 0 )
			{
				b2ShapeId shapeId = shapes[0];
				if ( m_shapeType == e_circleShape )
				{
					b2Circle circle = b2Shape_GetCircle( shapeId );
					float currentRadius = circle.radius;
					const float initialRadius = 0.325f;
					float growthFactor = 1.0f + 0.05f * ( initialRadius / currentRadius );
					circle.radius *= growthFactor;
					b2Shape_SetCircle( shapeId, &circle );
				}
				else if ( m_shapeType == e_capsuleShape )
				{
					b2Capsule capsule = b2Shape_GetCapsule( shapeId );
					float currentRadius = capsule.radius;
					const float initialRadius = 0.1625f;
					float growthFactor = 1.0f + 0.05f * ( initialRadius / currentRadius );
					capsule.radius *= growthFactor;
					capsule.center1.x *= growthFactor;
					capsule.center2.x *= growthFactor;
					b2Shape_SetCapsule( shapeId, &capsule );
				}
				else
				{
					b2Polygon box = b2Shape_GetPolygon( shapeId );
					float currentSize = fabs( box.vertices[0].x );
					const float initialSize = 0.325f;
					float growthFactor = 1.0f + 0.05f * ( initialSize / currentSize );
					for ( int j = 0; j < box.count; ++j )
					{
						box.vertices[j].x *= growthFactor;
						box.vertices[j].y *= growthFactor;
					}
					b2Shape_SetPolygon( shapeId, &box );
				}
				b2Body_ApplyMassFromShapes( m_bodyId );
			}
		}
	}

	void HandleHitEffects( const b2Vec2& impactPoint, float speed )
	{
		m_cameraBasePosition = m_context->camera.m_center;
		m_lastImpactPosition = impactPoint;
		m_lastImpactTime = m_stepCount;

		// File d’attente de son via AudioManager
		m_audioManager.HandleHitEffect( impactPoint, speed, m_stepCount );

		// Grossissement
		GrowBody();

		// Screen shake
		if ( m_enableScreenShake )
		{
			m_shakeDuration = 15;
			m_shakeIntensity = 0.3f;
		}
	}

	void ApplyShakeEffect()
	{
		if ( m_shakeDuration > 0 )
		{
			float shakeX = ( static_cast<float>( rand() ) / RAND_MAX - 0.5f ) * m_shakeIntensity;
			float shakeY = ( static_cast<float>( rand() ) / RAND_MAX - 0.5f ) * m_shakeIntensity;
			m_context->camera.m_center.x = m_cameraBasePosition.x + shakeX;
			m_context->camera.m_center.y = m_cameraBasePosition.y + shakeY;
			m_shakeDuration--;
			m_shakeIntensity *= 0.9f;
		}
		else
		{
			float returnSpeed = 0.15f;
			m_context->camera.m_center.x += ( m_cameraBasePosition.x - m_context->camera.m_center.x ) * returnSpeed;
			m_context->camera.m_center.y += ( m_cameraBasePosition.y - m_context->camera.m_center.y ) * returnSpeed;
		}
	}

private:
	std::array<HitEvent, 4> m_hitEvents;
	b2BodyId m_bodyId;
	ShapeType m_shapeType;
	bool m_enableHitEvents;
	b2MotionLocks m_motionLocks;
	bool m_enableScreenShake;

	b2BodyId m_wallBodyId;
	AspectRatio m_aspectRatio;

	// AudioManager utilisé partout
	AudioManager m_audioManager;

	float m_volume;

	// Gestion impacts/écran
	b2Vec2 m_lastImpactPosition = { 0.0f, 0.0f };
	int m_lastImpactTime;
	static constexpr float MIN_DISTANCE_BETWEEN_IMPACTS = 0.05f;
	static constexpr int MIN_STEPS_BETWEEN_IMPACTS = 2;

	int m_shakeDuration;
	float m_shakeIntensity;
	b2Vec2 m_cameraBasePosition = { 0.0f, 0.0f };

	std::vector<uint32_t> shapeColors = {
		0x4B0082, // Indigo
		0xFFFFF0, // Ivory
		0xF0E68C  // Khaki
	};
};

static int BouncingMetamorphosisSample = RegisterSample( "9:16", "Bouncing Metamorphosis", BouncingMetamorphosis::Create );

class MetamorphicBounce : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new MetamorphicBounce( context );
	}

	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape
	};

	explicit MetamorphicBounce( SampleContext* context )
		: Sample( context )
		, m_shapeType( e_boxShape )
		, m_enableHitEvents( true )
		, m_soundVolume( 30.0f )
		, m_fixedRotation( false )
		, m_enableScreenShake( true )
		, spawnTimer( 0 )
		, spawnInterval( 10 )
		, shakeDuration( 0 )
		, shakeIntensity( 0.0f )
		, m_anchorId( b2_nullBodyId )
		, m_motorJointId( b2_nullJointId )
	{
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 25.0f;
		}
		InitializeAudio();
		Launch();
	}

	void Step() override
	{
		Sample::Step();

		if ( m_enableHitEvents )
		{
			b2ContactEvents events = b2World_GetContactEvents( m_worldId );
			for ( int i = 0; i < events.hitCount; ++i )
			{
				m_audioManager.HandleHitEffect( events.hitEvents[i].point, events.hitEvents[i].approachSpeed, m_stepCount );
				GrowBody();
				if ( m_enableScreenShake )
				{
					shakeDuration = 15;
					shakeIntensity = 0.3f;
				}
			}
		}

		m_audioManager.PlayQueued();
		m_audioManager.DrawHitEffects( &m_context->draw, m_stepCount );

		ApplyShakeEffect();

		// Animation du motor joint (tous les ShapeType)
		if ( B2_IS_NON_NULL( m_motorJointId ) )
		{
			float t = m_stepCount * 0.03f;
			b2Transform xf;
			xf.p = { 0.0f, 0.0f }; // cible statique
			xf.q = b2MakeRot( t ); // fait tourner la forme
			b2Joint_SetLocalFrameA( m_motorJointId, xf );
		}

		spawnTimer++;
		if ( spawnTimer >= spawnInterval )
		{
			spawnTimer = 0;
			SpawnBombardmentObject();
		}
	}

	void UpdateGui() override
	{
		ImGuiIO& io = ImGui::GetIO();
		ImVec2 center = ImVec2( io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f );

		ImGui::SetNextWindowPos( center, ImGuiCond_Always, ImVec2( 0.5f, 0.5f ) );
		ImGui::SetNextWindowSize( ImVec2( 400, 0 ), ImGuiCond_Always );

		ImGui::Begin( "MetamorphicBounce Settings" );

		const char* shapeNames[] = { "Circle", "Capsule", "Box" };
		int shapeIndex = static_cast<int>( m_shapeType );
		if ( ImGui::Combo( "Shape Type", &shapeIndex, shapeNames, IM_ARRAYSIZE( shapeNames ) ) )
		{
			m_shapeType = static_cast<ShapeType>( shapeIndex );
			Launch();
		}
		if ( ImGui::Checkbox( "Fixed Rotation", &m_fixedRotation ) )
			Launch();

		if ( ImGui::Checkbox( "Enable Hit Events", &m_enableHitEvents ) )
			b2Body_EnableHitEvents( m_bodyId, m_enableHitEvents );

		ImGui::SliderFloat( "Shape Size", &m_shapeSize, 0.1f, 1.0f, "%.2f" );
		ImGui::SliderFloat( "Restitution", &m_restitution, 0.0f, 1.0f, "%.2f" );
		ImGui::SliderFloat( "Friction", &m_friction, 0.0f, 1.0f, "%.2f" );

		if ( ImGui::SliderFloat( "Sound Volume", &m_soundVolume, 0.0f, 100.0f, "%.1f%%" ) )
			m_audioManager.SetVolume( m_soundVolume );

		ImGui::Checkbox( "Enable Screen Shake", &m_enableScreenShake );

		ImGui::End();
	}

private:
	AudioManager m_audioManager;
	float m_soundVolume;

	ShapeType m_shapeType;
	bool m_enableHitEvents;
	bool m_fixedRotation;
	bool m_enableScreenShake;

	b2BodyId m_bodyId = b2_nullBodyId;
	b2BodyId m_wallBodyId = b2_nullBodyId;

	b2BodyId m_anchorId = b2_nullBodyId;
	b2JointId m_motorJointId = b2_nullJointId;

	float m_shapeSize = 0.40f;
	float m_restitution = 1.0f;
	float m_friction = 0.0f;

	int shakeDuration;
	float shakeIntensity;
	b2Vec2 cameraBasePosition = { 0.0f, 0.0f };

	int spawnTimer;
	int spawnInterval;

	void InitializeAudio()
	{
		std::filesystem::path path = "data/audio/xp";
		if ( !std::filesystem::exists( path ) )
			path = "D:/Sound & Fx/audio/minecraft/xp";
		m_audioManager.LoadFromDirectory( path.string() );
		m_audioManager.SetVolume( m_soundVolume );
	}

	void Launch()
	{
		// Détruit l'ancien body si besoin
		if ( B2_IS_NON_NULL( m_bodyId ) )
			b2DestroyBody( m_bodyId );

		// Détruit anchor/joint motorisés si besoin (reset propre)
		if ( B2_IS_NON_NULL( m_motorJointId ) )
		{
			b2DestroyJoint( m_motorJointId );
			m_motorJointId = b2_nullJointId;
		}
		if ( B2_IS_NON_NULL( m_anchorId ) )
		{
			b2DestroyBody( m_anchorId );
			m_anchorId = b2_nullBodyId;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.linearVelocity = { 0.0f, 0.0f };
		bodyDef.position = { 0.0f, 0.0f };
		bodyDef.gravityScale = 0.0f;
		bodyDef.motionLocks.angularZ = m_fixedRotation;
		m_bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material = b2DefaultSurfaceMaterial();
		shapeDef.material.restitution = m_restitution;
		shapeDef.material.friction = m_friction;
		shapeDef.enableHitEvents = m_enableHitEvents;
		shapeDef.material.customColor = 0x4B0082;

		// Création de la forme choisie
		if ( m_shapeType == e_circleShape )
		{
			b2Circle circle = { { 0.0f, 0.0f }, m_shapeSize };
			b2CreateCircleShape( m_bodyId, &shapeDef, &circle );
		}
		else if ( m_shapeType == e_capsuleShape )
		{
			b2Capsule capsule = { { -m_shapeSize, 0.0f }, { m_shapeSize, 0.0f }, m_shapeSize * 0.5f };
			b2CreateCapsuleShape( m_bodyId, &shapeDef, &capsule );
		}
		else // box
		{
			b2Polygon box = b2MakeBox( m_shapeSize, m_shapeSize );
			b2CreatePolygonShape( m_bodyId, &shapeDef, &box );
		}

		// Toujours un motor joint (quel que soit le type)
		b2BodyDef anchorDef = b2DefaultBodyDef();
		anchorDef.position = { 0.0f, 0.0f };
		m_anchorId = b2CreateBody( m_worldId, &anchorDef );

		b2MotorJointDef jointDef = b2DefaultMotorJointDef();
		jointDef.base.bodyIdA = m_anchorId;
		jointDef.base.bodyIdB = m_bodyId;
		jointDef.maxForce = 500.0f;
		jointDef.maxTorque = 1000.0f;
		jointDef.correctionFactor = 1.0f;
		m_motorJointId = b2CreateMotorJoint( m_worldId, &jointDef );
	}

	void GrowBody()
	{
		if ( B2_IS_NON_NULL( m_bodyId ) )
		{
			b2ShapeId shapes[10];
			int count = b2Body_GetShapes( m_bodyId, shapes, 10 );
			if ( count > 0 )
			{
				b2ShapeId shapeId = shapes[0];
				if ( m_shapeType == e_circleShape )
				{
					b2Circle circle = b2Shape_GetCircle( shapeId );
					circle.radius *= 1.05f;
					b2Shape_SetCircle( shapeId, &circle );
				}
				else if ( m_shapeType == e_capsuleShape )
				{
					b2Capsule capsule = b2Shape_GetCapsule( shapeId );
					capsule.radius *= 1.05f;
					capsule.center1.x *= 1.05f;
					capsule.center2.x *= 1.05f;
					b2Shape_SetCapsule( shapeId, &capsule );
				}
				else
				{
					b2Polygon box = b2Shape_GetPolygon( shapeId );
					for ( int j = 0; j < box.count; ++j )
					{
						box.vertices[j].x *= 1.05f;
						box.vertices[j].y *= 1.05f;
					}
					b2Shape_SetPolygon( shapeId, &box );
				}
				b2Body_ApplyMassFromShapes( m_bodyId );
			}
		}
	}

	void ApplyShakeEffect()
	{
		if ( shakeDuration > 0 )
		{
			float shakeX = ( static_cast<float>( rand() ) / RAND_MAX - 0.5f ) * shakeIntensity;
			float shakeY = ( static_cast<float>( rand() ) / RAND_MAX - 0.5f ) * shakeIntensity;
			m_context->camera.m_center.x = cameraBasePosition.x + shakeX;
			m_context->camera.m_center.y = cameraBasePosition.y + shakeY;
			shakeDuration--;
			shakeIntensity *= 0.9f;
		}
		else
		{
			float returnSpeed = 0.15f;
			m_context->camera.m_center.x += ( cameraBasePosition.x - m_context->camera.m_center.x ) * returnSpeed;
			m_context->camera.m_center.y += ( cameraBasePosition.y - m_context->camera.m_center.y ) * returnSpeed;
		}
	}

	void SpawnBombardmentObject()
	{
		float angle = static_cast<float>( rand() ) / RAND_MAX * 2.0f * 3.14159265359f;
		float radius = 20.0f;
		float x = radius * cos( angle );
		float y = radius * sin( angle );
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { x, y };
		bodyDef.gravityScale = 0.0f;
		float speed = 25.0f;
		bodyDef.linearVelocity = { -speed * cos( angle ), -speed * sin( angle ) };
		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material = b2DefaultSurfaceMaterial();
		shapeDef.material.restitution = 0.5f;
		shapeDef.material.friction = 0.3f;
		b2Circle circle = { { 0.0f, 0.0f }, 0.1f };
		b2CreateCircleShape( bodyId, &shapeDef, &circle );
	}
};

static int MetamorphicBounceSample = RegisterSample( "9:16", "Metamorphic Bounce", MetamorphicBounce::Create );


class ParticlesBlueprintSample : public Sample
{
public:
	enum ParticleType
	{
		Normal = 0,
		Sensor,
		Destructeur
	};

	static Sample* Create( SampleContext* context )
	{
		return new ParticlesBlueprintSample( context );
	}

	explicit ParticlesBlueprintSample( SampleContext* context )
		: Sample( context )
		, m_type( Normal )
	{
		m_emissionPos = { 0, 0 };
		m_particleSystem.SetColor( 0x44F8FF );
		m_particleSystem.SetLifetime( 1.2f );
		m_particleSystem.SetWorld( m_worldId ); // <-- Important pour PhysicParticleSystem !
	}

	void UpdateGui() override
	{
		ImGui::Begin( "Particles Blueprint" );

		static const char* types[] = { "Normal", "Sensor", "Destructeur" };
		int t = (int)m_type;
		if ( ImGui::Combo( "Type", &t, types, IM_ARRAYSIZE( types ) ) )
			m_type = (ParticleType)t;

		int maxP = m_particleSystem.GetMaxParticles();
		if ( ImGui::SliderInt( "Max Particles", &maxP, 10, 1000 ) )
			m_particleSystem.SetMaxParticles( maxP );

		float life = m_particleSystem.GetLifetime();
		if ( ImGui::SliderFloat( "Lifetime", &life, 0.05f, 8.0f ) )
			m_particleSystem.SetLifetime( life );

		float radius = m_particleSystem.GetRadius();
		if ( ImGui::SliderFloat( "Radius", &radius, 0.01f, 0.5f ) )
			m_particleSystem.SetRadius( radius );

		if ( ImGui::ColorEdit4( "Color", m_particleSystem.GetColorPtr() ) )
			m_particleSystem.SyncEditToColor();

		ImGui::InputFloat2( "Emission (X,Y)", &m_emissionPos.x );

		static int burstCount = 30;
		ImGui::SliderInt( "Burst Count", &burstCount, 1, 256 );

		if ( ImGui::Button( "Emit Burst" ) )
			EmitBurstOfType( m_emissionPos, burstCount );

		ImGui::Text( "Alive: %d", m_particleSystem.GetAliveCount() );
		ImGui::End();
	}

	void Step() override
	{
		m_particleSystem.Update( 1.0f / 60.0f );
		m_particleSystem.Render( m_context->draw );
		Sample::Step();
	}

private:
	PhysicParticleSystem m_particleSystem;
	ParticleType m_type;
	b2Vec2 m_emissionPos;

	void EmitBurstOfType( const b2Vec2& pos, int count )
	{
		if ( m_type == Normal )
		{
			m_particleSystem.SetSensor( false );
			m_particleSystem.EmitBurst( pos, count );
		}
		else if ( m_type == Sensor )
		{
			m_particleSystem.SetSensor( true );
			for ( int i = 0; i < count; ++i )
			{
				float angle = float( i ) / std::max( 1, count ) * 2.0f * b2_pi;
				float speed = 2.5f + 1.2f * ( float( rand() ) / RAND_MAX );
				b2Vec2 vel = { std::cos( angle ), std::sin( angle ) };
				m_particleSystem.SetColor( 0x80F8FF ); // cyan sensor
				m_particleSystem.SetRadius( 0.22f );   // visuel plus gros
				m_particleSystem.Emit( pos, vel * speed );
			}
			m_particleSystem.SetSensor( false ); // Rétablir mode normal
			m_particleSystem.SetColor( 0x44F8FF );
			m_particleSystem.SetRadius( 0.12f );
		}
		else if ( m_type == Destructeur )
		{
			m_particleSystem.SetSensor( false );
			for ( int i = 0; i < count; ++i )
			{
				float angle = float( i ) / std::max( 1, count ) * 2.0f * b2_pi;
				float speed = 5.5f + 2.5f * ( float( rand() ) / RAND_MAX );
				b2Vec2 vel = { std::cos( angle ), std::sin( angle ) };
				m_particleSystem.SetColor( 0xFF3333 ); // rouge
				m_particleSystem.SetRadius( 0.17f );
				m_particleSystem.Emit( pos, vel * speed );
			}
			m_particleSystem.SetColor( 0x44F8FF );
			m_particleSystem.SetRadius( 0.12f );
		}
	}
};

static int sampleParticlesBlueprint = RegisterSample( "9:16", "ParticlesBlueprint", ParticlesBlueprintSample::Create );

class Verticalquestions : public Sample
{
public:
	struct ShapeUserData
	{
		bool shouldDestroyVisitors;
	};

	struct WallUserData
	{
		int id;
	};

	struct GridSlot
	{
		b2Vec2 position;
		b2BodyId bodyId = b2_nullBodyId;
	};
	std::vector<GridSlot> m_gridSlots;

	float m_regenTimer = 0.0f;
	float m_regenDelay = 0.35f; // Vitesse de repop (en secondes)

	explicit Verticalquestions( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 7.5f };
			m_context->camera.m_zoom = 35.0f;
		}

		// Configuration des ShapeUserData
		m_sensorBlue.shouldDestroyVisitors = true;
		m_sensorRed.shouldDestroyVisitors = true;

		b2World_SetGravity( m_worldId, { 0.0f, -9.8f } );
		// === Bleu ===
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { 5.0f, 0.0f };
		bodyDef.linearVelocity = { 10.0f, -25.0f };
		bodyDef.motionLocks.angularZ = false;
		m_bodyBlue = b2CreateBody( m_worldId, &bodyDef );

		static const uint16_t CATEGORY_DYNAMIC = 0x0002;
		static const uint16_t CATEGORY_SENSOR = 0x0004;
		static const uint16_t CATEGORY_WALL = 0x0001;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material = b2DefaultSurfaceMaterial();
		shapeDef.material.restitution = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.material.customColor = 0x2ED573;
		// shapeDef.filter.categoryBits = CATEGORY_DYNAMIC;
		shapeDef.filter.maskBits = CATEGORY_WALL | CATEGORY_SENSOR;

		b2Polygon solidBox = b2MakeBox( 1.5f, 1.5f );
		b2CreatePolygonShape( m_bodyBlue, &shapeDef, &solidBox );

		b2ShapeDef sensorDef = b2DefaultShapeDef();
		sensorDef.isSensor = true;
		sensorDef.enableSensorEvents = true;
		sensorDef.userData = &m_sensorBlue;
		sensorDef.filter.categoryBits = CATEGORY_SENSOR;
		sensorDef.filter.maskBits = CATEGORY_DYNAMIC | CATEGORY_WALL;
		sensorDef.material.customColor = 0x2ED573;

		float padding = 0.1f;
		b2Polygon sensorBox = b2MakeBox( 1.5f + padding, 1.5f + padding );
		b2CreatePolygonShape( m_bodyBlue, &sensorDef, &sensorBox );

		// === Rouge ===
		bodyDef.position = { -5.0f, 0.0f };
		bodyDef.linearVelocity = { -10.0f, -25.0f };
		bodyDef.motionLocks.angularZ = false;
		m_bodyRed = b2CreateBody( m_worldId, &bodyDef );

		shapeDef.material.customColor = 0xFF4757;
		b2CreatePolygonShape( m_bodyRed, &shapeDef, &solidBox );

		sensorDef.userData = &m_sensorRed;
		sensorDef.material.customColor = 0xFF4757;
		b2CreatePolygonShape( m_bodyRed, &sensorDef, &sensorBox );

		CreateGrid();
		InitializeAudio();

		// Murs pleins (9:16)
		{
			float arenaHalfWidth = 13.5f;
			float arenaHalfHeight = 13.5f;
			const float WALL_THICKNESS = 0.5f;
			const float halfT = WALL_THICKNESS * 0.5f;

			b2BodyDef wallDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &wallDef );

			b2ShapeDef wallShape = b2DefaultShapeDef();
			wallShape.material = b2DefaultSurfaceMaterial();
			wallShape.material.restitution = 1.01f;
			wallShape.material.friction = 0.0f;
			wallShape.material.customColor = 0x555555;

			// Bas
			{
				b2Polygon bottom = b2MakeOffsetBox( arenaHalfWidth,						// demi-largeur
													halfT,								// demi-épaisseur
													{ 0.0f, -arenaHalfHeight - halfT }, // centre
													b2Rot_identity );
				b2CreatePolygonShape( groundId, &wallShape, &bottom );
			}

			// Haut
			{
				b2Polygon top = b2MakeOffsetBox( arenaHalfWidth, halfT, { 0.0f, +arenaHalfHeight + halfT }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &wallShape, &top );
			}

			// Gauche
			{
				b2Polygon left = b2MakeOffsetBox( halfT, arenaHalfHeight, { -arenaHalfWidth - halfT, 0.0f }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &wallShape, &left );
			}

			// Droite
			{
				b2Polygon right = b2MakeOffsetBox( halfT, arenaHalfHeight, { +arenaHalfWidth + halfT, 0.0f }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &wallShape, &right );
			}
		}
	}

	uint32_t HSVtoRGB( float h, float s, float v )
	{
		// Garder l'angle entre 0 et 360
		h = fmodf( h, 360.0f );
		if ( h < 0.0f )
			h += 360.0f;

		// Limiter les teintes à l'orange et au rouge
		if ( h < 30.0f || h > 60.0f )
			h = 30.0f + fmodf( h, 30.0f ); // Forcer les teintes entre 30° et 60°

		float c = v * s;
		float x = c * ( 1.0f - fabsf( fmodf( h / 60.0f, 2.0f ) - 1.0f ) );
		float m = v - c;
		float r = 0.0f, g = 0.0f, b = 0.0f;

		if ( h < 60.0f )
		{
			r = c;
			g = x;
		}
		else if ( h < 120.0f )
		{
			r = x;
			g = c;
		}
		else if ( h < 180.0f )
		{
			g = c;
			r = x;
		}
		else if ( h < 240.0f )
		{
			g = x;
			r = c;
		}
		else if ( h < 300.0f )
		{
			r = x;
			b = c;
		}
		else
		{
			r = c;
			b = x;
		}

		uint8_t R = static_cast<uint8_t>( ( r + m ) * 255 );
		uint8_t G = static_cast<uint8_t>( ( g + m ) * 255 );
		uint8_t B = static_cast<uint8_t>( ( b + m ) * 255 );

		return ( R << 16 ) | ( G << 8 ) | B;
	}

	void CreateGrid()
	{
		m_gridSlots.clear(); // Important pour reset lors d'une réinitialisation

		// Dimensions de l'arène
		float arenaHalfWidth = 13.5f;
		float arenaHalfHeight = 13.5f;
		float boxSize = 3.0f;
		float spacing = 0.0f;
		float cellSize = boxSize + spacing;

		int columnCount = static_cast<int>( ( arenaHalfWidth * 2 ) / cellSize );
		int rowCount = static_cast<int>( ( arenaHalfHeight * 2 ) / cellSize );

		float xStart = -arenaHalfWidth + ( cellSize * 0.5f );
		float yStart = arenaHalfHeight - ( cellSize * 0.5f );

		// Centre du pattern
		float centerX = 0.0f;
		float centerY = 0.0f;
		float maxRadius = std::sqrt( arenaHalfWidth * arenaHalfWidth + arenaHalfHeight * arenaHalfHeight );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.enableSensorEvents = true;
		shapeDef.material = b2DefaultSurfaceMaterial();

		for ( int row = 0; row < rowCount; ++row )
		{
			for ( int col = 0; col < columnCount; ++col )
			{
				float x = xStart + col * cellSize;
				float y = yStart - row * cellSize;

				// Calcul de la distance au centre
				float dx = x - centerX;
				float dy = y - centerY;
				float distance = std::sqrt( dx * dx + dy * dy );
				float factor = distance / maxRadius;

				// Teintes de rouge uniquement (0° à 30°)
				float hue = 0.0f + 30.0f * ( factor * factor );
				float saturation = 0.8f + 0.2f * std::cos( distance * 0.3f );
				float value = 0.9f - 0.3f * factor;

				shapeDef.material.customColor = HSVtoRGB( hue, saturation, value );

				b2BodyDef bodyDef = b2DefaultBodyDef(); // Nouveau à chaque case
				bodyDef.type = b2_kinematicBody;
				bodyDef.gravityScale = 0.0f;
				bodyDef.angularVelocity = 0.0f;
				bodyDef.position = { x, y };

				b2Polygon square = b2MakeBox( boxSize * 0.5f, boxSize * 0.5f );
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &square );

				// === Ajout du slot pour régénération ===
				m_gridSlots.push_back( { { x, y }, bodyId } );
			}
		}
	}

	void InitializeAudio()
	{
		m_hitAudio.LoadFromDirectory( "D:/Sound & Fx/audio/glo" );
		m_destroyAudio.LoadFromDirectory( "D:/Sound & Fx/audio/Ticks" );
		m_hitAudio.SetVolume( 10.0f );
		m_destroyAudio.SetVolume( 10.0f );

		m_yesBuffer.loadFromFile( "D:/Sound & Fx/audio/jojo/yes.wav" );
		m_noBuffer.loadFromFile( "D:/Sound & Fx/audio/jojo/no.wav" );
	}

	void Step() override
	{
		Sample::Step();

		if ( m_stepCount == m_lastStepCount )
		{
			return;
		}

		std::set<b2BodyId> zombies;

		// 🔎 Récupération des événements de capteur
		b2SensorEvents events = b2World_GetSensorEvents( m_worldId );

		for ( int i = 0; i < events.beginCount; ++i )
		{
			b2SensorBeginTouchEvent* event = events.beginEvents + i;
			ShapeUserData* userData = static_cast<ShapeUserData*>( b2Shape_GetUserData( event->sensorShapeId ) );

			// 💥 Si l'objet doit être détruit
			if ( userData->shouldDestroyVisitors )
			{
				zombies.emplace( b2Shape_GetBody( event->visitorShapeId ) );

				// 🏁 Score : qui détruit ?
				if ( userData == &m_sensorBlue )
				{
					m_blueScore++;
				}
				else if ( userData == &m_sensorRed )
				{
					m_redScore++;
				}

				// 🔊 Son de destruction
				m_destroyAudio.QueueSound( m_destroySoundIndex );
				m_destroySoundIndex = ( m_destroySoundIndex + 1 ) % m_destroyAudio.GetSoundCount();
			}
			else
			{
				// 🎯 Son de collision
				m_hitAudio.QueueSound( m_currentSoundIndex );
				m_currentSoundIndex = ( m_currentSoundIndex + 1 ) % m_hitAudio.GetSoundCount();

				// 🖌️ Changement de couleur
				b2SurfaceMaterial surfaceMaterial = b2Shape_GetSurfaceMaterial( event->visitorShapeId );
				surfaceMaterial.customColor = b2_colorLime;
				b2Shape_SetSurfaceMaterial( event->visitorShapeId, surfaceMaterial );
			}
		}

		// 🔁 Réinitialisation des couleurs (fin de contact)
		for ( int i = 0; i < events.endCount; ++i )
		{
			b2SensorEndTouchEvent* event = events.endEvents + i;

			if ( !b2Shape_IsValid( event->visitorShapeId ) )
			{
				continue;
			}

			b2SurfaceMaterial surfaceMaterial = b2Shape_GetSurfaceMaterial( event->visitorShapeId );
			surfaceMaterial.customColor = 0;
			b2Shape_SetSurfaceMaterial( event->visitorShapeId, surfaceMaterial );
		}

		// 💥 Détruire les bodies
		for ( b2BodyId bodyId : zombies )
		{
			b2DestroyBody( bodyId );
		}

		// ===== Régénération aléatoire d'une case manquante de la grille =====
		static std::mt19937 rng( std::random_device{}() );
		static std::uniform_real_distribution<float> regenChance( 0.0f, 1.0f );

		// Une fois par frame, on tente de régénérer 1 case avec une petite chance
		float spawnProbability = 0.25f; // % de chance par frame (ajuste à ton goût)
		std::vector<int> emptyIndices;
		for ( int i = 0; i < m_gridSlots.size(); ++i )
		{
			if ( !b2Body_IsValid( m_gridSlots[i].bodyId ) ) // Slot vide
				emptyIndices.push_back( i );
		}

		if ( !emptyIndices.empty() && regenChance( rng ) < spawnProbability )
		{
			int idx = emptyIndices[std::uniform_int_distribution<int>( 0, emptyIndices.size() - 1 )( rng )];
			auto& slot = m_gridSlots[idx];

			// Protection : n'apparait pas sous un compétiteur
			b2Vec2 posBlue = b2Body_GetPosition( m_bodyBlue );
			b2Vec2 posRed = b2Body_GetPosition( m_bodyRed );
			float minDist = 4.0f; // même taille que la case
			auto dist = []( b2Vec2 a, b2Vec2 b ) {
				float dx = a.x - b.x;
				float dy = a.y - b.y;
				return std::sqrt( dx * dx + dy * dy );
			};
			if ( dist( slot.position, posBlue ) < minDist || dist( slot.position, posRed ) < minDist )
			{
				return; // trop proche, pas de pop ce tour-ci !
			}

			// Respawn un carré à la position de ce slot
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_kinematicBody;
			bodyDef.position = slot.position;
			bodyDef.gravityScale = 0.0f;
			bodyDef.angularVelocity = 0.0f;

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.enableSensorEvents = true;
			shapeDef.material = b2DefaultSurfaceMaterial();

			float dx = slot.position.x;
			float dy = slot.position.y;
			float distance = std::sqrt( dx * dx + dy * dy );
			float maxRadius = 19.09f; // 13.5 * sqrt(2) environ si tu veux garder la même teinte
			float factor = distance / maxRadius;
			float hue = 0.0f + 30.0f * ( factor * factor );
			float saturation = 0.8f + 0.2f * std::cos( distance * 0.3f );
			float value = 0.9f - 0.3f * factor;
			shapeDef.material.customColor = HSVtoRGB( hue, saturation, value );

			b2Polygon square = b2MakeBox( 3.0f * 0.5f, 3.0f * 0.5f );
			slot.bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( slot.bodyId, &shapeDef, &square );
		}

		// 🔊 Jouer les sons en file d'attente
		m_hitAudio.PlayQueued();
		m_destroyAudio.PlayQueued();

		bool yesLead = m_blueScore > m_redScore;
		bool noLead = m_redScore > m_blueScore;

		// YES LEAD
		if ( yesLead && !m_isYesLooping )
		{
			if ( m_loopSound )
				m_loopSound->stop();

			m_loopSound = std::make_unique<sf::Sound>( m_yesBuffer );
			m_loopSound->setVolume( 20.0f );
			m_loopSound->play();
			m_isYesLooping = true;
			m_isNoLooping = false;
		}
		// NO LEAD
		else if ( noLead && !m_isNoLooping )
		{
			if ( m_loopSound )
				m_loopSound->stop();

			m_loopSound = std::make_unique<sf::Sound>( m_noBuffer );
			m_loopSound->setVolume( 20.0f );
			m_loopSound->play();
			m_isNoLooping = true;
			m_isYesLooping = false;
		}
		// ÉGALITÉ OU RESET
		else if ( !yesLead && !noLead )
		{
			if ( m_loopSound )
				m_loopSound->stop();
			m_isYesLooping = false;
			m_isNoLooping = false;
		}

		// --- Gestion du redémarrage "manuel" de la boucle ---
		if ( m_loopSound && m_loopSound->getStatus() == sf::Sound::Status::Stopped )
		{
			// Si la lead est toujours YES/NO, relance le son !
			if ( yesLead && m_isYesLooping )
				m_loopSound->play();
			else if ( noLead && m_isNoLooping )
				m_loopSound->play();
		}

		m_lastStepCount = m_stepCount;
	}

	void UpdateGui() override
	{
		// Fenêtre settings classique (optionnelle)
		ImGui::SetNextWindowPos( ImVec2( 20, 20 ), ImGuiCond_Always );
		ImGui::SetNextWindowSize( ImVec2( 280, 220 ), ImGuiCond_Always );
		ImGui::Begin( "Simulation Settings", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize );
		static float volume = 10.0f;
		if ( ImGui::SliderFloat( "Volume", &volume, 0.0f, 100.0f, "%.1f" ) )
		{
			m_hitAudio.SetVolume( volume );
			m_destroyAudio.SetVolume( volume );
		}
		static bool enableDestruction = true;
		if ( ImGui::Checkbox( "Enable Destructive Sensors", &enableDestruction ) )
		{
			m_sensorBlue.shouldDestroyVisitors = enableDestruction;
			m_sensorRed.shouldDestroyVisitors = enableDestruction;
		}
		ImGui::End();

		ImGuiIO& io = ImGui::GetIO();
		ImVec2 screenSize = io.DisplaySize;

		// --- Responsive layout ---

		ImVec2 qSize = ImVec2( 550, 150 );
		ImVec2 cardSize = ImVec2( 250, 90 );
		float cardSpacing = 20.0f;
		float paddingBelowQuestion = 20.0f;

		// Décale le "bloc" (question+cartes) verticalement si tu veux, ici -100 pour un look mobile friendly
		float groupYOffset = -500.0f;

		// 1. Centrer la question au milieu vertical (modifié par groupYOffset si besoin)
		ImVec2 qPos = ImVec2( ( screenSize.x - qSize.x ) * 0.5f,
							  ( screenSize.y - ( qSize.y + cardSize.y + paddingBelowQuestion ) ) * 0.5f + groupYOffset );

		ImGui::SetNextWindowPos( qPos, ImGuiCond_Always );
		ImGui::SetNextWindowSize( qSize, ImGuiCond_Always );
		ImGui::PushStyleColor( ImGuiCol_WindowBg, IM_COL32( 255, 255, 255, 255 ) );
		ImGui::PushStyleVar( ImGuiStyleVar_WindowBorderSize, 0.0f );
		ImGui::PushStyleVar( ImGuiStyleVar_WindowRounding, 5.0f );

		ImGui::Begin( "QuestionCard", nullptr,
					  ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
						  ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar );

		ImGui::PushFont( m_context->draw.m_largeFont );

		const char* question = "Are you\nugly?"; // <<< Ici, nouveau texte
		ImVec2 textSize = ImGui::CalcTextSize( question );
		ImGui::SetCursorPosY( ( qSize.y - textSize.y ) * 0.5f );
		ImGui::SetCursorPosX( ( qSize.x - textSize.x ) * 0.5f );
		ImGui::TextColored( ImVec4( 0, 0, 0, 1 ), "%s", question );

		ImGui::PopFont();

		ImGui::End();
		ImGui::PopStyleVar( 2 );
		ImGui::PopStyleColor();

		// 2. Cartes OUI/NON juste sous la question (toujours groupées)
		float cardY = qPos.y + qSize.y + paddingBelowQuestion;
		ImVec2 yesPos = ImVec2( screenSize.x * 0.5f - cardSize.x - cardSpacing / 2.0f, cardY );
		ImVec2 noPos = ImVec2( screenSize.x * 0.5f + cardSpacing / 2.0f, cardY );

		bool yesLead = m_blueScore > m_redScore;
		bool noLead = m_redScore > m_blueScore;

		// YES CARD
		ImGui::SetNextWindowPos( yesPos, ImGuiCond_Always );
		ImGui::SetNextWindowSize( cardSize, ImGuiCond_Always );
		ImGui::PushStyleColor( ImGuiCol_WindowBg, yesLead ? IM_COL32( 46, 213, 115, 255 ) : IM_COL32( 255, 255, 255, 255 ) );
		ImGui::PushStyleVar( ImGuiStyleVar_WindowBorderSize, 0.0f );
		ImGui::PushStyleVar( ImGuiStyleVar_WindowRounding, 5.0f );

		ImGui::Begin( "YesCard", nullptr,
					  ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
						  ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar );

		ImGui::PushFont( m_context->draw.m_largeFont );
		ImGui::SetCursorPosY( ( cardSize.y - ImGui::CalcTextSize( "Yes : 999" ).y ) * 0.5f );
		ImGui::SetCursorPosX( ( cardSize.x - ImGui::CalcTextSize( "Yes : 999" ).x ) * 0.5f );
		if ( yesLead )
			ImGui::TextColored( ImVec4( 1.0f, 1.0f, 1.0f, 1.0f ), "Yes : %d", m_blueScore );
		else
			ImGui::TextColored( ImVec4( 46 / 255.0f, 213 / 255.0f, 115 / 255.0f, 1.0f ), "Yes : %d", m_blueScore );
		ImGui::PopFont();

		ImGui::End();
		ImGui::PopStyleVar( 2 );
		ImGui::PopStyleColor();

		// NO CARD
		ImGui::SetNextWindowPos( noPos, ImGuiCond_Always );
		ImGui::SetNextWindowSize( cardSize, ImGuiCond_Always );
		ImGui::PushStyleColor( ImGuiCol_WindowBg, noLead ? IM_COL32( 255, 71, 87, 255 ) : IM_COL32( 255, 255, 255, 255 ) );
		ImGui::PushStyleVar( ImGuiStyleVar_WindowBorderSize, 0.0f );
		ImGui::PushStyleVar( ImGuiStyleVar_WindowRounding, 5.0f );

		ImGui::Begin( "NoCard", nullptr,
					  ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
						  ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar );

		ImGui::PushFont( m_context->draw.m_largeFont );
		ImGui::SetCursorPosY( ( cardSize.y - ImGui::CalcTextSize( "No : 999" ).y ) * 0.5f );
		ImGui::SetCursorPosX( ( cardSize.x - ImGui::CalcTextSize( "No : 999" ).x ) * 0.5f );
		if ( noLead )
			ImGui::TextColored( ImVec4( 1.0f, 1.0f, 1.0f, 1.0f ), "No : %d", m_redScore );
		else
			ImGui::TextColored( ImVec4( 255 / 255.0f, 71 / 255.0f, 87 / 255.0f, 1.0f ), "No : %d", m_redScore );
		ImGui::PopFont();

		ImGui::End();
		ImGui::PopStyleVar( 2 );
		ImGui::PopStyleColor();

		// --- Affichage du texte "YES" et "NO" sur les capteurs, bien centré ---

		b2Vec2 posBlue = b2Body_GetPosition( m_bodyBlue );
		b2Vec2 posRed = b2Body_GetPosition( m_bodyRed );

		b2Vec2 screenBlue = m_context->camera.ConvertWorldToScreen( posBlue );
		b2Vec2 screenRed = m_context->camera.ConvertWorldToScreen( posRed );

		ImFont* midFont = m_context->draw.m_mediumFont;

		ImVec2 sizeYES = midFont->CalcTextSizeA( midFont->FontSize, FLT_MAX, 0.0f, "YES" );
		ImVec2 sizeNO = midFont->CalcTextSizeA( midFont->FontSize, FLT_MAX, 0.0f, "NO" );

		int yesX = static_cast<int>( screenBlue.x - sizeYES.x * 0.5f );
		int yesY = static_cast<int>( screenBlue.y - sizeYES.y * 0.5f );
		int noX = static_cast<int>( screenRed.x - sizeNO.x * 0.5f );
		int noY = static_cast<int>( screenRed.y - sizeNO.y * 0.5f );

		ImGui::GetForegroundDrawList()->AddText( midFont, midFont->FontSize, ImVec2( yesX, yesY ), IM_COL32( 255, 255, 255, 255 ),
												 "YES" );
		ImGui::GetForegroundDrawList()->AddText( midFont, midFont->FontSize, ImVec2( noX, noY ), IM_COL32( 255, 255, 255, 255 ),
												 "NO" );
	}

	void StopLoopSound()
	{
		if ( m_loopSound )
		{
			m_loopSound->stop();
		}
		m_isYesLooping = false;
		m_isNoLooping = false;
	}

	static Sample* Create( SampleContext* context )
	{
		return new Verticalquestions( context );
	}

	static constexpr int m_columnCount = 5;
	static constexpr int m_rowCount = 5;
	int m_maxBeginCount;
	int m_maxEndCount;
	int m_lastStepCount = 0;

	ShapeUserData m_passiveSensor;
	ShapeUserData m_activeSensor;

	// ✅ Capteurs compétitifs
	ShapeUserData m_sensorBlue;
	ShapeUserData m_sensorRed;

	// ✅ Corps des compétiteurs
	b2BodyId m_bodyBlue = b2_nullBodyId;
	b2BodyId m_bodyRed = b2_nullBodyId;

	// ✅ Scores
	int m_blueScore = 0;
	int m_redScore = 0;

	// ✅ Audio
	AudioManager m_hitAudio;
	AudioManager m_destroyAudio;
	size_t m_currentSoundIndex = 0;
	size_t m_destroySoundIndex = 0;

	sf::SoundBuffer m_yesBuffer;
	sf::SoundBuffer m_noBuffer;
	std::unique_ptr<sf::Sound> m_loopSound; // Plus de vector : juste un unique_ptr
	bool m_isYesLooping = false;
	bool m_isNoLooping = false;
	std::vector<std::unique_ptr<WallUserData>> m_wallUserDatas;
	std::vector<b2ShapeId> m_wallShapeIds;
	int m_nextWallId = 0;
};

static int Verticalquestions = RegisterSample( "9:16", "Verticalquestions", Verticalquestions::Create );

class RestitutionWithHits : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new RestitutionWithHits( context );
	}

	enum ShapeType
	{
		e_circleShape,
		e_boxShape
	};
	struct HitEvent
	{
		b2Vec2 point;
		float speed;
		int stepIndex;
	};

	explicit RestitutionWithHits( SampleContext* context )
		: Sample( context )

	{
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 20.0f;
		}

		m_audioManager.LoadFromDirectory( "D:/Sound & Fx/audio/Ticks" );

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f };
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 100.0f, 1.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material = b2DefaultSurfaceMaterial();
			shapeDef.material.customColor = 0x888888; // <-- couleur grise personnalisée

			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		CreateBodies();
	}

	void Step() override
	{
		Sample::Step();

		// Joue tous les sons mis en queue ce step
		m_audioManager.PlayQueued();
	}

private:
	std::vector<uint32_t> rainbowColors = { 0xFFB3B3, 0xFFC2A1, 0xFFD199, 0xFFE8A1, 0xFFF0B3, 0xE8FFC4, 0xD1FFC4, 0xA1FFCC,
											0xA1E8FF, 0xA1D1FF, 0xB3A1FF, 0xD199FF, 0xE8A1FF, 0xF0B3FF, 0xFFB3E8, 0xFFB3C4 };

	int colorIndex = 0;
	AudioManager m_audioManager;

	void CreateBodies()
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.restitution = 1.0f;
		shapeDef.material.friction = 0.0f;

		float dx = 1.5f;
		float x = -dx * 4.5f;

		for ( int i = 0; i < 10; ++i )
		{
			bodyDef.position = { x, 16.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			shapeDef.material.customColor = rainbowColors[colorIndex++ % rainbowColors.size()];

			b2Polygon box = b2MakeBox( 0.5f, 0.5f );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
			x += dx;
		}
	}
};

static int sampleRestitution = RegisterSample( "9:16", "RestitutionWithHits", RestitutionWithHits::Create );

class BluePrintSample : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new BluePrintSample( context );
	}

	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape,
		e_polygon3,
		e_polygon4,
		e_polygon5,
		e_polygon6,
		e_polygon7,
		e_polygon8
	};

	explicit BluePrintSample( SampleContext* context )
		: Sample( context )
		, m_shapeType( e_boxShape )
		, m_enableHitEvents( true )
		, m_soundVolume( 50.0f )
		, m_fixedRotation( false )
		, m_linearDamping( 0.0f )
		, m_angularDamping( 0.0f )
		, m_gravityScale( 1.0f )
		, m_isBullet( false )
		, m_gravity{ 0.0f, -10.0f }
	{
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 8.0f };
			m_context->camera.m_zoom = 20.0f;
		}

		b2World_SetGravity( m_worldId, m_gravity );
		InitializeAudio();
		CreateGround();
		CreateBodies();
	}

	void Step() override
	{
		Sample::Step();

		if ( m_enableHitEvents )
		{
			b2ContactEvents events = b2World_GetContactEvents( m_worldId );
			for ( int i = 0; i < events.hitCount; ++i )
			{
				m_audioManager.HandleHitEffect( events.hitEvents[i].point, events.hitEvents[i].approachSpeed, m_stepCount );
			}
		}

		m_audioManager.PlayQueued();
		m_audioManager.DrawHitEffects( &m_context->draw, m_stepCount );
	}

	void UpdateGui() override
	{
		ImGuiIO& io = ImGui::GetIO();
		ImVec2 center = ImVec2( io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f );

		ImGui::SetNextWindowPos( center, ImGuiCond_Always, ImVec2( 0.5f, 0.5f ) );
		ImGui::SetNextWindowSize( ImVec2( 400, 0 ), ImGuiCond_Always );

		ImGui::Begin( "BluePrint Settings" );

		// --- Sélection du type de forme ---
		const char* shapeNames[] = { "Circle",	 "Capsule", "Box",		"Triangle", "Quad",
									 "Pentagon", "Hexagon", "Heptagon", "Octagon" };
		int shapeIndex = static_cast<int>( m_shapeType );
		if ( ImGui::Combo( "Shape Type", &shapeIndex, shapeNames, IM_ARRAYSIZE( shapeNames ) ) )
		{
			m_shapeType = static_cast<ShapeType>( shapeIndex );
			CreateBodies();
		}
		if ( ImGui::SliderInt( "Shape Count", &m_shapeCount, 1, 50 ) )
			CreateBodies();
		if ( ImGui::Checkbox( "Fixed Rotation", &m_fixedRotation ) )
			CreateBodies();
		if ( ImGui::SliderFloat( "Shape Size", &m_shapeSize, 0.1f, 1.0f, "%.2f" ) )
			CreateBodies();
		if ( ImGui::SliderFloat( "Spacing", &m_spacing, 0.5f, 5.0f, "%.2f" ) )
			CreateBodies();

		// --- Paramètres physiques sliders ---
		bool needUpdateBodies = false;
		needUpdateBodies |= ImGui::SliderFloat( "Restitution", &m_restitution, 0.0f, 1.0f, "%.2f" );
		needUpdateBodies |= ImGui::SliderFloat( "Friction", &m_friction, 0.0f, 1.0f, "%.2f" );
		needUpdateBodies |= ImGui::SliderFloat( "Density", &m_density, 0.01f, 10.0f, "%.2f" );
		needUpdateBodies |= ImGui::SliderFloat( "Linear Damping", &m_linearDamping, 0.0f, 10.0f, "%.2f" );
		needUpdateBodies |= ImGui::SliderFloat( "Angular Damping", &m_angularDamping, 0.0f, 5.0f, "%.2f" );
		needUpdateBodies |= ImGui::SliderFloat( "Gravity Scale", &m_gravityScale, 0.0f, 5.0f, "%.2f" );
		needUpdateBodies |= ImGui::Checkbox( "Bullet", &m_isBullet );

		if ( needUpdateBodies )
			CreateBodies();

		// Contrôle du volume sonore
		if ( ImGui::SliderFloat( "Sound Volume", &m_soundVolume, 0.0f, 100.0f, "%.1f%%" ) )
			m_audioManager.SetVolume( m_soundVolume );

		ImGui::Checkbox( "Enable Hit Events", &m_enableHitEvents );

		// --- Bloc gravité ---
		static const char* gravityPresets[] = { "Terre", "Lune", "Mars", "0", "Reverse", "Max" };
		static float presetValues[] = { -10.0f, -1.62f, -3.71f, 0.0f, 10.0f, -980.0f }; // -980 = ~100g
		static int presetIdx = 0;

		if ( ImGui::Combo( "Gravity Preset", &presetIdx, gravityPresets, IM_ARRAYSIZE( gravityPresets ) ) )
		{
			m_gravity = { 0.0f, presetValues[presetIdx] };
			b2World_SetGravity( m_worldId, m_gravity );
			// Optionnel : tu peux mettre à jour les bodies si nécessaire ici :
			// CreateBodies();
		}

		ImGui::Text( "Preset: %s", gravityPresets[presetIdx] );
		ImGui::End();
	}

private:
	// --- Audio ---
	AudioManager m_audioManager;
	float m_soundVolume;

	// --- Physics ---
	ShapeType m_shapeType;
	bool m_enableHitEvents;
	bool m_fixedRotation;
	b2Vec2 m_gravity;

	float m_linearDamping;
	float m_angularDamping;
	float m_gravityScale;
	bool m_isBullet;

	float m_restitution = 1.0f;
	float m_friction = 0.0f;
	float m_density = 1.0f;
	float m_shapeSize = 1.0f;
	float m_spacing = 1.0f;

	// --- Shapes ---
	std::vector<b2BodyId> m_bodies;
	int m_shapeCount = 1;

	uint32_t ComputeEllipticColor( int ix, int iy, int cols, int rows )
	{
		float cx = ( cols - 1 ) * 0.5f;
		float cy = ( rows - 1 ) * 0.5f;
		float dx = static_cast<float>( ix ) - cx;
		float dy = static_cast<float>( iy ) - cy;
		float a = std::max( cx, 1.0f );
		float b = std::max( cy, 1.0f );
		float r = std::sqrt( ( dx * dx ) / ( a * a ) + ( dy * dy ) / ( b * b ) );
		r = std::clamp( r / std::sqrt( 2.0f ), 0.0f, 1.0f );
		float hue = r;
		return HSVtoRGB( hue, 0.75f, 1.0f );
	}

	uint32_t HSVtoRGB( float h, float s, float v )
	{
		float r, g, b;
		int i = int( h * 6.0f );
		float f = h * 6.0f - i;
		float p = v * ( 1.0f - s );
		float q = v * ( 1.0f - f * s );
		float t = v * ( 1.0f - ( 1.0f - f ) * s );
		switch ( i % 6 )
		{
			case 0:
				r = v, g = t, b = p;
				break;
			case 1:
				r = q, g = v, b = p;
				break;
			case 2:
				r = p, g = v, b = t;
				break;
			case 3:
				r = p, g = q, b = v;
				break;
			case 4:
				r = t, g = p, b = v;
				break;
			case 5:
				r = v, g = p, b = q;
				break;
		}
		uint32_t R = uint32_t( r * 255.0f );
		uint32_t G = uint32_t( g * 255.0f );
		uint32_t B = uint32_t( b * 255.0f );
		return ( R << 16 ) | ( G << 8 ) | B;
	}

	void InitializeAudio()
	{
		std::filesystem::path path = "data/audio/Ticks";
		if ( !std::filesystem::exists( path ) )
			path = "D:/Sound & Fx/audio/Ticks";
		m_audioManager.LoadFromDirectory( path.string() );
		m_audioManager.SetVolume( m_soundVolume );
	}

	void CreateGround()
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = { 0.0f, -1.0f };
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2Polygon box = b2MakeBox( 100.0f, 1.0f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material = b2DefaultSurfaceMaterial();
		shapeDef.material.customColor = 0x888888;
		b2CreatePolygonShape( groundId, &shapeDef, &box );
	}

	void CreateBodies()
	{
		for ( b2BodyId id : m_bodies )
			if ( B2_IS_NON_NULL( id ) )
				b2DestroyBody( id );
		m_bodies.clear();

		// Calcul de la taille de la grille
		int n = m_shapeCount;
		int cols = int( std::ceil( std::sqrt( n ) ) ); // nombre de colonnes (grille carrée)
		int rows = ( n + cols - 1 ) / cols;			   // nombre de lignes

		float dx = m_spacing; // espace horizontal
		float dy = m_spacing; // espace vertical

		// Décalage pour centrer la grille autour de (0, posY)
		float totalW = dx * ( cols - 1 );
		float totalH = dy * ( rows - 1 );
		float startX = -totalW * 0.5f;
		float startY = 16.0f + totalH * 0.5f;

		for ( int i = 0; i < n; ++i )
		{
			int ix = i % cols;
			int iy = i / cols;

			float x = startX + ix * dx;
			float y = startY - iy * dy;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { x, y };
			bodyDef.linearVelocity = { 0.0f, -10.0f };
			bodyDef.motionLocks.angularZ = false;
			bodyDef.linearDamping = m_linearDamping;
			bodyDef.angularDamping = m_angularDamping;
			bodyDef.gravityScale = m_gravityScale;
			bodyDef.isBullet = m_isBullet;

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			m_bodies.push_back( bodyId );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = m_density;
			shapeDef.material = b2DefaultSurfaceMaterial();
			shapeDef.material.restitution = m_restitution;
			shapeDef.material.friction = m_friction;
			shapeDef.enableHitEvents = true;
			shapeDef.material.customColor = ComputeEllipticColor( ix, iy, cols, rows );

			ShapeType shapeType = static_cast<ShapeType>( m_shapeType );

			if ( shapeType == e_circleShape )
			{
				b2Circle circle = { { 0.0f, 0.0f }, 0.5f * m_shapeSize };
				b2CreateCircleShape( bodyId, &shapeDef, &circle );
			}
			else if ( shapeType == e_capsuleShape )
			{
				b2Capsule capsule = { { -0.5f * m_shapeSize, 0.0f }, { 0.5f * m_shapeSize, 0.0f }, 0.25f * m_shapeSize };
				b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			}
			else if ( shapeType == e_boxShape )
			{
				b2Polygon box = b2MakeBox( 0.5f * m_shapeSize, 0.5f * m_shapeSize );
				b2CreatePolygonShape( bodyId, &shapeDef, &box );
			}
			else // Polygones réguliers 3-8 côtés
			{
				int sides = int( shapeType ) - int( e_polygon3 ) + 3;
				std::vector<b2Vec2> verts( sides );
				const float R = 0.5f * m_shapeSize;
				const float twoPi = 2.0f * b2_pi;
				for ( int j = 0; j < sides; ++j )
					verts[j] = { std::cos( j * twoPi / sides ) * R, std::sin( j * twoPi / sides ) * R };
				b2Hull hull = b2ComputeHull( verts.data(), sides );
				if ( hull.count > 0 )
				{
					b2Polygon poly = b2MakePolygon( &hull, 0 );
					b2CreatePolygonShape( bodyId, &shapeDef, &poly );
				}
			}
		}
	}
};

static int sampleBluePrintSample = RegisterSample( "9:16", "BluePrintSample", BluePrintSample::Create );

class TestCollisionColor : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new TestCollisionColor( context );
	}

	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape,
		e_polygon3,
		e_polygon4,
		e_polygon5,
		e_polygon6,
		e_polygon7,
		e_polygon8
	};

	explicit TestCollisionColor( SampleContext* context )
		: Sample( context )
		, m_shapeType( e_boxShape )
		, m_enableHitEvents( true )
		, m_soundVolume( 50.0f )
		, m_fixedRotation( false )
		, m_linearDamping( 0.0f )
		, m_angularDamping( 0.0f )
		, m_gravityScale( 1.0f )
		, m_isBullet( false )
		, m_gravity{ 0.0f, -10.0f }
	{
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 8.0f };
			m_context->camera.m_zoom = 20.0f;
		}

		b2World_SetGravity( m_worldId, m_gravity );
		InitializeAudio();
		CreateGround();
		CreateBodies();
	}

	void Step() override
	{
		Sample::Step();

		// 1. Impact : met la couleur impact si collision
		if ( m_enableHitEvents )
		{
			b2ContactEvents events = b2World_GetContactEvents( m_worldId );
			for ( int i = 0; i < events.hitCount; ++i )
			{
				const b2ContactHitEvent& hit = events.hitEvents[i];

				b2SurfaceMaterial matA = b2Shape_GetSurfaceMaterial( hit.shapeIdA );
				matA.customColor = 0xFF3040;
				b2Shape_SetSurfaceMaterial( hit.shapeIdA, matA );

				b2SurfaceMaterial matB = b2Shape_GetSurfaceMaterial( hit.shapeIdB );
				matB.customColor = 0xFF3040;
				b2Shape_SetSurfaceMaterial( hit.shapeIdB, matB );

				m_audioManager.HandleHitEffect( hit.point, hit.approachSpeed, m_stepCount );
			}
		}

		// 2. Reset : si une shape n'est plus en contact, on remet sa couleur initiale de grille
		// (optionnel : pour "parfait", stocker le ComputeEllipticColor ou l'index via userData !)
		int n = m_shapeCount;
		int cols = int( std::ceil( std::sqrt( n ) ) );
		int rows = ( n + cols - 1 ) / cols;
		for ( size_t i = 0; i < m_shapes.size(); ++i )
		{
			b2ShapeId shapeId = m_shapes[i];

			b2ContactData contactData[4];
			int contactCount = b2Shape_GetContactData( shapeId, contactData, 4 );
			bool inContact = false;
			for ( int c = 0; c < contactCount; ++c )
			{
				if ( contactData[c].manifold.pointCount > 0 )
				{
					inContact = true;
					break;
				}
			}
			if ( !inContact )
			{
				// Recalcule la couleur d'origine selon la grille
				int ix = i % cols, iy = i / cols;
				uint32_t color = ComputeEllipticColor( ix, iy, cols, rows );

				b2SurfaceMaterial mat = b2Shape_GetSurfaceMaterial( shapeId );
				mat.customColor = color;
				b2Shape_SetSurfaceMaterial( shapeId, mat );
			}
		}

		m_audioManager.PlayQueued();
		m_audioManager.DrawHitEffects( &m_context->draw, m_stepCount );
	}

	void UpdateGui() override
	{
		ImGuiIO& io = ImGui::GetIO();
		ImVec2 center = ImVec2( io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f );

		ImGui::SetNextWindowPos( center, ImGuiCond_Always, ImVec2( 0.5f, 0.5f ) );
		ImGui::SetNextWindowSize( ImVec2( 400, 0 ), ImGuiCond_Always );

		ImGui::Begin( "BluePrint Settings" );

		// --- Sélection du type de forme ---
		const char* shapeNames[] = { "Circle",	 "Capsule", "Box",		"Triangle", "Quad",
									 "Pentagon", "Hexagon", "Heptagon", "Octagon" };
		int shapeIndex = static_cast<int>( m_shapeType );
		if ( ImGui::Combo( "Shape Type", &shapeIndex, shapeNames, IM_ARRAYSIZE( shapeNames ) ) )
		{
			m_shapeType = static_cast<ShapeType>( shapeIndex );
			CreateBodies();
		}
		if ( ImGui::SliderInt( "Shape Count", &m_shapeCount, 1, 50 ) )
			CreateBodies();
		if ( ImGui::Checkbox( "Fixed Rotation", &m_fixedRotation ) )
			CreateBodies();
		if ( ImGui::SliderFloat( "Shape Size", &m_shapeSize, 0.1f, 1.0f, "%.2f" ) )
			CreateBodies();
		if ( ImGui::SliderFloat( "Spacing", &m_spacing, 0.5f, 5.0f, "%.2f" ) )
			CreateBodies();

		// --- Paramètres physiques sliders ---
		bool needUpdateBodies = false;
		needUpdateBodies |= ImGui::SliderFloat( "Restitution", &m_restitution, 0.0f, 1.0f, "%.2f" );
		needUpdateBodies |= ImGui::SliderFloat( "Friction", &m_friction, 0.0f, 1.0f, "%.2f" );
		needUpdateBodies |= ImGui::SliderFloat( "Density", &m_density, 0.01f, 10.0f, "%.2f" );
		needUpdateBodies |= ImGui::SliderFloat( "Linear Damping", &m_linearDamping, 0.0f, 10.0f, "%.2f" );
		needUpdateBodies |= ImGui::SliderFloat( "Angular Damping", &m_angularDamping, 0.0f, 5.0f, "%.2f" );
		needUpdateBodies |= ImGui::SliderFloat( "Gravity Scale", &m_gravityScale, 0.0f, 5.0f, "%.2f" );
		needUpdateBodies |= ImGui::Checkbox( "Bullet", &m_isBullet );

		if ( needUpdateBodies )
			CreateBodies();

		// Contrôle du volume sonore
		if ( ImGui::SliderFloat( "Sound Volume", &m_soundVolume, 0.0f, 100.0f, "%.1f%%" ) )
			m_audioManager.SetVolume( m_soundVolume );

		ImGui::Checkbox( "Enable Hit Events", &m_enableHitEvents );

		// --- Bloc gravité ---
		static const char* gravityPresets[] = { "Terre", "Lune", "Mars", "0", "Reverse", "Max" };
		static float presetValues[] = { -10.0f, -1.62f, -3.71f, 0.0f, 10.0f, -980.0f }; // -980 = ~100g
		static int presetIdx = 0;

		if ( ImGui::Combo( "Gravity Preset", &presetIdx, gravityPresets, IM_ARRAYSIZE( gravityPresets ) ) )
		{
			m_gravity = { 0.0f, presetValues[presetIdx] };
			b2World_SetGravity( m_worldId, m_gravity );
			// Optionnel : tu peux mettre à jour les bodies si nécessaire ici :
			// CreateBodies();
		}

		ImGui::Text( "Preset: %s", gravityPresets[presetIdx] );
		ImGui::End();
	}

private:
	// --- Audio ---
	AudioManager m_audioManager;
	float m_soundVolume;

	// --- Physics ---
	ShapeType m_shapeType;
	bool m_enableHitEvents;
	bool m_fixedRotation;
	b2Vec2 m_gravity;

	float m_linearDamping;
	float m_angularDamping;
	float m_gravityScale;
	bool m_isBullet;

	float m_restitution = 1.0f;
	float m_friction = 0.0f;
	float m_density = 1.0f;
	float m_shapeSize = 1.0f;
	float m_spacing = 1.0f;

	std::vector<b2ShapeId> m_shapes;


	// --- Shapes ---
	std::vector<b2BodyId> m_bodies;
	int m_shapeCount = 1;

	uint32_t ComputeEllipticColor( int ix, int iy, int cols, int rows )
	{
		float cx = ( cols - 1 ) * 0.5f;
		float cy = ( rows - 1 ) * 0.5f;
		float dx = static_cast<float>( ix ) - cx;
		float dy = static_cast<float>( iy ) - cy;
		float a = std::max( cx, 1.0f );
		float b = std::max( cy, 1.0f );
		float r = std::sqrt( ( dx * dx ) / ( a * a ) + ( dy * dy ) / ( b * b ) );
		r = std::clamp( r / std::sqrt( 2.0f ), 0.0f, 1.0f );
		float hue = r;
		return HSVtoRGB( hue, 0.75f, 1.0f );
	}

	uint32_t HSVtoRGB( float h, float s, float v )
	{
		float r, g, b;
		int i = int( h * 6.0f );
		float f = h * 6.0f - i;
		float p = v * ( 1.0f - s );
		float q = v * ( 1.0f - f * s );
		float t = v * ( 1.0f - ( 1.0f - f ) * s );
		switch ( i % 6 )
		{
			case 0:
				r = v, g = t, b = p;
				break;
			case 1:
				r = q, g = v, b = p;
				break;
			case 2:
				r = p, g = v, b = t;
				break;
			case 3:
				r = p, g = q, b = v;
				break;
			case 4:
				r = t, g = p, b = v;
				break;
			case 5:
				r = v, g = p, b = q;
				break;
		}
		uint32_t R = uint32_t( r * 255.0f );
		uint32_t G = uint32_t( g * 255.0f );
		uint32_t B = uint32_t( b * 255.0f );
		return ( R << 16 ) | ( G << 8 ) | B;
	}

	void InitializeAudio()
	{
		std::filesystem::path path = "data/audio/Ticks";
		if ( !std::filesystem::exists( path ) )
			path = "D:/Sound & Fx/audio/Ticks";
		m_audioManager.LoadFromDirectory( path.string() );
		m_audioManager.SetVolume( m_soundVolume );
	}

	void CreateGround()
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = { 0.0f, -1.0f };
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2Polygon box = b2MakeBox( 100.0f, 1.0f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material = b2DefaultSurfaceMaterial();
		shapeDef.material.customColor = 0x888888;
		b2CreatePolygonShape( groundId, &shapeDef, &box );
	}

	void CreateBodies()
	{
		// Détruit bodies précédents
		for ( b2BodyId id : m_bodies )
			if ( B2_IS_NON_NULL( id ) )
				b2DestroyBody( id );
		m_bodies.clear();
		m_shapes.clear(); // <-- on vide les shapes aussi

		int n = m_shapeCount;
		int cols = int( std::ceil( std::sqrt( n ) ) );
		int rows = ( n + cols - 1 ) / cols;

		float dx = m_spacing, dy = m_spacing;
		float totalW = dx * ( cols - 1 );
		float totalH = dy * ( rows - 1 );
		float startX = -totalW * 0.5f;
		float startY = 16.0f + totalH * 0.5f;

		for ( int i = 0; i < n; ++i )
		{
			int ix = i % cols;
			int iy = i / cols;

			float x = startX + ix * dx;
			float y = startY - iy * dy;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { x, y };
			bodyDef.linearVelocity = { 0.0f, -10.0f };
			bodyDef.motionLocks.angularZ = false;
			bodyDef.linearDamping = m_linearDamping;
			bodyDef.angularDamping = m_angularDamping;
			bodyDef.gravityScale = m_gravityScale;
			bodyDef.isBullet = m_isBullet;

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			m_bodies.push_back( bodyId );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = m_density;
			shapeDef.material = b2DefaultSurfaceMaterial();
			shapeDef.material.restitution = m_restitution;
			shapeDef.material.friction = m_friction;
			shapeDef.enableHitEvents = true;
			shapeDef.material.customColor = ComputeEllipticColor( ix, iy, cols, rows );

			ShapeType shapeType = static_cast<ShapeType>( m_shapeType );

			b2ShapeId shapeId = b2_nullShapeId;

			if ( shapeType == e_circleShape )
			{
				b2Circle circle = { { 0.0f, 0.0f }, 0.5f * m_shapeSize };
				shapeId = b2CreateCircleShape( bodyId, &shapeDef, &circle );
			}
			else if ( shapeType == e_capsuleShape )
			{
				b2Capsule capsule = { { -0.5f * m_shapeSize, 0.0f }, { 0.5f * m_shapeSize, 0.0f }, 0.25f * m_shapeSize };
				shapeId = b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			}
			else if ( shapeType == e_boxShape )
			{
				b2Polygon box = b2MakeBox( 0.5f * m_shapeSize, 0.5f * m_shapeSize );
				shapeId = b2CreatePolygonShape( bodyId, &shapeDef, &box );
			}
			else // Polygones réguliers 3-8 côtés
			{
				int sides = int( shapeType ) - int( e_polygon3 ) + 3;
				std::vector<b2Vec2> verts( sides );
				const float R = 0.5f * m_shapeSize;
				const float twoPi = 2.0f * b2_pi;
				for ( int j = 0; j < sides; ++j )
					verts[j] = { std::cos( j * twoPi / sides ) * R, std::sin( j * twoPi / sides ) * R };
				b2Hull hull = b2ComputeHull( verts.data(), sides );
				if ( hull.count > 0 )
				{
					b2Polygon poly = b2MakePolygon( &hull, 0 );
					shapeId = b2CreatePolygonShape( bodyId, &shapeDef, &poly );
				}
			}
			if ( B2_IS_NON_NULL( shapeId ) )
				m_shapes.push_back( shapeId ); // <-- AJOUT ICI
		}
	}
};

static int sampleTestCollisionColor = RegisterSample( "9:16", "TestCollisionColor", TestCollisionColor::Create );

class GUITest : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new GUITest( context );
	}

	explicit GUITest( SampleContext* context )
		: Sample( context )
		, m_counter( 42 ) // Juste pour l'exemple
	{
	}

	void Step() override
	{
	}
	void CreateBodies()
	{
	}
	void InitializeAudio()
	{
	}

	void UpdateGui() override
	{
		ImGuiIO& io = ImGui::GetIO();
		const char* title = "PROJECTILES";
		ImFont* titleFont = m_context->draw.m_largeFont ? m_context->draw.m_largeFont : ImGui::GetFont();
		ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoBackground;

		for ( int variant = 1; variant <= 5; ++variant )
		{
			ImGui::PushFont( titleFont );
			ImVec2 ts = ImGui::CalcTextSize( title );
			ImGui::PopFont();

			float padX = ts.x * 0.2f;
			float padY = ts.y * 0.5f;
			ImVec2 winSize( ts.x + padX * 2, ts.y + padY * 2 );

			float yPos = 0.10f + variant * 0.07f;

			ImGui::SetNextWindowPos( ImVec2( io.DisplaySize.x * 0.5f, io.DisplaySize.y * yPos ), ImGuiCond_Always,
									 ImVec2( 0.5f, 0.0f ) );
			ImGui::SetNextWindowSize( winSize, ImGuiCond_Always );
			ImGui::SetNextWindowBgAlpha( 0.0f );
			ImGui::Begin( ( "##v" + std::to_string( variant ) ).c_str(), nullptr, flags );

			ImDrawList* dl = ImGui::GetWindowDrawList();
			ImVec2 wp = ImGui::GetWindowPos();
			ImVec2 ws = ImGui::GetWindowSize();
			ImVec2 tp( wp.x + ( ws.x - ts.x ) * 0.5f, wp.y + ( ws.y - ts.y ) * 0.5f );

			switch ( variant )
			{
				case 1:
					dl->AddText( titleFont, titleFont->FontSize, tp, IM_COL32( 51, 192, 255, 255 ), title );
					break;
				case 2:
					dl->AddText( titleFont, titleFont->FontSize, ImVec2( tp.x + 1.5f, tp.y + 1.5f ), IM_COL32( 0, 0, 0, 80 ),
								 title );
					dl->AddText( titleFont, titleFont->FontSize, tp, IM_COL32( 255, 255, 255, 230 ), title );
					break;
				case 3:
					for ( int dx = -1; dx <= 1; ++dx )
						for ( int dy = -1; dy <= 1; ++dy )
							if ( dx || dy )
								dl->AddText( titleFont, titleFont->FontSize, ImVec2( tp.x + dx, tp.y + dy ),
											 IM_COL32( 0, 0, 0, 150 ), title );
					dl->AddText( titleFont, titleFont->FontSize, tp, IM_COL32( 255, 255, 255, 255 ), title );
					break;
				case 4:
					for ( int dx = -2; dx <= 2; dx += 2 )
						for ( int dy = -2; dy <= 2; dy += 2 )
							dl->AddText( titleFont, titleFont->FontSize, ImVec2( tp.x + dx, tp.y + dy ),
										 IM_COL32( 100, 200, 255, 60 ), title );
					dl->AddText( titleFont, titleFont->FontSize, tp, IM_COL32( 220, 255, 255, 255 ), title );
					break;
				case 5:
					dl->AddText( titleFont, titleFont->FontSize, tp, IM_COL32( 218, 230, 255, 235 ), title );
					break;
			}
			ImGui::End();
		}
	}

private:
	int m_counter;
};

static int sampleGUITest = RegisterSample( "9:16", "GUITest", GUITest::Create );

class Testtttttttttttttttttttttt : public Sample
{
public:
	struct ShapeUserData
	{
		bool shouldDestroyVisitors;
	};
	explicit Testtttttttttttttttttttttt( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 25.0f;
		}

		m_activeSensor.shouldDestroyVisitors = true;

		// Création du body dynamique
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { 0.0f, 0.0f };
		bodyDef.linearVelocity = { 0.0f, -5.0f };
		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

		// Filtrage pour les collisions
		static const uint16_t CATEGORY_DYNAMIC = 0x0002;
		static const uint16_t CATEGORY_SENSOR = 0x0004;
		static const uint16_t CATEGORY_WALL = 0x0001;

		// Création de la forme solide (rebondissante)
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material = b2DefaultSurfaceMaterial();
		shapeDef.material.restitution = 1.0f; // Rebond élevé
		shapeDef.material.friction = 0.0f;
		shapeDef.material.customColor = 0xFF00FF;
		shapeDef.filter.categoryBits = CATEGORY_DYNAMIC;
		shapeDef.filter.maskBits = CATEGORY_WALL | CATEGORY_SENSOR; // Interagit avec les murs et les sensors

		b2Polygon solidBox = b2MakeBox( 2.5f, 2.5f );
		b2CreatePolygonShape( bodyId, &shapeDef, &solidBox );

		// Création de la "peau" sensor
		b2ShapeDef sensorDef = b2DefaultShapeDef();
		sensorDef.isSensor = true;
		sensorDef.enableSensorEvents = true;
		sensorDef.userData = &m_activeSensor;
		sensorDef.filter.categoryBits = CATEGORY_SENSOR;
		sensorDef.filter.maskBits = CATEGORY_DYNAMIC | CATEGORY_WALL; // Peut détecter les dynamiques et les murs
		sensorDef.material.customColor = 0xAAAAAA;					  // Couleur du sensor

		// Définition du sensor avec une légère extension pour éviter les erreurs de collision
		float padding = 0.1f;
		b2Polygon sensorBox = b2MakeBox( 2.5f + padding, 2.5f + padding );
		b2CreatePolygonShape( bodyId, &sensorDef, &sensorBox );

		// Les murs
		{
			// Paramètres de l'arène (9:16)
			float arenaHalfWidth = 14.0f;  // Largeur intermédiaire
			float arenaHalfHeight = 25.0f; // Hauteur intermédiaire

			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material = b2DefaultSurfaceMaterial();
			shapeDef.material.restitution = 1.0f;
			shapeDef.material.friction = 0.0f;
			shapeDef.material.customColor = 0xAAAAAA;

			// Mur du bas
			b2Segment bottomWall = { { -arenaHalfWidth, -arenaHalfHeight }, { arenaHalfWidth, -arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &bottomWall );

			// Mur du haut
			b2Segment topWall = { { -arenaHalfWidth, arenaHalfHeight }, { arenaHalfWidth, arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &topWall );

			// Mur gauche
			b2Segment leftWall = { { -arenaHalfWidth, -arenaHalfHeight }, { -arenaHalfWidth, arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &leftWall );

			// Mur droit
			b2Segment rightWall = { { arenaHalfWidth, -arenaHalfHeight }, { arenaHalfWidth, arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &rightWall );
		}
	}

	void CreateRow( float y )
	{
		float shift = 5.0f;
		float xCenter = 0.5f * shift * m_columnCount;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.gravityScale = 0.0f;
		bodyDef.linearVelocity = { 0.0f, -5.0f };

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.enableSensorEvents = true;

		b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
		for ( int i = 0; i < m_columnCount; ++i )
		{
			float x = -0.5f * ( m_columnCount - 1 ) * shift + i * shift;
			bodyDef.position = { x, 20.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
	}

	void Step() override
	{
		Sample::Step();

		if ( m_stepCount == m_lastStepCount )
		{
			return;
		}

		std::set<b2BodyId> zombies;

		b2SensorEvents events = b2World_GetSensorEvents( m_worldId );
		for ( int i = 0; i < events.beginCount; ++i )
		{
			b2SensorBeginTouchEvent* event = events.beginEvents + i;

			ShapeUserData* userData = static_cast<ShapeUserData*>( b2Shape_GetUserData( event->sensorShapeId ) );
			if ( userData->shouldDestroyVisitors )
			{
				zombies.emplace( b2Shape_GetBody( event->visitorShapeId ) );
			}
			else
			{

				b2SurfaceMaterial surfaceMaterial = b2Shape_GetSurfaceMaterial( event->visitorShapeId );
				surfaceMaterial.customColor = b2_colorLime;
				b2Shape_SetSurfaceMaterial( event->visitorShapeId, surfaceMaterial );
			}
		}

		for ( int i = 0; i < events.endCount; ++i )
		{
			b2SensorEndTouchEvent* event = events.endEvents + i;

			if ( b2Shape_IsValid( event->visitorShapeId ) == false )
			{
				continue;
			}

			b2SurfaceMaterial surfaceMaterial = b2Shape_GetSurfaceMaterial( event->visitorShapeId );
			surfaceMaterial.customColor = 0;
			b2Shape_SetSurfaceMaterial( event->visitorShapeId, surfaceMaterial );
		}

		for ( b2BodyId bodyId : zombies )
		{
			b2DestroyBody( bodyId );
		}

		int delay = 0x0F;

		if ( ( m_stepCount & delay ) == 0 )
		{
			CreateRow( 0.5f * m_rowCount + 5.0f );
		}

		m_lastStepCount = m_stepCount;

		m_maxBeginCount = b2MaxInt( events.beginCount, m_maxBeginCount );
		m_maxEndCount = b2MaxInt( events.endCount, m_maxEndCount );
		DrawTextLine( "max begin touch events = %d", m_maxBeginCount );
		DrawTextLine( "max end touch events = %d", m_maxEndCount );
	}

	static Sample* Create( SampleContext* context )
	{
		return new Testtttttttttttttttttttttt( context );
	}

	static constexpr int m_columnCount = 5;
	static constexpr int m_rowCount = 5;
	int m_maxBeginCount;
	int m_maxEndCount;
	ShapeUserData m_passiveSensor;
	ShapeUserData m_activeSensor;
	int m_lastStepCount;
};

static int Testtttttttttttttttttttttt = RegisterSample( "9:16", "Testtttttttttttttttttttttt", Testtttttttttttttttttttttt::Create );

class ColumnColorCrash : public Sample
{
public:
	explicit ColumnColorCrash( SampleContext* context )
		: Sample( context )
	{
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 25.0f;
		}

		// Création des murs de l’arène 9:16
		{
			const float arenaHalfWidth = 10.0f;
			const float arenaHalfHeight = 10.0f;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material = b2DefaultSurfaceMaterial();
			shapeDef.material.restitution = 1.0f;
			shapeDef.material.friction = 0.0f;
			shapeDef.material.customColor = 0xAAAAAA;
			shapeDef.filter.categoryBits = CATEGORY_WALL;
			shapeDef.filter.maskBits = CATEGORY_ROW;

			b2Segment bottom = { { -arenaHalfWidth, -arenaHalfHeight }, { arenaHalfWidth, -arenaHalfHeight } };
			b2Segment top = { { -arenaHalfWidth, arenaHalfHeight }, { arenaHalfWidth, arenaHalfHeight } };
			b2Segment left = { { -arenaHalfWidth, -arenaHalfHeight }, { -arenaHalfWidth, arenaHalfHeight } };
			b2Segment right = { { arenaHalfWidth, -arenaHalfHeight }, { arenaHalfWidth, arenaHalfHeight } };

			b2CreateSegmentShape( groundId, &shapeDef, &bottom );
			b2CreateSegmentShape( groundId, &shapeDef, &top );
			b2CreateSegmentShape( groundId, &shapeDef, &left );
			b2CreateSegmentShape( groundId, &shapeDef, &right );
		}

		// Ajout d’un gros objet dynamique lent
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 20.0f, 0.0f };
			bodyDef.linearVelocity = { -1.0f, 0.0f }; // vers la gauche lentement
			bodyDef.gravityScale = 0.0f;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon bigBox = b2MakeBox( 2.5f, 2.5f ); // 5x5 total

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 100.0f;
			shapeDef.material = b2DefaultSurfaceMaterial();
			shapeDef.material.friction = 0.3f;
			shapeDef.material.restitution = 0.0f;
			shapeDef.material.customColor = 0x666666; // gris foncé
			shapeDef.filter.categoryBits = CATEGORY_BIGBOX;
			shapeDef.filter.maskBits = CATEGORY_ROW;


			b2CreatePolygonShape( bodyId, &shapeDef, &bigBox );
		}
	}

	void CreateRow( float y )
	{
		const float shift = 2.0f;
		const float radius = 0.5f;

		constexpr uint32_t colors[] = {
			0xFF3030, // rouge
			0xFFA500, // orange
			0xFFD700, // or
			0x00AA00, // vert foncé
			0x800080, // violet
			0x8B4513, // brun
			0xFF69B4, // rose
			0xAAAA00  // olive
		};

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.gravityScale = 0.0f;
		bodyDef.linearVelocity = { 0.0f, -5.0f };

		b2Circle circle = { { 0.0f, 0.0f }, radius };

		for ( int i = 0; i < m_columnCount; ++i )
		{
			float x = -0.5f * ( m_columnCount - 1 ) * shift + i * shift;
			bodyDef.position = { x, y };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.enableSensorEvents = true;
			shapeDef.material.customColor = colors[i % std::size( colors )];
			shapeDef.filter.categoryBits = CATEGORY_ROW;
			shapeDef.filter.maskBits = CATEGORY_ROW | CATEGORY_WALL | CATEGORY_BIGBOX;

			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
	}

	void Step() override
	{
		Sample::Step();

		if ( ( m_stepCount & 0x0F ) == 0 )
		{
			CreateRow( 9.5f );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new ColumnColorCrash( context );
	}

	static constexpr int m_columnCount = 8;
	static constexpr uint16_t CATEGORY_WALL = 0x0001;
	static constexpr uint16_t CATEGORY_ROW = 0x0002;
	static constexpr uint16_t CATEGORY_BIGBOX = 0x0004;
};

static int sampleColumnColorCrash = RegisterSample( "9:16", "ColumnColorCrash", ColumnColorCrash::Create );

class CageGrowth : public Sample
{
public:
	// Couleurs harmonisées (fond bleu conseillé)
	static constexpr uint32_t kCageColor = 0xFFC542;	   // Gold-yellow
	static constexpr uint32_t kProjectileColor = 0xFF5876; // Pink/orange néon

	explicit CageGrowth( SampleContext* context )
		: Sample( context )
	{
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 20.0f;
		}

		// Valeurs par défaut UNIQUEMENT ici :
		m_segmentCount = 3;
		m_maxSegments = 200;
		m_radius = 8.0f;
		m_thickness = 0.25f;
		m_projSize = 0.5f;
		m_projRestitution = 1.0f;
		m_projFriction = 0.0f;
		m_projSpeedX = 6.0f;
		m_projSpeedY = 0.0f;
		m_audioVolume = 40.0f;
		m_recreateCage = true;
		m_recreateProjectile = true;
		m_cageColor = kCageColor;
		m_projColor = kProjectileColor;
		m_cageMotorSpeed = 0.1f;
		m_cageDensity = 100.0f;
		m_projectileId = b2_nullBodyId;
		m_cageBodyId = b2_nullBodyId;

		InitializeAudio();
		b2World_SetGravity( m_worldId, m_gravity );
		CreateCage( m_segmentCount );
		CreateProjectile( { 0.0f, 0.0f }, { m_projSpeedX, m_projSpeedY } );
	}

	void InitializeAudio()
	{
		std::filesystem::path p = "data/audio/Ticks";
		if ( !std::filesystem::exists( p ) )
			p = "D:/Sound & Fx/audio/polygones139";
		m_audioManager.LoadFromDirectory( p.string() );
		m_audioManager.SetVolume( m_audioVolume );
	}

	void Step() override
	{
		Sample::Step();

		b2ContactEvents evts = b2World_GetContactEvents( m_worldId );
		for ( int i = 0; i < evts.hitCount; ++i )
		{
			const b2ContactHitEvent& hit = evts.hitEvents[i];

			bool aIsProj = B2_ID_EQUALS( b2Shape_GetBody( hit.shapeIdA ), m_projectileId );
			bool bIsProj = B2_ID_EQUALS( b2Shape_GetBody( hit.shapeIdB ), m_projectileId );
			bool aIsCage = B2_ID_EQUALS( b2Shape_GetBody( hit.shapeIdA ), m_cageBodyId );
			bool bIsCage = B2_ID_EQUALS( b2Shape_GetBody( hit.shapeIdB ), m_cageBodyId );

			if ( ( aIsProj && bIsCage ) || ( bIsProj && aIsCage ) )
			{
				m_prevSegmentCount = m_segmentCount;
				if ( m_segmentCount < m_maxSegments )
				{
					++m_segmentCount;
					if ( m_recreateCage )
						RecreateCage();
				}
				m_audioManager.HandleHitEffect( hit.point, hit.approachSpeed, m_stepCount );
				break;
			}
		}
		m_audioManager.PlayQueued();
	}

	void UpdateGui() override
	{
		ImGuiIO& io = ImGui::GetIO();

		// --- Fenêtre de paramètres modifiables ---
		if ( m_context->draw.m_showUI )
		{
			bool recreateCage = false, recreateProj = false, recreateAudio = false;

			ImGui::SetNextWindowPos( ImVec2( io.DisplaySize.x * 0.16f, io.DisplaySize.y * 0.52f ), ImGuiCond_Always,
									 ImVec2( 0.5f, 0.0f ) );
			ImGui::SetNextWindowSize( ImVec2( 380, 0 ), ImGuiCond_Always );
			ImGui::Begin( "Growing Cage Settings" );
			ImGui::Text( "Elapsed Time: %.2f s", ImGui::GetTime() );
			ImGui::Text( "Cage Parameters" );
			recreateCage |= ImGui::SliderInt( "Segments", &m_segmentCount, 3, m_maxSegments );
			recreateCage |= ImGui::SliderFloat( "Cage radius", &m_radius, 2.0f, 25.0f, "%.2f" );
			recreateCage |= ImGui::SliderFloat( "Cage thickness", &m_thickness, 0.05f, 2.0f, "%.2f" );
			recreateCage |= ImGui::SliderFloat( "Cage density", &m_cageDensity, 0.1f, 1000.0f, "%.2f" );

			// Couleur cage (ColorEdit3, couleur imposée par la palette)
			ImGui::TextColored( ImVec4( 1.0f, 0.77f, 0.26f, 1.0f ), "Cage color is brand preset!" );

			recreateCage |= ImGui::SliderFloat( "Cage rotation speed", &m_cageMotorSpeed, -5.0f, 5.0f, "%.2f rad/s" );

			ImGui::Separator();
			ImGui::Text( "World Parameters" );
			if ( ImGui::SliderFloat2( "Gravity (x, y)", &m_gravity.x, -30.0f, 30.0f, "%.2f" ) )
			{
				b2World_SetGravity( m_worldId, m_gravity );
			}
			if ( ImGui::Button( "Reset Gravity" ) )
			{
				m_gravity = { 0.0f, -10.0f };
				b2World_SetGravity( m_worldId, m_gravity );
			}

			ImGui::Separator();
			ImGui::Text( "Projectile Parameters" );
			recreateProj |= ImGui::SliderFloat( "Proj. size", &m_projSize, 0.1f, 5.0f, "%.2f" );
			recreateProj |= ImGui::SliderFloat( "Proj. restitution", &m_projRestitution, 0.0f, 2.0f, "%.2f" );
			recreateProj |= ImGui::SliderFloat( "Proj. friction", &m_projFriction, 0.0f, 1.0f, "%.2f" );
			recreateProj |= ImGui::SliderFloat( "Proj. speed X", &m_projSpeedX, -30.0f, 30.0f, "%.2f" );
			recreateProj |= ImGui::SliderFloat( "Proj. speed Y", &m_projSpeedY, -30.0f, 30.0f, "%.2f" );
			recreateProj |= ImGui::SliderFloat( "Proj. linear damping", &m_projLinearDamping, 0.0f, 10.0f, "%.2f" );
			recreateProj |= ImGui::SliderFloat( "Proj. angular damping", &m_projAngularDamping, 0.0f, 5.0f, "%.2f" );

			ImGui::Separator();
			recreateAudio |= ImGui::SliderFloat( "Audio volume", &m_audioVolume, 0.0f, 100.0f, "%.0f%%" );

			ImGui::Checkbox( "Recreate cage on change", &m_recreateCage );
			ImGui::SameLine();
			ImGui::Checkbox( "Recreate projectile on change", &m_recreateProjectile );

			ImGui::End();

			if ( recreateAudio )
				m_audioManager.SetVolume( m_audioVolume );
			if ( m_recreateCage && recreateCage )
				RecreateCage();
			if ( m_recreateProjectile && recreateProj )
				RecreateProjectile();
		}

		// --- HUD titre polygone (toujours visible, centré) ---
		{
			const char* polyName = "Unknown";
			if ( m_segmentCount >= 3 && m_segmentCount - 3 < (int)( sizeof( s_polygonNames ) / sizeof( s_polygonNames[0] ) ) )
				polyName = s_polygonNames[m_segmentCount - 3];

			ImFont* titleFont = m_context->draw.m_largeFont ? m_context->draw.m_largeFont : ImGui::GetFont();
			ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoBackground;

			ImGui::PushFont( titleFont );
			ImVec2 ts = ImGui::CalcTextSize( polyName );
			ImGui::PopFont();

			float padX = ts.x * 0.25f;
			float padY = ts.y * 0.55f;
			ImVec2 winSize( ts.x + padX * 2, ts.y + padY * 2 );

			ImGui::SetNextWindowPos( ImVec2( io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.20f ), ImGuiCond_Always,
									 ImVec2( 0.5f, 0.0f ) );
			ImGui::SetNextWindowSize( winSize, ImGuiCond_Always );
			ImGui::SetNextWindowBgAlpha( 0.0f );
			ImGui::Begin( "##PolyTitle", nullptr, flags );

			ImDrawList* dl = ImGui::GetWindowDrawList();
			ImVec2 wp = ImGui::GetWindowPos();
			ImVec2 ws = ImGui::GetWindowSize();
			ImVec2 tp( wp.x + ( ws.x - ts.x ) * 0.5f, wp.y + ( ws.y - ts.y ) * 0.5f );

			// Ombre bleu foncé (adapté fond bleu TikTok)
			dl->AddText( titleFont, titleFont->FontSize, ImVec2( tp.x + 1.5f, tp.y + 1.5f ), IM_COL32( 23, 65, 145, 110 ),
						 polyName );
			// Texte principal : blanc éclatant
			dl->AddText( titleFont, titleFont->FontSize, tp, IM_COL32( 255, 255, 255, 245 ), polyName );

			ImGui::End();
		}

		// --- HUD nombre de côtés (toujours visible, sous le titre) ---
		{
			std::string sidesText = "Sides: " + std::to_string( m_segmentCount );

			ImFont* sidesFont = m_context->draw.m_mediumFont ? m_context->draw.m_mediumFont : ImGui::GetFont();
			ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoBackground;

			ImGui::PushFont( sidesFont );
			ImVec2 ts = ImGui::CalcTextSize( sidesText.c_str() );
			ImGui::PopFont();

			float padX = ts.x * 0.18f;
			float padY = ts.y * 0.42f;
			ImVec2 winSize( ts.x + padX * 2, ts.y + padY * 2 );

			ImGui::SetNextWindowPos( ImVec2( io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.245f ), ImGuiCond_Always,
									 ImVec2( 0.5f, 0.0f ) );
			ImGui::SetNextWindowSize( winSize, ImGuiCond_Always );
			ImGui::SetNextWindowBgAlpha( 0.0f );
			ImGui::Begin( "##PolySides", nullptr, flags );

			ImDrawList* dl = ImGui::GetWindowDrawList();
			ImVec2 wp = ImGui::GetWindowPos();
			ImVec2 ws = ImGui::GetWindowSize();
			ImVec2 tp( wp.x + ( ws.x - ts.x ) * 0.5f, wp.y + ( ws.y - ts.y ) * 0.5f );

			// Ombre bleu foncé (même cohérence visuelle)
			dl->AddText( sidesFont, sidesFont->FontSize, ImVec2( tp.x + 1.5f, tp.y + 1.5f ), IM_COL32( 23, 65, 145, 80 ),
						 sidesText.c_str() );
			// Texte principal : jaune lumineux, complémentaire fond bleu
			dl->AddText( sidesFont, sidesFont->FontSize, tp, IM_COL32( 255, 197, 66, 240 ), sidesText.c_str() );

			ImGui::End();
		}

		m_audioManager.DrawHitEffects( &m_context->draw, m_stepCount );
	}

	void CreateCage( int segmentCount )
	{
		if ( B2_IS_NON_NULL( m_cageBodyId ) )
		{
			b2DestroyBody( m_cageBodyId );
			m_cageBodyId = b2_nullBodyId;
		}

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = { 0.0f, 0.0f };
		m_cageBodyId = b2CreateBody( m_worldId, &bd );

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.customColor = m_cageColor;
		sd.material.friction = 0.0f;
		sd.density = m_cageDensity;
		sd.isSensor = false;

		float dAng = 2.0f * b2_pi / float( segmentCount );

		for ( int i = 0; i < segmentCount; ++i )
		{
			float a0 = i * dAng;
			float a1 = ( i + 1 ) * dAng;

			b2Vec2 p0 = { m_radius * std::cos( a0 ), m_radius * std::sin( a0 ) };
			b2Vec2 p1 = { m_radius * std::cos( a1 ), m_radius * std::sin( a1 ) };
			b2Vec2 c = 0.5f * ( p0 + p1 );
			float ang = std::atan2( p1.y - p0.y, p1.x - p0.x );
			float len = b2Distance( p0, p1 );

			b2Polygon rect = b2MakeOffsetBox( 0.5f * len, 0.5f * m_thickness, c, b2MakeRot( ang ) );
			b2CreatePolygonShape( m_cageBodyId, &sd, &rect );
		}

		// Ajout du pivot
		b2BodyDef pivotDef = b2DefaultBodyDef();
		pivotDef.position = { 0.0f, 0.0f };
		b2BodyId pivot = b2CreateBody( m_worldId, &pivotDef );

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = pivot;
		jd.base.bodyIdB = m_cageBodyId;
		jd.enableMotor = true;
		jd.motorSpeed = m_cageMotorSpeed;
		jd.maxMotorTorque = 1e5f;
		b2CreateRevoluteJoint( m_worldId, &jd );
	}

	void CreateProjectile( const b2Vec2& pos, const b2Vec2& velocity )
	{
		if ( B2_IS_NON_NULL( m_projectileId ) )
		{
			b2DestroyBody( m_projectileId );
			m_projectileId = b2_nullBodyId;
		}
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = velocity;
		bd.isBullet = true;
		bd.linearDamping = m_projLinearDamping;
		bd.angularDamping = m_projAngularDamping;

		m_projectileId = b2CreateBody( m_worldId, &bd );

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 1.0f;
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.restitution = m_projRestitution;
		sd.material.friction = m_projFriction;
		sd.material.customColor = m_projColor;
		sd.enableHitEvents = true;

		b2Circle circle = { { 0.0f, 0.0f }, m_projSize };
		b2CreateCircleShape( m_projectileId, &sd, &circle );
	}

	void RecreateCage()
	{
		CreateCage( m_segmentCount );
	}
	void RecreateProjectile()
	{
		CreateProjectile( { 0.0f, 0.0f }, { m_projSpeedX, m_projSpeedY } );
	}

	static constexpr const char* s_polygonNames[] = { "Triangle",
													  "Quadrilateral",
													  "Pentagon",
													  "Hexagon",
													  "Heptagon",
													  "Octagon",
													  "Nonagon",
													  "Decagon",
													  "Hendecagon",
													  "Dodecagon",
													  "Tridecagon",
													  "Tetradecagon",
													  "Pentadecagon",
													  "Hexadecagon",
													  "Heptadecagon",
													  "Octadecagon",
													  "Enneadecagon",
													  "Icosagon",
													  "Icosikaihenagon",
													  "Icosikaidigon",
													  "Icosikaitrigon",
													  "Icosikaitetragon",
													  "Icosikaipentagon",
													  "Icosikaihexagon",
													  "Icosikaiheptagon",
													  "Icosikaioctagon",
													  "Icosikaienneagon",
													  "Triacontagon",
													  "Triacontakaihenagon",
													  "Triacontakaidigon",
													  "Triacontakaitrigon",
													  "Triacontakaitetragon",
													  "Triacontakaipentagon",
													  "Triacontakaihexagon",
													  "Triacontakaiheptagon",
													  "Triacontakaioctagon",
													  "Triacontakaienneagon",
													  "Tetracontagon",
													  "Tetracontakaihenagon",
													  "Tetracontakaidigon",
													  "Tetracontakaitrigon",
													  "Tetracontakaitetragon",
													  "Tetracontakaipentagon",
													  "Tetracontakaihexagon",
													  "Tetracontakaiheptagon",
													  "Tetracontakaioctagon",
													  "Tetracontakaienneagon",
													  "Pentacontagon",
													  "Pentacontakaihenagon",
													  "Pentacontakaidigon",
													  "Pentacontakaitrigon",
													  "Pentacontakaitetragon",
													  "Pentacontakaipentagon",
													  "Pentacontakaihexagon",
													  "Pentacontakaiheptagon",
													  "Pentacontakaioctagon",
													  "Pentacontakaienneagon",
													  "Hexacontagon",
													  "Hexacontakaihenagon",
													  "Hexacontakaidigon",
													  "Hexacontakaitrigon",
													  "Hexacontakaitetragon",
													  "Hexacontakaipentagon",
													  "Hexacontakaihexagon",
													  "Hexacontakaiheptagon",
													  "Hexacontakaioctagon",
													  "Hexacontakaienneagon",
													  "Heptacontagon",
													  "Heptacontakaihenagon",
													  "Heptacontakaidigon",
													  "Heptacontakaitrigon",
													  "Heptacontakaitetragon",
													  "Heptacontakaipentagon",
													  "Heptacontakaihexagon",
													  "Heptacontakaiheptagon",
													  "Heptacontakaioctagon",
													  "Heptacontakaienneagon",
													  "Octacontagon",
													  "Octacontakaihenagon",
													  "Octacontakaidigon",
													  "Octacontakaitrigon",
													  "Octacontakaitetragon",
													  "Octacontakaipentagon",
													  "Octacontakaihexagon",
													  "Octacontakaiheptagon",
													  "Octacontakaioctagon",
													  "Octacontakaienneagon",
													  "Enneacontagon",
													  "Enneacontakaihenagon",
													  "Enneacontakaidigon",
													  "Enneacontakaitrigon",
													  "Enneacontakaitetragon",
													  "Enneacontakaipentagon",
													  "Enneacontakaihexagon",
													  "Enneacontakaiheptagon",
													  "Enneacontakaioctagon",
													  "Enneacontakaienneagon",
													  "Hectogon",
													  "Hectogonkaimonogon",
													  "Hectogonkaidigon",
													  "Hectogonkaitrigon",
													  "Hectogonkaitetragon",
													  "Hectogonkaipentagon",
													  "Hectogonkaihexagon",
													  "Hectogonkaiheptagon",
													  "Hectogonkaioctagon",
													  "Hectogonkainonagon",
													  "Hectogonkaidecagon",
													  "Hectogonkaihendecagon",
													  "Hectogonkaidodecagon",
													  "Hectogonkaitridecagon",
													  "Hectogonkaitetradecagon",
													  "Hectogonkaipentadecagon",
													  "Hectogonkaihexadecagon",
													  "Hectogonkaiheptadecagon",
													  "Hectogonkaioctadecagon",
													  "Hectogonkaienneadecagon",
													  "Hectogonkaiicosagon",
													  "Hectogonkaiicosikaihenagon",
													  "Hectogonkaiicosikaidigon",
													  "Hectogonkaiicosikaitrigon",
													  "Hectogonkaiicosikaitetragon",
													  "Hectogonkaiicosikaipentagon",
													  "Hectogonkaiicosikaihexagon",
													  "Hectogonkaiicosikaiheptagon",
													  "Hectogonkaiicosikaioctagon",
													  "Hectogonkaiicosikaienneagon",
													  "Hectogonkaitriacontagon",
													  "Hectogonkaitriacontakaihenagon",
													  "Hectogonkaitriacontakaidigon",
													  "Hectogonkaitriacontakaitrigon",
													  "Hectogonkaitriacontakaitetragon",
													  "Hectogonkaitriacontakaipentagon",
													  "Hectogonkaitriacontakaihexagon",
													  "Hectogonkaitriacontakaiheptagon",
													  "Hectogonkaitriacontakaioctagon",
													  "Hectogonkaitriacontakaienneagon" };

	static Sample* Create( SampleContext* context )
	{
		return new CageGrowth( context );
	}

private:
	b2BodyId m_cageBodyId;
	b2BodyId m_projectileId;
	b2Vec2 m_gravity = { 0.0f, -40.0f };
	int m_segmentCount, m_prevSegmentCount, m_maxSegments;
	float m_radius, m_thickness;
	float m_projSize, m_projRestitution, m_projFriction;
	float m_projSpeedX, m_projSpeedY;
	float m_projLinearDamping = 0.01f;
	float m_projAngularDamping = 0.0f;
	float m_audioVolume;
	bool m_recreateCage, m_recreateProjectile;
	uint32_t m_cageColor;
	uint32_t m_projColor;
	float m_cageMotorSpeed;
	float m_cageDensity = 100.0f;

	AudioManager m_audioManager;
};

static int sampleCageGrowth = RegisterSample( "9:16", "CageGrowth", CageGrowth::Create );

class CageDuplication : public Sample
{
public:
	//---------------------------------------------------------------
	// Types
	//---------------------------------------------------------------
	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape,
		e_polygon3,
		e_polygon4,
		e_polygon5,
		e_polygon6,
		e_polygon7,
		e_polygon8,
	};

	struct ShapeConfig
	{
		ShapeType type;
		uint32_t color;
	};

	struct b2ShapeIdHash
	{
		size_t operator()( const b2ShapeId& id ) const noexcept
		{
			return std::hash<uint64_t>()( b2StoreShapeId( id ) );
		}
	};
	struct b2ShapeIdEqual
	{
		bool operator()( const b2ShapeId& a, const b2ShapeId& b ) const noexcept
		{
			return B2_ID_EQUALS( a, b );
		}
	};

	using TouchSet = std::unordered_set<b2ShapeId, b2ShapeIdHash, b2ShapeIdEqual>;

	static Sample* Create( SampleContext* context )
	{
		return new CageDuplication( context );
	}

	static constexpr uint32_t kDefaultColors[2] = { 0xE53F3F, 0x3F6FE5 };

	//---------------------------------------------------------------
	// Construction
	//---------------------------------------------------------------
	explicit CageDuplication( SampleContext* context )
		: Sample( context )
		, m_enableHitEvents( true )
		, m_soundVolume( 50.0f )
		, m_linearDamping( 0.5f )
		, m_angularDamping( 0.2f )
		, m_gravityScale( 0.0f )
		, m_isBullet( false )
		, m_gravity{ 0.0f, -10.0f }
		, m_motionLocks{ false, false, false }
		, m_angVelMin( -10.0f )
		, m_angVelMax( 10.0f )
		, m_speedMin( 4.5f )
		, m_speedMax( 5.0f )
	{
		m_context->enableSleep = true;
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 25.0f;
		}
		b2World_SetGravity( m_worldId, m_gravity );
		InitializeAudio();
		CreateBodies();

		// Les murs
		{
			// Paramètres de l'arène (9:16)
			float arenaHalfWidth = 14.0f;  // Largeur intermédiaire
			float arenaHalfHeight = 25.0f; // Hauteur intermédiaire

			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material = b2DefaultSurfaceMaterial();
			shapeDef.material.restitution = 1.0f;
			shapeDef.material.friction = 0.0f;
			shapeDef.material.customColor = 0xAAAAAA;

			// Mur du bas
			b2Segment bottomWall = { { -arenaHalfWidth, -arenaHalfHeight }, { arenaHalfWidth, -arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &bottomWall );

			// Mur du haut
			b2Segment topWall = { { -arenaHalfWidth, arenaHalfHeight }, { arenaHalfWidth, arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &topWall );

			// Mur gauche
			b2Segment leftWall = { { -arenaHalfWidth, -arenaHalfHeight }, { -arenaHalfWidth, arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &leftWall );

			// Mur droit
			b2Segment rightWall = { { arenaHalfWidth, -arenaHalfHeight }, { arenaHalfWidth, arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &rightWall );
		}
	}

	//---------------------------------------------------------------
	// Step
	//---------------------------------------------------------------
	void Step() override
	{
		Sample::Step();

		b2SensorEvents events = b2World_GetSensorEvents( m_worldId );
		const b2Vec2 spawnCenter{ 0.0f, 0.0f };

		// --------------------------- BEGIN TOUCH -----------------------------
		for ( int i = 0; i < events.beginCount; ++i )
		{
			const b2SensorBeginTouchEvent& e = events.beginEvents[i];

			if ( !m_shapeColorMap.count( e.visitorShapeId ) )
				continue;
			if ( !m_segmentOriginalColor.count( e.sensorShapeId ) )
				continue;

			m_touchingSegments[e.visitorShapeId].insert( e.sensorShapeId );

			uint32_t col = m_shapeColorMap[e.visitorShapeId];
			b2SurfaceMaterial mat = b2Shape_GetSurfaceMaterial( e.sensorShapeId );
			mat.customColor = col;
			b2Shape_SetSurfaceMaterial( e.sensorShapeId, mat );
		}

		// ---------------------------- END TOUCH ------------------------------
		for ( int i = 0; i < events.endCount; ++i )
		{
			const b2SensorEndTouchEvent& e = events.endEvents[i];

			if ( !m_shapeColorMap.count( e.visitorShapeId ) )
				continue;
			if ( !m_segmentOriginalColor.count( e.sensorShapeId ) )
				continue;

			auto& set = m_touchingSegments[e.visitorShapeId];
			set.erase( e.sensorShapeId );

			if ( set.empty() )
			{
				m_touchingSegments.erase( e.visitorShapeId );

				if ( m_alreadyDuplicated.insert( e.visitorShapeId ).second )
				{
					int genLevel = m_generationLevel[e.visitorShapeId];
					for ( int k = 0; k < 2; ++k )
						CreateProjectile( spawnCenter, genLevel + 1 );

					if ( m_enableHitEvents )
						m_audioManager.PlaySpawnSound();
				}
			}

			uint32_t baseCol = m_segmentOriginalColor[e.sensorShapeId];
			b2SurfaceMaterial mat = b2Shape_GetSurfaceMaterial( e.sensorShapeId );
			mat.customColor = baseCol;
			b2Shape_SetSurfaceMaterial( e.sensorShapeId, mat );
		}

		// --------------------- Effets sonores (impacts physiques) ---------------------
		if ( m_enableHitEvents )
		{
			b2ContactEvents evts = b2World_GetContactEvents( m_worldId );
			for ( int i = 0; i < evts.hitCount; ++i )
				m_audioManager.HandleHitEffect( evts.hitEvents[i].point, evts.hitEvents[i].approachSpeed, m_stepCount );
		}
		m_audioManager.PlayQueued();
		m_audioManager.DrawHitEffects( &m_context->draw, m_stepCount );
	}

	//---------------------------------------------------------------
	// GUI
	//---------------------------------------------------------------
	void UpdateGui() override
	{
		ImGuiIO& io = ImGui::GetIO();

		// --- Fenêtre de paramètres contrôlables ---
		if ( m_context->draw.m_showUI )
		{
			ImGui::SetNextWindowPos( ImVec2( io.DisplaySize.x * 0.15f, io.DisplaySize.y * 0.54f ), ImGuiCond_Always,
									 ImVec2( 0.5f, 0.0f ) );
			ImGui::SetNextWindowSize( ImVec2( 400, 0 ), ImGuiCond_Always );
			ImGui::Begin( "BluePrint Settings" );

			const char* shapeNames[] = { "Circle",	 "Capsule", "Box",		"Triangle", "Quad",
										 "Pentagon", "Hexagon", "Heptagon", "Octagon" };
			bool recreate = false;

			bool lockChanged = false;
			lockChanged |= ImGui::Checkbox( "Lock X", &m_motionLocks.linearX );
			ImGui::SameLine();
			lockChanged |= ImGui::Checkbox( "Lock Y", &m_motionLocks.linearY );
			ImGui::SameLine();
			lockChanged |= ImGui::Checkbox( "Lock Rot", &m_motionLocks.angularZ );

			if ( ImGui::SliderFloat( "Shape Size", &m_shapeSize, 0.1f, 5.0f, "%.2f" ) )
				recreate = true;
			if ( ImGui::SliderFloat( "Spacing", &m_spacing, 0.5f, 5.0f, "%.2f" ) )
				recreate = true;

			bool physChanged = false;
			physChanged |= ImGui::SliderFloat( "Restitution", &m_restitution, 0.0f, 1.5f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Friction", &m_friction, 0.0f, 1.0f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Density", &m_density, 0.01f, 10.0f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Linear Damping", &m_linearDamping, 0.0f, 10.0f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Angular Damping", &m_angularDamping, 0.0f, 5.0f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Gravity Scale", &m_gravityScale, 0.0f, 5.0f, "%.2f" );
			physChanged |= ImGui::Checkbox( "Bullet", &m_isBullet );

			physChanged |= ImGui::SliderFloat( "Min Speed", &m_speedMin, 0.0f, 30.0f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Max Speed", &m_speedMax, 0.0f, 30.0f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Min Ang Vel", &m_angVelMin, -50.0f, 0.0f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Max Ang Vel", &m_angVelMax, 0.0f, 50.0f, "%.2f" );

			if ( ImGui::SliderFloat( "Sound Volume", &m_soundVolume, 0.0f, 100.0f, "%.0f%%" ) )
				m_audioManager.SetVolume( m_soundVolume );
			ImGui::Checkbox( "Enable Hit Events", &m_enableHitEvents );

			ImGui::Separator();
			bool cageChanged = false;
			cageChanged |= ImGui::SliderInt( "Segments par cage", &m_cageSegments, 4, 128 );
			cageChanged |= ImGui::SliderFloat( "Épaisseur cages", &m_cageThickness, 0.05f, 2.0f, "%.2f" );
			cageChanged |= ImGui::SliderFloat( "Vitesse rotation cages", &m_cageMotorSpeed, -5.0f, 5.0f, "%.2f rad/s" );
			ImGui::End();

			if ( recreate || lockChanged || physChanged || cageChanged )
				CreateBodies();
		}

		// --- HUD titre PROJECTILES (toujours visible, centré, ombre douce) ---
		{
			ImFont* titleFont = m_context->draw.m_largeFont ? m_context->draw.m_largeFont : ImGui::GetFont();
			const char* title = "PROJECTILES";
			ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoBackground;

			ImGui::PushFont( titleFont );
			ImVec2 ts = ImGui::CalcTextSize( title );
			ImGui::PopFont();

			float padX = ts.x * 0.2f;
			float padY = ts.y * 0.5f;
			ImVec2 winSize( ts.x + padX * 2, ts.y + padY * 2 );

			ImGui::SetNextWindowPos( ImVec2( io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.17f ), ImGuiCond_Always,
									 ImVec2( 0.5f, 0.0f ) );
			ImGui::SetNextWindowSize( winSize, ImGuiCond_Always );
			ImGui::SetNextWindowBgAlpha( 0.0f );
			ImGui::Begin( "##HUDTitleOmbre", nullptr, flags );

			ImDrawList* dl = ImGui::GetWindowDrawList();
			ImVec2 wp = ImGui::GetWindowPos();
			ImVec2 ws = ImGui::GetWindowSize();
			ImVec2 tp( wp.x + ( ws.x - ts.x ) * 0.5f, wp.y + ( ws.y - ts.y ) * 0.5f );

			// Ombre douce (décalage +1.5, alpha 80)
			dl->AddText( titleFont, titleFont->FontSize, ImVec2( tp.x + 1.5f, tp.y + 1.5f ), IM_COL32( 0, 0, 0, 80 ), title );
			// Texte principal blanc légèrement adouci (alpha 230)
			dl->AddText( titleFont, titleFont->FontSize, tp, IM_COL32( 255, 255, 255, 230 ), title );

			ImGui::End();
		}

		// --- HUD compteur de projectiles (toujours visible, sous le titre) ---
		{
			ImFont* largeFont = m_context->draw.m_largeFont ? m_context->draw.m_largeFont : ImGui::GetFont();
			std::string countText = std::to_string( m_projectileCount );

			ImGui::PushFont( largeFont );
			ImVec2 countSize = ImGui::CalcTextSize( countText.c_str() );
			ImGui::PopFont();

			const float padX = 40.0f;
			const float padY = 25.0f;
			ImVec2 winSize( countSize.x + padX * 2, countSize.y + padY * 2 );

			ImGui::SetNextWindowPos( ImVec2( io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.22f ), ImGuiCond_Always,
									 ImVec2( 0.5f, 0.0f ) );
			ImGui::SetNextWindowSize( winSize, ImGuiCond_Always );
			ImGui::SetNextWindowBgAlpha( 0.0f );

			ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
									 ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoBackground;

			ImGui::Begin( "##HUDCountOmbre", nullptr, flags );

			ImDrawList* dl = ImGui::GetWindowDrawList();
			ImVec2 wp = ImGui::GetWindowPos();
			ImVec2 ws = ImGui::GetWindowSize();
			ImVec2 tp( wp.x + ( ws.x - countSize.x ) * 0.5f, wp.y + ( ws.y - countSize.y ) * 0.5f );

			dl->AddText( largeFont, largeFont->FontSize, ImVec2( tp.x + 2.0f, tp.y + 2.0f ), IM_COL32( 0, 0, 0, 100 ),
						 countText.c_str() );
			dl->AddText( largeFont, largeFont->FontSize, tp, IM_COL32( 255, 255, 255, 240 ), countText.c_str() );

			ImGui::End();
		}
	}

private:
	// Audio
	AudioManager m_audioManager;
	float m_soundVolume;

	// Physique
	bool m_enableHitEvents;
	b2MotionLocks m_motionLocks;
	b2Vec2 m_gravity;
	float m_linearDamping;
	float m_angularDamping;
	float m_gravityScale;
	bool m_isBullet;
	float m_restitution = 1.0f;
	float m_friction = 0.0f;
	float m_density = 1.0f;
	float m_shapeSize = 1.0f;
	float m_spacing = 1.0f;
	// Nouvelles plages pour la vélocité
	float m_speedMin = 5.0f, m_speedMax = 5.0f;
	float m_angVelMin = -10.0f, m_angVelMax = 10.0f;

	// Équipes
	ShapeConfig m_shapes[2] = { { e_circleShape, 0xE53F3F }, { e_boxShape, 0x3F6FE5 } };
	std::unordered_map<b2ShapeId, uint32_t, b2ShapeIdHash, b2ShapeIdEqual> m_shapeColorMap;

	// Cage
	float m_cageThickness = 0.25f;
	float m_cageMotorSpeed = 0.0f;
	int m_cageSegments = 50;
	std::vector<b2BodyId> m_cageBodies;
	// compteur de projectiles
	int m_projectileCount = 0;

	// Projectiles
	std::vector<b2BodyId> m_bodies;

	// Segments → couleur d’origine
	std::unordered_map<b2ShapeId, uint32_t, b2ShapeIdHash, b2ShapeIdEqual> m_segmentOriginalColor;

	// Gestion duplication/génération
	std::unordered_set<b2ShapeId, b2ShapeIdHash, b2ShapeIdEqual> m_alreadyDuplicated;
	std::unordered_map<b2ShapeId, int, b2ShapeIdHash, b2ShapeIdEqual> m_generationLevel;

	// Ensemble des segments actuellement touchés par chaque projectile
	std::unordered_map<b2ShapeId, TouchSet, b2ShapeIdHash, b2ShapeIdEqual> m_touchingSegments;

	// Audio init
	void InitializeAudio()
	{
		std::filesystem::path p = "data/audio/Ticks";
		if ( !std::filesystem::exists( p ) )
			p = "D:/Sound & Fx/audio/Ticks";
		m_audioManager.LoadFromDirectory( p.string() );
		m_audioManager.SetVolume( m_soundVolume );
	}

	// Centralise la création d’un projectile (utilisé partout)
	void CreateProjectile( const b2Vec2& pos, int genLevel )
	{
		float speed = RandomFloatRange( m_speedMin, m_speedMax );
		float angle = RandomFloatRange( -b2_pi, b2_pi );

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { speed * std::cos( angle ), speed * std::sin( angle ) };
		bd.angularVelocity = RandomFloatRange( m_angVelMin, m_angVelMax );
		bd.motionLocks = m_motionLocks;
		bd.linearDamping = m_linearDamping;
		bd.angularDamping = m_angularDamping;
		bd.gravityScale = m_gravityScale;
		bd.isBullet = m_isBullet;

		b2BodyId body = b2CreateBody( m_worldId, &bd );
		m_bodies.push_back( body );

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = m_density;
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.restitution = m_restitution;
		sd.material.friction = m_friction;
		sd.material.customColor = ( uint32_t( RandomFloatRange( 0, 1 ) * 255 ) << 16 ) |
								  ( uint32_t( RandomFloatRange( 0, 1 ) * 255 ) << 8 ) |
								  uint32_t( RandomFloatRange( 0, 1 ) * 255 );
		sd.isSensor = true;
		sd.enableSensorEvents = true;
		sd.enableHitEvents = true;

		b2Circle c{ { 0.0f, 0.0f }, 0.5f * m_shapeSize };
		b2ShapeId sid = b2CreateCircleShape( body, &sd, &c );
		m_shapeColorMap[sid] = sd.material.customColor;
		m_generationLevel[sid] = genLevel;
		m_projectileCount++;
	}

	// Cage sensor
	b2BodyId CreateFullCage( b2WorldId worldId, float radius, float thickness, int segCount, float speed )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = { 0.0f, 0.0f };
		b2BodyId cage = b2CreateBody( worldId, &bd );

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.friction = 0.1f;
		sd.material.customColor = 0x44FFD5;
		sd.isSensor = true;
		sd.enableSensorEvents = true;

		float dAng = 2.0f * b2_pi / segCount;
		for ( int i = 0; i < segCount; ++i )
		{
			float a0 = i * dAng;
			float a1 = ( i + 1 ) * dAng;

			b2Vec2 p0 = { radius * std::cos( a0 ), radius * std::sin( a0 ) };
			b2Vec2 p1 = { radius * std::cos( a1 ), radius * std::sin( a1 ) };
			b2Vec2 c = 0.5f * ( p0 + p1 );
			float ang = std::atan2( p1.y - p0.y, p1.x - p0.x );
			float len = b2Distance( p0, p1 );

			b2Polygon rect = b2MakeOffsetBox( 0.5f * len, 0.5f * thickness, c, b2MakeRot( ang ) );
			b2ShapeId sid = b2CreatePolygonShape( cage, &sd, &rect );
			m_segmentOriginalColor[sid] = sd.material.customColor;
		}

		b2BodyDef pivotDef;
		pivotDef = b2DefaultBodyDef();
		pivotDef.position = { 0.0f, 0.0f };
		b2BodyId pivot = b2CreateBody( worldId, &pivotDef );

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = pivot;
		jd.base.bodyIdB = cage;
		jd.enableMotor = true;
		jd.motorSpeed = speed;
		jd.maxMotorTorque = 1e5f;
		b2CreateRevoluteJoint( worldId, &jd );

		return cage;
	}

	// Création bodies + reset duplications/overlap/génération
	void CreateBodies()
	{
		m_alreadyDuplicated.clear();
		m_generationLevel.clear();
		m_touchingSegments.clear();
		g_randomSeed = (uint32_t)std::time( nullptr );

		for ( b2BodyId id : m_bodies )
			if ( B2_IS_NON_NULL( id ) )
				b2DestroyBody( id );
		m_bodies.clear();

		for ( b2BodyId id : m_cageBodies )
			if ( B2_IS_NON_NULL( id ) )
				b2DestroyBody( id );
		m_cageBodies.clear();

		m_shapeColorMap.clear();
		m_segmentOriginalColor.clear();

		// Cage sensor
		float radius = 8.0f;
		m_cageBodies.push_back( CreateFullCage( m_worldId, radius, m_cageThickness, m_cageSegments, m_cageMotorSpeed ) );

		// 1 projectile central
		m_shapes[0].color = ( uint32_t( RandomFloatRange( 0, 1 ) * 255 ) << 16 ) |
							( uint32_t( RandomFloatRange( 0, 1 ) * 255 ) << 8 ) | uint32_t( RandomFloatRange( 0, 1 ) * 255 );

		m_projectileCount = 0; // Reset à chaque reset/simulation
		CreateProjectile( { 0.0f, 0.0f }, 0 );
		m_projectileCount++; // Incrémente ici
	}
};
static int sampleCageDuplication = RegisterSample( "9:16", "CageDuplication", CageDuplication::Create );

class CageDuplicationImageGuess : public Sample
{
public:
	//---------------------------------------------------------------
	// Types
	//---------------------------------------------------------------
	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape,
		e_polygon3,
		e_polygon4,
		e_polygon5,
		e_polygon6,
		e_polygon7,
		e_polygon8,
	};

	struct ShapeConfig
	{
		ShapeType type;
		uint32_t color;
	};

	struct b2ShapeIdHash
	{
		size_t operator()( const b2ShapeId& id ) const noexcept
		{
			return std::hash<uint64_t>()( b2StoreShapeId( id ) );
		}
	};
	struct b2ShapeIdEqual
	{
		bool operator()( const b2ShapeId& a, const b2ShapeId& b ) const noexcept
		{
			return B2_ID_EQUALS( a, b );
		}
	};

	using TouchSet = std::unordered_set<b2ShapeId, b2ShapeIdHash, b2ShapeIdEqual>;

	static Sample* Create( SampleContext* context )
	{
		return new CageDuplicationImageGuess( context );
	}

	static constexpr uint32_t kDefaultColors[2] = { 0x00FF00, 0x00FF00 }; // Vert

	//---------------------------------------------------------------
	// Construction
	//---------------------------------------------------------------
	explicit CageDuplicationImageGuess( SampleContext* context )
		: Sample( context )
		, m_enableHitEvents( true )
		, m_soundVolume( 50.0f )
		, m_linearDamping( 0.5f )
		, m_angularDamping( 0.2f )
		, m_gravityScale( 0.0f )
		, m_isBullet( false )
		, m_gravity{ 0.0f, -10.0f }
		, m_motionLocks{ false, false, false }
		, m_angVelMin( -10.0f )
		, m_angVelMax( 10.0f )
		, m_speedMin( 4.5f )
		, m_speedMax( 5.0f )
	{
		m_context->enableSleep = true;
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 25.0f;
		}
		b2World_SetGravity( m_worldId, m_gravity );
		InitializeAudio();
		CreateBodies();

		// Les murs (verts)
		{
			float arenaHalfWidth = 14.0f;
			float arenaHalfHeight = 25.0f;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material = b2DefaultSurfaceMaterial();
			shapeDef.material.restitution = 1.0f;
			shapeDef.material.friction = 0.0f;
			shapeDef.material.customColor = 0x00FF00; // Vert

			b2Segment bottomWall = { { -arenaHalfWidth, -arenaHalfHeight }, { arenaHalfWidth, -arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &bottomWall );

			b2Segment topWall = { { -arenaHalfWidth, arenaHalfHeight }, { arenaHalfWidth, arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &topWall );

			b2Segment leftWall = { { -arenaHalfWidth, -arenaHalfHeight }, { -arenaHalfWidth, arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &leftWall );

			b2Segment rightWall = { { arenaHalfWidth, -arenaHalfHeight }, { arenaHalfWidth, arenaHalfHeight } };
			b2CreateSegmentShape( groundId, &shapeDef, &rightWall );
		}
	}

	//---------------------------------------------------------------
	// Step
	//---------------------------------------------------------------
	void Step() override
	{
		Sample::Step();

		b2SensorEvents events = b2World_GetSensorEvents( m_worldId );
		const b2Vec2 spawnCenter{ 0.0f, 0.0f };

		// --------------------------- BEGIN TOUCH -----------------------------
		for ( int i = 0; i < events.beginCount; ++i )
		{
			const b2SensorBeginTouchEvent& e = events.beginEvents[i];

			if ( !m_shapeColorMap.count( e.visitorShapeId ) )
				continue;
			if ( !m_segmentOriginalColor.count( e.sensorShapeId ) )
				continue;

			m_touchingSegments[e.visitorShapeId].insert( e.sensorShapeId );

			uint32_t col = m_shapeColorMap[e.visitorShapeId];
			b2SurfaceMaterial mat = b2Shape_GetSurfaceMaterial( e.sensorShapeId );
			mat.customColor = col;
			b2Shape_SetSurfaceMaterial( e.sensorShapeId, mat );
		}

		// ---------------------------- END TOUCH ------------------------------
		for ( int i = 0; i < events.endCount; ++i )
		{
			const b2SensorEndTouchEvent& e = events.endEvents[i];

			if ( !m_shapeColorMap.count( e.visitorShapeId ) )
				continue;
			if ( !m_segmentOriginalColor.count( e.sensorShapeId ) )
				continue;

			auto& set = m_touchingSegments[e.visitorShapeId];
			set.erase( e.sensorShapeId );

			if ( set.empty() )
			{
				m_touchingSegments.erase( e.visitorShapeId );

				if ( m_alreadyDuplicated.insert( e.visitorShapeId ).second )
				{
					int genLevel = m_generationLevel[e.visitorShapeId];
					for ( int k = 0; k < 2; ++k )
						CreateProjectile( spawnCenter, genLevel + 1 );

					if ( m_enableHitEvents )
						m_audioManager.PlaySpawnSound();
				}
			}

			uint32_t baseCol = m_segmentOriginalColor[e.sensorShapeId];
			b2SurfaceMaterial mat = b2Shape_GetSurfaceMaterial( e.sensorShapeId );
			mat.customColor = baseCol;
			b2Shape_SetSurfaceMaterial( e.sensorShapeId, mat );
		}

		// --------------------- Effets sonores (impacts physiques) ---------------------
		if ( m_enableHitEvents )
		{
			b2ContactEvents evts = b2World_GetContactEvents( m_worldId );
			for ( int i = 0; i < evts.hitCount; ++i )
				m_audioManager.HandleHitEffect( evts.hitEvents[i].point, evts.hitEvents[i].approachSpeed, m_stepCount );
		}
		m_audioManager.PlayQueued();
		m_audioManager.DrawHitEffects( &m_context->draw, m_stepCount );
	}

	//---------------------------------------------------------------
	// GUI
	//---------------------------------------------------------------
	void UpdateGui() override
	{
		ImGuiIO& io = ImGui::GetIO();

		// --- Fenêtre de paramètres contrôlables ---
		if ( m_context->draw.m_showUI )
		{
			ImGui::SetNextWindowPos( ImVec2( io.DisplaySize.x * 0.15f, io.DisplaySize.y * 0.54f ), ImGuiCond_Always,
									 ImVec2( 0.5f, 0.0f ) );
			ImGui::SetNextWindowSize( ImVec2( 400, 0 ), ImGuiCond_Always );
			ImGui::Begin( "BluePrint Settings" );

			const char* shapeNames[] = { "Circle",	 "Capsule", "Box",		"Triangle", "Quad",
										 "Pentagon", "Hexagon", "Heptagon", "Octagon" };
			bool recreate = false;

			bool lockChanged = false;
			lockChanged |= ImGui::Checkbox( "Lock X", &m_motionLocks.linearX );
			ImGui::SameLine();
			lockChanged |= ImGui::Checkbox( "Lock Y", &m_motionLocks.linearY );
			ImGui::SameLine();
			lockChanged |= ImGui::Checkbox( "Lock Rot", &m_motionLocks.angularZ );

			if ( ImGui::SliderFloat( "Shape Size", &m_shapeSize, 0.1f, 5.0f, "%.2f" ) )
				recreate = true;
			if ( ImGui::SliderFloat( "Spacing", &m_spacing, 0.5f, 5.0f, "%.2f" ) )
				recreate = true;

			bool physChanged = false;
			physChanged |= ImGui::SliderFloat( "Restitution", &m_restitution, 0.0f, 1.5f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Friction", &m_friction, 0.0f, 1.0f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Density", &m_density, 0.01f, 10.0f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Linear Damping", &m_linearDamping, 0.0f, 10.0f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Angular Damping", &m_angularDamping, 0.0f, 5.0f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Gravity Scale", &m_gravityScale, 0.0f, 5.0f, "%.2f" );
			physChanged |= ImGui::Checkbox( "Bullet", &m_isBullet );

			physChanged |= ImGui::SliderFloat( "Min Speed", &m_speedMin, 0.0f, 30.0f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Max Speed", &m_speedMax, 0.0f, 30.0f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Min Ang Vel", &m_angVelMin, -50.0f, 0.0f, "%.2f" );
			physChanged |= ImGui::SliderFloat( "Max Ang Vel", &m_angVelMax, 0.0f, 50.0f, "%.2f" );

			if ( ImGui::SliderFloat( "Sound Volume", &m_soundVolume, 0.0f, 100.0f, "%.0f%%" ) )
				m_audioManager.SetVolume( m_soundVolume );
			ImGui::Checkbox( "Enable Hit Events", &m_enableHitEvents );

			ImGui::Separator();
			bool cageChanged = false;
			cageChanged |= ImGui::SliderInt( "Segments par cage", &m_cageSegments, 4, 128 );
			cageChanged |= ImGui::SliderFloat( "Épaisseur cages", &m_cageThickness, 0.05f, 2.0f, "%.2f" );
			cageChanged |= ImGui::SliderFloat( "Vitesse rotation cages", &m_cageMotorSpeed, -5.0f, 5.0f, "%.2f rad/s" );
			ImGui::End();

			if ( recreate || lockChanged || physChanged || cageChanged )
				CreateBodies();
		}

		// --- HUD titre (vert, centré, ombre douce) ---
		{
			ImFont* titleFont = m_context->draw.m_largeFont ? m_context->draw.m_largeFont : ImGui::GetFont();
			const char* title = "CAGEDUPLICATIONIMAGEGUESS";
			ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoBackground;

			ImGui::PushFont( titleFont );
			ImVec2 ts = ImGui::CalcTextSize( title );
			ImGui::PopFont();

			float padX = ts.x * 0.2f;
			float padY = ts.y * 0.5f;
			ImVec2 winSize( ts.x + padX * 2, ts.y + padY * 2 );

			ImGui::SetNextWindowPos( ImVec2( io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.17f ), ImGuiCond_Always,
									 ImVec2( 0.5f, 0.0f ) );
			ImGui::SetNextWindowSize( winSize, ImGuiCond_Always );
			ImGui::SetNextWindowBgAlpha( 0.0f );
			ImGui::Begin( "##HUDTitleOmbre", nullptr, flags );

			ImDrawList* dl = ImGui::GetWindowDrawList();
			ImVec2 wp = ImGui::GetWindowPos();
			ImVec2 ws = ImGui::GetWindowSize();
			ImVec2 tp( wp.x + ( ws.x - ts.x ) * 0.5f, wp.y + ( ws.y - ts.y ) * 0.5f );

			// Ombre douce (décalage +1.5, alpha 80)
			dl->AddText( titleFont, titleFont->FontSize, ImVec2( tp.x + 1.5f, tp.y + 1.5f ), IM_COL32( 0, 0, 0, 80 ), title );
			// Texte principal vert lumineux
			dl->AddText( titleFont, titleFont->FontSize, tp, IM_COL32( 0, 255, 0, 230 ), title );

			ImGui::End();
		}

		// --- HUD compteur de projectiles (vert, sous le titre) ---
		{
			ImFont* largeFont = m_context->draw.m_largeFont ? m_context->draw.m_largeFont : ImGui::GetFont();
			std::string countText = std::to_string( m_projectileCount );

			ImGui::PushFont( largeFont );
			ImVec2 countSize = ImGui::CalcTextSize( countText.c_str() );
			ImGui::PopFont();

			const float padX = 40.0f;
			const float padY = 25.0f;
			ImVec2 winSize( countSize.x + padX * 2, countSize.y + padY * 2 );

			ImGui::SetNextWindowPos( ImVec2( io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.22f ), ImGuiCond_Always,
									 ImVec2( 0.5f, 0.0f ) );
			ImGui::SetNextWindowSize( winSize, ImGuiCond_Always );
			ImGui::SetNextWindowBgAlpha( 0.0f );

			ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
									 ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoBackground;

			ImGui::Begin( "##HUDCountOmbre", nullptr, flags );

			ImDrawList* dl = ImGui::GetWindowDrawList();
			ImVec2 wp = ImGui::GetWindowPos();
			ImVec2 ws = ImGui::GetWindowSize();
			ImVec2 tp( wp.x + ( ws.x - countSize.x ) * 0.5f, wp.y + ( ws.y - countSize.y ) * 0.5f );

			dl->AddText( largeFont, largeFont->FontSize, ImVec2( tp.x + 2.0f, tp.y + 2.0f ), IM_COL32( 0, 80, 0, 100 ),
						 countText.c_str() );
			dl->AddText( largeFont, largeFont->FontSize, tp, IM_COL32( 0, 255, 0, 240 ), countText.c_str() );

			ImGui::End();
		}
	}

private:
	// Audio
	AudioManager m_audioManager;
	float m_soundVolume;

	// Physique
	bool m_enableHitEvents;
	b2MotionLocks m_motionLocks;
	b2Vec2 m_gravity;
	float m_linearDamping;
	float m_angularDamping;
	float m_gravityScale;
	bool m_isBullet;
	float m_restitution = 1.0f;
	float m_friction = 0.0f;
	float m_density = 1.0f;
	float m_shapeSize = 1.0f;
	float m_spacing = 1.0f;
	// Nouvelles plages pour la vélocité
	float m_speedMin = 5.0f, m_speedMax = 5.0f;
	float m_angVelMin = -10.0f, m_angVelMax = 10.0f;

	// Équipes
	ShapeConfig m_shapes[2] = { { e_circleShape, 0x00FF00 }, { e_boxShape, 0x00FF00 } };
	std::unordered_map<b2ShapeId, uint32_t, b2ShapeIdHash, b2ShapeIdEqual> m_shapeColorMap;

	// Cage
	float m_cageThickness = 1.0f;
	float m_cageMotorSpeed = 0.0f;
	int m_cageSegments = 50;
	std::vector<b2BodyId> m_cageBodies;
	// compteur de projectiles
	int m_projectileCount = 0;

	// Projectiles
	std::vector<b2BodyId> m_bodies;

	// Segments → couleur d’origine
	std::unordered_map<b2ShapeId, uint32_t, b2ShapeIdHash, b2ShapeIdEqual> m_segmentOriginalColor;

	// Gestion duplication/génération
	std::unordered_set<b2ShapeId, b2ShapeIdHash, b2ShapeIdEqual> m_alreadyDuplicated;
	std::unordered_map<b2ShapeId, int, b2ShapeIdHash, b2ShapeIdEqual> m_generationLevel;

	// Ensemble des segments actuellement touchés par chaque projectile
	std::unordered_map<b2ShapeId, TouchSet, b2ShapeIdHash, b2ShapeIdEqual> m_touchingSegments;

	// Audio init
	void InitializeAudio()
	{
		std::filesystem::path p = "data/audio/Ticks";
		if ( !std::filesystem::exists( p ) )
			p = "D:/Sound & Fx/audio/Ticks";
		m_audioManager.LoadFromDirectory( p.string() );
		m_audioManager.SetVolume( m_soundVolume );
	}

	// Centralise la création d’un projectile (utilisé partout)
	void CreateProjectile( const b2Vec2& pos, int genLevel )
	{
		float speed = RandomFloatRange( m_speedMin, m_speedMax );
		float angle = RandomFloatRange( -b2_pi, b2_pi );

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { speed * std::cos( angle ), speed * std::sin( angle ) };
		bd.angularVelocity = RandomFloatRange( m_angVelMin, m_angVelMax );
		bd.motionLocks = m_motionLocks;
		bd.linearDamping = m_linearDamping;
		bd.angularDamping = m_angularDamping;
		bd.gravityScale = m_gravityScale;
		bd.isBullet = m_isBullet;

		b2BodyId body = b2CreateBody( m_worldId, &bd );
		m_bodies.push_back( body );

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = m_density;
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.restitution = m_restitution;
		sd.material.friction = m_friction;
		sd.material.customColor = 0x00FF00; // VERT pour projectiles
		sd.isSensor = true;
		sd.enableSensorEvents = true;
		sd.enableHitEvents = true;

		b2Circle c{ { 0.0f, 0.0f }, 0.5f * m_shapeSize };
		b2ShapeId sid = b2CreateCircleShape( body, &sd, &c );
		m_shapeColorMap[sid] = sd.material.customColor;
		m_generationLevel[sid] = genLevel;
		m_projectileCount++;
	}

	// Cage sensor
	b2BodyId CreateFullCage( b2WorldId worldId, float radius, float thickness, int segCount, float speed )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = { 0.0f, 0.0f };
		b2BodyId cage = b2CreateBody( worldId, &bd );

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.friction = 0.1f;
		sd.material.customColor = 0x333333; // VERT pour la cage
		sd.isSensor = true;
		sd.enableSensorEvents = true;

		float dAng = 2.0f * b2_pi / segCount;
		for ( int i = 0; i < segCount; ++i )
		{
			float a0 = i * dAng;
			float a1 = ( i + 1 ) * dAng;

			b2Vec2 p0 = { radius * std::cos( a0 ), radius * std::sin( a0 ) };
			b2Vec2 p1 = { radius * std::cos( a1 ), radius * std::sin( a1 ) };
			b2Vec2 c = 0.5f * ( p0 + p1 );
			float ang = std::atan2( p1.y - p0.y, p1.x - p0.x );
			float len = b2Distance( p0, p1 );

			b2Polygon rect = b2MakeOffsetBox( 0.5f * len, 0.5f * thickness, c, b2MakeRot( ang ) );
			b2ShapeId sid = b2CreatePolygonShape( cage, &sd, &rect );
			m_segmentOriginalColor[sid] = sd.material.customColor;
		}

		b2BodyDef pivotDef;
		pivotDef = b2DefaultBodyDef();
		pivotDef.position = { 0.0f, 0.0f };
		b2BodyId pivot = b2CreateBody( worldId, &pivotDef );

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = pivot;
		jd.base.bodyIdB = cage;
		jd.enableMotor = true;
		jd.motorSpeed = speed;
		jd.maxMotorTorque = 1e5f;
		b2CreateRevoluteJoint( worldId, &jd );

		return cage;
	}

	// Création bodies + reset duplications/overlap/génération
	void CreateBodies()
	{
		m_alreadyDuplicated.clear();
		m_generationLevel.clear();
		m_touchingSegments.clear();
		g_randomSeed = (uint32_t)std::time( nullptr );

		for ( b2BodyId id : m_bodies )
			if ( B2_IS_NON_NULL( id ) )
				b2DestroyBody( id );
		m_bodies.clear();

		for ( b2BodyId id : m_cageBodies )
			if ( B2_IS_NON_NULL( id ) )
				b2DestroyBody( id );
		m_cageBodies.clear();

		m_shapeColorMap.clear();
		m_segmentOriginalColor.clear();

		// Cage sensor
		float radius = 8.0f;
		m_cageBodies.push_back( CreateFullCage( m_worldId, radius, m_cageThickness, m_cageSegments, m_cageMotorSpeed ) );

		// 1 projectile central
		m_shapes[0].color = 0x00FF00; // vert

		m_projectileCount = 0; // Reset à chaque reset/simulation
		CreateProjectile( { 0.0f, 0.0f }, 0 );
		m_projectileCount++; // Incrémente ici
	}
};

static int sampleCageDuplicationImageGuess = RegisterSample( "9:16", "CageDuplicationImageGuess", CageDuplicationImageGuess::Create );


class CageEscape : public Sample
{
public:
	struct CageHoleSensorData
	{
		int cageIndex;
		b2BodyId cageBody;
	};
	static Sample* Create( SampleContext* context )
	{
		return new CageEscape( context );
	}

	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape,
		e_polygon3,
		e_polygon4,
		e_polygon5,
		e_polygon6,
		e_polygon7,
		e_polygon8
	};

	explicit CageEscape( SampleContext* context )
		: Sample( context )
		, m_shapeType( e_boxShape )
		, m_enableHitEvents( true )
		, m_soundVolume( 50.0f )
		, m_linearDamping( 0.0f )
		, m_angularDamping( 0.0f )
		, m_gravityScale( 1.0f )
		, m_isBullet( false )
		, m_gravity{ 0.0f, -10.0f }
		, m_motionLocks{ false, false, false }
	{
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 20.0f;
		}
		b2World_SetGravity( m_worldId, m_gravity );
		InitializeAudio();

		CreateBodies();
	}

	void Step() override
	{
		Sample::Step();

		b2SensorEvents sensorEvents = b2World_GetSensorEvents( m_worldId );

		// --- Marquer les cages à détruire ---
		std::set<b2BodyId> cagesToDestroy;

		for ( int i = 0; i < sensorEvents.beginCount; ++i )
		{
			b2SensorBeginTouchEvent* event = sensorEvents.beginEvents + i;

			// On traite uniquement les sensors "trou de cage"
			auto* data = static_cast<CageHoleSensorData*>( b2Shape_GetUserData( event->sensorShapeId ) );
			if ( data )
			{
				b2BodyId visitorBody = b2Shape_GetBody( event->visitorShapeId );

				// FILTRE : seules les bodies dynamiques de la grille peuvent détruire la cage
				auto it = std::find_if( m_bodies.begin(), m_bodies.end(),
										[visitorBody]( const b2BodyId& id ) { return B2_ID_EQUALS( id, visitorBody ); } );
				if ( it != m_bodies.end() )
				{
					cagesToDestroy.insert( data->cageBody );
				}
			}

			// Coloration lime des shapes dynamiques traversant le sensor (facultatif)
			if ( b2Shape_IsValid( event->visitorShapeId ) )
			{
				b2SurfaceMaterial material = b2Shape_GetSurfaceMaterial( event->visitorShapeId );
				material.customColor = b2_colorLime;
				b2Shape_SetSurfaceMaterial( event->visitorShapeId, material );
			}
		}

		// Remet la couleur à zéro quand on quitte le sensor
		for ( int i = 0; i < sensorEvents.endCount; ++i )
		{
			b2SensorEndTouchEvent* event = sensorEvents.endEvents + i;
			if ( b2Shape_IsValid( event->visitorShapeId ) )
			{
				b2SurfaceMaterial material = b2Shape_GetSurfaceMaterial( event->visitorShapeId );
				material.customColor = 0;
				b2Shape_SetSurfaceMaterial( event->visitorShapeId, material );
			}
		}

		// --- Détruire les cages marquées ---
		for ( b2BodyId cageId : cagesToDestroy )
		{
			if ( B2_IS_NON_NULL( cageId ) )
			{
				b2DestroyBody( cageId );

				// Nettoyage du vector m_cageBodies
				auto it = std::find_if( m_cageBodies.begin(), m_cageBodies.end(),
										[cageId]( const b2BodyId& id ) { return B2_ID_EQUALS( id, cageId ); } );
				if ( it != m_cageBodies.end() )
					m_cageBodies.erase( it );
			}
		}

		// --- reste de Step inchangé ---
		if ( m_enableHitEvents )
		{
			b2ContactEvents events = b2World_GetContactEvents( m_worldId );
			for ( int i = 0; i < events.hitCount; ++i )
			{
				m_audioManager.HandleHitEffect( events.hitEvents[i].point, events.hitEvents[i].approachSpeed, m_stepCount );
			}
		}

		m_audioManager.PlayQueued();
		m_audioManager.DrawHitEffects( &m_context->draw, m_stepCount );
	}

	void UpdateGui() override
{
    if (!m_context->draw.m_showUI)
        return;

    ImGuiIO& io = ImGui::GetIO();
	ImVec2 pos = ImVec2( io.DisplaySize.x * 0.22f, io.DisplaySize.y * 0.62f );
	ImGui::SetNextWindowPos( pos, ImGuiCond_Always, ImVec2( 0.5f, 0.5f ) );
    ImGui::SetNextWindowSize(ImVec2(400, 0), ImGuiCond_Always);

    ImGui::Begin("BluePrint Settings");

    // --- Sélection du type de forme ---
    const char* shapeNames[] = { "Circle", "Capsule", "Box", "Triangle", "Quad",
                                 "Pentagon", "Hexagon", "Heptagon", "Octagon" };
    int shapeIndex = static_cast<int>(m_shapeType);
    if (ImGui::Combo("Shape Type", &shapeIndex, shapeNames, IM_ARRAYSIZE(shapeNames)))
    {
        m_shapeType = static_cast<ShapeType>(shapeIndex);
        CreateBodies();
    }
    if (ImGui::SliderInt("Shape Count", &m_shapeCount, 1, 50))
        CreateBodies();
    ImGui::Text("Locks:");
    bool changed = false;
    changed |= ImGui::Checkbox("Lock X", &m_motionLocks.linearX);
    ImGui::SameLine();
    changed |= ImGui::Checkbox("Lock Y", &m_motionLocks.linearY);
    ImGui::SameLine();
    changed |= ImGui::Checkbox("Lock Rotation (Z)", &m_motionLocks.angularZ);

    if (changed)
        CreateBodies();

    if (ImGui::SliderFloat("Shape Size", &m_shapeSize, 0.1f, 5.0f, "%.2f"))
        CreateBodies();
    if (ImGui::SliderFloat("Spacing", &m_spacing, 0.5f, 5.0f, "%.2f"))
        CreateBodies();

    // --- Paramètres physiques sliders ---
    bool needUpdateBodies = false;
    needUpdateBodies |= ImGui::SliderFloat("Restitution", &m_restitution, 0.0f, 1.5f, "%.2f");
    needUpdateBodies |= ImGui::SliderFloat("Friction", &m_friction, 0.0f, 1.0f, "%.2f");
    needUpdateBodies |= ImGui::SliderFloat("Density", &m_density, 0.01f, 10.0f, "%.2f");
    needUpdateBodies |= ImGui::SliderFloat("Linear Damping", &m_linearDamping, 0.0f, 10.0f, "%.2f");
    needUpdateBodies |= ImGui::SliderFloat("Angular Damping", &m_angularDamping, 0.0f, 5.0f, "%.2f");
    needUpdateBodies |= ImGui::SliderFloat("Gravity Scale", &m_gravityScale, 0.0f, 5.0f, "%.2f");
    needUpdateBodies |= ImGui::Checkbox("Bullet", &m_isBullet);

    if (needUpdateBodies)
        CreateBodies();

    // Contrôle du volume sonore
    if (ImGui::SliderFloat("Sound Volume", &m_soundVolume, 0.0f, 100.0f, "%.1f%%"))
        m_audioManager.SetVolume(m_soundVolume);

    ImGui::Checkbox("Enable Hit Events", &m_enableHitEvents);

    // --- Bloc gravité ---
    static const char* gravityPresets[] = { "Terre", "Lune", "Mars", "0", "Reverse", "Max" };
    static float presetValues[] = { -10.0f, -1.62f, -3.71f, 0.0f, 10.0f, -980.0f }; // -980 = ~100g
    static int presetIdx = 0;

    if (ImGui::Combo("Gravity Preset", &presetIdx, gravityPresets, IM_ARRAYSIZE(gravityPresets)))
    {
        m_gravity = { 0.0f, presetValues[presetIdx] };
        b2World_SetGravity(m_worldId, m_gravity);
        // Optionnel : tu peux mettre à jour les bodies si nécessaire ici :
        // CreateBodies();
    }

    ImGui::Text("Preset: %s", gravityPresets[presetIdx]);

    ImGui::Separator();
    ImGui::Text("Cages settings");
    bool updateCages = false;
    updateCages |= ImGui::SliderFloat("Rayon de base des cages", &m_baseRadius, 2.0f, 10.0f, "%.2f");
    updateCages |= ImGui::SliderInt("Nombre de cages", &m_cageCount, 1, 50);
    updateCages |= ImGui::SliderInt("Segments par cage", &m_cageSegments, 4, 128);
    updateCages |= ImGui::SliderFloat("Espacement cages", &m_cageSpacing, 0.05f, 5.0f, "%.2f");
    updateCages |= ImGui::SliderFloat("Épaisseur cages", &m_cageThickness, 0.05f, 2.0f, "%.2f");
    updateCages |= ImGui::SliderFloat("Angle du trou (deg)", &m_holeAngle, 0.0f, 180.0f, "%.1f");
    updateCages |= ImGui::SliderFloat("Vitesse rotation cages", &m_cageMotorSpeed, -5.0f, 5.0f, "%.2f rad/s");
    if (updateCages)
        CreateBodies();

    ImGui::End();
}


private:
	// --- Audio ---
	AudioManager m_audioManager;
	float m_soundVolume;

	// --- Physics ---
	ShapeType m_shapeType;
	bool m_enableHitEvents;
	b2MotionLocks m_motionLocks;
	b2Vec2 m_gravity;

	float m_linearDamping;
	float m_angularDamping;
	float m_gravityScale;
	bool m_isBullet;

	float m_restitution = 1.0f;
	float m_friction = 0.0f;
	float m_density = 1.0f;
	float m_shapeSize = 2.0f;
	float m_spacing = 1.0f;

	// --- Shapes ---
	std::vector<b2BodyId> m_bodies;
	int m_shapeCount = 2;

	// --- Cages imbriquées ---
	int m_cageCount = 30;
	float m_cageSpacing = 0.05f;
	float m_baseRadius = 8.0f;
	float m_cageThickness = 1.3f;
	float m_holeAngle = 45.0f;			// en degrés
	std::vector<b2BodyId> m_cageBodies; // Pour suivre/détruire les cages
	float m_cageMotorSpeed = 0.5f;		// en radians/seconde
	int m_cageSegments = 32;			// valeur par défaut, typiquement entre 12 et 64

	// --- Catégories collision ---
	static constexpr uint16_t CATEGORY_SHAPE = 0x0002;
	static constexpr uint16_t CATEGORY_SENSOR = 0x0004;
	static constexpr uint16_t CATEGORY_CAGE = 0x0008;

	uint32_t ComputeEllipticColor( int ix, int iy, int cols, int rows )
	{
		float cx = ( cols - 1 ) * 0.5f;
		float cy = ( rows - 1 ) * 0.5f;
		float dx = static_cast<float>( ix ) - cx;
		float dy = static_cast<float>( iy ) - cy;
		float a = std::max( cx, 1.0f );
		float b = std::max( cy, 1.0f );
		float r = std::sqrt( ( dx * dx ) / ( a * a ) + ( dy * dy ) / ( b * b ) );
		r = std::clamp( r / std::sqrt( 2.0f ), 0.0f, 1.0f );
		float hue = r;
		return HSVtoRGB( hue, 0.75f, 1.0f );
	}

	uint32_t HSVtoRGB( float h, float s, float v )
	{
		float r, g, b;
		int i = int( h * 6.0f );
		float f = h * 6.0f - i;
		float p = v * ( 1.0f - s );
		float q = v * ( 1.0f - f * s );
		float t = v * ( 1.0f - ( 1.0f - f ) * s );
		switch ( i % 6 )
		{
			case 0:
				r = v, g = t, b = p;
				break;
			case 1:
				r = q, g = v, b = p;
				break;
			case 2:
				r = p, g = v, b = t;
				break;
			case 3:
				r = p, g = q, b = v;
				break;
			case 4:
				r = t, g = p, b = v;
				break;
			case 5:
				r = v, g = p, b = q;
				break;
		}
		uint32_t R = uint32_t( r * 255.0f );
		uint32_t G = uint32_t( g * 255.0f );
		uint32_t B = uint32_t( b * 255.0f );
		return ( R << 16 ) | ( G << 8 ) | B;
	}

	void InitializeAudio()
	{
		std::filesystem::path path = "data/audio/Ticks";
		if ( !std::filesystem::exists( path ) )
			path = "D:/Sound & Fx/audio/Ticks";
		m_audioManager.LoadFromDirectory( path.string() );
		m_audioManager.SetVolume( m_soundVolume );
	}

	b2BodyId CreateRotatingCageWithHole( b2WorldId worldId, float radius, float thickness, float holeAngle, int segmentCount,
										 float motorSpeed )
	{
		// Corps central de la cage
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { 0.0f, 0.0f };
		b2BodyId cageBody = b2CreateBody( worldId, &bodyDef );

		// --- Création des "murs" de la cage ---
		float twoPi = 2.0f * b2_pi;
		float segmentArc = ( twoPi - holeAngle ) / segmentCount;
		float startAngle = -b2_pi / 2.0f + holeAngle * 0.5f;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material = b2DefaultSurfaceMaterial();
		shapeDef.material.friction = 0.1f;
		shapeDef.material.restitution = 0.0f;
		shapeDef.material.customColor = 0x44FFD5;

		// --- Catégorie/Masque : CAGE ---
		shapeDef.filter.categoryBits = CATEGORY_CAGE; // Marque ce shape comme "cage"
		shapeDef.filter.maskBits = CATEGORY_SHAPE;	  // Il collisionne seulement avec les shapes dynamiques

		for ( int i = 0; i < segmentCount; ++i )
		{
			float angleA = startAngle + i * segmentArc;
			float angleB = startAngle + ( i + 1 ) * segmentArc;
			b2Vec2 pA = { radius * std::cos( angleA ), radius * std::sin( angleA ) };
			b2Vec2 pB = { radius * std::cos( angleB ), radius * std::sin( angleB ) };
			b2Capsule capsule = { pA, pB, thickness * 0.5f };
			b2CreateCapsuleShape( cageBody, &shapeDef, &capsule );
		}

		// --- Création des sensors pour le trou ---
		int holeSensorCount = std::max( 1, int( std::ceil( segmentCount * ( holeAngle / ( twoPi - holeAngle ) ) ) ) );
		float sensorArc = holeAngle / holeSensorCount;
		float sensorStartAngle = -b2_pi / 2.0f - holeAngle * 0.5f;

		for ( int i = 0; i < holeSensorCount; ++i )
		{
			float angleA = sensorStartAngle + i * sensorArc;
			float angleB = sensorStartAngle + ( i + 1 ) * sensorArc;

			b2Vec2 pA = { radius * std::cos( angleA ), radius * std::sin( angleA ) };
			b2Vec2 pB = { radius * std::cos( angleB ), radius * std::sin( angleB ) };

			b2ShapeDef sensorDef = b2DefaultShapeDef();
			sensorDef.isSensor = true;
			sensorDef.enableSensorEvents = true;
			sensorDef.material.customColor = 0xFF0088;

			// --- Catégorie/Masque : SENSOR ---
			sensorDef.filter.categoryBits = CATEGORY_SENSOR; // "Sensor"
			sensorDef.filter.maskBits = CATEGORY_SHAPE;		 // Ne détecte que les shapes dynamiques

			// User data pour retrouver la cage à détruire
			CageHoleSensorData* data = new CageHoleSensorData;
			data->cageIndex = i;
			data->cageBody = cageBody;
			sensorDef.userData = data;

			b2Capsule holeSensorCapsule = { pA, pB, thickness * 0.4f };
			b2CreateCapsuleShape( cageBody, &sensorDef, &holeSensorCapsule );
		}

		// Fixe la cage sur pivot
		b2BodyDef pivotDef = b2DefaultBodyDef();
		pivotDef.position = { 0.0f, 0.0f };
		b2BodyId pivotId = b2CreateBody( worldId, &pivotDef );

		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		b2Vec2 pivot = { 0.0f, 0.0f };
		jointDef.base.bodyIdA = pivotId;
		jointDef.base.bodyIdB = cageBody;
		jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
		jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
		jointDef.enableMotor = true;
		jointDef.motorSpeed = motorSpeed;
		jointDef.maxMotorTorque = 1e5f;
		b2CreateRevoluteJoint( worldId, &jointDef );

		return cageBody;
	}

	void CreateBodies()
	{
		// Détruit les bodies "shapes" précédents
		for ( b2BodyId id : m_bodies )
			if ( B2_IS_NON_NULL( id ) )
				b2DestroyBody( id );
		m_bodies.clear();

		// Détruit les cages précédentes
		for ( b2BodyId id : m_cageBodies )
			if ( B2_IS_NON_NULL( id ) )
				b2DestroyBody( id );
		m_cageBodies.clear();

		// --- Création dynamique des cages imbriquées ---
		float baseRadius = m_baseRadius;
		for ( int i = 0; i < m_cageCount; ++i )
		{
			float radius = baseRadius + i * ( m_cageThickness + m_cageSpacing );
			int segmentCount = m_cageSegments;
			float holeAngleRad = m_holeAngle * b2_pi / 180.0f;
			b2BodyId cageId =
				CreateRotatingCageWithHole( m_worldId, radius, m_cageThickness, holeAngleRad, segmentCount, m_cageMotorSpeed );
			m_cageBodies.push_back( cageId );
		}

		// --- Création des bodies "shapes" (les projectiles/dynamiques de la grille) ---
		int n = m_shapeCount;
		int cols = int( std::ceil( std::sqrt( n ) ) );
		int rows = ( n + cols - 1 ) / cols;
		float dx = m_spacing;
		float dy = m_spacing;
		float totalW = dx * ( cols - 1 );
		float totalH = dy * ( rows - 1 );
		float startX = -totalW * 0.5f;
		float startY = totalH * 0.5f;

		for ( int i = 0; i < n; ++i )
		{
			int ix = i % cols;
			int iy = i / cols;
			float x = startX + ix * dx;
			float y = startY - iy * dy;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { x, y };
			bodyDef.linearVelocity = { 0.0f, -10.0f };
			bodyDef.motionLocks = m_motionLocks;
			bodyDef.linearDamping = m_linearDamping;
			bodyDef.angularDamping = m_angularDamping;
			bodyDef.gravityScale = m_gravityScale;
			bodyDef.isBullet = m_isBullet;

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			m_bodies.push_back( bodyId );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.enableSensorEvents = true;
			shapeDef.density = m_density;
			shapeDef.material = b2DefaultSurfaceMaterial();
			shapeDef.material.restitution = m_restitution;
			shapeDef.material.friction = m_friction;
			shapeDef.enableHitEvents = true;
			shapeDef.material.customColor = ComputeEllipticColor( ix, iy, cols, rows );

			// --- Catégorie/Masque : SHAPE (projectile dynamique) ---
			shapeDef.filter.categoryBits = CATEGORY_SHAPE;
			shapeDef.filter.maskBits = CATEGORY_SENSOR | CATEGORY_CAGE; // Collisionne avec cages, détecté par sensors

			ShapeType shapeType = static_cast<ShapeType>( m_shapeType );

			if ( shapeType == e_circleShape )
			{
				b2Circle circle = { { 0.0f, 0.0f }, 0.5f * m_shapeSize };
				b2CreateCircleShape( bodyId, &shapeDef, &circle );
			}
			else if ( shapeType == e_capsuleShape )
			{
				b2Capsule capsule = { { -0.5f * m_shapeSize, 0.0f }, { 0.5f * m_shapeSize, 0.0f }, 0.25f * m_shapeSize };
				b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			}
			else if ( shapeType == e_boxShape )
			{
				b2Polygon box = b2MakeBox( 0.5f * m_shapeSize, 0.5f * m_shapeSize );
				b2CreatePolygonShape( bodyId, &shapeDef, &box );
			}
			else // Polygones réguliers 3-8 côtés
			{
				int sides = int( shapeType ) - int( e_polygon3 ) + 3;
				std::vector<b2Vec2> verts( sides );
				const float R = 0.5f * m_shapeSize;
				const float twoPi = 2.0f * b2_pi;
				for ( int j = 0; j < sides; ++j )
					verts[j] = { std::cos( j * twoPi / sides ) * R, std::sin( j * twoPi / sides ) * R };
				b2Hull hull = b2ComputeHull( verts.data(), sides );
				if ( hull.count > 0 )
				{
					b2Polygon poly = b2MakePolygon( &hull, 0 );
					b2CreatePolygonShape( bodyId, &shapeDef, &poly );
				}
			}
		}
	}
};

static int sampleCageEscape = RegisterSample( "9:16", "CageEscape", CageEscape::Create );

class CageDuel : public Sample
{
public:
	//---------------------------------------------------------------
	// Types
	//---------------------------------------------------------------

	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape,
		e_polygon3,
		e_polygon4,
		e_polygon5,
		e_polygon6,
		e_polygon7,
		e_polygon8,
	};

	struct ShapeConfig
	{
		ShapeType type;
		uint32_t color;
	};

	struct b2ShapeIdHash
	{
		size_t operator()( const b2ShapeId& id ) const noexcept
		{
			return std::hash<uint64_t>()( b2StoreShapeId( id ) );
		}
	};

	struct b2ShapeIdEqual
	{
		bool operator()( const b2ShapeId& a, const b2ShapeId& b ) const noexcept
		{
			return B2_ID_EQUALS( a, b );
		}
	};

	static Sample* Create( SampleContext* context )
	{
		return new CageDuel( context );
	}

	static constexpr uint32_t kDefaultColors[2] = { 0xE53F3F, 0x3F6FE5 };

	//---------------------------------------------------------------
	// Construction
	//---------------------------------------------------------------

	explicit CageDuel( SampleContext* context )
		: Sample( context )
		, m_enableHitEvents( true )
		, m_soundVolume( 50.0f )
		, m_linearDamping( 0.0f )
		, m_angularDamping( 0.0f )
		, m_gravityScale( 1.0f )
		, m_isBullet( false )
		, m_gravity{ 0.0f, -10.0f }
		, m_motionLocks{ false, false, false }
	{
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 20.0f;
		}

		b2World_SetGravity( m_worldId, m_gravity );
		InitializeAudio();
		CreateBodies();
	}

	//---------------------------------------------------------------
	// Step
	//---------------------------------------------------------------

	void Step() override
	{
		Sample::Step();

		b2ContactEvents evts = b2World_GetContactEvents( m_worldId );

		for ( int i = 0; i < evts.beginCount; ++i )
		{
			const b2ContactBeginTouchEvent& e = evts.beginEvents[i];

			bool aIsProj = m_shapeColorMap.count( e.shapeIdA );
			bool bIsProj = m_shapeColorMap.count( e.shapeIdB );
			bool aIsSeg = m_segmentOriginalColor.count( e.shapeIdA );
			bool bIsSeg = m_segmentOriginalColor.count( e.shapeIdB );

			if ( aIsProj && bIsSeg )
			{
				uint32_t col = m_shapeColorMap[e.shapeIdA];
				b2SurfaceMaterial mat = b2Shape_GetSurfaceMaterial( e.shapeIdB );
				mat.customColor = col;
				b2Shape_SetSurfaceMaterial( e.shapeIdB, mat );
			}
			else if ( bIsProj && aIsSeg )
			{
				uint32_t col = m_shapeColorMap[e.shapeIdB];
				b2SurfaceMaterial mat = b2Shape_GetSurfaceMaterial( e.shapeIdA );
				mat.customColor = col;
				b2Shape_SetSurfaceMaterial( e.shapeIdA, mat );
			}
		}

		if ( m_enableHitEvents )
		{
			for ( int i = 0; i < evts.hitCount; ++i )
				m_audioManager.HandleHitEffect( evts.hitEvents[i].point, evts.hitEvents[i].approachSpeed, m_stepCount );
		}
		m_audioManager.PlayQueued();
		m_audioManager.DrawHitEffects( &m_context->draw, m_stepCount );
	}

	//---------------------------------------------------------------
	// GUI
	//---------------------------------------------------------------

	void UpdateGui() override
	{
		ImGuiIO& io = ImGui::GetIO();
		ImVec2 center = { io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f };

		ImGui::SetNextWindowPos( center, ImGuiCond_Always, { 0.5f, 0.5f } );
		ImGui::SetNextWindowSize( { 400, 0 }, ImGuiCond_Always );

		ImGui::Begin( "BluePrint Settings" );

		const char* shapeNames[] = { "Circle",	 "Capsule", "Box",		"Triangle", "Quad",
									 "Pentagon", "Hexagon", "Heptagon", "Octagon" };
		bool recreate = false;

		for ( int team = 0; team < 2; ++team )
		{
			ImGui::PushID( team );
			int idx = static_cast<int>( m_shapes[team].type );
			if ( ImGui::Combo( team == 0 ? "Team A" : "Team B", &idx, shapeNames, IM_ARRAYSIZE( shapeNames ) ) )
			{
				m_shapes[team].type = static_cast<ShapeType>( idx );
				recreate = true;
			}

			float col[3] = { ( ( m_shapes[team].color >> 16 ) & 0xFF ) / 255.0f,
							 ( ( m_shapes[team].color >> 8 ) & 0xFF ) / 255.0f, ( m_shapes[team].color & 0xFF ) / 255.0f };

			if ( ImGui::ColorEdit3( "##col", col, ImGuiColorEditFlags_NoInputs ) )
			{
				m_shapes[team].color =
					( (uint32_t)( col[0] * 255 ) << 16 ) | ( (uint32_t)( col[1] * 255 ) << 8 ) | (uint32_t)( col[2] * 255 );
				recreate = true;
			}
			ImGui::PopID();
		}

		if ( ImGui::SliderInt( "Bodies / team", &m_shapeCountPerType, 1, 50 ) )
			recreate = true;

		bool lockChanged = false;
		lockChanged |= ImGui::Checkbox( "Lock X", &m_motionLocks.linearX );
		ImGui::SameLine();
		lockChanged |= ImGui::Checkbox( "Lock Y", &m_motionLocks.linearY );
		ImGui::SameLine();
		lockChanged |= ImGui::Checkbox( "Lock Rot", &m_motionLocks.angularZ );

		if ( ImGui::SliderFloat( "Shape Size", &m_shapeSize, 0.1f, 5.0f, "%.2f" ) )
			recreate = true;
		if ( ImGui::SliderFloat( "Spacing", &m_spacing, 0.5f, 5.0f, "%.2f" ) )
			recreate = true;

		bool physChanged = false;
		physChanged |= ImGui::SliderFloat( "Restitution", &m_restitution, 0.0f, 1.5f, "%.2f" );
		physChanged |= ImGui::SliderFloat( "Friction", &m_friction, 0.0f, 1.0f, "%.2f" );
		physChanged |= ImGui::SliderFloat( "Density", &m_density, 0.01f, 10.0f, "%.2f" );
		physChanged |= ImGui::SliderFloat( "Linear Damping", &m_linearDamping, 0.0f, 10.0f, "%.2f" );
		physChanged |= ImGui::SliderFloat( "Angular Damping", &m_angularDamping, 0.0f, 5.0f, "%.2f" );
		physChanged |= ImGui::SliderFloat( "Gravity Scale", &m_gravityScale, 0.0f, 5.0f, "%.2f" );
		physChanged |= ImGui::Checkbox( "Bullet", &m_isBullet );

		if ( ImGui::SliderFloat( "Sound Volume", &m_soundVolume, 0.0f, 100.0f, "%.0f%%" ) )
			m_audioManager.SetVolume( m_soundVolume );
		ImGui::Checkbox( "Enable Hit Events", &m_enableHitEvents );

		static const char* gNames[] = { "Terre", "Lune", "Mars", "0", "Reverse", "Max" };
		static float gVals[] = { -10.0f, -1.62f, -3.71f, 0.0f, 10.0f, -980.0f };
		static int gIdx = 0;
		if ( ImGui::Combo( "Gravity Preset", &gIdx, gNames, IM_ARRAYSIZE( gNames ) ) )
		{
			m_gravity = { 0.0f, gVals[gIdx] };
			b2World_SetGravity( m_worldId, m_gravity );
		}
		ImGui::Text( "Preset: %s", gNames[gIdx] );

		ImGui::Separator();
		bool cageChanged = false;
		cageChanged |= ImGui::SliderInt( "Segments par cage", &m_cageSegments, 4, 128 );
		cageChanged |= ImGui::SliderFloat( "Épaisseur cages", &m_cageThickness, 0.05f, 2.0f, "%.2f" );
		cageChanged |= ImGui::SliderFloat( "Vitesse rotation cages", &m_cageMotorSpeed, -5.0f, 5.0f, "%.2f rad/s" );

		ImGui::End();

		if ( recreate || lockChanged || physChanged || cageChanged )
			CreateBodies();
	}

private:
	//---------------------------------------------------------------
	// Audio
	//---------------------------------------------------------------
	AudioManager m_audioManager;
	float m_soundVolume;

	//---------------------------------------------------------------
	// Physique
	//---------------------------------------------------------------
	bool m_enableHitEvents;
	b2MotionLocks m_motionLocks;
	b2Vec2 m_gravity;
	float m_linearDamping;
	float m_angularDamping;
	float m_gravityScale;
	bool m_isBullet;
	float m_restitution = 1.0f;
	float m_friction = 0.0f;
	float m_density = 1.0f;
	float m_shapeSize = 2.0f;
	float m_spacing = 1.0f;

	//---------------------------------------------------------------
	// Équipes
	//---------------------------------------------------------------
	ShapeConfig m_shapes[2] = { { e_circleShape, kDefaultColors[0] }, { e_boxShape, kDefaultColors[1] } };
	int m_shapeCountPerType = 25;

	std::unordered_map<b2ShapeId, uint32_t, b2ShapeIdHash, b2ShapeIdEqual> m_shapeColorMap;

	//---------------------------------------------------------------
	// Cages
	//---------------------------------------------------------------
	float m_cageThickness = 0.6f;
	float m_cageMotorSpeed = 0.5f;
	int m_cageSegments = 40;
	std::vector<b2BodyId> m_cageBodies;

	//---------------------------------------------------------------
	// Projectiles
	//---------------------------------------------------------------
	std::vector<b2BodyId> m_bodies;

	//---------------------------------------------------------------
	// Segments
	//---------------------------------------------------------------
	std::unordered_map<b2ShapeId, uint32_t, b2ShapeIdHash, b2ShapeIdEqual> m_segmentOriginalColor;

	//---------------------------------------------------------------
	// Utilitaires
	//---------------------------------------------------------------

	std::vector<b2ShapeId> m_projectileShapes; // Les formes des équipes (projectiles)
	std::vector<b2ShapeId> m_segmentShapes;	   // Les shapes de cage (segments)

	void InitializeAudio()
	{
		std::filesystem::path p = "data/audio/Ticks";
		if ( !std::filesystem::exists( p ) )
			p = "D:/Sound & Fx/audio/Ticks";
		m_audioManager.LoadFromDirectory( p.string() );
		m_audioManager.SetVolume( m_soundVolume );
	}

	b2BodyId CreateFullCage( b2WorldId worldId, float radius, float thickness, int segCount, float speed )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = { 0.0f, 0.0f };
		b2BodyId cage = b2CreateBody( worldId, &bd );

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.friction = 0.1f;
		sd.material.customColor = 0x44FFD5;
		sd.enableSensorEvents = true;
		sd.filter.categoryBits = CATEGORY_CAGE;
		sd.filter.maskBits = CATEGORY_SHAPE;

		float dAng = 2.0f * b2_pi / segCount;
		for ( int i = 0; i < segCount; ++i )
		{
			float a0 = i * dAng;
			float a1 = ( i + 1 ) * dAng;

			b2Vec2 p0 = { radius * std::cos( a0 ), radius * std::sin( a0 ) };
			b2Vec2 p1 = { radius * std::cos( a1 ), radius * std::sin( a1 ) };
			b2Vec2 c = 0.5f * ( p0 + p1 );
			float ang = std::atan2( p1.y - p0.y, p1.x - p0.x );
			float len = b2Distance( p0, p1 );

			b2Polygon rect = b2MakeOffsetBox( 0.5f * len, 0.5f * thickness, c, b2MakeRot( ang ) );
			b2ShapeId sid = b2CreatePolygonShape( cage, &sd, &rect );

			m_segmentOriginalColor[sid] = sd.material.customColor;
		}

		b2BodyDef pivotDef;
		pivotDef = b2DefaultBodyDef();
		pivotDef.position = { 0.0f, 0.0f };
		b2BodyId pivot = b2CreateBody( worldId, &pivotDef );

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = pivot;
		jd.base.bodyIdB = cage;
		jd.enableMotor = true;
		jd.motorSpeed = speed;
		jd.maxMotorTorque = 1e5f;
		b2CreateRevoluteJoint( worldId, &jd );

		return cage;
	}

	void CreateBodies()

	{
		g_randomSeed = (uint32_t)std::time( nullptr );
		for ( b2BodyId id : m_bodies )
			if ( B2_IS_NON_NULL( id ) )
				b2DestroyBody( id );
		m_bodies.clear();

		for ( b2BodyId id : m_cageBodies )
			if ( B2_IS_NON_NULL( id ) )
				b2DestroyBody( id );
		m_cageBodies.clear();

		m_shapeColorMap.clear();
		m_segmentOriginalColor.clear();

		float radius = 8.0f;
		m_cageBodies.push_back( CreateFullCage( m_worldId, radius, m_cageThickness, m_cageSegments, m_cageMotorSpeed ) );

		int total = 2 * m_shapeCountPerType;
		int cols = static_cast<int>( std::ceil( std::sqrt( total ) ) );
		int rows = ( total + cols - 1 ) / cols;

		float dx = m_spacing;
		float dy = m_spacing;

		float startX = -0.5f * dx * ( cols - 1 );
		float startY = 0.5f * dy * ( rows - 1 );

		for ( int i = 0; i < total; ++i )
		{
			int team = i & 1;
			const ShapeConfig& cfg = m_shapes[team];

			int ix = i % cols;
			int iy = i / cols;

			float x = startX + ix * dx;
			float y = startY - iy * dy;

			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = { x, y };
			float speed = RandomFloatRange( 5.0f, 5.0f );	 // module de vitesse aléatoire
			float angle = RandomFloatRange( -b2_pi, b2_pi ); // angle aléatoire
			bd.linearVelocity = { speed * std::cos( angle ), speed * std::sin( angle ) };
			bd.motionLocks = m_motionLocks;
			bd.linearDamping = m_linearDamping;
			bd.angularDamping = m_angularDamping;
			bd.gravityScale = m_gravityScale;
			bd.isBullet = m_isBullet;

			b2BodyId body = b2CreateBody( m_worldId, &bd );
			m_bodies.push_back( body );

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = m_density;
			sd.material = b2DefaultSurfaceMaterial();
			sd.material.restitution = m_restitution;
			sd.material.friction = m_friction;
			sd.material.customColor = cfg.color;
			sd.enableSensorEvents = true;
			sd.enableHitEvents = true;
			sd.filter.categoryBits = CATEGORY_SHAPE;
			sd.filter.maskBits = CATEGORY_CAGE;

			auto addShape = [&]( b2ShapeId sid ) { m_shapeColorMap[sid] = cfg.color; };

			switch ( cfg.type )
			{
				case e_circleShape:
				{
					b2Circle c = { { 0.0f, 0.0f }, 0.5f * m_shapeSize };
					addShape( b2CreateCircleShape( body, &sd, &c ) );
				}
				break;

				case e_capsuleShape:
				{
					b2Capsule cap = { { -0.5f * m_shapeSize, 0.0f }, { 0.5f * m_shapeSize, 0.0f }, 0.25f * m_shapeSize };
					addShape( b2CreateCapsuleShape( body, &sd, &cap ) );
				}
				break;

				case e_boxShape:
				{
					b2Polygon box = b2MakeBox( 0.5f * m_shapeSize, 0.5f * m_shapeSize );
					addShape( b2CreatePolygonShape( body, &sd, &box ) );
				}
				break;

				default:
				{
					int sides = int( cfg.type ) - int( e_polygon3 ) + 3;
					std::vector<b2Vec2> v( sides );
					float R = 0.5f * m_shapeSize;
					float twoPi = 2.0f * b2_pi;
					for ( int k = 0; k < sides; ++k )
						v[k] = { std::cos( k * twoPi / sides ) * R, std::sin( k * twoPi / sides ) * R };
					b2Hull h = b2ComputeHull( v.data(), sides );
					if ( h.count )
					{
						b2Polygon poly = b2MakePolygon( &h, 0.0f );
						addShape( b2CreatePolygonShape( body, &sd, &poly ) );
					}
				}
			}
		}
	}

	//---------------------------------------------------------------
	// Catégories
	//---------------------------------------------------------------
	static constexpr uint16_t CATEGORY_SHAPE = 0x0002;
	static constexpr uint16_t CATEGORY_CAGE = 0x0008;
};

static int sampleCageDuel = RegisterSample( "9:16", "CageDuel", CageDuel::Create );

class BallAndChain : public Sample
{
public:
	explicit BallAndChain( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, -8.0f };
			m_context->camera.m_zoom = 27.5f;
		}

		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );
		}

		m_frictionTorque = 100.0f;

		{
			float hx = 0.5f;
			b2Capsule capsule = { { -hx, 0.0f }, { hx, 0.0f }, 0.125f };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 20.0f;
			shapeDef.filter.categoryBits = 0x1;
			shapeDef.filter.maskBits = 0x2;
			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();

			int jointIndex = 0;

			b2BodyId prevBodyId = groundId;
			for ( int i = 0; i < m_count; ++i )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = { ( 1.0f + 2.0f * i ) * hx, m_count * hx };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );

				b2Vec2 pivot = { ( 2.0f * i ) * hx, m_count * hx };
				jointDef.base.bodyIdA = prevBodyId;
				jointDef.base.bodyIdB = bodyId;
				jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
				jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
				jointDef.enableMotor = true;
				jointDef.maxMotorTorque = m_frictionTorque;
				jointDef.enableSpring = i > 0;
				jointDef.hertz = 4.0f;
				m_jointIds[jointIndex++] = b2CreateRevoluteJoint( m_worldId, &jointDef );

				prevBodyId = bodyId;
			}

			b2Circle circle = { { 0.0f, 0.0f }, 4.0f };

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { ( 1.0f + 2.0f * m_count ) * hx + circle.radius - hx, m_count * hx };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			shapeDef.filter.categoryBits = 0x2;
			shapeDef.filter.maskBits = 0x1;
			b2CreateCircleShape( bodyId, &shapeDef, &circle );

			b2Vec2 pivot = { ( 2.0f * m_count ) * hx, m_count * hx };
			jointDef.base.bodyIdA = prevBodyId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
			jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = m_frictionTorque;
			jointDef.enableSpring = true;
			jointDef.hertz = 4.0f;
			m_jointIds[jointIndex++] = b2CreateRevoluteJoint( m_worldId, &jointDef );
			assert( jointIndex == m_count + 1 );
		}
	}

	void UpdateGui() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 60.0f;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->m_height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Ball and Chain", nullptr, ImGuiWindowFlags_NoResize );

		bool updateFriction = ImGui::SliderFloat( "Joint Friction", &m_frictionTorque, 0.0f, 1000.0f, "%2.f" );
		if ( updateFriction )
		{
			for ( int i = 0; i <= m_count; ++i )
			{
				b2RevoluteJoint_SetMaxMotorTorque( m_jointIds[i], m_frictionTorque );
			}
		}

		ImGui::End();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BallAndChain( context );
	}

	static constexpr int m_count = 30;
	b2JointId m_jointIds[m_count + 1];
	float m_frictionTorque;
};

static int sampleBallAndChainIndex = RegisterSample( "9:16", "Ball & Chain", BallAndChain::Create );

class Wichyear : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new Wichyear( context );
	}

	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape,
		e_polygon3,
		e_polygon4,
		e_polygon5,
		e_polygon6,
		e_polygon7,
		e_polygon8
	};

	// --- Config utilisateur ---
	uint32_t m_uniformColor = 0x00FF00;
	int m_uniformColorIndex = 0;
	bool m_enableDuplication = false;
	bool m_enableScreenShake = false;
	bool m_enableHitEvents = true;
	bool m_enableHitSounds = true;
	bool m_fixedRotation = false;

	// --- Paramètres physiques ---
	float m_density = 1.0f;
	float m_restitution = 1.0f;
	float m_friction = 0.0005f;
	float m_linearDamping = 0.0f;
	float m_angularDamping = 0.0f;
	float m_gravityScale = 5.0f;
	float m_initialAngularVelocity = 0.0f;

	ShapeType m_shapeType = e_circleShape;

	struct HitEvent
	{
		b2Vec2 point{ 0, 0 };
		float speed{ 0 };
		int stepIndex{ -1 };
	};
	std::array<HitEvent, 10> m_hitEvents;

	// --- Money logic ---
	double m_money = 1.0;				   // Valeur initiale ($1)
	const double m_moneyMultiplier = 1.12; // +12% par rebond (modifiable)
	int m_year = 2025;

	explicit Wichyear( SampleContext* context )
		: Sample( context )
		, m_currentSoundIndex( 0 )
		, lastImpactTime( -5 )
		, shakeDuration( 0 )
		, shakeIntensity( 0.0f )
		, m_destroySoundIndex( 0 )
	{
		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0, 0 };
			m_context->camera.m_zoom = 40.0f;
		}
		b2World_SetGravity( m_worldId, { 0.0f, -20.0f } );
		for ( auto& e : m_hitEvents )
			e = HitEvent();
		InitializeAudio();
		CreateArena();
		Launch();
	}

	void UpdateGui() override
	{
		ImGuiIO& io = ImGui::GetIO();

		// === UI contrôles classiques ===
		if ( m_context->draw.m_showUI )
		{

			ImGui::SetNextWindowSize( ImVec2( 400, 450 ), ImGuiCond_FirstUseEver );
			ImGui::Begin( "Blueprint" );
			const char* shapes[] = { "Circle",	 "Capsule", "Box",		"Triangle", "Quad",
									 "Pentagon", "Hexagon", "Heptagon", "Octagon" };
			int si = int( m_shapeType );
			if ( ImGui::Combo( "Shape", &si, shapes, IM_ARRAYSIZE( shapes ) ) )
			{
				m_shapeType = ShapeType( si );
				Launch();
			}
			if ( ImGui::Checkbox( "Fixed Rotation", &m_fixedRotation ) )
				Launch();
			if ( ImGui::Checkbox( "Enable Hit Events", &m_enableHitEvents ) )
				b2Body_EnableHitEvents( m_bodyId, m_enableHitEvents );
			ImGui::Checkbox( "Enable Screen Shake", &m_enableScreenShake );
			ImGui::Checkbox( "Enable Hit Sounds", &m_enableHitSounds );
			ImGui::Checkbox( "Enable Duplication", &m_enableDuplication );
			ImGui::Separator();
			ImGui::Text( "Physical Parameters (Object)" );
			bool update = false;
			update |= ImGui::SliderFloat( "Restitution (object)", &m_restitution, 0.0f, 2.0f, "%.2f" );
			update |= ImGui::SliderFloat( "Friction (object)", &m_friction, 0.0f, 2.0f, "%.2f" );
			update |= ImGui::SliderFloat( "Density", &m_density, 0.01f, 10.0f, "%.2f" );
			update |= ImGui::SliderFloat( "Linear Damping", &m_linearDamping, 0.0f, 10.0f, "%.2f" );
			update |= ImGui::SliderFloat( "Angular Damping", &m_angularDamping, 0.0f, 10.0f, "%.2f" );
			update |= ImGui::SliderFloat( "Gravity Scale", &m_gravityScale, 0.0f, 5.0f, "%.2f" );
			update |= ImGui::SliderFloat( "Angular Velocity", &m_initialAngularVelocity, -50.0f, 50.0f, "%.2f" );
			if ( update )
				Launch();

			ImGui::Separator();
			ImGui::Text( "Physical Parameters (Ground)" );
			CreateArena();

			const char* colors[] = { "Lime", "Magenta", "Orange", "Red", "Yellow" };
			static uint32_t vals[5] = { 0x00FF00, 0xFF00FF, 0xFFA500, 0xFF0000, 0xFFFF00 };
			int ci = m_uniformColorIndex;
			if ( ImGui::Combo( "Uniform Color", &ci, colors, IM_ARRAYSIZE( colors ) ) )
			{
				m_uniformColorIndex = ci;
				m_uniformColor = vals[ci];
				CreateArena();
				Launch();
			}

			static float vol = 10.0f;
			if ( ImGui::SliderFloat( "Audio Volume", &vol, 0.0f, 100.0f ) )
			{
				m_hitAudio.SetVolume( vol );
				m_destroyAudio.SetVolume( vol );
			}
			ImGui::Text( "Sounds Loaded: %d", int( m_hitAudio.GetSoundCount() ) );
			ImGui::End();
		}

		// === HUD titres/argent : TOUJOURS visible (même si UI masqué) ===
		ImFont* largestFont = m_context->draw.m_largeFont ? m_context->draw.m_largeFont : ImGui::GetFont();
		ImFont* titleFont = m_context->draw.m_mediumFont ? m_context->draw.m_mediumFont : ImGui::GetFont();
		ImVec2 screenSize = io.DisplaySize;

		auto DrawTextOutline = []( ImFont* font, float fontSize, ImVec2 pos, ImU32 color, const char* text ) {
			for ( int ox = 0; ox <= 3; ++ox )
				for ( int oy = 0; oy <= 3; ++oy )
					if ( ox != 0 || oy != 0 )
						ImGui::GetForegroundDrawList()->AddText( font, fontSize, ImVec2( pos.x + ox, pos.y + oy ), color, text );
		};

		// -------- Calcul proportions des cages --------
		const float cageHeight = screenSize.y * 0.23f;
		const float cageSpacing = screenSize.y * 0.085f;
		const float cagesBlockHeight = cageHeight * 2 + cageSpacing;
		const float cagesBlockY0 = ( screenSize.y - cagesBlockHeight ) * 0.5f;
		ImVec2 cage1Center = { screenSize.x * 0.5f, cagesBlockY0 + cageHeight * 0.5f };
		ImVec2 cage2Center = { screenSize.x * 0.5f, cagesBlockY0 + cageHeight * 1.5f + cageSpacing };

		// --- HUD dans la première cage (argent, multiplicateur, etc) ---
		const char* subtitle = "Your current net worth";
		ImVec2 subtitleSize = titleFont->CalcTextSizeA( titleFont->FontSize, FLT_MAX, 0.0f, subtitle );
		ImVec2 subtitlePos = ImVec2( cage1Center.x - subtitleSize.x * 0.5f, cage1Center.y - subtitleSize.y * 0.8f );
		DrawTextOutline( titleFont, titleFont->FontSize, subtitlePos, IM_COL32( 0, 0, 0, 255 ), subtitle );
		ImGui::GetForegroundDrawList()->AddText( titleFont, titleFont->FontSize, subtitlePos, IM_COL32( 255, 255, 255, 255 ),
												 subtitle );

		char moneyText[64];
		FormatMoney( m_money, moneyText, sizeof( moneyText ) );
		ImVec2 moneySize = largestFont->CalcTextSizeA( largestFont->FontSize, FLT_MAX, 0.0f, moneyText );
		ImVec2 moneyPos = ImVec2( cage1Center.x - moneySize.x * 0.5f, subtitlePos.y + subtitleSize.y + 12.0f );
		DrawTextOutline( largestFont, largestFont->FontSize, moneyPos, IM_COL32( 0, 0, 0, 255 ), moneyText );
		ImGui::GetForegroundDrawList()->AddText( largestFont, largestFont->FontSize, moneyPos, IM_COL32( 57, 255, 20, 255 ),
												 moneyText );

		char multiplierText[64];
		snprintf( multiplierText, sizeof( multiplierText ), "Each bounce: x%.2f", m_moneyMultiplier );
		ImVec2 multSize = titleFont->CalcTextSizeA( titleFont->FontSize * 0.85f, FLT_MAX, 0.0f, multiplierText );
		ImVec2 multPos = ImVec2( cage1Center.x - multSize.x * 0.5f, moneyPos.y + moneySize.y + 10.0f );
		DrawTextOutline( titleFont, titleFont->FontSize * 0.85f, multPos, IM_COL32( 0, 0, 0, 180 ), multiplierText );
		ImGui::GetForegroundDrawList()->AddText( titleFont, titleFont->FontSize * 0.85f, multPos, IM_COL32( 255, 255, 255, 255 ),
												 multiplierText );

		// --- Titre "Year" au centre de la DEUXIÈME cage ---
		const char* yearText = "In";
		ImU32 yearColor = 0xFF00A5FF; // Orange pour le compteur seulement
		float yearFontSize = titleFont->FontSize * 1.0f;
		ImVec2 yearSize = titleFont->CalcTextSizeA( yearFontSize, FLT_MAX, 0.0f, yearText );
		ImVec2 yearPos = ImVec2( cage2Center.x - yearSize.x * 0.5f, cage2Center.y - yearSize.y * 0.67f );

		// Titre "Year" en BLANC
		DrawTextOutline( titleFont, yearFontSize, yearPos, IM_COL32( 0, 0, 0, 255 ), yearText );
		ImGui::GetForegroundDrawList()->AddText( titleFont, yearFontSize, yearPos, IM_COL32( 255, 255, 255, 255 ), yearText );

		// --- Compteur sous "Year" en big font (largestFont) ---
		char yearCounterText[32];
		snprintf( yearCounterText, sizeof( yearCounterText ), "%d", m_year );
		float counterFontSize = largestFont->FontSize * 1.0f;
		ImVec2 counterSize = largestFont->CalcTextSizeA( counterFontSize, FLT_MAX, 0.0f, yearCounterText );
		ImVec2 counterPos = ImVec2( cage2Center.x - counterSize.x * 0.5f, yearPos.y + yearSize.y + 4.0f );
		DrawTextOutline( largestFont, counterFontSize, counterPos, IM_COL32( 0, 0, 0, 255 ), yearCounterText );
		ImGui::GetForegroundDrawList()->AddText( largestFont, counterFontSize, counterPos, yearColor, yearCounterText );
	}

	void Step() override
	{
		Sample::Step();

		b2ContactEvents contactEvents = b2World_GetContactEvents( m_worldId );
		for ( int i = 0; i < contactEvents.hitCount; ++i )
		{
			const b2ContactHitEvent& ev = contactEvents.hitEvents[i];

			// Récupère les filters des deux shapes du contact
			b2Filter filterA = b2Shape_GetFilter( ev.shapeIdA );
			b2Filter filterB = b2Shape_GetFilter( ev.shapeIdB );

			// On cherche qui est le projectile (0x10)
			bool aIsProjectile = ( filterA.categoryBits == 0x10 );
			bool bIsProjectile = ( filterB.categoryBits == 0x10 );

			// On veut un contact entre le projectile et un mur (argent ou année)
			if ( aIsProjectile == bIsProjectile )
				continue;

			// Le mur, c'est celui qui n'est PAS le projectile
			b2Filter wallFilter = aIsProjectile ? filterB : filterA;

			// **Maintenant tu délègues !**
			m_hitAudio.HandleHitEffect( ev.point, ev.approachSpeed, m_stepCount );

			// -- Money logic --
			if ( wallFilter.categoryBits == 0x01 )
				m_money *= m_moneyMultiplier; // cage du haut
			else if ( wallFilter.categoryBits == 0x02 )
				m_year += 1; // cage du bas
		}

		m_hitAudio.PlayQueued();

		if ( m_enableScreenShake )
			ApplyShakeEffect();

		// Affichage des effets d'impact
		m_hitAudio.DrawHitEffects( &m_context->draw, m_stepCount );
	}

	void DuplicateAt( const b2Vec2& position )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = position;
		bodyDef.linearVelocity = { 10.0f, 0.0f };
		bodyDef.gravityScale = m_gravityScale;
		bodyDef.motionLocks.angularZ = m_fixedRotation;
		bodyDef.isBullet = true;
		bodyDef.linearDamping = m_linearDamping;
		bodyDef.angularDamping = m_angularDamping;
		bodyDef.angularVelocity = m_initialAngularVelocity;
		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = m_density;
		shapeDef.enableHitEvents = true;
		shapeDef.material = b2DefaultSurfaceMaterial();
		shapeDef.material.restitution = m_restitution;
		shapeDef.material.friction = m_friction;
		shapeDef.material.customColor = m_uniformColor;

		const float RADIUS = 1.0f;
		if ( m_shapeType == e_circleShape )
		{
			b2Circle circle = { { 0.0f, 0.0f }, RADIUS };
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
		else if ( m_shapeType == e_capsuleShape )
		{
			b2Capsule capsule = { { -RADIUS, 0.0f }, { RADIUS, 0.0f }, RADIUS * 0.5f };
			b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
		}
		else if ( m_shapeType == e_boxShape )
		{
			b2Polygon box = b2MakeBox( RADIUS, RADIUS );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
		else
		{
			int sides = int( m_shapeType ) - int( e_polygon3 ) + 3;
			std::vector<b2Vec2> vertices( sides );
			const float twoPi = 2.0f * b2_pi;
			for ( int i = 0; i < sides; ++i )
				vertices[i] = { std::cos( i * twoPi / sides ) * RADIUS, std::sin( i * twoPi / sides ) * RADIUS };
			b2Hull hull = b2ComputeHull( vertices.data(), sides );
			if ( hull.count > 0 )
			{
				b2Polygon polygon = b2MakePolygon( &hull, 0 );
				b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
			}
		}
	}

private:
	b2BodyId m_bodyId{ b2_nullBodyId };
	b2BodyId m_wallBodyId1{ b2_nullBodyId };
	b2BodyId m_wallBodyId2{ b2_nullBodyId };
	b2Vec2 lastImpactPosition{ 0, 0 };
	int lastImpactTime;
	int shakeDuration;
	float shakeIntensity;
	b2Vec2 cameraBasePosition{ 0, 0 };

	AudioManager m_hitAudio;
	AudioManager m_destroyAudio;
	size_t m_currentSoundIndex = 0;
	size_t m_destroySoundIndex = 0;

	std::mt19937 m_rng{ std::random_device{}() };
	std::uniform_real_distribution<float> m_velDist{ -10.0f, 10.0f };

	static constexpr float MIN_DISTANCE_BETWEEN_IMPACTS = 0.5f;
	static constexpr int MIN_STEPS_BETWEEN_IMPACTS = 2;

	void CreateArena()
	{
		// Détruire les anciennes cages si besoin
		if ( B2_IS_NON_NULL( m_wallBodyId1 ) )
			b2DestroyBody( m_wallBodyId1 );
		if ( B2_IS_NON_NULL( m_wallBodyId2 ) )
			b2DestroyBody( m_wallBodyId2 );

		// Paramètres cages
		const float hw = 11.0f, hh = 11.0f, t = 0.5f, halfT = t * 0.5f;
		const float xMin = -hw, xMax = +hw;
		const float cageSpacing = 0.0f; // Espace vertical entre les deux cages
		const float y_center_top = hh + cageSpacing * 0.5f;
		const float y_center_bot = -hh - cageSpacing * 0.5f;

		// --- Cage du haut (ARGENT, couleur personnalisée, AVEC TOIT, catégorie 0x01) ---
		{
			b2BodyDef bodyDef1 = b2DefaultBodyDef();
			m_wallBodyId1 = b2CreateBody( m_worldId, &bodyDef1 );
			b2ShapeDef shapeDef1 = b2DefaultShapeDef();
			shapeDef1.enableHitEvents = true;
			shapeDef1.material = b2DefaultSurfaceMaterial();
			shapeDef1.material.customColor = m_uniformColor;
			shapeDef1.filter.categoryBits = 0x01;

			// Murs verticaux
			{
				float cy = y_center_top;
				float cx = xMin + halfT;
				b2Polygon box = b2MakeOffsetBox( halfT, hh, { cx, cy }, b2MakeRot( 0 ) );
				b2CreatePolygonShape( m_wallBodyId1, &shapeDef1, &box );
			}
			{
				float cy = y_center_top;
				float cx = xMax - halfT;
				b2Polygon box = b2MakeOffsetBox( halfT, hh, { cx, cy }, b2MakeRot( 0 ) );
				b2CreatePolygonShape( m_wallBodyId1, &shapeDef1, &box );
			}
			// Toit
			{
				float cx = 0.0f, cy_top = y_center_top + hh - halfT;
				b2Polygon box = b2MakeOffsetBox( hw, halfT, { cx, cy_top }, b2MakeRot( 0 ) );
				b2CreatePolygonShape( m_wallBodyId1, &shapeDef1, &box );
			}
			// Fond incliné
			const float angleLeft = -25.0f * b2_pi / 180.0f, angleRight = +25.0f * b2_pi / 180.0f;
			const float wallLen = hw * 0.97f, halfLen = wallLen * 0.5f;
			{
				b2Vec2 origin = { xMin, y_center_top - hh };
				b2Rot rot = b2MakeRot( angleLeft );
				b2Vec2 offset = { rot.c * halfLen - rot.s * halfT, rot.s * halfLen + rot.c * halfT };
				b2Vec2 center = { origin.x + offset.x, origin.y + offset.y };
				b2Polygon box = b2MakeOffsetBox( halfLen, halfT, center, rot );
				b2CreatePolygonShape( m_wallBodyId1, &shapeDef1, &box );
			}
			{
				b2Vec2 origin = { xMax, y_center_top - hh };
				b2Rot rot = b2MakeRot( angleRight );
				b2Vec2 offset = { -rot.c * halfLen - rot.s * halfT, -rot.s * halfLen + rot.c * halfT };
				b2Vec2 center = { origin.x + offset.x, origin.y + offset.y };
				b2Polygon box = b2MakeOffsetBox( halfLen, halfT, center, rot );
				b2CreatePolygonShape( m_wallBodyId1, &shapeDef1, &box );
			}
		}

		// --- Cage du bas (ANNÉE, orange, SANS TOIT, catégorie 0x02) ---
		{
			b2BodyDef bodyDef2 = b2DefaultBodyDef();
			m_wallBodyId2 = b2CreateBody( m_worldId, &bodyDef2 );
			b2ShapeDef shapeDef2 = b2DefaultShapeDef();
			shapeDef2.enableHitEvents = true;
			shapeDef2.material = b2DefaultSurfaceMaterial();
			shapeDef2.material.customColor = 0xFFA500; // Orange
			shapeDef2.filter.categoryBits = 0x02;

			// Murs verticaux
			{
				float cy = y_center_bot;
				float cx = xMin + halfT;
				b2Polygon box = b2MakeOffsetBox( halfT, hh, { cx, cy }, b2MakeRot( 0 ) );
				b2CreatePolygonShape( m_wallBodyId2, &shapeDef2, &box );
			}
			{
				float cy = y_center_bot;
				float cx = xMax - halfT;
				b2Polygon box = b2MakeOffsetBox( halfT, hh, { cx, cy }, b2MakeRot( 0 ) );
				b2CreatePolygonShape( m_wallBodyId2, &shapeDef2, &box );
			}
			// Pas de toit
			// Fond incliné
			const float angleLeft = -25.0f * b2_pi / 180.0f, angleRight = +25.0f * b2_pi / 180.0f;
			const float wallLen = hw * 0.97f, halfLen = wallLen * 0.5f;
			{
				b2Vec2 origin = { xMin, y_center_bot - hh };
				b2Rot rot = b2MakeRot( angleLeft );
				b2Vec2 offset = { rot.c * halfLen - rot.s * halfT, rot.s * halfLen + rot.c * halfT };
				b2Vec2 center = { origin.x + offset.x, origin.y + offset.y };
				b2Polygon box = b2MakeOffsetBox( halfLen, halfT, center, rot );
				b2CreatePolygonShape( m_wallBodyId2, &shapeDef2, &box );
			}
			{
				b2Vec2 origin = { xMax, y_center_bot - hh };
				b2Rot rot = b2MakeRot( angleRight );
				b2Vec2 offset = { -rot.c * halfLen - rot.s * halfT, -rot.s * halfLen + rot.c * halfT };
				b2Vec2 center = { origin.x + offset.x, origin.y + offset.y };
				b2Polygon box = b2MakeOffsetBox( halfLen, halfT, center, rot );
				b2CreatePolygonShape( m_wallBodyId2, &shapeDef2, &box );
			}
		}
	}

	void Launch()
	{
		if ( B2_IS_NON_NULL( m_bodyId ) )
			b2DestroyBody( m_bodyId );
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.linearVelocity = { m_velDist( m_rng ), m_velDist( m_rng ) };
		bodyDef.position = { 0, 20 };
		bodyDef.gravityScale = m_gravityScale;
		bodyDef.motionLocks.angularZ = m_fixedRotation;
		bodyDef.linearDamping = m_linearDamping;
		bodyDef.angularDamping = m_angularDamping;
		bodyDef.angularVelocity = m_initialAngularVelocity;
		m_bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = m_density;
		shapeDef.enableHitEvents = m_enableHitEvents;
		shapeDef.material = b2DefaultSurfaceMaterial();
		shapeDef.material.restitution = m_restitution;
		shapeDef.material.friction = m_friction;
		shapeDef.material.customColor = m_uniformColor;
		shapeDef.filter.categoryBits = 0x10;

		const float RADIUS = 1.0f;
		if ( m_shapeType == e_circleShape )
		{
			b2Circle c = { { 0, 0 }, RADIUS };
			b2CreateCircleShape( m_bodyId, &shapeDef, &c );
		}
		else if ( m_shapeType == e_capsuleShape )
		{
			b2Capsule c = { { -RADIUS, 0 }, { RADIUS, 0 }, RADIUS * 0.5f };
			b2CreateCapsuleShape( m_bodyId, &shapeDef, &c );
		}
		else if ( m_shapeType == e_boxShape )
		{
			b2Polygon p = b2MakeBox( RADIUS, RADIUS );
			b2CreatePolygonShape( m_bodyId, &shapeDef, &p );
		}
		else
		{
			int sides = int( m_shapeType ) - int( e_polygon3 ) + 3;
			std::vector<b2Vec2> verts( sides );
			float twoPi = 2.0f * b2_pi;
			for ( int i = 0; i < sides; i++ )
				verts[i] = { std::cos( i * twoPi / sides ) * RADIUS, std::sin( i * twoPi / sides ) * RADIUS };
			b2Hull h = b2ComputeHull( verts.data(), sides );
			if ( h.count > 0 )
			{
				b2Polygon pp = b2MakePolygon( &h, 0 );
				b2CreatePolygonShape( m_bodyId, &shapeDef, &pp );
			}
		}
	}

	void HandleHitEffects( const b2Vec2& pt )
	{
		cameraBasePosition = m_context->camera.m_center;
		lastImpactPosition = pt;
		lastImpactTime = m_stepCount;
		if ( m_enableHitSounds && m_hitAudio.GetSoundCount() > 0 )
		{
			m_hitAudio.QueueSound( m_currentSoundIndex );
			m_currentSoundIndex = ( m_currentSoundIndex + 1 ) % m_hitAudio.GetSoundCount();
		}
		if ( m_enableScreenShake )
		{
			shakeDuration = 15;
			shakeIntensity = 0.3f;
		}
	}

	void InitializeAudio()
	{
		m_hitAudio.LoadFromDirectory( "D:/Sound & Fx/audio/glo" );
		m_destroyAudio.LoadFromDirectory( "D:/Sound & Fx/audio/glo" );
		m_hitAudio.SetVolume( 10.0f );
		m_destroyAudio.SetVolume( 10.0f );
	}

	void ApplyShakeEffect()
	{
		if ( shakeDuration > 0 )
		{
			float sx = ( rand() / float( RAND_MAX ) - 0.5f ) * shakeIntensity;
			float sy = ( rand() / float( RAND_MAX ) - 0.5f ) * shakeIntensity;
			m_context->camera.m_center.x = cameraBasePosition.x + sx;
			m_context->camera.m_center.y = cameraBasePosition.y + sy;
			--shakeDuration;
			shakeIntensity *= 0.9f;
		}
		else
		{
			const float T = 0.01f, R = 0.15f;
			if ( std::abs( m_context->camera.m_center.x - cameraBasePosition.x ) < T &&
				 std::abs( m_context->camera.m_center.y - cameraBasePosition.y ) < T )
				m_context->camera.m_center = cameraBasePosition;
			else
			{
				m_context->camera.m_center.x += ( cameraBasePosition.x - m_context->camera.m_center.x ) * R;
				m_context->camera.m_center.y += ( cameraBasePosition.y - m_context->camera.m_center.y ) * R;
			}
		}
	}

	// Ajoute une fonction utilitaire pour formatage "$ 1 234 567"
	// Formate "$ 1,234,567.89"
	static void FormatMoney( double amount, char* out, size_t outSize )
	{
		char buf[64];
		snprintf( buf, sizeof( buf ), "%.2f", amount );

		// Sépare partie entière et décimale
		char* dot = strchr( buf, '.' );
		size_t intLen = dot ? (size_t)( dot - buf ) : strlen( buf );

		// Copie la partie entière avec virgules
		size_t outPos = 0;
		out[outPos++] = '$';
		out[outPos++] = ' ';

		for ( size_t i = 0; i < intLen; ++i )
		{
			if ( i > 0 && ( ( intLen - i ) % 3 == 0 ) )
				out[outPos++] = ',';
			out[outPos++] = buf[i];
		}

		// Copie la partie décimale (le point et les deux chiffres)
		if ( dot )
		{
			out[outPos++] = '.';
			out[outPos++] = dot[1] ? dot[1] : '0';
			out[outPos++] = dot[2] ? dot[2] : '0';
		}
		out[outPos] = 0;
	}
};
static int wichyearSample = RegisterSample( "9:16", "Wichyear", Wichyear::Create );

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

class PlinkoSample : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new PlinkoSample( context );
	}

	explicit PlinkoSample( SampleContext* context )
		: Sample( context )
	{
		// ---- Centrage vertical automatique ----
		float top = pinYOffset + rowSpacing * ( numRows - 2 );
		float baseY = pinYOffset - rowSpacing * 0.8f;
		float bottom = baseY - 0.45f;
		float centerY = 0.5f * ( top + bottom );

		if ( !m_context->restart )
		{
			m_context->camera.m_center = { 0.0f, centerY };
			m_context->camera.m_zoom = 20.0f;
		}
		b2World_SetGravity( m_worldId, { 0.0f, -15.0f } );

		CreatePlinkoPins();
		CreateSensors();
	}

private:
	static constexpr int numRows = 8;
	static constexpr float ballRadius = 0.5f;
	static constexpr float pinRadius = 0.2f;
	static constexpr float rowSpacing = 2.0f;
	static constexpr float colSpacing = 2.0f;
	static constexpr int numSlots = numRows + 1;
	static constexpr float pinYOffset = 8.0f;
	static constexpr float sensorSize = 1.5f;

	// Multiplicateurs pour 7 sensors (8 pins)
	float m_multipliers[numRows - 1] = { 10, 3.5, 0.9, 0.5, 0.9, 3.5, 10 };

	std::vector<b2BodyId> m_pins;
	std::vector<b2BodyId> m_sensors;

	// Ajoute une variable d'arrondi (modifiable live)
	float m_sensorRoundness = 0.1f;

	static uint32_t InterpColor( uint32_t c1, uint32_t c2, float t )
	{
		uint8_t r1 = ( c1 >> 16 ) & 0xFF, g1 = ( c1 >> 8 ) & 0xFF, b1 = c1 & 0xFF;
		uint8_t r2 = ( c2 >> 16 ) & 0xFF, g2 = ( c2 >> 8 ) & 0xFF, b2 = c2 & 0xFF;
		uint8_t r = static_cast<uint8_t>( r1 + t * ( r2 - r1 ) );
		uint8_t g = static_cast<uint8_t>( g1 + t * ( g2 - g1 ) );
		uint8_t b = static_cast<uint8_t>( b1 + t * ( b2 - b1 ) );
		return ( r << 16 ) | ( g << 8 ) | b;
	}

	void CreatePlinkoPins()
	{
		for ( b2BodyId id : m_pins )
			if ( B2_IS_NON_NULL( id ) )
				b2DestroyBody( id );
		m_pins.clear();

		for ( int row = 1; row < numRows; ++row )
		{
			int pinsThisRow = row + 1;
			float y = pinYOffset + rowSpacing * ( numRows - 1 - row );
			float xOffset = -colSpacing * 0.5f * row;
			for ( int i = 0; i < pinsThisRow; ++i )
			{
				float x = xOffset + i * colSpacing;
				b2BodyDef bd = b2DefaultBodyDef();
				bd.type = b2_staticBody;
				bd.position = { x, y };
				b2BodyId pinBody = b2CreateBody( m_worldId, &bd );

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 0.35f;
				sd.material.friction = 0.5f;
				sd.material.customColor = 0xFFFFFFFF;

				b2Circle circle = { { 0, 0 }, pinRadius };
				b2CreateCircleShape( pinBody, &sd, &circle );

				m_pins.push_back( pinBody );
			}
		}
	}

	void CreateSensors()
	{
		for ( b2BodyId id : m_sensors )
			if ( B2_IS_NON_NULL( id ) )
				b2DestroyBody( id );
		m_sensors.clear();

		const int lastRowPins = numRows;
		const int numSensors = lastRowPins - 1;
		const float baseY = pinYOffset - rowSpacing * 0.8f;
		const float totalWidth = ( lastRowPins - 1 ) * colSpacing;
		const float startX = -0.5f * totalWidth;

		for ( int s = 0; s < numSensors; ++s )
		{
			float x_left = startX + s * colSpacing;
			float x_right = startX + ( s + 1 ) * colSpacing;
			float x = 0.5f * ( x_left + x_right );

			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_staticBody;
			bd.position = { x, baseY - 0.45f };

			b2BodyId sensor = b2CreateBody( m_worldId, &bd );

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.isSensor = true;

			float t = numSensors > 1 ? float( s ) / ( numSensors - 1 ) : 0.0f;
			uint32_t colorLeft = 0xFF2211;
			uint32_t colorCenter = 0xFFEE33;
			uint32_t color;
			if ( t < 0.5f )
				color = InterpColor( colorLeft, colorCenter, t * 2.0f );
			else
				color = InterpColor( colorCenter, colorLeft, ( t - 0.5f ) * 2.0f );

			sd.material = b2DefaultSurfaceMaterial();
			sd.material.customColor = color;

			// Rounded box
			b2Polygon rounded = b2MakeRoundedBox( sensorSize * 0.5f, sensorSize * 0.5f, m_sensorRoundness );
			b2CreatePolygonShape( sensor, &sd, &rounded );

			m_sensors.push_back( sensor );
		}
	}

	void UpdateGui() override
	{
		ImGuiIO& io = ImGui::GetIO();
		ImFont* multFont = m_context->draw.m_mediumFont ? m_context->draw.m_mediumFont : ImGui::GetFont();

		// Slider pour tester l'arrondi en temps réel
		ImGui::SetNextWindowPos( ImVec2( 20, 20 ), ImGuiCond_Always );
		ImGui::SetNextWindowSize( ImVec2( 250, 0 ), ImGuiCond_Always );
		ImGui::Begin( "Sensor Arrondi" );
		if ( ImGui::SliderFloat( "Arrondi capteurs", &m_sensorRoundness, 0.0f, sensorSize * 0.5f, "%.2f" ) )
		{
			CreateSensors(); // Regénère les sensors à chaque changement d'arrondi
		}
		ImGui::End();

		// Multiplicateurs attachés aux sensors
		for ( int i = 0; i < (int)m_sensors.size(); ++i )
		{
			b2BodyId sensor = m_sensors[i];
			b2Vec2 pos = b2Body_GetPosition( sensor );
			b2Vec2 screen = m_context->camera.ConvertWorldToScreen( pos );

			char multText[16];
			// Test si entier ou non, tolérance 1e-6 pour éviter les imprécisions flottantes
			if ( std::fabs( m_multipliers[i] - int( m_multipliers[i] ) ) < 1e-6 )
				snprintf( multText, sizeof( multText ), "%dx", int( m_multipliers[i] ) );
			else
				snprintf( multText, sizeof( multText ), "%.1fx", m_multipliers[i] );

			ImVec2 textSize = multFont->CalcTextSizeA( multFont->FontSize, FLT_MAX, 0.0f, multText );
			float x = screen.x - textSize.x * 0.5f;
			float y = screen.y - textSize.y * 0.5f;

			ImGui::GetForegroundDrawList()->AddText( multFont, multFont->FontSize, ImVec2( x, y ), IM_COL32( 255, 255, 255, 255 ),
													 multText );
		}

	}

	void Step() override
	{
		Sample::Step();

		if ( ImGui::IsKeyPressed( ImGuiKey_Space ) )
		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = { 0.0f, pinYOffset + rowSpacing * numRows + 2.0f };
			bd.linearDamping = 0.03f;
			bd.angularDamping = 0.02f;
			bd.isBullet = true;
			b2BodyId ball = b2CreateBody( m_worldId, &bd );

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 1.5f;
			sd.material = b2DefaultSurfaceMaterial();
			sd.material.restitution = 0.65f;
			sd.material.friction = 0.12f;
			sd.material.customColor = 0x30c8ff;

			b2Circle circ = { { 0, 0 }, ballRadius };
			b2CreateCircleShape( ball, &sd, &circ );
		}
	}
};

static int samplePlinko = RegisterSample( "9:16", "Plinko (simple)", PlinkoSample::Create );
