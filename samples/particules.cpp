#include "particules.h"

#include "draw.h"

#include "box2d/box2d.h"

#include <algorithm>
#include <cmath>

constexpr float b2_pi = 3.14159265358979323846f; // Pi partout

// Utilitaire couleur
static void ColorToFloat4( uint32_t color, float* out )
{
	out[0] = ( ( color >> 16 ) & 0xFF ) / 255.0f;
	out[1] = ( ( color >> 8 ) & 0xFF ) / 255.0f;
	out[2] = ( ( color >> 0 ) & 0xFF ) / 255.0f;
	out[3] = 1.0f;
}

static uint32_t Float4ToColor( const float* in )
{
	uint32_t r = uint32_t( std::clamp( in[0], 0.0f, 1.0f ) * 255.0f );
	uint32_t g = uint32_t( std::clamp( in[1], 0.0f, 1.0f ) * 255.0f );
	uint32_t b = uint32_t( std::clamp( in[2], 0.0f, 1.0f ) * 255.0f );
	return ( r << 16 ) | ( g << 8 ) | b;
}

// Constructeur
PhysicParticleSystem::PhysicParticleSystem()
	: m_lifetime( 2.0f )
	, m_color( 0xFFFF00 )
	, m_radius( 0.12f )
	, m_worldId( b2_nullWorldId )
	, m_isSensor( false )
{
	SetMaxParticles( 256 );
	SyncColorToEdit();
}

void PhysicParticleSystem::SetWorld( b2WorldId world )
{
	m_worldId = world;
}

void PhysicParticleSystem::SetMaxParticles( size_t count )
{
	m_particles.resize( count );
}

void PhysicParticleSystem::Emit( const b2Vec2& pos, const b2Vec2& vel )
{
	for ( auto& p : m_particles )
	{
		if ( !p.alive )
		{
			// Détruit un body existant si nécessaire
			if ( B2_IS_NON_NULL( p.bodyId ) )
				b2DestroyBody( p.bodyId );

			// Création body dynamique ou sensor
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = pos;
			bd.linearVelocity = vel;
			bd.gravityScale = 0.0f;
			bd.isBullet = false;

			b2BodyId body = b2CreateBody( m_worldId, &bd );

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 1.0f;
			sd.material = b2DefaultSurfaceMaterial();
			sd.material.friction = 0.0f;
			sd.material.restitution = 1.0f;
			sd.material.customColor = m_color;
			sd.isSensor = m_isSensor; // <--- sensor/dynamique

			b2Circle c = { { 0, 0 }, m_radius };
			b2CreateCircleShape( body, &sd, &c );

			// Mise à jour des données particule
			p.bodyId = body;
			p.lifetime = m_lifetime;
			p.startLifetime = m_lifetime;
			p.color = m_color;
			p.radius = m_radius;
			p.alive = true;
			p.position = pos; // initialisé, sera mis à jour ensuite
			return;
		}
	}
}

void PhysicParticleSystem::EmitBurst( const b2Vec2& center, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		float angle = float( i ) / std::max( 1, count ) * 2.0f * b2_pi;
		float speed = 3.5f + 2.5f * ( float( rand() ) / RAND_MAX );
		b2Vec2 dir = { std::cos( angle ), std::sin( angle ) };
		Emit( center, dir * speed );
	}
}

void PhysicParticleSystem::Update( float dt )
{
	for ( auto& p : m_particles )
	{
		if ( !p.alive )
			continue;

		// Récupérer la position réelle du body
		if ( B2_IS_NON_NULL( p.bodyId ) )
		{
			p.position = b2Body_GetPosition( p.bodyId );
			// Tu peux aussi ajouter des effets physiques ici
		}

		p.lifetime -= dt;
		if ( p.lifetime < 0 )
		{
			if ( B2_IS_NON_NULL( p.bodyId ) )
				b2DestroyBody( p.bodyId );
			p.bodyId = b2_nullBodyId;
			p.alive = false;
		}
	}
}

int PhysicParticleSystem::GetAliveCount() const
{
	int n = 0;
	for ( const auto& p : m_particles )
		if ( p.alive )
			++n;
	return n;
}

float* PhysicParticleSystem::GetColorPtr()
{
	SyncColorToEdit();
	return m_colorEdit;
}

void PhysicParticleSystem::SyncColorToEdit()
{
	ColorToFloat4( m_color, m_colorEdit );
}
void PhysicParticleSystem::SyncEditToColor()
{
	m_color = Float4ToColor( m_colorEdit );
}

void PhysicParticleSystem::SetSensor( bool isSensor )
{
	m_isSensor = isSensor;
}

void PhysicParticleSystem::Render( Draw& draw )
{
	for ( const auto& p : m_particles )
	{
		if ( !p.alive )
			continue;
		float a = std::clamp( p.lifetime / p.startLifetime, 0.0f, 1.0f );
		uint32_t col = ( p.color & 0xFFFFFF ) | ( uint32_t( a * 255.0f ) << 24 );
		draw.DrawCircle( p.position, p.radius, static_cast<b2HexColor>( col ) );
	}
}




