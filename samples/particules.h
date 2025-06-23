#pragma once
#include "box2d/box2d.h"

#include <cstdint>
#include <vector>

// ========================
// === PIXEL ART PUBLIC ===
// ========================
enum class PixelArtType
{
	Creeper,
	Wolf,
	Zombie,
	Skeleton,
	Steve,
	Villager,
	Pig,
	Cow,
	Spider,
	Sheep,
	CaveSpider,
	Enderman
};

class PixelArt
{
public:
	static constexpr int Size = 8;
	// Renvoie le tableau 8x8 de couleurs pour le type voulu
	static const uint32_t ( &GetColors( PixelArtType type ) )[Size][Size];
};
// ========================

// Forward declaration du draw
class Draw;

struct PhysicParticle
{
	b2BodyId bodyId = b2_nullBodyId;
	b2Vec2 position{ 0, 0 };
	float lifetime = 0;
	float startLifetime = 0;
	uint32_t color = 0xFFFFFF;
	float radius = 0.1f;
	bool alive = false;
};

class PhysicParticleSystem
{
public:
	PhysicParticleSystem();

	void SetWorld( b2WorldId world );
	void SetMaxParticles( size_t count );
	int GetMaxParticles() const
	{
		return int( m_particles.size() );
	}

	void SetLifetime( float t )
	{
		m_lifetime = t;
	}
	float GetLifetime() const
	{
		return m_lifetime;
	}

	void SetColor( uint32_t color )
	{
		m_color = color;
	}
	uint32_t GetColor() const
	{
		return m_color;
	}
	float* GetColorPtr();
	void SyncEditToColor();

	void SetRadius( float r )
	{
		m_radius = r;
	}
	float GetRadius() const
	{
		return m_radius;
	}

	void Emit( const b2Vec2& pos, const b2Vec2& vel );
	void EmitBurst( const b2Vec2& center, int count );

	void Update( float dt );
	void Render( Draw& draw );

	int GetAliveCount() const;

	void SetSensor( bool sensor ); // <<---- Déclaré ici !

private:
	std::vector<PhysicParticle> m_particles;
	float m_lifetime = 2.0f;
	uint32_t m_color = 0xFFFF00;
	float m_radius = 0.12f;

	float m_colorEdit[4] = { 1, 1, 0, 1 };
	void SyncColorToEdit();

	b2WorldId m_worldId;
	bool m_isSensor = false; // <<---- Ajouté ici !
};
