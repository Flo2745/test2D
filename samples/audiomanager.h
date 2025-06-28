#pragma once
#include "draw.h"

#include <SFML/Audio.hpp>
#include <filesystem>
#include <queue>
#include <vector>

struct HitEvent
{
	b2Vec2 point;
	float speed;
	int stepIndex;
};

class AudioManager
{
public:
	AudioManager()
		: m_volume( 40.0f )
		, m_nextPlayIndex( 0 )
		, m_currentSoundIndex( 0 )
		, m_lastImpactStep( -1000 )
		, m_lastImpactPosition{ 0, 0 }
	{
		m_hitEvents.resize( 8 );
	}

	void LoadFromDirectory( const std::string& path );
	void SetVolume( float volume );
	void QueueSound( size_t index );
	void PlayQueued();
	void PlayImmediate( size_t index );
	void PlaySpawnSound();
	void Clear();
	void HandleHitEffect( const b2Vec2& impactPoint, float speed, int stepIndex );
	void DrawHitEffects( Draw* draw, int currentStep );

	size_t GetSoundCount() const
	{
		return m_sounds.size();
	}

private:
	std::vector<sf::SoundBuffer> m_buffers;
	std::vector<sf::Sound> m_sounds;
	float m_volume;
	std::queue<size_t> m_queue;
	size_t m_nextPlayIndex;		// Pour cycler les channels SFML
	size_t m_currentSoundIndex; // Pour l’ordre séquentiel
	std::vector<HitEvent> m_hitEvents;
	int m_lastImpactStep;
	b2Vec2 m_lastImpactPosition;

	static constexpr int MIN_STEPS_BETWEEN_IMPACTS = 4;
	static constexpr float MIN_DISTANCE_BETWEEN_IMPACTS = 0.5f;
};
