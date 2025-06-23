#pragma once

#include <SFML/Audio.hpp>
#include <array>
#include <box2d/box2d.h>
#include <queue>
#include <string>
#include <vector>

class Draw;


struct HitEvent
{
	b2Vec2 point{ 0.0f, 0.0f };
	float speed = 0.0f;
	int stepIndex = -1;
};

class AudioManager
{
public:
	void LoadFromDirectory( const std::string& path );
	size_t GetSoundCount() const
	{
		return m_sounds.size();
	}

	void SetVolume( float volume );
	void QueueSound( size_t index );
	void PlayQueued();
	void PlayImmediate( size_t index );
	void PlaySpawnSound();
	void Clear();

	// Nouvelle méthode pour gérer les impacts
	void HandleHitEffect( const b2Vec2& impactPoint, float speed, int stepIndex );
	// Dans audiomanager.h
	void DrawHitEffects( Draw* draw, int currentStep );

private:
	std::vector<sf::SoundBuffer> m_buffers;
	std::vector<sf::Sound> m_sounds;
	std::queue<size_t> m_queue;
	size_t m_nextPlayIndex = 0;
	float m_volume = 10.0f;

	// Gestion des HitEvents
	std::array<HitEvent, 4> m_hitEvents;
	int m_lastImpactStep = -1000;
	b2Vec2 m_lastImpactPosition{ 0.0f, 0.0f };
	static constexpr float MIN_DISTANCE_BETWEEN_IMPACTS = 0.05f;
	static constexpr int MIN_STEPS_BETWEEN_IMPACTS = 2;
};
