#include "audiomanager.h"
#include "draw.h"
#include <algorithm>
#include <filesystem>
#include <stdexcept>

// Fonction utilitaire pour trier les fichiers audio
std::vector<std::filesystem::path> GetSortedFiles( const std::string& path )
{
	std::vector<std::filesystem::path> files;
	for ( const auto& entry : std::filesystem::directory_iterator( path ) )
	{
		if ( entry.is_regular_file() && entry.path().extension() == ".wav" )
		{
			files.push_back( entry.path() );
		}
	}
	std::sort( files.begin(), files.end() );
	return files;
}

void AudioManager::LoadFromDirectory( const std::string& path )
{
	m_buffers.clear();
	m_sounds.clear();

	auto files = GetSortedFiles( path );
	for ( const auto& file : files )
	{
		sf::SoundBuffer buffer;
		if ( !buffer.loadFromFile( file.string() ) )
			throw std::runtime_error( "Cannot load sound: " + file.string() );

		m_buffers.push_back( buffer );
		m_sounds.emplace_back( buffer );
		m_sounds.back().setVolume( m_volume );
	}
}

void AudioManager::SetVolume( float volume )
{
	m_volume = volume;
	for ( auto& s : m_sounds )
		s.setVolume( volume );
}

void AudioManager::QueueSound( size_t index )
{
	if ( index < m_sounds.size() )
		m_queue.push( index );
}

void AudioManager::PlayQueued()
{
	while ( !m_queue.empty() )
	{
		PlayImmediate( m_queue.front() );
		m_queue.pop();
	}
}

void AudioManager::PlayImmediate( size_t index )
{
	if ( index >= m_sounds.size() )
		return;

	sf::Sound& s = m_sounds[m_nextPlayIndex];
	if ( s.getStatus() != sf::Sound::Status::Playing )
	{
		s.setBuffer( m_buffers[index] );
		s.play();
		m_nextPlayIndex = ( m_nextPlayIndex + 1 ) % m_sounds.size();
	}
}

void AudioManager::PlaySpawnSound()
{
	if ( !m_sounds.empty() )
		PlayImmediate( rand() % m_sounds.size() );
}

void AudioManager::Clear()
{
	m_buffers.clear();
	m_sounds.clear();
	std::queue<size_t> empty;
	std::swap( m_queue, empty );
	m_nextPlayIndex = 0;
}

// Nouvelle méthode pour gérer les impacts
void AudioManager::HandleHitEffect( const b2Vec2& impactPoint, float speed, int stepIndex )
{
	printf( "Impact! (%.2f, %.2f) speed %.2f\n", impactPoint.x, impactPoint.y, speed );
	// Vérification de la distance minimale entre les impacts
	if ( stepIndex - m_lastImpactStep < MIN_STEPS_BETWEEN_IMPACTS )
		return;

	b2Vec2 delta = impactPoint - m_lastImpactPosition;
	if ( b2LengthSquared( delta ) < MIN_DISTANCE_BETWEEN_IMPACTS * MIN_DISTANCE_BETWEEN_IMPACTS )
		return;

	// Mise à jour des informations d'impact
	m_lastImpactPosition = impactPoint;
	m_lastImpactStep = stepIndex;

	// Sélection de l'événement le plus ancien
	HitEvent* oldestEvent = &m_hitEvents[0];
	for ( auto& event : m_hitEvents )
	{
		if ( event.stepIndex < oldestEvent->stepIndex )
			oldestEvent = &event;
	}

	// Mise à jour de l'événement
	oldestEvent->point = impactPoint;
	oldestEvent->speed = speed;
	oldestEvent->stepIndex = stepIndex;

	// Mise en file du son correspondant
	QueueSound( oldestEvent->stepIndex % m_sounds.size() );
}


void AudioManager::DrawHitEffects( Draw* draw, int currentStep )
{
	for ( const auto& e : m_hitEvents )
	{
		if ( e.stepIndex > 0 && currentStep <= e.stepIndex + 30 )
		{
			draw->DrawCircle( e.point, 0.1f, b2_colorOrangeRed );
			draw->DrawString( e.point, "%.1f", e.speed );
		}
	}
}
