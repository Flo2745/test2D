#include "audiomanager.h"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <regex>
#include <stdexcept>

// === Trie naturel (numérique + alpha) pour fichiers audio ===
std::vector<std::filesystem::path> GetSortedFiles( const std::string& path )
{
	std::vector<std::filesystem::path> files;
	for ( const auto& entry : std::filesystem::directory_iterator( path ) )
	{
		if ( entry.is_regular_file() && entry.path().extension() == ".wav" )
			files.push_back( entry.path() );
	}
	auto naturalOrder = []( const std::filesystem::path& a, const std::filesystem::path& b ) {
		std::string sa = a.filename().string();
		std::string sb = b.filename().string();
		std::regex number_re( "(\\d+)" );
		std::sregex_token_iterator it_a( sa.begin(), sa.end(), number_re, 1 ), end_a, it_b( sb.begin(), sb.end(), number_re, 1 ),
			end_b;

		std::sregex_token_iterator ita = it_a, itb = it_b;
		while ( ita != end_a && itb != end_b )
		{
			int na = std::stoi( *ita );
			int nb = std::stoi( *itb );
			if ( na != nb )
				return na < nb;
			++ita;
			++itb;
		}
		if ( ita != end_a )
			return false;
		if ( itb != end_b )
			return true;
		return sa < sb;
	};
	std::sort( files.begin(), files.end(), naturalOrder );
	return files;
}

void AudioManager::LoadFromDirectory( const std::string& path )
{
	m_buffers.clear();
	m_sounds.clear();
	m_currentSoundIndex = 0;

	auto files = GetSortedFiles( path );

	std::cout << "Ordre des sons chargés :" << std::endl;
	for ( size_t i = 0; i < files.size(); ++i )
		std::cout << "[" << i << "] " << files[i].filename().string() << std::endl;

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
	m_currentSoundIndex = 0;
}

void AudioManager::HandleHitEffect( const b2Vec2& impactPoint, float speed, int stepIndex )
{
	//printf("Impact! (%.2f, %.2f) speed %.2f\n", impactPoint.x, impactPoint.y, speed);
	if ( stepIndex - m_lastImpactStep < MIN_STEPS_BETWEEN_IMPACTS )
		return;

	b2Vec2 delta = impactPoint - m_lastImpactPosition;
	if ( b2LengthSquared( delta ) < MIN_DISTANCE_BETWEEN_IMPACTS * MIN_DISTANCE_BETWEEN_IMPACTS )
		return;

	m_lastImpactPosition = impactPoint;
	m_lastImpactStep = stepIndex;

	HitEvent* oldestEvent = &m_hitEvents[0];
	for ( auto& event : m_hitEvents )
	{
		if ( event.stepIndex < oldestEvent->stepIndex )
			oldestEvent = &event;
	}

	oldestEvent->point = impactPoint;
	oldestEvent->speed = speed;
	oldestEvent->stepIndex = stepIndex;

	QueueSound( m_currentSoundIndex );
	m_currentSoundIndex = ( m_currentSoundIndex + 1 ) % m_sounds.size();
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