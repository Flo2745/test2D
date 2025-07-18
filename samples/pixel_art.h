#pragma once

#include "box2d/box2d.h"

#include <cstdint>
#include <string>
#include <vector>

/// Représente un motif de pixel art couleur arbitraire (un body, plusieurs fixtures)
struct PixelArtColor
{
	int width = 0;
	int height = 0;
	std::vector<uint32_t> pixels; // Couleur par pixel (0 = transparent sinon 0xRRGGBB ou 0xAARRGGBB)

	PixelArtColor() = default;
	PixelArtColor( int w, int h )
		: width( w )
		, height( h )
		, pixels( w * h, 0 )
	{
	}

	uint32_t at( int x, int y ) const
	{
		return pixels[y * width + x];
	}
	uint32_t& at( int x, int y )
	{
		return pixels[y * width + x];
	}
};

/// Enregistre un motif couleur (appelé en statique dans pixel_art.cpp)
void PixelArtColor_Register( const std::string& name, int w, int h, const uint32_t* data );

/// Récupère un motif par son nom, nullptr si absent
const PixelArtColor* PixelArtColor_GetByName( const std::string& name );

/// Liste tous les noms de motifs couleur disponibles
std::vector<std::string> PixelArtColor_GetAllNames();

/// Renvoie un motif par son index dans la liste des noms (nullptr si invalide)
const PixelArtColor* PixelArtColor_GetByIndex( size_t idx );

/// Crée un body unique, positionné à `origin`, avec une fixture par pixel coloré
b2BodyId CreatePixelArtBody( b2WorldId worldId, const PixelArtColor& art, float pixelSize = 1.0f, b2Vec2 origin = { 0, 0 },
							 float linearDamping = 2.0f, float angularDamping = 2.0f, float restitution = 0.1f,
							 float friction = 0.6f );

/// Convertit un tableau d'indices palette (uint8_t) + palette en PixelArtColor prêt à enregistrer.
/// indices doit être de taille w*h. palette doit contenir paletteSize couleurs.
void MakePixelArtColorFromPalette( int w, int h, const uint8_t* indices, const uint32_t* palette, size_t paletteSize,
								   PixelArtColor& out );
