#include "pixel_art.h"

#include <cassert>
#include <map>
#include <string>
#include <vector>

namespace
{
std::vector<std::string> _pixelArtColorNames;
std::map<std::string, PixelArtColor> _pixelArtColors;
} // namespace

// ========== Gestion des motifs couleur ==========

void PixelArtColor_Register( const std::string& name, int w, int h, const uint32_t* data )
{
	assert( w > 0 && h > 0 && data );
	PixelArtColor grid( w, h );
	for ( int i = 0; i < w * h; ++i )
		grid.pixels[i] = data[i];
	_pixelArtColorNames.push_back( name );
	_pixelArtColors[name] = std::move( grid );
}

const PixelArtColor* PixelArtColor_GetByName( const std::string& name )
{
	auto it = _pixelArtColors.find( name );
	if ( it != _pixelArtColors.end() )
		return &it->second;
	return nullptr;
}

std::vector<std::string> PixelArtColor_GetAllNames()
{
	return _pixelArtColorNames;
}

const PixelArtColor* PixelArtColor_GetByIndex( size_t idx )
{
	if ( idx < _pixelArtColorNames.size() )
		return PixelArtColor_GetByName( _pixelArtColorNames[idx] );
	return nullptr;
}

// ========== Création du body et des fixtures couleur ==========

b2BodyId CreatePixelArtBody( b2WorldId worldId, const PixelArtColor& art, float pixelSize, b2Vec2 origin, float linearDamping,
							 float angularDamping, float restitution, float friction )
{
	b2BodyDef bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.position = origin;
	bd.linearDamping = linearDamping;
	bd.angularDamping = angularDamping;
	b2BodyId bodyId = b2CreateBody( worldId, &bd );

	const float startX = -( art.width * pixelSize ) * 0.5f + pixelSize * 0.5f;
	const float startY = ( art.height - 1 ) * pixelSize * 0.5f;

	for ( int y = 0; y < art.height; ++y )
	{
		for ( int x = 0; x < art.width; ++x )
		{
			uint32_t color = art.at( x, y );
			if ( color == 0 )
				continue;
			b2ShapeDef sd = b2DefaultShapeDef();
			sd.material = b2DefaultSurfaceMaterial();
			sd.density = 1.0f;
			sd.material.restitution = restitution;
			sd.material.friction = friction;
			sd.material.customColor = color;
			sd.isSensor = true;
			float px = startX + x * pixelSize;
			float py = startY - y * pixelSize;
			b2Polygon box = b2MakeOffsetBox( pixelSize * 0.5f, pixelSize * 0.5f, { px, py }, b2MakeRot( 0 ) );
			b2CreatePolygonShape( bodyId, &sd, &box );
		}
	}
	return bodyId;
}

// ========== Conversion indices + palette -> PixelArtColor ==========

void MakePixelArtColorFromPalette( int w, int h, const uint8_t* indices, const uint32_t* palette, size_t paletteSize,
								   PixelArtColor& out )
{
	out.width = w;
	out.height = h;
	out.pixels.resize( w * h, 0 );
	for ( int i = 0; i < w * h; ++i )
		out.pixels[i] = ( indices[i] < paletteSize ) ? palette[indices[i]] : 0;
}

// ========== Palettes ==========

constexpr uint32_t weapon_palette[] = {
	0x00000000, // 0 (transparent, facultatif mais classique)
	0x281E0B,	// 1
	0x684E1E,	// 2
	0x493615,	// 3
	0x896727,	// 4
	0x082520,	// 5
	0x1E8A77,	// 6
	0x27B29A,	// 7
	0x2BC7AC,	// 8
	0x33EBCB,	// 9
	0x686868,	// 10
	0x6B6B6B,	// 11
	0x979797,	// 12
	0xCCCCCC,	// 13
	0xD72525,	// 14
	0x2B334C,	// 15
	0x2B334C,	// 16 (note: même couleur que 15)
	0x10131F	// 17
};

// Palette cœur
constexpr uint32_t heart_palette[] = { 0x00000000, 0xFF0000, 0xFF80C0, 0xFFFFFF };

// ========== Sprite indices 16x16 épée ==========

static const uint8_t sword_indices_16x16[16 * 16] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 9, 7, 5, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 5, 9, 7, 9, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 9, 7, 9, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5,
	9, 7, 9, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 9, 7, 9, 5, 0, 0, 0, 0, 0, 5, 5, 0, 0, 0, 5, 9, 7, 9, 5, 0, 0, 0,
	0, 0, 0, 5, 6, 5, 0, 5, 9, 7, 9, 5, 0, 0, 0, 0, 0, 0, 0, 0, 5, 7, 5, 9, 7, 9, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5,
	7, 5, 7, 9, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 7, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 5, 7, 7, 5,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 4, 1, 0, 5, 5, 6, 5, 0, 0, 0, 0, 0, 0, 5, 5, 2, 1, 0, 0, 0, 0, 5, 5, 0, 0, 0, 0,
	0, 0, 5, 6, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// ========== Sprite indices 16x16 trident ==========

static const uint8_t trident_indices_16x16[16 * 16] = {
	0, 0, 0, 0, 0, 0, 0, 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0, 0, 0, 0, 0, 0, 0, 0, 0,	 0,	 11, 13, 0,	 0,	 0,	 0,	 0,
	0, 0, 0, 0, 0, 0, 0, 0,	 11, 13, 12, 0,	 12, 13, 0,	 0, 0, 0, 0, 0, 0, 0, 0, 11, 13, 12, 0,	 12, 13, 11, 0,	 0,
	0, 0, 0, 0, 0, 0, 0, 11, 12, 0,	 12, 13, 11, 0,	 0,	 0, 0, 0, 0, 0, 0, 0, 0, 10, 6,	 12, 13, 11, 0,	 12, 13, 0,
	0, 0, 0, 0, 0, 0, 0, 6,	 8,	 6,	 11, 0,	 12, 13, 10, 0, 0, 0, 0, 0, 0, 0, 0, 6,	 7,	 7,	 6,	 12, 13, 10, 0,	 0,
	0, 0, 0, 0, 0, 0, 5, 7,	 5,	 5,	 10, 10, 10, 0,	 0,	 0, 0, 0, 0, 0, 0, 6, 7, 5,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,
	0, 0, 0, 0, 6, 8, 5, 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0, 0, 0, 0, 6, 8, 5, 0, 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,
	0, 0, 6, 8, 5, 0, 0, 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0, 0, 6, 8, 5, 0, 0, 0, 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,
	6, 7, 5, 0, 0, 0, 0, 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0, 5, 5, 0, 0, 0, 0, 0, 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0 };


// ========== Sprite indices 16x16 arc ==========

static const uint8_t bow_indices_16x16[16 * 16] = {
	0, 0, 0, 0,	 0,	 0, 0,	0, 0, 0, 0, 0,	0,	0,	0, 0, 0, 0, 0,	0,	0, 0, 0, 0, 0, 0, 0,  3,  3,  3, 3, 0, 0, 0,  0,  0, 0,
	0, 0, 0, 3,	 3,	 3, 4,	2, 2, 4, 1, 0,	0,	0,	0, 0, 0, 3, 3,	4,	2, 4, 1, 1, 1, 1, 0,  0,  0,  0, 0, 0, 3, 11, 2,  1, 1,
	1, 0, 0, 10, 0,	 0, 0,	0, 0, 0, 3, 11, 12, 11, 0, 0, 0, 0, 10, 0,	0, 0, 0, 0, 0, 3, 11, 12, 11, 0, 0, 0, 0, 10, 0,  0, 0,
	0, 0, 0, 0,	 3,	 2, 11, 0, 0, 0, 0, 10, 0,	0,	0, 0, 0, 0, 0,	3,	4, 1, 0, 0, 0, 0, 10, 0,  0,  0, 0, 0, 0, 0,  0,  3, 2,
	1, 0, 0, 0,	 10, 0, 0,	0, 0, 0, 0, 0,	0,	0,	3, 4, 1, 0, 0,	10, 0, 0, 0, 0, 0, 0, 0,  0,  0,  3, 4, 1, 0, 0,  10, 0, 0,
	0, 0, 0, 0,	 0,	 0, 0,	0, 3, 2, 1, 0,	10, 0,	0, 0, 0, 0, 0,	0,	0, 0, 0, 0, 3, 2, 1,  10, 0,  0, 0, 0, 0, 0,  0,  0, 0,
	0, 0, 0, 3,	 4,	 1, 0,	0, 0, 0, 0, 0,	0,	0,	0, 0, 0, 0, 0,	0,	1, 0, 0, 0, 0, 0, 0,  0,  0,  0, 0, 0, 0, 0 };

static const uint8_t bow_arrow_indices_16x16[16 * 16] = {
	0, 0, 0, 0,	 0,	 0,	 0,	 0, 0, 0, 0,  0, 0,	 0,	 0,	 0, 0, 0, 0, 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0, 3, 3, 3,  3,  0,
	0, 0, 0, 13, 0,	 0,	 0,	 0, 3, 3, 3,  4, 2,	 2,	 4,	 1, 0, 0, 0, 11, 12, 0,	 3,	 3,	 4,	 2,	 4, 1, 1, 1,  1,  0,
	0, 0, 0, 0,	 1,	 4,	 11, 2, 1, 1, 1,  0, 0,	 0,	 10, 0, 0, 0, 0, 0,	 3,	 1,	 4,	 11, 0,	 0,	 0, 0, 0, 0,  10, 0,
	0, 0, 0, 3,	 11, 12, 1,	 4, 0, 0, 0,  0, 0,	 10, 0,	 0, 0, 0, 0, 3,	 2,	 11, 0,	 1,	 4,	 0,	 0, 0, 0, 10, 0,  0,
	0, 0, 3, 4,	 1,	 0,	 0,	 0, 1, 4, 0,  0, 0,	 10, 0,	 0, 0, 0, 3, 2,	 1,	 0,	 0,	 0,	 0,	 1,	 4, 0, 0, 10, 0,  0,
	0, 0, 3, 4,	 1,	 0,	 0,	 0, 0, 0, 1,  4, 10, 0,	 0,	 0, 0, 3, 4, 1,	 0,	 0,	 0,	 0,	 0,	 0,	 0, 1, 0, 0,  0,  0,
	0, 3, 2, 1,	 0,	 0,	 0,	 0, 0, 0, 10, 0, 0,	 0,	 0,	 0, 0, 3, 2, 1,	 0,	 0,	 10, 10, 10, 10, 0, 0, 0, 0,  0,  0,
	0, 3, 4, 1,	 10, 10, 0,	 0, 0, 0, 0,  0, 0,	 0,	 0,	 0, 0, 0, 1, 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0, 0, 0, 0,  0,  0 };

static const uint8_t arrow_indices_16x16[16 * 16] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0,	0,	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0,	0,	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0,	0,	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 12, 11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 1, 0, 0, 0, 0, 0, 0,	0, 0, 0, 0, 0, 0,
	0, 4, 1, 0, 0, 0, 0, 0, 0,	0,	0, 0, 0, 0, 0, 0, 4, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	4, 1, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0,	4,	1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 1, 0, 0, 0, 0, 0,	0, 0, 0, 0, 0, 0,
	0, 0, 4, 1, 0, 0, 0, 0, 0,	0,	0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	0, 0, 0 };

// ========== Sprite indices 16x16 dagger ==========

static const uint8_t dagger_indices_16x16[16 * 16] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 5, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 5, 9, 7, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 9, 7, 9, 5, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5,
	0, 9, 7, 9, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 7, 5, 7, 9, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 7, 5, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 5, 7, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 4, 1, 0, 5, 5, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 5, 6, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static const uint8_t axe_indices_16x16[16 * 16] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 9, 9, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 5, 9, 7, 8, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 9, 7, 7, 7, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 9, 8, 7,
	6, 7, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 3, 7, 6, 7, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 1, 7, 7, 5, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 3, 4, 1, 0, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 4, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 1, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 4, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 4, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


// ========== Sprite indices heart 8x8 (motif exemple) ==========

static const uint8_t heart_indices_8x8[8 * 8] = { 0, 1, 1, 0, 0, 1, 1, 0, 1, 2, 1, 1, 1, 1, 2, 1, 1, 1, 1, 2, 1, 1,
												  2, 1, 1, 1, 1, 1, 1, 2, 2, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1,
												  1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static const uint8_t spear_indices_16x16[16 * 16] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 9, 9, 5, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 5, 8, 9, 9, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 8, 7, 8, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	3, 4, 8, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 1, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 4, 1, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 4, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 4, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 1, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 4, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 3, 4, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static const uint8_t crossbowfirework_indices_16x16[16 * 16] = {
	0, 0, 0, 2,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0, 0, 0, 0, 3, 2,	3,	2,	2,	2,	0,	0,	0,	0,	0,	0,	0, 0,
	0, 0, 3, 2,	 4,	 4,	 4,	 4,	 2,	 2,	 0,	 14, 14, 0,	 0, 0, 0, 0, 0, 10, 0,	0,	3,	3,	4,	4,	11, 13, 14, 14, 0, 0,
	0, 0, 0, 10, 0,	 0,	 0,	 0,	 1,	 14, 14, 13, 13, 14, 0, 0, 0, 0, 0, 10, 0,	0,	0,	1,	3,	17, 14, 14, 11, 0,	0, 0,
	0, 0, 0, 10, 0,	 0,	 1,	 3,	 17, 16, 17, 14, 4,	 2,	 0, 0, 0, 0, 0, 10, 0,	1,	3,	17, 16, 17, 3,	1,	4,	2,	0, 0,
	0, 0, 0, 10, 1,	 3,	 17, 16, 17, 3,	 1,	 0,	 3,	 4,	 2, 0, 0, 0, 0, 10, 3,	17, 16, 17, 3,	1,	0,	0,	3,	4,	2, 0,
	0, 0, 0, 10, 17, 16, 17, 3,	 1,	 0,	 0,	 0,	 0,	 4,	 2, 0, 0, 0, 1, 10, 16, 17, 3,	1,	0,	0,	0,	0,	0,	4,	3, 0,
	0, 1, 3, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 2,	 2, 2, 1, 4, 2, 3,	1,	0,	0,	0,	0,	0,	0,	0,	0,	3,	3, 0,
	1, 1, 4, 1,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0, 0, 0, 1, 1, 0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0, 0 };

static const uint8_t crossbow_indices_16x16[16 * 16] = {
	0, 0, 0,  2,  0, 0,	 0,	 0,	 0,	 0, 0, 0,  0, 0, 0,	 0, 0, 0, 3,  2,  3, 2,	 2,	 2, 0,	0,	0, 0, 0,  0, 0, 0,	0, 0, 3, 2, 4,
	4, 4, 4,  2,  2, 0,	 11, 11, 0,	 0, 0, 0,  0, 0, 10, 0, 0, 3, 3,  4,  4, 11, 0,	 0, 11, 0,	0, 0, 0,  0, 0, 10, 0, 0, 0, 1, 2,
	4, 3, 0,  11, 0, 0,	 0,	 0,	 0,	 0, 0, 10, 0, 1, 3,	 2, 2, 4, 11, 0,  0, 0,	 0,	 0, 0,	0,	0, 0, 10, 3, 3, 3,	2, 2, 4, 2, 0,
	0, 0, 0,  0,  0, 0,	 1,	 3,	 10, 2, 3, 3,  1, 4, 2,	 0, 0, 0, 0,  0,  0, 1,	 3,	 4, 2,	10, 3, 1, 0,  3, 4, 2,	0, 0, 0, 0, 1,
	3, 4, 2,  4,  3, 10, 0,	 0,	 3,	 4, 2, 0,  0, 0, 0,	 3, 4, 2, 4,  3,  1, 0,	 10, 0, 0,	4,	2, 0, 0,  0, 1, 4,	2, 4, 3, 1, 0,
	0, 0, 10, 0,  4, 3,	 0,	 0,	 1,	 3, 2, 4,  3, 1, 0,	 0, 0, 0, 0,  10, 2, 2,	 2,	 1, 4,	2,	3, 1, 0,  0, 0, 0,	0, 0, 0, 0, 3,
	3, 0, 1,  1,  4, 1,	 0,	 0,	 0,	 0, 0, 0,  0, 0, 0,	 0, 0, 0, 0,  1,  1, 0,	 0,	 0, 0,	0,	0, 0, 0,  0, 0, 0,	0, 0 };

static const uint8_t firework_indices_16x16[16 * 16] = {
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0, 0,
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0, 0,
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0, 0,
	0,	0,	0,	0,	0,	0,	0,	14, 14, 0,	0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  11, 13, 14, 14, 0, 0, 0, 0, 0, 0,
	0,	0,	0,	0,	0,	14, 14, 13, 13, 14, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  17, 14, 14, 11, 0,  0, 0, 0, 0, 0, 0,
	0,	0,	0,	0,	17, 16, 17, 14, 0,	0,	0, 0, 0, 0, 0, 0, 0,  0,  0,  17, 16, 17, 0,  0,  0,  0,  0, 0, 0, 0, 0, 0,
	0,	0,	17, 16, 17, 0,	0,	0,	0,	0,	0, 0, 0, 0, 0, 0, 0,  17, 16, 17, 0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0, 0,
	17, 16, 17, 0,	0,	0,	0,	0,	0,	0,	0, 0, 0, 0, 0, 0, 16, 17, 0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0, 0 };

// ===== Enregistrement dynamique =====
struct PixelArtColorRegisterer
{
	PixelArtColorRegisterer()
	{
		PixelArtColor swordDiamond;
		MakePixelArtColorFromPalette( 16, 16, sword_indices_16x16, weapon_palette, sizeof( weapon_palette ) / sizeof( uint32_t ),
									  swordDiamond );
		PixelArtColor_Register( "DIAMOND_SWORD", 16, 16, swordDiamond.pixels.data() );

		PixelArtColor axeDiamond;
		MakePixelArtColorFromPalette( 16, 16, axe_indices_16x16, weapon_palette, sizeof( weapon_palette ) / sizeof( uint32_t ),
									  axeDiamond );
		PixelArtColor_Register( "DIAMOND_AXE", 16, 16, axeDiamond.pixels.data() );

		PixelArtColor daggerDiamond;
		MakePixelArtColorFromPalette( 16, 16, dagger_indices_16x16, weapon_palette, sizeof( weapon_palette ) / sizeof( uint32_t ),
									  daggerDiamond );
		PixelArtColor_Register( "DIAMOND_DAGGER", 16, 16, daggerDiamond.pixels.data() );

		PixelArtColor bow;
		MakePixelArtColorFromPalette( 16, 16, bow_indices_16x16, weapon_palette, sizeof( weapon_palette ) / sizeof( uint32_t ),
									  bow );
		PixelArtColor_Register( "BOW", 16, 16, bow.pixels.data() );

		PixelArtColor bowArrow;
		MakePixelArtColorFromPalette( 16, 16, bow_arrow_indices_16x16, weapon_palette,
									  sizeof( weapon_palette ) / sizeof( uint32_t ), bowArrow );
		PixelArtColor_Register( "BOW_ARROW", 16, 16, bowArrow.pixels.data() );

		PixelArtColor arrow;
		MakePixelArtColorFromPalette( 16, 16, arrow_indices_16x16, weapon_palette, sizeof( weapon_palette ) / sizeof( uint32_t ),
									  arrow );
		PixelArtColor_Register( "ARROW", 16, 16, arrow.pixels.data() );

		PixelArtColor trident;
		MakePixelArtColorFromPalette( 16, 16, trident_indices_16x16, weapon_palette,
									  sizeof( weapon_palette ) / sizeof( uint32_t ), trident );
		PixelArtColor_Register( "TRIDENT", 16, 16, trident.pixels.data() );


		PixelArtColor heart;
		MakePixelArtColorFromPalette( 8, 8, heart_indices_8x8, heart_palette, sizeof( heart_palette ) / sizeof( uint32_t ),
									  heart );
		PixelArtColor_Register( "HEART", 8, 8, heart.pixels.data() );

		PixelArtColor spearDiamond;
		MakePixelArtColorFromPalette( 16, 16, spear_indices_16x16, weapon_palette, sizeof( weapon_palette ) / sizeof( uint32_t ),
									  spearDiamond );
		PixelArtColor_Register( "DIAMOND_SPEAR", 16, 16, spearDiamond.pixels.data() );

		PixelArtColor crossbowFirework;
		MakePixelArtColorFromPalette( 16, 16, crossbowfirework_indices_16x16, weapon_palette,
									  sizeof( weapon_palette ) / sizeof( uint32_t ), crossbowFirework );
		PixelArtColor_Register( "CROSSBOWFIREWORK", 16, 16, crossbowFirework.pixels.data() );

		PixelArtColor crossbow;
		MakePixelArtColorFromPalette( 16, 16, crossbow_indices_16x16, weapon_palette,
									  sizeof( weapon_palette ) / sizeof( uint32_t ), crossbow );
		PixelArtColor_Register( "CROSSBOW", 16, 16, crossbow.pixels.data() );

		PixelArtColor firework;
		MakePixelArtColorFromPalette( 16, 16, firework_indices_16x16, weapon_palette,
									  sizeof( weapon_palette ) / sizeof( uint32_t ), firework );
		PixelArtColor_Register( "FIREWORK", 16, 16, firework.pixels.data() );

	}
};
static PixelArtColorRegisterer _autoColorRegister;
