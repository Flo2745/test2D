#pragma once

#include <cstdint>

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
