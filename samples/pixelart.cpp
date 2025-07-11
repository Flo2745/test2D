#include "pixelart.h"

static const uint32_t creeperColors[8][8] = {
	{ 0x00b600, 0x007200, 0x009800, 0x909090, 0xbbbbbb, 0x69ff69, 0x3b913b, 0x007200 },
	{ 0x007200, 0x00b600, 0x009800, 0x3b913b, 0x006b00, 0x4fc14f, 0x307430, 0x909090 },
	{ 0x009800, 0x242424, 0x242424, 0x5fe85f, 0x4fc14f, 0x242424, 0x242424, 0xbbbbbb },
	{ 0x009800, 0x242424, 0x101010, 0x3b913b, 0x204d20, 0x101010, 0x242424, 0x3b913b },
	{ 0x3b913b, 0x00b600, 0x5fe85f, 0x242424, 0x242424, 0xbbbbbb, 0x006b00, 0x00b600 },
	{ 0x5fe85f, 0x204d20, 0x242424, 0x101010, 0x101010, 0x242424, 0x00b600, 0xbbbbbb },
	{ 0xbbbbbb, 0x3b913b, 0x101010, 0x101010, 0x101010, 0x101010, 0x3b913b, 0x006b00 },
	{ 0x5fe85f, 0x00b600, 0x242424, 0x5fe85f, 0x00b600, 0x242424, 0x006b00, 0x00b600 } };

static const uint32_t wolfColors[8][8] = { { 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff },
										   { 0xffffff, 0xe4dedf, 0xe4dedf, 0xffffff, 0xffffff, 0xe4dedf, 0xe4dedf, 0xffffff },
										   { 0xffffff, 0xc8c2c3, 0xc8c2c3, 0xffffff, 0xffffff, 0xc8c2c3, 0xc8c2c3, 0xffffff },
										   { 0xffffff, 0xc8c2c3, 0xb7b3b4, 0xc4c0c0, 0xc4c0c0, 0xb7b3b4, 0xc8c2c3, 0xffffff },
										   { 0xffffff, 0xffffff, 0x121416, 0xa38775, 0xa38775, 0x121416, 0xffffff, 0xffffff },
										   { 0xffffff, 0xa78f7e, 0x9a8c88, 0x9a8c88, 0x9a8c88, 0x8c6f52, 0xa78f7e, 0xffffff },
										   { 0xffffff, 0x8c6f52, 0x9a8c88, 0x9a8c88, 0x9a8c88, 0x9a8c88, 0x8c6f52, 0xffffff },
										   { 0xffffff, 0x9a8c88, 0xe4d8d9, 0xe4d8d9, 0xe4d8d9, 0xe4d8d9, 0x9a8c88, 0xffffff } };

static const uint32_t zombieColors[8][8] = { { 0x4A4A4A, 0x4A4A4A, 0x6B8E23, 0x6B8E23, 0x6B8E23, 0x6B8E23, 0x4A4A4A, 0x4A4A4A },
											 { 0x4A4A4A, 0x6B8E23, 0x6B8E23, 0x7FFF00, 0x7FFF00, 0x6B8E23, 0x6B8E23, 0x4A4A4A },
											 { 0x6B8E23, 0x6B8E23, 0x7FFF00, 0x7FFF00, 0x7FFF00, 0x7FFF00, 0x6B8E23, 0x6B8E23 },
											 { 0x6B8E23, 0x7FFF00, 0x7FFF00, 0x556B2F, 0x556B2F, 0x7FFF00, 0x7FFF00, 0x6B8E23 },
											 { 0x6B8E23, 0x7FFF00, 0x7FFF00, 0x556B2F, 0x556B2F, 0x7FFF00, 0x7FFF00, 0x6B8E23 },
											 { 0x6B8E23, 0x6B8E23, 0x7FFF00, 0x7FFF00, 0x7FFF00, 0x7FFF00, 0x6B8E23, 0x6B8E23 },
											 { 0x4A4A4A, 0x6B8E23, 0x6B8E23, 0x7FFF00, 0x7FFF00, 0x6B8E23, 0x6B8E23, 0x4A4A4A },
											 { 0x4A4A4A, 0x4A4A4A, 0x6B8E23, 0x6B8E23, 0x6B8E23, 0x6B8E23, 0x4A4A4A, 0x4A4A4A } };

static const uint32_t skeletonColors[8][8] = {
	{ 0x7a7a7a, 0x636363, 0x7a7a7a, 0x636363, 0x636363, 0x7a7a7a, 0x7a7a7a, 0x7a7a7a },
	{ 0x636363, 0x636363, 0x636363, 0x818181, 0x909090, 0x909090, 0x7a7a7a, 0x7a7a7a },
	{ 0x636363, 0xa1a1a1, 0xb5b5b5, 0xa1a1a1, 0xb5b5b5, 0xa09191, 0xb5b5b5, 0x949494 },
	{ 0xa09191, 0xb5b5b5, 0xa09191, 0xb5b5b5, 0x949494, 0xa1a1a1, 0x949494, 0x949494 },
	{ 0xaaaaaa, 0x343434, 0x343434, 0x949494, 0x949494, 0x343434, 0x343434, 0xaaaaaa },
	{ 0x949494, 0xaaaaaa, 0xaaaaaa, 0x636363, 0x636363, 0xaaaaaa, 0xaaaaaa, 0x636363 },
	{ 0xbbbbbb, 0x343434, 0x343434, 0x343434, 0x343434, 0x343434, 0x343434, 0x636363 },
	{ 0x636363, 0x636363, 0x7a7a7a, 0x636363, 0x7a7a7a, 0x636363, 0x636363, 0x636363 } };

static const uint32_t steveColors[8][8] = { { 0x2b1e0d, 0x2b1e0d, 0x2f1f0f, 0x2b1e0d, 0x2b1e0d, 0x2b1e0d, 0x2b1e0d, 0x2b1e0d },
											{ 0x2c1e0e, 0x2c1e0e, 0x2c1e0e, 0x332411, 0x3f2a15, 0x3f2a15, 0x2c1e0e, 0x2c1e0e },
											{ 0x2b1e0d, 0xb6896c, 0xbd8e72, 0xc69680, 0xbd8b72, 0xbd8e74, 0xac765a, 0x2b1e0d },
											{ 0xaa7d66, 0xb4846d, 0xaa7d66, 0xad806d, 0x9c725c, 0xbb8972, 0x9c694c, 0x9c694c },
											{ 0xb4846d, 0xffffff, 0x523d89, 0xb57b67, 0xbe886c, 0x523d89, 0xffffff, 0xb4846d },
											{ 0x9c6346, 0xb37b62, 0xbe886c, 0x6a4030, 0x6a4030, 0xbe886c, 0xa26a47, 0x815339 },
											{ 0x9c6346, 0x9c6346, 0x45220e, 0x8a4c3d, 0x8a4c3d, 0x45220e, 0x8f5e3e, 0x815339 },
											{ 0x6f452c, 0x6f452c, 0x45220e, 0x45220e, 0x45220e, 0x45220e, 0x83553b, 0x815339 } };

static const uint32_t villagerColors[8][8] = {
	{ 0xbd8b72, 0xb57b67, 0xbd8b72, 0xb57b67, 0xb57b67, 0xb57b67, 0xbd8b72, 0xb57b67 },
	{ 0xbd8b72, 0xb57b67, 0xb57b67, 0xb57b67, 0xb57b67, 0xb57b67, 0xbd8b72, 0xb57b67 },
	{ 0xb57b67, 0xb57b67, 0xb57b67, 0xb57b67, 0xb57b67, 0xb57b67, 0xb57b67, 0xb57b67 },
	{ 0xb57b67, 0x332411, 0x332411, 0x332411, 0x332411, 0x332411, 0x332411, 0xb57b67 },
	{ 0xbd8b72, 0xffffff, 0x009611, 0xb57b67, 0xb57b67, 0x009611, 0xffffff, 0xbd8b72 },
	{ 0x905e43, 0xb57b67, 0xbd8b72, 0x905e43, 0x905e43, 0xbd8b72, 0xb57b67, 0x905e43 },
	{ 0x905e43, 0xb57b67, 0x774235, 0x905e43, 0x905e43, 0x774235, 0xb57b67, 0x905e43 },
	{ 0x905e43, 0xb57b67, 0xb57b67, 0x905e43, 0x905e43, 0xb57b67, 0xb57b67, 0x905e43 } };

static const uint32_t pigColors[8][8] = { { 0xe59494, 0xc37e84, 0xec9899, 0xec9899, 0xeda1a1, 0xeda1a1, 0xee9d9f, 0xefa7a6 },
										  { 0xe59494, 0xc37e84, 0xef9f9f, 0xef9f9f, 0xec9a9a, 0xf0a7a5, 0xf1b3ad, 0xf1b3ad },
										  { 0x020001, 0xffffff, 0xee9e9e, 0xee9e9e, 0xf0a7a5, 0xeda7a4, 0xffffff, 0x020001 },
										  { 0xeda7a4, 0xeea29f, 0xf1c3c5, 0xf1c3c5, 0xf1c3c5, 0xf1c3c5, 0xf1afad, 0xf1afad },
										  { 0xeea29f, 0xeea29f, 0x9c5150, 0xffb5b5, 0xffb5b5, 0x9c5150, 0xf1afad, 0xf1afad },
										  { 0xeea3a2, 0xeea3a2, 0xeea3a2, 0xeea3a2, 0xeeb0ac, 0xeea3a2, 0xeea3a2, 0xeea3a2 } };

static const uint32_t cowColors[8][8] = { { 0x391800, 0x312100, 0x391800, 0xe7e7e7, 0xe7e7e7, 0xa5a5a5, 0xa5a5a5, 0x391800 },
										  { 0x312100, 0x312100, 0x312100, 0xe7e7e7, 0xe7e7e7, 0xcecece, 0x312100, 0x312100 },
										  { 0xe7e7e7, 0xe7e7e7, 0x312100, 0xe7e7e7, 0xe7e7e7, 0x312100, 0xdedede, 0xdedede },
										  { 0x000008, 0xffffff, 0x391800, 0xbdbdbd, 0x391800, 0x391800, 0xffffff, 0x000008 },
										  { 0x391800, 0x391800, 0x391800, 0x391800, 0x391800, 0x391800, 0x391800, 0x391800 },
										  { 0x391800, 0x391800, 0xefefef, 0xefefef, 0xefefef, 0xefefef, 0x391800, 0x391800 },
										  { 0x391800, 0xefefef, 0x000008, 0xa5a5a5, 0xa5a5a5, 0x000008, 0xefefef, 0x391800 },
										  { 0x391800, 0xefefef, 0xa5a5a5, 0x949494, 0x949494, 0xa5a5a5, 0xefefef, 0x391800 } };

static const uint32_t spiderColors[8][8] = { { 0x2B2B2B, 0xa60202, 0x313131, 0x313131, 0x272727, 0x202020, 0x620000, 0x202020 },
											 { 0x2A2A2A, 0x333333, 0x620000, 0x313131, 0x272727, 0xa60202, 0x202020, 0x202020 },
											 { 0xa60202, 0x2E2E2E, 0x353535, 0x353535, 0x272727, 0x262626, 0x202020, 0xff0e0e },
											 { 0x620000, 0x2B2B2B, 0xff0e0e, 0xa60202, 0xff0e0e, 0xa60202, 0x1C1C1C, 0xa60202 },
											 { 0x272727, 0x292929, 0xa60202, 0x620000, 0xa60202, 0x620000, 0x1A1A1A, 0x212121 },
											 { 0x272727, 0x282828, 0x2D2D2D, 0x1F1F1F, 0x111111, 0x272727, 0x1C1C1C, 0x262626 },
											 { 0x272727, 0x313131, 0x141414, 0x1F1F1F, 0x111111, 0x141414, 0x313131, 0x2A2A2A },
											 { 0x272727, 0x2D2D2D, 0x141414, 0x1F1F1F, 0x191919, 0x141414, 0x3E3E3E, 0x242424 } };

static const uint32_t sheepColors[8][8] = { { 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff },
											{ 0xffffff, 0xd6d6d6, 0xd6d6d6, 0xc7c7c7, 0xc7c7c7, 0xc7c7c7, 0xc7c7c7, 0xffffff },
											{ 0xffffff, 0xb6977f, 0xb6977f, 0xb6977f, 0xb6977f, 0xb6977f, 0xb6977f, 0xffffff },
											{ 0xffffff, 0x080808, 0xffffff, 0xb08f76, 0xb08f76, 0xffffff, 0x080808, 0xffffff },
											{ 0xffffff, 0x8e725e, 0xb6977f, 0xb6977f, 0xb6977f, 0xb6977f, 0x8e725e, 0xffffff },
											{ 0xffffff, 0xd6d6d6, 0x8e725e, 0xffb5b5, 0xffb5b5, 0x8e725e, 0xd6d6d6, 0xffffff },
											{ 0xffffff, 0xd6d6d6, 0x8e725e, 0xda9d9d, 0xda9d9d, 0x8e725e, 0xd6d6d6, 0xffffff },
											{ 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff } };

static const uint32_t cavespiderColors[8][8] = {
	{ 0x073945, 0xa60202, 0x0d3e49, 0x0d3e49, 0x053440, 0x0b2830, 0x620000, 0x17232b },
	{ 0x073842, 0x0c424e, 0x620000, 0x0d3e49, 0x053440, 0xa60202, 0x17232b, 0x17232b },
	{ 0xa60202, 0x173740, 0x14414c, 0x14414c, 0x053440, 0x0d2f39, 0x17232b, 0xff0e0e },
	{ 0x620000, 0x17323c, 0xff0e0e, 0xa60202, 0xff0e0e, 0xa60202, 0x1e1a20, 0xa60202 },
	{ 0x06323c, 0x1b2d36, 0xa60202, 0x620000, 0xa60202, 0x620000, 0x101d26, 0x0d2830 },
	{ 0x06323c, 0x0c323c, 0x123742, 0x032932, 0x04161c, 0x09323d, 0x06242d, 0x07313b },
	{ 0x06323c, 0x0d3e49, 0x05191e, 0x032932, 0x04161c, 0x05191e, 0x0d3e49, 0x073842 },
	{ 0x06323c, 0x0c3944, 0x05191e, 0x032932, 0x00222c, 0x05191e, 0x134e5b, 0x03313b } };

static const uint32_t endermanColors[8][8] = {
	{ 0x101010, 0x242424, 0x101010, 0x101010, 0x101010, 0x101010, 0x242424, 0x101010 },
	{ 0x101010, 0x242424, 0x242424, 0x101010, 0x101010, 0x242424, 0x242424, 0x101010 },
	{ 0x242424, 0x101010, 0x242424, 0x242424, 0x242424, 0x242424, 0x101010, 0x242424 },
	{ 0x101010, 0x101010, 0x242424, 0x242424, 0x242424, 0x242424, 0x101010, 0x101010 },
	{ 0xe079fa, 0xcc00fa, 0xe079fa, 0x242424, 0x242424, 0xe079fa, 0xcc00fa, 0xe079fa },
	{ 0x242424, 0x101010, 0x101010, 0x242424, 0x242424, 0x101010, 0x101010, 0x242424 },
	{ 0x101010, 0x242424, 0x242424, 0x101010, 0x101010, 0x242424, 0x242424, 0x101010 },
	{ 0x242424, 0x101010, 0x101010, 0x101010, 0x101010, 0x101010, 0x101010, 0x242424 } };

const uint32_t ( &PixelArt::GetColors( PixelArtType type ) )[PixelArt::Size][PixelArt::Size]
{
	switch ( type )
	{
		case PixelArtType::Creeper:
			return creeperColors;
		case PixelArtType::Wolf:
			return wolfColors;
		case PixelArtType::Zombie:
			return zombieColors;
		case PixelArtType::Skeleton:
			return skeletonColors;
		case PixelArtType::Steve:
			return steveColors;
		case PixelArtType::Villager:
			return villagerColors;
		case PixelArtType::Pig:
			return pigColors;
		case PixelArtType::Cow:
			return cowColors;
		case PixelArtType::Spider:
			return spiderColors;
		case PixelArtType::Sheep:
			return sheepColors;
		case PixelArtType::CaveSpider:
			return cavespiderColors;
		case PixelArtType::Enderman:
			return endermanColors;
		default:
			return creeperColors;
	}
}
