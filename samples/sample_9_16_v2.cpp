#include "audiomanager.h"
#include "benchmarks.h"
#include "draw.h"
#include "human.h"
#include "particules.h"
#include "random.h"
#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/id.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <SFML/Audio.hpp>
#include <algorithm>
#include <array>
#include <filesystem>
#include <imgui.h>
#include <queue>
#include <random>
#include <set>
#include <vector>

constexpr float b2_pi = 3.14159265359f;

inline bool operator<( b2BodyId a, b2BodyId b )
{
	uint64_t ua = b2StoreBodyId( a );
	uint64_t ub = b2StoreBodyId( b );
	return ua < ub;
}
