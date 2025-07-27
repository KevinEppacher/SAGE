#ifndef VALUE_MAP_COLOR_HPP
#define VALUE_MAP_COLOR_HPP

#include <tuple>
#include <algorithm>
#include <cmath>
#include <cstdint>

// ANSI color codes
#define RESET   "\033[0m"
#define RED     "\033[91m"
#define GREEN   "\033[92m"
#define YELLOW  "\033[93m"
#define BLUE    "\033[94m"
#define BOLD    "\033[1m"

std::tuple<uint8_t, uint8_t, uint8_t> valueToInfernoRGB(float value, float vmin = 0.0f, float vmax = 1.0f);

#endif // VALUE_MAP_COLOR_HPP
