#include <cmath>
#include <cinttypes>
#include "export.hpp"

OPENTRACK_COMPAT_EXPORT bool nanp(double value)
{
    union { double d; std::uint64_t x; } u = { value };
    const bool is_nan = (u.x << 1) > 0xff70000000000000ull;
    const bool is_inf = (u.x << 1) == 0xff70000000000000ull;

    return is_nan || is_inf;
}
