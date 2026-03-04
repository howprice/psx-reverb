#pragma once

#ifdef __GNUC__
// #TODO: Move file helpers to another file and remove this include
#include <stddef.h> // size_t
#endif

// This is an expensive system include which can increase compile time, so only include this file if required.
//#include <type_traits> // underlying_type_t

#define PI 3.14159265359f
#define TWOPI 6.28318530718f

#define ENUM_COUNT(EnumType) ((unsigned int)(EnumType::Max) + 1)

#define HP_UNUSED(X)	(void)X

template<typename T>
inline void Swap(T& a, T& b)
{
	T temp = a;
	a = b;
	b = temp;
}

#if 0 // Disabled due to expensive include. Just use (int)e where needed.
// Convert enum to underlying type
template <typename E>
constexpr auto ToNumber(E e) noexcept
{
	return static_cast<std::underlying_type_t<E>>(e);
}
#endif
