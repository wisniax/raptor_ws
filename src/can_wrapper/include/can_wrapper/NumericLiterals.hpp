#ifndef NUMERICLITERALS_H
#define NUMERICLITERALS_H

#include <stdint-gcc.h>

constexpr int8_t operator""i8(unsigned long long int x)
{
    return (int8_t)x;
}

constexpr int8_t operator""i16(unsigned long long int x)
{
    return (int16_t)x;
}

constexpr int8_t operator""i32(unsigned long long int x)
{
    return (int32_t)x;
}

constexpr uint8_t operator""ui8(unsigned long long int x)
{
    return (uint8_t)x;
}

constexpr uint16_t operator""ui16(unsigned long long int x)
{
    return (uint16_t)x;
}

constexpr uint32_t operator""ui32(unsigned long long int x)
{
    return (uint32_t)x;
}

#endif