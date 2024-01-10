#pragma once
#include <arm.hpp>


namespace bit {

//inspired by and some levied from notorious beeg
template<u8 start, u8 end> //inclusive
constexpr auto get_range(const u32 instruction) {
    constexpr auto mask = ((1 << start + 1) - 1) ^ ((1 << end) - 1);
    return (instruction & mask) >> end;
}

static_assert(get_range<3,0>(0xF) == 0xF);

template<u8 index>
constexpr auto get_bit(u32 instruction) {
    return (instruction & (1 << index)) >> index;
}

static_assert(get_bit<3>(0x7) == 0B0);

constexpr auto decode_hash(u32 instruction) {
    return (get_range<27, 20>(instruction) << 4) | get_range<7, 4>(instruction);
}

template<auto index>
constexpr auto is_set(u32 v){
    return get_bit<index>(v) == 0b1;
}

template<auto b>
constexpr auto decoded_is_set(u32 v)
{
    // 27-20 and 7-4
    static_assert((b <= 27 && b >= 20) || (b <= 7 && b >= 4), "invalid");
    if constexpr(b <= 27 && b >= 20)
    {
        constexpr auto new_bit = (b - 20) + 4;
        return is_set<new_bit>(v);
    }
    else
    {
        constexpr auto new_bit = b - 4;
        return is_set<new_bit>(v);
    }
}

}; //Namespace bit