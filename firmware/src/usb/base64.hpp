/*
 * Copyright (C) 2014-2015  Zubax Robotics  <info@zubax.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

/*
 * Base64 encoder and decoder optimized for deeply embedded systems.
 * This implementation is somewhat inspired by this source (released into public domain):
 *      https://en.wikibooks.org/wiki/Algorithm_Implementation/Miscellaneous/Base64#C.2B.2B
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <cstddef>
#include <iterator>

namespace base64
{
/**
 * @param num_bytes     Number of bytes of source data (non-encoded, raw bytes).
 *
 * @returns             Size of base64-encoded data.
 *                      Note that this value will not account for space required for the zero terminator byte.
 */
constexpr static inline std::size_t predictEncodedDataLength(const std::size_t num_bytes)
{
    return ((4UL * num_bytes / 3UL) + 3UL) & ~3UL;
}

/**
 * Encodes binary data to base64 string.
 *
 * @param input         Binary input to be encoded.
 *                      Requirements:
 *                        - Applicability of std::begin()
 *                        - Method size()
 *
 * @param output_buffer Output string will be written here.
 *                      Must be large enough to accommodate the output and the zero terminator byte.
 *
 * @returns             Pointer to the first byte of output.
 */
template <typename Container>
static const char* encode(const Container& input, char* const output_buffer)
{
    static const char* const Alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    auto in = std::begin(input);
    char* out = &output_buffer[0];

    for (std::size_t i = 0; i < input.size() / 3U; i++)
    {
        std::uint32_t x = std::uint32_t(*in++) << 16;
        x +=              std::uint32_t(*in++) << 8;
        x +=              std::uint32_t(*in++);
        *out++ = Alphabet[(x & 0x00FC0000UL) >> 18];
        *out++ = Alphabet[(x & 0x0003F000UL) >> 12];
        *out++ = Alphabet[(x & 0x00000FC0UL) >> 6];
        *out++ = Alphabet[(x & 0x0000003FUL)];
    }

    switch (input.size() % 3)
    {
    case 1:
    {
        const std::uint32_t x = (*in++) << 16; // Convert to big endian
        *out++ = Alphabet[(x & 0x00FC0000UL) >> 18];
        *out++ = Alphabet[(x & 0x0003F000UL) >> 12];
        *out++ = '=';
        *out++ = '=';
        break;
    }
    case 2:
    {
        std::uint32_t x = std::uint32_t(*in++) << 16;
        x +=              std::uint32_t(*in++) << 8;
        *out++ = Alphabet[(x & 0x00FC0000UL) >> 18];
        *out++ = Alphabet[(x & 0x0003F000UL) >> 12];
        *out++ = Alphabet[(x & 0x00000FC0UL) >> 6];
        *out++ = '=';
        break;
    }
    default:
    {
        break;
    }
    }

    *out = '\0';

    return &output_buffer[0];
}

/**
 * @param in    Base64-encoded string.
 *
 * @returns     Number of data bytes when the string is decoded.
 */
static inline std::size_t predictDecodedDataLength(const char* const in)
{
    const std::size_t input_len = std::strlen(in);
    std::size_t padding = 0;

    if ((input_len >= 1) && (in[input_len - 1U] == '='))
    {
        padding++;
    }

    if ((input_len >= 2) && (in[input_len - 2U] == '='))
    {
        padding++;
    }

    return ((input_len / 4U) * 3U) - padding;
}

/**
 * Decodes a base64-encoded messsage.
 * The output container MUST be resized correctly before this function is called, otherwise the function will fail.
 * Refer to @ref predictDecodedDataLength() for size computation.
 *
 * @param out_container The container where the output data will be stored.
 *
 * @param in            Base64-encoded data.
 *
 * @returns             True if decoding was successful, false on error.
 */
template <typename Container>
bool decode(Container& out_container, const char* in)
{
    if (in == nullptr)
    {
        return false;
    }

    const auto input_len = std::strlen(in);
    const char* const end = in + input_len;

    if ((input_len % 4) != 0) // Sanity check
    {
        return false;
    }

    const auto decoded_len = predictDecodedDataLength(in);

    if (decoded_len != out_container.size())
    {
        return false;
    }

    auto out = std::begin(out_container);
    std::uint32_t x = 0;

    while (*in != '\0')
    {
        for (unsigned i = 0; i < 4; i++)
        {
            x <<= 6;
            if      (*in >= 0x41 && *in <= 0x5A) { x |= *in - 0x41; }
            else if (*in >= 0x61 && *in <= 0x7A) { x |= *in - 0x47; }
            else if (*in >= 0x30 && *in <= 0x39) { x |= *in + 0x04; }
            else if (*in == 0x2B)                { x |= 0x3E; }
            else if (*in == 0x2F)                { x |= 0x3F; }
            else if (*in == '=')
            {
                switch (end - in)
                {
                case 1: //One pad character
                {
                    *out++ = static_cast<std::uint8_t>((x >> 16) & 0x000000FFUL);
                    *out++ = static_cast<std::uint8_t>((x >> 8 ) & 0x000000FFUL);
                    return true;
                }
                case 2:
                {
                    *out++ = static_cast<std::uint8_t>((x >> 10) & 0x000000FFUL);
                    return true;
                }
                default:
                {
                    return false;
                }
                }
            }
            else
            {
                return false;
            }

            in++;
        }

        *out++ = static_cast<std::uint8_t>((x >> 16) & 0x000000FFUL);
        *out++ = static_cast<std::uint8_t>((x >> 8 ) & 0x000000FFUL);
        *out++ = static_cast<std::uint8_t>((x >> 0 ) & 0x000000FFUL);
    }

    return true;
}

}
