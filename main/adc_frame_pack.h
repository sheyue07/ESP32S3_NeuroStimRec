#pragma once
#include <stddef.h>
#include <stdint.h>

#define ADC_PACKED_FRAME_BYTES 132U
/* Input is a validated RAW260 frame. Preserve byte order and FFFF0000. */
static inline void adc_frame_pack(uint8_t out[ADC_PACKED_FRAME_BYTES],
                                  const uint8_t in[260])
{
    for (size_t i = 0; i < 4; ++i) out[i] = in[i];
    for (size_t ch = 0; ch < 64; ++ch) {
        out[4 + ch * 2] = in[4 + ch * 4];
        out[5 + ch * 2] = in[5 + ch * 4];
    }
}
