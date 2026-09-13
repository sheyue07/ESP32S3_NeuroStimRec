#include <assert.h>
#include <string.h>
#include "adc_frame_pack.h"
#include "raw_sd_segment_format.h"

int main(void)
{
    uint8_t raw[260] = {255,255,0,0}, packed[132];
    for (unsigned n = 0; n < 1024; ++n) {
        for (unsigned ch = 0; ch < 64; ++ch) {
            unsigned value = n * 64 + ch;
            raw[4 + ch * 4] = value >> 8;
            raw[5 + ch * 4] = value;
        }
        adc_frame_pack(packed, raw);
        assert(memcmp(packed, raw, 4) == 0);
        for (unsigned ch = 0; ch < 64; ++ch)
            assert(memcmp(packed + 4 + ch * 2, raw + 4 + ch * 4, 2) == 0);
    }
    raw_sd_segment_t segment = {0};
    raw_sd_segment_finalize(&segment);
    assert(segment.version == 3 && raw_sd_segment_is_valid(&segment));
    assert(raw_sd_segment_frame_bytes(&segment) == 132);
    segment.version = 2;
    raw_sd_segment_finalize(&segment);
    assert(segment.version == 2 && raw_sd_segment_is_valid(&segment));
    assert(raw_sd_segment_frame_bytes(&segment) == 260);
    segment.version = 99;
    raw_sd_segment_finalize(&segment);
    assert(!raw_sd_segment_is_valid(&segment));
    return 0;
}
