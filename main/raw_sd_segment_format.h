#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RAW_SD_SECTOR_BYTES UINT32_C(512)
#define RAW_SD_SEGMENT_MAGIC "ADCSEG1"
#define RAW_SD_SEGMENT_MAGIC_BYTES 7U
#define RAW_SD_EVENT_MAGIC "ADCEVT2"
#define RAW_SD_EVENT_MAGIC_BYTES 7U
#define RAW_SD_FORMAT_VERSION UINT32_C(2)
#define RAW_SD_FRAME_BYTES UINT32_C(260)

#define RAW_SD_SUPERBLOCK_LBA_A UINT32_C(0)
#define RAW_SD_SUPERBLOCK_LBA_B UINT32_C(1)
#define RAW_SD_SEGMENT_DIRECTORY_START_LBA UINT32_C(2)
#define RAW_SD_SEGMENT_DIRECTORY_CAPACITY UINT32_C(254)
#define RAW_SD_EVENT_AREA_START_LBA UINT32_C(256)
#define RAW_SD_EVENT_AREA_END_LBA UINT32_C(2048)
#define RAW_SD_EVENTS_PER_SECTOR UINT32_C(7)
#define RAW_SD_DATA_START_LBA UINT32_C(2048)
#define RAW_SD_DATA_CAPACITY_SECTORS UINT32_C(2097152)

typedef enum {
    RAW_SD_RUN_RUNNING = 1,
    RAW_SD_RUN_COMPLETE = 2,
    RAW_SD_RUN_FAILED = 3,
} raw_sd_run_state_t;

typedef enum {
    RAW_SD_SEGMENT_OPEN = 1,
    RAW_SD_SEGMENT_CLOSED = 2,
    RAW_SD_SEGMENT_FAILED = 3,
} raw_sd_segment_state_t;

typedef enum {
    RAW_SD_CAPTURE_CLEAN = 1,
    RAW_SD_CAPTURE_CLOSED_WITH_GAPS = 2,
    RAW_SD_CAPTURE_FAILED_UNRESOLVED_SYNC = 3,
    RAW_SD_CAPTURE_FAILED_AMBIGUOUS_SYNC = 4,
    RAW_SD_CAPTURE_FAILED_PIPELINE = 5,
    RAW_SD_CAPTURE_CLOSED_UNCERTAIN_SYNC = 6,
} raw_sd_capture_outcome_t;

typedef struct __attribute__((packed)) {
    uint64_t raw_bit_position;
    uint64_t phase_bit_position;
    uint64_t discard_start_bit;
    uint64_t discard_end_bit;
    uint64_t dma_block_sequence;
    uint32_t verified_frames;
    uint16_t validation_errors;
    uint16_t longest_consecutive_errors;
    uint16_t holdover_good_frames;
    uint16_t holdover_bad_frames;
    uint32_t best_candidate_score;
    uint32_t second_candidate_score;
    uint8_t event_type;
    uint8_t sync_state;
    uint8_t bit_shift;
    uint8_t flags;
} raw_sd_sync_event_t;

typedef struct __attribute__((packed)) {
    uint8_t magic[8];
    uint32_t version;
    uint32_t segment_id;
    uint32_t sector_index;
    uint32_t record_count;
    uint32_t checksum;
    uint8_t reserved[36];
    raw_sd_sync_event_t events[RAW_SD_EVENTS_PER_SECTOR];
} raw_sd_event_sector_t;

typedef struct __attribute__((packed)) {
    uint8_t magic[8];
    uint32_t version;
    uint32_t state;
    uint32_t generation;
    uint32_t run_id;
    uint32_t data_start_lba;
    uint32_t data_capacity_sectors;
    uint32_t segment_count;
    uint32_t closed_segment_count;
    uint64_t next_write_lba;
    uint64_t physical_bytes_written;
    uint64_t valid_bytes_written;
    uint64_t start_time_us;
    uint64_t last_update_time_us;
    uint32_t failure_code;
    uint32_t checksum;
    uint32_t directory_capacity;
    uint32_t event_area_start_lba;
    uint32_t next_event_lba;
    uint32_t event_records_written;
    uint32_t event_records_overflow;
    uint8_t reserved[404];
} raw_sd_superblock_t;

typedef struct __attribute__((packed)) {
    uint8_t magic[8];
    uint32_t version;
    uint32_t state;
    uint32_t segment_id;
    uint32_t run_id;
    uint64_t start_lba;
    uint64_t physical_bytes;
    uint64_t valid_bytes;
    uint64_t frame_count;
    uint64_t start_time_us;
    uint64_t end_time_us;
    uint32_t metadata_generation;
    uint32_t data_checksum;
    uint32_t failure_code;
    uint32_t checksum;
    uint32_t capture_outcome;
    uint32_t final_sync_state;
    uint32_t event_start_lba;
    uint32_t event_sector_count;
    uint32_t event_count;
    uint32_t event_overflow;
    uint64_t resync_events;
    uint64_t resync_discarded_bytes;
    uint64_t last_resync_start_bit;
    uint64_t last_resync_lock_bit;
    uint64_t dma_last_sequence;
    uint32_t dma_sequence_gaps;
    uint32_t diagnostic_flags;
    uint8_t reserved[352];
} raw_sd_segment_t;

_Static_assert(sizeof(raw_sd_sync_event_t) == 64U,
               "raw SD sync event must be exactly 64 bytes");
_Static_assert(sizeof(raw_sd_event_sector_t) == RAW_SD_SECTOR_BYTES,
               "raw SD event sector must be exactly one sector");
_Static_assert(sizeof(raw_sd_superblock_t) == RAW_SD_SECTOR_BYTES,
               "raw SD superblock must be exactly one sector");
_Static_assert(sizeof(raw_sd_segment_t) == RAW_SD_SECTOR_BYTES,
               "raw SD directory entry must be exactly one sector");

uint32_t raw_sd_sector_checksum(const void *sector);
void raw_sd_superblock_finalize(raw_sd_superblock_t *superblock);
void raw_sd_segment_finalize(raw_sd_segment_t *segment);
void raw_sd_event_sector_finalize(raw_sd_event_sector_t *event_sector);
bool raw_sd_superblock_is_valid(const raw_sd_superblock_t *superblock);
bool raw_sd_segment_is_valid(const raw_sd_segment_t *segment);
bool raw_sd_event_sector_is_valid(const raw_sd_event_sector_t *event_sector);

#ifdef __cplusplus
}
#endif
