#include "raw_sd_segment_format.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

uint32_t raw_sd_sector_checksum(const void *sector)
{
    const uint8_t *bytes = sector;
    uint32_t checksum = 0U;
    for (size_t index = 0U; index < RAW_SD_SECTOR_BYTES; ++index) {
        checksum = checksum * 33U + bytes[index];
    }
    return checksum;
}

static void initialize_magic(uint8_t magic[8], const char *value, size_t bytes)
{
    memset(magic, 0, 8U);
    memcpy(magic, value, bytes);
}

void raw_sd_superblock_finalize(raw_sd_superblock_t *superblock)
{
    initialize_magic(superblock->magic, RAW_SD_SEGMENT_MAGIC,
                     RAW_SD_SEGMENT_MAGIC_BYTES);
    superblock->version = RAW_SD_FORMAT_VERSION;
    superblock->checksum = 0U;
    superblock->checksum = raw_sd_sector_checksum(superblock);
}

void raw_sd_segment_finalize(raw_sd_segment_t *segment)
{
    initialize_magic(segment->magic, RAW_SD_SEGMENT_MAGIC,
                     RAW_SD_SEGMENT_MAGIC_BYTES);
    segment->version = RAW_SD_FORMAT_VERSION;
    segment->checksum = 0U;
    segment->checksum = raw_sd_sector_checksum(segment);
}

void raw_sd_event_sector_finalize(raw_sd_event_sector_t *event_sector)
{
    initialize_magic(event_sector->magic, RAW_SD_EVENT_MAGIC,
                     RAW_SD_EVENT_MAGIC_BYTES);
    event_sector->version = RAW_SD_FORMAT_VERSION;
    event_sector->checksum = 0U;
    event_sector->checksum = raw_sd_sector_checksum(event_sector);
}

static bool has_magic(const uint8_t magic[8])
{
    return memcmp(magic, RAW_SD_SEGMENT_MAGIC, RAW_SD_SEGMENT_MAGIC_BYTES) == 0 &&
           magic[RAW_SD_SEGMENT_MAGIC_BYTES] == 0U;
}

bool raw_sd_superblock_is_valid(const raw_sd_superblock_t *superblock)
{
    raw_sd_superblock_t copy = *superblock;
    const uint32_t expected_checksum = copy.checksum;
    copy.checksum = 0U;
    return has_magic(copy.magic) &&
           copy.version == RAW_SD_FORMAT_VERSION &&
           expected_checksum == raw_sd_sector_checksum(&copy);
}

bool raw_sd_segment_is_valid(const raw_sd_segment_t *segment)
{
    raw_sd_segment_t copy = *segment;
    const uint32_t expected_checksum = copy.checksum;
    copy.checksum = 0U;
    return has_magic(copy.magic) &&
           copy.version == RAW_SD_FORMAT_VERSION &&
           expected_checksum == raw_sd_sector_checksum(&copy);
}

bool raw_sd_event_sector_is_valid(const raw_sd_event_sector_t *event_sector)
{
    raw_sd_event_sector_t copy = *event_sector;
    const uint32_t expected_checksum = copy.checksum;
    copy.checksum = 0U;
    return memcmp(copy.magic, RAW_SD_EVENT_MAGIC,
                  RAW_SD_EVENT_MAGIC_BYTES) == 0 &&
           copy.magic[RAW_SD_EVENT_MAGIC_BYTES] == 0U &&
           copy.version == RAW_SD_FORMAT_VERSION &&
           copy.record_count <= RAW_SD_EVENTS_PER_SECTOR &&
           expected_checksum == raw_sd_sector_checksum(&copy);
}
