#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_FRAME_SIZE_BYTES                         260U
#define ADC_FRAME_SIZE_BITS                          2080U
#define ADC_CHANNEL_COUNT                            64U
#define ADC_SYNC_WORD                                UINT32_C(0xFFFF0000)
#define ADC_SYNC_SEARCH_LANES                        8U
#define ADC_SYNC_PHASE_COUNT                         ADC_FRAME_SIZE_BITS
#define ADC_SYNC_CONFIRM_HEADERS                     8U
#define ADC_SYNC_COHORT_BYTES                        ADC_FRAME_SIZE_BYTES
#define ADC_SYNC_HOLDOVER_MAX_FRAMES                 8U
#define ADC_SYNC_HOLDOVER_GOOD_FRAMES                4U
#define ADC_SYNC_UNRESOLVED_BITS                     UINT64_C(300000000)

typedef enum {
    FRAME_SYNC_ACQUIRE = 0,
    FRAME_SYNC_COHORT = 1,
    FRAME_SYNC_LOCKED = 2,
    FRAME_SYNC_HOLDOVER = 3,
    FRAME_SYNC_AMBIGUOUS = 4, /* Legacy metadata value; never entered. */
    FRAME_SYNC_FATAL = 5,
} frame_sync_state_t;

typedef enum {
    FRAME_SYNC_ERROR_NONE,
    FRAME_SYNC_ERROR_HEADER,
    FRAME_SYNC_ERROR_PADDING,
    FRAME_SYNC_ERROR_OUTPUT,
} frame_sync_error_t;

/* Values 0..7 retain their legacy on-disk meanings. */
typedef enum {
    FRAME_SYNC_EVENT_VERIFY_STARTED = 0,
    FRAME_SYNC_EVENT_CANDIDATE_REJECTED = 1,
    FRAME_SYNC_EVENT_LOCKED = 2,
    FRAME_SYNC_EVENT_HOLDOVER_STARTED = 3,
    FRAME_SYNC_EVENT_HOLDOVER_RECOVERED = 4,
    FRAME_SYNC_EVENT_HOLDOVER_FAILED = 5,
    FRAME_SYNC_EVENT_AMBIGUOUS = 6,
    FRAME_SYNC_EVENT_UNRESOLVED = 7,
    FRAME_SYNC_EVENT_FAST_LOCKED = 8,
} frame_sync_event_type_t;

typedef struct {
    frame_sync_event_type_t type;
    frame_sync_state_t state;
    uint64_t raw_bit_position;
    uint64_t phase_bit_position;
    uint64_t discard_start_bit;
    uint64_t discard_end_bit;
    uint32_t verified_frames;
    uint16_t validation_errors;
    uint16_t longest_consecutive_errors;
    uint16_t holdover_good_frames;
    uint16_t holdover_bad_frames;
    uint32_t best_candidate_score;
    uint32_t second_candidate_score;
    uint16_t active_candidates;
    uint8_t bit_shift;
    bool recovery;
} frame_sync_event_t;

typedef void (*frame_sync_event_cb_t)(const frame_sync_event_t *event,
                                      void *user_ctx);

typedef struct {
    uint64_t raw_bits;
    uint64_t discarded_bits;
    uint64_t sync_candidates;
    uint64_t confirmed_headers;
    uint64_t lock_events;
    uint64_t input_gap_events;
    uint64_t frames_output;
    uint64_t header_errors;
    uint64_t padding_errors;
    uint64_t output_errors;
    uint64_t output_dropped_frames;
    uint64_t resync_events;
    uint64_t resync_discarded_bytes;
    uint64_t last_resync_start_bit;
    uint64_t last_resync_lock_bit;
    uint64_t holdover_recoveries;
    uint64_t global_searches;
    uint64_t ambiguous_events;      /* Legacy counter; new code leaves it zero. */
    uint64_t candidate_rejections;  /* Legacy counter; new code leaves it zero. */
    uint64_t unresolved_warnings;
    uint64_t uncertain_locks;
} frame_sync_stats_t;

typedef struct {
    frame_sync_state_t state;
    frame_sync_error_t error;
    uint64_t raw_bit_position;
    uint64_t phase_bit_position;
    uint64_t discard_start_bit;
    uint64_t discarded_bytes;
    uint32_t verified_frames;             /* Legacy diagnostic field. */
    uint16_t validation_errors;           /* Legacy diagnostic field. */
    uint16_t longest_consecutive_errors;  /* Legacy diagnostic field. */
    uint16_t active_candidates;
    uint16_t last_lock_candidates;
    uint16_t holdover_checked_frames;
    uint16_t holdover_good_streak;
    uint32_t best_candidate_score;        /* Legacy diagnostic field. */
    uint32_t second_candidate_score;      /* Legacy diagnostic field. */
    uint8_t bit_shift;
    bool ever_locked;
    bool had_gaps;
    bool unresolved_warned;
    bool uncertain_lock_seen;
} frame_sync_status_t;

typedef bool (*frame_sync_frame_cb_t)(
    const uint8_t frame[ADC_FRAME_SIZE_BYTES], void *user_ctx);

typedef struct frame_sync_workspace frame_sync_workspace_t;

typedef struct {
    frame_sync_state_t state;
    frame_sync_error_t error;
    frame_sync_stats_t stats;
    frame_sync_workspace_t *workspace;
    size_t workspace_bytes;
    frame_sync_event_cb_t event_cb;
    void *event_user_ctx;

    uint64_t raw_byte_position;
    uint64_t state_start_bit;
    uint64_t discard_start_bit;
    uint64_t cohort_close_raw_byte;
    uint16_t active_candidates;
    uint16_t selected_phase;
    uint16_t last_lock_candidates;
    uint16_t holdover_checked_frames;
    uint16_t holdover_good_frames;
    uint16_t holdover_good_streak;
    uint16_t holdover_bad_frames;
    uint8_t alignment_shift;
    uint8_t previous_raw_byte;
    bool have_previous_raw_byte;
    bool cohort_open;
    bool ever_locked;
    bool had_gaps;
    bool recovery_active;
    bool unresolved_warned;
    bool uncertain_lock_seen;

    uint8_t frame[ADC_FRAME_SIZE_BYTES];
    uint16_t frame_byte_count;
    uint16_t locked_skip_bytes;
} frame_sync_t;

size_t frame_sync_workspace_size(void);
bool frame_sync_init(frame_sync_t *sync, void *workspace, size_t workspace_bytes);
void frame_sync_reset(frame_sync_t *sync);
void frame_sync_set_event_callback(frame_sync_t *sync,
                                   frame_sync_event_cb_t event_cb,
                                   void *user_ctx);

/* A missing raw block is a fatal pipeline error in every synchronization state. */
void frame_sync_notify_gap(frame_sync_t *sync, uint64_t dropped_raw_bytes);

frame_sync_error_t frame_sync_feed(frame_sync_t *sync,
                                   const uint8_t *data,
                                   size_t length,
                                   frame_sync_frame_cb_t frame_cb,
                                   void *user_ctx);

const frame_sync_stats_t *frame_sync_get_stats(const frame_sync_t *sync);
frame_sync_status_t frame_sync_get_status(const frame_sync_t *sync);
const char *frame_sync_state_name(frame_sync_state_t state);

#ifdef __cplusplus
}
#endif
