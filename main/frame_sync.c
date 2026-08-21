#include "frame_sync.h"

#include <limits.h>
#include <string.h>

typedef struct {
    uint8_t header_streak;
    bool in_cohort;
} frame_sync_phase_tracker_t;

struct frame_sync_workspace {
    frame_sync_phase_tracker_t phase_trackers[ADC_SYNC_PHASE_COUNT];
    uint32_t lane_history[ADC_SYNC_SEARCH_LANES];
    uint16_t ring_index;
    uint8_t lane_history_bytes[ADC_SYNC_SEARCH_LANES];
    uint8_t previous_raw_byte;
    bool have_previous_raw_byte;
};

static frame_sync_error_t frame_sync_fail(frame_sync_t *sync,
                                          frame_sync_error_t error)
{
    sync->error = error;
    sync->state = FRAME_SYNC_FATAL;
    return error;
}

size_t frame_sync_workspace_size(void)
{
    return sizeof(frame_sync_workspace_t);
}

bool frame_sync_init(frame_sync_t *sync, void *workspace, size_t workspace_bytes)
{
    if (sync == NULL || workspace == NULL ||
        workspace_bytes < sizeof(frame_sync_workspace_t)) {
        return false;
    }
    memset(sync, 0, sizeof(*sync));
    sync->workspace = workspace;
    sync->workspace_bytes = workspace_bytes;
    frame_sync_reset(sync);
    return true;
}

void frame_sync_reset(frame_sync_t *sync)
{
    if (sync == NULL) {
        return;
    }
    frame_sync_workspace_t *const workspace = sync->workspace;
    const size_t workspace_bytes = sync->workspace_bytes;
    const frame_sync_event_cb_t event_cb = sync->event_cb;
    void *const event_user_ctx = sync->event_user_ctx;
    memset(sync, 0, sizeof(*sync));
    sync->workspace = workspace;
    sync->workspace_bytes = workspace_bytes;
    sync->event_cb = event_cb;
    sync->event_user_ctx = event_user_ctx;
    sync->state = FRAME_SYNC_ACQUIRE;
    sync->error = FRAME_SYNC_ERROR_NONE;
    sync->selected_phase = UINT16_MAX;
    if (workspace != NULL && workspace_bytes >= sizeof(*workspace)) {
        memset(workspace, 0, sizeof(*workspace));
    }
}

void frame_sync_set_event_callback(frame_sync_t *sync,
                                   frame_sync_event_cb_t event_cb,
                                   void *user_ctx)
{
    if (sync == NULL) {
        return;
    }
    sync->event_cb = event_cb;
    sync->event_user_ctx = user_ctx;
}

static void emit_event(frame_sync_t *sync,
                       frame_sync_event_type_t type,
                       uint16_t phase)
{
    if (sync->event_cb == NULL) {
        return;
    }
    const bool fast_lock = type == FRAME_SYNC_EVENT_FAST_LOCKED;
    const frame_sync_event_t event = {
        .type = type,
        .state = sync->state,
        .raw_bit_position = sync->stats.raw_bits,
        .phase_bit_position = phase == UINT16_MAX ? UINT64_MAX : phase,
        .discard_start_bit = sync->discard_start_bit,
        .discard_end_bit = sync->stats.raw_bits,
        .verified_frames = fast_lock ? ADC_SYNC_CONFIRM_HEADERS : 0U,
        .validation_errors = 0U,
        .longest_consecutive_errors = 0U,
        .holdover_good_frames = sync->holdover_good_frames,
        .holdover_bad_frames = sync->holdover_bad_frames,
        .best_candidate_score = 0U,
        .second_candidate_score = 0U,
        .active_candidates = sync->active_candidates,
        .bit_shift = phase == UINT16_MAX
                         ? sync->alignment_shift
                         : (uint8_t)(phase / ADC_FRAME_SIZE_BYTES),
        .recovery = sync->recovery_active,
    };
    sync->event_cb(&event, sync->event_user_ctx);
}

void frame_sync_notify_gap(frame_sync_t *sync, uint64_t dropped_raw_bytes)
{
    if (sync == NULL || dropped_raw_bytes == 0U) {
        return;
    }
    ++sync->stats.input_gap_events;
    sync->stats.discarded_bits += dropped_raw_bytes * 8U;
    (void)frame_sync_fail(sync, FRAME_SYNC_ERROR_OUTPUT);
}

static frame_sync_error_t validate_frame_structure(const uint8_t *frame)
{
    if (frame[0] != 0xFFU || frame[1] != 0xFFU ||
        frame[2] != 0x00U || frame[3] != 0x00U) {
        return FRAME_SYNC_ERROR_HEADER;
    }
    for (size_t channel = 0U; channel < ADC_CHANNEL_COUNT; ++channel) {
        const size_t padding_offset = 6U + channel * 4U;
        if (frame[padding_offset] != 0x00U ||
            frame[padding_offset + 1U] != 0x00U) {
            return FRAME_SYNC_ERROR_PADDING;
        }
    }
    return FRAME_SYNC_ERROR_NONE;
}

static void clear_search_workspace(frame_sync_t *sync)
{
    frame_sync_workspace_t *const workspace = sync->workspace;
    const uint8_t previous_raw_byte = sync->previous_raw_byte;
    const bool have_previous_raw_byte = sync->have_previous_raw_byte;
    memset(workspace, 0, sizeof(*workspace));
    workspace->ring_index = (uint16_t)(
        sync->raw_byte_position % ADC_FRAME_SIZE_BYTES);
    workspace->previous_raw_byte = previous_raw_byte;
    workspace->have_previous_raw_byte = have_previous_raw_byte;
}

static void begin_global_acquire(frame_sync_t *sync)
{
    const bool from_holdover = sync->state == FRAME_SYNC_HOLDOVER;
    clear_search_workspace(sync);
    sync->state = FRAME_SYNC_ACQUIRE;
    sync->cohort_close_raw_byte = 0U;
    sync->active_candidates = 0U;
    sync->selected_phase = UINT16_MAX;
    sync->cohort_open = false;
    sync->frame_byte_count = 0U;
    sync->locked_skip_bytes = 0U;
    sync->recovery_active = sync->ever_locked;
    if (from_holdover) {
        sync->state_start_bit = sync->stats.raw_bits;
        sync->unresolved_warned = false;
        if (sync->recovery_active) {
            ++sync->stats.global_searches;
        }
    }
}

static void start_holdover(frame_sync_t *sync,
                           frame_sync_error_t error,
                           uint64_t frame_end_raw_byte)
{
    if (error == FRAME_SYNC_ERROR_HEADER) {
        ++sync->stats.header_errors;
    } else {
        ++sync->stats.padding_errors;
    }
    ++sync->stats.resync_events;
    sync->stats.last_resync_start_bit =
        (frame_end_raw_byte - ADC_FRAME_SIZE_BYTES) * 8U;
    sync->discard_start_bit = sync->stats.last_resync_start_bit;
    sync->stats.resync_discarded_bytes += ADC_FRAME_SIZE_BYTES;
    sync->stats.discarded_bits += ADC_FRAME_SIZE_BITS;
    sync->state = FRAME_SYNC_HOLDOVER;
    sync->holdover_checked_frames = 0U;
    sync->holdover_good_frames = 0U;
    sync->holdover_good_streak = 0U;
    sync->holdover_bad_frames = 1U;
    sync->had_gaps = true;
    sync->recovery_active = true;
    emit_event(sync, FRAME_SYNC_EVENT_HOLDOVER_STARTED,
               (uint16_t)(sync->alignment_shift * ADC_FRAME_SIZE_BYTES +
                          ((frame_end_raw_byte - ADC_FRAME_SIZE_BYTES) %
                           ADC_FRAME_SIZE_BYTES)));
}

static void process_holdover_frame(frame_sync_t *sync,
                                   frame_sync_error_t error,
                                   uint64_t frame_end_raw_byte)
{
    ++sync->holdover_checked_frames;
    sync->stats.resync_discarded_bytes += ADC_FRAME_SIZE_BYTES;
    sync->stats.discarded_bits += ADC_FRAME_SIZE_BITS;
    if (error == FRAME_SYNC_ERROR_NONE) {
        ++sync->holdover_good_frames;
        ++sync->holdover_good_streak;
    } else {
        sync->holdover_good_streak = 0U;
        ++sync->holdover_bad_frames;
        if (error == FRAME_SYNC_ERROR_HEADER) {
            ++sync->stats.header_errors;
        } else {
            ++sync->stats.padding_errors;
        }
    }
    if (sync->holdover_good_streak >= ADC_SYNC_HOLDOVER_GOOD_FRAMES) {
        sync->state = FRAME_SYNC_LOCKED;
        ++sync->stats.holdover_recoveries;
        sync->stats.last_resync_lock_bit = frame_end_raw_byte * 8U;
        emit_event(sync, FRAME_SYNC_EVENT_HOLDOVER_RECOVERED,
                   (uint16_t)(sync->alignment_shift * ADC_FRAME_SIZE_BYTES +
                              ((frame_end_raw_byte - ADC_FRAME_SIZE_BYTES) %
                               ADC_FRAME_SIZE_BYTES)));
        sync->recovery_active = false;
        return;
    }
    if (sync->holdover_checked_frames >= ADC_SYNC_HOLDOVER_MAX_FRAMES) {
        emit_event(sync, FRAME_SYNC_EVENT_HOLDOVER_FAILED, UINT16_MAX);
        begin_global_acquire(sync);
    }
}

static uint8_t aligned_byte(uint8_t previous, uint8_t current, uint8_t shift)
{
    if (shift == 0U) {
        return current;
    }
    return (uint8_t)(((uint16_t)previous << shift) |
                     ((uint16_t)current >> (8U - shift)));
}

static void add_confirmed_candidate(frame_sync_t *sync,
                                    uint16_t phase,
                                    uint64_t header_end_raw_byte)
{
    frame_sync_phase_tracker_t *const tracker =
        &sync->workspace->phase_trackers[phase];
    if (tracker->in_cohort) {
        return;
    }
    tracker->in_cohort = true;
    ++sync->active_candidates;
    ++sync->stats.sync_candidates;
    sync->stats.confirmed_headers += ADC_SYNC_CONFIRM_HEADERS;
    if (!sync->cohort_open) {
        sync->state = FRAME_SYNC_COHORT;
        sync->cohort_open = true;
        sync->selected_phase = phase;
        sync->cohort_close_raw_byte =
            header_end_raw_byte + ADC_SYNC_COHORT_BYTES;
    }
}

static void complete_fast_lock(frame_sync_t *sync, uint8_t current_raw_byte)
{
    const uint16_t phase = sync->selected_phase;
    sync->last_lock_candidates = sync->active_candidates;
    if (sync->last_lock_candidates > 1U) {
        sync->uncertain_lock_seen = true;
        ++sync->stats.uncertain_locks;
    }
    sync->state = FRAME_SYNC_LOCKED;
    sync->alignment_shift = (uint8_t)(phase / ADC_FRAME_SIZE_BYTES);
    sync->frame_byte_count = 0U;
    /* The current aligned byte is the fourth byte of the cohort header. */
    sync->locked_skip_bytes = ADC_FRAME_SIZE_BYTES - 4U;
    sync->previous_raw_byte = current_raw_byte;
    sync->have_previous_raw_byte = true;
    sync->cohort_open = false;
    sync->ever_locked = true;
    sync->stats.last_resync_lock_bit = sync->stats.raw_bits;
    ++sync->stats.lock_events;
    emit_event(sync, FRAME_SYNC_EVENT_FAST_LOCKED, phase);
}

static void process_search_byte(frame_sync_t *sync, uint8_t raw_byte)
{
    frame_sync_workspace_t *const workspace = sync->workspace;
    ++sync->raw_byte_position;
    sync->stats.raw_bits += 8U;
    sync->stats.discarded_bits += 8U;
    if (sync->recovery_active) {
        ++sync->stats.resync_discarded_bytes;
    }

    const uint16_t ring_index = workspace->ring_index;
    const uint16_t frame_phase_byte =
        ring_index + 1U == ADC_FRAME_SIZE_BYTES
            ? 0U
            : (uint16_t)(ring_index + 1U);
    const uint16_t header_phase_byte = frame_phase_byte >= 4U
        ? (uint16_t)(frame_phase_byte - 4U)
        : (uint16_t)(frame_phase_byte + ADC_FRAME_SIZE_BYTES - 4U);

    for (uint8_t shift = 0U; shift < ADC_SYNC_SEARCH_LANES; ++shift) {
        if (shift != 0U && !workspace->have_previous_raw_byte) {
            continue;
        }
        const uint8_t value = aligned_byte(
            workspace->previous_raw_byte, raw_byte, shift);
        workspace->lane_history[shift] =
            (workspace->lane_history[shift] << 8U) | value;
        if (workspace->lane_history_bytes[shift] < 4U) {
            ++workspace->lane_history_bytes[shift];
        }

        const uint16_t phase = (uint16_t)(
            shift * ADC_FRAME_SIZE_BYTES + header_phase_byte);
        frame_sync_phase_tracker_t *const tracker =
            &workspace->phase_trackers[phase];
        const bool header_matches =
            workspace->lane_history_bytes[shift] >= 4U &&
            workspace->lane_history[shift] == ADC_SYNC_WORD;
        if (header_matches) {
            if (tracker->header_streak < UINT8_MAX) {
                ++tracker->header_streak;
            }
        } else {
            tracker->header_streak = 0U;
        }
        if (tracker->header_streak == ADC_SYNC_CONFIRM_HEADERS) {
            add_confirmed_candidate(sync, phase, sync->raw_byte_position);
        }
    }

    workspace->previous_raw_byte = raw_byte;
    workspace->have_previous_raw_byte = true;
    workspace->ring_index = ring_index + 1U == ADC_FRAME_SIZE_BYTES
        ? 0U
        : (uint16_t)(ring_index + 1U);
    sync->previous_raw_byte = raw_byte;
    sync->have_previous_raw_byte = true;

    if (sync->cohort_open &&
        sync->raw_byte_position >= sync->cohort_close_raw_byte) {
        complete_fast_lock(sync, raw_byte);
    }
    if (sync->state != FRAME_SYNC_LOCKED && !sync->unresolved_warned &&
        sync->stats.raw_bits - sync->state_start_bit >=
            ADC_SYNC_UNRESOLVED_BITS) {
        sync->unresolved_warned = true;
        ++sync->stats.unresolved_warnings;
        emit_event(sync, FRAME_SYNC_EVENT_UNRESOLVED, UINT16_MAX);
    }
}

static frame_sync_error_t process_complete_locked_frame(
    frame_sync_t *sync,
    const uint8_t *frame,
    uint64_t frame_end_raw_byte,
    frame_sync_frame_cb_t frame_cb,
    void *user_ctx)
{
    const frame_sync_error_t error = validate_frame_structure(frame);
    if (sync->state == FRAME_SYNC_HOLDOVER) {
        process_holdover_frame(sync, error, frame_end_raw_byte);
        if (sync->state == FRAME_SYNC_ACQUIRE) {
            return FRAME_SYNC_ERROR_HEADER;
        }
        return FRAME_SYNC_ERROR_NONE;
    }
    if (error != FRAME_SYNC_ERROR_NONE) {
        start_holdover(sync, error, frame_end_raw_byte);
        return error;
    }
    if (frame_cb == NULL || !frame_cb(frame, user_ctx)) {
        ++sync->stats.output_errors;
        ++sync->stats.output_dropped_frames;
        return frame_sync_fail(sync, FRAME_SYNC_ERROR_OUTPUT);
    }
    ++sync->stats.frames_output;
    return FRAME_SYNC_ERROR_NONE;
}

static void account_locked_raw_bytes(frame_sync_t *sync, size_t count)
{
    sync->raw_byte_position += count;
    sync->stats.raw_bits += (uint64_t)count * 8U;
}

static void account_locked_discarded_bytes(frame_sync_t *sync, size_t count)
{
    account_locked_raw_bytes(sync, count);
    sync->stats.discarded_bits += (uint64_t)count * 8U;
    if (sync->recovery_active) {
        sync->stats.resync_discarded_bytes += count;
    }
    if (sync->locked_skip_bytes == 0U) {
        sync->recovery_active = false;
    }
}

static frame_sync_error_t process_locked_aligned_block(
    frame_sync_t *sync,
    const uint8_t *data,
    size_t length,
    frame_sync_frame_cb_t frame_cb,
    void *user_ctx)
{
    if (sync->locked_skip_bytes > 0U) {
        const size_t skip = length < sync->locked_skip_bytes
                                ? length
                                : sync->locked_skip_bytes;
        if (skip > 0U) {
            sync->previous_raw_byte = data[skip - 1U];
            sync->have_previous_raw_byte = true;
        }
        sync->locked_skip_bytes -= (uint16_t)skip;
        data += skip;
        length -= skip;
        account_locked_discarded_bytes(sync, skip);
    }
    if (sync->frame_byte_count > 0U && length > 0U) {
        const size_t needed = ADC_FRAME_SIZE_BYTES - sync->frame_byte_count;
        const size_t copy_size = length < needed ? length : needed;
        memcpy(sync->frame + sync->frame_byte_count, data, copy_size);
        sync->frame_byte_count += (uint16_t)copy_size;
        data += copy_size;
        length -= copy_size;
        account_locked_raw_bytes(sync, copy_size);
        if (sync->frame_byte_count == ADC_FRAME_SIZE_BYTES) {
            sync->frame_byte_count = 0U;
            sync->previous_raw_byte = sync->frame[ADC_FRAME_SIZE_BYTES - 1U];
            sync->have_previous_raw_byte = true;
            const frame_sync_error_t result = process_complete_locked_frame(
                sync, sync->frame, sync->raw_byte_position,
                frame_cb, user_ctx);
            if (result != FRAME_SYNC_ERROR_NONE) {
                return result;
            }
        }
    }
    while (length >= ADC_FRAME_SIZE_BYTES) {
        data += ADC_FRAME_SIZE_BYTES;
        length -= ADC_FRAME_SIZE_BYTES;
        account_locked_raw_bytes(sync, ADC_FRAME_SIZE_BYTES);
        sync->previous_raw_byte = data[-1];
        sync->have_previous_raw_byte = true;
        const frame_sync_error_t result = process_complete_locked_frame(
            sync, data - ADC_FRAME_SIZE_BYTES, sync->raw_byte_position,
            frame_cb, user_ctx);
        if (result != FRAME_SYNC_ERROR_NONE) {
            return result;
        }
    }
    if (length > 0U) {
        memcpy(sync->frame, data, length);
        sync->frame_byte_count = (uint16_t)length;
        sync->previous_raw_byte = data[length - 1U];
        sync->have_previous_raw_byte = true;
        account_locked_raw_bytes(sync, length);
    }
    return FRAME_SYNC_ERROR_NONE;
}

static frame_sync_error_t process_locked_shifted_block(
    frame_sync_t *sync,
    const uint8_t *data,
    size_t length,
    frame_sync_frame_cb_t frame_cb,
    void *user_ctx)
{
    const uint8_t shift = sync->alignment_shift;
    uint8_t previous = sync->previous_raw_byte;
    if (!sync->have_previous_raw_byte && length > 0U) {
        previous = *data++;
        --length;
        sync->have_previous_raw_byte = true;
        account_locked_raw_bytes(sync, 1U);
    }
    if (sync->locked_skip_bytes > 0U) {
        const size_t skip = length < sync->locked_skip_bytes
                                ? length
                                : sync->locked_skip_bytes;
        if (skip > 0U) {
            previous = data[skip - 1U];
            data += skip;
            length -= skip;
            sync->locked_skip_bytes -= (uint16_t)skip;
            account_locked_discarded_bytes(sync, skip);
        }
    }
    while (length > 0U) {
        const size_t frame_space =
            ADC_FRAME_SIZE_BYTES - sync->frame_byte_count;
        const size_t produce = length < frame_space ? length : frame_space;
        uint8_t *const destination = sync->frame + sync->frame_byte_count;
        for (size_t index = 0U; index < produce; ++index) {
            const uint8_t current = data[index];
            destination[index] = aligned_byte(previous, current, shift);
            previous = current;
        }
        data += produce;
        length -= produce;
        sync->frame_byte_count += (uint16_t)produce;
        account_locked_raw_bytes(sync, produce);
        sync->previous_raw_byte = previous;
        sync->have_previous_raw_byte = true;
        if (sync->frame_byte_count == ADC_FRAME_SIZE_BYTES) {
            sync->frame_byte_count = 0U;
            const frame_sync_error_t result = process_complete_locked_frame(
                sync, sync->frame, sync->raw_byte_position,
                frame_cb, user_ctx);
            if (result != FRAME_SYNC_ERROR_NONE) {
                sync->previous_raw_byte = previous;
                return result;
            }
        }
    }
    sync->previous_raw_byte = previous;
    return FRAME_SYNC_ERROR_NONE;
}

static frame_sync_error_t process_locked_block(
    frame_sync_t *sync,
    const uint8_t *data,
    size_t length,
    frame_sync_frame_cb_t frame_cb,
    void *user_ctx)
{
    if (sync->alignment_shift == 0U) {
        return process_locked_aligned_block(
            sync, data, length, frame_cb, user_ctx);
    }
    return process_locked_shifted_block(
        sync, data, length, frame_cb, user_ctx);
}

frame_sync_error_t frame_sync_feed(frame_sync_t *sync,
                                   const uint8_t *data,
                                   size_t length,
                                   frame_sync_frame_cb_t frame_cb,
                                   void *user_ctx)
{
    if (sync == NULL || sync->workspace == NULL ||
        sync->workspace_bytes < sizeof(frame_sync_workspace_t) ||
        (data == NULL && length != 0U)) {
        return FRAME_SYNC_ERROR_OUTPUT;
    }
    if (sync->state == FRAME_SYNC_FATAL) {
        return sync->error;
    }
    size_t index = 0U;
    while (index < length) {
        if (sync->state != FRAME_SYNC_LOCKED &&
            sync->state != FRAME_SYNC_HOLDOVER) {
            process_search_byte(sync, data[index]);
            ++index;
            continue;
        }
        const uint64_t position_before = sync->raw_byte_position;
        const frame_sync_error_t result = process_locked_block(
            sync, data + index, length - index, frame_cb, user_ctx);
        index += (size_t)(sync->raw_byte_position - position_before);
        if (result == FRAME_SYNC_ERROR_NONE) {
            return FRAME_SYNC_ERROR_NONE;
        }
        if (result == FRAME_SYNC_ERROR_OUTPUT) {
            return result;
        }
    }
    return FRAME_SYNC_ERROR_NONE;
}

const frame_sync_stats_t *frame_sync_get_stats(const frame_sync_t *sync)
{
    return sync == NULL ? NULL : &sync->stats;
}

frame_sync_status_t frame_sync_get_status(const frame_sync_t *sync)
{
    frame_sync_status_t status = {0};
    if (sync == NULL) {
        status.state = FRAME_SYNC_FATAL;
        status.error = FRAME_SYNC_ERROR_OUTPUT;
        return status;
    }
    status.state = sync->state;
    status.error = sync->error;
    status.raw_bit_position = sync->stats.raw_bits;
    status.phase_bit_position = sync->selected_phase;
    status.discard_start_bit = sync->discard_start_bit;
    status.discarded_bytes = sync->stats.resync_discarded_bytes;
    status.verified_frames = sync->active_candidates > 0U
                                 ? ADC_SYNC_CONFIRM_HEADERS
                                 : 0U;
    status.active_candidates = sync->active_candidates;
    status.last_lock_candidates = sync->last_lock_candidates;
    status.holdover_checked_frames = sync->holdover_checked_frames;
    status.holdover_good_streak = sync->holdover_good_streak;
    status.bit_shift = sync->alignment_shift;
    status.ever_locked = sync->ever_locked;
    status.had_gaps = sync->had_gaps;
    status.unresolved_warned = sync->unresolved_warned;
    status.uncertain_lock_seen = sync->uncertain_lock_seen;
    return status;
}

const char *frame_sync_state_name(frame_sync_state_t state)
{
    switch (state) {
    case FRAME_SYNC_ACQUIRE:
        return "ACQUIRE";
    case FRAME_SYNC_COHORT:
        return "COHORT";
    case FRAME_SYNC_LOCKED:
        return "LOCKED";
    case FRAME_SYNC_HOLDOVER:
        return "HOLDOVER";
    case FRAME_SYNC_AMBIGUOUS:
        return "AMBIGUOUS_LEGACY";
    case FRAME_SYNC_FATAL:
        return "FATAL";
    default:
        return "UNKNOWN";
    }
}
