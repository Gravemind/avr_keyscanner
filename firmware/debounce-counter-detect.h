#pragma once

#include <stdint.h>
#include "keyscanner.h"

/**
 * This debouncer is a modified `split-counter` where the 'split' has been
 * replaced by a 'bad key detection' inspired by `state-machine`. It is
 * theoretically not as good as `state-machine` but it can run under a
 * KEYSCAN_INTERVAL=14.
 *
 * Press and release delay are both `DELAY` by default (to avoid press
 * overlap). But if a key is detected as "bad", the next release debounce delay
 * will be `BAD_RELEASE_DELAY`.
 *
 * A key is "bad" when it detects a bounce lasting between `BAD_THRESHOLD` and
 * `DELAY` (changes with `DEBOUNCE_BAD_THRESHOLD_2_POW2`, see below)
 *
 * We want to use `BAD_RELEASE_DELAY` as little as possible, so bounces lasting
 * less than `BAD_THRESHOLD` are meant to be ignored because even acceptable
 * keys bounce a little.
 *
 */
#ifndef DEBOUNCE_DELAY
#define DEBOUNCE_DELAY                  17 // @14 -> 7.6ms
#define DEBOUNCE_BAD_RELEASE_DELAY      63 // @14 -> 28.2ms

#define DEBOUNCE_BAD_THRESHOLD_POW2     3 // 2^3 = 8 // @14 -> 3.6ms

/**
 * (DEBOUNCE_BAD_THRESHOLD_2_POW2 can be commented to disable it)
 *
 * DEBOUNCE_BAD_THRESHOLD_2_POW2 makes the "bad" key detection requiring 2
 * bounces for the next release delay to be BAD_RELEASE_DELAY: a first bounce
 * lasting between BAD_THRESHOLD and DELAY, then a second lasting between
 * BAD_THRESHOLD_2 and DELAY. (detection is still reset after a key release is
 * registered)
 *
 * It seem to help reduce "bad" detection false-positives, so BAD_RELEASE_DELAY
 * even less.
 *
 * It costs a bit more performance, but still runs under KEYSCAN_INTERVAL=14
 * when tested with simple boot breath led animation and solid color led.
 *
 */
//#define DEBOUNCE_BAD_THRESHOLD_2_POW2     1 // 2^1 = 2 // @14 -> 0.9ms
#endif

#define _MAX(a, b) ((b) > (a) ? (b) : (a))
#define _NUM_BITS(x) ((x)<1?0:(x)<2?1:(x)<4?2:(x)<8?3:(x)<16?4:(x)<32?5:(x)<64?6:(x)<128?7:(x)<256?8:-1)

// because BAD_RELEASE is the highest
#define NUM_COUNTER_BITS    _NUM_BITS(DEBOUNCE_BAD_RELEASE_DELAY)

typedef struct {
    uint8_t counter_bits[NUM_COUNTER_BITS];
    uint8_t bad;
#ifdef DEBOUNCE_BAD_THRESHOLD_2_POW2
    uint8_t flag;
#endif
    uint8_t last_changes;
    uint8_t state;
} debounce_t;

static inline
uint8_t debounce(uint8_t sample, debounce_t *debouncer) {

    uint8_t state = debouncer->state;
    uint8_t state_changed = sample ^ state;
    uint8_t carry_inc = ~0;

    uint8_t last_not_changed = ~debouncer->last_changes;

    uint8_t waited_delay = ~0;
    uint8_t waited_bad_delay = ~0;
    uint8_t past_threshold1 = 0;
#ifdef DEBOUNCE_BAD_THRESHOLD_2_POW2
    uint8_t past_threshold2 = 0;
#endif

    for(uint8_t i = 0; i < NUM_COUNTER_BITS; i++) {
        uint8_t     c = debouncer->counter_bits[i];

        // see if the last counter value past the threshold
        if (i >= DEBOUNCE_BAD_THRESHOLD_POW2)
            past_threshold1 |= c;
#ifdef DEBOUNCE_BAD_THRESHOLD_2_POW2
        if (i >= DEBOUNCE_BAD_THRESHOLD_2_POW2)
            past_threshold2 |= c;
#endif

        // increment
        c &= last_not_changed; // set counter to 0 (before inc) if state changed last scan
        c ^= carry_inc;
        c &= state_changed; // set counter to 0 if state unchanged
        carry_inc &= ~c;
        debouncer->counter_bits[i] = c;

        if (i < _NUM_BITS(DEBOUNCE_DELAY))
            waited_delay &= (((DEBOUNCE_DELAY + 1) & _BV(i)) ? c : ~c);

        if (i < _NUM_BITS(DEBOUNCE_BAD_RELEASE_DELAY))
            waited_bad_delay &= (((DEBOUNCE_BAD_RELEASE_DELAY + 1) & _BV(i)) ? c : ~c);
    }

    uint8_t bad = debouncer->bad;
#ifdef DEBOUNCE_BAD_THRESHOLD_2_POW2
    uint8_t flag = debouncer->flag;
    bad |= flag & last_not_changed & ~state_changed & past_threshold2;
    flag |= last_not_changed & ~state_changed & past_threshold1;
    uint8_t use_bad = state & flag & bad;
#else
    // flag as bad if the state return to not-changed but the last counter past
    // the threshold
    bad |= last_not_changed & ~state_changed & past_threshold1;
    uint8_t use_bad = state & bad;
#endif

    uint8_t use_def = ~use_bad;
    uint8_t changes = state_changed & (
        (use_def & waited_delay) |
        (use_bad & waited_bad_delay)
    );

    // reset bad flag if key released
    uint8_t reset_bad = state & changes;
    bad &= ~reset_bad;
    debouncer->bad = bad;
#ifdef DEBOUNCE_BAD_THRESHOLD_2_POW2
    flag &= ~reset_bad;
    debouncer->flag = flag;
#endif

    debouncer->last_changes = changes;
    debouncer->state ^= changes;

    //printf("%d %d %d %d\n", sample & 1, debouncer->state & 1, debouncer->flag & 1, bad & 1);

    return changes;
}
