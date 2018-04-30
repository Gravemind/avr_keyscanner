#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include "main.h"
#include DEBOUNCER
#include "wire-protocol.h"
#include "ringbuf.h"
#include "keyscanner.h"

debounce_t db[COUNT_OUTPUT];

// do_scan gets set any time we should actually do a scan
volatile uint8_t do_scan = 1;

volatile uint8_t g_input_changed = 0;

volatile uint8_t g_last_input = ~0;
uint8_t g_seen_sample_change = 0;

static uint8_t g_saved_input_rows[4] = { ~0, ~0, ~0, ~0 };
static uint8_t g_reading_row = 0;

void keyscanner_init(void) {
    CONFIGURE_OUTPUT_PINS;

    CONFIGURE_INPUT_PINS;

    PORT_OUTPUT |= MASK_OUTPUT;
    PORT_OUTPUT &= ~_BV(g_reading_row);
    asm volatile("nop\n\t");
    PCICR |= _BV(PCIE2);
    PCMSK2 |= MASK_INPUT;

    // Initialize our debouncer datastructure.
    memset(db, 0, sizeof(*db) * COUNT_OUTPUT);

    keyscanner_timer1_init();
}

//__attribute__ ((noinline))
void keyscanner_main(void) {
    if (__builtin_expect(do_scan == 0, EXPECT_TRUE)) {
        return;
    }
    do_scan = 0;

    // -- test, WIP --
    //
    // scans only one output row per keyscanner_main (so every KEYSCAN_INTERVAL)
    // (so debouncer delays are x4)
    //
    // goals:
    //
    // - wait for a full KEYSCAN_INTERVAL for the "lit up" output row to
    //   stabilize before reading the input (the thought is that it might help
    //   the switch to make contact)
    //
    // - the interrupt watch a single output row for an entire KEYSCAN_INTERVAL
    //   every 4 KEYSCAN_INTERVAL (so caught pin changes correspond to a single
    //   key, and not an entire column)
    //
    // currently only debouncer counter and split-counters uses interrupt data
    // (g_seen_sample_change)
    //
    // press and release delay/latency range, with debouncer-counter and
    // - KEYSCAN_INTERVAL=14 -> 5.4 to 7.1 ms.
    // - KEYSCAN_INTERVAL=20 -> 7.8 to 10.25 ms.
    //

    g_seen_sample_change = g_input_changed;

    uint8_t     read_input = PIN_INPUT;
    uint8_t     read_row = g_reading_row;
    uint8_t     next_row = (read_row + 1) % 4;
    uint8_t     next_last_input = g_saved_input_rows[next_row];
    g_reading_row = next_row;
    g_saved_input_rows[read_row] = read_input;

    PCICR &= ~_BV(PCIE2); // disable PCINT2 input interrupt
    {
        g_last_input = next_last_input;
        g_input_changed = 0;

        // switch to next output row
        PORT_OUTPUT |= _BV(read_row);
        PORT_OUTPUT &= ~_BV(next_row);

        // don't do anything here
        // we try to re-enable PCINT2 asap to catch bouncing
        //asm volatile("nop\n\t");
    }
    PCICR |= _BV(PCIE2); // re-enable PCINT2 input interrupt

    uint8_t     debounced_changes = 0;
    debounced_changes |= debounce(KEYSCANNER_CANONICALIZE_PINS(read_input), db + read_row);

    // Most of the time there will be no new key events
    if (__builtin_expect(debounced_changes != 0, EXPECT_FALSE)) {
        RECORD_KEY_STATE;
    }
}

inline void keyscanner_record_state_rotate_ccw (void) {
    // The wire protocol expects data to be four rows of data, rather than 8 cols
    // of data. So we rotate it to match the original outputs
    uint8_t scan_data_as_rows[COUNT_OUTPUT]= {0};
    for(int i=0; i<COUNT_OUTPUT; ++i) {
        for(int j=0; j<COUNT_OUTPUT; ++j) {
            scan_data_as_rows[i] = (  ( (db[j].state & (1 << (7-i) ) ) >> (7-i) ) << j ) | scan_data_as_rows[i];
        }
    }

    DISABLE_INTERRUPTS({
        for(int i =7  ; i>= ( 8-KEY_REPORT_SIZE_BYTES); i--) {
            ringbuf_append(scan_data_as_rows[i]);
        }
    });


}

inline void keyscanner_record_state (void) {

    // Snapshot the keystate to add to the ring buffer
    // Run this with interrupts off to make sure that
    // when we read from the ringbuffer, we always get
    // four bytes representing a single keyboard state.
    DISABLE_INTERRUPTS({
        for(int i =0 ; i< KEY_REPORT_SIZE_BYTES; i++) {
            ringbuf_append(db[i].state);
        }
    });

}

// initialize timer, interrupt and variable
void keyscanner_timer1_init(void) {

    // set up timer with prescaler = 256 and CTC mode
    TCCR1B |= _BV(WGM12)| _BV( CS12);

    // initialize counter
    TCNT1 = 0;

    // initialize compare value
    OCR1A = KEYSCAN_INTERVAL_DEFAULT;

    // enable compare interrupt
    TIMSK1 |= _BV(OCIE1A);

    // enable global interrupts
    sei();
}

// interrupt service routine (ISR) for timer 1 A compare match
ISR(TIMER1_COMPA_vect) {
    do_scan = 1; // Yes! Let's do a scan
}

// interrupt for pin change for pins 16 to 23
ISR(PCINT2_vect) {
    uint8_t input = PIN_INPUT;
    uint8_t input_changed = g_last_input ^ input;
    g_input_changed |= input_changed;
    g_last_input = input;
}
