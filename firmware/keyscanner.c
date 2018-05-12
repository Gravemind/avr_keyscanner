#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include "main.h"
#include DEBOUNCER
#include "wire-protocol.h"
#include "ringbuf.h"
#include "keyscanner.h"

#include "led-spiout.h"

debounce_t db[COUNT_OUTPUT];

// do_scan gets set any time we should actually do a scan
volatile uint8_t do_scan = 1;

uint8_t g_sample_unstable = 0;

void keyscanner_set_interval(uint8_t interval) {
    OCR1A = interval;
}
uint8_t keyscanner_get_interval(void) {
    return OCR1A;
}

void keyscanner_init(void) {

    CONFIGURE_OUTPUT_PINS;

    CONFIGURE_INPUT_PINS;

    // Initialize our debouncer datastructure.
    memset(db, 0, sizeof(*db) * COUNT_OUTPUT);

    keyscanner_timer1_init();
}

//__attribute__((noinline))
void keyscanner_main(void) {
    if (__builtin_expect(do_scan == 0, EXPECT_TRUE)) {
        return;
    }
    do_scan = 0;

    uint8_t pins_data[COUNT_OUTPUT];
    uint8_t pins_unstable[COUNT_OUTPUT] = {0};

    // Sample matrix

    // How many full scans in a row to catch instability
    const uint8_t   stability_test_output_loops = 3;
    // How many raw input sampling in a row to catch instability
    const uint8_t   stability_test_input_loops = 5;
    for (uint8_t stability_output_loop = 0; stability_output_loop < stability_test_output_loops; ++stability_output_loop)
    {
        // For each enabled row...
        for (uint8_t output_pin = 0; output_pin < COUNT_OUTPUT; ++output_pin) {

            REINIT_INPUT_PINS;

            // Toggle the output we want to check
            ACTIVATE_OUTPUT_PIN(output_pin);
            asm volatile("nop\n\t");

            // Read pin data
            if (stability_output_loop == 0)
                pins_data[output_pin] = PIN_INPUT;

            for (uint8_t stability_input_loop = 0; stability_input_loop < stability_test_input_loops; ++stability_input_loop)
                pins_unstable[output_pin] |= PIN_INPUT ^ pins_data[output_pin];

            // Toggle the output we want to read back off
            DEACTIVATE_OUTPUT_PIN(output_pin);
            asm volatile("nop\n\t");

            CLEANUP_INPUT_PINS;
        }
    }

    // Run debouncer

    uint8_t debounced_changes = 0;
    for (uint8_t output_pin = 0; output_pin < COUNT_OUTPUT; ++output_pin) {
        uint8_t     sample = KEYSCANNER_CANONICALIZE_PINS(pins_data[output_pin]);
        g_sample_unstable = pins_unstable[output_pin];
        // Debounce key state
        debounced_changes |= debounce(sample, db + output_pin);

#if 0
        // Debug stuff by fade-in/out key's LED
        // ! overridden anytime a "true" LED update is received
        // ! e.g. broken during boot breath LED animation

        const uint8_t   fade_in_step = 20;
        const uint8_t   fade_out_step = 1;
        const uint8_t   colcpnt = 1; // green

        // fades-in each time pins is detected unstable
        // and fades-out when not
        uint8_t     litup = pins_unstable[output_pin];
        //uint8_t     litup = sample ^ db[output_pin].state;
        //uint8_t     litup = sample;
        //uint8_t     litup = db[output_pin].state;

        // @FIXME: this is the left-hand, right-hand key_led_map is slightly different !
        static const uint8_t key_led_map[4][16] = {
            {3, 4, 11, 12, 19, 20, 26, 27},
            {2, 5, 10, 13, 18, 21, 25, 28},
            {1, 6, 9, 14, 17, 22, 24, 29},
            {0, 7, 8, 15, 16, 23, 31, 30},
        };
        for (uint8_t i = 0; i < 8; ++i)
        {
            uint8_t     k = key_led_map[output_pin][7 - i];
            uint8_t     rgb[] = {0, 0, 0};
            led_get_one(k, rgb);
            if (litup & _BV(i))
            {
                rgb[colcpnt] = rgb[colcpnt] < 255 - fade_in_step ? rgb[colcpnt] + fade_in_step : 255;
                led_set_one_to(k, rgb);
            }
            else
            {
                if (rgb[colcpnt] > 0)
                {
                    rgb[colcpnt] = rgb[colcpnt] > fade_out_step ? rgb[colcpnt] - fade_out_step : 0;
                    led_set_one_to(k, rgb);
                }
            }
        }
#endif
    }

    // Send data
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
    keyscanner_set_interval(KEYSCAN_INTERVAL_DEFAULT);

    // enable compare interrupt
    TIMSK1 |= _BV(OCIE1A);

    // enable global interrupts
    sei();
}

// interrupt service routine (ISR) for timer 1 A compare match
ISR(TIMER1_COMPA_vect) {
    do_scan = 1; // Yes! Let's do a scan
}
