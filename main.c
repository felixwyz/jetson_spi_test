#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "comms_spi.h"
#include "spi_protocol.h"

// Stage 1 bench test ONLY. No motor driver / servo calls in this file.
#define COMMAND_TIMEOUT_US 250000u
#define PARTIAL_TIMEOUT_US 10000u

int main(void) {
    stdio_init_all();
    comms_spi_init();

    uint8_t frame[SPI_CMD_BYTES];
    unsigned used = 0;
    uint32_t valid_total = 0, rejected_total = 0, missing_total = 0;
    uint32_t ticks_total = 0;
    uint32_t valid_this_second = 0, ticks_this_second = 0;
    uint32_t prev_hw_errors = 0, prev_ring_errors = 0;
    uint16_t last_sequence = 0;
    bool have_sequence = false, command_timeout = false;
    SpiCommand latest = {0};
    bool have_command = false;
    uint64_t now = time_us_64();
    uint64_t last_valid_us = now, last_byte_us = now;
    uint64_t next_tick_us = now + SPI_CMD_PERIOD_US;
    uint64_t next_status_us = now + 1000000u;

    puts("SPI BINARY RX READY - ACTUATORS DISABLED");
    puts("SPI1 GP10=SCK GP12=RX GP11=TX GP13=CS; 10 bytes; 50 Hz");

    while (true) {
        uint8_t b;
        while (comms_spi_read_byte(&b)) {
            last_byte_us = time_us_64();

            // Stream resync: look for magic bytes 5A A5, then 8 more bytes.
            if (used == 0) {
                if (b != (uint8_t)(SPI_CMD_MAGIC & 0xffu)) continue;
                frame[used++] = b;
                continue;
            }
            if (used == 1) {
                if (b == (uint8_t)(SPI_CMD_MAGIC >> 8)) {
                    frame[used++] = b;
                } else if (b == (uint8_t)(SPI_CMD_MAGIC & 0xffu)) {
                    frame[0] = b; // Another candidate marker begins here.
                } else {
                    used = 0;
                }
                continue;
            }

            frame[used++] = b;
            if (used == SPI_CMD_BYTES) {
                SpiCommand cmd;
                if (spi_command_decode(frame, &cmd)) {
                    latest = cmd;
                    have_command = true;
                    ++valid_total;
                    ++valid_this_second;
                    last_valid_us = time_us_64();
                    command_timeout = false;

                    if (have_sequence) {
                        uint16_t delta = (uint16_t)(cmd.sequence - last_sequence);
                        if (delta > 1u && delta < 32768u) {
                            missing_total += (uint32_t)(delta - 1u);
                        }
                    }
                    last_sequence = cmd.sequence;
                    have_sequence = true;
                    // Print first sample and every new requested command.
                    static int16_t shown_speed = 0, shown_steer = 0;
                    static uint8_t shown_brake = 0;
                    static bool shown_once = false;
                    if (!shown_once || cmd.target_speed_mm_s != shown_speed ||
                        cmd.target_steering_mrad != shown_steer ||
                        cmd.emergency_brake != shown_brake) {
                        printf("[RX] seq=%u speed_mm_s=%d steering_mrad=%d brake=%u\n",
                               (unsigned)cmd.sequence, (int)cmd.target_speed_mm_s,
                               (int)cmd.target_steering_mrad,
                               (unsigned)cmd.emergency_brake);
                        shown_speed = cmd.target_speed_mm_s;
                        shown_steer = cmd.target_steering_mrad;
                        shown_brake = cmd.emergency_brake;
                        shown_once = true;
                    }
                    used = 0;
                } else {
                    ++rejected_total;
                    // Shift by one byte; recover if a marker began inside
                    // a damaged packet, rather than discarding the next one.
                    unsigned keep = 0;
                    for (unsigned i = 1; i < SPI_CMD_BYTES; ++i) {
                        if (frame[i] == (uint8_t)(SPI_CMD_MAGIC & 0xffu)) {
                            if (i + 1u < SPI_CMD_BYTES &&
                                frame[i + 1u] == (uint8_t)(SPI_CMD_MAGIC >> 8)) {
                                keep = SPI_CMD_BYTES - i;
                                for (unsigned j = 0; j < keep; ++j)
                                    frame[j] = frame[i + j];
                                break;
                            }
                            if (i + 1u == SPI_CMD_BYTES) {
                                keep = 1;
                                frame[0] = (uint8_t)(SPI_CMD_MAGIC & 0xffu);
                            }
                        }
                    }
                    used = keep;
                }
            }
        }

        now = time_us_64();
        const uint32_t hw_errors = comms_spi_hw_overrun_count();
        const uint32_t ring_errors = comms_spi_ring_overrun_count();
        if (hw_errors != prev_hw_errors || ring_errors != prev_ring_errors) {
            // Damaged stream: wait for next start marker.
            printf("[SPI OVERRUN] hardware=%lu ring=%lu\n",
                   (unsigned long)hw_errors, (unsigned long)ring_errors);
            used = 0;
            prev_hw_errors = hw_errors;
            prev_ring_errors = ring_errors;
        }
        if (used && now - last_byte_us > PARTIAL_TIMEOUT_US) {
            ++rejected_total;
            used = 0;
            puts("[RX REJECT] partial binary packet timeout");
        }
        if (!command_timeout && now - last_valid_us > COMMAND_TIMEOUT_US) {
            command_timeout = true;
            have_command = false; // Stale commands must NOT be acted upon.
            puts("[COMM TIMEOUT] no valid packet for 250 ms");
        }

        // Distinct 50 Hz MCU control scheduler (TIDR 1-3).
        if (now >= next_tick_us) {
            ++ticks_total;
            ++ticks_this_second;
            // Stage 1: only sample latest validated data. DO NOT actuate.
            (void)latest;
            (void)have_command;
            next_tick_us += SPI_CMD_PERIOD_US;
            if (now >= next_tick_us) next_tick_us = now + SPI_CMD_PERIOD_US;
        }

        // Unconditional heartbeat: monitor remains informative if SPI is silent.
        if (now >= next_status_us) {
            printf("[STATUS] rx_bytes=%lu valid/s=%lu valid_total=%lu "
                   "rejected=%lu missing_seq=%lu ticks/s=%lu hw_ovr=%lu ring_ovr=%lu\n",
                   (unsigned long)comms_spi_rx_byte_count(),
                   (unsigned long)valid_this_second,
                   (unsigned long)valid_total,
                   (unsigned long)rejected_total,
                   (unsigned long)missing_total,
                   (unsigned long)ticks_this_second,
                   (unsigned long)hw_errors,
                   (unsigned long)ring_errors);
            valid_this_second = 0;
            ticks_this_second = 0;
            next_status_us += 1000000u;
            if (now >= next_status_us) next_status_us = now + 1000000u;
        }
        tight_loop_contents();
    }
}
