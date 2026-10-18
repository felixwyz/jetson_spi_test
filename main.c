#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/regs/spi.h"
#include "comms_spi.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// Parse and print one complete command.
// Returns true only when all four fields are valid.
static bool print_command(const char *line) {
    int sequence;
    int speed;
    int steering;
    int brake;
    char extra;

    // Width limits keep malformed numbers small enough for int.
    // A fifth character means unwanted data followed the four fields.
    const int fields = sscanf(
        line,
        "%5d,%4d,%4d,%1d%c",
        &sequence,
        &speed,
        &steering,
        &brake,
        &extra
    );

    if (fields != 4) {
        return false;
    }

    // Demonstration ranges, not calibrated actuator limits.
    if (sequence < 0 || sequence > 65535 ||
        speed < -500 || speed > 500 ||
        steering < -450 || steering > 450 ||
        (brake != 0 && brake != 1)) {
        return false;
    }

    printf(
        "[RX] seq=%d speed_mm_s=%d steering_mrad=%d brake=%d\n",
        sequence,
        speed,
        steering,
        brake
    );

    // Stage 2 can later pass validated values through safety/control.
    // No motor or servo commands are issued in this demonstration.
    return true;
}

int main(void) {
    // Use the existing working standard-output configuration.
    stdio_init_all();

    // Existing function configures SPI1 slave on GPIO10/11/12/13.
    comms_spi_init();

    spi_set_format(
        spi1,
        8,
        SPI_CPOL_0,
        SPI_CPHA_1,
        SPI_MSB_FIRST
    );

    char line[64];
    unsigned int used = 0;

    bool discard_line = false;
    bool timeout_reported = false;

    uint64_t last_byte_us = time_us_64();
    uint64_t last_valid_us = last_byte_us;

    printf("\nSPI RECEIVER READY - ACTUATORS DISABLED\n");
    printf("SCK=GPIO10, MOSI/RX=GPIO12, CS=GPIO13\n");

    while (true) {
        // Detect receive-FIFO overflow and discard the damaged line.
        if (spi_get_hw(spi1)->ris & SPI_SSPRIS_RORRIS_BITS) {
            spi_get_hw(spi1)->icr = SPI_SSPICR_RORIC_BITS;
            used = 0;
            discard_line = true;
        }

        // Read only available bytes. Never wait for more master clocks.
        if (spi_is_readable(spi1)) {
            const char received = (char)spi_get_hw(spi1)->dr;
            last_byte_us = time_us_64();

            if (received == '\n') {
                line[used] = '\0';

                if (!discard_line && print_command(line)) {
                    last_valid_us = time_us_64();

                    if (timeout_reported) {
                        printf("[COMM] valid reception resumed\n");
                    }

                    timeout_reported = false;
                } else {
                    printf("[RX REJECT] invalid or incomplete command\n");
                }

                used = 0;
                discard_line = false;
            } else if (!discard_line) {
                if (used < sizeof(line) - 1) {
                    line[used++] = received;
                } else {
                    // Ignore an oversized command until its newline.
                    discard_line = true;
                }
            }
        }

        const uint64_t now = time_us_64();

        // Discard a partial command if the remaining bytes never arrive.
        if ((used != 0 || discard_line) &&
            now - last_byte_us > 10000) {
            used = 0;
            discard_line = false;
            printf("[RX REJECT] incomplete command timed out\n");
        }

        // Report once when no valid command has arrived for 250 ms.
        if (!timeout_reported &&
            now - last_valid_us > 250000) {
            printf("[COMM TIMEOUT] no valid command for 250 ms\n");
            timeout_reported = true;
        }

        // Poll continuously so the small SPI FIFO is drained promptly.
        tight_loop_contents();
    }
}
