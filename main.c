#include "pico/stdlib.h"
#include "comms_spi.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#define MAX_LINE 64u
#define PARTIAL_TIMEOUT_US 20000u
#define COMMAND_TIMEOUT_US 250000u

// The Jetson sends one 32-byte SPI slot containing:
//     @sequence,speed_mm_s,steering_mrad,brake*XX\n
// XX is a two-digit XOR checksum of all characters between @ and *.
// Any remaining bytes in the slot are zero padding (not part of the text).

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static bool print_command(char *line) {
    if (line[0] != '@') return false;
    char *star = strchr(line, '*');
    if (!star || strlen(star) != 3u) return false; // *XX\0

    int high = hex_value(star[1]);
    int low  = hex_value(star[2]);
    if (high < 0 || low < 0) return false;

    uint8_t checksum = 0;
    for (char *p = line + 1; p < star; ++p) checksum ^= (uint8_t)*p;
    if (checksum != (uint8_t)((high << 4) | low)) return false;

    *star = '\0';
    unsigned int sequence = 0, brake = 0;
    int speed = 0, steering = 0, consumed = -1;
    int fields = sscanf(line + 1, "%5u,%4d,%4d,%1u%n",
                        &sequence, &speed, &steering, &brake, &consumed);
    if (fields != 4 || consumed < 0 || line[1 + consumed] != '\0') return false;
    if (sequence > 65535u || speed < -500 || speed > 500 ||
        steering < -450 || steering > 450 || brake > 1u) return false;

    printf("[RX] seq=%u speed_mm_s=%d steering_mrad=%d brake=%u\n",
           sequence, speed, steering, brake);
    return true;
}

int main(void) {
    stdio_init_all();
    comms_spi_init();
    printf("\nSPI TEXT RECEIVER READY - ACTUATORS DISABLED\n");
    printf("SPI1: SCK=10 MOSI/RX=12 MISO/TX=11 CS=13\n");

    char line[MAX_LINE];
    unsigned int used = 0;
    bool receiving = false;
    bool timeout_reported = false;
    uint64_t last_byte_us = time_us_64();
    uint64_t last_valid_us = last_byte_us;
    uint32_t last_error_count = 0;
    uint8_t byte;

    while (true) {
        // Read buffered bytes; the SPI interrupt continues receiving during printf.
        while (comms_spi_read_byte(&byte)) {
            last_byte_us = time_us_64();
            if (byte == 0) continue; // Fixed-slot zero padding.
            if (byte == '@') {
                used = 0;
                receiving = true; // Start of a new text frame.
                line[used++] = '@';
                continue;
            }
            if (!receiving) continue;
            if (byte == '\n') {
                line[used] = '\0';
                if (print_command(line)) {
                    last_valid_us = time_us_64();
                    if (timeout_reported) puts("[COMM] valid reception resumed");
                    timeout_reported = false;
                } else {
                    puts("[RX REJECT] checksum, format, or range error");
                }
                used = 0;
                receiving = false;
            } else if (byte >= 32u && byte <= 126u && used < MAX_LINE - 1u) {
                line[used++] = (char)byte;
            } else {
                // Bad character or long frame: wait for the next '@' to resync.
                used = 0;
                receiving = false;
            }
        }

        uint32_t errors = comms_spi_error_count();
        if (errors != last_error_count) {
            printf("[SPI ERROR] hardware/ring overflow: count=%lu\n",
                   (unsigned long)errors);
            last_error_count = errors;
            used = 0;
            receiving = false;
        }

        uint64_t now_us = time_us_64();
        if (receiving && now_us - last_byte_us > PARTIAL_TIMEOUT_US) {
            puts("[RX REJECT] incomplete frame timeout");
            used = 0;
            receiving = false;
        }
        if (!timeout_reported && now_us - last_valid_us > COMMAND_TIMEOUT_US) {
            puts("[COMM TIMEOUT] no valid command for 250 ms");
            timeout_reported = true;
        }
        tight_loop_contents();
    }
}
