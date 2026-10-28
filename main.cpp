#include <cerrno>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "spi_protocol.h"

// Standalone Stage 1 mock commands.
// The MCU Stage 1 receiver keeps motors and servo disabled.
struct TestCommand {
    int16_t speed;
    int16_t steer;
    uint8_t brake;
};

static constexpr TestCommand commands[] = {
    {0,   0,    0}, // Stationary.
    {100, 0,    0}, // Forward request.
    {100, 150,  0}, // Right steering request.
    {100, -150, 0}, // Left steering request.
    {0,   0,    1}, // Emergency brake request.
    {0,   0,    0}  // Brake released.
};

// Advance an absolute deadline by one command period.
static void add_period(struct timespec *t) {
    t->tv_nsec += (long)SPI_CMD_PERIOD_US * 1000L;

    if (t->tv_nsec >= 1000000000L) {
        ++t->tv_sec;
        t->tv_nsec -= 1000000000L;
    }
}

// Return true when timestamp a is later than timestamp b.
static bool time_is_later(
    const struct timespec &a,
    const struct timespec &b
) {
    return a.tv_sec > b.tv_sec ||
           (a.tv_sec == b.tv_sec && a.tv_nsec > b.tv_nsec);
}

// Print the encoded command bytes for comparison with MCU reception.
static void dump_hex(const uint8_t *data) {
    for (unsigned i = 0; i < SPI_CMD_BYTES; ++i) {
        std::printf(" %02X", (unsigned)data[i]);
    }

    std::putchar('\n');
}

int main(int argc, char **argv) {
    if (argc != 2) {
        std::fprintf(
            stderr,
            "Usage: %s /dev/spidevB.C  OR  %s --dry-run\n",
            argv[0],
            argv[0]
        );
        return 1;
    }

    const bool dry_run = std::strcmp(argv[1], "--dry-run") == 0;
    int fd = -1;

    if (!dry_run) {
        // Open the SPI device configured on the Jetson.
        fd = open(argv[1], O_RDWR);

        if (fd < 0) {
            std::perror("open SPI");
            return 1;
        }

        // Match the MCU: mode 0, eight-bit words, MSB first.
        uint8_t mode = SPI_MODE_0;
        uint8_t bits = 8;
        uint8_t lsb_first = 0;
        uint32_t speed = SPI_CMD_CLOCK_HZ;

        if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
            ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
            ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb_first) < 0 ||
            ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
            std::perror("SPI setup");
            close(fd);
            return 1;
        }
    }

    std::puts(
        dry_run
            ? "[DRY RUN] generating binary packets only"
            : "[TX READY] SPI mode 0, 100 kHz SCK, 10 bytes @ 50 Hz"
    );

    uint16_t sequence = 0;
    unsigned step = 0;
    unsigned repeats = 0;
    unsigned sent_count = 0;

    // Generate all six mock steps, then exit in dry-run mode.
    const unsigned max_dry_packets = 300;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (true) {
        const TestCommand v = commands[step];

        SpiCommand cmd = {};
        cmd.sequence = sequence;
        cmd.target_speed_mm_s = v.speed;
        cmd.target_steering_mrad = v.steer;
        cmd.emergency_brake = v.brake;

        // Encode the shared ten-byte layout, including its CRC.
        uint8_t tx[SPI_CMD_BYTES];
        spi_command_encode(&cmd, tx);

        if (!dry_run) {
            // Received bytes are discarded; Stage 1 has no acknowledgement.
            uint8_t unused_rx[SPI_CMD_BYTES] = {};

            // Mode 0: request a CS pulse between each eight-bit word.
            // All ten bytes still form one command.
            spi_ioc_transfer transfers[SPI_CMD_BYTES] = {};

            for (unsigned i = 0; i < SPI_CMD_BYTES; ++i) {
                transfers[i].tx_buf = (uint64_t)(uintptr_t)&tx[i];
                transfers[i].rx_buf = (uint64_t)(uintptr_t)&unused_rx[i];
                transfers[i].len = 1;
                transfers[i].speed_hz = SPI_CMD_CLOCK_HZ;
                transfers[i].bits_per_word = 8;

                // Pulse CS after the first nine bytes.
                // The final transfer ends the message with CS deasserted.
                transfers[i].cs_change = (i + 1u < SPI_CMD_BYTES);
            }

            // Submit all ten byte transfers in one system call.
            const int n = ioctl(
                fd,
                SPI_IOC_MESSAGE(SPI_CMD_BYTES),
                transfers
            );

            // A successful message transfers ten bytes in total.
            if (n != (int)SPI_CMD_BYTES) {
                std::fprintf(
                    stderr,
                    "SPI transfer failed: rc=%d errno=%d (%s)\n",
                    n,
                    errno,
                    std::strerror(errno)
                );
                close(fd);
                return 1;
            }
        }

        // Print representative commands instead of every frame.
        if (repeats == 0 || (sequence % 50u) == 0u) {
            std::printf(
                "[TX] seq=%u speed_mm_s=%d steer_mrad=%d "
                "brake=%u crc=%02X bytes:",
                (unsigned)cmd.sequence,
                (int)cmd.target_speed_mm_s,
                (int)cmd.target_steering_mrad,
                (unsigned)cmd.emergency_brake,
                (unsigned)cmd.crc8
            );

            dump_hex(tx);
            std::fflush(stdout);
        }

        // uint16_t wraps automatically after 65535.
        ++sequence;
        ++sent_count;

        // Hold each mock command for 50 packets: nominally one second.
        if (++repeats == 50u) {
            repeats = 0;
            step = (step + 1u) %
                   (sizeof(commands) / sizeof(commands[0]));
        }

        if (dry_run && sent_count == max_dry_packets) {
            break;
        }

        // Absolute 20 ms pacing avoids accumulating transfer/sleep drift.
        add_period(&next);

        struct timespec current;
        clock_gettime(CLOCK_MONOTONIC, &current);

        if (time_is_later(current, next)) {
            next = current; // Existing missed-deadline behavior.
        }

        // Dry-run generates packets without waiting or accessing SPI.
        if (!dry_run) {
            int rc;

            do {
                rc = clock_nanosleep(
                    CLOCK_MONOTONIC,
                    TIMER_ABSTIME,
                    &next,
                    nullptr
                );
            } while (rc == EINTR);

            if (rc != 0) {
                std::fprintf(
                    stderr,
                    "clock_nanosleep error=%d\n",
                    rc
                );
                close(fd);
                return 1;
            }
        }
    }

    if (fd >= 0) {
        close(fd);
    }

    return 0;
}
