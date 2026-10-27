#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

static constexpr size_t FRAME_BYTES = 32;
static constexpr uint32_t SPI_CLOCK_HZ = 100000;

static bool make_frame(char (&tx)[FRAME_BYTES], unsigned int sequence,
                       const char *command) {
    char body[24];
    int body_len = std::snprintf(body, sizeof(body), "%u,%s", sequence, command);
    if (body_len < 0 || (size_t)body_len >= sizeof(body)) return false;

    uint8_t checksum = 0;
    for (int i = 0; i < body_len; ++i) checksum ^= (uint8_t)body[i];
    std::memset(tx, 0, sizeof(tx));
    int length = std::snprintf(tx, sizeof(tx), "@%s*%02X\n", body, checksum);
    return length > 0 && (size_t)length < sizeof(tx);
}

int main(int argc, char **argv) {
    const bool dry_run = argc == 2 && std::strcmp(argv[1], "--dry-run") == 0;
    if (argc != 2) {
        std::printf("Usage: %s /dev/spidevB.C   OR   %s --dry-run\n", argv[0], argv[0]);
        return 1;
    }

    int fd = -1;
    if (!dry_run) {
        fd = open(argv[1], O_RDWR);
        if (fd < 0) { std::perror("Cannot open SPI"); return 1; }

        uint8_t mode = SPI_MODE_0, bits = 8, lsb_first = 0;
        uint32_t speed = SPI_CLOCK_HZ;
        if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
            ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
            ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb_first) < 0 ||
            ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
            std::perror("Cannot configure SPI");
            close(fd);
            return 1;
        }
    }

    // speed_mm_s, steering_mrad, brake (NOT raw motor/servo outputs).
    const char* commands[] = {
        "0,0,0",     // Stationary
        "100,0,0",   // Forward
        "100,150,0", // Right steering
        "100,-150,0",// Left steering
        "0,0,1",     // Brake
        "0,0,0"      // Brake released
    };

    unsigned int sequence = 0, step = 0, repetitions = 0;
    std::puts(dry_run ? "DRY RUN (no SPI transfers)" :
                        "SPI sender ready: 100 kHz, 32-byte ASCII slots, mode 0");
    while (true) {
        char tx[FRAME_BYTES];
        if (!make_frame(tx, sequence, commands[step])) {
            std::fputs("Command did not fit fixed SPI frame\n", stderr);
            if (fd >= 0) close(fd);
            return 1;
        }

        if (!dry_run) {
            uint8_t rx[FRAME_BYTES] = {0}; // No MCU response defined in Stage 1.
            spi_ioc_transfer transfer = {};
            transfer.tx_buf = (uint64_t)(uintptr_t)tx;
            transfer.rx_buf = (uint64_t)(uintptr_t)rx;
            transfer.len = (uint32_t)FRAME_BYTES;
            transfer.speed_hz = SPI_CLOCK_HZ;
            transfer.bits_per_word = 8;
            int sent = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
            if (sent != (int)FRAME_BYTES) {
                std::perror("SPI_IOC_MESSAGE failed / incomplete");
                close(fd);
                return 1;
            }
        }

        std::printf("[TX] %s", tx);
        std::fflush(stdout);
        ++sequence;
        if (sequence > 65535u) sequence = 0;
        if (++repetitions == 20u) {
            repetitions = 0;
            step = (step + 1u) % 6u;
        }
        if (dry_run && sequence == 120u) break; // One complete cycle for offline testing.
        if (!dry_run) usleep(50000); // ~20 packets/s excluding transfer duration.
    }
    if (fd >= 0) close(fd);
    return 0;
}
