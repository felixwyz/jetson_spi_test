#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::printf("Usage: %s /dev/spidevB.C\n", argv[0]);
        return 1;
    }

    // Open the SPI device identified on your Jetson.
    const int fd = open(argv[1], O_RDWR);

    if (fd < 0) {
        std::perror("Cannot open SPI");
        return 1;
    }

    // Match the MCU: mode 1, eight-bit words, MSB first.
    uint8_t mode = SPI_MODE_1;
    uint8_t bits = 8;
    uint8_t lsb_first = 0;
    uint32_t clock_hz = 100000;

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb_first) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &clock_hz) < 0) {
        std::perror("Cannot configure SPI");
        close(fd);
        return 1;
    }

    // Each entry contains: speed_mm_s, steering_mrad, brake.
    // These are simulated requests, not motor/servo outputs.
    const char* commands[] = {
        "0,0,0",       // Stationary.
        "100,0,0",     // Forward request.
        "100,150,0",   // Right steering request.
        "100,-150,0",  // Left steering request.
        "0,0,1",       // Brake request.
        "0,0,0"        // Stationary, brake released.
    };

    unsigned int sequence = 0;
    unsigned int step = 0;
    unsigned int repetitions = 0;

    std::puts("SPI sender ready: 100 kHz, mode 0.");
    std::puts("Press Ctrl+C to stop.");

    while (true) {
        char line[64];

        // A newline marks the end of each text command.
        std::snprintf(
            line,
            sizeof(line),
            "%u,%s\n",
            sequence,
            commands[step]
        );

        const size_t length = std::strlen(line);

        // Send the complete line through physical SPI.
        // No MCU response is requested for Stage 1.
        const ssize_t sent = write(fd, line, length);

        if (sent != static_cast<ssize_t>(length)) {
            std::fprintf(
                stderr,
                "SPI write failed/incomplete: sent=%zd expected=%zu\n",
                sent,
                length
            );
            close(fd);
            return 1;
        }

        std::printf("[TX] %s", line);
        std::fflush(stdout);

        sequence = (sequence + 1) % 65536;

        // Hold each test command for approximately one second.
        if (++repetitions == 20) {
            repetitions = 0;
            step = (step + 1) % 6;
        }

        // Approximately 20 transmissions per second.
        usleep(50000);
    }
}
