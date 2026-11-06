#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <ctime>

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "spi_protocol.h"

// 100 packets per 2-second phase at 50 Hz.
constexpr int PACKETS_PER_PHASE = 100;
constexpr int TOTAL_PACKETS = 400;

static volatile sig_atomic_t stop_requested = 0;

static void handle_stop(int) {
    stop_requested = 1;
}

struct TestCommand {
    int16_t motor_effort;
    int16_t steering_pulse_us;
    uint8_t emergency_brake;
};

// Integer ramp with the beginning and end included.
static int16_t ramp_value(int start, int end, int step) {
    return static_cast<int16_t>(
        start + (end - start) * step / (PACKETS_PER_PHASE - 1)
    );
}

static TestCommand get_test_command(int packet_index) {
    // 0-2s: steering left to right; motor remains stopped.
    if (packet_index < 100) {
        return {0, ramp_value(1750, 2250, packet_index), 0};
    }

    // 2-4s: steering right to left; motor remains stopped.
    if (packet_index < 200) {
        return {0, ramp_value(2250, 1750, packet_index - 100), 0};
    }

    // 4-6s: steering centered; motor effort ramps from 0 to 75.
    if (packet_index < 300) {
        return {ramp_value(0, 75, packet_index - 200),
                SPI_TEST_STEERING_CENTER_US, 0};
    }

    // 6-8s: immediately set motor effort to zero.
    return {0, SPI_TEST_STEERING_CENTER_US, 0};
}

// Send one 10-byte binary transaction with one CS assertion.
static bool send_command(int fd, SpiCommand *cmd) {
    if (!spi_command_test_ranges_ok(cmd)) {
        std::fprintf(stderr, "Command out of range\n");
        return false;
    }

    uint8_t tx[SPI_CMD_BYTES] = {};
    uint8_t unused_rx[SPI_CMD_BYTES] = {};
    spi_command_encode(cmd, tx);

    spi_ioc_transfer transfer = {};
    transfer.tx_buf = static_cast<uint64_t>(reinterpret_cast<uintptr_t>(tx));
    transfer.rx_buf = static_cast<uint64_t>(reinterpret_cast<uintptr_t>(unused_rx));
    transfer.len = SPI_CMD_BYTES;
    transfer.speed_hz = SPI_CMD_CLOCK_HZ;
    transfer.bits_per_word = 8;
    transfer.cs_change = 0;

    const int result = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
    if (result < 0) {
        std::perror("SPI transfer failed");
        return false;
    }
    if (result != static_cast<int>(SPI_CMD_BYTES)) {
        std::fprintf(stderr, "Incomplete SPI transfer: %d bytes\n", result);
        return false;
    }
    return true;
}

// Schedule against CLOCK_MONOTONIC, not a relative usleep loop.
static void add_period(timespec *time_point) {
    time_point->tv_nsec += static_cast<long>(SPI_CMD_PERIOD_US) * 1000L;
    if (time_point->tv_nsec >= 1000000000L) {
        ++time_point->tv_sec;
        time_point->tv_nsec -= 1000000000L;
    }
}

static bool wait_until(const timespec *next_time) {
    while (!stop_requested) {
        int result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                     next_time, nullptr);
        if (result == 0) return true;
        if (result != EINTR) {
            errno = result;
            std::perror("clock_nanosleep failed");
            return false;
        }
    }
    return true;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        std::printf("Usage: %s /dev/spidev0.0\n", argv[0]);
        return 1;
    }

    std::signal(SIGINT, handle_stop);
    std::signal(SIGTERM, handle_stop);

    const int fd = open(argv[1], O_RDWR);
    if (fd < 0) {
        std::perror("Cannot open SPI");
        return 1;
    }

    // Match RP2350 slave: SPI Mode 1, 8-bit words, MSB first.
    uint8_t mode = SPI_MODE_1;
    uint8_t bits = 8;
    uint8_t lsb_first = 0;
    uint32_t speed_hz = SPI_CMD_CLOCK_HZ;

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb_first) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) {
        std::perror("Cannot configure SPI");
        close(fd);
        return 1;
    }

    std::printf("[TX READY] SPI mode 1, requested %u Hz, 10 bytes @ 50 Hz\n",
                (unsigned)SPI_CMD_CLOCK_HZ);
    std::puts("0-2s: steering 1750 -> 2250 us, motor 0");
    std::puts("2-4s: steering 2250 -> 1750 us, motor 0");
    std::puts("4-6s: steering 2000 us, motor 0 -> 75 effort");
    std::puts("6-8s: steering 2000 us, motor 0 effort");
    std::puts("Brake flag stays 0. Press Ctrl+C to stop.\n");

    timespec next_time = {};
    if (clock_gettime(CLOCK_MONOTONIC, &next_time) != 0) {
        std::perror("clock_gettime failed");
        close(fd);
        return 1;
    }

    uint16_t sequence = 0;
    bool success = true;

    for (int i = 0; i < TOTAL_PACKETS && !stop_requested; ++i) {
        TestCommand test = get_test_command(i);
        SpiCommand cmd = {};
        cmd.sequence = sequence;
        cmd.target_motor_effort = test.motor_effort;
        cmd.target_steering_pulse_us = test.steering_pulse_us;
        cmd.emergency_brake = test.emergency_brake;

        if (!send_command(fd, &cmd)) {
            success = false;
            break;
        }

        // Print once per second and on each ramp endpoint.
        if (i % 50 == 0 || i == 99 || i == 199 || i == 299 || i == 399) {
            std::printf("[TX] seq=%u time=%.2fs motor_effort=%d "
                        "steering_us=%d brake=%u crc=%02X\n",
                        (unsigned)cmd.sequence,
                        i * 0.02,
                        (int)cmd.target_motor_effort,
                        (int)cmd.target_steering_pulse_us,
                        (unsigned)cmd.emergency_brake,
                        (unsigned)cmd.crc8);
            std::fflush(stdout);
        }

        ++sequence;
        add_period(&next_time);
        if (!wait_until(&next_time)) {
            success = false;
            break;
        }
    }

    // Best-effort final stop command, including when Ctrl+C was pressed.
    // RP2350 must still have its own communication-timeout stop protection.
    SpiCommand stop_cmd = {};
    stop_cmd.sequence = sequence;
    stop_cmd.target_motor_effort = 0;
    stop_cmd.target_steering_pulse_us = SPI_TEST_STEERING_CENTER_US;
    stop_cmd.emergency_brake = 0;

    if (!send_command(fd, &stop_cmd)) {
        success = false;
    }

    std::puts("[TX STOP] Motor effort=0, steering=2000 us");
    close(fd);
    return success ? 0 : 1;
}
