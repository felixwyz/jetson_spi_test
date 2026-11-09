
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

// ---------------------------------------------
// Test configuration
// ---------------------------------------------

// Steering PWM (microseconds)
constexpr int16_t STEER_LEFT = 1000;
constexpr int16_t STEER_CENTER = 1500;
constexpr int16_t STEER_RIGHT = 2000;

// Motor effort
constexpr int16_t MOTOR_RUN = 75;
constexpr int16_t MOTOR_STOP = 0;

// Emergency brake
constexpr uint8_t BRAKE_ON = 1;
constexpr uint8_t BRAKE_OFF = 0;

// Total test duration
constexpr int TEST_DURATION_S = 15;

// 50 packets per second
constexpr int PACKETS_PER_SECOND =
    1000000 / SPI_CMD_PERIOD_US;

static_assert(1000000 % SPI_CMD_PERIOD_US == 0,
              "Packet period must divide one second");

constexpr int TOTAL_PACKETS =
    TEST_DURATION_S * PACKETS_PER_SECOND;

// Command to transmit
struct TestCommand {
    int16_t motor_effort;
    int16_t steering_pulse_us;
    uint8_t emergency_brake;
};

// Each step starts at a specified time.
struct TestStep {
    int start_time_s;
    int16_t steering_us;
    int16_t motor_effort;
    uint8_t brake;
};

// ---------------------------------------------
// Edit test commands here
// Format: {start_time, steering, motor, brake}
// ---------------------------------------------

constexpr TestStep test_steps[] = {

    // 0-3s: Drive while steering left
    {0, STEER_LEFT, MOTOR_RUN, BRAKE_OFF},

    // 3-6s: Center steering, stop motor
    {3, STEER_CENTER, MOTOR_RUN, BRAKE_OFF},

    // 6-9s: Drive while steering right
    {6, STEER_RIGHT, MOTOR_RUN, BRAKE_OFF},

    // 9-12s: Inject emergency brake
    {9, STEER_CENTER, MOTOR_RUN, BRAKE_ON},

    // 12-15s: Release brake, restart motor
    {12, STEER_CENTER, MOTOR_RUN, BRAKE_OFF}
};

constexpr int NUM_STEPS =
    sizeof(test_steps) / sizeof(test_steps[0]);

// Stop request from Ctrl+C
static volatile sig_atomic_t stop_requested = 0;

static void handle_stop(int) {
    stop_requested = 1;
}

// ---------------------------------------------
// Select test command
// ---------------------------------------------

static TestCommand get_test_command(int packet_index) {

    int elapsed_seconds =
        packet_index / PACKETS_PER_SECOND;

    // Default to motor stopped and steering centered.
    TestCommand command = {
        MOTOR_STOP,
        STEER_CENTER,
        BRAKE_OFF
    };

    // Find the most recent scheduled command.
    for (int i = 0; i < NUM_STEPS; ++i) {

        if (elapsed_seconds >= test_steps[i].start_time_s) {
            command.motor_effort = test_steps[i].motor_effort;
            command.steering_pulse_us = test_steps[i].steering_us;
            command.emergency_brake = test_steps[i].brake;
        } else {
            break;
        }
    }

    return command;
}

// ---------------------------------------------
// SPI transmission
// ---------------------------------------------

static bool send_command(int fd, SpiCommand *cmd) {

    // Check command values before sending.
    if (!spi_command_test_ranges_ok(cmd)) {
        std::fprintf(stderr, "Command out of range\n");
        return false;
    }

    uint8_t tx[SPI_CMD_BYTES] = {};
    uint8_t unused_rx[SPI_CMD_BYTES] = {};

    // Encode the 10-byte packet with CRC.
    spi_command_encode(cmd, tx);

    spi_ioc_transfer transfer = {};

    transfer.tx_buf =
        static_cast<uint64_t>(
            reinterpret_cast<uintptr_t>(tx)
        );

    transfer.rx_buf =
        static_cast<uint64_t>(
            reinterpret_cast<uintptr_t>(unused_rx)
        );

    transfer.len = SPI_CMD_BYTES;
    transfer.speed_hz = SPI_CMD_CLOCK_HZ;
    transfer.bits_per_word = 8;
    transfer.cs_change = 0;

    // Send all 10 bytes in one SPI transaction.
    int result = ioctl(
        fd,
        SPI_IOC_MESSAGE(1),
        &transfer
    );

    if (result < 0) {
        std::perror("SPI transfer failed");
        return false;
    }

    if (result != static_cast<int>(SPI_CMD_BYTES)) {
        std::fprintf(
            stderr,
            "Incomplete SPI transfer: %d bytes\n",
            result
        );
        return false;
    }

    return true;
}

// ---------------------------------------------
// 50 Hz timing
// ---------------------------------------------

// Add 20 ms to the next transmission time.
static void add_period(timespec *time_point) {

    time_point->tv_nsec +=
        static_cast<long>(SPI_CMD_PERIOD_US) * 1000L;

    if (time_point->tv_nsec >= 1000000000L) {
        time_point->tv_sec++;
        time_point->tv_nsec -= 1000000000L;
    }
}

// Wait until the next scheduled packet.
static bool wait_until(const timespec *next_time) {

    while (!stop_requested) {

        int result = clock_nanosleep(
            CLOCK_MONOTONIC,
            TIMER_ABSTIME,
            next_time,
            nullptr
        );

        if (result == 0) {
            return true;
        }

        if (result != EINTR) {
            errno = result;
            std::perror("clock_nanosleep failed");
            return false;
        }
    }

    return false;
}

// ---------------------------------------------
// Main program
// ---------------------------------------------

int main(int argc, char **argv) {

    if (argc != 2) {
        std::printf(
            "Usage: %s /dev/spidev0.0\n",
            argv[0]
        );
        return 1;
    }

    std::signal(SIGINT, handle_stop);
    std::signal(SIGTERM, handle_stop);

    // Validate the configured test timeline.
    if (test_steps[0].start_time_s != 0) {
        std::fprintf(stderr,
                     "First test step must start at 0s\n");
        return 1;
    }

    for (int i = 0; i < NUM_STEPS; ++i) {

        if (test_steps[i].start_time_s >= TEST_DURATION_S ||
            (i > 0 &&
             test_steps[i].start_time_s <=
                 test_steps[i - 1].start_time_s)) {

            std::fprintf(stderr,
                         "Invalid test step timing\n");
            return 1;
        }

        SpiCommand check = {};
        check.target_motor_effort =
            test_steps[i].motor_effort;
        check.target_steering_pulse_us =
            test_steps[i].steering_us;
        check.emergency_brake =
            test_steps[i].brake;

        if (!spi_command_test_ranges_ok(&check)) {
            std::fprintf(stderr,
                         "Test step %d is out of range\n",
                         i);
            return 1;
        }
    }

    // Open the Jetson SPI device.
    int fd = open(argv[1], O_RDWR);

    if (fd < 0) {
        std::perror("Cannot open SPI");
        return 1;
    }

    // Match the RP2350 SPI configuration.
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

    std::printf(
        "[TX READY] Mode 1, requested %u Hz, "
        "%u bytes @ %d Hz\n",
        (unsigned)SPI_CMD_CLOCK_HZ,
        (unsigned)SPI_CMD_BYTES,
        PACKETS_PER_SECOND
    );

    std::printf(
        "Test duration: %d seconds\n",
        TEST_DURATION_S
    );

    std::puts("\nTest sequence:");

    // Show configured commands before starting.
    for (int i = 0; i < NUM_STEPS; ++i) {

        int end_time = (i + 1 < NUM_STEPS)
            ? test_steps[i + 1].start_time_s
            : TEST_DURATION_S;

        std::printf(
            "[%2d-%2ds] steering=%d us "
            "motor=%d brake=%u\n",
            test_steps[i].start_time_s,
            end_time,
            (int)test_steps[i].steering_us,
            (int)test_steps[i].motor_effort,
            (unsigned)test_steps[i].brake
        );
    }

    std::puts("\nPress Ctrl+C to stop.\n");

    // Get the initial transmission time.
    timespec next_time = {};

    if (clock_gettime(CLOCK_MONOTONIC, &next_time) != 0) {
        std::perror("clock_gettime failed");
        close(fd);
        return 1;
    }

    uint16_t sequence = 0;
    bool success = true;

    // Transmit commands at 50 Hz.
    for (int i = 0;
         i < TOTAL_PACKETS && !stop_requested;
         ++i) {

        TestCommand test = get_test_command(i);

        SpiCommand cmd = {};

        cmd.sequence = sequence;
        cmd.target_motor_effort = test.motor_effort;
        cmd.target_steering_pulse_us =
            test.steering_pulse_us;
        cmd.emergency_brake = test.emergency_brake;

        if (!send_command(fd, &cmd)) {
            success = false;
            break;
        }

        // Print once per second.
        if (i % PACKETS_PER_SECOND == 0) {

            std::printf(
                "[TX] seq=%u time=%.2fs "
                "motor=%d steering=%d "
                "brake=%u crc=%02X\n",
                (unsigned)cmd.sequence,
                i * (SPI_CMD_PERIOD_US / 1000000.0),
                (int)cmd.target_motor_effort,
                (int)cmd.target_steering_pulse_us,
                (unsigned)cmd.emergency_brake,
                (unsigned)cmd.crc8
            );

            std::fflush(stdout);
        }

        sequence++;

        // Schedule next transmission.
        add_period(&next_time);

        if (!wait_until(&next_time)) {
            if (!stop_requested) {
                success = false;
            }
            break;
        }
    }

    // -----------------------------------------
    // Final motor stop
    // -----------------------------------------

    SpiCommand stop_cmd = {};

    stop_cmd.target_motor_effort = MOTOR_STOP;
    stop_cmd.target_steering_pulse_us = STEER_CENTER;
    stop_cmd.emergency_brake = BRAKE_OFF;

    // Repeat zero effort for 0.5 seconds.
    // This is more robust than a single stop packet.
    constexpr int STOP_PACKETS = 25;

    bool stop_sent = true;

    for (int i = 0; i < STOP_PACKETS; ++i) {

        stop_cmd.sequence = sequence++;

        if (!send_command(fd, &stop_cmd)) {
            stop_sent = false;
            success = false;
            break;
        }

        if (i + 1 < STOP_PACKETS) {
            usleep(SPI_CMD_PERIOD_US);
        }
    }

    if (stop_sent) {
        std::puts(
            "\n[TX STOP] Motor=0, steering=2000 us, "
            "brake=0"
        );
    } else {
        std::puts(
            "\n[TX WARNING] Final stop transmission failed"
        );
    }

    close(fd);

    if (stop_requested) {
        std::puts("Test interrupted.");
    } else if (success) {
        std::puts("Test sequence complete.");
    }

    return success ? 0 : 1;
}
