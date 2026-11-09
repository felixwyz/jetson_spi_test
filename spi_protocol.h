#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Shared Jetson <-> RP2350 SPI command (little-endian on the wire).
// Copy this exact header to BOTH projects before running the actuator test.
#define SPI_CMD_MAGIC       0xA55Au
#define SPI_CMD_BYTES       10u
#define SPI_CMD_PERIOD_US   20000u  // 50 packets/s
#define SPI_CMD_CLOCK_HZ    100000u // Requested SCK; Jetson measured ~150 kHz with clk_m

// Bench-test command limits. Check servo endpoints and motor drive calibration.
#define SPI_TEST_MOTOR_EFFORT_MAX    100
#define SPI_TEST_STEERING_MIN_US     1000
#define SPI_TEST_STEERING_MAX_US     2000
#define SPI_TEST_STEERING_CENTER_US  1500

// Byte layout (10 bytes total):
// [0..1] marker 5A A5, [2..3] sequence, [4..5] motor effort,
// [6..7] steering pulse in microseconds, [8] brake, [9] CRC-8.
typedef struct {
    uint16_t start_marker;
    uint16_t sequence;
    int16_t target_motor_effort;
    int16_t target_steering_pulse_us;
    uint8_t emergency_brake;
    uint8_t crc8;
} SpiCommand;

#if defined(__cplusplus)
static_assert(sizeof(SpiCommand) == SPI_CMD_BYTES, "Unexpected SpiCommand layout");
static_assert(offsetof(SpiCommand, crc8) == 9, "Unexpected CRC offset");
#else
_Static_assert(sizeof(SpiCommand) == SPI_CMD_BYTES, "Unexpected SpiCommand layout");
_Static_assert(offsetof(SpiCommand, crc8) == 9, "Unexpected CRC offset");
#endif

// CRC-8: polynomial 0x07, initial 0, no reflection, xor-out 0.
// Covers packet bytes 0 through 8; byte 9 contains the CRC.
static inline uint8_t spi_command_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u)
                                : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static inline void spi_put_u16le(uint8_t *out, uint16_t value) {
    out[0] = (uint8_t)(value & 0xffu);
    out[1] = (uint8_t)((value >> 8) & 0xffu);
}

static inline uint16_t spi_get_u16le(const uint8_t *in) {
    return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

static inline int16_t spi_get_i16le(const uint8_t *in) {
    uint16_t raw = spi_get_u16le(in);
    int32_t signed_value = (raw <= 32767u) ? (int32_t)raw
                                            : ((int32_t)raw - 65536);
    return (int16_t)signed_value;
}

// Check that received commands are in our test ranges.
static inline bool spi_command_test_ranges_ok(const SpiCommand *cmd) {
    if (cmd == NULL) return false;
    return cmd->target_motor_effort >= 0 &&
           cmd->target_motor_effort <= SPI_TEST_MOTOR_EFFORT_MAX &&
           cmd->target_steering_pulse_us >= SPI_TEST_STEERING_MIN_US &&
           cmd->target_steering_pulse_us <= SPI_TEST_STEERING_MAX_US &&
           cmd->emergency_brake <= 1u;
}

// Encode explicitly so both devices see the same byte layout.
static inline void spi_command_encode(SpiCommand *cmd,
                                      uint8_t out[SPI_CMD_BYTES]) {
    cmd->start_marker = SPI_CMD_MAGIC;
    spi_put_u16le(out + 0, cmd->start_marker);
    spi_put_u16le(out + 2, cmd->sequence);
    spi_put_u16le(out + 4, (uint16_t)cmd->target_motor_effort);
    spi_put_u16le(out + 6, (uint16_t)cmd->target_steering_pulse_us);
    out[8] = cmd->emergency_brake;
    out[9] = spi_command_crc8(out, 9);
    cmd->crc8 = out[9];
}

static inline bool spi_command_decode(const uint8_t in[SPI_CMD_BYTES],
                                      SpiCommand *out) {
    if (in == NULL || out == NULL) return false;
    if (spi_get_u16le(in + 0) != SPI_CMD_MAGIC) return false;
    if (spi_command_crc8(in, 9) != in[9]) return false;

    SpiCommand decoded;
    decoded.start_marker = SPI_CMD_MAGIC;
    decoded.sequence = spi_get_u16le(in + 2);
    decoded.target_motor_effort = spi_get_i16le(in + 4);
    decoded.target_steering_pulse_us = spi_get_i16le(in + 6);
    decoded.emergency_brake = in[8];
    decoded.crc8 = in[9];

    if (!spi_command_test_ranges_ok(&decoded)) return false;
    *out = decoded;
    return true;
}
