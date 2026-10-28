#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Shared Jetson <-> MCU wire contract (Stage 1, little-endian).
// The named C struct is the command representation on BOTH sides.
// spi_command_encode/decode create the exact 10-byte wire representation.
#define SPI_CMD_MAGIC           0xA55Au
#define SPI_CMD_BYTES           10u
#define SPI_CMD_PERIOD_US       20000u  // 50 Hz
#define SPI_CMD_CLOCK_HZ        100000u // Wire bit clock; NOT command frequency

// Prototyping ranges only; final limits require team calibration/approval.
#define SPI_TEST_SPEED_LIMIT_MM_S 500
#define SPI_TEST_STEER_LIMIT_MRAD 450

typedef struct {
    uint16_t start_marker;
    uint16_t sequence;
    int16_t target_speed_mm_s;
    int16_t target_steering_mrad;
    uint8_t emergency_brake;
    uint8_t crc8;
} SpiCommand;

#if defined(__cplusplus)
static_assert(sizeof(SpiCommand) == SPI_CMD_BYTES, "Unexpected SpiCommand layout");
static_assert(offsetof(SpiCommand, crc8) == 9, "Unexpected SpiCommand offset");
#else
_Static_assert(sizeof(SpiCommand) == SPI_CMD_BYTES, "Unexpected SpiCommand layout");
_Static_assert(offsetof(SpiCommand, crc8) == 9, "Unexpected SpiCommand offset");
#endif

// CRC-8 polynomial 0x07, init 0, no reflection, xor-out 0.
// Covers packet bytes [0..8]; packet[9] is the CRC.
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

static inline void spi_put_u16le(uint8_t *out, uint16_t n) {
    out[0] = (uint8_t)(n & 0xffu);
    out[1] = (uint8_t)((n >> 8) & 0xffu);
}

static inline uint16_t spi_get_u16le(const uint8_t *in) {
    return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

static inline int16_t spi_get_i16le(const uint8_t *in) {
    const uint16_t raw = spi_get_u16le(in);
    const int32_t signed_value = (raw <= 32767u) ? (int32_t)raw
                                                  : ((int32_t)raw - 65536);
    return (int16_t)signed_value;
}

static inline bool spi_command_test_ranges_ok(const SpiCommand *cmd) {
    if (cmd == NULL) return false;
    return cmd->target_speed_mm_s >= -SPI_TEST_SPEED_LIMIT_MM_S &&
           cmd->target_speed_mm_s <= SPI_TEST_SPEED_LIMIT_MM_S &&
           cmd->target_steering_mrad >= -SPI_TEST_STEER_LIMIT_MRAD &&
           cmd->target_steering_mrad <= SPI_TEST_STEER_LIMIT_MRAD &&
           cmd->emergency_brake <= 1u;
}

// Always encode explicitly rather than memcpy'ing a compiler-native struct.
static inline void spi_command_encode(SpiCommand *cmd,
                                      uint8_t out[SPI_CMD_BYTES]) {
    cmd->start_marker = SPI_CMD_MAGIC;
    spi_put_u16le(out + 0, cmd->start_marker);
    spi_put_u16le(out + 2, cmd->sequence);
    spi_put_u16le(out + 4, (uint16_t)cmd->target_speed_mm_s);
    spi_put_u16le(out + 6, (uint16_t)cmd->target_steering_mrad);
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
    decoded.target_speed_mm_s = spi_get_i16le(in + 4);
    decoded.target_steering_mrad = spi_get_i16le(in + 6);
    decoded.emergency_brake = in[8];
    decoded.crc8 = in[9];
    if (!spi_command_test_ranges_ok(&decoded)) return false;
    *out = decoded;
    return true;
}
