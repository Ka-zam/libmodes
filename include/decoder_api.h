/*
 * Unified Decoder API for libmodes
 *
 * Provides a common interface for multiple Mode-S demodulator implementations
 * with built-in resampling and quantization support.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef LIBMODES_DECODER_API_H
#define LIBMODES_DECODER_API_H

#include <stdint.h>
#include "mode-s.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Decoder type enumeration */
typedef enum {
    DECODER_LEGACY_2MHZ,    /* Original 2 MHz demod (mode_s_detect) */
    DECODER_2400_PHASE,     /* 2.4 MHz with phase tracking (demod_2400) */
    DECODER_COUNT
} decoder_type_t;

/* Decoder configuration */
typedef struct {
    decoder_type_t type;    /* Which demodulator to use */
    int adc_bits;           /* Quantization bits: 0=none, 4-16 bits */
    int fix_errors;         /* Enable single-bit error correction */
    int nfix_crc;           /* Max CRC bits to fix (0, 1, or 2) - 2400 mode */
    int fix_df;             /* Allow DF field correction - 2400 mode */
} decoder_config_t;

/* Decoder statistics */
typedef struct {
    uint32_t samples_processed;
    uint32_t preambles_detected;
    uint32_t messages_decoded;
    uint32_t crc_ok;
    uint32_t crc_corrected;
    uint32_t crc_failed;
} decoder_stats_t;

/* Decode result with timing and estimated impairments */
typedef struct {
    /* Message timing (in INPUT sample rate, not native rate) */
    float message_start_sample;     /* Fractional sample index where preamble starts */
    float symbol_rate_error_ppm;    /* Estimated symbol rate error in ppm (0 if not estimated) */

    /* Estimated impairments (0 if not estimated) */
    float snr_db;                   /* Estimated SNR in dB */
    float cfo_hz;                   /* Estimated carrier frequency offset */
    float phase_offset_rad;         /* Estimated carrier phase offset */

    /* Flags indicating which estimates are valid */
    uint8_t has_snr;
    uint8_t has_cfo;
    uint8_t has_phase;
    uint8_t has_timing;
} decoder_result_t;

/* Extended callback with decode result */
typedef void (*decoder_callback_t)(mode_s_t *state, struct mode_s_msg *mm,
                                   const decoder_result_t *result, void *user_data);

/* Opaque decoder instance */
typedef struct decoder_instance decoder_instance_t;

/*
 * Create a decoder instance with the given configuration.
 * Returns NULL on failure.
 */
decoder_instance_t *decoder_create(const decoder_config_t *config);

/*
 * Destroy a decoder instance and free resources.
 */
void decoder_destroy(decoder_instance_t *dec);

/*
 * Process float IQ samples through the decoder.
 *
 * The decoder will:
 * 1. Resample from input_rate to the decoder's native rate
 * 2. Apply ADC quantization if configured
 * 3. Compute magnitude
 * 4. Run the demodulator
 *
 * Parameters:
 *   dec         - Decoder instance
 *   i_samples   - I channel samples (float, -1.0 to 1.0)
 *   q_samples   - Q channel samples (float, -1.0 to 1.0)
 *   num_samples - Number of samples
 *   input_rate  - Input sample rate in Hz (e.g., 12000000 for 12 MSPS)
 *   cb          - Callback function for decoded messages
 *   user_data   - User data passed to callback
 *
 * Returns number of messages decoded.
 */
int decoder_process_float(decoder_instance_t *dec,
                          const float *i_samples,
                          const float *q_samples,
                          uint32_t num_samples,
                          int input_rate,
                          mode_s_callback_t cb,
                          void *user_data);

/*
 * Process float IQ samples with extended callback.
 *
 * Same as decoder_process_float but uses the extended callback that
 * provides timing and estimated impairment information.
 *
 * The decoder_result_t will contain:
 *   - message_start_sample: Sample index in INPUT rate where preamble starts
 *   - Estimated impairments (SNR, CFO, phase) when available
 */
int decoder_process_float_ex(decoder_instance_t *dec,
                             const float *i_samples,
                             const float *q_samples,
                             uint32_t num_samples,
                             int input_rate,
                             decoder_callback_t cb,
                             void *user_data);

/*
 * Get decoder statistics.
 */
void decoder_get_stats(const decoder_instance_t *dec, decoder_stats_t *stats);

/*
 * Reset decoder state (ICAO cache, statistics).
 */
void decoder_reset(decoder_instance_t *dec);

/*
 * Get human-readable decoder name.
 */
const char *decoder_get_name(decoder_type_t type);

/*
 * Get the decoder's native sample rate in Hz.
 */
int decoder_get_native_rate(decoder_type_t type);

#ifdef __cplusplus
}
#endif

#endif /* LIBMODES_DECODER_API_H */
