/*
 * Unified Decoder API for libmodes
 *
 * Provides float input with resampling and quantization for multiple demodulators.
 *
 * SPDX-License-Identifier: MIT
 */

#include "decoder_api.h"
#include "mode-s.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Internal decoder instance structure */
struct decoder_instance {
    decoder_config_t config;
    mode_s_t state;
    decoder_stats_t stats;

    /* Working buffers - allocated on first use */
    float *resample_i;
    float *resample_q;
    uint32_t resample_alloc;

    uint8_t *iq_buffer;
    uint32_t iq_alloc;

    uint16_t *mag_buffer;
    uint32_t mag_alloc;

    /* Callback wrapper state */
    mode_s_callback_t user_cb;
    decoder_callback_t user_cb_ex;  /* Extended callback with result */
    void *user_data;

    /* Rate conversion for timing */
    int input_rate;      /* Current input sample rate */
    int decimate_factor; /* Decimation factor (input_rate / native_rate) */
};

/* Native sample rates for each decoder type */
static const int native_rates[] = {
    [DECODER_LEGACY_2MHZ] = 2000000,
    [DECODER_2400_PHASE]  = 2400000,
};

/* Decoder names */
static const char *decoder_names[] = {
    [DECODER_LEGACY_2MHZ] = "Legacy 2 MHz",
    [DECODER_2400_PHASE]  = "2.4 MHz Phase",
};

/*
 * Decimate by integer factor with simple averaging filter.
 * For Mode-S PPM signals, averaging works well as anti-aliasing.
 */
static void decimate(const float *in, float *out, uint32_t in_len, int factor)
{
    uint32_t out_len = in_len / factor;
    for (uint32_t i = 0; i < out_len; i++) {
        float sum = 0.0f;
        for (int j = 0; j < factor; j++) {
            sum += in[i * factor + j];
        }
        out[i] = sum / factor;
    }
}

/*
 * Quantize float IQ samples to uint8 with configurable bit depth.
 *
 * Input:  float samples in range [-1.0, 1.0]
 * Output: uint8 samples in RTL-SDR format (DC at 127.5)
 *
 * adc_bits controls quantization:
 *   0 or >= 8: Full 8-bit resolution
 *   4-7: Reduced resolution (simulates lower-bit ADC)
 */
static void quantize_float_to_uint8(const float *i_in, const float *q_in,
                                    uint8_t *out, uint32_t n, int adc_bits)
{
    if (adc_bits <= 0 || adc_bits >= 8) {
        /* No quantization - direct float to uint8 */
        for (uint32_t j = 0; j < n; j++) {
            /* Clamp to [-1, 1] then scale to [0, 255] with DC at 127.5 */
            float i_clamped = i_in[j];
            float q_clamped = q_in[j];
            if (i_clamped < -1.0f) i_clamped = -1.0f;
            if (i_clamped >  1.0f) i_clamped =  1.0f;
            if (q_clamped < -1.0f) q_clamped = -1.0f;
            if (q_clamped >  1.0f) q_clamped =  1.0f;

            out[j * 2]     = (uint8_t)(i_clamped * 127.0f + 127.5f);
            out[j * 2 + 1] = (uint8_t)(q_clamped * 127.0f + 127.5f);
        }
    } else {
        /* Quantize to fewer bits, then scale to 8-bit range */
        int levels = 1 << adc_bits;
        float scale = (float)levels / 256.0f;
        float inv_scale = 256.0f / (float)levels;

        for (uint32_t j = 0; j < n; j++) {
            float i_clamped = i_in[j];
            float q_clamped = q_in[j];
            if (i_clamped < -1.0f) i_clamped = -1.0f;
            if (i_clamped >  1.0f) i_clamped =  1.0f;
            if (q_clamped < -1.0f) q_clamped = -1.0f;
            if (q_clamped >  1.0f) q_clamped =  1.0f;

            /* Scale to 0-255, quantize, scale back */
            float i_scaled = i_clamped * 127.0f + 127.5f;
            float q_scaled = q_clamped * 127.0f + 127.5f;

            int i_quant = (int)(i_scaled * scale);
            int q_quant = (int)(q_scaled * scale);

            /* Clamp to valid range */
            if (i_quant >= levels) i_quant = levels - 1;
            if (q_quant >= levels) q_quant = levels - 1;
            if (i_quant < 0) i_quant = 0;
            if (q_quant < 0) q_quant = 0;

            out[j * 2]     = (uint8_t)(i_quant * inv_scale);
            out[j * 2 + 1] = (uint8_t)(q_quant * inv_scale);
        }
    }
}

/*
 * Internal callback wrapper to track statistics and generate decoder_result_t
 */
static void stats_callback_wrapper(mode_s_t *self, struct mode_s_msg *mm)
{
    /* Find the decoder instance from the mode_s_t pointer */
    /* This is a bit hacky but avoids needing to modify the callback signature */
    decoder_instance_t *dec = (decoder_instance_t *)((char *)self -
                               offsetof(decoder_instance_t, state));

    dec->stats.messages_decoded++;
    if (mm->crcok) {
        if (mm->errorbit >= 0) {
            dec->stats.crc_corrected++;
        } else {
            dec->stats.crc_ok++;
        }
    } else {
        dec->stats.crc_failed++;
    }

    /* Call user's callback (legacy or extended) */
    if (dec->user_cb_ex) {
        /* Build decoder result with timing information */
        decoder_result_t result = {0};

        /* Convert sample offset from native rate to input rate */
        /* mm->sample_offset is in native rate samples */
        result.message_start_sample = (float)mm->sample_offset * dec->decimate_factor;
        result.has_timing = 1;

        /* SNR estimate from signal level if available */
        /* The demodulator's signal_level gives us peak magnitude */
        /* For now, we don't have a noise floor estimate, so leave SNR as invalid */
        result.has_snr = 0;
        result.has_cfo = 0;
        result.has_phase = 0;

        /* TODO: Add rate error estimation from preamble timing */
        result.symbol_rate_error_ppm = 0.0f;

        dec->user_cb_ex(self, mm, &result, dec->user_data);
    } else if (dec->user_cb) {
        dec->user_cb(self, mm);
    }
}

/*
 * Ensure working buffers are large enough
 */
static int ensure_buffers(decoder_instance_t *dec, uint32_t samples)
{
    /* Resample buffer */
    if (dec->resample_alloc < samples) {
        free(dec->resample_i);
        free(dec->resample_q);
        dec->resample_i = malloc(samples * sizeof(float));
        dec->resample_q = malloc(samples * sizeof(float));
        if (!dec->resample_i || !dec->resample_q) return -1;
        dec->resample_alloc = samples;
    }

    /* IQ buffer (interleaved uint8) */
    if (dec->iq_alloc < samples * 2) {
        free(dec->iq_buffer);
        dec->iq_buffer = malloc(samples * 2);
        if (!dec->iq_buffer) return -1;
        dec->iq_alloc = samples * 2;
    }

    /* Magnitude buffer (need extra for lookahead) */
    uint32_t mag_needed = samples + 300;  /* Extra for demod lookahead */
    if (dec->mag_alloc < mag_needed) {
        free(dec->mag_buffer);
        dec->mag_buffer = malloc(mag_needed * sizeof(uint16_t));
        if (!dec->mag_buffer) return -1;
        dec->mag_alloc = mag_needed;
    }

    return 0;
}

/* Public API */

decoder_instance_t *decoder_create(const decoder_config_t *config)
{
    if (!config || config->type >= DECODER_COUNT) {
        return NULL;
    }

    decoder_instance_t *dec = calloc(1, sizeof(decoder_instance_t));
    if (!dec) return NULL;

    dec->config = *config;

    /* Initialize mode_s state */
    mode_s_init(&dec->state);
    dec->state.fix_errors = config->fix_errors;

#ifdef LIBMODES_2400
    if (config->type == DECODER_2400_PHASE) {
        dec->state.use_2400 = 1;
        dec->state.nfix_crc = config->nfix_crc;
        dec->state.fix_df = config->fix_df;
        mode_s_init_2400(&dec->state);
    }
#endif

    return dec;
}

void decoder_destroy(decoder_instance_t *dec)
{
    if (!dec) return;

    free(dec->resample_i);
    free(dec->resample_q);
    free(dec->iq_buffer);
    free(dec->mag_buffer);
    free(dec);
}

/*
 * Common implementation for processing float IQ samples.
 * Handles resampling, quantization, magnitude computation, and demodulation.
 */
static int decoder_process_internal(decoder_instance_t *dec,
                                    const float *i_samples,
                                    const float *q_samples,
                                    uint32_t num_samples,
                                    int input_rate,
                                    mode_s_callback_t cb,
                                    decoder_callback_t cb_ex,
                                    void *user_data)
{
    if (!dec || !i_samples || !q_samples || num_samples == 0) {
        return 0;
    }

    int native_rate = native_rates[dec->config.type];
    uint32_t output_samples;
    const float *i_data;
    const float *q_data;
    int decimate_factor = 1;

    /* Step 1: Resample if needed */
    if (input_rate == native_rate) {
        /* No resampling needed */
        i_data = i_samples;
        q_data = q_samples;
        output_samples = num_samples;
        decimate_factor = 1;
    } else if (input_rate > native_rate && (input_rate % native_rate) == 0) {
        /* Integer decimation */
        decimate_factor = input_rate / native_rate;
        output_samples = num_samples / decimate_factor;

        if (ensure_buffers(dec, output_samples) < 0) return 0;

        decimate(i_samples, dec->resample_i, num_samples, decimate_factor);
        decimate(q_samples, dec->resample_q, num_samples, decimate_factor);

        i_data = dec->resample_i;
        q_data = dec->resample_q;
    } else {
        /* Unsupported rate conversion - would need rational resampler */
        /* For now, just return 0 */
        return 0;
    }

    if (ensure_buffers(dec, output_samples) < 0) return 0;

    /* Store rate conversion info for callback to use */
    dec->input_rate = input_rate;
    dec->decimate_factor = decimate_factor;

    /* Step 2: Quantize to uint8 */
    quantize_float_to_uint8(i_data, q_data, dec->iq_buffer,
                            output_samples, dec->config.adc_bits);

    /* Step 3: Compute magnitude */
    mode_s_compute_magnitude_vector(dec->iq_buffer, dec->mag_buffer, output_samples * 2);

    /* Step 4: Run demodulator */
    dec->user_cb = cb;
    dec->user_cb_ex = cb_ex;
    dec->user_data = user_data;

    uint32_t messages_before = dec->stats.messages_decoded;

#ifdef LIBMODES_2400
    if (dec->config.type == DECODER_2400_PHASE) {
        /* 2.4 MHz demodulator needs extra samples for lookahead */
        if (output_samples > 289) {
            mode_s_detect_2400(&dec->state, dec->mag_buffer, output_samples,
                               0, stats_callback_wrapper);
        }
    } else
#endif
    {
        /* Legacy 2 MHz demodulator */
        mode_s_detect(&dec->state, dec->mag_buffer, output_samples,
                      stats_callback_wrapper);
    }

    dec->stats.samples_processed += output_samples;

    /* Clear callbacks */
    dec->user_cb = NULL;
    dec->user_cb_ex = NULL;

    return dec->stats.messages_decoded - messages_before;
}

int decoder_process_float(decoder_instance_t *dec,
                          const float *i_samples,
                          const float *q_samples,
                          uint32_t num_samples,
                          int input_rate,
                          mode_s_callback_t cb,
                          void *user_data)
{
    return decoder_process_internal(dec, i_samples, q_samples, num_samples,
                                    input_rate, cb, NULL, user_data);
}

int decoder_process_float_ex(decoder_instance_t *dec,
                             const float *i_samples,
                             const float *q_samples,
                             uint32_t num_samples,
                             int input_rate,
                             decoder_callback_t cb,
                             void *user_data)
{
    return decoder_process_internal(dec, i_samples, q_samples, num_samples,
                                    input_rate, NULL, cb, user_data);
}

void decoder_get_stats(const decoder_instance_t *dec, decoder_stats_t *stats)
{
    if (!dec || !stats) return;
    *stats = dec->stats;
}

void decoder_reset(decoder_instance_t *dec)
{
    if (!dec) return;

    /* Reset statistics */
    memset(&dec->stats, 0, sizeof(dec->stats));

    /* Reinitialize mode_s state (clears ICAO cache) */
    mode_s_init(&dec->state);
    dec->state.fix_errors = dec->config.fix_errors;

#ifdef LIBMODES_2400
    if (dec->config.type == DECODER_2400_PHASE) {
        dec->state.use_2400 = 1;
        dec->state.nfix_crc = dec->config.nfix_crc;
        dec->state.fix_df = dec->config.fix_df;
        mode_s_init_2400(&dec->state);
    }
#endif
}

const char *decoder_get_name(decoder_type_t type)
{
    if (type >= DECODER_COUNT) return "Unknown";
    return decoder_names[type];
}

int decoder_get_native_rate(decoder_type_t type)
{
    if (type >= DECODER_COUNT) return 0;
    return native_rates[type];
}
