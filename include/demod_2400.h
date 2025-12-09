// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// demod_2400.h: 2.4MHz Mode S demodulator prototypes.
//
// Copyright (c) 2014,2015 Oliver Jowett <oliver@mutability.co.uk>
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef LIBMODES_DEMOD_2400_H
#define LIBMODES_DEMOD_2400_H

#include <stdint.h>
#include "mode-s.h"

// Score ranking for decoded messages (best to worst)
typedef enum {
    SR_NOT_SET = 0,               // message has not been scored yet

    SR_ALL_ZEROS,                 // a message that's all zeros
    SR_UNKNOWN_DF,                // message with unrecognized DF
    SR_UNCORRECTABLE,             // message with uncorrectable errors

    SR_UNKNOWN_THRESHOLD,         // cutoff for message that might be valid, but don't match an existing aircraft

    SR_UNRELIABLE_UNKNOWN,        // Address/Parity,                 unknown aircraft

    SR_DF11_IID_1ERROR_UNKNOWN,   // DF11, non-zero IID, 1 error,    unknown aircraft
    SR_DF11_ACQ_1ERROR_UNKNOWN,   // DF11, zero IID,     1 error,    unknown aircraft
    SR_DF11_IID_UNKNOWN,          // DF11, non-zero IID, no errors,  unknown aircraft

    SR_DF18_2ERROR_UNKNOWN,       // DF18,               2 errors,   unknown aircraft
    SR_DF17_2ERROR_UNKNOWN,       // DF17,               2 errors,   unknown aircraft

    SR_ACCEPT_THRESHOLD,          // cutoff for accepting messages

    // Address/Parity is unreliable, prefer anything else but this
    SR_UNRELIABLE_KNOWN,          // Address/Parity,                 known aircraft

    // 2-bit error correction is quite unreliable, put it low down the ranking even for known aircraft
    SR_DF18_2ERROR_KNOWN,         // DF18,               2 errors,   known aircraft
    SR_DF17_2ERROR_KNOWN,         // DF17,               2 errors,   known aircraft

    // 1-bit error when we haven't previously seen anything from this address, low priority
    SR_DF18_1ERROR_UNKNOWN,       // DF18,               1 error,    unknown aircraft
    SR_DF17_1ERROR_UNKNOWN,       // DF17,               1 error,    unknown aircraft

    // We need to accept at least one non-ES message type from unknown aircraft
    // or else we'd never accept message from Mode-S only aircraft
    SR_DF11_ACQ_UNKNOWN,          // DF11, zero IID,     no errors,  unknown aircraft

    SR_DF11_IID_1ERROR_KNOWN,     // DF11, non-zero IID, 1 error,    known aircraft
    SR_DF11_ACQ_1ERROR_KNOWN,     // DF11, zero IID,     1 error,    known aircraft
    SR_DF11_IID_KNOWN,            // DF11, non-zero IID, no errors,  known aircraft

    SR_DF18_1ERROR_KNOWN,         // DF18,               1 error,    known aircraft
    SR_DF17_1ERROR_KNOWN,         // DF17,               1 error,    known aircraft

    SR_DF11_ACQ_KNOWN,            // DF11, zero IID,     no errors,  known aircraft

    SR_DF18_UNKNOWN,              // DF18,               no errors,  unknown aircraft
    SR_DF17_UNKNOWN,              // DF17,               no errors,  unknown aircraft
    SR_DF18_KNOWN,                // DF18,               no errors,  known aircraft
    SR_DF17_KNOWN,                // DF17,               no errors,  known aircraft
} score_rank;

// Configuration for 2.4MHz demodulator
typedef struct {
    int nfix_crc;      // Max bit errors to correct (0, 1, or 2)
    int fix_df;        // Allow correction of DF field bits
    int enable_df24;   // Enable DF24 (Comm-D ELM) messages
} demod_2400_config_t;

// Statistics from demodulation
typedef struct {
    uint32_t demod_preambles;            // Preambles detected
    uint32_t demod_rejected_bad;         // Messages rejected (bad CRC/score)
    uint32_t demod_rejected_unknown_icao; // Messages rejected (unknown ICAO)
    uint32_t demod_accepted[3];          // Messages accepted by corrected bits [0,1,2]
} demod_2400_stats_t;

// Initialize the 2.4MHz demodulator
// config: configuration options
void demod_2400_init(const demod_2400_config_t *config);

// Demodulate Mode S messages from 2.4MHz magnitude samples
// mag: magnitude samples (must have at least maglen + 289 overlap samples)
// maglen: number of valid samples to process
// timestamp: sample timestamp (12MHz units, or 0 if not available)
// ctx: mode_s_t context for legacy callbacks
// cb: callback for decoded messages
// stats: optional statistics output (can be NULL)
void demod_2400_process(uint16_t *mag, uint32_t maglen,
                        uint64_t timestamp,
                        mode_s_t *ctx,
                        mode_s_callback_t cb,
                        demod_2400_stats_t *stats);

// Score a candidate message
// msg: 14-byte message buffer
// Returns: score rank
score_rank scoreModesMessage(const unsigned char *msg);

// Get the length of a Mode S message by its downlink format
int modesMessageLenByType(int type);

// Helper: extract bits from a message (1-indexed, MSB first)
static inline unsigned getbit(const unsigned char *data, unsigned bitnum)
{
    unsigned bi = bitnum - 1;
    unsigned by = bi >> 3;
    unsigned mask = 1 << (7 - (bi & 7));

    return (data[by] & mask) != 0;
}

static inline unsigned getbits(const unsigned char *data, unsigned firstbit, unsigned lastbit)
{
    unsigned fbi = firstbit - 1;
    unsigned lbi = lastbit - 1;

    unsigned fby = fbi >> 3;
    unsigned lby = lbi >> 3;
    unsigned nby = (lby - fby) + 1;

    unsigned shift = 7 - (lbi & 7);
    unsigned topmask = 0xFF >> (fbi & 7);

    if (nby == 5) {
        return
            ((data[fby] & topmask) << (32 - shift)) |
            (data[fby + 1] << (24 - shift)) |
            (data[fby + 2] << (16 - shift)) |
            (data[fby + 3] << (8 - shift)) |
            (data[fby + 4] >> shift);
    } else if (nby == 4) {
        return
            ((data[fby] & topmask) << (24 - shift)) |
            (data[fby + 1] << (16 - shift)) |
            (data[fby + 2] << (8 - shift)) |
            (data[fby + 3] >> shift);
    } else if (nby == 3) {
        return
            ((data[fby] & topmask) << (16 - shift)) |
            (data[fby + 1] << (8 - shift)) |
            (data[fby + 2] >> shift);
    } else if (nby == 2) {
        return
            ((data[fby] & topmask) << (8 - shift)) |
            (data[fby + 1] >> shift);
    } else if (nby == 1) {
        return
            (data[fby] & topmask) >> shift;
    } else {
        return 0;
    }
}

#endif // LIBMODES_DEMOD_2400_H
