// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// demod_2400.c: 2.4MHz Mode S demodulator.
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

#include "demod_2400.h"
#include "crc.h"
#include "icao_filter.h"
#include <string.h>
#include <assert.h>

#define MODES_SHORT_MSG_BITS 56
#define MODES_LONG_MSG_BITS 112
#define MODES_SHORT_MSG_BYTES (56/8)
#define MODES_LONG_MSG_BYTES (112/8)

// 2.4MHz sampling rate version
//
// When sampling at 2.4MHz we have exactly 6 samples per 5 symbols.
// Each symbol is 500ns wide, each sample is 416.7ns wide
//
// We maintain a phase offset that is expressed in units of 1/5 of a sample i.e. 1/6 of a symbol, 83.333ns
// Each symbol we process advances the phase offset by 6 i.e. 6/5 of a sample, 500ns
//
// The correlation functions below correlate a 1-0 pair of symbols (i.e. manchester encoded 1 bit)
// starting at the given sample, and assuming that the symbol starts at a fixed 0-5 phase offset within
// m[0]. They return a correlation value, generally interpreted as >0 = 1 bit, <0 = 0 bit

// nb: the correlation functions sum to zero, so we do not need to adjust for the DC offset in the input signal
// (adding any constant value to all of m[0..3] does not change the result)

static inline int slice_phase0(uint16_t *m) {
    return 5 * m[0] - 3 * m[1] - 2 * m[2];
}
static inline int slice_phase1(uint16_t *m) {
    return 4 * m[0] - m[1] - 3 * m[2];
}
static inline int slice_phase2(uint16_t *m) {
    return 3 * m[0] + m[1] - 4 * m[2];
}
static inline int slice_phase3(uint16_t *m) {
    return 2 * m[0] + 3 * m[1] - 5 * m[2];
}
static inline int slice_phase4(uint16_t *m) {
    return m[0] + 5 * m[1] - 5 * m[2] - m[3];
}

// Module-level configuration
static demod_2400_config_t g_config;
static uint32_t valid_df_short_bitset;        // set of acceptable DF values for short messages
static uint32_t valid_df_long_bitset;         // set of acceptable DF values for long messages
static int g_initialized = 0;

static uint32_t generate_damage_set(uint8_t df, unsigned damage_bits)
{
    uint32_t result = (1 << df);
    if (!damage_bits)
        return result;

    for (unsigned bit = 0; bit < 5; ++bit) {
        unsigned damaged_df = df ^ (1 << bit);
        result |= generate_damage_set(damaged_df, damage_bits - 1);
    }

    return result;
}

static void init_bitsets(void)
{
    // DFs that we directly understand without correction
    valid_df_short_bitset = (1 << 0) | (1 << 4) | (1 << 5) | (1 << 11);
    valid_df_long_bitset = (1 << 16) | (1 << 17) | (1 << 18) | (1 << 20) | (1 << 21);

    if (g_config.enable_df24)
        valid_df_long_bitset |= (1 << 24) | (1 << 25) | (1 << 26) | (1 << 27) | (1 << 28) | (1 << 29) | (1 << 30) | (1 << 31);

    // if we can also repair DF damage, include those corrections
    if (g_config.fix_df && g_config.nfix_crc) {
        valid_df_short_bitset |= generate_damage_set(11, 1);
        valid_df_long_bitset |= generate_damage_set(17, g_config.nfix_crc);
        valid_df_long_bitset |= generate_damage_set(18, g_config.nfix_crc);
    }
}

void demod_2400_init(const demod_2400_config_t *config)
{
    if (config) {
        g_config = *config;
    } else {
        // Default configuration
        g_config.nfix_crc = 1;
        g_config.fix_df = 1;
        g_config.enable_df24 = 0;
    }

    modesChecksumInit(g_config.nfix_crc);
    icaoFilterInit();
    init_bitsets();
    g_initialized = 1;
}

int modesMessageLenByType(int type) {
    return (type & 0x10) ? MODES_LONG_MSG_BITS : MODES_SHORT_MSG_BITS;
}

// is this message a long-form message with a DF that uses Parity/Interrogator?
static int isLongPIMessage(const unsigned char *msg)
{
    const unsigned df = getbits(msg, 1, 5);
    if (df == 17 || df == 18)
        return 1;
    return 0;
}

// is this message a short-form message with a DF that uses Parity/Interrogator?
static int isShortPIMessage(const unsigned char *msg)
{
    const unsigned df = getbits(msg, 1, 5);
    return (df == 11); // assume IID==0
}

#define UNCHECKED_SYNDROME 0xFFFFFFFFU

static int correctMessage(const unsigned char *in, unsigned char *out, uint32_t *short_syndrome, uint32_t *long_syndrome)
{
    // Possible DF values of the first byte of a message that could be a valid DF11/17/18
    // message after correction.
    static const uint32_t df_correctable_short[MODES_MAX_BITERRORS + 1] = {
        0x00000800, 0x08008e08, 0x08008e08
    };
    static const uint32_t df_correctable_long[MODES_MAX_BITERRORS + 1] = {
        0x00060000, 0x066f0006, 0x6fff066f
    };

    *short_syndrome = UNCHECKED_SYNDROME;
    *long_syndrome = UNCHECKED_SYNDROME;

    const unsigned uncorrected_df = getbits(in, 1, 5);
    const uint32_t df_bit = 1 << uncorrected_df;

    const unsigned fix_df_bits = (g_config.fix_df ? g_config.nfix_crc : 0);

    struct errorinfo *long_ei = NULL;
    if (df_correctable_long[fix_df_bits] & df_bit) {
        *long_syndrome = modesChecksum(in, MODES_LONG_MSG_BITS);
        if (isLongPIMessage(in) && *long_syndrome == 0) {
            // DF17/18 message with correct checksum
            memcpy(out, in, MODES_LONG_MSG_BYTES);
            return 0;
        }

        long_ei = modesChecksumDiagnose(*long_syndrome, MODES_LONG_MSG_BITS);
    }

    struct errorinfo *short_ei = NULL;
    if (df_correctable_short[fix_df_bits] & df_bit) {
        *short_syndrome = modesChecksum(in, MODES_SHORT_MSG_BITS);
        if (isShortPIMessage(in) && (*short_syndrome & 0xFFFF80) == 0) {
            // DF11 message with correct checksum
            memcpy(out, in, MODES_SHORT_MSG_BYTES);
            return 0;
        }

        short_ei = modesChecksumDiagnose(*short_syndrome, MODES_SHORT_MSG_BITS);
    }

    unsigned long_errors = (long_ei ? long_ei->errors : 999);
    unsigned short_errors = (short_ei ? short_ei->errors : 999);

    if (long_ei && long_errors <= short_errors) {
        memcpy(out, in, MODES_LONG_MSG_BYTES);
        modesChecksumFix(out, long_ei);
        if (isLongPIMessage(out)) {
            return long_errors;
        }
    }

    // Don't try to correct >1 error in DF11
    if (short_ei && short_errors == 1) {
        memcpy(out, in, MODES_SHORT_MSG_BYTES);
        modesChecksumFix(out, short_ei);
        if (isShortPIMessage(out)) {
            return short_errors;
        }
    }

    if (long_ei && long_errors > short_errors) {
        memcpy(out, in, MODES_LONG_MSG_BYTES);
        modesChecksumFix(out, long_ei);
        if (isLongPIMessage(out)) {
            return long_errors;
        }
    }

    memcpy(out, in, MODES_LONG_MSG_BYTES);
    return -1;
}

// Score how plausible this ModeS message looks.
score_rank scoreModesMessage(const unsigned char *uncorrected)
{
    static const unsigned char all_zeros[MODES_SHORT_MSG_BYTES] = { 0, 0, 0, 0, 0, 0, 0 };
    if (!memcmp(all_zeros, uncorrected, sizeof(all_zeros)))
        return SR_ALL_ZEROS;

    unsigned char corrected[14];
    uint32_t short_syndrome, long_syndrome;
    int corrections = correctMessage(uncorrected, corrected, &short_syndrome, &long_syndrome);

    unsigned df = getbits(corrected, 1, 5);
    switch (df) {
    case 0:  // short air-air surveillance
    case 4:  // surveillance, altitude reply
    case 5:  // surveillance, identity reply
        {
            if (short_syndrome == UNCHECKED_SYNDROME)
                short_syndrome = modesChecksum(corrected, MODES_SHORT_MSG_BITS);
            int recent = icaoFilterTest(short_syndrome);
            return recent ? SR_UNRELIABLE_KNOWN : SR_UNRELIABLE_UNKNOWN;
        }

    case 16: // long air-air surveillance
    case 20: // Comm-B, altitude reply
    case 21: // Comm-B, identity reply
        {
            if (long_syndrome == UNCHECKED_SYNDROME)
                long_syndrome = modesChecksum(corrected, MODES_LONG_MSG_BITS);
            int recent = icaoFilterTest(long_syndrome);
            return recent ? SR_UNRELIABLE_KNOWN : SR_UNRELIABLE_UNKNOWN;
        }

    case 24: // Comm-D (ELM)
    case 25:
    case 26:
    case 27:
    case 28:
    case 29:
    case 30:
    case 31:
        {
            if (!g_config.enable_df24)
                return SR_UNCORRECTABLE;
            if (long_syndrome == UNCHECKED_SYNDROME)
                long_syndrome = modesChecksum(corrected, MODES_LONG_MSG_BITS);
            int recent = icaoFilterTest(long_syndrome);
            return recent ? SR_UNRELIABLE_KNOWN : SR_UNRELIABLE_UNKNOWN;
        }

    case 11:
        {
            uint32_t addr = getbits(corrected, 9, 32);
            if (short_syndrome == UNCHECKED_SYNDROME)
                short_syndrome = modesChecksum(corrected, MODES_SHORT_MSG_BITS);
            uint32_t iid = short_syndrome & 0x7F;
            int recent = icaoFilterTest(addr);

            switch (corrections) {
            case 0:
                if (iid == 0)
                    return recent ? SR_DF11_ACQ_KNOWN : SR_DF11_ACQ_UNKNOWN;
                else
                    return recent ? SR_DF11_IID_KNOWN : SR_DF11_IID_UNKNOWN;
            case 1:
                if (iid == 0)
                    return recent ? SR_DF11_ACQ_1ERROR_KNOWN : SR_DF11_ACQ_1ERROR_UNKNOWN;
                else
                    return recent ? SR_DF11_IID_1ERROR_KNOWN : SR_DF11_IID_1ERROR_UNKNOWN;
            default:
                return SR_UNCORRECTABLE;
            }
        }

    case 17:   // Extended squitter
        {
            uint32_t addr = getbits(corrected, 9, 32);
            int recent = icaoFilterTest(addr);

            switch (corrections) {
            case 0:
                return recent ? SR_DF17_KNOWN : SR_DF17_UNKNOWN;
            case 1:
                return recent ? SR_DF17_1ERROR_KNOWN : SR_DF17_1ERROR_UNKNOWN;
            case 2:
                return recent ? SR_DF17_2ERROR_KNOWN : SR_DF17_2ERROR_UNKNOWN;
            default:
                return SR_UNCORRECTABLE;
            }
        }

    case 18:   // Extended squitter/non-transponder
        {
            uint32_t addr = getbits(corrected, 9, 32);
            int recent = icaoFilterTest(addr | ICAO_FILTER_ADSB_NT);

            switch (corrections) {
            case 0:
                return recent ? SR_DF18_KNOWN : SR_DF18_UNKNOWN;
            case 1:
                return recent ? SR_DF18_1ERROR_KNOWN : SR_DF18_1ERROR_UNKNOWN;
            case 2:
                return recent ? SR_DF18_2ERROR_KNOWN : SR_DF18_2ERROR_UNKNOWN;
            default:
                return SR_UNCORRECTABLE;
            }
        }

    default:
        return SR_UNKNOWN_DF;
    }
}

// Main demodulation function
void demod_2400_process(uint16_t *m, uint32_t mlen,
                        uint64_t timestamp,
                        mode_s_t *ctx,
                        mode_s_callback_t cb,
                        demod_2400_stats_t *stats)
{
    unsigned char msg1[MODES_LONG_MSG_BYTES], msg2[MODES_LONG_MSG_BYTES], *msg;
    uint32_t j;

    if (!g_initialized) {
        demod_2400_init(NULL);
    }

    unsigned char *bestmsg;
    int bestscore, bestphase;

    // We need lookahead of: 19 (preamble) + 1 + 269 (max message) samples
    // Caller must provide at least 289 samples of overlap
    assert(mlen >= 289);
    uint32_t process_len = mlen - 289;

    msg = msg1;

    for (j = 0; j < process_len; j++) {
        uint16_t *preamble = &m[j];
        int high;
        uint32_t base_signal, base_noise;
        int try_phase;
        int msglen;

        // Look for a message starting at around sample 0 with phase offset 3..7

        // quick check: we must have a rising edge 0->1 and a falling edge 12->13
        if (! (preamble[0] < preamble[1] && preamble[12] > preamble[13]) )
           continue;

        if (preamble[1] > preamble[2] &&                                       // 1
            preamble[2] < preamble[3] && preamble[3] > preamble[4] &&          // 3
            preamble[8] < preamble[9] && preamble[9] > preamble[10] &&         // 9
            preamble[10] < preamble[11]) {                                     // 11-12
            // peaks at 1,3,9,11-12: phase 3
            high = (preamble[1] + preamble[3] + preamble[9] + preamble[11] + preamble[12]) / 4;
            base_signal = preamble[1] + preamble[3] + preamble[9];
            base_noise = preamble[5] + preamble[6] + preamble[7];
        } else if (preamble[1] > preamble[2] &&                                // 1
                   preamble[2] < preamble[3] && preamble[3] > preamble[4] &&   // 3
                   preamble[8] < preamble[9] && preamble[9] > preamble[10] &&  // 9
                   preamble[11] < preamble[12]) {                              // 12
            // peaks at 1,3,9,12: phase 4
            high = (preamble[1] + preamble[3] + preamble[9] + preamble[12]) / 4;
            base_signal = preamble[1] + preamble[3] + preamble[9] + preamble[12];
            base_noise = preamble[5] + preamble[6] + preamble[7] + preamble[8];
        } else if (preamble[1] > preamble[2] &&                                // 1
                   preamble[2] < preamble[3] && preamble[4] > preamble[5] &&   // 3-4
                   preamble[8] < preamble[9] && preamble[10] > preamble[11] && // 9-10
                   preamble[11] < preamble[12]) {                              // 12
            // peaks at 1,3-4,9-10,12: phase 5
            high = (preamble[1] + preamble[3] + preamble[4] + preamble[9] + preamble[10] + preamble[12]) / 4;
            base_signal = preamble[1] + preamble[12];
            base_noise = preamble[6] + preamble[7];
        } else if (preamble[1] > preamble[2] &&                                 // 1
                   preamble[3] < preamble[4] && preamble[4] > preamble[5] &&    // 4
                   preamble[9] < preamble[10] && preamble[10] > preamble[11] && // 10
                   preamble[11] < preamble[12]) {                               // 12
            // peaks at 1,4,10,12: phase 6
            high = (preamble[1] + preamble[4] + preamble[10] + preamble[12]) / 4;
            base_signal = preamble[1] + preamble[4] + preamble[10] + preamble[12];
            base_noise = preamble[5] + preamble[6] + preamble[7] + preamble[8];
        } else if (preamble[2] > preamble[3] &&                                 // 1-2
                   preamble[3] < preamble[4] && preamble[4] > preamble[5] &&    // 4
                   preamble[9] < preamble[10] && preamble[10] > preamble[11] && // 10
                   preamble[11] < preamble[12]) {                               // 12
            // peaks at 1-2,4,10,12: phase 7
            high = (preamble[1] + preamble[2] + preamble[4] + preamble[10] + preamble[12]) / 4;
            base_signal = preamble[4] + preamble[10] + preamble[12];
            base_noise = preamble[6] + preamble[7] + preamble[8];
        } else {
            // no suitable peaks
            continue;
        }

        // Check for enough signal (~3.5dB SNR)
        if (base_signal * 2 < 3 * base_noise)
            continue;

        // Check that the "quiet" bits 6,7,15,16,17 are actually quiet
        if (preamble[5] >= (uint16_t)high ||
            preamble[6] >= (uint16_t)high ||
            preamble[7] >= (uint16_t)high ||
            preamble[8] >= (uint16_t)high ||
            preamble[14] >= (uint16_t)high ||
            preamble[15] >= (uint16_t)high ||
            preamble[16] >= (uint16_t)high ||
            preamble[17] >= (uint16_t)high ||
            preamble[18] >= (uint16_t)high) {
            continue;
        }

        // try all phases
        if (stats) stats->demod_preambles++;
        bestmsg = NULL; bestscore = SR_NOT_SET; bestphase = -1;
        for (try_phase = 4; try_phase <= 8; ++try_phase) {
            uint16_t *pPtr;
            int phase, score;

            // Decode all the next 112 bits, regardless of the actual message
            // size. We'll check the actual message type later

            pPtr = &m[j+19] + (try_phase/5);
            phase = try_phase % 5;

            unsigned bytelen = 1;
            for (unsigned i = 0; i < bytelen; ++i) {
                uint8_t theByte = 0;

                switch (phase) {
                case 0:
                    theByte =
                        (slice_phase0(pPtr) > 0 ? 0x80 : 0) |
                        (slice_phase2(pPtr+2) > 0 ? 0x40 : 0) |
                        (slice_phase4(pPtr+4) > 0 ? 0x20 : 0) |
                        (slice_phase1(pPtr+7) > 0 ? 0x10 : 0) |
                        (slice_phase3(pPtr+9) > 0 ? 0x08 : 0) |
                        (slice_phase0(pPtr+12) > 0 ? 0x04 : 0) |
                        (slice_phase2(pPtr+14) > 0 ? 0x02 : 0) |
                        (slice_phase4(pPtr+16) > 0 ? 0x01 : 0);

                    phase = 1;
                    pPtr += 19;
                    break;

                case 1:
                    theByte =
                        (slice_phase1(pPtr) > 0 ? 0x80 : 0) |
                        (slice_phase3(pPtr+2) > 0 ? 0x40 : 0) |
                        (slice_phase0(pPtr+5) > 0 ? 0x20 : 0) |
                        (slice_phase2(pPtr+7) > 0 ? 0x10 : 0) |
                        (slice_phase4(pPtr+9) > 0 ? 0x08 : 0) |
                        (slice_phase1(pPtr+12) > 0 ? 0x04 : 0) |
                        (slice_phase3(pPtr+14) > 0 ? 0x02 : 0) |
                        (slice_phase0(pPtr+17) > 0 ? 0x01 : 0);

                    phase = 2;
                    pPtr += 19;
                    break;

                case 2:
                    theByte =
                        (slice_phase2(pPtr) > 0 ? 0x80 : 0) |
                        (slice_phase4(pPtr+2) > 0 ? 0x40 : 0) |
                        (slice_phase1(pPtr+5) > 0 ? 0x20 : 0) |
                        (slice_phase3(pPtr+7) > 0 ? 0x10 : 0) |
                        (slice_phase0(pPtr+10) > 0 ? 0x08 : 0) |
                        (slice_phase2(pPtr+12) > 0 ? 0x04 : 0) |
                        (slice_phase4(pPtr+14) > 0 ? 0x02 : 0) |
                        (slice_phase1(pPtr+17) > 0 ? 0x01 : 0);

                    phase = 3;
                    pPtr += 19;
                    break;

                case 3:
                    theByte =
                        (slice_phase3(pPtr) > 0 ? 0x80 : 0) |
                        (slice_phase0(pPtr+3) > 0 ? 0x40 : 0) |
                        (slice_phase2(pPtr+5) > 0 ? 0x20 : 0) |
                        (slice_phase4(pPtr+7) > 0 ? 0x10 : 0) |
                        (slice_phase1(pPtr+10) > 0 ? 0x08 : 0) |
                        (slice_phase3(pPtr+12) > 0 ? 0x04 : 0) |
                        (slice_phase0(pPtr+15) > 0 ? 0x02 : 0) |
                        (slice_phase2(pPtr+17) > 0 ? 0x01 : 0);

                    phase = 4;
                    pPtr += 19;
                    break;

                case 4:
                    theByte =
                        (slice_phase4(pPtr) > 0 ? 0x80 : 0) |
                        (slice_phase1(pPtr+3) > 0 ? 0x40 : 0) |
                        (slice_phase3(pPtr+5) > 0 ? 0x20 : 0) |
                        (slice_phase0(pPtr+8) > 0 ? 0x10 : 0) |
                        (slice_phase2(pPtr+10) > 0 ? 0x08 : 0) |
                        (slice_phase4(pPtr+12) > 0 ? 0x04 : 0) |
                        (slice_phase1(pPtr+15) > 0 ? 0x02 : 0) |
                        (slice_phase3(pPtr+17) > 0 ? 0x01 : 0);

                    phase = 0;
                    pPtr += 20;
                    break;
                }

                msg[i] = theByte;

                if (i == 0) {
                    // inspect DF field early, only continue processing
                    // messages where the DF appears valid
                    unsigned df = theByte >> 3;
                    if (valid_df_long_bitset & (1 << df))
                        bytelen = MODES_LONG_MSG_BYTES;
                    else if (valid_df_short_bitset & (1 << df))
                        bytelen = MODES_SHORT_MSG_BYTES;
                }
            }

            if (bytelen == 1) {
                // rejected early by the DF filter
                if (stats) stats->demod_rejected_bad++;
                continue;
            }

            // Score the mode S message and see if it's any good.
            score = scoreModesMessage(msg);
            if (score > bestscore) {
                // new high score!
                bestmsg = msg;
                bestscore = score;
                bestphase = try_phase;

                // swap to using the other buffer
                msg = (msg == msg1) ? msg2 : msg1;
            }
        }

        // Do we have a candidate?
        if (bestscore < SR_ACCEPT_THRESHOLD) {
            if (stats) {
                if (bestscore >= SR_UNKNOWN_THRESHOLD)
                    stats->demod_rejected_unknown_icao++;
                else
                    stats->demod_rejected_bad++;
            }
            continue;
        }

        msglen = modesMessageLenByType(bestmsg[0] >> 3);

        // Add the ICAO address to our filter for future scoring
        unsigned df = bestmsg[0] >> 3;
        if (df == 11 || df == 17 || df == 18) {
            uint32_t addr = getbits(bestmsg, 9, 32);
            if (df == 18)
                addr |= ICAO_FILTER_ADSB_NT;
            icaoFilterAdd(addr);
        }

        // Create mode_s_msg structure and call the callback
        if (cb && ctx) {
            struct mode_s_msg mm;
            memset(&mm, 0, sizeof(mm));

            // Copy the message
            memcpy(mm.msg, bestmsg, msglen / 8);
            mm.msgbits = msglen;
            mm.msgtype = bestmsg[0] >> 3;

            // Compute CRC status
            uint32_t crc = modesChecksum(bestmsg, msglen);
            mm.crc = crc;
            mm.crcok = (crc == 0) || (df == 11 && (crc & 0xFFFF80) == 0);

            // ICAO address
            mm.aa1 = bestmsg[1];
            mm.aa2 = bestmsg[2];
            mm.aa3 = bestmsg[3];

            // Decode extended squitter fields (simplified version)
            if (mm.msgtype == 17) {
                mm.metype = bestmsg[4] >> 3;
                mm.mesub = bestmsg[4] & 7;
            }

            // Phase info
            mm.phase_corrected = 0;
            mm.errorbit = -1;

            // Count accepted messages
            if (stats) {
                int corrected = 0;
                if (bestscore >= SR_DF17_1ERROR_UNKNOWN && bestscore <= SR_DF17_1ERROR_KNOWN) corrected = 1;
                else if (bestscore >= SR_DF17_2ERROR_UNKNOWN && bestscore <= SR_DF17_2ERROR_KNOWN) corrected = 2;
                else if (bestscore >= SR_DF18_1ERROR_UNKNOWN && bestscore <= SR_DF18_1ERROR_KNOWN) corrected = 1;
                else if (bestscore >= SR_DF18_2ERROR_UNKNOWN && bestscore <= SR_DF18_2ERROR_KNOWN) corrected = 2;
                stats->demod_accepted[corrected]++;
            }

            // Call the user callback
            cb(ctx, &mm);
        }

        // Skip over the message
        j += (msglen + 8) * 12/5 - 8*12/5;
    }

    (void)timestamp;  // timestamp available for future MLAT support
    (void)bestphase;  // phase available for future timing refinement
}
