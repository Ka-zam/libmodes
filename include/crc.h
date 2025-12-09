// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// crc.h: Mode S checksum prototypes.
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

#ifndef LIBMODES_CRC_H
#define LIBMODES_CRC_H

#include <stdint.h>

// Global max for fixable bit errors
#define MODES_MAX_BITERRORS 2

struct errorinfo {
    uint32_t syndrome;                 // CRC syndrome
    int      errors;                   // number of errors
    int8_t   bit[MODES_MAX_BITERRORS]; // bit positions to fix (-1 = no bit)
};

// Initialize CRC tables. Call once at startup.
// fixBits: 0 = no error correction, 1 = 1-bit, 2 = 2-bit
void modesChecksumInit(int fixBits);

// Compute CRC-24 checksum of a message
// msg: message bytes
// bitlen: message length in bits (56 or 112)
// Returns: 24-bit CRC value
uint32_t modesChecksum(const uint8_t *msg, int bitlen);

// Look up error correction info for a given syndrome
// syndrome: CRC mismatch value
// bitlen: message length in bits (56 or 112)
// Returns: pointer to errorinfo, or NULL if uncorrectable
struct errorinfo *modesChecksumDiagnose(uint32_t syndrome, int bitlen);

// Apply error correction to a message
// msg: message bytes (modified in place)
// info: error correction info from modesChecksumDiagnose
void modesChecksumFix(uint8_t *msg, struct errorinfo *info);

#endif // LIBMODES_CRC_H
