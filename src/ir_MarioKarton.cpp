// Copyright 2020 David Conran (crankyoldgit)
/// @file
/// @brief Support for MarioKarton protocol

// Supports:
//   Brand: MarioKarton,  Model: TODO add device and remote

#include "IRrecv.h"
#include "IRsend.h"
#include "IRutils.h"

// WARNING: This probably isn't directly usable. It's a guide only.

// See https://github.com/crankyoldgit/IRremoteESP8266/wiki/Adding-support-for-a-new-IR-protocol
// for details of how to include this in the library.
const uint16_t kMarioKartonHdrMark = 2500;
const uint16_t kMarioKartonBitMark = 650;
const uint16_t kMarioKartonHdrSpace = 2000;
const uint16_t kMarioKartonOneSpace = 1650;
const uint16_t kMarioKartonZeroSpace = 550;
const uint16_t kMarioKartonFreq = 38000;  // Hz. (Guessing the most common frequency.)
//const uint16_t kMarioKartonBits = 2;  // Move to IRremoteESP8266.h
const uint16_t kMarioKartonOverhead = 3;
const uint16_t kMarioKartonGap = 41000;  // uSeconds

#if SEND_MARIOKARTON
// Function should be safe up to 64 bits.
/// Send a MarioKarton formatted message.
/// Status: ALPHA / Untested.
/// @param[in] data containing the IR command.
/// @param[in] nbits Nr. of bits to send. usually kMarioKartonBits
/// @param[in] repeat Nr. of times the message is to be repeated.
void IRsend::sendMarioKarton(const uint64_t data, const uint16_t nbits, const uint16_t repeat) {
  enableIROut(kMarioKartonFreq);
  for (uint16_t r = 0; r <= repeat; r++) {
    uint64_t send_data = data;
    // Header
    mark(kMarioKartonHdrMark);
    space(kMarioKartonHdrSpace);
    // Data Section #1
    // e.g. data = 0x1, nbits = 2
    sendData(kMarioKartonBitMark, kMarioKartonOneSpace, kMarioKartonBitMark, kMarioKartonZeroSpace, send_data, 2, true);
    send_data >>= 2;
    // Footer
    mark(kMarioKartonBitMark);
    space(kMarioKartonGap);  // A 100% made up guess of the gap between messages.
  }
}
#endif  // SEND_MARIOKARTON

#if DECODE_MARIOKARTON
// Function should be safe up to 64 bits.
/// Decode the supplied MarioKarton message.
/// Status: ALPHA / Untested.
/// @param[in,out] results Ptr to the data to decode & where to store the decode
/// @param[in] offset The starting index to use when attempting to decode the
///   raw data. Typically/Defaults to kStartOffset.
/// @param[in] nbits The number of data bits to expect.
/// @param[in] strict Flag indicating if we should perform strict matching.
/// @return A boolean. True if it can decode it, false if it can't.
bool IRrecv::decodeMarioKarton(decode_results *results, uint16_t offset, const uint16_t nbits, const bool strict) {
  if (results->rawlen < 2 * nbits + kMarioKartonOverhead - offset)
    return false;  // Too short a message to match.
  if (strict && nbits != kMarioKartonBits)
    return false;

  uint64_t data = 0;
  match_result_t data_result;

  // Header
  if (!matchMark(results->rawbuf[offset++], kMarioKartonHdrMark))
    return false;
  if (!matchSpace(results->rawbuf[offset++], kMarioKartonHdrSpace))
    return false;

  // Data Section #1
  // e.g. data_result.data = 0x1, nbits = 2
  data_result = matchData(&(results->rawbuf[offset]), 2,
                          kMarioKartonBitMark, kMarioKartonOneSpace,
                          kMarioKartonBitMark, kMarioKartonZeroSpace);
  offset += data_result.used;
  if (data_result.success == false) return false;  // Fail
  data <<= 2;  // Make room for the new bits of data.
  data |= data_result.data;

  // Footer
  if (!matchMark(results->rawbuf[offset++], kMarioKartonBitMark))
    return false;

  // Success
  results->decode_type = decode_type_t::MARIOKARTON;
  results->bits = nbits;
  results->value = data;
  results->command = 0;
  results->address = 0;
  return true;
}
#endif  // DECODE_MARIOKARTON