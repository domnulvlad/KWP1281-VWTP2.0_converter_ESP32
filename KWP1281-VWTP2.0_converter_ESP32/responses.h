#ifndef RESPONSES_H
#define RESPONSES_H

// Known KWP2000 SIDs and their PIDs
#include "KWP2000_SID.h"

// Negative response template for any unknown request
uint8_t negative_response[] =
{
  0x00, 0x03, // Length
  0x7F, 0,    // Refuse, SID
  0x11        // Reason
};

void VWTP_send_negative_response(VWTPLib *instance, uint8_t SID)
{
  // Set the SID and send the negative response.
  negative_response[3] = SID;
  instance->send(negative_response, sizeof(negative_response));
}

uint8_t session_89_response[] =
{
  0x00, 0x02,                                                                           // Length
  SID_startDiagnosticSession + POSITIVE_RESPONSE, diagnosticMode_standardDiagnosticMode // SID, SID
};

uint8_t identification_91_response[] =
{
  0x00, 0x11,                                                                       // Length
  SID_readEcuIdentification + POSITIVE_RESPONSE, ECUIdentification_hardwareNumber,  // SID, PID
  0x0E,                                                                             // Constant
  'H', 'W', '0', '0', '0', ' ', ' ', ' ', ' ', ' ', ' ',                            // Hardware number
  0x20, 0x20, 0xFF                                                                  // Constants
};

uint8_t identification_9B_response[] =
{
  0x00, 0x30,                                                                                   // Length
  SID_readEcuIdentification + POSITIVE_RESPONSE, ECUIdentification_standardIdentification,      // SID, PID
  'K', 'W', 'P', 'E', 'M', 'U', '0', '0', '1', ' ', ' ',                                        // Software version
  0x20,                                                                                         // Constant
  '0', '0', '0', '0',                                                                           // Programming status
  0x00,                                                                                         // Coding (00 - no coding, 03 - short coding, 10 - long coding)
  0x00,                                                                                         // Constant
  0x00, 0x00,                                                                                   // Short coding (high byte, low byte)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                                                           // WSC, Imp., Geraet
  'K', 'W', 'P', '1', '2', '8', '1', '<', '-', '>', 'V', 'W', 'T', 'P', '2', '.', '0', ' ', ' ' // Identification
};

// Supported functions (through this message, the KWP2000 module tells the diagnostic interface which functions it supports)
/*
  0000 - no function supported
  0101 - basic settings in KWP1281 mode
  0102 - output tests with fixed sequence in KWP1281 mode
  0104 - 20-bit coding possible
  0105 - Coding-II possible
  0106 - reading measuring blocks in KWP1281 mode
  0107 - output tests with selective (non-fixed) sequence
  0108 - developer functions possible
  010D - immobilizer Gen4 supported
  0118 - HEX-coded fault codes supported
*/
uint8_t supported_functions_response[] =
{
  0x00, 0x04,                                                                          // Length
  SID_startRoutineByLocalIdentifier + POSITIVE_RESPONSE, routineLocalIdentifier_START, // SID, PID
  0x01, 0x06,                                                                          // Supported function: KWP1281-style measuring blocks
};

// Structure used for registering automatic responses to SIDs+PIDs
struct known_SID
{
  uint8_t SID;

  uint8_t *PIDs;
  size_t amount_of_PIDs;

  uint8_t **responses;
  size_t *response_sizes;
};

// Automatic responses for SID_readEcuIdentification
uint8_t known_PIDs_1A[] = {ECUIdentification_hardwareNumber, ECUIdentification_standardIdentification};
uint8_t *responses_1A[] = {identification_91_response, identification_9B_response};
size_t response_sizes_1A[] = {sizeof(identification_91_response), sizeof(identification_9B_response)};
known_SID known_SID_1A =
{
  SID_readEcuIdentification, known_PIDs_1A, sizeof(known_PIDs_1A), responses_1A, response_sizes_1A
};

// Automatic responses for SID_startDiagnosticSession
uint8_t known_PIDs_10[] = {diagnosticMode_standardDiagnosticMode};
uint8_t *responses_10[] = {session_89_response};
size_t response_sizes_10[] = {sizeof(session_89_response)};
known_SID known_SID_10 =
{
  SID_startDiagnosticSession, known_PIDs_10, sizeof(known_PIDs_10), responses_10, response_sizes_10
};

// Automatic responses for SID_startRoutineByLocalIdentifier
uint8_t known_PIDs_31[] = {0xB8};
// Warning: this SID is used for output tests; if they are enabled (in the `supported_functions_response`), the routine number must also be checked when sending a response.
// So, `supported_functions_response` must only be sent when the diagnostic interface requests to "start" the routine with number 0x0000.
// But for this app, it doesn't matter, output tests are disabled, so routine 0x0000 is the only one that can appear.
uint8_t *responses_31[] = {supported_functions_response};
size_t response_sizes_31[] = {sizeof(supported_functions_response)};
known_SID known_SID_31 =
{
  SID_startRoutineByLocalIdentifier, known_PIDs_31, sizeof(known_PIDs_31), responses_31, response_sizes_31
};

// Store the automatic response structures in an array.
known_SID known_responses[] =
{
  known_SID_1A,
  known_SID_10,
  known_SID_31,
};

static uint8_t VWTP_clear_fault_codes_response[] =
{
  0x00, 0x03,                              // Length
  SID_clearFaultCodes + POSITIVE_RESPONSE, // SID
  0xFF, 0x00,                              // DTC group
};

static uint8_t VWTP_fault_codes_response_header[] =
{
  0x00, 0x00,                             // Length
  SID_readFaultCodes + POSITIVE_RESPONSE, // SID
  0x00,                                   // DTC count
                                          // DTCs: High, Low, Status
};

static uint8_t VWTP_measuring_block_response_header[] =
{
  0x00, 0x00,                                        // Length
  SID_readDataByLocalIdentifier + POSITIVE_RESPONSE, // SID
  0,                                                 // Block
};

struct KWP1281_measuring_block_t
{
  uint8_t block_id;
  bool exists;
  uint8_t data[255];
  uint8_t length;
  unsigned long last_request_ms; // when the block was last requested via VWTP
  unsigned long last_refresh_ms; // when the data in the block was last retrieved from KWP1281
};

#define KWP1281_ACTIVE_BLOCK_THRESHOLD_MS 1000 // after how much time a block is no longer considered "active"
uint8_t global__KWP_currently_active_measuring_blocks[10]; // list of 5 "active" block numbers

// Lookup table for approximating a KWP1281 fault elaboration code to KWP2000
uint8_t KWP1281_to_KWP2000_fault_status_table[] =
{ /*         0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F */
  /* 0 */ 0x00, 0x06, 0x07, 0x04, 0x03, 0x0B, 0x01, 0x02, 0x01, 0x01, 0x02, 0x02, 0x01, 0x02, 0x01, 0x02,
  /* 1 */ 0x08, 0x08, 0x01, 0x02, 0x05, 0x0E, 0x0E, 0x0E, 0x0E, 0x08, 0x0B, 0x08, 0x06, 0x07, 0x0A, 0x09,
  /* 2 */ 0x01, 0x02, 0x00, 0x00, 0x0B, 0x0E, 0x06, 0x07, 0x0C, 0x04, 0x01, 0x0E, 0x0C, 0x03, 0x0E, 0x04,
  /* 3 */ 0x08, 0x04, 0x03, 0x03, 0x01, 0x02, 0x05, 0x05, 0x0E, 0x0C, 0x03, 0x03, 0x0E, 0x0E, 0x05, 0x01,
  /* 4 */ 0x0F, 0x0F, 0x0F, 0x03, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0C, 0x0C, 0x04, 0x0D,
  /* 5 */ 0x0C, 0x0B, 0x0B
};

// Response sent in case a "basic settings" response is received when requesting measuring blocks from KWP1281
// This kind of response (I believe) is only sent when requesting group 0 on some modules, so this converter should never receive it.
static const uint8_t KWP_unsupported_block_format_response_BSC[] =
  "\x5F" "\x15"
  "BLK-BSC not supported";

// Response sent in case a "header/body" response is received when requesting measuring blocks from KWP1281
static const uint8_t KWP_unsupported_block_format_response_HDR[] =
  "\x5F" "\x15"
  "BLK-HDR not supported";

#endif
