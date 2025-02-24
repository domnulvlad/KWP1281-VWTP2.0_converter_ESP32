/*

  Warning: measuring blocks (KWP2000 SID 0x21) are not handled according to standards, but for good reason.

  ---

  "KWP1281" will refer to "KWP1281-on-KLine", "VWTP"/"KWP2000" will refer to "KWP2000-on-TP2.0"

  ---

  Normally, via KWP2000, only blocks 1-254 are accessible, and blocks 128-254 are actually requested as 1-127.
  For example, requesting block 1 will provide the data for blocks 1 and 128 in the same message: 4 measurements for 1 and 4 measurements for 128.

  In the case of this converter, this would mean it would be necessary to request 2 blocks from KWP1281 when VWTP only requests one, which would be incredibly slow (was tested).
  For example, if the user wants to look at only block 1, both block 1 and 128 would be requested, because there is no way to know that the user isn't looking at 128 too.

  I decided to deviate from the standard, to achieve an acceptable refresh rate.
  As such, every VWTP block only provides at most 4 measurements (instead of 8), corresponding to one KWP1281 block.

  Additionally, in KWP1281, block 0, if supported, would contain a list of 10 raw bytes, as miscellaneous measurements.
  I find those quite useless, as there is no way to infer what they represent without checking technical documents for the specific module.
  So, block 0 from KWP1281 cannot be requested by this converter.
  Instead, requesting block 0 from VWTP (non-standard) will now provide the module's identification as an ASCII string (formula 0x5F), or "?" if the KWP1281 module is not yet connected.

  The rest of the blocks (1-255) are completely accessible, by requesting the exact block number from VWTP, without "mirrorring" (non-standard).
  To avoid delays/communication errors, VWTP block requests will receive a response instantly, even before the actual block was requested from KWP1281.
  Every block starts off as 4 blank measurements, and they get filled in with real data once it is requested from KWP1281.
  Then, after a block was requested from KWP1281 at least once, leaving it and coming back in VWTP will provide its last values for a moment, not blank measurements again.

  Because the measurement response is copied directly from KWP1281 to VWTP, issues may arise for some formulas that were phased out by KWP2000.
  An example is 0xA0 (value with variable scaling and units), I noticed it is not handled properly by diagnostic software when received via VWTP.

  ---

  Another point to mention is fault codes.
  The codes themselves are exactly the same between KWP1281 and KWP2000, so they are copied directly.
  The elaboration code, however, has different meanings between the protocols.
  KWP1281 elaborations are much more detailed, so I made a lookup table to convert the code so that it keeps most of its meaning, but it's nothing more than an approximation.

  Fault codes are requested from KWP1281 immediately after connecting to it, so that they are instantly available when requested via VWTP.
  Each VWTP fault code request causes fault codes to be requested again from KWP1281.
  So, in a way, whenever fault codes are requested from VWTP, the response contains the "previous" fault codes.
  Again, this is to avoid incredible delays/connection errors that would arise if VWTP had to wait for the KWP1281 request to finish.
  And yes, I checked, the VWTP negative response with reason 0x78 ("please wait a bit for my response") DOES NOT cause diagnostic software to wait at all, at least for the one I tested.

  ---

  Considering diagnostic software can only read blocks 1-127 from this converter, it's best when paired with a DIY custom diagnostic tool.
  This would mean you could also change the fault code elaboration conversion so that the original KWP1281 byte is sent via VWTP, then your software can display it properly as if it was coming from KWP1281.

  Since the emulated VWTP module's identification strings never change ("KWP1281<->VWTP2.0  " etc., editable in the file "responses.h"), your software can check for them.
  So, if your software detects this converter, it knows that it is allowed to request real blocks 0-255 and the elaboration code should be handled differently, if you decided to change it as described above.
  Also, additional care should be taken when handling measurements, for formulas that shouldn't be supported by KWP2000 but which may appear in KWP1281.
  
  ---
  
  Functions not implemented: recoding, output tests, basic settings, adaptation, login.
  The coding value and workshop ID are not used or converted, I didn't find them important.
  
  ---
  
  There are some modules that can be accessed via KWP1281 but not via VWTP.
  If you wish to convert such a module, edit the table `MODULE_ID_TO_LOGICAL_ID` in "vwtp.h" in this project's folder, or modify the code which uses that table.
  
*/

/*

  This app has been tested on an ESP32-C6 development board, using its integrated CAN interface along with a CAN transceiver.
  It should be possible to change it to use an MCP2515 interface instead.
  All CAN functions are defined in "can.h" and communication with other tasks is done by FreeRTOS queues.

*/

// Select the parameters of the KWP1281 the converter should connect to (this will also choose which module is emulated via VWTP).
uint8_t K_LINE_MODULE = 0x01;
uint16_t K_LINE_MODULE_BAUD_RATE = 10400;

// Enable debugging by uncommenting the following lines.
//#define DEBUG_MISC
//#define DEBUG_CAN_TRAFFIC
//#define DEBUG_KLINE_TRAFFIC
//#define DEBUG_VWTP_TRAFFIC

// Files
#include "autodefines.h"
#include "kline.h"
#include "can.h"
#include "vwtp.h"
#include "responses.h"
#include "lists.h"

// Helper to calculate how many items an array can hold
#ifndef ARRAYSIZE
#define ARRAYSIZE(a) ((sizeof(a) / sizeof(*(a))) / static_cast<size_t>(!(sizeof(a) % sizeof(*(a)))))
#endif

// Shared "VWTP module info" resource
SemaphoreHandle_t VWTP_module_info_mutex;
bool global__VWTP_is_connected = false;

// Task that processes frames received on Diagnostic CAN
void diagnostic_CAN_provider_task(void *arg)
{
  // The frames arrive on the queue in CAN_frame format.
  CAN_frame frame;

  while (true)
  {
    // Wait for a frame to arrive on the queue.
    xQueueReceive(diagnostic_CAN_receive_queue, &frame, portMAX_DELAY);

    // CAN ID 200 with length 7 is used for TP2.0 dynamic diagnostic channel requests.
    if (frame.can_id == 0x200 && frame.can_dlc == 7)
    {
      // Only respond to the request if it is directed at the emulated module.
      if (!(K_LINE_MODULE < sizeof(MODULE_ID_TO_LOGICAL_ID) && MODULE_ID_TO_LOGICAL_ID[K_LINE_MODULE] == frame.data[0]))
      {
        DEBUG("Request is for another module");
        continue;
      }

      // The "requested sending ID" from the channel setup frame represents a sort of "session", of which multiple can be opened at the same time.
      // The ID used by most diagnostic software is 300, but sessions with IDs 301, 302, etc. can be opened at the same time, but not for the same module.
      unsigned long VWTP_requested_sending_id = (frame.data[5] << 8) | frame.data[4];

      // Take the mutex, to access the shared "VWTP module info" resource (to check if VWTP is connected and its sending ID).
      xSemaphoreTake(VWTP_module_info_mutex, portMAX_DELAY);

      // Copy the info locally.
      bool local__VWTP_is_connected = global__VWTP_is_connected;
      unsigned long VWTP_send_ID = diag_VWTP.getSendID();

      // Release the mutex.
      xSemaphoreGive(VWTP_module_info_mutex);

      // If VWTP is connected and its sending ID matches the dynamic request, wake it up from waiting for a message, to be able to respond as soon as possible.
      if (local__VWTP_is_connected && (VWTP_requested_sending_id == VWTP_send_ID))
      {
        diag_VWTP.wakeupFromRead();
      }

      // Put the frame on the queue.
      xQueueSend(CAN_dynamic_diagnostics_channel_message_queue, (void*)&frame, 0);
    }
    // Any other ID is given to the VWTP library instance.
    else if (frame.can_id != 0x200)
    {
      diag_VWTP.provideMessage(frame.can_id, frame.can_dlc, frame.data, NULL);
    }
  }
}

// Shared "KWP module info" resource
SemaphoreHandle_t KWP_module_info_mutex;
bool global__KWP_is_connected = false;
char global__KWP_module_part_number[13];
char global__KWP_module_identification[25];
char global__KWP_module_extra_identification[37];

// Shared "KWP fault codes" resource
SemaphoreHandle_t KWP_fault_codes_mutex;
uint8_t global__KWP_fault_code_buffer[255];
uint8_t global__KWP_amount_of_fault_codes;
uint8_t global__KWP_error_reading_fault_codes;
bool global__KWP_need_to_request_fault_codes = true;
bool global__KWP_need_to_clear_fault_codes = false;

// Shared "KWP measuring blocks" resource
SemaphoreHandle_t KWP_measuring_blocks_mutex;
KWP1281_measuring_block_t global__KWP_measuring_blocks[256];

void init_KWP_measuring_blocks()
{
  // Initialize every structure in the array.
  for (size_t i = 0; i < ARRAYSIZE(global__KWP_measuring_blocks); i++)
  {
    // Set its ID.
    global__KWP_measuring_blocks[i].block_id = i;

    // Each block starts with the assumption that it exists and sends all-blank measurements until actual measurements are read.
    global__KWP_measuring_blocks[i].exists = true;

    // Copy an empty response for each measurement in the block.
    static const uint8_t empty_measurement[3] = {0x25, 0x00, 0x00};
    for (uint8_t j = 0; j < 4; j++)
    {
      memcpy(&global__KWP_measuring_blocks[i].data[sizeof(empty_measurement) * j], empty_measurement, sizeof(empty_measurement));
    }

    // Set the size of the block response.
    global__KWP_measuring_blocks[i].length = 4 * sizeof(empty_measurement);
  }
}

// Function called when a connection error occurs on the K-line
void KWP1281customErrorFunction(uint8_t module, unsigned long baud)
{
  // The provided arguments are not used.
  (void)module;
  (void)baud;

  DEBUG("KWP module error occurred");

  // Indicate that the KWP module is not connected anymore.
  xSemaphoreTake(KWP_module_info_mutex, portMAX_DELAY);
  global__KWP_is_connected = false;
  xSemaphoreGive(KWP_module_info_mutex);
}

// If a connection error occurs, the errorFunction defined above will unset the global connection flag.
// The function during which the error occurred will then return KLineKWP1281Lib::ERROR, that is when this function is mostly called.
bool KWP_had_connection_error()
{
  // Check if the KWP module is actually connected.
  xSemaphoreTake(KWP_module_info_mutex, portMAX_DELAY);
  bool local__KWP_is_connected = global__KWP_is_connected;
  xSemaphoreGive(KWP_module_info_mutex);
  return !local__KWP_is_connected;
}

// Task that manages the K-line
void kline_task(void *arg)
{
  // Local variable, storing the connection state of the module
  static bool local__KWP_is_connected = false;

  // Local variables, used for requesting fault codes
  static uint8_t local__KWP_amount_of_fault_codes = 0;
  static uint8_t local__KWP_fault_code_buffer[sizeof(global__KWP_fault_code_buffer)];

  // Local variables, used for copying global buffers
  static unsigned long local__KWP_measuring_blocks_last_request_ms[sizeof(global__KWP_currently_active_measuring_blocks)];
  static bool local__KWP_measuring_blocks_exist[sizeof(global__KWP_currently_active_measuring_blocks)];
  static uint8_t local__KWP_currently_active_measuring_blocks[sizeof(global__KWP_currently_active_measuring_blocks)];

  // Local variables, used for requesting measuring blocks
  static uint8_t local__KWP_amount_of_measurements = 0;
  static uint8_t local__KWP_measurement_buffer[sizeof(global__KWP_measuring_blocks[0].data)];

  while (true)
  {
    // If not currently connected to the module, attempt to connect.
    if (!local__KWP_is_connected)
    {
      DEBUG("Attempting to connect to %02X @ %d", K_LINE_MODULE, K_LINE_MODULE_BAUD_RATE);
      if (diag_K.attemptConnect(K_LINE_MODULE, K_LINE_MODULE_BAUD_RATE) == KLineKWP1281Lib::SUCCESS)
      {
        DEBUG("Connected to K-line");

        // Update the local flag to indicate the module is currently connected.
        local__KWP_is_connected = true;

        // Take the mutex, to access the shared "KWP module info" resource.
        xSemaphoreTake(KWP_module_info_mutex, portMAX_DELAY);

        // Update the global flag.
        global__KWP_is_connected = true;

        // Update the global strings.
        strcpy(global__KWP_module_part_number, diag_K.getPartNumber());
        strcpy(global__KWP_module_identification, diag_K.getIdentification());
        strcpy(global__KWP_module_extra_identification, diag_K.getExtraIdentification());

        // Release the mutex.
        xSemaphoreGive(KWP_module_info_mutex);

        // After connecting successfully, read the module's fault codes, to have them ready when they are requested.
        KLineKWP1281Lib::executionStatus readFaults_status = diag_K.readFaults(local__KWP_amount_of_fault_codes, local__KWP_fault_code_buffer, sizeof(local__KWP_fault_code_buffer));

        // If the function reports a communication error, check if the module has disconnected.
        if (readFaults_status == KLineKWP1281Lib::ERROR && KWP_had_connection_error())
        {
          // Update the local flag to indicate the module is currently disconnected, and skip over the rest of the functions.
          local__KWP_is_connected = false;
          continue;
        }

        // Take the mutex, to access the shared "KWP fault codes" resource.
        xSemaphoreTake(KWP_fault_codes_mutex, portMAX_DELAY);

        // Check if the fault codes were read successfully.
        if (readFaults_status == KLineKWP1281Lib::SUCCESS)
        {
          // Update the global flag, to indicate reading fault codes was successful.
          global__KWP_error_reading_fault_codes = false;

          // Update the global fault code count and buffer.
          global__KWP_amount_of_fault_codes = local__KWP_amount_of_fault_codes;
          memcpy(global__KWP_fault_code_buffer, local__KWP_fault_code_buffer, 3 * local__KWP_amount_of_fault_codes);
        }
        else
        {
          // Update the global flag, to indicate reading fault codes failed.
          global__KWP_error_reading_fault_codes = true;
        }

        // Update the global flag, to indicate it is not neccessary to request fault codes.
        global__KWP_need_to_request_fault_codes = false;

        // Release the mutex.
        xSemaphoreGive(KWP_fault_codes_mutex);
      }

      // Delay failed connection attempts by 1 second.
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // If currently connected to the module, manage it.
    else
    {
      // Maintain the connection with the module, in case no other requests have to be made.
      diag_K.update();
      // A possible optimization would be only calling update() if really nothing needs to be requested,
      // as this function introduces a small delay between measuring block requests.
      // Additional logic would need to be added for KWP1281 "header+body" measurement responses,
      // because they require update() to be called when requesting a new block, to force the header to be sent.
      // Those responses are not handled by this converter.

      // Check if the module has disconnected during the previous function.
      if (KWP_had_connection_error())
      {
        // Update the local flag to indicate the module is currently disconnected, and skip over the rest of the functions.
        local__KWP_is_connected = false;
        continue;
      }

      // Take the mutex, to access the shared "KWP measuring blocks" resource.
      xSemaphoreTake(KWP_measuring_blocks_mutex, portMAX_DELAY);

      // Copy the global array of active measuring blocks locally.
      memcpy(local__KWP_currently_active_measuring_blocks, global__KWP_currently_active_measuring_blocks, sizeof(global__KWP_currently_active_measuring_blocks));

      // Copy the details of each currently active block.
      size_t currently_active_measuring_blocks_list_count = get_list_count(local__KWP_currently_active_measuring_blocks, sizeof(local__KWP_currently_active_measuring_blocks));
      for (size_t i = 0; i < currently_active_measuring_blocks_list_count; i++)
      {
        uint8_t block = local__KWP_currently_active_measuring_blocks[i];
        local__KWP_measuring_blocks_last_request_ms[i] = global__KWP_measuring_blocks[block].last_request_ms;
        local__KWP_measuring_blocks_exist[i] = global__KWP_measuring_blocks[block].exists;
      }

      // Release the mutex (it is not held during requesting blocks from the module).
      xSemaphoreGive(KWP_measuring_blocks_mutex);

      // Request each active block.
      for (size_t i = 0; i < currently_active_measuring_blocks_list_count; i++)
      {
        // Get the current block in the list.
        uint8_t block = local__KWP_currently_active_measuring_blocks[i];

        // Calculate how many milliseconds have passed since the last time the current block was requested.
        unsigned long ms_since_last_request = millis() - local__KWP_measuring_blocks_last_request_ms[i];

        // If too much time passed since the last request, this block is no longer active.
        if (ms_since_last_request > KWP1281_ACTIVE_BLOCK_THRESHOLD_MS)
        {
          // Take the mutex, to access the shared "KWP measuring blocks" resource.
          xSemaphoreTake(KWP_measuring_blocks_mutex, portMAX_DELAY);

          // Remove the block from the list of active blocks.
          if (remove_from_list(global__KWP_currently_active_measuring_blocks, sizeof(global__KWP_currently_active_measuring_blocks), block))
          {
            DEBUG("#Removed block %d from active list after %lums", block, ms_since_last_request);
          }

          // Release the mutex.
          xSemaphoreGive(KWP_measuring_blocks_mutex);

          // Skip requesting this block.
          DEBUG("Skipping block %d because it went inactive", block);
          continue;
        }

        // If the current block is somehow known not to be requestable from the module, skip it (though it shouldn't be "active" in the first place).
        if (!local__KWP_measuring_blocks_exist[i])
        {
          DEBUG("#Error: should not be possible for block in active list not to exist");
          continue;
        }

        // Request the block.
        KLineKWP1281Lib::executionStatus readGroup_status = diag_K.readGroup(local__KWP_amount_of_measurements, block, local__KWP_measurement_buffer, sizeof(local__KWP_measurement_buffer));

        // Take the mutex, to access the shared "KWP measuring blocks" resource.
        xSemaphoreTake(KWP_measuring_blocks_mutex, portMAX_DELAY);

        // Check if the measurement block was read successfully.
        if (readGroup_status == KLineKWP1281Lib::SUCCESS)
        {
          // Count how many data bytes the block contains.
          uint8_t response_bytes = 0;
          for (uint8_t i = 0; i < local__KWP_amount_of_measurements; i++)
          {
            // Add one for the "formula" byte.
            response_bytes++;

            // Add the bytes used by the rest of the measurement.
            response_bytes += KLineKWP1281Lib::getMeasurementDataLength(i, local__KWP_amount_of_measurements, local__KWP_measurement_buffer, sizeof(local__KWP_measurement_buffer));
          }

          // Update the fields of the global structure.
          DEBUG("#Block %d contains %d measurements, %d bytes", block, local__KWP_amount_of_measurements, response_bytes);
          global__KWP_measuring_blocks[block].length = response_bytes;
          memcpy(global__KWP_measuring_blocks[block].data, local__KWP_measurement_buffer, response_bytes);
          global__KWP_measuring_blocks[block].last_refresh_ms = millis();
        }
        else if (readGroup_status == KLineKWP1281Lib::FAIL)
        {
          // Update the global flag in the structure, to indicate this measuring block does not exist on the module.
          DEBUG("#Found out that block %d does not exist", block);
          global__KWP_measuring_blocks[block].exists = false;

          // Remove the block from the list of active blocks.
          if (remove_from_list(global__KWP_currently_active_measuring_blocks, sizeof(global__KWP_currently_active_measuring_blocks), block))
          {
            DEBUG("#Removed block %d from active list because it does not exist", block);
          }
        }
        // If the block response is of the unsupported "Basic Settings" format, store an ASCII message instead of values.
        else if (readGroup_status == KLineKWP1281Lib::GROUP_BASIC_SETTINGS)
        {
          // Update the fields of the global structure.
          global__KWP_measuring_blocks[block].length = sizeof(KWP_unsupported_block_format_response_BSC) - 1;
          memcpy(global__KWP_measuring_blocks[block].data, KWP_unsupported_block_format_response_BSC, global__KWP_measuring_blocks[block].length);
          global__KWP_measuring_blocks[block].last_refresh_ms = millis();
        }
        // If the block response is of the unsupported "Header" format, store an ASCII message instead of values.
        else if (readGroup_status == KLineKWP1281Lib::GROUP_HEADER || readGroup_status == KLineKWP1281Lib::GROUP_BODY)
        {
          // Update the fields of the global structure.
          global__KWP_measuring_blocks[block].length = sizeof(KWP_unsupported_block_format_response_HDR) - 1;
          memcpy(global__KWP_measuring_blocks[block].data, KWP_unsupported_block_format_response_HDR, global__KWP_measuring_blocks[block].length);
          global__KWP_measuring_blocks[block].last_refresh_ms = millis();
        }

        // If the function reports a communication error, check if the module has disconnected.
        if (readGroup_status == KLineKWP1281Lib::ERROR && KWP_had_connection_error())
        {
          // Update the local flag to indicate the module is currently disconnected, and skip over the rest of the functions.
          local__KWP_is_connected = false;
          continue;
        }

        // Release the mutex.
        xSemaphoreGive(KWP_measuring_blocks_mutex);
      }

      // Take the mutex, to access the shared "KWP fault codes" resource (to check if it's necessary to request fault codes).
      xSemaphoreTake(KWP_fault_codes_mutex, portMAX_DELAY);

      // Copy the global flags locally.
      bool local__KWP_need_to_clear_fault_codes = global__KWP_need_to_clear_fault_codes;
      bool local__KWP_need_to_request_fault_codes = global__KWP_need_to_request_fault_codes;

      // Release the mutex.
      xSemaphoreGive(KWP_fault_codes_mutex);

      // If necessary, clear the fault codes.
      if (local__KWP_need_to_clear_fault_codes)
      {
        KLineKWP1281Lib::executionStatus clearFaults_status = diag_K.clearFaults();

        // If the function reports a communication error, check if the module has disconnected.
        if (clearFaults_status == KLineKWP1281Lib::ERROR && KWP_had_connection_error())
        {
          // Update the local flag to indicate the module is currently disconnected, and skip over the rest of the functions.
          local__KWP_is_connected = false;
          continue;
        }

        // After clearing the fault codes, force a request (this will also clear the flag).
        local__KWP_need_to_request_fault_codes = true;
      }

      // If necessary, read the fault codes.
      if (local__KWP_need_to_request_fault_codes)
      {
        KLineKWP1281Lib::executionStatus readFaults_status = diag_K.readFaults(local__KWP_amount_of_fault_codes, local__KWP_fault_code_buffer, sizeof(local__KWP_fault_code_buffer));

        // Take the mutex, to access the shared "KWP fault codes" resource.
        xSemaphoreTake(KWP_fault_codes_mutex, portMAX_DELAY);

        // Check if the fault codes were read successfully.
        if (readFaults_status == KLineKWP1281Lib::SUCCESS)
        {
          // Update the global flag, to indicate reading fault codes was successful.
          global__KWP_error_reading_fault_codes = false;

          // Update the global fault code count and buffer.
          global__KWP_amount_of_fault_codes = local__KWP_amount_of_fault_codes;
          memcpy(global__KWP_fault_code_buffer, local__KWP_fault_code_buffer, 3 * local__KWP_amount_of_fault_codes);
        }
        else
        {
          // Update the global flag, to indicate reading fault codes failed.
          global__KWP_error_reading_fault_codes = true;
        }

        // Update the global flags.
        global__KWP_need_to_request_fault_codes = false;
        global__KWP_need_to_clear_fault_codes = false;

        // Release the mutex.
        xSemaphoreGive(KWP_fault_codes_mutex);

        // If the function reports a communication error, check if the module has disconnected.
        if (readFaults_status == KLineKWP1281Lib::ERROR && KWP_had_connection_error())
        {
          // Update the local flag to indicate the module is currently disconnected, and skip over the rest of the functions.
          local__KWP_is_connected = false;
          continue;
        }
      }
    }
  }
}

// Executed when a request to clear fault codes is received via CAN
void VWTP_clear_fault_codes(VWTPLib *instance)
{
  // Update the global flag, to tell the KWP task to clear the fault codes from the module.
  xSemaphoreTake(KWP_fault_codes_mutex, portMAX_DELAY);
  global__KWP_need_to_clear_fault_codes = true;
  xSemaphoreGive(KWP_fault_codes_mutex);

  // Send a positive response.
  instance->send(VWTP_clear_fault_codes_response, sizeof(VWTP_clear_fault_codes_response));
}

// Executed when a request to read fault codes is received via CAN
void VWTP_provide_fault_codes(VWTPLib *instance, uint8_t *buffer)
{
  // Copy the response "template" into the given buffer.
  memcpy(buffer, VWTP_fault_codes_response_header, sizeof(VWTP_fault_codes_response_header));

  // Take the mutex, to access the shared "KWP fault codes" resource.
  xSemaphoreTake(KWP_fault_codes_mutex, portMAX_DELAY);

  // Copy the global variables locally.
  bool local__KWP_error_reading_fault_codes = global__KWP_error_reading_fault_codes;
  uint8_t local__KWP_amount_of_fault_codes = global__KWP_amount_of_fault_codes;

  // If the KWP task has not yet requested/cleared fault codes after it was instructed to, send a "no fault codes" response.
  if (global__KWP_need_to_request_fault_codes || global__KWP_need_to_clear_fault_codes)
  {
    local__KWP_amount_of_fault_codes = 0;
  }
  // If fault codes were retrieved from the module, copy them into the response.
  else if (local__KWP_amount_of_fault_codes)
  {
    memcpy(&buffer[4], global__KWP_fault_code_buffer, 3 * local__KWP_amount_of_fault_codes);

    // For each fault code, convert its elaboration code.
    for (uint8_t i = 0; i < local__KWP_amount_of_fault_codes; i++)
    {
      // Retrieve the elaboration code.
      uint8_t &elaboration_code = buffer[6 + i * 3];

      // Determine if the fault is "intermittent".
      bool fault_is_intermittent = false;
      if (elaboration_code & 0x80)
      {
        fault_is_intermittent = true;
      }
      elaboration_code &= ~0x80;

      // Convert the elaboration code.
      if (elaboration_code < sizeof(KWP1281_to_KWP2000_fault_status_table))
      {
        elaboration_code = KWP1281_to_KWP2000_fault_status_table[elaboration_code];
      }
      else
      {
        elaboration_code = 0;
      }

      // If the fault is "intermittent" in KWP1281, convert it to "Active" in KWP2000.
      elaboration_code |= 0b00010000;
      if (!fault_is_intermittent)
      {
        elaboration_code |= 0b01000000;
      }
    }
  }

  // Update the global flag, to tell the KWP task to request fault codes.
  global__KWP_need_to_request_fault_codes = true;

  // Release the mutex.
  xSemaphoreGive(KWP_fault_codes_mutex);

  // If there was an error reading fault codes from the KWP module, send a negative VWTP response.
  if (local__KWP_error_reading_fault_codes)
  {
    VWTP_send_negative_response(instance, SID_readFaultCodes);
  }
  // Otherwise, send the fault codes.
  else
  {
    // Calculate the response length.
    uint16_t response_length = 2 + 3 * local__KWP_amount_of_fault_codes;

    // Update the response.
    buffer[0] = (response_length >> 8);
    buffer[1] = (response_length & 0xFF);
    buffer[3] = local__KWP_amount_of_fault_codes;

    // Send the positive response.
    instance->send(buffer, response_length + 2);
  }
}

// Executed when a request to read a measuring block is received via CAN
void VWTP_provide_measuring_block(VWTPLib *instance, uint8_t block, uint8_t *buffer)
{
  // This variable will contain the length of the response to be sent.
  uint16_t local__measuring_block_response_length = 0;

  // Copy the response "template" into the given buffer.
  memcpy(buffer, VWTP_measuring_block_response_header, sizeof(VWTP_measuring_block_response_header));

  // If block 0 was requested, send the KWP module's identification as ASCII text.
  if (block == 0)
  {
    // Take the mutex, to access the shared "KWP module info" resource.
    xSemaphoreTake(KWP_module_info_mutex, portMAX_DELAY);

    // Copy the global variables locally.
    bool local__KWP_is_connected = global__KWP_is_connected;
    static char local__KWP_module_part_number[13];
    strcpy(local__KWP_module_part_number, global__KWP_module_part_number);
    static char local__KWP_module_identification[25];
    strcpy(local__KWP_module_identification, global__KWP_module_identification);
    static char local__KWP_module_extra_identification[37];
    strcpy(local__KWP_module_extra_identification, global__KWP_module_extra_identification);

    // Release the mutex.
    xSemaphoreGive(KWP_module_info_mutex);

    // If the KWP module is not connected, a single question mark will be sent as ASCII.
    static char identification_string[sizeof(local__KWP_module_part_number) - 1 + sizeof(local__KWP_module_identification) - 1 + 1];
    if (!local__KWP_is_connected)
    {
      strcpy(identification_string, "?");
    }
    // If it is connected, its identification strings will be sent as ASCII.
    // The "extra identification" is not sent, because the string becomes too long for dianostic software to display, but you could add it.
    else
    {
      strcpy(identification_string, local__KWP_module_part_number);
      strcat(identification_string, local__KWP_module_identification);
    }

    // Update the response.
    size_t string_length = strlen(identification_string);
    buffer[4] = 0x5F;
    buffer[5] = string_length;
    memcpy(&buffer[6], identification_string, string_length);

    // The length of measurement data counts the SID, PID (0), formula (0x5F), counter (string_length), and `string_length` bytes.
    local__measuring_block_response_length = 4 + string_length;
  }
  // A block other than 0 was requested.
  else
  {
    // Copy the global flag locally (to check if the KWP module is connected).
    xSemaphoreTake(KWP_module_info_mutex, portMAX_DELAY);
    bool local__KWP_is_connected = global__KWP_is_connected;
    xSemaphoreGive(KWP_module_info_mutex);

    // If the module isn't connected via K-line, send a negative response immediately.
    if (!local__KWP_is_connected)
    {
      DEBUG("No module, can't send blocks other than 0");
      VWTP_send_negative_response(instance, SID_readDataByLocalIdentifier);
      return;
    }

    // Copy the global flag locally (to check if the requested block exists).
    xSemaphoreTake(KWP_measuring_blocks_mutex, portMAX_DELAY);
    bool _measuring_block_exists = global__KWP_measuring_blocks[block].exists;
    xSemaphoreGive(KWP_measuring_blocks_mutex);

    // If it is known that the block doesn't exist, send a negative response immediately.
    if (!_measuring_block_exists)
    {
      DEBUG("Block %d does not exist", block);
      VWTP_send_negative_response(instance, SID_readDataByLocalIdentifier);
      return;
    }

    // Take the mutex, to access the shared "KWP measuring blocks" resource.
    xSemaphoreTake(KWP_measuring_blocks_mutex, portMAX_DELAY);

    // Store the time at which the current request is being made.
    global__KWP_measuring_blocks[block].last_request_ms = millis();

    // If the current block isn't in the list of "active" blocks, add it.
    if (!is_in_list(global__KWP_currently_active_measuring_blocks, sizeof(global__KWP_currently_active_measuring_blocks), block))
    {
      if (add_to_list(global__KWP_currently_active_measuring_blocks, sizeof(global__KWP_currently_active_measuring_blocks), block))
      {
        DEBUG("Added block %d to active list", block);
      }
    }

    // Copy the block's data from the structure into the response.
    memcpy(&buffer[4], global__KWP_measuring_blocks[block].data, global__KWP_measuring_blocks[block].length);

    // The length of measurement data counts the SID, PID (block number), and `length` bytes.
    local__measuring_block_response_length = 2 + global__KWP_measuring_blocks[block].length;

    // Release the mutex.
    xSemaphoreGive(KWP_measuring_blocks_mutex);
  }

  // Update the length in the response.
  buffer[0] = ((local__measuring_block_response_length >> 8) & 0xFF);
  buffer[1] = (local__measuring_block_response_length & 0xFF);

  // Update the block number in the response.
  buffer[3] = block;

  // Send the response.
  DEBUG("Sending block %d", block);
  instance->send(buffer, local__measuring_block_response_length + 2);
}

// Task that manages VWTP
void vwtp_task(void *arg)
{
  CAN_frame frame;
  bool local__VWTP_is_connected = false;

  uint8_t local__VWTP_receive_buffer[4096];
  uint16_t local__VWTP_received_bytes;

  uint8_t local__VWTP_measuring_block_response[259];
  uint8_t local__VWTP_fault_codes_response[259];

  VWTPLib *instance = &diag_VWTP;

  bool vwtp_forced_wakeup_received = false;

  while (true)
  {
    // Check for a TP2.0 dynamic diagnostic channel setup message.
    // If connected to VWTP, the queue is checked only "briefly".
    // If not connected (or VWTP was forced to wake up from reading), WAIT until a message arrives on the queue.
    if (xQueueReceive(CAN_dynamic_diagnostics_channel_message_queue, &frame, ((local__VWTP_is_connected && !vwtp_forced_wakeup_received) ? 0 : portMAX_DELAY)))
    {
      // Unset the flag.
      vwtp_forced_wakeup_received = false;

      // Regular TP2.0 dynamic channel setup messages have CAN ID 200 and are 7 bytes long.
      if (frame.can_id == 0x200 && frame.can_dlc == 7)
      {
        DEBUG("Dynamic channel message received by VWTP task");

        // Extract the "session ID".
        unsigned long VWTP_requested_sending_id = (frame.data[5] << 8) | frame.data[4];

        // Construct the response.
        static uint8_t dynamic_channel_response[] = {0x00, 0x00, 0x00, 0x03, 0x2E, 0x03, 0x01};

        // Fill the response with the VWTP instance's details.
        dynamic_channel_response[4] = diag_VWTP.getReceiveID() & 0xFF;
        dynamic_channel_response[5] = diag_VWTP.getReceiveID() >> 8;

        // The ID on which the response is sent is calculated from the module ID (first byte in request).
        unsigned long response_send_id = 0x200 + frame.data[0];

        // Get the VWTP instance's current sending ID.
        xSemaphoreTake(VWTP_module_info_mutex, portMAX_DELAY);
        unsigned long VWTP_send_ID = diag_VWTP.getSendID();
        xSemaphoreGive(VWTP_module_info_mutex);

        // The CAN provider task already checked that the dynamic channel setup message is for this module.
        bool need_to_send_dynamic_channel_response = false;
        if (local__VWTP_is_connected)
        {
          // If VWTP is already connected, send a negative response to all requests with different "session IDs".
          if (VWTP_requested_sending_id != VWTP_send_ID)
          {
            dynamic_channel_response[1] = 0xD8; // CHANNEL_NO_RESOURCES
            dynamic_channel_response[2] = VWTP_requested_sending_id & 0xFF;
            dynamic_channel_response[3] = VWTP_requested_sending_id >> 8;

            VWTP_sendFunction(response_send_id, sizeof(dynamic_channel_response), dynamic_channel_response);
            DEBUG("Sent negative channel response");
          }
          // Receiving a setup message for the same session that is already connected shouldn't happen, but handle it anyways.
          else
          {
            need_to_send_dynamic_channel_response = true;
          }
        }
        // If VWTP is not connected, change the isntance's sending ID to what was specified in the setup message, send a positive response and connect.
        else
        {
          xSemaphoreTake(VWTP_module_info_mutex, portMAX_DELAY);
          DEBUG("Changing send ID to %03lX", VWTP_requested_sending_id);
          diag_VWTP.changeSendID(VWTP_requested_sending_id);
          xSemaphoreGive(VWTP_module_info_mutex);

          need_to_send_dynamic_channel_response = true;
        }

        // Send a positive response and connect to VWTP.
        if (need_to_send_dynamic_channel_response)
        {
          // Send the response.
          dynamic_channel_response[1] = 0xD0; // CHANNEL_OK
          dynamic_channel_response[2] = VWTP_send_ID & 0xFF;
          dynamic_channel_response[3] = VWTP_send_ID >> 8;
          VWTP_sendFunction(response_send_id, sizeof(dynamic_channel_response), dynamic_channel_response);

          // Try to start the VWTP session, at most 5 times.
          DEBUG("Connecting to VWTP");
          uint8_t attempt_counter = 0;
          while (true)
          {
            // If connecting is successful, update the flag and stop attempting.
            if (diag_VWTP.attemptConnect(true))
            {
              DEBUG("Connected to VWTP");

              // Set the local flag.
              local__VWTP_is_connected = true;

              // Set the global flag.
              xSemaphoreTake(VWTP_module_info_mutex, portMAX_DELAY);
              global__VWTP_is_connected = true;
              xSemaphoreGive(VWTP_module_info_mutex);
              break;
            }

            // If too many attempts failed, stop attempting.
            if (attempt_counter++ >= 5)
            {
              DEBUG("Failed to connect to VWTP");
              break;
            }
          }
        }
      }
      // An ID of 0 is invalid; instead, it's used to notify this task that the VWTP session ended (sent by the VWTPterminationFunction).
      else if (frame.can_id == 0)
      {
        DEBUG("Disconnect message received by VWTP task");

        // Update the local flag.
        local__VWTP_is_connected = false;

        // Update the global flag.
        xSemaphoreTake(VWTP_module_info_mutex, portMAX_DELAY);
        global__VWTP_is_connected = false;
        xSemaphoreGive(VWTP_module_info_mutex);
      }
      else
      {
        DEBUG("Unknown frame received by VWTP task");
      }
    }

    // If VWTP is connected, manage it.
    if (local__VWTP_is_connected)
    {
      // Wait until a message is received ("forever").
      if ((local__VWTP_received_bytes = diag_VWTP.receive(local__VWTP_receive_buffer, sizeof(local__VWTP_receive_buffer), true, -1)))
      {
        // Extract the SID and PID bytes from the response.
        uint8_t SID_byte = local__VWTP_receive_buffer[2];
        uint8_t PID_byte = local__VWTP_receive_buffer[3];

        // Process "measuring block" requests separately.
        if (SID_byte == SID_readDataByLocalIdentifier)
        {
          VWTP_provide_measuring_block(instance, PID_byte, local__VWTP_measuring_block_response);
          continue;
        }

        // Process "read fault codes" requests separately (not when requesting supported codes, a negative response will be sent for that).
        if (SID_byte == SID_readFaultCodes && PID_byte != readFaultCodes_getSupportedFaultCodes)
        {
          VWTP_provide_fault_codes(instance, local__VWTP_fault_codes_response);
          continue;
        }

        // Process "clear fault codes" requests separately.
        if (SID_byte == SID_clearFaultCodes)
        {
          VWTP_clear_fault_codes(instance);
          continue;
        }

        // Search for the received SID in the list of supported services.
        bool found_SID = false;
        bool found_PID = false;
        for (size_t i = 0; i < ARRAYSIZE(known_responses); i++)
        {
          // If the SID was found, search for the received PID in the list of supported responses.
          if (known_responses[i].SID == SID_byte)
          {
            for (size_t j = 0; j < known_responses[i].amount_of_PIDs; j++)
            {
              // If the PID was found, send the response.
              if (known_responses[i].PIDs[j] == PID_byte)
              {
                DEBUG("Sending response for SID %02X, PID %02X", SID_byte, PID_byte);
                diag_VWTP.send(known_responses[i].responses[j], known_responses[i].response_sizes[j]);

                // Indicate that the PID was found.
                found_PID = true;
                break;
              }
            }

            if (!found_PID)
            {
              DEBUG("Unknown PID %02X for SID %02X", PID_byte, SID_byte);
            }

            // Indicate that the SID was found.
            found_SID = true;
            break;
          }
        }

        if (!found_SID)
        {
          DEBUG("Unknown SID %02X", SID_byte);
        }

        // If the response was not found, send a negative response.
        if (!found_SID || !found_PID)
        {
          VWTP_send_negative_response(instance, SID_byte);
        }
      }
      // Because receive() has the last argument -1, it should wait "forever" (until a message is received).
      // If 0 bytes are returned by it, it means it stopped waiting because of a forced wakeup by the CAN provider task.
      else
      {
        DEBUG("Forced wakeup, wait on queue");
        vwtp_forced_wakeup_received = true;
      }
    }
  }
}

void setup()
{
  DEBUG("KWP1281<->VWTP2.0 ESP32");

  // If enabled, attach the debug function to the K-line library.
#ifdef DEBUG_KLINE_TRAFFIC
  diag_K.KWP1281debugFunction(KWP1281debugFunction);
#endif

  // Attach the custom error function to the K-line library.
  diag_K.customErrorFunction(KWP1281customErrorFunction);

  // Create the mutexes used for protecting shared resources.
  VWTP_module_info_mutex = xSemaphoreCreateMutex();
  KWP_module_info_mutex = xSemaphoreCreateMutex();
  KWP_measuring_blocks_mutex = xSemaphoreCreateMutex();
  KWP_fault_codes_mutex = xSemaphoreCreateMutex();

  // Initialize the global array of measuring block structures.
  init_KWP_measuring_blocks();

  // Start CAN.
  init_CAN();

  // If enabled, attach the debug function to the VWTP library.
#ifdef DEBUG_VWTP_TRAFFIC
  diag_VWTP.VWTPdebugFunction(VWTPdebugFunction);
#endif

  // Attach the termination function to the VWTP library.
  diag_VWTP.terminationFunction(VWTPterminationFunction);

  // Start the task responsible for sending CAN frames.
  diagnostic_CAN_send_queue = xQueueCreate(10, sizeof(CAN_frame));
  xTaskCreate(CAN_send_task, "CANs", 4096, NULL, 12, NULL);

  // Start the task responsible for receiving CAN frames.
  diagnostic_CAN_receive_queue = xQueueCreate(10, sizeof(CAN_frame));
  xTaskCreate(CAN_receive_task, "CANr", 4096, NULL, 8, NULL);

  // Initalize the queue used for sending TP2.0 dynamic channel setup request messages to the VWTP task.
  CAN_dynamic_diagnostics_channel_message_queue = xQueueCreate(10, sizeof(CAN_frame));

  // Start the task responsible for processing received CAN frames.
  xTaskCreate(diagnostic_CAN_provider_task, "CANp", 4096, NULL, 10, NULL);

  // Create the event group used for notifying the KWP library that a byte was received via K-line.
  kline_event_group = xEventGroupCreate();

  // Start the task responsible for managing the K-line.
  xTaskCreate(kline_task, "K", 4096, NULL, 7, NULL);

  // Start the task responsible for managing VWTP.
  xTaskCreate(vwtp_task, "VWTP", 8192, NULL, 11, NULL);
}

void loop()
{
  // The Arduino loop is not used; delete its task.
  vTaskDelete(NULL);
}
