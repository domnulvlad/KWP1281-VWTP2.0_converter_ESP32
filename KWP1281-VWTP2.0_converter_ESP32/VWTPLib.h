#ifndef VWTPLib_H
#define VWTPLib_H

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"

// DEBUG OPTIONS:
//#define VWTP_SHOW_INFO

// ADJUSTMENT OPTIONS:
#define VWTP_DELAY 5     // default milliseconds to wait between CAN messages
#define VWTP_TIMEOUT 100 // default milliseconds to wait for an acknowledgement/response
#define T_WAIT 100       // milliseconds to wait before repeating a not-acknowledged message

//////////////////////////////////////////////////////////////////////////////////////////

class VWTPLib
{
public:
  // Function pointer types for callbacks
  using terminationFunction_type = void (*)(VWTPLib *instance);                                                                  // terminationFunction
  using sendFunction_type = bool (*)(unsigned long can_id, uint8_t can_dlc, uint8_t *data);                                      // sendFunction
  using VWTPdebugFunction_type = void (*)(bool type, unsigned long delta, unsigned long can_id, uint8_t can_dlc, uint8_t *data); // debugFunction

  // Constructor
  VWTPLib(sendFunction_type send_function, unsigned long send_id, unsigned long recv_id);

  // Attach a function to be called for debugging VWTP CAN messages
  void VWTPdebugFunction(VWTPdebugFunction_type debug_function);

  // Manually set the amount of milliseconds to wait between CAN frames
  void setMessageDelay(uint8_t message_delay);
  // Manually set the amount of milliseconds to wait for receiving acknowledgement CAN frames
  void setMessageTimeout(uint8_t message_timeout);

  // Get the CAN ID used for sending
  unsigned long getSendID();
  // Get the CAN ID used for receiving
  unsigned long getReceiveID();

  // Change the CAN ID used for sending
  void changeSendID(unsigned long send_id);
  // Change the CAN ID used for receiving
  void changeReceiveID(unsigned long recv_id);

  void enableRepeatedResendError(bool execute_termination_function = false);
  void disableRepeatedResendError();
  bool checkRepeatedResendError();

  // Attempt to connect to the module
  bool attemptConnect(bool passive = false);
  // Connect to the module
  void connect(bool passive = false);
  // Check if a connection is established
  bool isConnected();

  // Maintain the connection with the module
  void update();
  // Break the connection with the module
  bool disconnect(bool wait_for_response = true, bool send_message = true);

  // Send a block (byte array); if a message interrupts the transmission, save it in a buffer
  uint16_t send(uint8_t *block, uint8_t block_length, uint8_t *buffer, uint16_t buffer_size);
  // Send a block (constant byte array); if a message interrupts the transmission, save it in a buffer
  uint16_t send(const uint8_t *block, uint8_t block_length, uint8_t *buffer, uint16_t buffer_size);
  // Send a block (byte array)
  uint16_t send(const uint8_t *block, uint8_t block_length);
  // Send a block (constant byte array)
  uint16_t send(uint8_t *block, uint8_t block_length);

  bool isMessageValid(unsigned long can_id);
  // Provide the Transport Protocol with a CAN message (must be called in the requestFunction)
  bool provideMessage(unsigned long can_id, uint8_t can_dlc, uint8_t *data, bool *message_accepted = nullptr);

  // Receive a message, saving it in a byte array
  uint16_t receive(uint8_t *buffer, uint16_t buffer_size, bool wait_for_message = true, unsigned long timeout = 0);

  void wakeupFromRead();

  // Define a function to be executed when the connection is terminated (because of an error)
  void terminationFunction(terminationFunction_type termination_function);

private:
  // FreeRTOS objects
  EventGroupHandle_t receiveEvent;
  TickType_t xLastWakeTime;
  SemaphoreHandle_t copyingMessageMutex;

  // Return values for internal functions
  enum _ERROR
  {
    _ERROR_OK,
    _ERROR_RQ,
    _ERROR_PR,
    _ERROR_ACK,
    _ERROR_NAK,
    _ERROR_TIMEOUT,
    _ERROR_MESSAGE,
    _ERROR_NO_MESSAGE,
    _ERROR_SENDING_ERROR,
    _ERROR_VWTP_TERMINATED
  };

  // Types of debugging events for printing info
  enum DEBUG_TYPE
  {
    CONNECTED,
    NOT_CONNECTED,
    DISCONNECTED,

    NULL_SIZE_SEND,
    NULL_SIZE_BLOCK,
    IMPOSSIBLE_BUFFER_INDEX,

    CONNECTION_TERMINATED,
    MODULE_DISCONNECTED,

    FRAME_MISSED,
    FRAME_NAK,

    CHECK_CONNECTION,
    CONNECTION_OK,
    CONNECTION_NOT_OK
  };

  // Receive buffer
  struct
  {
    unsigned long can_id;
    uint8_t can_dlc;
    uint8_t data[8];
  } _received_message;

  // Function pointers for sending/receiving CAN frames
  sendFunction_type _send_function;

  // Flag to check if the user called the public receive() function while the library was checking for incoming messages before sending.
  bool user_called_receive_function = false;

  // User-chosen CAN IDs on which communication takes place
  unsigned long _send_id, _recv_id;

  // Timing parameters
  unsigned long _message_delay = VWTP_DELAY, _message_timeout = VWTP_TIMEOUT;
  bool _manual_message_delay = false, _manual_message_timeout = false;

  // Powers of 10 for timing parameter calculation
  uint8_t pow10[4] = {0, 1, 10, 100};

  // Pointers to user-defined functions
  terminationFunction_type _termination_function = nullptr;
  VWTPdebugFunction_type _debug_function = nullptr;

  // Internal flags
  bool _connected = false;
  bool _resend_frame = false;
  bool _wait_for_nak = false;
  bool _executing_termination = false;

  //"Default value" for anything whose value must be checked for changes
  const uint8_t _default_value = 0xFF;

  // Timer for delaying after sending CAN messages
  unsigned long _last_frame_time = 0;

  // Timer for debugging
  unsigned long _debug_time = 0;

  // Timer for timeout detection
  unsigned long _timeout_timer = 0;

  bool repeated_resend_error_enabled, repeated_resend_error_execute_termination, repeated_resend_error_occurred;
  uint8_t ACK_counter, NAK_counter, FC_counter;

  /*
    A block refers to a group of CAN frames which transport a message.

    If the message contains <= 7 bytes, a single-block is formed (containing one frame).
    If it contains > 7 bytes, a multi-block is formed (containing as many frames as necessary).
    If, during transmission, the block limit of the device is exceeded, a flow control frame is used.
    This limit is expressed by the device when connecting.
  */

  // PCI (protocol control information) types
  const uint8_t PCI_FF = 0x1;  // FIRST/FINAL FRAME  - first frame for single-blocks, last for multi-blocks - needs acknowledgement
  const uint8_t PCI_CF = 0x2;  // CONSECUTIVE FRAME  - intermediate message in a block                      - doesn't need acknowledgement
  const uint8_t PCI_LF = 0x3;  // LAST FRAME         - seldom used (usually, FF is used instead of LF)      - doesn't need acknowledgement
  const uint8_t PCI_FC = 0x0;  // FLOW CONTROL       - intermediate message in a long block                 - needs acknowledgement
  const uint8_t PCI_ACK = 0xB; // frame acknowledged - ready for more
  const uint8_t PCI_NAK = 0x9; // frame acknowledged - temporarily not ready for more
  const uint8_t PCI_CMD = 0xA; // channel command    - used for setting up a VWTP channel

  // VWTP channel messages
  const uint8_t VWTP_REQUEST_PARAMETERS[6] = {0xA0, 0x0F, 0x8A, 0xFF, 0x4A, 0xFF};
  const uint8_t VWTP_PROVIDE_PARAMETERS[6] = {0xA1, 0x0F, 0x8A, 0xFF, 0x4A, 0xFF};
  const uint8_t VWTP_KEEP_ALIVE[1] = {0xA3};
  const uint8_t VWTP_DISCONNECT[1] = {0xA8};

  // VWTP sequence counters
  uint8_t seq_outbound, seq_inbound, ack_seq_expected, msg_seq_expected, TP_max_block_length = 0xF;

  // Internal buffer for receiving messages
  uint8_t _receive_buffer[7];

  /// FUNCTIONS:

  // Send a constant byte array over CAN
  bool _send_raw(const uint8_t *frame, uint8_t size);
  // Send a byte array over CAN
  bool _send_raw(uint8_t *frame, uint8_t size);

  // Add a sequence to the first byte in a constant byte array, send it, and save any incoming message in a buffer
  _ERROR _send_with_sequence(const uint8_t *frame, uint8_t size, uint8_t *buffer, uint16_t buffer_size, uint16_t &received_message_size, bool is_first_frame = false);
  // Add a sequence to the first byte in a byte array, send it, and save any incoming message in a buffer
  _ERROR _send_with_sequence(uint8_t *frame, uint8_t size, uint8_t *buffer, uint16_t buffer_size, uint16_t &received_message_size, bool is_first_frame = false);

  // Receive messages, respond to them if necessary, and save them in a buffer
  _ERROR _receive(uint8_t *buffer, uint16_t buffer_size, uint16_t &received_message_size, bool wait_for_message = true, unsigned long timeout = 0, bool take_precedence = false);

  // Acknowledge messages
  bool _acknowledge(uint8_t seq);

  // Respond to VWTP-command messages
  _ERROR _respond(uint8_t opcode);

  // Wait for the module's disconnect response
  bool _wait_for_disconnect_response();

  // Determine if a sequence is in the range defined by two other sequences (returning its position)
  uint8_t _seq_position_in_block(uint8_t value, uint8_t lower, uint8_t upper, bool &is_in_range);

  // Executed when the connection is terminated unexpectadly
  void _on_termination(bool also_disconnect = true, bool wait_for_disconnect_response = true);

  // Check if the recipient is still connected
  bool _check_connection(bool &got_ack);

  // Show debug information in the Serial Monitor
  void show_debug_info(DEBUG_TYPE type);
};

#endif
