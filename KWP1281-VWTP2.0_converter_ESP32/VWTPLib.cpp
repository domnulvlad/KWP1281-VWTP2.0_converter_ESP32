#include "VWTPLib.h"

// ˚. ✦.✧ No words can describe this mess ·˖✶ ⋆.
// The plan was to release this as a standalone library, but... eh
// It's very patched-together and also has some hardcoded things specifically for the KWP1281-TP2.0 converter app

VWTPLib::VWTPLib(sendFunction_type send_function, unsigned long send_id, unsigned long recv_id) : _send_function(send_function),
  _send_id(send_id),
  _recv_id(recv_id)
{
  xLastWakeTime = xTaskGetTickCount();
  assert(receiveEvent = xEventGroupCreate());
  assert(copyingMessageMutex = xSemaphoreCreateMutex());
}

void VWTPLib::VWTPdebugFunction(VWTPdebugFunction_type debug_function)
{
  _debug_function = debug_function;
}

void VWTPLib::setMessageDelay(uint8_t message_delay)
{
  if (message_delay)
  {
    _manual_message_delay = true;
    _message_delay = message_delay;
  }
  else
  {
    _manual_message_delay = false;
    _message_delay = VWTP_DELAY;
  }
}

void VWTPLib::setMessageTimeout(uint8_t message_timeout)
{
  if (message_timeout)
  {
    _manual_message_timeout = true;
    _message_timeout = message_timeout;
  }
  else
  {
    _manual_message_timeout = false;
    _message_timeout = VWTP_TIMEOUT;
  }
}

unsigned long VWTPLib::getSendID()
{
  return _send_id;
}

unsigned long VWTPLib::getReceiveID()
{
  return _recv_id;
}

void VWTPLib::changeSendID(unsigned long send_id)
{
  _send_id = send_id;
}

void VWTPLib::changeReceiveID(unsigned long recv_id)
{
  _recv_id = recv_id;
}

void VWTPLib::enableRepeatedResendError(bool execute_termination_function)
{
  repeated_resend_error_enabled = true;
  repeated_resend_error_execute_termination = execute_termination_function;
  repeated_resend_error_occurred = false;
  NAK_counter = 0;
}

void VWTPLib::disableRepeatedResendError()
{
  repeated_resend_error_enabled = false;
  repeated_resend_error_occurred = false;
}

bool VWTPLib::checkRepeatedResendError()
{
  if (repeated_resend_error_occurred)
  {
    repeated_resend_error_occurred = false;
    return true;
  }
  return false;
}

bool VWTPLib::attemptConnect(bool passive)
{
  // Clear the event bit, so a message received before is "forgotten".
  xEventGroupClearBits(receiveEvent, 1);

  // Reset the sequence counters.
  seq_inbound = seq_outbound = msg_seq_expected = 0;

  // Disable responding to messages before a connection is established.
  _connected = false;

  uint16_t msg_size;           // variable where the received message's size will be stored
  uint8_t attempt_counter = 0; // counter for retrying if a disconnect request is received

  // If a disconnect request is received as a response to the connection establishment request, retry up to 10 times.
  while (true)
  {
    // Send a connection establishment request.
    if (!passive)
    {
      if (!_send_raw(VWTP_REQUEST_PARAMETERS, sizeof(VWTP_REQUEST_PARAMETERS)))
      {
        return false;
      }
    }

    bool send_again = false;
    while (true)
    {
      // Attempt to receive a connection establishment response.
      switch (_receive(_receive_buffer, sizeof(_receive_buffer), msg_size)) // the received message is stored in the global buffer
      {
        // The expected response was received:
        case _ERROR_PR:
        case _ERROR_RQ:
          // Save the maximum block size.
          TP_max_block_length = _receive_buffer[1];

          // If the received message is 6 bytes long, it contains extra timing parameters.
          if (msg_size == 6)
          {
            uint8_t message_timeout_value = _receive_buffer[2] & 0x3F;      // T1 parameter value
            uint8_t message_timeout_scale = pow10[_receive_buffer[2] >> 6]; // T1 parameter scaling
            uint8_t message_delay_value = _receive_buffer[4] & 0x3F;        // T3 parameter value
            uint8_t message_delay_scale = pow10[_receive_buffer[4] >> 6];   // T3 parameter scaling

            uint8_t message_timeout, message_delay; // T1 = time to wait for a response, T3 = time to wait between frames

            // If the T1 scaling value isn't 0, multiply the value by it.
            if (message_timeout_scale)
            {
              message_timeout = message_timeout_value * message_timeout_scale;
            }
            // If it is 0, divide the value by 10.
            else
            {
              message_timeout = message_timeout_value / 10;
            }

            // If the T3 scaling value isn't 0, multiply the value by it.
            if (message_delay_scale)
            {
              message_delay = message_delay_value * message_delay_scale;
            }
            // If it is 0, divide the value by 10.
            else
            {
              message_delay = message_delay_value / 10;
            }

#ifdef VWTP_SHOW_INFO
            printf("VWTP: received T1=%dms; T3=%dms\n", message_timeout, message_delay);
#endif

            // If the message timeout wasn't set using the setMessageTimeout() function, assign it the calculated value.
            if (!_manual_message_timeout)
            {
              _message_timeout = message_timeout;
            }

            // If the message delay wasn't set using the setMessageDelay() function, assign it the calculated value.
            if (!_manual_message_delay)
            {
              _message_delay = message_delay;
            }
          }

#ifdef VWTP_SHOW_INFO
          printf("VWTP: using T1=%lums; T3=%lums\n", _message_timeout, _message_delay);
#endif

          // Indicate that the connection was established successfully.
          show_debug_info(CONNECTED);
          _connected = true;
          return true;

        case _ERROR_VWTP_TERMINATED: // if a disconnect request is received, try to establish a connection again
          if (attempt_counter < 10)
          {
            attempt_counter++;
            send_again = true;

#ifdef VWTP_SHOW_INFO
            printf("VWTP got Disconnect in attemptConnect, but trying again\n");
#endif
            break;
          }
          else
          {
            disconnect(false); // disconnect (without waiting for a response)
            return false;
          }
          break;

        case _ERROR_TIMEOUT:
#ifdef VWTP_SHOW_INFO
          printf("VWTP got timeout in attemptConnect\n");
#endif
          return false;

        case _ERROR_ACK:
        case _ERROR_NAK:
          break;

        default: // nothing was received, or something else than what was expected
          show_debug_info(NOT_CONNECTED);
          return false;
      }

      if (send_again)
      {
        break;
      }
    }
  }
}

void VWTPLib::connect(bool passive)
{
  while (!attemptConnect(passive))
    ;
}

bool VWTPLib::isConnected()
{
  return _connected;
}

void VWTPLib::update()
{
  uint16_t msg_size;                                                   // dummy variable
  _receive(_receive_buffer, sizeof(_receive_buffer), msg_size, false); // check for messages, responding to them automatically
}

bool VWTPLib::disconnect(bool wait_for_response, bool send_message)
{
  if (!_connected)
  {
    show_debug_info(NOT_CONNECTED);
    return true;
  }

  // Send a disconnect request.
  if (send_message)
  {
    if (!_send_raw(VWTP_DISCONNECT, sizeof(VWTP_DISCONNECT)))
    {
      return false;
    }
  }

  // Disable responding to messages.
  _connected = false;

  // If requested, wait for the module's response before proceeding.
  if (wait_for_response)
  {
    _wait_for_disconnect_response();
  }

  show_debug_info(DISCONNECTED);

  // Reset the sequence counters.
  seq_inbound = seq_outbound = msg_seq_expected = 0;

  return true;
}

uint16_t VWTPLib::send(const uint8_t *block, uint8_t block_length, uint8_t *buffer, uint16_t buffer_size)
{
  return send((uint8_t *)block, block_length, buffer, buffer_size); // call the function which takes a non-const byte array
}

uint16_t VWTPLib::send(uint8_t *block, uint8_t block_length)
{
  // Use the internal buffer if one isn't provided.
  return send(block, block_length, _receive_buffer, sizeof(_receive_buffer));
}

uint16_t VWTPLib::send(const uint8_t *block, uint8_t block_length)
{
  // Use the internal buffer if one isn't provided.
  return send((uint8_t *)block, block_length, _receive_buffer, sizeof(_receive_buffer)); // call the function which takes a non-const byte array
}

uint16_t VWTPLib::send(uint8_t *block, uint8_t block_length, uint8_t *buffer, uint16_t buffer_size)
{
  if (!block_length) // if the block has a length of 0, exit
  {
    show_debug_info(NULL_SIZE_SEND);
    return 0;
  }

  // Variables for splitting a block into frames:
  uint8_t frame[8];                                  // will hold the current frame to be sent
  uint8_t number_of_frames = (block_length + 6) / 7; // calculate into how many frames the block will be split
  uint8_t FC_frames = 0;                             // will keep track of how many "flow control" frames have been sent so far
  uint8_t bytes_copied = 0;                          // will keep track of how many bytes have been sent so far

  // Variables for retransmitting frames in case of errors:
  bool sent_once = false;                                               // for detecting if the current frame was sent already (meaning it is being resent)
  uint8_t starting_block_from_frame = 0;                                // will change from 0 if a frame is being resent (to only retransmit the part that was missed)
  ack_seq_expected = _default_value;                                    // set the expected sequence to a default value so its change can be detected if a frame is missed
  uint8_t starting_sequence = seq_outbound % 16;                        // lower bound of the range of sequences carried by the frames
  uint8_t ending_sequence = (seq_outbound + number_of_frames - 1) % 16; // upper bound of the range of sequences carried by the frames
  uint8_t prev_seq = seq_outbound;                                      // previous message sequence
  uint8_t send_attempt_counter = 0;

  // The loop will repeat if a frame needs to be resent.
  do
  {
    if (send_attempt_counter++ > 5)
    {
      return 0;
    }

    // Reset the resending flag so its change can be detected again.
    _resend_frame = false;
    _wait_for_nak = false;

    // If resending, the frame sequence must be set to the previous value.
    if (sent_once)
    {
      // If resending because a wrong-ack or a not-ack, set the frame sequence correctly.
      if (ack_seq_expected != _default_value)
      {
        // Determine from which frame to start retransmitting.
        bool is_in_block;                                                                                        // will indicate whether or not the acknowledgement's sequence exists in the range of sequences carried by the block's frames
        uint8_t pos = _seq_position_in_block(ack_seq_expected, starting_sequence, ending_sequence, is_in_block); // calculate the starting frame

        // If the acknowledgement's sequence exists in the range, start retransmitting from the position indicated by it.
        if (is_in_block)
        {
          starting_block_from_frame = pos + FC_frames * TP_max_block_length; // calculate from which frame to start, taking flow-control frames into account
          seq_outbound = ack_seq_expected;                                   // set the starting sequence to the acknowledgement's sequence
          prev_seq = seq_outbound;                                           // update the previous sequence
        }
        // If it isn't in range (improbable), set the sequence to whatever was received (and retransmit all frames).
        else
        {
          seq_outbound = ack_seq_expected;
        }

        // Reset to a default value so its change can be detected if a frame is missed again.
        ack_seq_expected = _default_value;
      }
      // If resending because of other reasons, revert the sequence to the beginning sequence.
      else
      {
        if (!user_called_receive_function)
        {
          seq_outbound = prev_seq;
        }
      }
    }

    // Calculate how many bytes have been sent so far (can be different than 0 when resending).
    bytes_copied = starting_block_from_frame * 7;

    // Construct and send each frame.
    for (uint8_t current_frame = starting_block_from_frame; current_frame < number_of_frames; current_frame++)
    {
      // Block size 0, improbable error.
      if (!block_length)
      {
        show_debug_info(NULL_SIZE_BLOCK);
        return 0;
      }

      // More bytes copied than the given block size, probably impossible.
      if (bytes_copied > block_length)
      {
        show_debug_info(IMPOSSIBLE_BUFFER_INDEX);
        return 0;
      }

      // On the last (or only) frame in the block, the PCI type is "FIRST FRAME".
      if (current_frame == (number_of_frames - 1))
      {
        frame[0] = PCI_FF << 4; // 0x1

        // Copy the rest of the frame into the buffer.
        memcpy(frame + 1, block + bytes_copied, block_length - bytes_copied);
      }
      // On an intermediate frame in the block, the PCI type is "CONSECUTIVE FRAME", but it becomes "FLOW CONTROL" if the maximum block size is reached.
      else
      {
        // If the module's maximum block size is exceeded, use a flow control frame.
        if ((current_frame + 1) % TP_max_block_length == 0)
        {
          frame[0] = PCI_FC << 4; // 0x0
          FC_frames++;
        }
        // Otherwise, use a consecutive frame.
        else
        {
          frame[0] = PCI_CF << 4; // 0x2
        }

        // Copy the rest of the frame into the buffer.
        memcpy(frame + 1, block + bytes_copied, 7);

        // Increment the counter, so the next frame starts copying bytes from where the previous one left off.
        bytes_copied += 7;
      }

      // Determine how many bytes the current frame contains.
      uint8_t send_length =
        (current_frame == number_of_frames - 1) // if on the last (or only) frame,
        ? (block_length - bytes_copied + 1) // then however many bytes are left to send,
        : 8;                                // otherwise 8 bytes

      // Send the frame, and if the module sends a message during transmission, save it in the given buffer.
      uint16_t received_message_size = 0;
      VWTPLib::_ERROR err = _send_with_sequence(frame, send_length, buffer, buffer_size, received_message_size, (current_frame == starting_block_from_frame));

      if (_wait_for_nak)
      {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(T_WAIT));
      }

      switch (err)
      {
        // If sending fails, don't continue transmitting the rest of the block.
        case _ERROR_SENDING_ERROR:
        case _ERROR_VWTP_TERMINATED: // connection broken
          return 0;

        case _ERROR_MESSAGE: // message interrupted transmission
          return received_message_size;

        default: // sent successfully
          break;
      }
    }

    // If the loop repeats (_resend_frame = true), this ensures the sequence is assigned properly when resending.
    sent_once = true;
  }
  // Repeat the loop if the block (or a part of it) must be resent.
  while (_resend_frame);

  // Transmission finished.
  return 1;
}

bool VWTPLib::isMessageValid(unsigned long can_id)
{
  return (can_id == _recv_id);
}

bool VWTPLib::provideMessage(unsigned long can_id, uint8_t can_dlc, uint8_t *data, bool *message_accepted)
{
  if (message_accepted)
  {
    *message_accepted = false;
  }

  // If the provided message isn't coming from the desired ID, exit immediately.
  if (!isMessageValid(can_id))
  {
    return false;
  }

  if (message_accepted)
  {
    *message_accepted = true;
  }

  // Save the provided message's details in the private struct.
  xSemaphoreTake(copyingMessageMutex, portMAX_DELAY);
  _received_message.can_id = can_id;
  _received_message.can_dlc = can_dlc;
  memcpy(_received_message.data, data, can_dlc);
  xSemaphoreGive(copyingMessageMutex);

  // If a channel command was received, respond to it immediately.
  if ((_received_message.data[0] >> 4) == PCI_CMD)
  {
    // Even if connected, don't respond to a channel command automatically if there is no termination function defined.
    // If a termination were to be triggered, it could lock up the system trying to execute it from a provider task.
    if (_connected && !(_received_message.data[0] != VWTP_KEEP_ALIVE[0] && !_termination_function))
    {
      uint16_t msg_size;
      _ERROR ret = _receive(_receive_buffer, sizeof(_receive_buffer), msg_size, false, 0, true); // check for messages, responding to them automatically

      if (ret == _ERROR_VWTP_TERMINATED)
      {
        // Bit 2 in the event group is used for indicating that the VWTP session was ended.
        xEventGroupSetBits(receiveEvent, 2);
      }
    }
    else
    {
      // Set the event bit, so the message can be responded to.
      xEventGroupSetBits(receiveEvent, 1);
    }

    // Indicate that the message was NOT accepted, so it is ignored.
    return false;
  }

  // Set the event bit, so the message can be responded to.
  xEventGroupSetBits(receiveEvent, 1);

  // Indicate that the message was accepted.
  return true;
}

uint16_t VWTPLib::receive(uint8_t *buffer, uint16_t buffer_size, bool wait_for_message, unsigned long timeout)
{
  if (!_connected)
  {
    return 0;
  }

  user_called_receive_function = true;

  // Will store the received message length (or less, if the buffer was not large enough).
  uint16_t received_message_size = 0;

  // Check for a message and save it in a buffer if available.
  _ERROR ret = _receive(buffer, buffer_size, received_message_size, wait_for_message, timeout);
  if (ret == _ERROR_MESSAGE)
  {
    return received_message_size;
  }

  return 0;
}

void VWTPLib::wakeupFromRead()
{
#ifdef VWTP_SHOW_INFO
  printf("VWTP waking\n");
#endif
  
  // Bit 4 in the event group is used for indicating that VWTP was manually woken up while waiting to receive a message.
  xEventGroupSetBits(receiveEvent, 4);
}

void VWTPLib::terminationFunction(terminationFunction_type termination_function)
{
  _termination_function = termination_function;
}

/// PRIVATE

bool VWTPLib::_send_raw(const uint8_t *frame, uint8_t size)
{
  return _send_raw((uint8_t *)frame, size); // call the function which takes a non-const byte array
}

bool VWTPLib::_send_raw(uint8_t *frame, uint8_t size)
{
  // Wait before sending a frame (if necessary).
  vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(_message_delay));

  // Use the defined sendFunction to send the frame on the CAN Bus.
  if (_send_function)
  {
    if (!_send_function(_send_id, size, frame))
    {
      return false;
    }
  }

  // Save the current time after sending the frame, to delay the next frame properly.
  _last_frame_time = esp_timer_get_time() / 1000;
  xLastWakeTime = xTaskGetTickCount();

  // Print what was just sent.
  if (_debug_function)
  {
    _debug_function(0, _last_frame_time - _debug_time, _send_id, size, frame); // call the defined function
  }
  _debug_time = _last_frame_time; // reset the timer

  return true;
}

VWTPLib::_ERROR VWTPLib::_send_with_sequence(const uint8_t *frame, uint8_t size, uint8_t *buffer, uint16_t buffer_size, uint16_t &received_message_size, bool is_first_frame)
{
  return _send_with_sequence((uint8_t *)frame, size, buffer, buffer_size, received_message_size, is_first_frame); // call the function which takes a non-const byte array
}

VWTPLib::_ERROR VWTPLib::_send_with_sequence(uint8_t *frame, uint8_t size, uint8_t *buffer, uint16_t buffer_size, uint16_t &received_message_size, bool is_first_frame)
{
  // If a connection isn't established, don't send anything (exit the function).
  if (!_connected)
  {
    show_debug_info(NOT_CONNECTED);
    return _ERROR_VWTP_TERMINATED;
  }

  // Check for incoming messages before sending.
  user_called_receive_function = false;

  // If a message (not a VWTP channel command) is received, save it in the given buffer and exit.
  switch (_receive(buffer, buffer_size, received_message_size, false))
  {
    case _ERROR_VWTP_TERMINATED: // don't continue sending the message if the connection is broken
      return _ERROR_VWTP_TERMINATED;

    case _ERROR_MESSAGE: // exit if a message was received
      return _ERROR_MESSAGE;

    case _ERROR_SENDING_ERROR:
      return _ERROR_SENDING_ERROR;

    default: // if everything is ok, continue
      if (user_called_receive_function)
      {
        return _ERROR_SENDING_ERROR;
      }
      break;
  }

  if (is_first_frame)
  {
    _resend_frame = false;
    _wait_for_nak = false;
  }

  // Increment the sequence number after adding it to the PCI byte.
  frame[0] += seq_outbound++ % 16;

  // Send the sequenced frame.
  if (!_send_raw(frame, size))
  {
    return _ERROR_SENDING_ERROR;
  }

  // Determine what PCI type the sent frame was (high nibble of the first byte).
  uint8_t PCI = frame[0] >> 4;

  // FIRST FRAMEs and FLOW CONTROL frames need to receive an acknowledgement.
  if (PCI == PCI_FF || PCI == PCI_FC)
  {
    // Wait for a response.
    while (true)
    {
      // Check what was received.
      switch (_receive(buffer, buffer_size, received_message_size, true, _message_timeout))
      {
        case _ERROR_MESSAGE: // message received (not a VWTP channel command)
          return _ERROR_MESSAGE;

        case _ERROR_ACK: // acknowledgement
          return _ERROR_ACK;

        case _ERROR_NAK: // not-acknowledgement
          return _ERROR_NAK;

        case _ERROR_TIMEOUT: // timeout passed without any response
          {
            // Check if the module is still online.
            show_debug_info(CHECK_CONNECTION);

            bool got_ack;
            if (_check_connection(got_ack))
            {
              show_debug_info(CONNECTION_OK);

              // If the acknowledgement was received while checcking the connection, take it into account.
              if (got_ack)
              {
                return _ERROR_ACK;
              }

              if (user_called_receive_function)
              {
                return _ERROR_SENDING_ERROR;
              }

              // If it is online, resend the message.
              _resend_frame = true;
              return _ERROR_TIMEOUT;
            }
            else
            {
              // If it isn't online anymore, the connection is broken.
              return _ERROR_VWTP_TERMINATED;
            }
          }

        case _ERROR_SENDING_ERROR:
          return _ERROR_SENDING_ERROR;

        case _ERROR_VWTP_TERMINATED: // connection broken
          return _ERROR_VWTP_TERMINATED;

        default: // in an unhandled case, wait again
          break;
      }
    }
  }
  // For other frame types, acknowledgement is not needed.
  else
  {
    return _ERROR_ACK;
  }
}

VWTPLib::_ERROR VWTPLib::_receive(uint8_t *buffer, uint16_t buffer_size, uint16_t &received_message_size, bool wait_for_message, unsigned long timeout, bool take_precedence)
{
  // If provided with a timeout of 0, use the default acknowledgement timeout.
  if (!timeout)
  {
    timeout = _message_timeout;
  }

  // Will be assigned the PCI type of the received frame.
  uint8_t PCI = _default_value;

  // Used for properly receiving a block if a frame is missed.
  bool on_first_frame = true;
  uint8_t block_low_bound = _default_value, block_high_bound = _default_value, repeat_msg_seq = _default_value;
  bool force_continue = false;
  uint8_t force_continue_counter = 0;
  _ERROR ret_value = _ERROR_OK;
  FC_counter = 0;

  /*
    It is possible for the module to send a special message (ACK/NACK/etc.) while transmitting a block.
    To get a clean reconstructed message, that message has to be suppressed.
    For that, it will be redirected to the internal buffer instead of the function's provided buffer, and block receiving will be forced to continue.
  */
  uint8_t *reconstruction_buffer;           // points to the buffer where messages are reconstructed (internal/provided buffer)
  uint16_t reconstruction_buffer_size;      // contains the total size of the buffer where messages are reconstructed (internal/provided buffer)
  uint16_t reconstruction_buffer_index = 0; // used for navigating the provided buffer when saving the received message

  // When a message is received, it will be copied to these local variables.
  // When executing a send (like acknowledging), the global "_received_message" struct may contain new message parameters, which must not be used for
  // calculations involving the previous message.
  uint8_t received_message_can_dlc;
  uint8_t received_message_data[8];

  // Receive all frames in a block.
  do
  {
    // Reset the force flag in case it was set.
    force_continue = false;

    /*
      In case the special situation mentioned above does happen, the reconstruction buffer may have been switched to the internal one.
      So now switch it back to the provided buffer.
    */
    reconstruction_buffer = buffer;           // point to the provided buffer
    reconstruction_buffer_size = buffer_size; // set the total buffer size to what was provided

    // If a null buffer/size was provided, use the internal buffer to receive the message.
    if (!reconstruction_buffer || !reconstruction_buffer_size)
    {
      reconstruction_buffer = _receive_buffer;              // point to the internal buffer
      reconstruction_buffer_size = sizeof(_receive_buffer); // the internal buffer's size is known at compile time
    }

    if (!take_precedence)
    {
      // Not waiting for message, and on the first frame:
      if (!wait_for_message && on_first_frame)
      {
        // Check for a message "without waiting" (run for one tick).
        EventBits_t eventBits = xEventGroupWaitBits(receiveEvent, 1, pdTRUE, pdTRUE, 1);

        // No message available.
        if (!(eventBits & 1))
        {
          // If waiting for a frame is not requested, exit the function (only if not waiting for the rest of a block).
          return _ERROR_NO_MESSAGE;
        }
      }
      // Waiting for message, or not waiting for message but on intermediate frames:
      else
      {
        // If receiving times out, wait for a message again.
        while (true)
        {
          // Check if the requested timeout is -1.
          static const unsigned long ulong_max = -1;
          TickType_t wait_ticks = pdMS_TO_TICKS(timeout);
          EventBits_t bitsToWaitFor = 1 | 2;
          if (timeout == ulong_max)
          {
            // If the timeout is -1, the function will wait until a message is received.
            wait_ticks = portMAX_DELAY;
            
            // It is still possible to stop waiting even without a message, if wakeupFromRead() is called.
            bitsToWaitFor |= 4;
            xEventGroupClearBits(receiveEvent, 4);
          }

          // Wait for a message.
          EventBits_t eventBits = xEventGroupWaitBits(receiveEvent, bitsToWaitFor, pdTRUE, pdFALSE, wait_ticks);

          // Message 0xA8 was received by provideMessage(), disconnected.
          if (eventBits & 2)
          {
            return _ERROR_VWTP_TERMINATED;
          }
          
          // If a manual wakeup was expected and it happened, report a timeout error.
          if ((bitsToWaitFor & 4) && (eventBits & 4))
          {
#ifdef VWTP_SHOW_INFO
            printf("VWTP woken\n");
#endif
            return _ERROR_TIMEOUT;
          }

          // Timed out.
          if (!(eventBits & 1))
          {
            // If a frame out of a message was received (PCI_CF) but the end of the block wasn't (PCI_FF)
            if (!on_first_frame)
            {
              // Send an acknowledgement with the sequence that was actually expected.
              if (!_acknowledge(msg_seq_expected))
              {
                return _ERROR_SENDING_ERROR;
              }
              
              // Don't get stuck in a loop.
              if (++ACK_counter >= 5)
              {
                ACK_counter = 0;

                // Execute the channel termination procedure (disconnect (wait for rsp.) + custom function/reconnect).
                _on_termination(true, true);
                return _ERROR_VWTP_TERMINATED;
              }
            }
            else
            {
              // If waiting for a frame is requested but the timeout passes, exit the function.
              return _ERROR_TIMEOUT;
            }
          }
          // Message received.
          else
          {
            break;
          }
        }
      }
    }

    // Copy the new message into the local variables, so that the parameters of a new message are not used for calculations if one is received when an
    // acknowledgement is sent that triggers the requestFunction while waiting.
    xSemaphoreTake(copyingMessageMutex, portMAX_DELAY);
    received_message_can_dlc = _received_message.can_dlc;
    memcpy(received_message_data, _received_message.data, received_message_can_dlc);
    xSemaphoreGive(copyingMessageMutex);
    
    ACK_counter = 0;

    // Save the current time after receiving the frame, to delay the next frame properly.
    _last_frame_time = esp_timer_get_time() / 1000;
    xLastWakeTime = xTaskGetTickCount();

    // Print what was just received.
    if (_debug_function)
    {
      _debug_function(1, _last_frame_time - _debug_time, _recv_id, received_message_can_dlc, received_message_data); // call the defined function
    }
    _debug_time = _last_frame_time; // reset the timer

    // Determine the PCI of the received frame (high nibble of the first byte).
    PCI = received_message_data[0] >> 4;

    // The low nibble of the PCI byte for FIRST FRAMEs, CONSECUTIVE FRAMEs, FLOW CONTROL frames and LAST FRAMEs represents the inbound message sequence.
    if (PCI == PCI_FF || PCI == PCI_CF || PCI == PCI_FC || PCI == PCI_LF)
    {
      /*
        Bug:
           If a module's keep-alive timer goes off while it's sending a block, it
           sends the keep-alive but doesn't continue the block afterwards.
           The solution is to send an "acknowledge" message with the last received
           frame's sequence +1, to tell the module to retransmit from that point.
           The problem occurs if the module thinks it has finished sending that
           block, as its last frame(s) were part of the next "rolling sequence".
           For example, if the last received message had the sequence "E", but the
           module thinks it has finished on sequence "1", acknowledging with the
           sequence "F" will confuse it and trigger the bug.
           The module will start sending an apparently infinite block containing garbage.

        Patch:
           When a second FLOW CONTROL frame is detected in a block, the connection
           is terminated, and the function pretends nothing was received, because
           I don't believe any actual message can be that long, only the bug.
           If it is in fact possible, just increase the threshold from 2.
      */

      // On FLOW CONTROL frames, increment the counter.
      if (PCI == PCI_FC)
      {
        FC_counter++;

        // If this is the second FLOW CONTROL frame, patch the bug.
        if (FC_counter >= 2)
        {
          // Execute the channel termination procedure (disconnect (wait for rsp.) + custom function/reconnect).
          _on_termination(true, true);
          return _ERROR_VWTP_TERMINATED;
        }
      }

      // Sync the inbound sequence to what was received.
      seq_inbound = received_message_data[0] & 0xF;

      /*
        If a frame is detected as missing (inconsistent sequence number), block retransmission will be requested starting from that missed frame.
        To determine to which position to return in the reconstruction buffer, the missed frame's sequence will be checked and its position determined based on
        whether or not it is included in the block defined by the currently known lower and upper bounds.
      */

      // The upper bound is determined from the sequence of the last frame in a block.
      // TODO: support long blocks with FC messages
      if (PCI == PCI_FF || PCI == PCI_LF)
      {
        block_high_bound = seq_inbound;
      }

      // If the received message sequence matches what was expected, everything is fine.
      if (seq_inbound == msg_seq_expected)
      {
        // If on the first frame, the lower bound is determined by the frame's sequence.
        if (on_first_frame)
        {
          block_low_bound = seq_inbound;
        }
      }
      // Otherwise, retransmission must be requested.
      else
      {
        // If the repeat sequence wasn't set already, set it to what was expected.
        if (repeat_msg_seq == _default_value)
        {
          repeat_msg_seq = msg_seq_expected;
        }
      }

      // If the block's lower bound wasn't set already, set it to the expected sequence.
      if (block_low_bound == _default_value)
      {
        block_low_bound = msg_seq_expected;
      }

      // Increment the expected message sequence.
      msg_seq_expected = (seq_inbound + 1) & 0xF;

      // FIRST FRAMEs and FLOW CONTROL frames need to be acknowledged.
      if (PCI != PCI_CF && PCI != PCI_LF)
      {
        // If retransmission is necessary, acknowledge using the repeat sequence.
        if (repeat_msg_seq != _default_value)
        {
          if (repeat_msg_seq < msg_seq_expected)
          {
            if (!_acknowledge(repeat_msg_seq))
            {
              return _ERROR_SENDING_ERROR;
            }
          }
          else
          {
            // Nevermind (expected is greater than inbound on ack), acknowledge normally.
            if (!_acknowledge(msg_seq_expected))
            {
              return _ERROR_SENDING_ERROR;
            }
            // Pretend nothing was received.
            return _ERROR_NO_MESSAGE;
          }
        }
        // Normally acknowledge with the received sequence + 1.
        else
        {
          if (!_acknowledge(msg_seq_expected))
          {
            return _ERROR_SENDING_ERROR;
          }
        }
      }
    }

    // Reconstruct the message by progressively saving each frame in the buffer, excluding the PCI byte.

    // Regular message types are (normally) stored in the given buffer.
    if (PCI != PCI_ACK && PCI != PCI_NAK && PCI != PCI_CMD)
    {
      // Calculate how many bytes can be written in the buffer.
      uint16_t free_space = reconstruction_buffer_size - reconstruction_buffer_index;

      // If there is some space in the buffer, copy the frame.
      if (free_space)
      {
        // Determine how many bytes to copy from the received frame into the buffer.
        uint16_t bytes_to_copy =
          (uint16_t(received_message_can_dlc - 1) <= free_space) // if there is enough space for the entire frame,
          ? (received_message_can_dlc - 1)                   // then copy the entire frame,
          : free_space;                                      // otherwise only copy however many bytes are available in the buffer

        // When copying from the frame into the buffer, 1 must be added to the source pointer to start from the second byte.
        memcpy(reconstruction_buffer + reconstruction_buffer_index, received_message_data + 1, bytes_to_copy);

        // Increment the buffer index by however many bytes were copied.
        reconstruction_buffer_index += bytes_to_copy;
      }
    }
    // Acknowledgements, not-acknowledgements and VWTP channel commands are saved in the private buffer.
    else
    {
      reconstruction_buffer = _receive_buffer;
      reconstruction_buffer_size = sizeof(_receive_buffer);

      // Determine how many bytes to copy from the received frame into the buffer.
      uint16_t bytes_to_copy =
        (received_message_can_dlc <= reconstruction_buffer_size) // if there is enough space for the entire frame,
        ? received_message_can_dlc                           // then copy the entire frame,
        : reconstruction_buffer_size;                        // otherwise only copy however many bytes are available in the buffer

      // Save the frame.
      memcpy(reconstruction_buffer, received_message_data, bytes_to_copy);

      // Save the opcode of the received frame.
      uint8_t opcode = received_message_data[0];

      /*
        Normally, receiving such message types would require exiting the function.
        But in the special case mentioned above, block receiving must be forced to continue.
      */
      ret_value = _respond(opcode); // save what _respond() returns

      // Force continuing to receive the block if PROVIDE_PARAMETERS/ACK/NAK was received.
      if ((opcode == VWTP_PROVIDE_PARAMETERS[0] || PCI == PCI_ACK || PCI == PCI_NAK) && !on_first_frame)
      {
        force_continue = true;
      }
      else
      {
        // Store how many bytes were copied into the buffer (into the variable passed as reference).
        received_message_size = bytes_to_copy;

        // This message is guaranteed to be the last in the block.
        return ret_value;
      }
    }

    // If on an acknowledgeable frame, set parameters to prepare for block re-receiving.
    if ((PCI == PCI_FF || PCI == PCI_FC) && (repeat_msg_seq != _default_value))
    {
      // If, in a block, some frames were already received properly, figure out from which position in the block re-receiving is necessary.
      bool is_in_block;
      uint8_t pos = _seq_position_in_block(repeat_msg_seq, block_low_bound, block_high_bound, is_in_block);

      // Every frame received properly in a block will have contained 7 bytes, so change the rec. buffer index to the appropriate value.
      reconstruction_buffer_index = pos * 7;

      // Constrain the buffer index.
      if (reconstruction_buffer_index > reconstruction_buffer_size)
      {
        reconstruction_buffer_index = reconstruction_buffer_size;
      }

      // Change the expected message sequence to the sequence which will be repeated.
      msg_seq_expected = repeat_msg_seq;

      // Change the repeated message sequence to a default value so its change can be detected next time.
      repeat_msg_seq = _default_value;

      // Force the loop to repeat in order to receive the corrected block.
      force_continue = true;

      if (++force_continue_counter >= 5)
      {
        msg_seq_expected = seq_inbound;
      }
    }

    // No longer on the first frame if the loop repeats.
    on_first_frame = false;
  }
  // Repeat the loop to continue receiving the rest of the block.
  while (PCI == PCI_CF || PCI == PCI_FC || force_continue);

  // The last frame received was the end of a block.
  if (PCI == PCI_FF || PCI == PCI_LF)
  {
    // Store how many bytes were copied into the buffer (into the variable passed as reference).
    received_message_size = reconstruction_buffer_index;

    // If a special message interrupted receiving of a data block, return the special message's type.
    if (ret_value != _ERROR_OK)
    {
      return ret_value;
    }

    // Otherwise, indicate that a block was received.
    return _ERROR_MESSAGE;
  }

  // Should not be able to happen.
  return ret_value;
}

bool VWTPLib::_acknowledge(uint8_t seq)
{
  // Will hold the acknowledgement frame.
  uint8_t acknowledgement[1];

  // Add 0xB0 and the given sequence to the frame.
  acknowledgement[0] = (PCI_ACK << 4) | (seq & 0xF);

  // Send the frame.
  return _send_raw(acknowledgement, 1);
}

VWTPLib::_ERROR VWTPLib::_respond(uint8_t opcode)
{
  // The way in which the function is called ensures the opcode can only be a VWTP channel command, acknowledgement or not-acknowledgement.
  uint8_t message_type = opcode >> 4; // determine what VWTP type the opcode is

  // VWTP channel command
  if (message_type == PCI_CMD)
  {
    // The connection was reinitialized.
    if (opcode == VWTP_REQUEST_PARAMETERS[0])
    {
      // Respond.
      if (!_send_raw(VWTP_PROVIDE_PARAMETERS, sizeof(VWTP_PROVIDE_PARAMETERS)))
      {
        return _ERROR_SENDING_ERROR;
      }

      // If the message was received while the connection was established, an error occurred.
      if (_connected)
      {
        show_debug_info(CONNECTION_TERMINATED);

        // Execute the channel termination procedure (disconnect (wait for rsp.) + custom function/reconnect).
        _on_termination(true, true);
        return _ERROR_VWTP_TERMINATED;
      }
      // If there was no connection established, don't react to the message.
      else
      {
        return _ERROR_RQ;
      }
    }

    // The module is requesting a response for maintaining the connection.
    if (opcode == VWTP_KEEP_ALIVE[0])
    {
      // Respond.
      if (!_send_raw(VWTP_PROVIDE_PARAMETERS, sizeof(VWTP_PROVIDE_PARAMETERS)))
      {
        return _ERROR_SENDING_ERROR;
      }

      // Receiving such message causes update() to be called automatically, so it cannot be received in any other function.
      return _ERROR_OK;
    }

    // The connection was initialized.
    if (opcode == VWTP_PROVIDE_PARAMETERS[0])
    {
      return _ERROR_PR;
    }

    // The module disconnected.
    if (opcode == VWTP_DISCONNECT[0])
    {
      // If the message was received while the connection was established, an error occurred.
      if (_connected)
      {
        show_debug_info(MODULE_DISCONNECTED);

        // Execute the channel termination procedure (disconnect (without waiting for rsp.) + custom function/reconnect).
        _on_termination(true, false);
      }
      return _ERROR_VWTP_TERMINATED;
    }
  }

  // Acknowledgement
  if (message_type == PCI_ACK)
  {
    // Check if the acknowledgement matches the expected sequence.

    // Determine the sequence received in the not-acknowledgement.
    uint8_t received_seq = opcode & 0xF;

    // If the received sequence doesn't match what was expected, the frame must be resent.
    if (received_seq != seq_outbound % 16)
    {
      show_debug_info(FRAME_MISSED);

      // Send the frame again, with the received sequence.
      ack_seq_expected = received_seq;
      _resend_frame = true;
    }

    NAK_counter = 0;
    return _ERROR_ACK;
  }

  // Not-acknowledgement
  if (message_type == PCI_NAK)
  {
    // Determine the sequence received in the not-acknowledgement.
    uint8_t received_seq = opcode & 0xF;

    show_debug_info(FRAME_NAK);
    NAK_counter++;

    if (NAK_counter >= 5 && repeated_resend_error_enabled)
    {
      NAK_counter = 0;

      if (repeated_resend_error_execute_termination)
      {
        // Execute the channel termination procedure (disconnect (wait for rsp.) + custom function/reconnect).
        _on_termination(true, true);
      }
      else
      {
        disconnect();
        connect();
      }

      repeated_resend_error_occurred = true;
      return _ERROR_SENDING_ERROR;
    }
    else
    {
      // Send the frame again, with the received sequence.
      ack_seq_expected = received_seq;
      _resend_frame = true;
      _wait_for_nak = true;
      return _ERROR_NAK;
    }
  }

  // Leave other cases unhandled.
  return _ERROR_OK;
}

bool VWTPLib::_wait_for_disconnect_response()
{
  // Attempt to receive a message.
  uint16_t msg_size; // dummy variable
  if (_receive(_receive_buffer, sizeof(_receive_buffer), msg_size, true, _message_timeout) == _ERROR_VWTP_TERMINATED)
  {
    if (_receive_buffer[0] == VWTP_DISCONNECT[0])
    {
      // Disconnect message received.
      return true;
    }
  }

  // Disconnect message not received.
  return false;
}

uint8_t VWTPLib::_seq_position_in_block(uint8_t value, uint8_t lower, uint8_t upper, bool &is_in_range)
{
  uint8_t range_size = (upper >= lower) ? (upper - lower + 1) : (upper + 16 - lower + 1);
  uint8_t position = (value - lower + 16) % 16;
  is_in_range = (position < range_size);
  return is_in_range ? position : 0;
}

void VWTPLib::_on_termination(bool also_disconnect, bool wait_for_disconnect_response)
{
  // Avoid recursion if a connection-breaking message is received while executing the termination function.
  if (_executing_termination)
  {
    return;
  }

  // Ensure the function is not executed multiple times.
  _executing_termination = true;

  // If disconnecting is requested, do it, taking into account the request for waiting for a response.
  if (also_disconnect)
  {
    disconnect(wait_for_disconnect_response);
  }

  // Reset the sequence counters.
  seq_inbound = seq_outbound = msg_seq_expected = 0;

  // If a custom function is defined, execute it.
  if (_termination_function)
  {
    _termination_function(this);
  }
  // Otherwise, reconnect automatically.
  else
  {
    //printf("No VWTP terminationFunction defined, doing connect() from task %s\n", pcTaskGetName(NULL));
    connect();
  }

  // Finished executing.
  _executing_termination = false;
}

bool VWTPLib::_check_connection(bool &got_ack)
{
  // Reset the flag's value.
  got_ack = false;

  // Will repeat 10 times.
  uint8_t counter = 10;
  while (counter--)
  {
    // Send a keep-alive request.
    if (!_send_raw(VWTP_KEEP_ALIVE, sizeof(VWTP_KEEP_ALIVE)))
    {
      return false;
    }

    // Ensure VWTP channel commands are not responded to automatically.
    _connected = false;

    // Try to receive a message.
    uint16_t msg_size; // dummy variable
    _ERROR ret_value = _receive(_receive_buffer, sizeof(_receive_buffer), msg_size);

    // Re-enable automatic responses.
    _connected = true;

    switch (ret_value)
    {
      case _ERROR_ACK: // received the acknowledgement that was expected before _check_connection() was called
        got_ack = true;
        return true;

      case _ERROR_PR: // expected message
        return true;

      case _ERROR_VWTP_TERMINATED: // connection broken (module disconnected already)
        show_debug_info(MODULE_DISCONNECTED);

        // Execute the channel termination procedure (disconnect (without waiting for rsp.) + custom function/reconnect).
        _on_termination(true, false);
        return false;

      case _ERROR_RQ: // connection broken
        show_debug_info(CONNECTION_TERMINATED);

        // Execute the channel termination procedure (disconnect (wait for rsp.) + custom function/reconnect).
        _on_termination(true, true);
        return false;

      default: // try again
        break;
    }
  }

  // If 10 attempts have failed, the connection is broken.
  show_debug_info(CONNECTION_NOT_OK);

  // Disable automatic responses.
  _connected = false;

  // Execute the channel termination procedure (custom function/reconnect (no disconnect)).
  _on_termination(false);
  return false;
}

void VWTPLib::show_debug_info(DEBUG_TYPE type)
{
#ifdef VWTP_SHOW_INFO
  switch (type)
  {
    case CONNECTED:
      printf("Info: connected\n");
      break;

    case NOT_CONNECTED:
      printf("Error: not connected\n");
      break;

    case DISCONNECTED:
      printf("Info: disconnected\n");
      break;

    case NULL_SIZE_SEND:
      printf("Error: send() with a block_length of 0\n");
      break;

    case NULL_SIZE_BLOCK:
      printf("Fatal error: block_length = 0\n");
      break;

    case IMPOSSIBLE_BUFFER_INDEX:
      printf("Fatal error: bytes_copied > block_length\n");
      break;

    case CONNECTION_TERMINATED:
      printf("Error: VWTP connection terminated\n");
      break;

    case MODULE_DISCONNECTED:
      printf("Error: Module requested disconnect\n");
      break;

    case FRAME_MISSED:
      printf("Error: frame missed; resending\n");
      break;

    case FRAME_NAK:
      printf("Error: frame not acknowledged; resending\n");
      break;

    case CHECK_CONNECTION:
      printf("Info: checking connection\n");
      break;

    case CONNECTION_OK:
      printf("Info: connection OK\n");
      break;

    case CONNECTION_NOT_OK:
      printf("Info: connection not OK\n");
      break;
  }
#else
  (void)type; // avoid unused variable compiler warning if not using debug
#endif
}
