#ifndef CAN_H
#define CAN_H

// Pin assignments for the CAN transceiver
#define DIAG_TX_PIN GPIO_NUM_20
#define DIAG_RX_PIN GPIO_NUM_21

// CAN library (for using the ESP32's integrated CAN interface)
#include <driver/twai.h>

// CAN interface handle
twai_handle_t diagnostic_can_bus;

// Structure that holds the contents of a CAN frame, either to be sent, or which was received
struct CAN_frame
{
  unsigned long can_id;
  uint8_t can_dlc;
  uint8_t data[8];
};

// FreeRTOS queues for sending and receiving frames
QueueHandle_t diagnostic_CAN_send_queue;
QueueHandle_t diagnostic_CAN_receive_queue;

// Function that queues a CAN frame to be sent on the appropriate bus
void send_CAN(CAN_frame *frame)
{
  // Ensure the pointer is valid.
  if (!frame)
  {
    return;
  }

  // Queue the frame.
  xQueueSend(diagnostic_CAN_send_queue, (void*)frame, 0);
}

// Task that sends the frames received on its queue
void CAN_send_task(void *arg)
{
  // The frames arrive in CAN_frame format, they must be sent in twai_message_t format.
  CAN_frame frame_can;
  twai_message_t frame_twai;

  while (true)
  {
    // Wait for a frame to arrive on the queue.
    xQueueReceive(diagnostic_CAN_send_queue, &frame_can, portMAX_DELAY);

    // Copy the received frame's details into the TWAI frame structure.
    frame_twai.identifier = frame_can.can_id;
    frame_twai.extd = 0;
    frame_twai.rtr = 0;
    frame_twai.data_length_code = frame_can.can_dlc;
    memcpy(frame_twai.data, frame_can.data, frame_can.can_dlc);

    // Send the frame.
    uint32_t alerts_triggered;
    twai_reconfigure_alerts_v2(diagnostic_can_bus, TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED, NULL);
    twai_transmit_v2(diagnostic_can_bus, &frame_twai, portMAX_DELAY);
    twai_read_alerts_v2(diagnostic_can_bus, &alerts_triggered, portMAX_DELAY);
    if (alerts_triggered & TWAI_ALERT_TX_SUCCESS)
    {
#ifdef DEBUG_CAN_TRAFFIC
      DEBUG_TRAFFIC("[ CAN SEND] | %03lX | ", frame_can.can_id);
      for (size_t i = 0; i < frame_can.can_dlc; i++)
      {
        DEBUG_TRAFFIC("%02X ", frame_can.data[i]);
      }
      DEBUG_TRAFFIC("\n");
#endif
    }
  }
}

// Task that queues the frames received on its bus
void CAN_receive_task(void *arg)
{
  // The frames arrive in twai_message_t format, they must be queued in CAN_frame format.
  twai_message_t frame_twai;
  CAN_frame frame_can;

  while (true)
  {
    // Wait for a frame to arrive on the bus.
    twai_receive_v2(diagnostic_can_bus, &frame_twai, portMAX_DELAY);

    // Copy the received frame's details into the CAN frame structure.
    frame_can.can_id = frame_twai.identifier;
    frame_can.can_dlc = frame_twai.data_length_code;
    memcpy(frame_can.data, frame_twai.data, frame_twai.data_length_code);

#ifdef DEBUG_CAN_TRAFFIC
    DEBUG_TRAFFIC("[ CAN RECV] | %03lX | ", frame_can.can_id);
    for (size_t i = 0; i < frame_can.can_dlc; i++)
    {
      DEBUG_TRAFFIC("%02X ", frame_can.data[i]);
    }
    DEBUG_TRAFFIC("\n");
#endif

    // Queue the frame.
    xQueueSend(diagnostic_CAN_receive_queue, (void*)&frame_can, 0);
  }
}

// Initialize CAN
void init_CAN()
{
  twai_general_config_t g_config_can =
  {
    .controller_id = 0,
    .mode = TWAI_MODE_NORMAL,
    .tx_io = DIAG_TX_PIN,
    .rx_io = DIAG_RX_PIN,
    .clkout_io = TWAI_IO_UNUSED,
    .bus_off_io = TWAI_IO_UNUSED,
    .tx_queue_len = 1,
    .rx_queue_len = 5,
    .alerts_enabled = TWAI_ALERT_NONE,
    .clkout_divider = 0,
    .intr_flags = ESP_INTR_FLAG_LEVEL1
  };
  twai_timing_config_t t_config_can = TWAI_TIMING_CONFIG_500KBITS(); // The Diagnostic CAN-Bus runs at a baud rate of 500K.
  twai_filter_config_t f_config_can = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install_v2(&g_config_can, &t_config_can, &f_config_can, &diagnostic_can_bus);
  twai_start_v2(diagnostic_can_bus);
}

#endif
