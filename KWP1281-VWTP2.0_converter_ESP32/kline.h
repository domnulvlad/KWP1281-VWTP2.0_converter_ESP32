#ifndef KLINE_H
#define KLINE_H

// Pin assignments for the K-Line transceiver
#define K_LINE_RX GPIO_NUM_1
#define K_LINE_TX GPIO_NUM_11

// Serial interface for the K-Line, should not need to be changed.
#define K_line Serial1

// KWP1281 library
#include <KLineKWP1281Lib_ESP32.h>

// KWP1281 diagnostic instance
void KLine_beginFunction(unsigned long);
void KLine_endFunction();
void KLine_sendFunction(uint8_t);
bool KLine_receiveFunction(uint8_t*, unsigned long);
KLineKWP1281Lib diag_K(KLine_beginFunction, KLine_endFunction, KLine_sendFunction, KLine_receiveFunction, K_LINE_TX);

// FreeRTOS event group for alerting the KWP1281 library when a byte is received via the serial interface
EventGroupHandle_t kline_event_group;
#define KLINE_EVENT_GROUP_RX_BIT (1UL << 0)

// Callback executed by the serial event task when serial data is available
void KLineOnReceiveFunction()
{
  // Set the bit in the event group.
  xEventGroupSetBits(kline_event_group, KLINE_EVENT_GROUP_RX_BIT);
}

// Initialize the serial port
void KLine_beginFunction(unsigned long baud)
{
  // Start serial communication.
  K_line.begin(baud, SERIAL_8N1, K_LINE_RX, K_LINE_TX);

  // Attach the callback that executes every time a byte is received.
  K_line.onReceive(KLineOnReceiveFunction, false);
  K_line.setRxFIFOFull(1);
  
  // Clear the bit in the event group.
  xEventGroupClearBits(kline_event_group, KLINE_EVENT_GROUP_RX_BIT);
}

// Stop communication on the serial port
void KLine_endFunction()
{
  K_line.end();
}

// Send a byte
void KLine_sendFunction(uint8_t data)
{
  K_line.write(data);
}

// Receive a byte
bool KLine_receiveFunction(uint8_t *data, unsigned long timeout_ticks)
{
  // If no byte is immediately available, wait.
  if (K_line.available() <= 0)
  {
    if (xEventGroupWaitBits(kline_event_group, KLINE_EVENT_GROUP_RX_BIT, pdTRUE, pdFALSE, timeout_ticks) != pdTRUE)
    {
      return false;
    }
  }

  // A byte is available; store it in the variable passed by pointer.
  *data = K_line.read();
  return true;
}

#ifdef DEBUG_KLINE_TRAFFIC
void KWP1281debugFunction(bool direction, uint8_t message_sequence, uint8_t message_type, uint8_t *data, size_t length)
{
  DEBUG_TRAFFIC("[KLNE %s] | S:%02X T:%02X L:%d | ", (direction ? "RECV" : "SEND"), message_sequence, message_type, length);
  if (length)
  {
    DEBUG_TRAFFIC("D:");
    for (size_t i = 0; i < length; i++)
    {
      DEBUG_TRAFFIC("%02X ", data[i]);
    }
  }
  DEBUG_TRAFFIC("\n");
}
#endif

#endif
