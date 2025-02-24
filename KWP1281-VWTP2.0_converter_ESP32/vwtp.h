#ifndef VWTP_H
#define VWTP_H

// FreeRTOS queue for dynamic diagnostic channel setup messages (and VWTP connection error events)
QueueHandle_t CAN_dynamic_diagnostics_channel_message_queue;

// TP2.0 library
#include "VWTPLib.h"

// VWTP diagnostic instance
// 0x740 is the CAN ID on which VWTP will request to receive from
// This ID is normally used by the Engine module, you might need to change it to avoid problems if opening a diagnostic session with the original ECM and with this converter at the same time.
bool VWTP_sendFunction(unsigned long, uint8_t, uint8_t*);
VWTPLib diag_VWTP(VWTP_sendFunction, 0, 0x740);

bool VWTP_sendFunction(unsigned long can_id, uint8_t can_dlc, uint8_t* data)
{
  // Copy the given frame to an object that can be queued.
  CAN_frame frame;
  frame.can_id = can_id;
  frame.can_dlc = can_dlc;
  memcpy(frame.data, data, can_dlc);
  
  // Queue the frame for sending.
  send_CAN(&frame);
  return true;
}

#ifdef DEBUG_VWTP_TRAFFIC
void VWTPdebugFunction(bool type, unsigned long delta, unsigned long can_id, uint8_t can_dlc, uint8_t* data)
{
  DEBUG_TRAFFIC("[VWTP %s] | %5lu | %03lX | ", (type ? "RECV" : "SEND"), delta, can_id);
  for (uint8_t i = 0; i < can_dlc; i++)
  {
    DEBUG_TRAFFIC("%02X ", data[i]);
  }
  DEBUG_TRAFFIC("\n");
}
#endif

void VWTPterminationFunction(VWTPLib* instance)
{
  DEBUG("VWTP terminated!");
  
  // This app only uses one VWTP instance, it's clear the connection error event is coming from it.
  (void)instance;
  
  // Queue a "frame" with the invalid ID 0, to indicate a connection error event.
  CAN_frame frame;
  frame.can_id = 0;
  xQueueSend(CAN_dynamic_diagnostics_channel_message_queue, (void*)&frame, portMAX_DELAY);
}

// Lookup table from classic KWP1281 ID to VWTP ID
const uint8_t MODULE_ID_TO_LOGICAL_ID[]
{
  0x00, 0x01, 0x02, 0x03, // 00-N/A,                               01-Engine Control Module 1,        02-Transmission Control Module,     03-Brakes 1,
  0x13, 0x31, 0x35, 0x3F, // 04-Steering angle,                    05-Kessy,                          06-Seat Adjustement Passenger Side, 07-Display Control Unit,
  0x2C, 0x20, 0x00, 0x00, // 08-Air Conditioning,                  09-Central Electrics,              0A-N/A,                             0B-N/A,
  0x00, 0x24, 0x58, 0x4F, // 0C-N/A,                               0D-Sliding Door Left,              0E-Media Player Position 1,         0F-Radio Tuner - Digital,
  0x1D, 0x15, 0x00, 0x0B, // 10-Parking Assistance 2,              11-Engine Control Module 2,        12-N/A,                             13-Adaptive Cruise Control,
  0x0C, 0x05, 0x2A, 0x07, // 14-Wheel Dampening Electronics,       15-Airbag,                         16-Steering Column Electronics,     17-Dash Board,
  0x2F, 0x1F, 0x00, 0x1E, // 18-Auxiiliary Parking Heater,         19-Gateway,                        1A-N/A,                             1B-Active Steering,
  0x40, 0x2B, 0x50, 0x5F, // 1C-Vehicle Position Detection,        1D-Driver Identification,          1E-Media Player Position 2,         1F-Radio Tuner - Satelite,
  0x16, 0x16, 0x0A, 0x0D, // 20-High Beam Assistance,              21-Engine Control Module 3,        22-All Wheel Control,               23-Brake Boost,
  0x00, 0x14, 0x30, 0x3E, // 24-N/A,                               25-Immobilizer,                    26-Electronic Roof Control,         27-Display Control Unit Rear,
  0x45, 0x38, 0x00, 0x00, // 28-Climate Control Unit Rear,         29-Light Control Left,             2A-N/A,                             2B-N/A,
  0x00, 0x59, 0x54, 0x4C, // 2C-N/A,                               2D-Voice Amplifier,                2E-Media Player Position 3,         2F-TV Tuner - Digital,
  0x3B, 0x01, 0x18, 0x00, // 30-Special Function 2,                31-Engine Electronics Group,       32-Lock Electronics,                33-N/A,
  0x04, 0x00, 0x26, 0x5B, // 34-Ride Control System,               35-N/A,                            36-Seat Adjustement Driver Side,    37-Navigation,
  0x27, 0x39, 0x00, 0x00, // 38-Roof Electronic Control Module,    39-Light Control Right,            3A-N/A,                             3B-N/A,
  0x1C, 0x3A, 0x62, 0x00, // 3C-Lane Change Assistant,             3D-Special Function,               3E-Media Player Position 4,         3F-N/A,
  0x00, 0x00, 0x22, 0x00, // 40-N/A,                               41-N/A,                            42-Door Electronics Driver Side,    43-N/A,
  0x09, 0x00, 0x21, 0x53, // 44-Steering Assistance,               45-N/A,                            46-Central Module Comfort System,   47-Sound System,
  0x37, 0x00, 0x00, 0x39, // 48-Seat Adjustement Rear Driver Side, 49-N/A,                            4A-N/A,                             4B-Multifunction module,
  0x08, 0x00, 0x5D, 0x28, // 4C-Tire Pressure Monitoring 2,        4D-N/A,                            4E-Control Head Rear Right,         4F-Central Electrics 2,
  0x00, 0x00, 0x23, 0x19, // 50-N/A,                               51-N/A,                            52-Door Electronics Passenger Side, 53-Parking Brake,
  0x00, 0x06, 0x52, 0x57, // 54-N/A,                               55-Headlight Regulation,           56-Radio,                           57-TV Tuner,
  0x00, 0x41, 0x00, 0x44, // 58-N/A,                               59-Towing Protection,              5A-N/A,                             5B-Seat Adjustement Rear Passenger Side,
  0x1A, 0x4B, 0x5E, 0x4D, // 5C-Lane Departure Warning System,     5D-Operation,                      5E-Control Head Rear Left,          5F-Information Control Unit 1,
  0x00, 0x33, 0x24, 0x46, // 60-N/A,                               61-Battery Regulation,             62-Door Electronics Rear Left,      63-Entry Assistance Driver Side,
  0x00, 0x29, 0x36, 0x5C, // 64-N/A,                               65-Tire Pressure Monitoring 1,     66-Seat Adjustement Rear,           67-Voice Control,
  0x32, 0x43, 0x00, 0x17, // 68-Wiper Control Unit,                69-Trailer Function,               6A-N/A,                             6B-Aerodynamics Control Unit
  0x49, 0x34, 0x3D, 0x3C, // 6C-Camera System Rear View,           6D-Deck Lid Control Unit,          6E-Display Control Roof,            6F-Central Module Comfort System 2,
  0x00, 0x0E, 0x25, 0x47, // 70-N/A,                               71-Battery Charger,                72-Door Electronics Rear Right,     73-Entry Assistance Passenger Side,
  0x1B, 0x55, 0x2D, 0x5A, // 74-Chassis Control,                   75-Telematics,                     76-Parking Assistance,              77-Telephone,
  0x25, 0x00, 0x00, 0x00, // 78-Sliding Door Right,                79-N/A,                            7A-N/A,                             7B-N/A,
  0x00, 0x2E, 0x48, 0x4E  // 7C-N/A,                               7D-Auxiliary Heater,               7E-Dash Board Display Unit,         7F-Information Control Unit 2
};

#endif
