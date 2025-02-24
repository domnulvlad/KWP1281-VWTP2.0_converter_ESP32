#ifndef AUTODEFINES_H
#define AUTODEFINES_H

#ifdef DEBUG_MISC
#define DEBUG(x, ...) printf(x "\n", ##__VA_ARGS__)
#else
#define DEBUG(x, ...)
#endif

#if defined(DEBUG_CAN_TRAFFIC) or defined(DEBUG_KLINE_TRAFFIC) or defined(DEBUG_VWTP_TRAFFIC)
#warning DEBUG_TRAFFIC enabled
#define DEBUG_TRAFFIC(x, ...) printf(x, ##__VA_ARGS__)
#else
#define DEBUG_TRAFFIC(x, ...)
#endif

#endif
