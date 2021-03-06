#ifndef CABBUS_CONSTANTS_H
#define CABBUS_CONSTANTS_H

// Size definitions
// For transmit, only 5 are needed for a 5 byte response
// For receive, CABBUS_BUFFER_SIZE includes ping, command, and data
// Command 0xCB appears to the the longest: ping+cmd+8data+loc = 11
#define CABBUS_BUFFER_SIZE  16

#define CABBUS_BAUD 9600
#define XPRESSNET_BAUD 62500

#endif
