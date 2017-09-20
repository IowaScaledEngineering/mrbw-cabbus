#ifndef CABBUS_CONSTANTS_H
#define CABBUS_CONSTANTS_H

// Size definitions
// CABBUS_BUFFER_SIZE should include length byte + data
// Only 6 are needed (5 byte response + length), but 10 to be safe...
#define CABBUS_BUFFER_SIZE  10

#define CABBUS_BAUD 9600

#endif
