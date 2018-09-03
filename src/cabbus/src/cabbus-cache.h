#ifndef _CABBUS_CACHE_H_
#define _CABBUS_CACHE_H_

#define CAB_DATA_LOCO_ADDRESS     0x01
#define CAB_DATA_SPEED_DIRECTION  0x02
#define CAB_DATA_FN_GROUP_1       0x04
#define CAB_DATA_FN_GROUP_2       0x08
#define CAB_DATA_FN_GROUP_3       0x10
#define CAB_DATA_FN_GROUP_4       0x20
#define CAB_DATA_FN_GROUP_5       0x40

#define IS_LOCO_ADDRESS_CHANGED(X)       (X & CAB_DATA_LOCO_ADDRESS)
#define IS_SPEED_DIRECTION_CHANGED(X)    (X & CAB_DATA_SPEED_DIRECTION)
#define IS_FN_GROUP_1_CHANGED(X)         (X & CAB_DATA_FN_GROUP_1)
#define IS_FN_GROUP_2_CHANGED(X)         (X & CAB_DATA_FN_GROUP_2)
#define IS_FN_GROUP_3_CHANGED(X)         (X & CAB_DATA_FN_GROUP_3)
#define IS_FN_GROUP_4_CHANGED(X)         (X & CAB_DATA_FN_GROUP_4)
#define IS_FN_GROUP_5_CHANGED(X)         (X & CAB_DATA_FN_GROUP_5)
#define IS_FN_GROUP_CHANGED(X)           (X & (CAB_DATA_FN_GROUP_1 | CAB_DATA_FN_GROUP_2 | CAB_DATA_FN_GROUP_3 | CAB_DATA_FN_GROUP_4 | CAB_DATA_FN_GROUP_5))

typedef struct
{
	uint16_t locoAddress;
	uint8_t speedDirection;
	uint8_t functionGroup1;
	uint8_t functionGroup2;
	uint8_t functionGroup3;
	uint8_t functionGroup4;
	uint8_t functionGroup5;
} CabData;

uint8_t compareCabData(uint8_t addr, CabData* c);
void updateCabData(uint8_t addr, CabData* c);

#endif

