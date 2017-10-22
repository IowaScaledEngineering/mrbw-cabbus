/*************************************************************************
Title:    CABBUS-CACHE Cab Bus Caching Functions
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     cabbus-cache.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2017 Michael Petersen & Nathan Holmes
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*************************************************************************/

#include <stdlib.h>
#include <stdint.h>
#include "cabbus-cache.h"

CabData cache[256];

uint8_t compareCabData(uint8_t addr, CabData* c)
{
	uint8_t delta = 0;
	
	if(c->locoAddress != cache[addr].locoAddress)
		delta |= CAB_DATA_LOCO_ADDRESS;

	if(c->speedDirection != cache[addr].speedDirection)
		delta |= CAB_DATA_SPEED_DIRECTION;
	
	if(c->functionGroup1 != cache[addr].functionGroup1)
		delta |= CAB_DATA_FN_GROUP_1;
	
	if(c->functionGroup2 != cache[addr].functionGroup2)
		delta |= CAB_DATA_FN_GROUP_2;
	
	if(c->functionGroup3 != cache[addr].functionGroup3)
		delta |= CAB_DATA_FN_GROUP_3;
	
	if(c->functionGroup4 != cache[addr].functionGroup4)
		delta |= CAB_DATA_FN_GROUP_4;
	
	if(c->functionGroup5 != cache[addr].functionGroup5)
		delta |= CAB_DATA_FN_GROUP_5;
	
	return delta;
}

void updateCabData(uint8_t addr, CabData* c)
{
	cache[addr] = *c;
}

