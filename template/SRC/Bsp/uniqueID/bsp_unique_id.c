

#include "bsp_unique_id.h"



void get_unique_id(uint32_t* id)
{
	*id = *(volatile uint32_t*)(UNIQUE_ID_BASE);
	*(id+1) = *(volatile uint32_t*)(UNIQUE_ID_BASE+4);
	*(id+2) = *(volatile uint32_t*)(UNIQUE_ID_BASE+8);
}



