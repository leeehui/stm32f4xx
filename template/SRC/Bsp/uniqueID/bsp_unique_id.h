#ifndef BSP_UNIQUE_ID__
#define BSP_UNIQUE_ID__

#include <stdint.h>

//#define UNIQUE_ID_BASE 0x1FFFF7E8
#define UNIQUE_ID_BASE 0x1FFF7A10

void get_unique_id(uint32_t* id);

#endif

