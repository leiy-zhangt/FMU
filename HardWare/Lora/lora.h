#ifndef __LORA_H
#define __LORA_H

#include "sys.h"
#include "usart.h"

#define Lora_channel 0x25

#define LORA_M0 PBout(0)
#define LORA_M1 PBout(1)
#define LORA_NRST PBout(6)
#define LORA_Status PBin(5)


void LORA_WriteCmd(uint8_t addr,uint8_t cmd);
void LORA_Configuration(uint16_t lora_addr,int32_t bound);

#endif

