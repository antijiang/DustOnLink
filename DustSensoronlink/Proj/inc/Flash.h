#ifndef __FLASH_H__
#define __FLASH_H__


#include <stdio.h>
#include "FMC.h"

#define FLASH_START                     0x0001f000  // Flash开始地址
#define FLASH_SIZE                      0x00001000  // 4K
        
#ifdef FLASH_COMMON     
#define BLOCK_SIZE                      32          // 每个块的大小
#define BLOCK_STAT_ENABLED              0xEEEEEEEE  // 可用
#define BLOCK_STAT_WRITING              0xCCCCCCCC  // 正在写入
#define BLOCK_STAT_DISABLED             0x88888888  // 已过期
#define BLOCK_STAT_EMPTY                0xFFFFFFFF  // 已擦除
#endif

#ifdef SIMPLE_FLASH
//#define USED_SIZE                       (0x3C+5*4)        // 可用字节数, 增加将延长CRC计算时间, 数据数*4
#define BLOCK_SIZE                      0x200       // 块大小
#define NEW_BLOCK_ADDR                  0x0001f000  // 最新数据块地址
#define BAK_BLOCK_ADDR                  NEW_BLOCK_ADDR + BLOCK_SIZE // 备份块地址
#endif

void Flash_Init(void);
void Flash_Write(uint8_t addr, uint32_t value);
_Bool Flash_Read(uint8_t addr, uint32_t* dist);

#endif
