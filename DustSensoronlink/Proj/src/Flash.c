/************************************************************************
* flash读写寿命优化算法.
*  4K flash分为 n * BLOCK_SIZE Byte的块,
*  对于外部而言,只看到flash的BLOCK_SIZE-4个字节, 
*  每次只能操作32位的值(ADDR = 0~(BLOCK_SIZE-4)/4)
*  
*  每个块的开头32bit为可用标志, 
*  值为0xEEEEEEEE表示接下来BLOCK_SIZE-4个字节可用,其他值表示无效,
*  程序做无效操作后的值为0x88888888
*  
*  读取操作: 
*      判断第一个块的标志是否为0xEEEEEEEE或者0x88888888,
*      如果是, 则找到第一个0xEEEEEEEE, 并从接下来BLOCK_SIZE-4个字节中取出值
*      如果不是0x88888888 则判断是否为0xFFFFFFFF
*      如果为0xFFFFFFFF,则说明flash为空,需要初始化,否则清空整片flash并初始化
*      初始化之后直接返回默认值
*  写入操作: 
*      判断第一个块的标志是否为0xEEEEEEEE或者0x88888888,
*      如果是, 则找到第一个0xEEEEEEEE,
*      修改0xEEEEEEEE为0x88888888, 初始化下个块并写入数据
*      如果不是0x88888888 则判断是否为0xFFFFFFFF
*      如果为0xFFFFFFFF,则说明flash为空,需要初始化,否则清空整片flash并初始化
*      初始化之后继续写入值
*************************************************************************/
#define FLASH_COMMON

#include "Flash.h"
#include "M051Series.h"

volatile _Bool FirstUse = FALSE;
volatile uint32_t readTemp2;

void Flash_Init(void)
{
    SYS_UnlockReg();
    _FMC_ENABLE_ISP();
    //SYS_LockReg();
}

void Block_Init(uint32_t addr)
{
    FMC_Write(addr, BLOCK_STAT_ENABLED);
}

void Flash_Clear(void)
{
    uint32_t i;
    for (i = FLASH_START; i < FLASH_START + FLASH_SIZE; i += 512 /*512B*/)
        FMC_Erase(i);
}

uint32_t GetBlockAddr(void)
{
    uint32_t i, value;
    
    for (i = FLASH_START; i < FLASH_START + FLASH_SIZE; i += BLOCK_SIZE /*32bit*/)
    {
        value = FMC_Read(i);
        switch(value)
        {
            case BLOCK_STAT_DISABLED:
                continue;
            case BLOCK_STAT_ENABLED:
                return i;
            case BLOCK_STAT_EMPTY:
                Block_Init(FLASH_START);
                FirstUse = TRUE;
                return FLASH_START;
        }
    }
    Flash_Clear();
    Block_Init(FLASH_START);
    FirstUse = TRUE;
    return FLASH_START;
}

uint32_t Flash_Read(uint8_t addr)
{
    uint32_t block_addr = GetBlockAddr();
    if(addr < BLOCK_SIZE / 4 - 1)
    {
        if(FirstUse)
        {
            return 0xff;
        }
        else
        {
            return FMC_Read(block_addr + 4 * addr);
        }
    }
    return 0;
}

void Flash_Write(uint8_t addr, uint32_t value)
{
    uint32_t block_addr = GetBlockAddr();
    if(addr < BLOCK_SIZE / 4 - 1)
    {
        if(FirstUse)
        {
            FMC_Write(block_addr + 4 * (addr + 1), value);
            FirstUse = FALSE;
        }
        else
        {
            uint32_t i, write_addr;
            for(i = block_addr; i < block_addr + BLOCK_SIZE; i += 4)
            {
                write_addr = (i - (block_addr + 4)) / 4;
                if(write_addr != addr)
                {
                    FMC_Write(i + BLOCK_SIZE, FMC_Read(i));
                }
            }
            FMC_Write(block_addr, BLOCK_STAT_DISABLED);
            block_addr += BLOCK_SIZE;
            Block_Init(block_addr);
            FMC_Write(block_addr + (4 * (addr + 1)), value);
        }
    }
}












