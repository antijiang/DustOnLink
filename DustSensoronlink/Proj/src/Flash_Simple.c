#define SIMPLE_FLASH

#include "Flash.h"
#include "M051Series.h"

#include "Dustsensor.h"
#ifdef	DEBUG_FLASH
#define	DPRINTF(a) printf a
#else
#define	DPRINTF(a) 
#endif

uint32_t Using_Values[USED_SIZE / 4];
_Bool ReadRequed = TRUE;

void Flash_Init(void)
{
    uint8_t i;
    for (i = 0; i < USED_SIZE / 4; i++)
    {
        Using_Values[i] = 0xFFFFFFFF;
    }
    SYS_UnlockReg();
    _FMC_ENABLE_ISP();
    //SYS_LockReg();
}

void Block_Clear(uint32_t block)
{
    FMC_Erase(block);
}

void Block_Read(uint32_t block)
{
    uint8_t i;
    for (i = 0; i < USED_SIZE / 4; i++)
    {
        Using_Values[i] = FMC_Read(i * 4 + block);
    }
}

void Block_Write(uint32_t block, uint32_t* values)
{
    uint8_t i;
    uint32_t CRC = USED_SIZE;
    for (i = 0; i < USED_SIZE / 4; i++)
    {
        CRC ^= Using_Values[i];
        FMC_Write(block + i * 4, Using_Values[i]);
    }
    FMC_Write(block + USED_SIZE, CRC);
}

_Bool Flash_Read(uint8_t addr, uint32_t* dist)
{
    if (addr < USED_SIZE / 4)
    {
        if (ReadRequed)
        {
            uint8_t i;
            uint32_t CRC = USED_SIZE;
            Block_Read(NEW_BLOCK_ADDR);
            for (i = 0; i < USED_SIZE / 4; i++)
            {
                CRC ^= Using_Values[i];
            }
            if (CRC == FMC_Read(NEW_BLOCK_ADDR + USED_SIZE))
            {
                *dist = FMC_Read(NEW_BLOCK_ADDR + addr * 4);
                /*if (*dist == 0xffff)
                {
                return FALSE;
                }*/
                ReadRequed = FALSE;
                return TRUE;
            }
            else
            {
                FMC_Erase(NEW_BLOCK_ADDR);
                Flash_Init();
							
                DPRINTF(("Area 1 error! \n"));
							
            }
            Block_Read(BAK_BLOCK_ADDR);
            for (i = 0; i < USED_SIZE / 4; i++)
            {
                CRC ^= Using_Values[i];
            }
            if (CRC == FMC_Read(BAK_BLOCK_ADDR + USED_SIZE))
            {
                Block_Clear(NEW_BLOCK_ADDR);
                for (i = 0; i < USED_SIZE / 4; i++)
                {
                    FMC_Write(NEW_BLOCK_ADDR + i * 4, Using_Values[i]);
                }
                FMC_Write(NEW_BLOCK_ADDR + USED_SIZE, CRC);
                *dist = FMC_Read(BAK_BLOCK_ADDR + addr * 4);
                /*if (*dist == 0xffff)
                {
                    return FALSE;
                }*/
                ReadRequed = FALSE;
                return TRUE;
            }
            else
            {
                FMC_Erase(BAK_BLOCK_ADDR);
                Flash_Init();
                 DPRINTF(("Area 2 CRC error! \n"));
            }
        }
        else
        {
            *dist = Using_Values[addr];
            return TRUE;
        }
    }
    else
    {
         DPRINTF(("Err: Addr: %d  overrange!\n", addr));
    }
    return FALSE;
}

void Flash_Write(uint8_t addr, uint32_t value)
{
    if (addr < USED_SIZE / 4)
    {
        /* 缓存Flash中其他位置的数据 */
        uint32_t dist = 0;
        Flash_Read(0, &dist);

        Using_Values[addr] = value;
        FMC_Erase(NEW_BLOCK_ADDR);
        Block_Write(NEW_BLOCK_ADDR, Using_Values);
        FMC_Erase(BAK_BLOCK_ADDR);
        Block_Write(BAK_BLOCK_ADDR, Using_Values);
    }
}





