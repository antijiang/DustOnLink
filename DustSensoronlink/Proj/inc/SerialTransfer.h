#ifndef __SERIALTRANSFER_H__
#define __SERIALTRANSFER_H__

#include "M051Series.h"

#define SERIAL_BUFF_SIZE        (50)            // 串口缓冲区大小
#define START_BYTE              (0x01)          // 起始符
#define END_BYTE                (0x03)          // 结束符
#define ESC_BYTE                (0x02)          // 转义符
#define END_BYTE1                (0x0A)          // 结束符

#if 1
#define	STX_BYTE		0x02
#define	ETX_BYTE		0x03
#define STRING_END	0x00
#else
#define	STX_BYTE		0xA5
#define	ETX_BYTE		0x0A
#define STRING_END	0x00
#endif

#define	CR_BYTE	0x0d

uint8_t GetDataLength(uint8_t *pu8EscData);
_Bool TranslateData(uint8_t *pu8EscData, uint8_t *pu8Data);
_Bool CheckDATA(uint8_t *pu8Data);

#endif

