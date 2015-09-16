/*************************************************************
 * 翻译前:
 *  起始符 命令类型 命令长度 校验码 数据... 结束符
 *    0       1       2           3   4       n
 *  0x10以下的字节属于保留字节, 需要转义(n^0x10)并在之前加0x02
 * 翻译方法: 0x02的下一个字节使用0x10异或后则为原始字节
 * 翻译后:
 *  命令类型 命令长度    校验码 数据0 数据1 ...
 *      0       1           2     3     4   ...
 ***************************************************************/

#include "SerialTransfer.h"

//02 esc
//01 lenth ...  03
uint8_t GetDataLength(uint8_t *pu8EscData)
{
	return pu8EscData[2] == 0x02 ? (pu8EscData[3] ^ 0x10) : (pu8EscData[2]);
}

_Bool TranslateData(uint8_t *pu8EscData, uint8_t *pu8Data)
{
	uint8_t i, bool_esc = FALSE, trans_seek = 0, dataLength;
	dataLength = GetDataLength(pu8EscData) + 3;
	for (i = 1; i < SERIAL_BUFF_SIZE; i++)
	{
		uint8_t data = pu8EscData[i];
		// 数据过长
		if (trans_seek >= dataLength)
		{
			// 正常结束
			if (data == END_BYTE)
			{
				//pu8Data[trans_seek++] = u8data;
				return TRUE;
			}
			return FALSE;
		}
		// 开始转义
		if (bool_esc)
		{
			pu8Data[trans_seek++] = (data ^ 0x10);
			bool_esc = FALSE;
			continue;
		}
		// 转义字符
		if (data == 02)
		{
			bool_esc = TRUE;
		}
		else // 非转义字符
		{
			pu8Data[trans_seek++] = data;
		}
	}
	return FALSE;
}

_Bool CheckDATA(uint8_t *pu8Data)
{
	int n;
	uint8_t u8CRC;
	uint8_t length;

	u8CRC = pu8Data[0]; // 数据类型
	length = pu8Data[1]; // 数据长度
	u8CRC ^= length;

	for (n = 3; n < length + 3; n++)
	{
		u8CRC ^= pu8Data[n];
	}

	return (u8CRC == pu8Data[2]);
}
