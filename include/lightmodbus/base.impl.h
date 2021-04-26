#include "base.h"
#include <stdlib.h>

LIGHTMODBUS_RET_ERROR modbusDefaultAllocator(uint8_t **ptr, uint16_t size, ModbusBufferPurpose purpose, void *ctx)
{
	(void) purpose;
	(void) ctx;

	// Make sure to handle the case when *ptr = NULL and size = 0
	// We don't want to allocate any memory then
	if (!size)
	{
		free(*ptr);
		*ptr = NULL;
	}
	else
	{
		uint8_t *old_ptr = *ptr;
		*ptr = (uint8_t*)realloc(*ptr, size);
		
		if (!*ptr)
		{
			free(old_ptr);
			return MODBUS_ERROR_ALLOC;
		}
	}

	return MODBUS_OK;
}

/**
	\brief Reads n-th bit from an array

	\param mask A pointer to the array
	\param n Number of the bit to be read
	\returns The bit value
*/
uint8_t modbusMaskRead(const uint8_t *mask, uint16_t n)
{
	return (mask[n >> 3] & (1 << (n & 7))) != 0;
}

/**
	\brief Writes n-th bit in an array

	\param mask A pointer to the array
	\param n Number of the bit to write
	\param value Bit value to be written
*/
void modbusMaskWrite(uint8_t *mask, uint16_t n, uint8_t value)
{
	if (value)
		mask[n >> 3] |= (1 << (n & 7));
	else
		mask[n >> 3] &= ~(1 << (n & 7));
}

/**
	\brief Calculates 16-bit Modbus CRC of provided data

	\param data A pointer to the data to be processed
	\param length Number of bytes, starting at the `data` pointer, to process
	\returns 16-bit Modbus CRC value
*/
uint16_t modbusCRC(const uint8_t *data, uint16_t length)
{
	uint16_t crc = 0xFFFF;

	for (uint16_t i = 0; i < length; i++)
	{
		crc ^= (uint16_t) data[i];
		for (uint8_t j = 8; j != 0; j--)
		{
			if ((crc & 0x0001) != 0)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
				crc >>= 1;
		}
	}

	return crc;
}


/**
	\brief Returns number of bytes necessary to hold given number of bits
	\param n Number of bits
	\returns Number of bytes
*/
inline uint16_t modbusBitsToBytes(uint16_t n)
{
	return (n + 1) >> 3;
}

/**
	\brief Safely reads a little-endian 16-bit word from provided pointer
*/
inline uint16_t modbusRLE(const uint8_t *p)
{
#ifdef LIGHTMODBUS_BIG_ENDIAN
	uint8_t lo = *(p + 1);
	uint8_t hi = *p;
#else
	uint8_t lo = *p;
	uint8_t hi = *(p + 1);
#endif
	return (uint16_t) lo | ((uint16_t) hi << 8);
}

/**
	\brief Safely writes a little-endian 16-bit word to provided pointer
*/
inline uint16_t modbusWLE(uint8_t *p, uint16_t val)
{
#ifdef LIGHTMODBUS_BIG_ENDIAN
	*p = val >> 8;
	*(p + 1) = val;
#else
	*p = val;
	*(p + 1) = val >> 8;
#endif
	return val;
}

/**
	\brief Safely reads a big-endian 16-bit word from provided pointer
*/
inline uint16_t modbusRBE(const uint8_t *p)
{
#ifdef LIGHTMODBUS_BIG_ENDIAN
	uint8_t lo = *(p + 1);
	uint8_t hi = *p;
#else
	uint8_t lo = *p;
	uint8_t hi = *(p + 1);
#endif
	return (uint16_t) lo | ((uint16_t) hi << 8);
}

/**
	\brief Safely writes a big-endian 16-bit word to provided pointer
*/
inline uint16_t modbusWBE(uint8_t *p, uint16_t val)
{
#ifdef LIGHTMODBUS_BIG_ENDIAN
	*p = val >> 8;
	*(p + 1) = val;
#else
	*p = val;
	*(p + 1) = val >> 8;
#endif
	return val;
}