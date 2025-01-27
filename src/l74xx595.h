/*
 * 74xx165.h
 *
 *  Created on: Aug 12, 2024
 *      Author: pablo-jean
 */

#ifndef l74XX165_H_
#define l74XX165_H_

/*
 * Includes
 */

#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <malloc.h>

#include "l74xx595_defs.h"


/*
 * Macros
 */

#define l74XX595_VALUE		0

/*
 * Enumerates
 */

typedef enum{
	L74XX595_OK,
	L74XX595_FAIL
}l74xx595_err_e;

/*
 * Typedefs
 */

typedef void (*l74xx595_gpio_t)(bool sig);
typedef void (*l74xx595_mtx_t)(void);
typedef uint8_t (*l74xx595_spi_t)(uint8_t *bff, uint8_t len);

/*
 * Structs and Unions
 */

typedef struct{
	struct{
		l74xx595_spi_t fxnSpiTransmit;
		l74xx595_gpio_t fxnGpioLATCH;
		l74xx595_gpio_t fxnGpioMR;
		l74xx595_gpio_t fxnGpioOE;
		l74xx595_mtx_t fxnMtxLock;
		l74xx595_mtx_t fxnMtxUnlock;
	}fxns;
	bool bInitialized;
	uint8_t u8NumberOfDevices;
	uint8_t *pu8IntBuffer;
}l74xx595_t;

/**
 * @brief
 *
 */
typedef struct{
	l74xx595_spi_t fxnSpiTransmit; /** comment */
	l74xx595_gpio_t fxnGpioLATCH;
	l74xx595_gpio_t fxnGpioMR;
	l74xx595_gpio_t fxnGpioOE;
	l74xx595_mtx_t fxnMtxLock;
	l74xx595_mtx_t fxnMtxUnlock;
	uint8_t u8NumberOfDevices;
	uint8_t *pu8ExtBuffer;
}l74xx595_params_t;

/*
 * Publics Function Prototypes
 */

/**
 * @fn l74xx595_err_e l74xx595_init(l74xx595_t*, l74xx595_params_t*)
 * @brief Initialize the 74xx595 handler, the functions checks for errors.
 *
 * @pre none
 * @post allow you to use l74xx595_read_byte and l74xx595_read_bit routines.
 * @param handler ....
 * @param params ...
 * @return ....
 */
l74xx595_err_e l74xx595_init(l74xx595_t *handler, l74xx595_params_t *params);

l74xx595_err_e l74xx595_write_byte(l74xx595_t *handler, uint8_t Index, uint8_t WriteByte);

l74xx595_err_e l74xx595_write_bit(l74xx595_t *handler, uint8_t Index, uint8_t bit, bool WriteBit);

#endif /* l74XX165_H_ */
