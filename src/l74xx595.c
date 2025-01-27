/*
 * 75xx165.c
 *
 *  Created on: Aug 12, 2024
 *      Author: pablo-jean
 */

/*
 * Include
 */

#include "l74xx595.h"

/*
 * Privates
 */

/* Macros */
#define _OE_ENABLED			false
#define _OE_DISABLED		true
#define _MR_ENABLED			false
#define _MR_DISABLED		true
#define _LATCH_ENABLED		true
#define _LATCH_DISABLED		false

#define _SUCCESS			0

/* Functions */

static void _mtx_lock(l74xx595_t *handler){
	if (handler->fxns.fxnMtxLock != NULL){
		handler->fxns.fxnMtxLock();
	}
}

static void _mtx_unlock(l74xx595_t *handler){
	if (handler->fxns.fxnMtxUnlock != NULL){
		handler->fxns.fxnMtxUnlock();
	}
}

static void _set_latch(l74xx595_t *handler, bool Signal){
	assert(handler->fxns.fxnGpioLATCH != NULL);
	handler->fxns.fxnGpioLATCH(Signal);
}

static void _set_mr(l74xx595_t *handler, bool Signal){
	if(handler->fxns.fxnGpioMR != NULL){
		handler->fxns.fxnGpioMR(Signal);
	}
}

static void _set_oe(l74xx595_t *handler, bool Signal){
	if(handler->fxns.fxnGpioOE != NULL){
		handler->fxns.fxnGpioOE(Signal);
	}
}

static uint8_t _spi_tx(l74xx595_t *handler, uint8_t *Buffer, uint8_t len){
	assert(handler->fxns.fxnSpiTransmit != NULL);
	return handler->fxns.fxnSpiTransmit(Buffer, len);
}

/*
 * Publics
 */

l74xx595_err_e l74xx595_init(l74xx595_t *handler, l74xx595_params_t *params){
	assert(handler != NULL);
	assert(params != NULL);
	assert(params->fxnGpioLATCH != NULL);
	assert(params->fxnSpiTransmit != NULL);
	assert(params->u8NumberOfDevices > 0);

	if (handler->bInitialized == true){
		return L74XX595_OK;
	}

	handler->fxns.fxnGpioLATCH = params->fxnGpioLATCH;
	handler->fxns.fxnGpioMR = params->fxnGpioMR;
	handler->fxns.fxnGpioOE = params->fxnGpioOE;
	handler->fxns.fxnSpiTransmit = params->fxnSpiTransmit;
	handler->fxns.fxnMtxLock = params->fxnMtxLock;
	handler->fxns.fxnMtxUnlock = params->fxnMtxUnlock;
	handler->u8NumberOfDevices = params->u8NumberOfDevices;

	if (params->pu8ExtBuffer != NULL){
		handler->pu8IntBuffer = params->pu8ExtBuffer;
	}
	else{
		handler->pu8IntBuffer = (uint8_t*)malloc(handler->u8NumberOfDevices);
	}
	assert(handler->pu8IntBuffer != NULL);

	_mtx_lock(handler);
	_set_mr(handler, _MR_DISABLED);
	_set_oe(handler, _OE_ENABLED);
	_set_latch(handler, _LATCH_DISABLED);
	_mtx_unlock(handler);

	handler->bInitialized = true;

	return L74XX595_OK;
}

l74xx595_err_e l74xx595_write_byte(l74xx595_t *handler, uint8_t Index, uint8_t WriteByte){
	uint8_t u8Ret;

	assert(handler != NULL);
	assert(handler->bInitialized == true);
	assert(Index < handler->u8NumberOfDevices);

	handler->pu8IntBuffer[Index] = WriteByte;

	_mtx_lock(handler);

	_set_latch(handler, _LATCH_DISABLED);
	u8Ret = _spi_tx(handler, handler->pu8IntBuffer, handler->u8NumberOfDevices);
	_set_latch(handler, _LATCH_ENABLED);

	_mtx_unlock(handler);

	if (u8Ret != _SUCCESS){
		return L74XX595_FAIL;
	}

	return L74XX595_OK;
}

l74xx595_err_e l74xx595_write_bit(l74xx595_t *handler, uint8_t Index, uint8_t bit, bool WriteBit){
	assert(handler != NULL);
	assert(handler->bInitialized == true);
	assert(Index < handler->u8NumberOfDevices);
	assert(bit < 8);

	if (WriteBit == 1){
		handler->pu8IntBuffer[Index] |= (1 << bit);
	}
	else{
		handler->pu8IntBuffer[Index] &= ~(1 << bit);
	}

	return l74xx595_write_byte(handler, Index, handler->pu8IntBuffer[Index]);
}

