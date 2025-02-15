/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
//! USCI_A0, SPI 3-Wire Master Incremented Data
//! This example shows how SPI master talks to a 74HC595 shift register.
//! In this example we perform a effect with 10 LEDs, using two 74HC595 in daisy-chain configuration.
//! initialization waits for DCO to stabilize against SMCLK.
//! MCLK = SMCLK = ACLK = ~32.768kHz
//!
//! ATTENTION: This example is based on driverlib MSP430Ware 3.80.14. You must correctly configure
//! your Code Composer Studio/Theia with the correct SDK, it's easy, trust me!
//! 
//!
//! 		    Tested on  MSP430FR2433 
//!                 -----------------
//!            /|\ |                 |
//!             |  |                 |
//!    Master---+->|RST              |
//!                |                 |
//!                |             P1.0|-> 74HC595 Output Enable
//!                |                 |
//!                |             P1.3|-> 74HC595 SRCLK (Latch)
//!                |                 |
//!                |             P1.4|-> Data Out (UCA0SIMO)
//!                |                 |
//!                |             P1.5|-> 74HC595 Clear (Master Reset)
//!                |                 |
//!                |             P1.6|-> Serial Clock Out (UCA0CLK)
//!                |                 |
//!
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - SPI peripheral
//! - GPIO Port peripheral (for SPI pins)
//! - UCA0SIMO
//! - UCA0CLK
//!
//! This example uses the polling mode in SPI, just to simplify application.
//! In a professional application, use Interrupts.
//!
//*****************************************************************************
#include <stdint.h>

#include "cs.h"
#include "eusci_a_spi.h"
#include "gpio.h"
#include "driverlib.h"

#include "intrinsics.h"
#include "l74xx595.h"
#include "msp430fr2433.h"
#include "stdbool.h"

/* Project Macros */

#define XTAL_FREQ           32768

#define HC595_CS_GPIO       GPIO_PORT_P1
#define HC595_CS_Pin        GPIO_PIN3

#define HC595_MR_GPIO       GPIO_PORT_P1
#define HC595_MR_Pin        GPIO_PIN5

#define HC595_OE_GPIO       GPIO_PORT_P1
#define HC595_OE_Pin        GPIO_PIN0

#define SPIB_MOSI_GPIO      GPIO_PORT_P1
#define SPIB_MOSI_Pin       GPIO_PIN4

#define SPIB_SCK_GPIO       GPIO_PORT_P1
#define SPIB_SCK_Pin        GPIO_PIN6

/* Globals */

l74xx595_t l74xx595 = {0};

/* Function Calls */

void _delay_ms(uint32_t ms){
    const uint16_t delay_block = XTAL_FREQ/100;

    // perfor delay in blocks of 10ms
    while (ms > 0){
        _delay_cycles(delay_block);
        ms -= 10;
    }
}

void _platform_init(){
    /* Clock Initialize */
        //Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    // Configure Pins for XIN
    //Set P4.1 and P4.2 as Module Function Input.
    /*

    * Select Port 2
    * Set Pin 0, 1 to input Module Function, (XIN).
    */
    GPIO_setAsPeripheralModuleFunctionInputPin(
    	GPIO_PORT_P2,
    	GPIO_PIN0 + GPIO_PIN1,
    	GPIO_PRIMARY_MODULE_FUNCTION
    );

    //Set external frequency for XT1
    CS_setExternalClockSource(XTAL_FREQ);

    //Select XT1 as the clock source for SMCLK with no frequency divider
    CS_initClockSignal(CS_SMCLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);

    //Start XT1 with no time out
    CS_turnOnXT1(CS_XT1_DRIVE_0);

    // Configure SPI Pins for UCB0CLK UCB0SIMO
    GPIO_setAsPeripheralModuleFunctionInputPin(
    	SPIB_MOSI_GPIO,
    	SPIB_SCK_Pin + SPIB_MOSI_Pin,
    	GPIO_PRIMARY_MODULE_FUNCTION
    );

    /* Configure other outputs of 74HC595 */
    GPIO_setAsOutputPin(HC595_CS_GPIO, HC595_CS_Pin);
    GPIO_setAsOutputPin(HC595_OE_GPIO, HC595_OE_Pin);
    GPIO_setAsOutputPin(HC595_MR_GPIO, HC595_MR_Pin);

    GPIO_setOutputHighOnPin(HC595_OE_GPIO, HC595_OE_Pin);
    GPIO_setOutputHighOnPin(HC595_CS_GPIO, HC595_CS_Pin);
    GPIO_setOutputHighOnPin(HC595_MR_GPIO, HC595_MR_Pin);

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();
}

void _spi_init(){
    //Initialize Master
    EUSCI_A_SPI_initMasterParam param = {0};
    param.selectClockSource = EUSCI_A_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = CS_getSMCLK();
    param.desiredSpiClock = 5000000;
    param.msbFirst = EUSCI_A_SPI_MSB_FIRST;
    param.clockPhase = EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    param.clockPolarity = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    param.spiMode = EUSCI_A_SPI_3PIN;
    EUSCI_A_SPI_initMaster(EUSCI_A0_BASE, &param);

    //Enable SPI module
    EUSCI_A_SPI_enable(EUSCI_A0_BASE);
}

uint8_t _hc595_tx(uint8_t *bff, uint8_t len){
    uint8_t i;
    
    for (i=0 ; i<len ; i++){
        EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, bff[i]);
        // Wait if SPI is busy
        while (EUSCI_A_SPI_isBusy(EUSCI_A0_BASE) == EUSCI_A_SPI_BUSY);
    }
	
    return 0;
}

void _hc595_oe(bool sig){
    if (sig == true){
        GPIO_setOutputHighOnPin(HC595_OE_GPIO, HC595_OE_Pin);
    }
    else{
        GPIO_setOutputLowOnPin(HC595_OE_GPIO, HC595_OE_Pin);
    }
}

void _hc595_mr(bool sig){
    if (sig == true){
        GPIO_setOutputHighOnPin(HC595_MR_GPIO, HC595_MR_Pin);
    }
    else{
        GPIO_setOutputLowOnPin(HC595_MR_GPIO, HC595_MR_Pin);
    }
}

void _hc595_latch(bool sig){
    if (sig == true){
        GPIO_setOutputHighOnPin(HC595_CS_GPIO, HC595_CS_Pin);
    }
    else{
        GPIO_setOutputLowOnPin(HC595_CS_GPIO, HC595_CS_Pin);
    }
}

void main(void)
{
    l74xx595_params_t l74xx595Params = {0};
	uint8_t LedsArr[2] = {0x08, 0x80};
    bool Inc = true;

    /* Initialize GPIOS and System */
    _platform_init();

    /* Initialize the Spi driver */
    _spi_init();

    l74xx595Params.fxnGpioLATCH = _hc595_latch;
    l74xx595Params.fxnGpioMR = _hc595_mr;
    l74xx595Params.fxnGpioOE = _hc595_oe;
    l74xx595Params.fxnMtxLock = NULL;
    l74xx595Params.fxnMtxUnlock = NULL;
    l74xx595Params.fxnSpiTransmit = _hc595_tx;
    l74xx595Params.u8NumberOfDevices = 2;
    l74xx595Params.pu8ExtBuffer = NULL;
    l74xx595_init(&l74xx595, &l74xx595Params);

    while(1){
        l74xx595_write_byte(&l74xx595, 0, LedsArr[0]);
        l74xx595_write_byte(&l74xx595, 1, LedsArr[1]);
        if (Inc == true){
            LedsArr[0] <<= 1;
            LedsArr[1] >>= 1;
            if (LedsArr[1] == 0x08){
                Inc = false;
            }
        }
        else{
            LedsArr[0] >>= 1;
            LedsArr[1] <<= 1;
            if (LedsArr[1] == 0x80){
                Inc = true;
            }
        }

        _delay_ms(250);
    }
}


