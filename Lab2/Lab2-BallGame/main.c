//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - SPI Demo
// Application Overview - The demo application focuses on showing the required 
//                        initialization sequence to enable the CC3200 SPI 
//                        module in full duplex 4-wire master and slave mode(s).
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup SPI_Demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "gpio.h"

// Common interface includes
#include "uart_if.h"
#include "pin_mux_config.h"

#include "oled_test.h"

#include "Adafruit_SSD1351.h"

#include "glcdfont.h"

#include "Adafruit_GFX.h"

#include "i2c_if.h"



#define APPLICATION_VERSION     "1.4.0"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}
#define SUCCESS                 0

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************




//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}



int
GetTilt(unsigned char ucDevAddr, unsigned char ucRegOffset, unsigned char ucRdLen, unsigned char *aucRdDataBuf)
{

    //
    // Write the register address to be read from.
    // Stop bit implicitly assumed to be 0.
    //
    RET_IF_ERR(I2C_IF_Write(ucDevAddr,&ucRegOffset,1,0));

    //
    // Read the specified length of data
    //
    RET_IF_ERR(I2C_IF_Read(ucDevAddr, &aucRdDataBuf[0], ucRdLen));

    return SUCCESS;
}


void fastCircle(int x, int y, int r, unsigned int color, unsigned int background, unsigned int **prev_colors) {
    int row = 0, col = 0;
    for (row = y - r; row < y + r; row++) {
        for (col = x - r; col < x + r; col++) {
            if (prev_colors[row - y + r][col - x + r] == background && (row - y) * (row - y) + (col - x) * (col - x) < r * r) {
                drawPixel(col, row, color);
                prev_colors[row - y + r][col - x + r] = color;
            }
        }
    }
}

void fastCircleUpdate(int x, int y, int r, unsigned int color, int prev_x, int prev_y, unsigned int background, unsigned int **prev_colors) {
    // Get rid of old ball pixels that won't overlap with the new ball
    int row, col;
    for (row = prev_y - r; row < prev_y + r; row++) {
        for (col = prev_x - r; col < prev_x + r; col++) {
            // Check if the pixel was part of the old ball but not part of the new ball
            if ((row - prev_y) * (row - prev_y) + (col - prev_x) * (col - prev_x) < r * r &&
                !((row - y) * (row - y) + (col - x) * (col - x) < r * r)) {
                drawPixel(col, row, background);
                prev_colors[row - (prev_y - r)][col - (prev_x - r)] = background;
            }
        }
    }

    // Draw new ball pixels that weren't part of the old ball
    for (row = y - r; row < y + r; row++) {
        for (col = x - r; col < x + r; col++) {
            // Check if the pixel is part of the new ball but wasn't part of the old ball
            if ((row - y) * (row - y) + (col - x) * (col - x) < r * r &&
                !((row - prev_y) * (row - prev_y) + (col - prev_x) * (col - prev_x) < r * r)) {
                drawPixel(col, row, color);
                prev_colors[row - (y - r)][col - (x - r)] = color;
            }
        }
    }


}

//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI and I2C lines.
    //
    PinMuxConfig();

    I2C_IF_Open(I2C_MASTER_MODE_FST);

    //
    // Enable the SPI module clock
    //

    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    MAP_PRCMPeripheralReset(PRCM_GSPI);
    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);


    // Configure SPI interface

    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));



    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    Adafruit_Init();

    // End of Initialization


    fillScreen(BLACK);

    setCursor(20, 64);
    Outstr("Press SW3 to Play\0");
    while (!GPIOPinRead(GPIOA1_BASE, GPIO_PIN_5)) {
        //printf("Still waiting %d\n", GPIOPinRead(GPIOA1_BASE, GPIO_PIN_5));
    }

    MAP_UtilsDelay(4000000);
    unsigned char aucRdDataBuf[6];



    int radius = 5;

    //unsigned int prev_colors[2 * radius][2 * radius];
    unsigned int **prev_colors = (unsigned int **)malloc(2 * radius * sizeof(unsigned int *));


    // Initialize ball colors as black
    int i = 0, j = 0;
    for (i = 0; i < 2 * radius; i++) {
        prev_colors[i] = (unsigned int *)malloc(2 * radius * sizeof(unsigned int));
        for (j = 0; j < 2 * radius; j++) {
            prev_colors[i][j] = BLACK;
        }
    }
    // X and Y tilt values from accelerometer
    int x_tilt = 0;
    int y_tilt = 0;

    // Location of ball on screen
    int x_ball = 64;
    int y_ball = 64;

    int x_ball_prev = x_ball;
    int y_ball_prev = y_ball;
    fastCircle(x_ball, y_ball, radius, WHITE, BLACK, prev_colors);

    while (1) {
        GetTilt(0x18, 0x2, 6, aucRdDataBuf);


        //printf("x=%i, y=%i, z=%d\n", aucRdDataBuf[1], aucRdDataBuf[3], aucRdDataBuf[5]);

        if (aucRdDataBuf[1] > 127) {
            x_tilt = aucRdDataBuf[1] - 256;
        } else {
            x_tilt = aucRdDataBuf[1];
        }

        if (aucRdDataBuf[3] > 127) {
            y_tilt = aucRdDataBuf[3] - 256;
        } else {
            y_tilt = aucRdDataBuf[3];
        }

        x_ball_prev = x_ball;
        y_ball_prev = y_ball;

        // Get new x, y for ball
        x_ball -= x_tilt / 2;
        y_ball += y_tilt / 2;

        // Check if at edge of screen
        if (x_ball + radius > 128) {
            x_ball = 128 - radius;
        } else if (x_ball - radius < 0) {
            x_ball = radius;
        }

        if (y_ball + radius > 128) {
            y_ball = 128 - radius;
        } else if (y_ball - radius < 0) {
            y_ball = radius;
        }

        // Draw new ball at x, y
        fastCircleUpdate(x_ball, y_ball, radius, WHITE, x_ball_prev, y_ball_prev, BLACK, prev_colors);


        //printf("x_tilt=%d, y_tilt=%d\n", x_tilt, y_tilt);

    }



}

