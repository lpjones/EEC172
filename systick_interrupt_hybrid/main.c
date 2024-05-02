
//*****************************************************************************
//
// Application Name     - TV Remote Decoder (TV Code: Zonda 1355)
// Application Overview - The objective of this application is to demonstrate
//                          GPIO interrupts using SW2 and SW3.
//                          NOTE: the switches are not debounced!
//
//*****************************************************************************

// Standard includes
#include <stdio.h>
#include <stdint.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_nvic.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "prcm.h"
#include "utils.h"
#include "systick.h"
#include "rom_map.h"
#include "spi.h"

// Common interface includes
#include "uart_if.h"

// Pin configurations
#include "pin_mux_config.h"

#include "gpio.h"

#include "oled_test.h"

#include "Adafruit_SSD1351.h"

#include "glcdfont.h"

#include "Adafruit_GFX.h"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
// Interrupt global variables
extern void (* const g_pfnVectors[])(void);

volatile unsigned long SW2_intcount;
volatile unsigned long SW3_intcount;
volatile unsigned char SW2_intflag;
volatile unsigned char SW3_intflag;

// Remote variables
volatile unsigned long Remote_intcount;
volatile unsigned long prev_time = 0;
volatile int32_t bits, prev_bits;
volatile unsigned long delta_time;
volatile unsigned char remote_flag;

volatile unsigned char str[32];
volatile uint8_t idx = 0;
volatile uint8_t cycle = 0;

// some helpful macros for systick:

// the cc3200's fixed clock frequency of 80 MHz
// note the use of ULL to indicate an unsigned long long constant
#define SYSCLKFREQ 80000000ULL

// macro to convert ticks to microseconds
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

// macro to convert microseconds to ticks
#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 40ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 3200000UL

// track systick counter periods elapsed
// if it is not 0, we know the transmission ended
volatile int systick_cnt = 0;
volatile int systick_cnt2 = 0;

extern void (* const g_pfnVectors[])(void);



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
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

// an example of how you can use structs to organize your pin settings for easier maintenance
typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

static PinSetting switch2 = { .port = GPIOA2_BASE, .pin = 0x40};

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void BoardInit(void);

/**
 * Reset SysTick Counter
 */
static inline void SysTickReset(void) {
    // any write to the ST_CURRENT register clears it
    // after clearing it automatically gets reset without
    // triggering exception logic
    // see reference manual section 3.2.1
    HWREG(NVIC_ST_CURRENT) = 1;

    // clear the global count variable
    //systick_cnt = 0;
}

/**
 * SysTick Interrupt Handler
 *
 * Keep track of whether the systick counter wrapped
 */
static void SysTickHandler(void) {
    // increment every time the systick handler fires
    systick_cnt++;
    systick_cnt2++;
}

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
BoardInit(void) {
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

/**
 * Initializes SysTick Module
 */
static void SysTickInit(void) {

    // configure the reset value for the systick countdown register
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);

    // register interrupts on the systick module
    MAP_SysTickIntRegister(SysTickHandler);

    // enable interrupts on systick
    // (trigger SysTickHandler when countdown reaches 0)
    MAP_SysTickIntEnable();

    // enable the systick module itself
    MAP_SysTickEnable();
}

static void remote_handler(void) {
    unsigned long ulStatus, cur_time;
    remote_flag = 1;

    ulStatus = MAP_GPIOIntStatus(GPIOA0_BASE, true);
    MAP_GPIOIntClear(GPIOA0_BASE, ulStatus);
    cur_time = SysTickValueGet();
    delta_time = TICKS_TO_US(SYSTICK_RELOAD_VAL - cur_time);

    if (Remote_intcount >= 32 && systick_cnt * 40000 + delta_time < 50000) {
        systick_cnt = 0;
        systick_cnt2 = 0;
        Remote_intcount++;
        SysTickReset();
        return;
    }


    systick_cnt = 0;
    if (delta_time < 1660) {
        Remote_intcount++;
        bits = bits << 1;
    } else if (delta_time > 1660 && delta_time < 3000){
        Remote_intcount++;
        bits = bits << 1;
        bits += 1;
    } else {
        Remote_intcount = 0;
    }
    SysTickReset();
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//!
//!
//! \return None.
//
//****************************************************************************
int main() {

    unsigned long ulStatus;

    BoardInit();
    
    PinMuxConfig();
    
    // Enable SysTick
    SysTickInit();

    // Initialize UART Terminal
    InitTerm();

    // Clear UART Terminal
    ClearTerm();

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

    fillScreen(BLACK);
    drawCircle(64, 64, 5, WHITE);

    //drawCircle(64, 64, 5, WHITE);

    // End of Initialization

    //
    // Register the interrupt handlers
    //
    MAP_GPIOIntRegister(GPIOA0_BASE, remote_handler); // Remote

    //
    // Configure rising edge interrupts on Remote
    //
    MAP_GPIOIntTypeSet(GPIOA0_BASE, 0x1, GPIO_FALLING_EDGE); // Remote falling edge

    ulStatus = MAP_GPIOIntStatus (GPIOA0_BASE, false);
    MAP_GPIOIntClear(GPIOA0_BASE, ulStatus);            // clear interrupts on GPIOA0


    // Enable Remote interrupts
    MAP_GPIOIntEnable(GPIOA0_BASE, 0x1);

    Message("\t\t****************************************************\n\r");
    Message("\t\t\tPush SW3 or SW2 to generate an interrupt\n\r");
    Message("\t\t ****************************************************\n\r");
    Message("\n\n\n\r");

    uint32_t x = 0, y = 0;

    while (1) {
        if (remote_flag) {
            remote_flag = 0;
            if (Remote_intcount == 32) {
                unsigned long cur_time = SysTickValueGet();
                delta_time = TICKS_TO_US(SYSTICK_RELOAD_VAL - cur_time);
                ////Report("time=%d\r\n", systick_cnt2 * 40000 + delta_time);
                // Same button pressed twice, within 1 second, except for last and 0
                if (bits == prev_bits && systick_cnt2 * 40000 + delta_time < 1000000 && bits != -522139593 && bits != -522155913) {
                    // 2, 3, 4, 5, 6, 8
                    if (bits == -522149793 || bits == -522166113 || bits == -522186513 || bits == -522153873 || bits == -522170193 || bits == -522145713) {
                        if (cycle < 2) {
                            str[idx - 1]++;
                            cycle++;
                        } else {
                            str[idx - 1] -= 2;
                            cycle = 0;
                        }
                    }
                    // 7, 9
                    else if (bits == -522178353 || bits == -522162033) {
                        if (cycle < 3) {
                            str[idx - 1]++;
                            cycle++;
                        } else {
                            str[idx - 1] -= 3;
                            cycle = 0;
                        }
                    }

                } else {
                    cycle = 0;
                    switch (bits) {
                        case -522182433:
                            ////Report("1 Pressed\r\n");
                            break;
                        case -522149793:
                            ////Report("2 Pressed\r\n");
                            str[idx++] = 'A';
                            break;
                        case -522166113:
                            ////Report("3 Pressed\r\n");
                            str[idx++] = 'D';
                            break;
                        case -522186513:
                            //Report("4 Pressed\r\n");
                            str[idx++] = 'G';
                            break;
                        case -522153873:
                            //Report("5 Pressed\r\n");
                            str[idx++] = 'J';
                            break;
                        case -522170193:
                            //Report("6 Pressed\r\n");
                            str[idx++] = 'M';
                            break;
                        case -522178353:
                            //Report("7 Pressed\r\n");
                            str[idx++] = 'P';
                            break;
                        case -522145713:
                            //Report("8 Pressed\r\n");
                            str[idx++] = 'T';
                            break;
                        case -522162033:
                            //Report("9 Pressed\r\n");
                            str[idx++] = 'W';
                            break;
                        case -522155913:
                            //Report("0 Pressed\r\n");
                            str[idx++] = ' ';
                            break;
                        case -522129393:
                            //Report("Mute Pressed\r\n");
                            break;
                        case -522139593:
                            //Report("Last Pressed\r\n");
                            fillRect(idx * 6, y, 6, 7, BLACK);
                            idx--;
                            break;
                        default:
                            break;
                    }
                }
            int i;
            for (i = 0; i < idx; i++) {
                //Report("%c", str[i]);
            }
            if (bits != -522139593) {
                drawChar(idx * 6, y, str[idx - 1], WHITE, BLACK, 1);
            }

            //Report("\r\n");
            prev_bits = bits;
            systick_cnt2 = 0;
            }

        }


    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
