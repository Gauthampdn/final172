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
// Application Name     -   SSL Demo
// Application Overview -   This is a sample application demonstrating the
//                          use of secure sockets on a CC3200 device.The
//                          application connects to an AP and
//                          tries to establish a secure connection to the
//                          Google server.
// Application Details  -
// docs\examples\CC32xx_SSL_Demo_Application.pdf
// or
// http://processors.wiki.ti.com/index.php/CC32xx_SSL_Demo_Application
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup ssl
//! @{
//
//*****************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"

#include "hw_ints.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "spi.h"


//Common interface includes
#include "pinmux.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"
#include "i2c_if.h"

#include "Adafruit_GFX.h"
#include "glcdfont.h"
#include "oled_test.h"
#include "Adafruit_SSD1351.h"

#include "adc.h"
#include "hw_adc.h"


// Custom includes
#include "utils/network_utils.h"

// DS1307 RTC defines
#define DS1307_I2C_ADDR    0x68  // 7-bit I2C address

// Compile-time default time: Jun 2, 2025, 11:20:10, Monday (2)
#define INIT_HOUR        11
#define INIT_MINUTE      20
#define INIT_SECOND      10
#define INIT_DAYOFWEEK    2    // 1 = Sunday … 7 = Saturday
#define INIT_DATE         2
#define INIT_MONTH        6
#define INIT_YEAR        25   // DS1307 stores "25" for 2025

// Menu defines
#define MENU_FEEDING_HISTORY 0
#define MENU_SETTINGS 1
#define MENU_BACK 2
#define MENU_ITEMS 3

// Feeding log defines
#define MAX_FEEDING_RECORDS 10

// Structure to store feeding timestamp
typedef struct {
    unsigned char hour;
    unsigned char minute;
    unsigned char second;
    unsigned char date;
    unsigned char month;
    unsigned char year;
    unsigned char valid;  // 1 if record is valid, 0 if empty
} FeedingRecord;

// Global feeding log array
static FeedingRecord feedingLog[MAX_FEEDING_RECORDS];
static int feedingLogIndex = 0;  // Current index for circular buffer

// Settings defines
#define PORTION_SMALL 0
#define PORTION_MEDIUM 1
#define PORTION_LARGE 2

#define SETTINGS_PORTION_SIZE 0
#define SETTINGS_DAILY_LIMIT 1
#define SETTINGS_BACK 2
#define SETTINGS_MENU_ITEMS 3

// Global settings variables
static int currentPortionSize = PORTION_MEDIUM;  // Default to medium
static int dailyFeedingLimit = 2;  // Default to 2 times per day
static int todayFeedingCount = 0;  // Count of feedings today
static unsigned char lastFeedingDay = 0;  // Track the last day we fed

//  function delays 3*ulCount cycles
static void delay(unsigned long ulCount){
    int i;

  do{
    ulCount--;
        for (i=0; i< 65535; i++) ;
    }while(ulCount);
}


//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                2    /* Current Date */
#define MONTH               6     /* Month 1-12 */
#define YEAR                2025  /* Current year */
#define HOUR                15    /* Time - hours */
#define MINUTE              26    /* Time - minutes */
#define SECOND              0     /* Time - seconds */




#define APPLICATION_NAME      "SSL"
#define APPLICATION_VERSION   "SQ24"
#define SERVER_NAME           "as38mk89cstxi-ats.iot.us-west-1.amazonaws.com" // CHANGE ME
#define GOOGLE_DST_PORT       8443


#define SERVO_PIN_MASK      0x1   // PA0
#define DELAY_COUNTS_PER_US 13

#define SPI_IF_BIT_RATE  1000000
#define TR_BUFF_SIZE     100

#define SW3_PRESSED          (GPIOPinRead(GPIOA1_BASE, 0x20) == 0x20)


#define POSTHEADER "POST /things/Gautham_CC3200_board/shadow HTTP/1.1\r\n"             // CHANGE ME
#define HOSTHEADER "Host: as38mk89cstxi-ats.iot.us-west-1.amazonaws.com\r\n"  // CHANGE ME
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

#define DATA1 "{" \
            "\"state\": {\r\n"                                              \
                "\"desired\" : {\r\n"                                       \
                    "\"var\" :\""                                           \
                        "Dispensing the food"                               \
                        "\"\r\n"                                            \
                "}"                                                         \
            "}"                                                             \
        "}\r\n\r\n"

#define RED     0xF800
#define GREEN   0x07E0

// Dog ASCII art data (128 lines)
const char* dogArt[128] = {
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "                                                                                                   ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                732555555221                                        125555555227                                                                      ",
    "                          149696666666666666666699941                        149969666666666666666699941                                                               ",
    "                      3996666699696445555446996999966666663  73255237  3666666666999966445555446699699666663                                                           ",
    "                  3696996665                    79666666999999996699699966666667                    5666666663                                                        ",
    "                5966996                      36966666669699666666999999966666669961                      6996695                                                      ",
    "              1966993                      4966699962                      2699969664                      3996991                                                    ",
    "              699961                      69966641                              14699696                      199996                                                   ",
    "            796665                      399969                                      696663                      469997                                                 ",
    "          79969                       26669                                          69692                       66697                                                ",
    "          9999                       36966                                            69693                       9999                                                ",
    "          6666                        6969                                              6666                        6996                                               ",
    "          9665                       46693                                              36994                       4666                                               ",
    "        49691                       6669                                                9999                       19664                                              ",
    "        6999                        9666                                                6999                        9666                                              ",
    "        9999                       26662                                                26993                       6669                                              ",
    "        9999                       5966                                                  9695                       6666                                              ",
    "        56693                      4666                                                  9664                      39665                                              ",
    "        76662                      6996                                                  6666                      29667                                              ",
    "          9664                      9994                                                  4666                      4999                                               ",
    "          6666                      9995                                                  5666                      6966                                               ",
    "          4669                     29993                                                  36692                     9994                                               ",
    "          26697                    69967                                                   6696                    79992                                               ",
    "          36695                    9999                                                    6699                    59993                                               ",
    "          6696                    6666                                                    6999                    6999                                                ",
    "          6696                   26693                                                    39992                   6699                                                ",
    "          69997                  6666                                                      9996                  76696                                                ",
    "          79995                 76664        3999962                        2699963        46997                 46667                                                ",
    "            46667                6969        466666665                      566666694        9669                79664                                                 ",
    "            96997              49995        999966996                      666666669        29964              79969                                                  ",
    "              69964            99664         769966663                      266666697         59699            49666                                                   ",
    "              996664       5966692            36992                          26963            3999962       499699                                                    ",
    "                79669699669999965                                                                59699666969999997                                                     ",
    "                  26966666965                                                                      56666669962                                                        ",
    "                      76996                                                                            66667                                                           ",
    "                      29961                                354669966453                                16662                                                           ",
    "                      4999                              699666666666699696                              9664                                                           ",
    "                      6996                             69666666666666666966                             6666                                                           ",
    "                      6666                             19666666666666666993                             6666                                                           ",
    "                      4999                               9966666666666666                               6664                                                           ",
    "                      5996                                 169666669961                                 6665                                                           ",
    "                      16996                                   299662                                   69661                                                           ",
    "                      99997                           42      9666      54                           79966                                                            ",
    "                        6666                       4           5695          74                       6999                                                             ",
    "                        26966                                   69                                   66662                                                             ",
    "                        66996                                  96                                  66666                                                              ",
    "                          56699                                 66                                 66965                                                               ",
    "                          196665                              5965                              596991                                                                ",
    "                            699994                        169996999961                        499996                                                                  ",
    "                              19966994                1669966996669669966661                46996991                                                                   ",
    "                                66999669665525546666666699997      76999966666664552546966669996                                                                      ",
    "                                    569999999999999969962                266696699999999999695                                                                         ",
    "                                        3544444527                            7254444453                                                                              ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      ",
    "                                                                                                                                                                      "
};


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//                 GLOBAL VARIABLES -- End: df
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static int set_time();
static void BoardInit(void);
static int http_post(int);
static void InitializeADC(void);
static unsigned long ReadADCChannel2(void);
static unsigned long ReadADCChannel3(void);
static void drawDogArt(void);
static void displayMessage(const char* message);
static void InitializeI2C(void);
static unsigned char dec_to_bcd(unsigned char val);
static unsigned char bcd_to_dec(unsigned char bcd);
static void DS1307_ReadAllRegs(unsigned char out[7]);
static void displayDateTimeOnOLED(void);
static unsigned long ReadADCChannel1(void);
static void displayMenu(int selectedOption);
static void recordFeedingTime(void);
static void displayFeedingHistory(void);
static void initializeFeedingLog(void);
static void displaySettingsMenu(int selectedOption);
static void handleSettingsNavigation(void);
static void checkDailyFeedingReset(void);
static int canFeedToday(void);
static void adjustedPulseSpeed(void);

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void BoardInit(void) {
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


static void pulse_speed(int us_high) {
    MAP_GPIOPinWrite(GPIOA3_BASE, SERVO_PIN_MASK, SERVO_PIN_MASK);
    UtilsDelay(DELAY_COUNTS_PER_US * us_high);
    MAP_GPIOPinWrite(GPIOA3_BASE, SERVO_PIN_MASK, 0);
    UtilsDelay(DELAY_COUNTS_PER_US * (20000 - us_high));
}

//*****************************************************************************
//
//! Initialize ADC
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void InitializeADC(void) {
    // Enable ADC channel 1 (PIN_58)
    MAP_ADCChannelEnable(ADC_BASE, ADC_CH_1);
    
    // Enable ADC channel 2 (PIN_59)
    MAP_ADCChannelEnable(ADC_BASE, ADC_CH_2);
    
    // Enable ADC channel 3 (PIN_60)
    MAP_ADCChannelEnable(ADC_BASE, ADC_CH_3);
    
    // Configure internal timer for time stamping (optional)
    MAP_ADCTimerConfig(ADC_BASE, 80000);
    MAP_ADCTimerEnable(ADC_BASE);
    
    // Enable the ADC module
    MAP_ADCEnable(ADC_BASE);
}

//*****************************************************************************
//
//! Read ADC Channel 2
//!
//! \param  None
//!
//! \return ADC sample value (12-bit)
//
//*****************************************************************************
static unsigned long ReadADCChannel2(void) {
    unsigned long ulSample = 0;
    
    // Check if data is available in FIFO
    if(MAP_ADCFIFOLvlGet(ADC_BASE, ADC_CH_2)) {
        // Read the sample from FIFO
        ulSample = MAP_ADCFIFORead(ADC_BASE, ADC_CH_2);
        
        // Extract the 12-bit ADC sample from bits [13:2]
        ulSample = (ulSample & 0x3FFC) >> 2;
    }
    
    return ulSample;
}

//*****************************************************************************
//
//! Read ADC Channel 3
//!
//! \param  None
//!
//! \return ADC sample value (12-bit)
//
//*****************************************************************************
static unsigned long ReadADCChannel3(void) {
    unsigned long ulSample = 0;
    
    // Check if data is available in FIFO
    if(MAP_ADCFIFOLvlGet(ADC_BASE, ADC_CH_3)) {
        // Read the sample from FIFO
        ulSample = MAP_ADCFIFORead(ADC_BASE, ADC_CH_3);
        
        // Extract the 12-bit ADC sample from bits [13:2]
        ulSample = (ulSample & 0x3FFC) >> 2;
    }
    
    return ulSample;
}

//*****************************************************************************
//
//! Read ADC Channel 1
//!
//! \param  None
//!
//! \return ADC sample value (12-bit)
//
//*****************************************************************************
static unsigned long ReadADCChannel1(void) {
    unsigned long ulSample = 0;
    
    // Check if data is available in FIFO
    if(MAP_ADCFIFOLvlGet(ADC_BASE, ADC_CH_1)) {
        // Read the sample from FIFO
        ulSample = MAP_ADCFIFORead(ADC_BASE, ADC_CH_1);
        
        // Extract the 12-bit ADC sample from bits [13:2]
        ulSample = (ulSample & 0x3FFC) >> 2;
    }
    
    return ulSample;
}

//*****************************************************************************
//
//! Draw Dog ASCII Art using drawPixel
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void drawDogArt(void) {
    int y, x;
    
    // Clear the screen first
    
    // Go through each line of the dog art
    for(y = 0; y < 128; y++) {
        const char* line = dogArt[y];
        int lineLength = strlen(line);
        
        // Go through each character in the line (left to right)
        for(x = 0; x < lineLength && x < 128; x++) {
            char ch = line[x];
            
            // If character is not a space, draw a white pixel
            if(ch != ' ' && ch != '\0') {
                drawPixel(x, y, WHITE);
            }
        }
    }
}

void MasterMain()
{
    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
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

    // Add a small delay before initialization
    delay(100);
    Adafruit_Init();

    // Clear the display
    fillScreen(BLACK);
    delay(100);
}


//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! Main
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void main() {
    long lRetVal = -1;
    //
    // Initialize board configuration
    //
    BoardInit();

    PinMuxConfig();

    MasterMain();

    // Initialize ADC
    InitializeADC();
    
    // Initialize I2C for RTC
    InitializeI2C();

    // Initialize feeding log
    initializeFeedingLog();

//    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);


    InitTerm();
    ClearTerm();
    UART_PRINT("My terminal works!\n\r");

    // initialize global default app configuration
    g_app_config.host = SERVER_NAME;
    g_app_config.port = GOOGLE_DST_PORT;

    //Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();
    //Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }
    //Connect to the website with TLS encryption
    lRetVal = tls_connect();
    if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
    }

    // Variable to track previous GPIO value for edge detection
    int prev_value = 0;

    // Main loop - only execute when SW3 is pressed
    pulse_speed(2000);

    // Track if food is empty
    int isFoodEmpty = 0;
    
    while(1) {
        // Read all ADC Channel values
        unsigned long adcValue1 = ReadADCChannel1();
        unsigned long adcValue2 = ReadADCChannel2();
        unsigned long adcValue3 = ReadADCChannel3();

        // Only output ADC values if they are greater than 200
        UART_PRINT("ADC Values: CH1 (PIN_58): %lu, CH2 (PIN_59): %lu, CH3 (PIN_60): %lu\r\n", 
                   adcValue1, adcValue2, adcValue3);

        // Calculate water percentage (ADC value is 12-bit, so max is 4095)
        int waterPercentage = (adcValue2 * 100) / 4095;
        
        // Display water percentage in top left
        char waterStr[10];
        sprintf(waterStr, "Water: %d%", waterPercentage);
        unsigned int x;
        for(x = 0; x < strlen(waterStr); x++) {
            drawChar(2 + 6*x, 2, waterStr[x], BLUE, BLACK, 1);
        }


        // Read the bitmask (0x1) from GPIOA1_BASE
        int raw = GPIOPinRead(GPIOA1_BASE, 0x1);
        // Shift down so you get a 0 or 1
        int value = (raw & 0x1) ? 0 : 1;

        // Check if SW3 is pressed to enter menu mode
        if(SW3_PRESSED) {
            fillScreen(BLACK);

            
            // Enter menu mode
            int currentOption = 0;
            int menuActive = 1;
            int prevADCValue = adcValue1;
            int inSubScreen = 0; // Flag to track if we're in a sub-screen
            
            while(menuActive) {
                // Only display the menu if we're not in a sub-screen
                if(!inSubScreen) {
                    displayMenu(currentOption);
                }
                
                // Read ADC for cursor movement
                adcValue1 = ReadADCChannel1();
                
                // Only handle cursor movement if we're not in a sub-screen
                if(!inSubScreen) {
                    // Move cursor up if ADC value goes above 3000
                    if(adcValue1 > 3000 && prevADCValue <= 3000) {
                        currentOption = (currentOption > 0) ? currentOption - 1 : MENU_ITEMS - 1;
                    }
                    // Move cursor down if ADC value goes below 1000
                    else if(adcValue1 < 1000 && prevADCValue >= 1000) {
                        currentOption = (currentOption < MENU_ITEMS - 1) ? currentOption + 1 : 0;
                    }
                }
                
                prevADCValue = adcValue1;
                
                // Check for selection (SW3 press)
                if(SW3_PRESSED) {
                    fillScreen(BLACK);

                    
                    if(!inSubScreen) {
                        // We're in the main menu, handle menu selection
                        switch(currentOption) {
                            case MENU_FEEDING_HISTORY:
                                // Display feeding history
                                displayFeedingHistory();
                                inSubScreen = 1; // Enter sub-screen mode
                                break;
                                
                            case MENU_SETTINGS:
                                // Enter settings submenu
                                handleSettingsNavigation();
                                inSubScreen = 1; // Enter sub-screen mode
                                break;
                                
                            case MENU_BACK:
                                menuActive = 0; // Exit menu mode
                                fillScreen(BLACK); // Clear screen
                                break;
                        }
                    } else {
                        // We're in a sub-screen, go back to main menu
                        inSubScreen = 0;
                        // Menu will be redrawn on next iteration
                    }
                }
            }
            
            continue; // Skip the rest of the main loop iteration after exiting menu
        }

        // Handle food empty detection
        if (prev_value == 0 && value == 1) {
            Report("top photo = %d\r\n", value);
            isFoodEmpty = 1;
        } else if (prev_value == 1 && value == 0) {
            // Food has been refilled
            isFoodEmpty = 0;
            // Clear the "Food is Empty" message area
            fillRect(0, 95, 128, 20, BLACK);  // Clear the message area
        }

        // Update previous value for next iteration
        prev_value = value;

        // Execute servo action when ADC Channel 3 goes over 300 AND food is not empty AND within daily limit
        if(adcValue3 > 300 && !isFoodEmpty && canFeedToday()) {
            http_post(lRetVal);

            UART_PRINT("trying to draw ");

            const char *msg1 = "Dispensing Food";
            displayMessage(msg1);

            UART_PRINT("now servo");

            // Use adjusted pulse speed based on portion size setting
            adjustedPulseSpeed();

            UART_PRINT("done servo");
            
            // Record the feeding time and increment daily count
            recordFeedingTime();
            todayFeedingCount++;
            
            // Add a small delay to prevent rapid repeated execution
            delay(100);
        }
        // If daily limit reached, show message
        else if(adcValue3 > 300 && !isFoodEmpty && !canFeedToday()) {
            const char *limitMsg = "Daily Limit Reached";
            displayMessage(limitMsg);
            delay(200); // Show message longer
        }

        // Draw the dog art instead of filling screen with red
        drawDogArt();
        
        // Display "Food is Empty" in red if food is empty
        if(isFoodEmpty) {
            const char* emptyMsg = "Food is Empty";
            unsigned int msgLen = strlen(emptyMsg);
            unsigned int startX = (128 - 6*msgLen) / 2;
            
            // Display the message in red text
            for(x = 0; x < msgLen; x++) {
                drawChar(startX + 6*x, 100, emptyMsg[x], RED, BLACK, 1);
            }
        }
        
        // Update date and time display every second
        displayDateTimeOnOLED();
        
        // Delay for 1 second
        delay(3);  // delay() function uses a multiplier, so 3 gives roughly 1 second
    }

    sl_Stop(SL_STOP_TIMEOUT);
    LOOP_FOREVER();
}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(DATA1);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, DATA1);
    pcBufHeaders += strlen(DATA1);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

//*****************************************************************************
//
//! Display Message on OLED
//!
//! \param message - String message to display
//!
//! \return None
//
//*****************************************************************************
static void displayMessage(const char* message) {
    unsigned int msgLen = strlen(message);
    unsigned int startX = (128 - 6*msgLen) / 2;
    unsigned int x;
    
    // Display the message in white text
    for(x = 0; x < msgLen; x++) {
        drawChar(startX + 6*x, 100, message[x], GREEN, BLACK, 1);
    }
    
    delay(200);
    
    // Clear the message by drawing it in black
    for(x = 0; x < msgLen; x++) {
        drawChar(startX + 6*x, 100, message[x], BLACK, BLACK, 1);
    }
}

//*****************************************************************************
//
//! Initialize I2C
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void InitializeI2C(void) {
    MAP_UtilsDelay(10000);  // ~0.4 ms to let the clock "unstick"
    I2C_IF_Open(I2C_MASTER_MODE_STD);  // 100 kHz master
}

//*****************************************************************************
//
//! Convert a (0..99) decimal to BCD for storing in DS1307 registers
//!
//! \param val - decimal value to convert
//!
//! \return BCD encoded value
//
//*****************************************************************************
static unsigned char dec_to_bcd(unsigned char val) {
    return (unsigned char)(((val / 10) << 4) | (val % 10));
}

//*****************************************************************************
//
//! Convert a BCD encoded byte (00..99) into a decimal integer (0..99)
//!
//! \param bcd - BCD encoded value to convert
//!
//! \return decimal value
//
//*****************************************************************************
static unsigned char bcd_to_dec(unsigned char bcd) {
    return (unsigned char)(((bcd >> 4) * 10) + (bcd & 0x0F));
}

//*****************************************************************************
//
//! Read all seven DS1307 registers into out[0..6]
//!
//! \param out - array to store the 7 register values
//!
//! \return None
//
//*****************************************************************************
static void DS1307_ReadAllRegs(unsigned char out[7]) {
    unsigned char pointer = 0x00;

    // 1) Write "pointer = 0x00" + STOP
    I2C_IF_Write(DS1307_I2C_ADDR, &pointer, 1, 1);

    // 2) Read 7 bytes in one burst
    I2C_IF_Read(DS1307_I2C_ADDR, out, 7);
}

//*****************************************************************************
//
//! Display current date and time on top right of OLED
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
static void displayDateTimeOnOLED(void) {
    unsigned char allregs[7];
    char dateStr[12];  // "MM/DD/YY"
    char timeStr[12];  // "HH:MM:SS"
    
    // Read DS1307 registers
    DS1307_ReadAllRegs(allregs);
    
    // Convert BCD to decimal for all time/date fields
    unsigned char rsec  = bcd_to_dec(allregs[0] & 0x7F);
    unsigned char rmin  = bcd_to_dec(allregs[1] & 0x7F);
    unsigned char rhr   = bcd_to_dec(allregs[2] & 0x3F);
    unsigned char rdate = bcd_to_dec(allregs[4] & 0x3F);
    unsigned char rmon  = bcd_to_dec(allregs[5] & 0x1F);
    unsigned char ryr   = bcd_to_dec(allregs[6]);
    
    // Format date string: MM/DD/YY
    sprintf(dateStr, "%02u/%02u/%02u", rmon, rdate, ryr);
    
    // Format time string: HH:MM:SS
    sprintf(timeStr, "%02u:%02u:%02u", rhr, rmin, rsec);
    
    // Display date on top right (starting at x=72, y=2)
    int x;
    for(x = 0; x < 8; x++) {  // 8 characters in "MM/DD/YY"
        drawChar(72 + 6*x, 2, dateStr[x], WHITE, BLACK, 1);
    }
    
    // Display time right below date (starting at x=72, y=12)
    for(x = 0; x < 8; x++) {  // 8 characters in "HH:MM:SS"
        drawChar(72 + 6*x, 12, timeStr[x], WHITE, BLACK, 1);
    }
}

//*****************************************************************************
//
//! Display Menu on OLED
//!
//! \param selectedOption - Index of the selected menu option
//!
//! \return None
//
//*****************************************************************************
static void displayMenu(int selectedOption) {
    // Clear the screen
    
    // Menu title
    const char* title = "Menu";
    unsigned int titleLen = strlen(title);
    unsigned int titleX = (128 - 6*titleLen) / 2;
    unsigned int x;
    for(x = 0; x < titleLen; x++) {
        drawChar(titleX + 6*x, 10, title[x], WHITE, BLACK, 1);
    }
    
    // Menu options
    const char* options[] = {
        "Feeding History",
        "Settings",
        "Back"
    };
    
    int i;
    for(i = 0; i < MENU_ITEMS; i++) {
        unsigned int len = strlen(options[i]);
        unsigned int startX = (128 - 6*len) / 2;
        
        // Draw cursor for selected option
        if(i == selectedOption) {
            drawChar(startX - 12, 35 + i*20, '>', WHITE, BLACK, 1);
        }
        
        // Draw option text
        for(x = 0; x < len; x++) {
            drawChar(startX + 6*x, 35 + i*20, options[i][x], 
                    (i == selectedOption) ? GREEN : WHITE, BLACK, 1);
        }
    }
}

//*****************************************************************************
//
//! Initialize Feeding Log
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
static void initializeFeedingLog(void) {
    int i;
    for(i = 0; i < MAX_FEEDING_RECORDS; i++) {
        feedingLog[i].valid = 0;  // Mark all records as invalid
    }
    feedingLogIndex = 0;
}

//*****************************************************************************
//
//! Record Feeding Time
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
static void recordFeedingTime(void) {
    unsigned char allregs[7];
    
    // Read current time from DS1307
    DS1307_ReadAllRegs(allregs);
    
    // Convert BCD to decimal and store in feeding log
    feedingLog[feedingLogIndex].hour = bcd_to_dec(allregs[2] & 0x3F);
    feedingLog[feedingLogIndex].minute = bcd_to_dec(allregs[1] & 0x7F);
    feedingLog[feedingLogIndex].second = bcd_to_dec(allregs[0] & 0x7F);
    feedingLog[feedingLogIndex].date = bcd_to_dec(allregs[4] & 0x3F);
    feedingLog[feedingLogIndex].month = bcd_to_dec(allregs[5] & 0x1F);
    feedingLog[feedingLogIndex].year = bcd_to_dec(allregs[6]);
    feedingLog[feedingLogIndex].valid = 1;  // Mark as valid
    
    // Move to next index (circular buffer)
    feedingLogIndex = (feedingLogIndex + 1) % MAX_FEEDING_RECORDS;
}

//*****************************************************************************
//
//! Display Feeding History
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
static void displayFeedingHistory(void) {
    int i, displayCount = 0;
    char timeStr[20];
    unsigned int x;
    
    // Clear screen
    fillScreen(BLACK);
    
    // Title
    const char* title = "Feeding History";
    unsigned int titleLen = strlen(title);
    unsigned int titleX = (128 - 6*titleLen) / 2;
    for(x = 0; x < titleLen; x++) {
        drawChar(titleX + 6*x, 5, title[x], WHITE, BLACK, 1);
    }
    
    // Find and display valid feeding records (show most recent first)
    int currentIndex = (feedingLogIndex - 1 + MAX_FEEDING_RECORDS) % MAX_FEEDING_RECORDS;
    
    for(i = 0; i < MAX_FEEDING_RECORDS && displayCount < 8; i++) {
        if(feedingLog[currentIndex].valid) {
            // Format: MM/DD HH:MM
            sprintf(timeStr, "%02u/%02u %02u:%02u", 
                   feedingLog[currentIndex].month,
                   feedingLog[currentIndex].date,
                   feedingLog[currentIndex].hour,
                   feedingLog[currentIndex].minute);
            
            // Display the feeding time
            unsigned int len = strlen(timeStr);
            unsigned int startX = (128 - 6*len) / 2;
            for(x = 0; x < len; x++) {
                drawChar(startX + 6*x, 25 + displayCount*12, timeStr[x], GREEN, BLACK, 1);
            }
            displayCount++;
        }
        
        // Move to previous record
        currentIndex = (currentIndex - 1 + MAX_FEEDING_RECORDS) % MAX_FEEDING_RECORDS;
    }
    
    // If no feeding records, show message
    if(displayCount == 0) {
        const char* noRecordsMsg = "No feeding records";
        unsigned int msgLen = strlen(noRecordsMsg);
        unsigned int startX = (128 - 6*msgLen) / 2;
        for(x = 0; x < msgLen; x++) {
            drawChar(startX + 6*x, 50, noRecordsMsg[x], YELLOW, BLACK, 1);
        }
    }
    
    // Show instruction to go back
    const char* backMsg = "Press SW3 to go back";
    unsigned int msgLen = strlen(backMsg);
    unsigned int startX = (128 - 6*msgLen) / 2;
    for(x = 0; x < msgLen; x++) {
        drawChar(startX + 6*x, 110, backMsg[x], WHITE, BLACK, 1);
    }
}

static void displaySettingsMenu(int selectedOption) {
    // Clear screen
    fillScreen(BLACK);
    
    // Title
    const char* title = "Settings";
    unsigned int titleLen = strlen(title);
    unsigned int titleX = (128 - 6*titleLen) / 2;
    unsigned int x;
    for(x = 0; x < titleLen; x++) {
        drawChar(titleX + 6*x, 5, title[x], WHITE, BLACK, 1);
    }
    
    // Current portion size setting
    char portionStr[30];
    const char* portionNames[] = {"Small", "Medium", "Large"};
    sprintf(portionStr, "Portion: %s", portionNames[currentPortionSize]);
    unsigned int len = strlen(portionStr);
    unsigned int startX = (128 - 6*len) / 2;
    
    if(selectedOption == SETTINGS_PORTION_SIZE) {
        drawChar(startX - 12, 30, '>', WHITE, BLACK, 1);
    }
    for(x = 0; x < len; x++) {
        drawChar(startX + 6*x, 30, portionStr[x], 
                (selectedOption == SETTINGS_PORTION_SIZE) ? GREEN : WHITE, BLACK, 1);
    }
    
    // Daily feeding limit setting
    char limitStr[30];
    sprintf(limitStr, "Daily Limit: %d", dailyFeedingLimit);
    len = strlen(limitStr);
    startX = (128 - 6*len) / 2;
    
    if(selectedOption == SETTINGS_DAILY_LIMIT) {
        drawChar(startX - 12, 50, '>', WHITE, BLACK, 1);
    }
    for(x = 0; x < len; x++) {
        drawChar(startX + 6*x, 50, limitStr[x], 
                (selectedOption == SETTINGS_DAILY_LIMIT) ? GREEN : WHITE, BLACK, 1);
    }
    
    // Back option
    const char* backStr = "Back";
    len = strlen(backStr);
    startX = (128 - 6*len) / 2;
    
    if(selectedOption == SETTINGS_BACK) {
        drawChar(startX - 12, 70, '>', WHITE, BLACK, 1);
    }
    for(x = 0; x < len; x++) {
        drawChar(startX + 6*x, 70, backStr[x], 
                (selectedOption == SETTINGS_BACK) ? GREEN : WHITE, BLACK, 1);
    }
    
    // Show current daily feeding status
    char statusStr[30];
    sprintf(statusStr, "Today: %d/%d fed", todayFeedingCount, dailyFeedingLimit);
    len = strlen(statusStr);
    startX = (128 - 6*len) / 2;
    for(x = 0; x < len; x++) {
        drawChar(startX + 6*x, 90, statusStr[x], YELLOW, BLACK, 1);
    }
    
    // Instructions
    const char* instrStr = "SW3=Select ADC=Change";
    len = strlen(instrStr);
    startX = (128 - 6*len) / 2;
    for(x = 0; x < len; x++) {
        drawChar(startX + 6*x, 110, instrStr[x], WHITE, BLACK, 1);
    }
}

static void handleSettingsNavigation(void) {
    int settingsOption = 0;
    int settingsActive = 1;
    int prevADCValue = ReadADCChannel1();
    
    while(settingsActive) {
        displaySettingsMenu(settingsOption);
        
        // Read ADC for navigation
        unsigned long adcValue1 = ReadADCChannel1();
        
        // Navigate up/down through settings
        if(adcValue1 > 3000 && prevADCValue <= 3000) {
            settingsOption = (settingsOption > 0) ? settingsOption - 1 : SETTINGS_MENU_ITEMS - 1;
        }
        else if(adcValue1 < 1000 && prevADCValue >= 1000) {
            settingsOption = (settingsOption < SETTINGS_MENU_ITEMS - 1) ? settingsOption + 1 : 0;
        }
        
        prevADCValue = adcValue1;
        
        // Handle selection with SW3
        if(SW3_PRESSED) {
            switch(settingsOption) {
                case SETTINGS_PORTION_SIZE:
                    // Cycle through portion sizes
                    currentPortionSize = (currentPortionSize + 1) % 3;
                    delay(50); // Debounce
                    break;
                    
                case SETTINGS_DAILY_LIMIT:
                    // Cycle through daily limits (1, 2, 3)
                    dailyFeedingLimit++;
                    if(dailyFeedingLimit > 3) {
                        dailyFeedingLimit = 1;
                    }
                    delay(50); // Debounce
                    break;
                    
                case SETTINGS_BACK:
                    settingsActive = 0; // Exit settings menu
                    break;
            }
        }
        
        delay(10); // Small delay to prevent too rapid updates
    }
}

static void checkDailyFeedingReset(void) {
    unsigned char allregs[7];
    
    // Read current date from DS1307
    DS1307_ReadAllRegs(allregs);
    unsigned char currentDay = bcd_to_dec(allregs[4] & 0x3F);
    
    // If the day has changed, reset the feeding count
    if(currentDay != lastFeedingDay) {
        todayFeedingCount = 0;
        lastFeedingDay = currentDay;
    }
}

static int canFeedToday(void) {
    checkDailyFeedingReset();
    return (todayFeedingCount < dailyFeedingLimit);
}

static void adjustedPulseSpeed(void) {
    int pulseDelay;
    int numPulses;
    
    // Set pulse parameters based on portion size
    switch(currentPortionSize) {
        case PORTION_SMALL:
            pulseDelay = 1300;  // Small portion - shorter pulse
            numPulses = 1;      // Single pulse
            break;
        case PORTION_MEDIUM:
            pulseDelay = 1500;  // Medium portion - medium pulse
            numPulses = 1;      // Single pulse
            break;
        case PORTION_LARGE:
            pulseDelay = 1700;  // Large portion - longer pulse
            numPulses = 2;      // Double pulse for more food
            break;
        default:
            pulseDelay = 1500;  // Default to medium
            numPulses = 1;
            break;
    }
    
    // Execute the feeding pulses
    int i;
    for(i = 0; i < numPulses; i++) {
        pulse_speed(pulseDelay);
        if(numPulses > 1) {
            delay(50);  // Brief pause between pulses for large portions
        }
    }
    
    // Return to neutral position
    delay(80);
    pulse_speed(2000);
}
