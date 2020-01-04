// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSC oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = ON        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is MCLR)
#pragma config BOREN = OFF      // *** 3.3V! Brown-out Detect Enable bit (BOD enabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <stdint.h>
#include <xc.h>
#include "hw.h"
#include "si_fm_radio.h"

// When triggered to 1, data received from LM7001
static bit s_lmCeDeasserted;
static uint8_t s_lmDataCount;

typedef union {
    uint24_t data;
    struct {
        unsigned FREQ_LO: 8;
        unsigned FREQ_HI: 6; // 14 bit: steps of ref freqs, base 0
        unsigned TEST: 2; // always 0
        unsigned BAND_SELECTORS: 4; // regulates GPIOs
        unsigned REF_FREQ: 3;  // the base step frequency: 0: 100kHz, 4: 50kHz
        unsigned DIVIDER: 1; // 1: FM, 0: AM
    };
} LM_DATA;
static LM_DATA s_lmData;  // LSB are received first

void debug(DEBUG_CODES debugCode) {
    for (uint8_t i = 0; i < debugCode; i++) {
        DEBUG_TX_PORT = 1;
        DEBUG_TX_PORT = 0;
    }
}

/*
static void debug8(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        DEBUG_TX_PORT = 1;
        if (data & 0x80) {
            NOP();
            NOP();
            NOP();
            NOP();
            NOP();
        }
        DEBUG_TX_PORT = 0;
        data <<= 1;
    }
}
*/

// Decode data received from Pioneer MCU. Not time critical, Pioneer
// waits > 100ms between an update to the next
static void decodeLmData() {    
    if (s_lmData.DIVIDER != 1 || s_lmData.TEST != 0 || s_lmData.REF_FREQ != 4) {
        // Invalid (AM or invalid data)
        si_fm_mute(1);
        debug(DEBUG_INVALID_CODE);
    } else {
        si_fm_mute(0);
        
        //debug8(s_lmData.FREQ_LO);
        //debug8(s_lmData.FREQ_HI);
        
        uint16_t freq = (s_lmData.FREQ_LO | (s_lmData.FREQ_HI << 8)) - (875 * 2) - (107 * 2); // Base is 87.5 , subtract IF too
        if (freq > 410) { // (108 - 87.5 Mhz) / 50 kHz
            debug(DEBUG_INVALID_CODE);
        } else {
            si_fm_tune(freq);
            debug(DEBUG_VALID_CODE);
        }
    }
}

static bit s_dat;
static void __interrupt interruptVector() {    
    s_dat = MCU_DAT_PORT;
    // If CE is low, deassert everything and process data, if count == 24 is valid
    if (!MCU_CE_PORT) {
        s_lmCeDeasserted = 1;
    } else if (INTCONbits.INTF) {
        // else INTF. Sample data
        s_lmData.data >>= 1;
        if (s_dat) {
            s_lmData.data |= 0x800000;
        }
        s_lmDataCount++;
    }

    // Reset interrupt flags
    INTCONbits.RBIF = 0;
    INTCONbits.INTF = 0;
}

void main(void) {
    CLRWDT();
    // Assign prescaler to WDT. 1:256
    OPTION_REGbits.PSA = 1;
    OPTION_REGbits.PS = 7;
    OPTION_REGbits.T0CS = 0;
    CLRWDT();
    // Wdt now up to 2.3s
    
    // Disable comparators to use full porta
    CMCON = 7;
    
    DEBUG_TX_TRIS = 0;
    NOP();
    DEBUG_TX_PORT = 0;
        
    // Init LM7001 interface 
    MCU_CE_TRIS = MCU_CLK_TRIS = MCU_DAT_TRIS = 1;
    MCU_MONO_TRIS = 1;
    
    MCU_TUNE_TRIS = MCU_STEREO_TRIS = 0;
    MCU_TUNE_PORT = MCU_STEREO_PORT = 0;
            
    // Init interrupts. Only RB change and INT enabled
    INTCON = 0;
    INTCONbits.RBIE = 1;
    INTCONbits.INTE = 1;
    // Data sampled on rising edge
    OPTION_REGbits.INTEDG = 1;
    // Enable pull-up to avoid spurious RB change interrupts
    OPTION_REGbits.nRBPU = 1;
            
    // Init SI module
    si_fm_init();
    
    // Now setup LM port
    // Wait until CE is deasserted
    s_lmCeDeasserted = 0;
    s_lmDataCount = 0;
    while (MCU_CE_PORT);

    debug(DEBUG_RESET);
    
    // Start receiving data
    INTCONbits.GIE = 1;

    // Loop
    while (1) {
        CLRWDT();
        uint8_t dataValid = 0;
        
        // Data packet received?
        if (s_lmCeDeasserted) {
            s_lmCeDeasserted = 0;
            dataValid = s_lmDataCount == 24;
            s_lmDataCount = 0;
        }

        if (dataValid) {
            //DEBUG_TX_PORT = 1;
            // Data valid. Decode it and send it to SI, decouple it from the buffer
            // so next packets can be received
            decodeLmData();
        }
        
        // Propagate MONO command
        si_fm_forceMono(MCU_MONO_PORT);
        
        // Poll status and update output STEREO and SIGNAL lines
        si_fm_pollStatus();
        // Update the output line as soon as possible for the Pioneer MCU seek algo
        MCU_TUNE_PORT = si_fm_isTuned() ? 0 : 1;
        MCU_STEREO_PORT = si_fm_isStereo() ? 1 : 0;
        si_fm_postStatus();
    }
}
