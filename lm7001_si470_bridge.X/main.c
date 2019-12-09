// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSC oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF        // Watchdog Timer Enable bit (WDT enabled)
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

// Decode data received from Pioneer MCU. Not time critical, Pioneer
// waits > 100ms between an update to the next
static void decodeLmData() {    
    if (s_lmData.DIVIDER != 1 || s_lmData.TEST != 0 || s_lmData.REF_FREQ != 4) {
        // Invalid (AM or invalid data)
        si_fm_mute(1);
    } else {
        si_fm_mute(0);
        uint16_t freq = (s_lmData.FREQ_LO | (s_lmData.FREQ_HI << 8)) - 1750; // Base is 87.5 
        si_fm_tune(freq);
    }
}

static void __interrupt interruptVector() {    
    // If CE is low, deassert everything and process data, if count == 24 is valid
    if (!MCU_CE_PORT) {
        s_lmCeDeasserted = 1;
    } else if (INTCONbits.INTF) {
        // else INTF. Sample data
        s_lmData.data >>= 1;
        if (MCU_DAT_PORT) {
            s_lmData.data |= 0x800000;
        }
        s_lmDataCount++;
    }

    // Reset interrupt flags
    INTCONbits.RBIF = 0;
    INTCONbits.INTF = 0;
}

// For debug only
static uint8_t s_intCounter = 0;
static uint8_t s_tune = 0;

void main(void) {
    // Assign prescaler to TMR0. 1:256
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS = 7;
    // TMR0 internal
    OPTION_REGbits.T0CS = 0;
    
    // Disable comparators to use full porta
    CMCON = 7;
    
    // Reset si4702
    SI_SCK_TRIS = SI_SDA_TRIS = SI_RES_TRIS = 0;
    DEBUG_TX_TRIS = 0;

    SI_RES_PORT = 0;
    DEBUG_TX_PORT = 0;
    SI_SCK_PORT = SI_SDA_PORT = 1; 
        
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
    OPTION_REGbits.nRBPU = 0;
            
    // Reset with GPIO1 high = I2C
    SI_RES_PORT = 1;    
    
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

    // == End of Init
    
    // Load defaults (muted)
    si_fm_tune(s_tune = 0);

    // Loop
    while (1) {
        CLRWDT();
        
        if (INTCONbits.T0IF) {
            INTCONbits.T0IF = 0;
            s_intCounter++;
            if (s_intCounter > 16) {
                s_intCounter = 0;
                // Set freq.
                s_tune++;
                if (s_tune > 208) s_tune = 0;
                si_fm_tune(s_tune);
            }
        }

        // Data packet received?
        if (s_lmCeDeasserted) {
            s_lmCeDeasserted = 0;
            int ok = s_lmDataCount == 24;
            s_lmDataCount = 0;
            if (ok) {
                // 1 pulse for OK received
                debug(DEBUG_VALID_CODE);
                
                // Data valid. Decode it and send it to SI, decouple it from the buffer
                // so next packets can be received
                decodeLmData();
            } else {
                // 2 pulses for size not valid
                debug(DEBUG_INVALID_CODE);
            }
        }

        // Propagate MONO command
        si_fm_forceMono(MCU_MONO_PORT);
        
        // Poll status and update output STEREO and SIGNAL lines
        SI_FM_STATUS status = si_fm_status();
        MCU_TUNE_PORT = status.TUNED;
        MCU_STEREO_PORT = status.STEREO;
    }
}
