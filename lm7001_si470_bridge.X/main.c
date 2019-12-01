#include <stdint.h>
#include <xc.h>

// PIC 16F628A
// FOSC 4MHz (internal)

// RB1/RB2 (RX/TX) for future expansions and for debug
// RB4/RB6/RB7 for programming interface

#define DEBUG_TX_TRIS TRISBbits.TRISB2
#define DEBUG_TX_PORT PORTBbits.RB2

/**
 * Si4703 interface
 */
// Reset pin
#define SI_RES_TRIS TRISAbits.TRISA0
#define SI_RES_PORT PORTAbits.RA0
// SCK
#define SI_SCK_TRIS TRISAbits.TRISA1
#define SI_SCK_PORT PORTAbits.RA1
// SDA
#define SI_SDA_TRIS TRISAbits.TRISA2
#define SI_SDA_PORT PORTAbits.RA2

/**
 * LM7001 driver interface from Pioneer MCU
 */
// CE on RB5 for interrupt on change
#define MCU_CE_TRIS TRISBbits.TRISB5
#define MCU_CE_PORT PORTBbits.RB5
// Clock on RB0 for interrupt on edge rising
#define MCU_CLK_TRIS TRISBbits.TRISB0
#define MCU_CLK_PORT PORTBbits.RB0

#define MCU_DAT_TRIS TRISAbits.TRISA3
#define MCU_DAT_PORT PORTAbits.RA3

// MONO is active high
#define MCU_MONO_TRIS TRISAbits.TRISA6
#define MCU_MONO_PORT PORTAbits.RA6

// TUNE is active low
#define MCU_TUNE_TRIS TRISBbits.TRISB1
#define MCU_TUNE_PORT PORTBbits.RB1

// STEREO is unknown
#define MCU_STEREO_TRIS TRISBbits.TRISB3
#define MCU_STEREO_PORT PORTBbits.RB3

// When triggered to 1, data received from LM7001
static bit _lmCeDeasserted;
static uint8_t _lmDataCount;
static uint24_t _lmData;  // LSB are received first

static void init_hw() {
    // Disable comparators to use full porta
    CMCON = 7;
    
    // Reset si4702
    SI_RES_TRIS = 0;
    DEBUG_TX_TRIS = 0;
    SI_RES_PORT = 0;
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
    OPTION_REGbits.nRBPU = 0;
            
    // Reset with GPIO1 high = I2C
    SI_RES_PORT = 1;    
    
    // Init SI module
    // TODO
    
    // Now setup LM port
    // Wait until CE is deasserted
    _lmCeDeasserted = 0;
    _lmDataCount = 0;
    while (MCU_CE_PORT);
    
    // Start receiving data
    INTCONbits.GIE = 1;
}

static void __interrupt interruptVector() {    
    // If CE is low, deassert everything and process data, if count == 24 is valid
    if (!MCU_CE_PORT) {
        _lmCeDeasserted = 1;
    } else if (INTCONbits.INTF) {
        // else INTF. Sample data
        _lmData >>= 1;
        if (MCU_DAT_PORT) {
            _lmData |= 0x800000;
        }
        _lmDataCount++;
    }

    // Reset interrupt flags
    INTCONbits.RBIF = 0;
    INTCONbits.INTF = 0;
}

static bit _forceMono;

static void updateSiMono() {
    // Update mono status
    // TODO
}

static void updateSiFreq() {
    // Send frequency and band to Si module
    // TODO
}

// Decode data received from Pioneer MCU. Not time critical, Pioneer
// waits > 100ms between an update to the next
static void decodeLmData(uint24_t data) {    
    // TODO
}

// Read SI status from i2c and update PIC ports
static void pollSiStatus() {    
    // TODO
}

void main(void) {
    init_hw();

    // Load defaults (muted)
    updateSiFreq();
    _forceMono = !!MCU_MONO_PORT;
    updateSiMono();

    // Loop
    while (1) {
        CLRWDT();

        // Data packet received?
        if (_lmCeDeasserted) {
            _lmCeDeasserted = 0;
            int ok = _lmDataCount == 24;
            _lmDataCount = 0;
            if (ok) {
                // 1 pulse for OK received
                DEBUG_TX_PORT = 1;
                DEBUG_TX_PORT = 0;
                
                // Data valid. Decode it and send it to SI, decouple it from the buffer
                // so next packets can be received
                decodeLmData(_lmData);
                updateSiFreq();
            } else {
                // 2 pulses for size not valid
                DEBUG_TX_PORT = 1;
                DEBUG_TX_PORT = 0;
                DEBUG_TX_PORT = 1;
                DEBUG_TX_PORT = 0;
            }
        }

        // Propagate MONO command
        if (MCU_MONO_PORT != _forceMono) {
            _forceMono = MCU_MONO_PORT;
            updateSiMono();
        }
        
        // Poll status and update output STEREO and SIGNAL lines
        pollSiStatus();
    }
}
