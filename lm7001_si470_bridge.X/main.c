// PIC 16F628A
// FOSC 4MHz (internal)

// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSC oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = ON        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is MCLR)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <stdint.h>
#include <xc.h>

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
static bit _forceMono;
static bit _seeking; // seek still in progress

typedef union {
    struct {
        bit ENABLE: 1;
        bit _RES: 5;
        bit DISABLE: 1;
        bit _RES2: 1;
        bit SEEK: 1;
        bit SEEKUP: 1;
        bit SKMODE: 1;
        bit RDSM: 1;
        bit _RES3: 1;
        bit MONO: 1;
        bit DMUTE: 1;
        bit DSMUTE: 1;           
    };
    uint16_t data;
} SI_POWER_T;
static SI_POWER_T si_powerCfg;

typedef enum {
    DEBUG_VALID_CODE = 1,
    SEEK_COMPLETE = 2,
    DEBUG_INVALID_CODE = 3,
    DEBUG_RESET = 4
} DEBUG_CODES;

typedef union {
    struct {
        uint16_t CHANNEL: 10;
        uint8_t RES: 5;
        bit TUNE: 1;
    };
    uint16_t data;
} SI_CHANNEL_T;
static SI_CHANNEL_T si_channel;

typedef union {
    struct {
        uint8_t RSSI: 8;
        bit ST: 1;
        uint8_t BLEAR: 2;
        bit RDSS: 1;
        bit AFCRL: 1;
        bit SF_BL: 1;
        bit STC: 1;
        bit RDSR: 1;
    };
    uint16_t data;
} SI_STATUS_T;

static void debug(DEBUG_CODES debugCode) {
    for (uint8_t i = 0; i < debugCode; i++) {
        DEBUG_TX_PORT = 1;
        DEBUG_TX_PORT = 0;
    }
}

static void i2c_begin() {
}

static void i2c_write_16(uint16_t data) {
}

static uint16_t i2c_read_16() {
}

static void i2c_write_end() {
}

static void i2c_read_end() {
}

static void initSi() {
    wait(200ms);

    si_powerCfg.data = 0;
    si_channel.TUNE = 0;
    si_channel.CHANNEL = 0; // 87.5MHz
    
    i2c_begin();
    i2c_write_16(si_powerCfg);
    i2c_write_16(si_channel); // channel, tune disabled
    i2c_write_16(0x0080); // System configuration 1: AGC ON, deemphasis Europe, interrupt disabled, RDS disabled
    i2c_write_16(0x001F); // System configuration 2: 0db volume, Europe band (100 kHz), min RSSI
    i2c_write_16(0x0000); // System configuration 3: most stops in seeks, standard volume
    i2c_write_16(0x3c04 | 0x8000); // see datahseet
    i2c_write_end();
    
    // Wait before initializing other registries
    wait(150ms);

    // Start muted and DISABLED
    si_powerCfg.ENABLE = 1;
    i2c_begin();
    i2c_write_16(si_powerCfg);
    i2c_write_end();
}

static void updateSiMono() {
    // Update mono status
    si_powerCfg.MONO = _forceMono;
    // Only write register 2 (the first)
    i2c_begin();
    i2c_write_16(si_powerCfg);
    i2c_write_end();
}

static void updateSiFreq() {
    // Reset the STC bit
    si_channel.TUNE = 0;
    i2c_begin();
    i2c_write_16(si_powerCfg);
    i2c_write_16(si_channel); 
    i2c_write_end();

    // Now tune
    si_channel.TUNE = 1;
    i2c_begin();
    i2c_write_16(si_powerCfg);
    i2c_write_16(si_channel); 
    i2c_write_end();
    
    _seeking = 1;
}

// Decode data received from Pioneer MCU. Not time critical, Pioneer
// waits > 100ms between an update to the next
static void decodeLmData(uint24_t data) {    
    // TODO
}

// Read SI status from i2c and update PIC ports
static void pollSiStatus() {
    i2c_begin();
    SI_STATUS_T status = i2c_read_16();
    i2c_read_end();
    
    if (status.STC && _seeking) {
        // Seek complete
        _seeking = 0;
        debug(SEEK_COMPLETE);
    }
    
    MCU_TUNE_PORT = status.STC && status.AFCRL;
    MCU_STEREO_PORT = status.ST;
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

static void getNewFreq() {
    si_channel.CHANNEL++;
    // 108.0 - 87.5 = 205 steps of 100kHz
    if (si_channel.CHANNEL > 205) {
        si_channel.CHANNEL = 0;
    }
}

static uint8_t s_intCounter = 0;

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
    initSi();
    
    // Now setup LM port
    // Wait until CE is deasserted
    _lmCeDeasserted = 0;
    _lmDataCount = 0;
    while (MCU_CE_PORT);

    debug(DEBUG_RESET);
    
    // Start receiving data
    INTCONbits.GIE = 1;

    // == End of Init
    
    // Load defaults (muted)
    updateSiFreq();
    _forceMono = !!MCU_MONO_PORT;
    updateSiMono();

    // Loop
    while (1) {
        CLRWDT();
        
        if (INTCONbits.T0IF) {
            INTCONbits.T0IF = 0;
            s_intCounter++;
            if (s_intCounter > 16) {
                s_intCounter = 0;
                // Set freq.
                getNewFreq();
                // 1 pulse for OK received
                debug(DEBUG_VALID_CODE);
                updateSiFreq();
            }
        }

        // Data packet received?
        if (_lmCeDeasserted) {
            _lmCeDeasserted = 0;
            int ok = _lmDataCount == 24;
            _lmDataCount = 0;
            if (ok) {
                // 1 pulse for OK received
                debug(DEBUG_VALID_CODE);
                
                // Data valid. Decode it and send it to SI, decouple it from the buffer
                // so next packets can be received
                decodeLmData(_lmData);
                updateSiFreq();
            } else {
                // 2 pulses for size not valid
                debug(DEBUG_INVALID_CODE);
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
