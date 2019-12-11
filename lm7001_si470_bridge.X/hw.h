// PIC 16F628A
// FOSC 4MHz (internal)
#define _XTAL_FREQ 4000000

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

typedef enum {
    DEBUG_INVALID_CODE = 3,
    DEBUG_RESET = 4
} DEBUG_CODES;
void debug(DEBUG_CODES debugCode);
