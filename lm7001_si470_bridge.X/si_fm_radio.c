#include <stdint.h>
#include <xc.h>
#include "hw.h"
#include "i2c.h"
#include "si_fm_radio.h"

#define SI_WRITE_ADDRESS 0x20
#define SI_READ_ADDRESS 0x21

typedef union {
    struct {
        unsigned ENABLE: 1;
        unsigned _RES: 5;
        unsigned DISABLE: 1;
        unsigned _RES2: 1;
        unsigned SEEK: 1;
        unsigned SEEKUP: 1;
        unsigned SKMODE: 1;
        unsigned RDSM: 1;
        unsigned _RES3: 1;
        unsigned MONO: 1;
        unsigned DMUTE: 1;
        unsigned DSMUTE: 1;           
    };
    uint16_t data;
} SI_POWER_T;
static SI_POWER_T si_powerCfg;

typedef union {
    struct {
        unsigned CHANNEL_LO: 8;
        unsigned CHANNEL_HI: 2;
        unsigned _RES: 5;
        unsigned TUNE: 1;
    };
    uint16_t data;
} SI_CHANNEL_T;
static SI_CHANNEL_T si_channel;

typedef union {
    struct {
        unsigned RSSI: 8;
        unsigned ST: 1;
        unsigned BLEAR: 2;
        unsigned RDSS: 1;
        unsigned AFCRL: 1;
        unsigned SF_BL: 1;
        unsigned STC: 1;
        unsigned RDSR: 1;
    };
    uint16_t data;
} SI_STATUS_T;
static SI_STATUS_T si_status;

void si_fm_init() {
    i2c_init();
    
    // Reset si4702
    SI_RES_TRIS = 0;
    NOP();
    SI_RES_PORT = 0;

    // Reset with GPIO1 high = I2C
    __delay_ms(1);
    SI_RES_PORT = 1;    

    __delay_ms(100);

    si_powerCfg.data = 0; // muted and not ENABLED
    si_channel.TUNE = 0;
    si_channel.CHANNEL_LO = 0; // 87.5MHz
    si_channel.CHANNEL_HI = 0; // 87.5MHz
    
    i2c_start(SI_WRITE_ADDRESS);
    i2c_write_16(si_powerCfg.data);
    i2c_write_16(si_channel.data); // channel, tune disabled
    i2c_write_16(0x0080); // System configuration 1: AGC ON, deemphasis Europe, interrupt disabled, RDS disabled
    i2c_write_16(0x002F); // System configuration 2: 0db volume, fine band (50 kHz), min RSSI
    i2c_write_16(0x0000); // System configuration 3: most stops in seeks, standard volume
    //i2c_write_16(0x3c04 | 0x8000, I2C_ACK_STOP); // see datahseet
    i2c_write_16(0x8100); // the above doesn't work!
    i2c_stop();
    
    // Wait before initializing other registries
    CLRWDT();
    __delay_ms(150);
    CLRWDT();

    // Start muted and ENABLED now
    si_powerCfg.ENABLE = 1;
    i2c_start(SI_WRITE_ADDRESS);
    i2c_write_16(si_powerCfg.data);
    i2c_stop();
}

void si_fm_forceMono(uint8_t forceMono) {
    uint8_t isMono = si_powerCfg.MONO;
    if (forceMono != isMono) {
        // Update mono status
        si_powerCfg.MONO = forceMono;
        // Only write register 2 (the first)
        i2c_start(SI_WRITE_ADDRESS);
        i2c_write_16(si_powerCfg.data);
        i2c_stop();
    }
}

void si_fm_mute(uint8_t mute) {
    uint8_t isMute = si_powerCfg.DMUTE;
    if (isMute != mute) {
        // Update mute status
        si_powerCfg.DMUTE = mute;
        si_powerCfg.DSMUTE = mute;
        
        // Only write register 2 (the first)
        i2c_start(SI_WRITE_ADDRESS);
        i2c_write_16(si_powerCfg.data);
        i2c_stop();
    }
}

// Freq is 50kHz step count over 87.5MHz. Max 410
void si_fm_tune(uint16_t freq) {
    si_channel.CHANNEL_HI = freq >> 8;
    si_channel.CHANNEL_LO = freq & 0xff;
    si_powerCfg.DMUTE = 1;
    si_powerCfg.DSMUTE = 1;

    if (si_channel.TUNE) {
        // Reset the STC first bit
        si_channel.TUNE = 0;
        i2c_start(SI_WRITE_ADDRESS);
        i2c_write_16(si_powerCfg.data);
        i2c_write_16(si_channel.data); 
        i2c_stop();
    }

    // Now tune
    si_channel.TUNE = 1;
    i2c_start(SI_WRITE_ADDRESS);
    i2c_write_16(si_powerCfg.data);
    i2c_write_16(si_channel.data); 
    i2c_stop();
}

// Read SI status from i2c and update PIC ports
void si_fm_pollStatus() {
    i2c_start(SI_READ_ADDRESS);
    si_status.data = i2c_read_16(I2C_NACK);
    i2c_stop();
}

uint8_t si_fm_isTuned() {
    return si_status.AFCRL;
}

uint8_t si_fm_isStereo() {
    return si_status.ST;
}

void si_fm_postStatus() {
    // Still tuning?
    if (si_status.STC && si_channel.TUNE) {
        // Seek complete, reset the TUNE flag
        si_channel.TUNE = 0;
        DEBUG_TX_PORT = 0;

        i2c_start(SI_WRITE_ADDRESS);
        i2c_write_16(si_powerCfg.data);
        i2c_write_16(si_channel.data); 
        i2c_stop();
        // Wait for the STC flag 
        do {
            si_fm_pollStatus();
        } while (si_status.STC);
    }
}
