#include <stdint.h>
#include <xc.h>
#include "hw.h"
#include "i2c.h"
#include "si_fm_radio.h"

static bit _seeking; // seek still in progress

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

void si_fm_init() {
    CLRWDT();
    __delay_ms(200);
    CLRWDT();

    si_powerCfg.data = 0;
    si_channel.TUNE = 0;
    si_channel.CHANNEL_LO = 0; // 87.5MHz
    si_channel.CHANNEL_HI = 0; // 87.5MHz
    
    i2c_begin(SI_WRITE_ADDRESS);
    i2c_write_16(si_powerCfg.data, I2C_ACK_CONTINUE);
    i2c_write_16(si_channel.data, I2C_ACK_CONTINUE); // channel, tune disabled
    i2c_write_16(0x0080, I2C_ACK_CONTINUE); // System configuration 1: AGC ON, deemphasis Europe, interrupt disabled, RDS disabled
    i2c_write_16(0x001F, I2C_ACK_CONTINUE); // System configuration 2: 0db volume, Europe band (100 kHz), min RSSI
    i2c_write_16(0x0000, I2C_ACK_CONTINUE); // System configuration 3: most stops in seeks, standard volume
    i2c_write_16(0x3c04 | 0x8000, I2C_ACK_STOP); // see datahseet
    
    // Wait before initializing other registries
    CLRWDT();
    __delay_ms(150);
    CLRWDT();

    // Start muted and DISABLED
    si_powerCfg.ENABLE = 1;
    i2c_begin(SI_WRITE_ADDRESS);
    i2c_write_16(si_powerCfg.data, I2C_ACK_STOP);
}

void si_fm_forceMono(uint8_t forceMono) {
    uint8_t isMono = si_powerCfg.MONO;
    if (forceMono != isMono) {
        // Update mono status
        si_powerCfg.MONO = forceMono;
        // Only write register 2 (the first)
        i2c_begin(SI_WRITE_ADDRESS);
        i2c_write_16(si_powerCfg.data, I2C_ACK_STOP);
    }
}

void si_fm_mute(uint8_t mute) {
    uint8_t isMute = si_powerCfg.DMUTE;
    if (isMute != mute) {
        // Update mute status
        si_powerCfg.DMUTE = mute;
        // Only write register 2 (the first)
        i2c_begin(SI_WRITE_ADDRESS);
        i2c_write_16(si_powerCfg.data, I2C_ACK_STOP);
    }
}

// Freq is 50kHz step count over 87.5MHz. Max 410
void si_fm_tune(uint16_t freq) {
    si_channel.CHANNEL_HI = freq >> 8;
    si_channel.CHANNEL_LO = freq & 0xff;
    si_powerCfg.DMUTE = 0;

    // Reset the STC bit
    si_channel.TUNE = 0;
    i2c_begin(SI_WRITE_ADDRESS);
    i2c_write_16(si_powerCfg.data, I2C_ACK_CONTINUE);
    i2c_write_16(si_channel.data, I2C_ACK_STOP); 

    // Now tune
    si_channel.TUNE = 1;
    i2c_begin(SI_WRITE_ADDRESS);
    i2c_write_16(si_powerCfg.data, I2C_ACK_CONTINUE);
    i2c_write_16(si_channel.data, I2C_ACK_STOP); 

    _seeking = 1;
}

// Read SI status from i2c and update PIC ports
SI_FM_STATUS si_fm_status() {
    i2c_begin(SI_READ_ADDRESS);
    SI_STATUS_T status;
    status.data = i2c_read_16(I2C_ACK_STOP);
    
    if (status.STC && _seeking) {
        // Seek complete
        _seeking = 0;
        debug(SEEK_COMPLETE);
    }
    
    SI_FM_STATUS ret;
    ret.TUNED = status.STC && status.AFCRL;
    ret.STEREO = status.ST;
    return ret;
}

