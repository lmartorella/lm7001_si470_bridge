
void si_fm_init();
// Freq is 50kHz step count over 87.5MHz. Max 410.
void si_fm_tune(uint16_t freq);

void si_fm_forceMono(uint8_t mono);
void si_fm_mute(uint8_t mute);

typedef struct {
    unsigned TUNED: 1;
    unsigned STEREO: 1;
} SI_FM_STATUS;

SI_FM_STATUS si_fm_status();
