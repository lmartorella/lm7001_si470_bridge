
void si_fm_init();
// Freq is 50kHz step count over 87.5MHz. Max 410.
void si_fm_tune(uint16_t freq);

void si_fm_forceMono(uint8_t mono);
void si_fm_mute(uint8_t mute);

void si_fm_pollStatus();
uint8_t si_fm_isTuned();
uint8_t si_fm_isStereo();
void si_fm_postStatus();

