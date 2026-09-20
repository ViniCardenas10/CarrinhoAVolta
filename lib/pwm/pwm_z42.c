#include "pwm_z42.h"

bool pwm_tpm_Init(TPM_MemMapPtr tpm, uint16_t clk, uint16_t module, uint8_t clock_mode, uint8_t ps, bool counting_mode) {
    if(tpm == TPM0) SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
    else if(tpm == TPM1) SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
    else if(tpm == TPM2) SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
    else return false;

    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(clk);
    tpm->MOD = module;
    tpm->SC |= TPM_SC_CMOD(clock_mode) | TPM_SC_PS(ps);

    if(counting_mode == CENTER_PWM) tpm->SC |= TPM_SC_CPWMS_MASK;
    else if(counting_mode == EDGE_PWM) tpm->SC &= ~TPM_SC_CPWMS_MASK;
    else return false;

    return true;
}

bool pwm_tpm_Ch_Init(TPM_MemMapPtr tpm, uint16_t channel, uint8_t mode, GPIO_MemMapPtr gpio, uint8_t pin) {
    if (tpm == TPM0) {
        if (gpio == GPIOD) {
            SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
            PORTD->PCR[pin] = PORT_PCR_MUX(4);
        }
    }
    
    tpm->CONTROLS[channel].CnSC |= mode;
    return true;
}

void pwm_tpm_CnV(TPM_MemMapPtr tpm, uint16_t channel, uint16_t value) {
    tpm->CONTROLS[channel].CnV = value;
}