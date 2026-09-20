#ifndef PWM_H
#define PWM_H

#include <stdbool.h>
#include <stdint.h>
#include "MKL25Z4.h"

#define TPM_INPUT_CAPTURE_RISING  (TPM_CnSC_ELSA_MASK)
#define TPM_INPUT_CAPTURE_FALLING (TPM_CnSC_ELSB_MASK)
#define TPM_CHANNEL_INTERRUPT     (TPM_CnSC_CHIE_MASK)

#define TPM_CLK_DIS     0
#define TPM_PLLFLL      1
#define TPM_OSCERCLK    2
#define TPM_MCGIRCLK    3
#define TPM_CLK         1

#define PS_128          7

#define TPM_PWM_H       (TPM_CnSC_MSB_MASK|TPM_CnSC_ELSB_MASK)
#define EDGE_PWM        0
#define CENTER_PWM      1

#ifndef TPM_MemMapPtr
#define TPM_MemMapPtr TPM_Type*
#endif

#ifndef GPIO_MemMapPtr
#define GPIO_MemMapPtr GPIO_Type*
#endif

bool pwm_tpm_Init(TPM_MemMapPtr tpm, uint16_t clk, uint16_t module, uint8_t clock_mode, uint8_t ps, bool counting_mode);
bool pwm_tpm_Ch_Init(TPM_MemMapPtr tpm, uint16_t channel, uint8_t mode, GPIO_MemMapPtr gpio, uint8_t pin);
void pwm_tpm_CnV(TPM_MemMapPtr tpm, uint16_t channel, uint16_t value);

#endif /* PWM_H */