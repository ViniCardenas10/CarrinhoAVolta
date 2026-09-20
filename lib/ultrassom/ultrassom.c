
#include "ultrassom.h"
#include "pwm_z42.h"
#include <zephyr/kernel.h>
#include "MKL25Z4.h"

#define VelocidadeSOM 34300.0f

volatile float distancia_atual = 999.0f;

void thread_ultrassom_func(void *arg1, void *arg2, void *arg3) {
    /* 1. Habilita o clock da porta A */
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    PORTA->PCR[4] = PORT_PCR_MUX(1);
    GPIOA->PDDR |= (1 << 4);

    PORTA->PCR[12] = PORT_PCR_MUX(1);
    GPIOA->PDDR &= ~(1 << 12);

    GPIOA->PCOR = (1 << 4);
    k_msleep(100);
    uint32_t freq_cpu = sys_clock_hw_cycles_per_sec();

    while (1) {
        GPIOA->PSOR = (1 << 4);
        k_busy_wait(15);
        GPIOA->PCOR = (1 << 4);

        int timeout = 100000;
        while ((GPIOA->PDIR & (1 << 12)) == 0 && timeout > 0) {
            timeout--;
        }

        if (timeout > 0) {
            uint32_t start_time = k_cycle_get_32();

            timeout = 500000;
            while ((GPIOA->PDIR & (1 << 12)) != 0 && timeout > 0) {
                timeout--;
            }

            if (timeout > 0) {
                uint32_t end_time = k_cycle_get_32();
                uint32_t ciclos = end_time - start_time;


                float tempo_segundos = (float)ciclos / (float)freq_cpu;
                
                /* Calcula a distância (d = v*t / 2) */
                distancia_atual = (tempo_segundos * VelocidadeSOM) / 2.0f;
            } else {
                distancia_atual = 999.0f; 
            }
        } else {
            distancia_atual = 999.0f;
        }

        k_msleep(60); 
    }
}