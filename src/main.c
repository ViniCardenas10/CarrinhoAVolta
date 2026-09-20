#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <stdbool.h>

// ==========================================================
// CONTROLE DE VELOCIDADE DO AVANÇO
// ==========================================================
//
// DRIVE_SPEED_PERCENT:
//     velocidade geral do carrinho.
//
// Exemplos:
//     20 = aproximadamente 20%
//     30 = aproximadamente 30%
//     40 = aproximadamente 40%
//
// ATENÇÃO:
// O PWM abaixo usa período de 20 ms com resolução de 1 ms.
// Portanto, na prática, a resolução fica em aproximadamente
// 5% por passo.
//
// ----------------------------------------------------------

// Velocidade geral
#define DRIVE_SPEED_PERCENT      40

// Correção individual das rodas.
//
// Se o carro estiver virando para a DIREITA:
//     normalmente a roda esquerda está mais rápida.
//     Nesse caso, reduza RIGHT_TRIM_PERCENT.
//
// Se o carro estiver virando para a ESQUERDA:
//     normalmente a roda direita está mais rápida.
//     Nesse caso, reduza LEFT_TRIM_PERCENT.
//
// Exemplos:
//
//    0 / 0   -> sem correção
//    0 / -5  -> reduz a roda direita
//   -5 / 0   -> reduz a roda esquerda
//
// ----------------------------------------------------------

#define LEFT_TRIM_PERCENT         -3
#define RIGHT_TRIM_PERCENT        0

// Período do PWM
#define DRIVE_PWM_PERIOD_MS      20

// ==========================================================
// DEFINIÇÕES DE PINOS
// ==========================================================

// Ultrassom - Porta E
#define ECHO_PIN 20
#define TRIG_PIN 21

// Ponte H - Porta B
#define OUT1_PIN 0
#define OUT2_PIN 1
#define OUT3_PIN 2
#define OUT4_PIN 3

// Encoder óptico LM393 - Porta D
#define ENC1_PIN 7

// LED verde
#define GREEN_PIN 19

// LED vermelho
#define LED_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec led =
    GPIO_DT_SPEC_GET(LED_NODE, gpios);

// ==========================================================
// PARÂMETROS DAS CURVAS
// ==========================================================

// Tempo total aproximado da curva
#define TURN_TIME_MS     3000

// PWM da curva
#define TURN_PERIOD_MS   10
#define TURN_ON_MS       2

// Empurrão inicial para vencer o atrito
#define TURN_KICK_MS     30

// Distância considerada como obstáculo
#define OBSTACLE_CM      30

// ==========================================================
// ESTADOS DAS CURVAS
// ==========================================================
//
// CURVA DIREITA
//
// Motor A gira para trás
// Motor B fica freado
//
#define TURN_STATE_RIGHT   1, 0, 1, 1

// ==========================================================
// RETORNO PARA A POSIÇÃO ORIGINAL
// ==========================================================
//
// Faz o movimento contrário da curva direita
//
#define TURN_STATE_RETURN  0, 1, 1, 1

// ==========================================================
// CURVA ESQUERDA
// ==========================================================
//
// Agora usa a outra roda
//
#define TURN_STATE_LEFT    1, 1, 0, 1

// ==========================================================
// ENCODER
// ==========================================================

static volatile uint32_t leituras_enc = 0;

K_MSGQ_DEFINE(enc_q, sizeof(uint32_t), 128, 4);

static struct gpio_callback enc1_cb_data;

// ==========================================================
// ISR DO ENCODER
// ==========================================================

void enc1_isr(const struct device *dev,
              struct gpio_callback *cb,
              uint32_t pins)
{
    uint32_t n = ++leituras_enc;

    k_msgq_put(&enc_q, &n, K_NO_WAIT);
}

// ==========================================================
// THREAD DE IMPRESSÃO DO ENCODER
// ==========================================================

static void plot_thread(void *p1, void *p2, void *p3)
{
    uint32_t n;

    while (1) {

        k_msgq_get(&enc_q, &n, K_FOREVER);

        printk("%u\n", (unsigned int)n);
    }
}

K_THREAD_DEFINE(
    plot_tid,
    1024,
    plot_thread,
    NULL,
    NULL,
    NULL,
    7,
    0,
    0
);

// ==========================================================
// PONTE H
// ==========================================================

static void ponte_h(const struct device *dev,
                    int o1,
                    int o2,
                    int o3,
                    int o4)
{
    gpio_pin_set(dev, OUT1_PIN, o1);
    gpio_pin_set(dev, OUT2_PIN, o2);
    gpio_pin_set(dev, OUT3_PIN, o3);
    gpio_pin_set(dev, OUT4_PIN, o4);
}

// ==========================================================
// LED VERMELHO
// ==========================================================

static void led_vermelho(const struct device *gpio_b_dev)
{
    // Vermelho ligado
    gpio_pin_set_dt(&led, 1);

    // Verde desligado
    gpio_pin_set(gpio_b_dev, GREEN_PIN, 0);
}

// ==========================================================
// LED VERDE
// ==========================================================

static void led_verde(const struct device *gpio_b_dev)
{
    // Vermelho desligado
    gpio_pin_set_dt(&led, 0);

    // Verde ligado
    gpio_pin_set(gpio_b_dev, GREEN_PIN, 1);
}

// ==========================================================
// LED AMARELO
// ==========================================================
//
// Assume que vermelho + verde acesos = amarelo.
//

static void led_amarelo(const struct device *gpio_b_dev)
{
    // Vermelho ligado
    gpio_pin_set_dt(&led, 1);

    // Verde ligado
    gpio_pin_set(gpio_b_dev, GREEN_PIN, 1);
}

// ==========================================================
// LED DESLIGADO
// ==========================================================

static void leds_desligados(const struct device *gpio_b_dev)
{
    gpio_pin_set_dt(&led, 0);
    gpio_pin_set(gpio_b_dev, GREEN_PIN, 0);
}

// ==========================================================
// CALCULA A VELOCIDADE DA RODA
// ==========================================================

static int calcular_velocidade(int velocidade_base,
                               int correcao)
{
    int velocidade = velocidade_base + correcao;

    if (velocidade < 0) {
        velocidade = 0;
    }

    if (velocidade > 100) {
        velocidade = 100;
    }

    return velocidade;
}

// ==========================================================
// MEDIÇÃO DO ULTRASSOM
// ==========================================================

static uint32_t medir_distancia_cm(const struct device *gpio_e_dev)
{
    uint32_t timeout = 100000;
    uint32_t distance_cm = 999;

    // ------------------------------------------------------
    // TRIGGER
    // ------------------------------------------------------

    gpio_pin_set(gpio_e_dev, TRIG_PIN, 1);

    k_busy_wait(10);

    gpio_pin_set(gpio_e_dev, TRIG_PIN, 0);

    // ------------------------------------------------------
    // ESPERA ECHO SUBIR
    // ------------------------------------------------------

    while (gpio_pin_get(gpio_e_dev, ECHO_PIN) == 0 &&
           timeout > 0) {
        timeout--;
    }

    if (timeout > 0) {

        uint32_t start_time = k_cycle_get_32();

        timeout = 100000;

        // --------------------------------------------------
        // ESPERA ECHO DESCER
        // --------------------------------------------------

        while (gpio_pin_get(gpio_e_dev, ECHO_PIN) == 1 &&
               timeout > 0) {
            timeout--;
        }

        if (timeout > 0) {

            uint32_t end_time = k_cycle_get_32();

            uint32_t duration_us =
                k_cyc_to_us_floor32(
                    end_time - start_time
                );

            distance_cm = duration_us / 58;
        }
    }

    return distance_cm;
}

// ==========================================================
// INDICA A DISTÂNCIA PELO LED
// ==========================================================

static void indicar_distancia(const struct device *gpio_b_dev,
                              uint32_t distance_cm)
{
    if (distance_cm < OBSTACLE_CM) {

        led_vermelho(gpio_b_dev);

    } else {

        led_verde(gpio_b_dev);
    }
}

// ==========================================================
// EXECUTA UMA CURVA
// ==========================================================
//
// Durante a curva:
//     LED amarelo
//
// Depois da curva:
//     freia
//     mede
//     LED vermelho ou verde
//     espera 3 segundos
//
// Retorna a distância encontrada.
//

static uint32_t executar_curva(
    const struct device *gpio_b_dev,
    const struct device *gpio_e_dev,
    int o1,
    int o2,
    int o3,
    int o4)
{
    // ======================================================
    // LED AMARELO
    // ======================================================

    led_amarelo(gpio_b_dev);

    // ======================================================
    // EMPURRÃO INICIAL
    // ======================================================

    ponte_h(
        gpio_b_dev,
        o1,
        o2,
        o3,
        o4
    );

    k_msleep(TURN_KICK_MS);

    // ======================================================
    // EXECUÇÃO DA CURVA
    // ======================================================

    int64_t t0 = k_uptime_get();

    while ((k_uptime_get() - t0) < TURN_TIME_MS) {

        // Motores ligados
        ponte_h(
            gpio_b_dev,
            o1,
            o2,
            o3,
            o4
        );

        k_msleep(TURN_ON_MS);

        // Motores desligados
        ponte_h(
            gpio_b_dev,
            0,
            0,
            0,
            0
        );

        k_msleep(
            TURN_PERIOD_MS - TURN_ON_MS
        );
    }

    // ======================================================
    // FREIO
    // ======================================================

    ponte_h(
        gpio_b_dev,
        1,
        1,
        1,
        1
    );

    k_msleep(300);

    // ======================================================
    // MEDE A DISTÂNCIA
    // ======================================================

    uint32_t distance_cm =
        medir_distancia_cm(gpio_e_dev);

    // ======================================================
    // LED INDICA SE HÁ OBSTÁCULO
    // ======================================================

    indicar_distancia(
        gpio_b_dev,
        distance_cm
    );

    // ======================================================
    // PAUSA DE 3 SEGUNDOS
    // ======================================================

    k_msleep(3000);

    return distance_cm;
}

// ==========================================================
// ANDAR PARA FRENTE
// ==========================================================
//
// Usa PWM independente para cada roda.
//
// LEFT_SPEED  = velocidade da esquerda
// RIGHT_SPEED = velocidade da direita
//
// Isso permite compensar o desbalanceamento.
//
static void andar_para_frente(const struct device *dev)
{
    // ======================================================
    // CALCULA VELOCIDADE DE CADA RODA
    // ======================================================

    int left_percent =
        calcular_velocidade(
            DRIVE_SPEED_PERCENT,
            LEFT_TRIM_PERCENT
        );

    int right_percent =
        calcular_velocidade(
            DRIVE_SPEED_PERCENT,
            RIGHT_TRIM_PERCENT
        );

    // ======================================================
    // CONVERTE % PARA TEMPO LIGADO
    // ======================================================

    int left_on_ms =
        (DRIVE_PWM_PERIOD_MS * left_percent) / 100;

    int right_on_ms =
        (DRIVE_PWM_PERIOD_MS * right_percent) / 100;

    // ======================================================
    // PWM
    // ======================================================

    for (int t = 0;
         t < DRIVE_PWM_PERIOD_MS;
         t++) {

        // --------------------------------------------------
        // ESTADO DAS RODAS
        // --------------------------------------------------

        bool left_on =
            (t < left_on_ms);

        bool right_on =
            (t < right_on_ms);

        // --------------------------------------------------
        // RODA ESQUERDA
        // --------------------------------------------------

        int out1 = 0;
        int out2 = left_on ? 1 : 0;

        // --------------------------------------------------
        // RODA DIREITA
        // --------------------------------------------------

        int out3 = right_on ? 1 : 0;
        int out4 = 0;

        // --------------------------------------------------
        // APLICA NA PONTE H
        // --------------------------------------------------

        ponte_h(
            dev,
            out1,
            out2,
            out3,
            out4
        );

        k_msleep(1);
    }

    // ------------------------------------------------------
    // DESLIGA
    // ------------------------------------------------------

    ponte_h(
        dev,
        0,
        0,
        0,
        0
    );
}

// ==========================================================
// MAIN
// ==========================================================

int main(void)
{
    // ======================================================
    // VERIFICA LED
    // ======================================================

    if (!device_is_ready(led.port)) {
        return 0;
    }

    // ======================================================
    // DISPOSITIVOS GPIO
    // ======================================================

    const struct device *gpio_b_dev =
        DEVICE_DT_GET(DT_NODELABEL(gpiob));

    const struct device *gpio_d_dev =
        DEVICE_DT_GET(DT_NODELABEL(gpiod));

    const struct device *gpio_e_dev =
        DEVICE_DT_GET(DT_NODELABEL(gpioe));

    // ======================================================
    // VERIFICA DISPOSITIVOS
    // ======================================================

    if (!device_is_ready(gpio_b_dev) ||
        !device_is_ready(gpio_d_dev) ||
        !device_is_ready(gpio_e_dev)) {

        while (1) {

            gpio_pin_toggle_dt(&led);

            k_msleep(50);
        }
    }

    // ======================================================
    // CONFIGURAÇÃO DOS PINOS
    // ======================================================

    // ------------------------------------------------------
    // LED VERMELHO
    // ------------------------------------------------------

    gpio_pin_configure_dt(
        &led,
        GPIO_OUTPUT_INACTIVE
    );

    // ------------------------------------------------------
    // LED VERDE
    // ------------------------------------------------------

    gpio_pin_configure(
        gpio_b_dev,
        GREEN_PIN,
        GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW
    );

    // ------------------------------------------------------
    // ULTRASSOM
    // ------------------------------------------------------

    gpio_pin_configure(
        gpio_e_dev,
        TRIG_PIN,
        GPIO_OUTPUT_INACTIVE
    );

    gpio_pin_configure(
        gpio_e_dev,
        ECHO_PIN,
        GPIO_INPUT
    );

    // ------------------------------------------------------
    // PONTE H
    // ------------------------------------------------------

    gpio_pin_configure(
        gpio_b_dev,
        OUT1_PIN,
        GPIO_OUTPUT_INACTIVE
    );

    gpio_pin_configure(
        gpio_b_dev,
        OUT2_PIN,
        GPIO_OUTPUT_INACTIVE
    );

    gpio_pin_configure(
        gpio_b_dev,
        OUT3_PIN,
        GPIO_OUTPUT_INACTIVE
    );

    gpio_pin_configure(
        gpio_b_dev,
        OUT4_PIN,
        GPIO_OUTPUT_INACTIVE
    );

    // ------------------------------------------------------
    // ENCODER
    // ------------------------------------------------------

    gpio_pin_configure(
        gpio_d_dev,
        ENC1_PIN,
        GPIO_INPUT
    );

    // ======================================================
    // INTERRUPÇÃO DO ENCODER
    // ======================================================

    gpio_pin_interrupt_configure(
        gpio_d_dev,
        ENC1_PIN,
        GPIO_INT_EDGE_TO_ACTIVE
    );

    gpio_init_callback(
        &enc1_cb_data,
        enc1_isr,
        BIT(ENC1_PIN)
    );

    gpio_add_callback(
        gpio_d_dev,
        &enc1_cb_data
    );

    // ======================================================
    // ESTADO INICIAL DOS LEDS
    // ======================================================

    leds_desligados(gpio_b_dev);

    // ======================================================
    // LOOP PRINCIPAL
    // ======================================================

    while (1) {

        // --------------------------------------------------
        // MEDE A DISTÂNCIA À FRENTE
        // --------------------------------------------------

        uint32_t distance_cm =
            medir_distancia_cm(gpio_e_dev);

        // ==================================================
        // EXISTE OBSTÁCULO?
        // ==================================================

        if (distance_cm < OBSTACLE_CM) {

            // ------------------------------------------------
            // LED VERMELHO
            // ------------------------------------------------

            led_vermelho(gpio_b_dev);

            // ------------------------------------------------
            // FREIA
            // ------------------------------------------------

            ponte_h(
                gpio_b_dev,
                1,
                1,
                1,
                1
            );

            // Aguarda 3 segundos
            k_msleep(3000);

            // =================================================
            // 1. VIRA PARA A DIREITA
            // =================================================

            distance_cm =
                executar_curva(
                    gpio_b_dev,
                    gpio_e_dev,
                    TURN_STATE_RIGHT
                );

            // =================================================
            // 2. A DIREITA ESTÁ BLOQUEADA?
            // =================================================

            if (distance_cm < OBSTACLE_CM) {

                // =================================================
                // VOLTA PARA A POSIÇÃO ORIGINAL
                // =================================================

                distance_cm =
                    executar_curva(
                        gpio_b_dev,
                        gpio_e_dev,
                        TURN_STATE_RETURN
                    );

                // =================================================
                // 3. VIRA PARA A ESQUERDA
                // =================================================

                distance_cm =
                    executar_curva(
                        gpio_b_dev,
                        gpio_e_dev,
                        TURN_STATE_LEFT
                    );

                // =================================================
                // 4. A ESQUERDA TAMBÉM ESTÁ BLOQUEADA?
                // =================================================

                if (distance_cm < OBSTACLE_CM) {

                    // ------------------------------------------------
                    // LED VERMELHO
                    // ------------------------------------------------

                    led_vermelho(gpio_b_dev);

                    // ------------------------------------------------
                    // PARA COMPLETAMENTE
                    // ------------------------------------------------

                    ponte_h(
                        gpio_b_dev,
                        1,
                        1,
                        1,
                        1
                    );

                    // ------------------------------------------------
                    // FICA PARADO ENQUANTO ESTIVER BLOQUEADO
                    // ------------------------------------------------

                    while (
                        medir_distancia_cm(gpio_e_dev)
                        < OBSTACLE_CM
                    ) {

                        k_msleep(100);
                    }

                } else {

                    // ------------------------------------------------
                    // ESQUERDA LIVRE
                    // ------------------------------------------------

                    led_verde(gpio_b_dev);
                }
            }

            // Se direita estiver livre, o carro continua
            // na direção para a qual virou.

        } else {

            // =================================================
            // CAMINHO LIVRE
            // =================================================

            led_verde(gpio_b_dev);

            // ------------------------------------------------
            // ANDA PARA FRENTE
            // ------------------------------------------------

            andar_para_frente(gpio_b_dev);
        }
    }

    return 0;
}