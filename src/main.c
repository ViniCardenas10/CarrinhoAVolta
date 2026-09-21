#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <stdbool.h>
#include <stdint.h>


/* ==========================================================
 *                 CONFIGURAÇÕES DO PROJETO
 * ==========================================================
 *
 * Todos os parâmetros ajustáveis estão aqui.
 *
 * ========================================================== */


/* ==========================================================
 * VELOCIDADE DO AVANÇO
 * ========================================================== */

/*
 * Velocidade geral do carrinho.
 *
 * 20 -> mais lento
 * 30 -> lento
 * 40 -> referência
 * 50 -> mais rápido
 * 70 -> configuração atual
 */
#define DRIVE_SPEED_PERCENT       70

/*
 * Correção individual das rodas.
 *
 * Se o carrinho estiver desviando para a DIREITA:
 *
 * LEFT_TRIM_PERCENT    0
 * RIGHT_TRIM_PERCENT  -5
 *
 * Se estiver desviando para a ESQUERDA:
 *
 * LEFT_TRIM_PERCENT   -5
 * RIGHT_TRIM_PERCENT   0
 */
#define LEFT_TRIM_PERCENT          0
#define RIGHT_TRIM_PERCENT         0

/*
 * Período do PWM do avanço.
 */
#define DRIVE_PWM_PERIOD_MS       10


/* ==========================================================
 * VELOCIDADE DAS CURVAS
 * ========================================================== */

/*
 * Período total do PWM da curva.
 */
#define TURN_PERIOD_MS            10

/*
 * Tempo que as rodas ficam ligadas dentro do período.
 *
 * 2 -> ~20%
 * 3 -> ~30%
 * 4 -> ~40%
 * 5 -> ~50%
 * 6 -> ~60%
 *
 * Quanto maior, mais rápida a curva.
 */
#define TURN_ON_MS                 4


/* ==========================================================
 * CURVA CONTROLADA PELO ENCODER
 * ========================================================== */

/*
 * Quantidade de pulsos necessária para a curva.
 *
 * Esse valor deve ser calibrado para obter aproximadamente
 * 90 graus.
 */
#define TURN_ENCODER_PULSES        5

/*
 * Tempo MÍNIMO que a curva deve durar.
 *
 * Mesmo se o encoder atingir os pulsos rapidamente,
 * a curva não termina antes desse tempo.
 */
#define TURN_MIN_TIME_MS         1200

/*
 * Tempo MÁXIMO permitido para uma curva.
 *
 * Segurança caso o encoder pare de gerar pulsos.
 */
#define TURN_MAX_TIME_MS       10000

/*
 * Pausa depois de cada curva.
 */
#define TURN_PAUSE_MS           3000


/* ==========================================================
 * ULTRASSOM
 * ========================================================== */

/*
 * Distância abaixo da qual há obstáculo.
 */
#define OBSTACLE_CM               20

/*
 * Tempo parado ao detectar obstáculo antes de iniciar
 * a tentativa de desvio.
 */
#define OBSTACLE_INITIAL_PAUSE_MS 3000

/*
 * Tempo para estabilização após o freio.
 */
#define BRAKE_SETTLE_MS            300


/* ==========================================================
 * ENCODER HW-201
 * ========================================================== */

/*
 * Tempo que o sinal precisa permanecer LOW para
 * confirmar que voltou para a região preta.
 *
 * Valor encontrado experimentalmente.
 */
#define ENC_LOW_STABLE_MS          30

/*
 * Intervalo MÍNIMO absoluto entre dois pulsos aceitos.
 *
 * Isso evita que oscilações muito rápidas do HW-201
 * sejam consideradas vários pulsos diferentes.
 */
#define ENC_MIN_PULSE_MS           30

/*
 * Período da thread que monitora o retorno ao preto.
 */
#define ENC_MONITOR_PERIOD_MS       1


/* ==========================================================
 * DISTÂNCIA DA RODA / ENCODER
 * ========================================================== */

/*
 * Diâmetro da roda:
 *
 * 7 cm = 70 mm
 */
#define WHEEL_DIAMETER_MM         70

/*
 * Quantidade de regiões brancas por volta.
 */
#define ENCODER_TRACKS             8

/*
 * PI em ponto fixo.
 *
 * 3.14159
 */
#define PI_NUM                 314159
#define PI_DEN                 100000


/* ==========================================================
 * PINOS
 * ========================================================== */

/*
 * Ultrassom - Porta E
 */
#define ECHO_PIN                   20
#define TRIG_PIN                   21

/*
 * Ponte H - Porta B
 */
#define OUT1_PIN                    0
#define OUT2_PIN                    1
#define OUT3_PIN                    2
#define OUT4_PIN                    3

/*
 * Encoder HW-201 - PTD7
 */
#define ENC1_PIN                    7

/*
 * LED verde
 */
#define GREEN_PIN                  19

/*
 * LED vermelho
 */
#define LED_NODE                    DT_ALIAS(led2)


/* ==========================================================
 * ESTADOS DAS CURVAS
 * ========================================================== */

/*
 * CURVA PARA A DIREITA
 *
 * Roda esquerda -> FRENTE
 * Roda direita  -> RÉ
 *
 * O carrinho gira para a direita.
 */
#define TURN_STATE_RIGHT           0, 1, 0, 1


/*
 * RETORNO PARA A POSIÇÃO ORIGINAL
 *
 * Para desfazer a curva para a direita:
 *
 * Roda esquerda -> RÉ
 * Roda direita  -> FRENTE
 *
 * Este estado é igual ao movimento da curva para esquerda.
 */
#define TURN_STATE_RETURN          1, 0, 1, 0


/*
 * CURVA PARA A ESQUERDA
 *
 * Roda esquerda -> RÉ
 * Roda direita  -> FRENTE
 */
#define TURN_STATE_LEFT            1, 0, 1, 0


/* ==========================================================
 * LED
 * ========================================================== */

static const struct gpio_dt_spec led =
    GPIO_DT_SPEC_GET(LED_NODE, gpios);


/* ==========================================================
 * ESTADO DO ENCODER
 * ========================================================== */

/*
 * Contador TOTAL de regiões brancas detectadas.
 *
 * Inclui:
 *
 * - linha reta
 * - curvas
 * - retorno
 */
static volatile uint32_t encoder_count = 0;


/*
 * Contador somente durante linha reta.
 */
static volatile uint32_t encoder_reta_count = 0;


/*
 * Indica se o carrinho está andando em linha reta.
 */
static volatile bool encoder_em_reta = false;


/*
 * true:
 *     a região branca atual já foi contada.
 *
 * false:
 *     uma nova região branca pode ser contada.
 */
static volatile bool aguardando_preto = false;


/*
 * Momento do último pulso aceito.
 *
 * Usado pelo filtro ENC_MIN_PULSE_MS.
 */
static volatile int64_t ultimo_pulso_ms = -1000;


/*
 * GPIO da Porta D.
 *
 * Global porque também é utilizado pelas threads.
 */
static const struct device *gpio_d_dev;


/* ==========================================================
 * EVENTO DO ENCODER
 * ========================================================== */

struct encoder_event {
    uint32_t total;
    uint32_t reta;
};


/* ==========================================================
 * FILA DO ENCODER
 * ========================================================== */

K_MSGQ_DEFINE(
    enc_q,
    sizeof(struct encoder_event),
    128,
    4
);


/* ==========================================================
 * CALLBACK
 * ========================================================== */

static struct gpio_callback enc_cb_data;


/* ==========================================================
 * CONVERSÃO DE PULSOS PARA DISTÂNCIA
 * ========================================================== */

/*
 * Retorna a distância percorrida em milímetros.
 *
 * Para a sua roda:
 *
 * diâmetro = 70 mm
 *
 * circunferência ≈ 219,91 mm
 *
 * 8 pulsos por volta
 *
 * 1 pulso ≈ 27,49 mm
 * 1 pulso ≈ 2,75 cm
 */
static uint32_t encoder_para_distancia_mm(
    uint32_t pulsos)
{
    /*
     * Circunferência em micrômetros.
     */
    uint64_t circunferencia_um =
        (
            (uint64_t)WHEEL_DIAMETER_MM *
            1000ULL *
            PI_NUM
        ) /
        PI_DEN;

    /*
     * Distância total em micrômetros.
     */
    uint64_t distancia_um =
        (
            (uint64_t)pulsos *
            circunferencia_um
        ) /
        ENCODER_TRACKS;

    /*
     * Retorna em milímetros.
     */
    return (uint32_t)(
        distancia_um / 1000ULL
    );
}


/* ==========================================================
 * INTERRUPÇÃO DO ENCODER
 * ========================================================== */

/*
 * Consideramos:
 *
 * PRETO  = LOW
 * BRANCO = HIGH
 *
 * A borda de subida representa entrada na região branca.
 */
static void encoder_isr(
    const struct device *dev,
    struct gpio_callback *cb,
    uint32_t pins)
{
    /*
     * Se ainda estamos dentro da mesma região branca,
     * ignoramos.
     */
    if (aguardando_preto) {
        return;
    }

    /*
     * Confirma que o pino realmente está HIGH.
     */
    if (gpio_pin_get_raw(dev, ENC1_PIN) != 1) {
        return;
    }

    /*
     * Tempo atual.
     */
    int64_t agora_ms = k_uptime_get();

    /*
     * ======================================================
     * FILTRO DE TEMPO MÍNIMO ENTRE PULSOS
     * ======================================================
     */
    if (
        (agora_ms - ultimo_pulso_ms)
        <
        ENC_MIN_PULSE_MS
    ) {
        return;
    }

    /*
     * Atualiza instante do último pulso aceito.
     */
    ultimo_pulso_ms = agora_ms;

    /*
     * Incrementa contador TOTAL.
     */
    encoder_count++;

    /*
     * Se estiver em linha reta,
     * incrementa também o contador da reta.
     */
    if (encoder_em_reta) {
        encoder_reta_count++;
    }

    /*
     * Bloqueia novas contagens até retornar ao preto.
     */
    aguardando_preto = true;

    /*
     * Prepara evento para o Serial Monitor.
     */
    struct encoder_event event = {
        .total = encoder_count,
        .reta = encoder_reta_count
    };

    /*
     * Não bloqueia dentro da ISR.
     */
    k_msgq_put(
        &enc_q,
        &event,
        K_NO_WAIT
    );
}


/* ==========================================================
 * THREAD DE IMPRESSÃO
 * ========================================================== */

static void plot_thread(
    void *p1,
    void *p2,
    void *p3)
{
    struct encoder_event event;

    while (1) {

        /*
         * Aguarda novo pulso.
         */
        k_msgq_get(
            &enc_q,
            &event,
            K_FOREVER
        );

        /*
         * Distância TOTAL.
         */
        uint32_t distancia_total_mm =
            encoder_para_distancia_mm(
                event.total
            );

        /*
         * Distância somente em linha reta.
         */
        uint32_t distancia_reta_mm =
            encoder_para_distancia_mm(
                event.reta
            );

        /*
         * Total em centímetros.
         */
        uint32_t total_cm =
            distancia_total_mm / 10;

        uint32_t total_decimo =
            distancia_total_mm % 10;

        /*
         * Reta em centímetros.
         */
        uint32_t reta_cm =
            distancia_reta_mm / 10;

        uint32_t reta_decimo =
            distancia_reta_mm % 10;

        /*
         * Mostra no Serial Monitor.
         */
        printk(
            "Encoder: %u | "
            "Reta: %u | "
            "Distancia total: %u.%u cm | "
            "Distancia reta: %u.%u cm\n",

            (unsigned int)event.total,

            (unsigned int)event.reta,

            (unsigned int)total_cm,
            (unsigned int)total_decimo,

            (unsigned int)reta_cm,
            (unsigned int)reta_decimo
        );
    }
}


/* ==========================================================
 * THREAD DE MONITORAMENTO DO HW-201
 * ========================================================== */

static void encoder_monitor_thread(
    void *p1,
    void *p2,
    void *p3)
{
    while (1) {

        if (aguardando_preto) {

            int estado =
                gpio_pin_get_raw(
                    gpio_d_dev,
                    ENC1_PIN
                );

            /*
             * LOW = voltou para o preto.
             */
            if (estado == 0) {

                /*
                 * Confirma LOW por 30 ms.
                 */
                k_msleep(
                    ENC_LOW_STABLE_MS
                );

                /*
                 * Lê novamente.
                 */
                estado =
                    gpio_pin_get_raw(
                        gpio_d_dev,
                        ENC1_PIN
                    );

                /*
                 * Se continua LOW,
                 * libera nova contagem.
                 */
                if (estado == 0) {
                    aguardando_preto = false;
                }
            }
        }

        k_msleep(
            ENC_MONITOR_PERIOD_MS
        );
    }
}


/* ==========================================================
 * THREADS
 * ========================================================== */

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

K_THREAD_DEFINE(
    encoder_monitor_tid,
    1024,
    encoder_monitor_thread,
    NULL,
    NULL,
    NULL,
    7,
    0,
    0
);


/* ==========================================================
 * PONTE H
 * ========================================================== */

static void ponte_h(
    const struct device *dev,
    int o1,
    int o2,
    int o3,
    int o4)
{
    gpio_pin_set(
        dev,
        OUT1_PIN,
        o1
    );

    gpio_pin_set(
        dev,
        OUT2_PIN,
        o2
    );

    gpio_pin_set(
        dev,
        OUT3_PIN,
        o3
    );

    gpio_pin_set(
        dev,
        OUT4_PIN,
        o4
    );
}


/* ==========================================================
 * LED VERMELHO
 * ========================================================== */

static void led_vermelho(
    const struct device *gpio_b_dev)
{
    gpio_pin_set_dt(
        &led,
        1
    );

    gpio_pin_set(
        gpio_b_dev,
        GREEN_PIN,
        0
    );
}


/* ==========================================================
 * LED VERDE
 * ========================================================== */

static void led_verde(
    const struct device *gpio_b_dev)
{
    gpio_pin_set_dt(
        &led,
        0
    );

    gpio_pin_set(
        gpio_b_dev,
        GREEN_PIN,
        1
    );
}


/* ==========================================================
 * LED AMARELO
 * ========================================================== */

static void led_amarelo(
    const struct device *gpio_b_dev)
{
    /*
     * Vermelho + verde ligados.
     */
    gpio_pin_set_dt(
        &led,
        1
    );

    gpio_pin_set(
        gpio_b_dev,
        GREEN_PIN,
        1
    );
}


/* ==========================================================
 * LED DESLIGADO
 * ========================================================== */

static void leds_desligados(
    const struct device *gpio_b_dev)
{
    gpio_pin_set_dt(
        &led,
        0
    );

    gpio_pin_set(
        gpio_b_dev,
        GREEN_PIN,
        0
    );
}


/* ==========================================================
 * LIMITA VELOCIDADE
 * ========================================================== */

static int limitar_velocidade(
    int valor)
{
    if (valor < 0) {
        return 0;
    }

    if (valor > 100) {
        return 100;
    }

    return valor;
}


/* ==========================================================
 * MEDIÇÃO DO ULTRASSOM
 * ========================================================== */

static uint32_t medir_distancia_cm(
    const struct device *gpio_e_dev)
{
    uint32_t timeout = 100000;
    uint32_t distance_cm = 999;

    /*
     * Trigger.
     */
    gpio_pin_set(
        gpio_e_dev,
        TRIG_PIN,
        1
    );

    k_busy_wait(10);

    gpio_pin_set(
        gpio_e_dev,
        TRIG_PIN,
        0
    );

    /*
     * Espera Echo subir.
     */
    while (
        gpio_pin_get(
            gpio_e_dev,
            ECHO_PIN
        ) == 0 &&
        timeout > 0
    ) {
        timeout--;
    }

    if (timeout > 0) {

        uint32_t start_time =
            k_cycle_get_32();

        timeout = 100000;

        /*
         * Espera Echo descer.
         */
        while (
            gpio_pin_get(
                gpio_e_dev,
                ECHO_PIN
            ) == 1 &&
            timeout > 0
        ) {
            timeout--;
        }

        if (timeout > 0) {

            uint32_t end_time =
                k_cycle_get_32();

            uint32_t duration_us =
                k_cyc_to_us_floor32(
                    end_time - start_time
                );

            /*
             * Conversão aproximada.
             */
            distance_cm =
                duration_us / 58;
        }
    }

    return distance_cm;
}


/* ==========================================================
 * INDICA DISTÂNCIA PELO LED
 * ========================================================== */

static void indicar_distancia(
    const struct device *gpio_b_dev,
    uint32_t distance_cm)
{
    if (
        distance_cm <
        OBSTACLE_CM
    ) {

        led_vermelho(
            gpio_b_dev
        );

    } else {

        led_verde(
            gpio_b_dev
        );
    }
}


/* ==========================================================
 * SINCRONIZA O ENCODER ANTES DE UMA CURVA
 * ========================================================== */

/*
 * Evita que o estado do HW-201 no instante em que a
 * curva começa cause uma contagem indevida.
 */
static void sincronizar_encoder_antes_da_curva(void)
{
    /*
     * Desabilita temporariamente a interrupção.
     */
    gpio_pin_interrupt_configure(
        gpio_d_dev,
        ENC1_PIN,
        GPIO_INT_DISABLE
    );

    /*
     * Lê o estado atual.
     */
    int estado =
        gpio_pin_get_raw(
            gpio_d_dev,
            ENC1_PIN
        );

    /*
     * ------------------------------------------------------
     * SENSOR NO BRANCO
     * ------------------------------------------------------
     */
    if (estado == 1) {

        /*
         * Já estamos sobre uma região branca.
         * Essa região não será contada.
         */
        aguardando_preto = true;

    } else {

        /*
         * Sensor no PRETO.
         */
        aguardando_preto = true;

        /*
         * Confirma estabilidade por 30 ms.
         */
        k_msleep(
            ENC_LOW_STABLE_MS
        );

        /*
         * Confirma novamente.
         */
        estado =
            gpio_pin_get_raw(
                gpio_d_dev,
                ENC1_PIN
            );

        if (estado == 0) {

            /*
             * Realmente está no preto.
             */
            aguardando_preto = false;

        } else {

            /*
             * Ainda está no branco.
             */
            aguardando_preto = true;
        }
    }

    /*
     * Reativa interrupção.
     */
    gpio_pin_interrupt_configure(
        gpio_d_dev,
        ENC1_PIN,
        GPIO_INT_EDGE_RISING
    );
}


/* ==========================================================
 * EXECUTA CURVA CONTROLADA PELO ENCODER
 * ========================================================== */

static uint32_t executar_curva(
    const struct device *gpio_b_dev,
    const struct device *gpio_e_dev,
    int o1,
    int o2,
    int o3,
    int o4)
{
    /*
     * Não estamos andando em linha reta.
     */
    encoder_em_reta = false;

    /*
     * Sincroniza o HW-201.
     */
    sincronizar_encoder_antes_da_curva();

    /*
     * Guarda a contagem no início.
     */
    uint32_t encoder_inicial =
        encoder_count;

    /*
     * LED AMARELO durante a curva.
     */
    led_amarelo(
        gpio_b_dev
    );

    /*
     * Marca começo da curva.
     */
    int64_t inicio =
        k_uptime_get();

    /*
     * ======================================================
     * CURVA
     * ======================================================
     *
     * Para terminar:
     *
     * 1) tempo >= TURN_MIN_TIME_MS
     *
     * E
     *
     * 2) pulsos >= TURN_ENCODER_PULSES
     *
     * TURN_MAX_TIME_MS é apenas uma proteção.
     */
    while (1) {

        /*
         * Tempo decorrido.
         */
        int64_t tempo_decorrido =
            k_uptime_get() -
            inicio;

        /*
         * Pulsos utilizados na curva.
         */
        uint32_t pulsos =
            encoder_count -
            encoder_inicial;

        /*
         * Critério normal.
         */
        if (
            tempo_decorrido >=
            TURN_MIN_TIME_MS
            &&
            pulsos >=
            TURN_ENCODER_PULSES
        ) {
            break;
        }

        /*
         * Proteção.
         */
        if (
            tempo_decorrido >=
            TURN_MAX_TIME_MS
        ) {

            printk(
                "AVISO: limite maximo da curva atingido!\n"
            );

            break;
        }

        /*
         * Liga as duas rodas.
         */
        ponte_h(
            gpio_b_dev,
            o1,
            o2,
            o3,
            o4
        );

        k_msleep(
            TURN_ON_MS
        );

        /*
         * Desliga as duas rodas.
         */
        ponte_h(
            gpio_b_dev,
            0,
            0,
            0,
            0
        );

        k_msleep(
            TURN_PERIOD_MS -
            TURN_ON_MS
        );
    }

    /*
     * ======================================================
     * FREIO
     * ======================================================
     */
    ponte_h(
        gpio_b_dev,
        1,
        1,
        1,
        1
    );

    k_msleep(
        BRAKE_SETTLE_MS
    );

    /*
     * Pulsos usados na curva.
     */
    uint32_t pulsos_curva =
        encoder_count -
        encoder_inicial;

    /*
     * Tempo efetivo.
     */
    int64_t tempo_curva =
        k_uptime_get() -
        inicio;

    /*
     * Distância correspondente aos pulsos da curva.
     */
    uint32_t distancia_curva_mm =
        encoder_para_distancia_mm(
            pulsos_curva
        );

    uint32_t distancia_curva_cm =
        distancia_curva_mm / 10;

    uint32_t distancia_curva_decimo =
        distancia_curva_mm % 10;

    /*
     * Informações da curva no Serial Monitor.
     */
    printk(
        "Curva finalizada | "
        "Pulsos: %u | "
        "Tempo: %lld ms | "
        "Distancia encoder: %u.%u cm | "
        "Encoder total: %u\n",

        (unsigned int)pulsos_curva,

        (long long)tempo_curva,

        (unsigned int)distancia_curva_cm,
        (unsigned int)distancia_curva_decimo,

        (unsigned int)encoder_count
    );

    /*
     * ======================================================
     * MEDE DISTÂNCIA NA NOVA DIREÇÃO
     * ======================================================
     */
    uint32_t distance_cm =
        medir_distancia_cm(
            gpio_e_dev
        );

    /*
     * ======================================================
     * LED
     * ======================================================
     */
    indicar_distancia(
        gpio_b_dev,
        distance_cm
    );

    /*
     * ======================================================
     * PAUSA
     * ======================================================
     */
    k_msleep(
        TURN_PAUSE_MS
    );

    return distance_cm;
}


/* ==========================================================
 * ANDAR PARA FRENTE
 * ========================================================== */

static void andar_para_frente(
    const struct device *dev)
{
    /*
     * Habilita a contagem da linha reta.
     */
    encoder_em_reta = true;

    /*
     * Velocidade da esquerda.
     */
    int left_percent =
        limitar_velocidade(
            DRIVE_SPEED_PERCENT +
            LEFT_TRIM_PERCENT
        );

    /*
     * Velocidade da direita.
     */
    int right_percent =
        limitar_velocidade(
            DRIVE_SPEED_PERCENT +
            RIGHT_TRIM_PERCENT
        );

    /*
     * Converte porcentagem em tempo ligado.
     */
    int left_on_ms =
        (
            DRIVE_PWM_PERIOD_MS *
            left_percent
        ) / 100;

    int right_on_ms =
        (
            DRIVE_PWM_PERIOD_MS *
            right_percent
        ) / 100;

    /*
     * PWM.
     */
    for (
        int t = 0;
        t < DRIVE_PWM_PERIOD_MS;
        t++
    ) {

        bool left_on =
            (t < left_on_ms);

        bool right_on =
            (t < right_on_ms);

        /*
         * As duas rodas para frente.
         */
        ponte_h(
            dev,

            0,
            left_on ? 1 : 0,

            right_on ? 1 : 0,
            0
        );

        k_msleep(1);
    }

    /*
     * Desliga.
     */
    ponte_h(
        dev,
        0,
        0,
        0,
        0
    );
}


/* ==========================================================
 * MAIN
 * ========================================================== */

int main(void)
{
    /*
     * ======================================================
     * LED
     * ======================================================
     */
    if (
        !device_is_ready(
            led.port
        )
    ) {
        return 0;
    }


    /*
     * ======================================================
     * DISPOSITIVOS GPIO
     * ======================================================
     */

    const struct device *gpio_b_dev =
        DEVICE_DT_GET(
            DT_NODELABEL(
                gpiob
            )
        );

    gpio_d_dev =
        DEVICE_DT_GET(
            DT_NODELABEL(
                gpiod
            )
        );

    const struct device *gpio_e_dev =
        DEVICE_DT_GET(
            DT_NODELABEL(
                gpioe
            )
        );


    /*
     * ======================================================
     * VERIFICA DISPOSITIVOS
     * ======================================================
     */

    if (
        !device_is_ready(
            gpio_b_dev
        )
        ||
        !device_is_ready(
            gpio_d_dev
        )
        ||
        !device_is_ready(
            gpio_e_dev
        )
    ) {

        while (1) {

            gpio_pin_toggle_dt(
                &led
            );

            k_msleep(50);
        }
    }


    /*
     * ======================================================
     * LED VERMELHO
     * ======================================================
     */

    gpio_pin_configure_dt(
        &led,
        GPIO_OUTPUT_INACTIVE
    );


    /*
     * ======================================================
     * LED VERDE
     * ======================================================
     */

    gpio_pin_configure(
        gpio_b_dev,
        GREEN_PIN,
        GPIO_OUTPUT_INACTIVE |
        GPIO_ACTIVE_LOW
    );


    /*
     * ======================================================
     * ULTRASSOM
     * ======================================================
     */

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


    /*
     * ======================================================
     * PONTE H
     * ======================================================
     */

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


    /*
     * ======================================================
     * ENCODER HW-201
     * ======================================================
     */

    int ret =
        gpio_pin_configure(
            gpio_d_dev,
            ENC1_PIN,
            GPIO_INPUT |
            GPIO_PULL_UP
        );

    if (ret < 0) {

        printk(
            "ERRO configurando PTD7: %d\n",
            ret
        );

        return 0;
    }


    /*
     * ======================================================
     * INTERRUPÇÃO DO ENCODER
     * ======================================================
     */

    ret =
        gpio_pin_interrupt_configure(
            gpio_d_dev,
            ENC1_PIN,
            GPIO_INT_EDGE_RISING
        );

    if (ret < 0) {

        printk(
            "ERRO configurando interrupcao do encoder: %d\n",
            ret
        );

        return 0;
    }


    /*
     * ======================================================
     * CALLBACK
     * ======================================================
 */

    gpio_init_callback(
        &enc_cb_data,
        encoder_isr,
        BIT(ENC1_PIN)
    );


    /*
     * ======================================================
     * ADICIONA CALLBACK
     * ======================================================
 */

    ret =
        gpio_add_callback(
            gpio_d_dev,
            &enc_cb_data
        );

    if (ret < 0) {

        printk(
            "ERRO adicionando callback: %d\n",
            ret
        );

        return 0;
    }


    /*
     * ======================================================
     * MENSAGEM INICIAL
     * ======================================================
 */

    printk("\n");
    printk("========================================\n");
    printk("          CARRINHO FRDM-KL25Z\n");
    printk("========================================\n");
    printk("Encoder: HW-201\n");
    printk("Entrada: PTD7\n");
    printk("Filtro preto: %d ms\n",
           ENC_LOW_STABLE_MS);
    printk("Intervalo minimo entre pulsos: %d ms\n",
           ENC_MIN_PULSE_MS);
    printk("Pulsos por curva: %d\n",
           TURN_ENCODER_PULSES);
    printk("Tempo minimo da curva: %d ms\n",
           TURN_MIN_TIME_MS);
    printk("Tempo maximo da curva: %d ms\n",
           TURN_MAX_TIME_MS);
    printk("Velocidade reta: %d%%\n",
           DRIVE_SPEED_PERCENT);
    printk(
        "Velocidade curva: %d%%\n",
        (
            TURN_ON_MS * 100
        ) / TURN_PERIOD_MS
    );
    printk("Roda: %d mm de diametro\n",
           WHEEL_DIAMETER_MM);
    printk("Tracos brancos: %d\n",
           ENCODER_TRACKS);
    printk(
        "Distancia por pulso: aproximadamente 2.75 cm\n"
    );
    printk("Obstaculo: %d cm\n",
           OBSTACLE_CM);
    printk("Primeira tentativa: DIREITA\n");
    printk("Segunda tentativa: ESQUERDA\n");
    printk("========================================\n");
    printk("\n");


    /*
     * ======================================================
     * ESTADO INICIAL
     * ======================================================
 */

    encoder_count = 0;
    encoder_reta_count = 0;
    encoder_em_reta = false;
    aguardando_preto = false;
    ultimo_pulso_ms = -1000;

    leds_desligados(
        gpio_b_dev
    );


    /*
     * ======================================================
     * LOOP PRINCIPAL
     * ======================================================
 */

    while (1) {

        /*
         * ==================================================
         * MEDIR DISTÂNCIA
         * ==================================================
         */
        uint32_t distance_cm =
            medir_distancia_cm(
                gpio_e_dev
            );


        /*
         * ==================================================
         * EXISTE OBSTÁCULO?
         * ==================================================
         */

        if (
            distance_cm <
            OBSTACLE_CM
        ) {

            /*
             * Não estamos em linha reta.
             */
            encoder_em_reta = false;


            /*
             * LED vermelho.
             */
            led_vermelho(
                gpio_b_dev
            );


            /*
             * Freia.
             */
            ponte_h(
                gpio_b_dev,
                1,
                1,
                1,
                1
            );


            /*
             * Pausa inicial.
             */
            k_msleep(
                OBSTACLE_INITIAL_PAUSE_MS
            );


            /*
             * =================================================
             * 1. PRIMEIRA TENTATIVA: DIREITA
             * =================================================
             */

            distance_cm =
                executar_curva(
                    gpio_b_dev,
                    gpio_e_dev,
                    TURN_STATE_RIGHT
                );


            /*
             * =================================================
             * 2. DIREITA BLOQUEADA?
             * =================================================
             */

            if (
                distance_cm <
                OBSTACLE_CM
            ) {

                /*
                 * ------------------------------------------------
                 * RETORNA PARA A POSIÇÃO ORIGINAL
                 * ------------------------------------------------
                 *
                 * O retorno usa o movimento oposto da curva
                 * para a direita.
                 */
                distance_cm =
                    executar_curva(
                        gpio_b_dev,
                        gpio_e_dev,
                        TURN_STATE_RETURN
                    );


                /*
                 * ------------------------------------------------
                 * 3. SEGUNDA TENTATIVA: ESQUERDA
                 * ------------------------------------------------
                 */
                distance_cm =
                    executar_curva(
                        gpio_b_dev,
                        gpio_e_dev,
                        TURN_STATE_LEFT
                    );


                /*
                 * =================================================
                 * 4. ESQUERDA TAMBÉM BLOQUEADA?
                 * =================================================
                 */

                if (
                    distance_cm <
                    OBSTACLE_CM
                ) {

                    /*
                     * LED vermelho.
                     */
                    led_vermelho(
                        gpio_b_dev
                    );


                    /*
                     * Para.
                     */
                    ponte_h(
                        gpio_b_dev,
                        1,
                        1,
                        1,
                        1
                    );


                    /*
                     * Espera até ficar livre.
                     */
                    while (
                        medir_distancia_cm(
                            gpio_e_dev
                        )
                        <
                        OBSTACLE_CM
                    ) {

                        encoder_em_reta = false;

                        k_msleep(100);
                    }

                } else {

                    /*
                     * Esquerda livre.
                     */
                    led_verde(
                        gpio_b_dev
                    );
                }
            }

        } else {

            /*
             * =================================================
             * CAMINHO LIVRE
             * =================================================
             */

            led_verde(
                gpio_b_dev
            );


            /*
             * Anda para frente.
             */
            andar_para_frente(
                gpio_b_dev
            );
        }
    }

    return 0;
}
