/*

IFMA - Instituto Federal do Maranhão 
Disciplina: Sistemas Embarcados / Arquitetura de Microcontroladores
Aluno(a): Valmir Linhares de Sousa de Mesquita
Matrícula:  20251RSE.MTC0100
Enunciado: Realizar práticas utilizando aplicação MQTT    
Data: 15 de Junho de 2025  

*/


/**
 * @file main.c
 * @brief Núcleo 0 - Controle principal do sistema embarcado com Raspberry Pi Pico W.
 *
 * Este código é executado no núcleo 0 do RP2040 e desempenha as seguintes funções:
 * - Inicialização da interface OLED para exibição de mensagens ao usuário;
 * - Inicialização do PWM para controle de um LED RGB;
 * - Comunicação com o núcleo 1 por meio de FIFO para receber mensagens relacionadas à conexão Wi-Fi;
 * - Exibição e tratamento das mensagens de status do Wi-Fi;
 * - Inicialização do cliente MQTT após o recebimento do IP válido;
 * - Envio periódico da mensagem "PING" via MQTT;
 * - Exibição da confirmação da publicação MQTT recebida do núcleo 1.
 */

#include "fila_circular.h"
#include "rgb_pwm_control.h"
#include "configura_geral.h"
#include "oled_utils.h"
#include "ssd1306_i2c.h"
#include "mqtt_lwip.h"
#include "lwip/ip_addr.h"
#include "pico/multicore.h"
#include <stdio.h>
#include "estado_mqtt.h"



// Define os pinos PWM conectados ao LED RGB (ajuste conforme seu hardware)
#define PIN_RED    13
#define PIN_GREEN  11
#define PIN_BLUE   12

// Inicializa o gerador de números aleatórios com uma semente baseada no tempo
void inicializar_aleatorio() {
    absolute_time_t now = get_absolute_time();
    uint64_t micros = to_us_since_boot(now);
    srand((unsigned int)(micros & 0xFFFFFFFF));
}

// Gera um número aleatório no intervalo [min, max]
int numero_aleatorio(int min, int max) {
    return min + rand() % (max - min + 1);
}

// Configura o pino como saída PWM e retorna o slice
uint pwm_configura_canal(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_enabled(slice, true);
    pwm_set_wrap(slice, 255); // brilho de 0 a 255
    pwm_set_clkdiv(slice, 4.0f); // frequência PWM mais visível
    return slice;
}

// Define a cor do LED RGB (valores de 0 a 255 para cada cor)
void definir_cor_rgb(uint8_t r, uint8_t g, uint8_t b) {
    pwm_set_gpio_level(PIN_RED,   255 - r); // Inversão para LED comum (cátodo comum)
    pwm_set_gpio_level(PIN_GREEN, 255 - g);
    pwm_set_gpio_level(PIN_BLUE,  255 - b);
}
#define INTERVALO_PING_MS 5000  // Intervalo entre envios de "PING" (modificável)

extern void funcao_wifi_nucleo1(void);
extern void espera_usb();
extern void tratar_ip_binario(uint32_t ip_bin);
extern void tratar_mensagem(MensagemWiFi msg);
void inicia_hardware();
void inicia_core1();
void verificar_fifo(void);
void tratar_fila(void);
void inicializar_mqtt_se_preciso(void);
void enviar_ping_periodico(void);

FilaCircular fila_wifi;
absolute_time_t proximo_envio;

    char mensagem_str[50];
    bool ip_recebido = false;

int main() {
    inicia_hardware();
    inicia_core1();

    // Inicializa pinos PWM para cada cor
    pwm_configura_canal(PIN_RED);
    pwm_configura_canal(PIN_GREEN);
    pwm_configura_canal(PIN_BLUE);

    // Inicializa o gerador de números aleatórios
    inicializar_aleatorio();
    while (true) {

        // Gera valores RGB aleatórios entre 0 e 255
        uint8_t r = numero_aleatorio(0, 255);
        uint8_t g = numero_aleatorio(0, 255);
        uint8_t b = numero_aleatorio(0, 255);

        // Define a nova cor no LED
        definir_cor_rgb(r, g, b);

        // Mostra no terminal a cor atual
        printf("Cor RGB: R=%d, G=%d, B=%d\n", r, g, b);

        sleep_ms(1000); // Muda a cor a cada segundo

        bool mensagem_processada = false;

        verificar_fifo();
        tratar_fila();
        inicializar_mqtt_se_preciso();
        enviar_ping_periodico();
        sleep_ms(50);
    }

    return 0;
}
/*******************************************************************/
void verificar_fifo(void) {
    bool mensagem_processada = false;

    if (!multicore_fifo_rvalid()) return;

    uint32_t pacote = multicore_fifo_pop_blocking();
    uint16_t tentativa = pacote >> 16;

    if (tentativa == 0xFFFE) {
        uint32_t ip_bin = multicore_fifo_pop_blocking();
        tratar_ip_binario(ip_bin);
        ip_recebido = true;
        return;
    }

    uint16_t status = pacote & 0xFFFF;

    if (status > 2 && tentativa != 0x9999) {
        snprintf(mensagem_str, sizeof(mensagem_str),
                 "Status inválido: %u (tentativa %u)", status, tentativa);
        ssd1306_draw_utf8_multiline(buffer_oled, 0, 0, "Status inválido.");
        render_on_display(buffer_oled, &area);
        sleep_ms(3000);
        oled_clear(buffer_oled, &area);
        render_on_display(buffer_oled, &area);
        printf("%s\n", mensagem_str);
        return;
    }

    MensagemWiFi msg = {.tentativa = tentativa, .status = status};
    if (!fila_inserir(&fila_wifi, msg)) {
        ssd1306_draw_utf8_multiline(buffer_oled, 0, 0, "Fila cheia. Descartado.");
        render_on_display(buffer_oled, &area);
        sleep_ms(3000);
        oled_clear(buffer_oled, &area);
        render_on_display(buffer_oled, &area);
        printf("Fila cheia. Mensagem descartada.\n");
    }
}

void tratar_fila(void) {
    MensagemWiFi msg_recebida;
    if (fila_remover(&fila_wifi, &msg_recebida)) {
        tratar_mensagem(msg_recebida);
    }
}

void inicializar_mqtt_se_preciso(void) {
    if (!mqtt_iniciado && ultimo_ip_bin != 0) {
        printf("[MQTT] Iniciando cliente MQTT...\n");
        iniciar_mqtt_cliente();
        mqtt_iniciado = true;
        proximo_envio = make_timeout_time_ms(INTERVALO_PING_MS);
    }
}

void enviar_ping_periodico(void) {
    if (mqtt_iniciado && absolute_time_diff_us(get_absolute_time(), proximo_envio) <= 0) {
        publicar_mensagem_mqtt("PING");
        ssd1306_draw_utf8_multiline(buffer_oled, 0, 0, "PING enviado...");
        render_on_display(buffer_oled, &area);
        proximo_envio = make_timeout_time_ms(INTERVALO_PING_MS);
    }
}

/************/
void inicia_hardware(){
    stdio_init_all();
    setup_init_oled();
    espera_usb();
    oled_clear(buffer_oled, &area);
    render_on_display(buffer_oled, &area);
}

void inicia_core1(){
// Mensagem de inicialização
    ssd1306_draw_utf8_multiline(buffer_oled, 0, 0, "Núcleo 0");
    ssd1306_draw_utf8_multiline(buffer_oled, 0, 16, "Iniciando!");
    render_on_display(buffer_oled, &area);
    sleep_ms(3000);
    oled_clear(buffer_oled, &area);
    render_on_display(buffer_oled, &area);

    printf(">> Núcleo 0 iniciado. Aguardando mensagens do núcleo 1...\n");

    init_rgb_pwm();
    fila_inicializar(&fila_wifi);
    multicore_launch_core1(funcao_wifi_nucleo1);
}