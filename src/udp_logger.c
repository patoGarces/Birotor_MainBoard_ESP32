#include "udp_logger.h"
#include "lwip/sockets.h"
#include "esp_log.h"

static vprintf_like_t original_vprintf = NULL;

static int udp_socket = -1;
static struct sockaddr_in dest_addr;

int udpVprintf(const char *fmt, va_list args) {
    // --- 1) Mandar por UART0 ---
    // clonamos la lista porque va_list se consume al usarlo
    va_list args_copy;
    va_copy(args_copy, args);

    int len_uart = 0;
    if (original_vprintf) {
        len_uart = original_vprintf(fmt, args_copy);
    }
    va_end(args_copy);

    // --- 2) Mandar por UDP ---
    char buffer[256];
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    if (len > 0 && udp_socket >= 0) {
        sendto(udp_socket, buffer, len, 0,
               (struct sockaddr *)&dest_addr,
               sizeof(dest_addr));
    }

    return len_uart;   // ESP-IDF usa el valor de UART
}

void udpLoggerInit(uint16_t port) {
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    int broadcast = 1;
    setsockopt(udp_socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    dest_addr.sin_addr.s_addr = inet_addr("255.255.255.255");

    // Guardo el vprintf original
    original_vprintf = esp_log_set_vprintf(NULL);   // (esto solo lo lee, no lo desactiva)

    // Registro el vprintf compuesto
    esp_log_set_vprintf(udpVprintf);
}