#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>

#define UDP_PORT 8000
#define BUFF_SIZE 1024
#define UART_BUFF_SIZE 256
#define UART_PORT "/dev/serial0"
#define UART_PORT_2 "/dev/serial1"
#define SPI_PORT "/dev/spidev0.0"
#define SPI_BITS_PER_WORD 8
#define SPI_SPEED 1000000
#define SPI_MODE SPI_MODE_0

typedef unsigned char u1;
typedef char a1;
typedef char i1;
typedef int16_t i2;
typedef uint16_t u2;
typedef int i4;
typedef unsigned int u4;
typedef int64_t i8;
typedef uint64_t u8;
typedef float f4; 
typedef double f8;

enum mess_types{
    PV,
    ET,
    AI,
    PG,
    VG,
    AR,
    AV,
    MA,
    ha,
    mr,
};

enum {
    bits = 8,
    lShift = 2,
    rShift = bits - lShift
};

#define ROT_LEFT(val) ((val << lShift) | (val >> rShift))

typedef struct{
    f8 lat;
    f8 lon;
    f8 alt;
    f4 pSigma;
    u1 solType;
    u1 cheksum;
}geo_pos_t;

typedef struct{
    char* uart_port;
    int rate;
    char* log_file;
}uart_config_t;

u1 checksum(u1 const* src, int count);
char* skip_message(char* mess, i4 lenght);
void* uart_thread(void* arg);
int parse_mess(char *mess);
char* parse_geo_pos(char* mess);
int spi_communication(char* mess, ssize_t len);
char* recieve_spi();
int send_data_uart(geo_pos_t data);
int send_data_uart(geo_pos_t data);
int send_data_uart(geo_pos_t data);