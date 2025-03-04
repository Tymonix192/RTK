#include <stdint.h>

#define UDP_PORT 8000
#define BUFF_SIZE 1024
#define UART_BUFF_SIZE 256
#define UART_PORT "/dev/serial0"

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

enum {
    bits = 8,
    lShift = 2,
    rShift = bits - lShift
};

#define ROT_LEFT(val) ((val << lShift) | (val >> rShift))