#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>

#define UDP_PORT 8000
#define BUFF_SIZE 1024
#define UART_BUFF_SIZE 256
#define UART_PORT "/dev/serial0"
#define UART_PORT_2 "/dev/serial1"
#define BAUD_RATE B115200
#define SPI_PORT "/dev/spidev0.0"
#define SPI_BITS_PER_WORD 8
#define SPI_SPEED 1000000
#define SPI_MODE SPI_MODE_0

typedef uint8_t u1;
typedef char a1;
typedef int8_t i1;
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

enum CAN_ID {
    ID_PG = 0x610,
    PITCH_ROLL_AR = 0x611,
    LAT_PG = 0x612,
    LON_PG = 0x613,
    ALT_PG = 0x614,
    LAT_LON_VEL_VG = 0x615,
    ALT_VEL_VG = 0x616,
};

enum mess_types{
    PV, //cartesian position - not used
    ET, // epoch time - end of frame
    AI, // antenna information -not used for now
    PG, // Geodetic position
    VG, // geodetic velocity
    AR, //rotation angles
    AV, // angular velocities
    MA, //Accelerometer and Magnetometer Measurements
    ha, // Heading and Pitch
    MR, //Heading and Pitch
    IM, //Inartial measurements
    RD, //Reciever date
};

typedef struct 
{
    u4 time;
    u1 checksum;
}ET_data_t;

typedef union{
    ET_data_t data;
    uint8_t bytes[8];
}data_et_u;

typedef struct {
    f4 accelerations[3];
    f4 angular_velocities[3];
    u1 checksum;
}IM_data_t;

typedef union 
{
    IM_data_t data;
    uint8_t bytes[sizeof(IM_data_t)];
}data_im_u;

typedef struct{
    u2 year;
    u1 month;
    u1 day;
    u1 base;
    u1 cs;
}RD_data_t;

typedef struct
{
    f8 lat;
    f8 lon;
    f8 alt;
    f4 pSigma;
    u1 solType;
    u1 cheksum;
}PG_data_t;

typedef struct
{
    f4 lat_velocity;
    f4 lon_velocity;
    f4 alt_velocity;
    f4 vSigma;
    u1 sol_type;
    u1 checksum;
}VG_data_t;

typedef struct
{
    f4 rec_time;
    f4 pitch;
    f4 roll;
    f4 heading;
    f4 pitch_rms;
    f4 roll_rms;
    f4 heading_rms;
    u1 solTypes[3];
    u1 flags;   // flags [bitfield]:
                // 0: 0 - no data available
                // 1 - data are valid
                // 7…1: reserved
    u1 checksum;
}AR_data_t;

typedef union{
    AR_data_t data_AR;
    uint8_t bytes[sizeof(AR_data_t)];
}data_ar_u;

typedef struct
{
    u4 reciever_time;
    f4 angular_vel_x;
    f4 angular_vel_y;
    f4 angular_vel_z;
    f4 angular_velocity_rms;
    u1 flags;   // flags [bitfield]:
                // 0: 0 - no data available
                // 1 - data are valid
                // 7…1: reserved
    u1 checksum;
}AV_data_t;

typedef struct
{
    u4 reciever_time;
    f4 accelerations[3]; //ax, ay, az in cm/s^2 prob not implementing rn cause of wierd position of reciever in the car
    f4 induction[3]; //bx,by,bz
    f4 magnitude; //value of mag field
    f4 temperature; //temp of mag sensor
    u1 calibrated; // 1 calibrated, 0 not calibrated
    u1 checksum;
}MA_data_t;

typedef struct
{
    f4 heading; // Heading of the baseline between the base and the
                // rover receiver [degrees]
    f4 pitch;   // Pitch of the baseline between the base and the
                // rover receiver [degrees]
    u1 solType;
    u1 checksum;
}ha_data_t;

typedef struct
{
    u4 time; //reciever time
    f4 q00, q01, q02, q03; //components of the rotation matrix q
    f4 rms[3]; // estimated accuracy of the 3 baseline vectors
    u1 solType[3]; 
    u1 flag; // 0 – components of matrix Q are invalid, 1 - valid
    u1 checksum;
}MR_data_t;

typedef struct{
    int uart_port;
    FILE* log_file;
    pthread_mutex_t uart_mutex;
}uart_config_t;

typedef union 
{
    PG_data_t data;
    uint8_t bytes[sizeof(PG_data_t)];
}pg_data_u;

typedef struct{
    int lenght;
    int mess_type;
}mess_data_t;

#define NOT_MESS_START (mess_data_t){-1,-1} ;
#define CORRUPTED_MESS (mess_data_t){-1, 0} ;

u1 checksum(u1 const* src, int count);
void* uart_thread(void* arg);
mess_data_t get_mess_data(char *mess);
int configure_uart(const char* device, int baudrate);
int parse_message(char* message, mess_data_t message_type);

void init_spi(void);
void mcp2515_init(void);
int mcp2515_send_message(uint32_t id, uint8_t dlc, uint8_t *data);
int spi_transfer(uint8_t *tx_buf, uint8_t *rx_buf, int len);
void mcp2515_reset(void);
uint8_t mcp2515_read_reg(uint8_t address);
void mcp2515_write_reg(uint8_t address, uint8_t value);

uint8_t mcp2515_read_rx_status(void);
int mcp2515_read_message(uint32_t *id, uint8_t *dlc, uint8_t *data);