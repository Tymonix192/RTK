#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <time.h>
#include <getopt.h>
#include <linux/spi/spidev.h>
#include "rtk_definitions.h"

pthread_mutex_t uart_mutex_1 = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t uart_mutex_2 = PTHREAD_MUTEX_INITIALIZER;
int spi_fd; // File descriptor for SPI device
pthread_mutex_t spi_mutex = PTHREAD_MUTEX_INITIALIZER;

int main(int argc, char* argv[]){
    char *mode = "udp";
    struct option long_options[] = {
        {"mode", required_argument, 0, 'm'},
        {0, 0, 0, 0}
    };
    int opt;
    while ((opt = getopt_long(argc, argv, "", long_options, NULL)) != -1) {
        switch (opt) {
            case 'm':
                mode = optarg;
                break;
            default:
                fprintf(stderr, "Usage: %s [--mode=udp|spi]\n", argv[0]);
                exit(EXIT_FAILURE);
        }
    }
    if (strcmp(mode, "udp") != 0 && strcmp(mode, "spi") != 0) {
        fprintf(stderr, "Invalid mode: %s. Must be 'udp' or 'spi'.\n", mode);
        exit(EXIT_FAILURE);
    }
    
    pthread_t uart_thread;
    
    int udp_rx_sock;
    struct sockaddr_in my_addr = {  .sin_family = AF_INET,
                                    .sin_addr.s_addr = INADDR_ANY,
                                    .sin_port = htons(UDP_PORT)};
    char buffer[BUFF_SIZE];

    if((udp_rx_sock = socket(AF_INET, SOCK_DGRAM, 0))<=0){
        perror("failed to create sock");
        return EXIT_FAILURE;
    }

    int broadcast_enable = 1;
    int opt_result;
    if((setsockopt(udp_rx_sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)))<0){
        perror("Options not set properly");
        return EXIT_FAILURE;
    }

    int result = bind(udp_rx_sock, (struct sockaddr*)&my_addr, sizeof(my_addr));

    int uart_stream = configure_uart("/dev/serial0", B115200);
    if(uart_stream == -1) return EXIT_FAILURE;
    
    mcp2515_init();
    init_spi();

//    int uart_stream2 = configure_uart("dev/ttyAMA1", B9600);
//    if(uart_stream2 == -1) return EXIT_FAILURE;

    FILE* udp_log = fopen("udp_log.txt", "w+");
    if (!udp_log) {
        perror("Failed to open udp_log.txt");
        close(uart_stream);
        close(udp_rx_sock);
        return EXIT_FAILURE;
    }

    int uart_1_mess_rd = 0x0;
    int uart_2_mess_rd = 0x0;
    ssize_t msg_len;
    int bytes_read1;
    char uart_buff[UART_BUFF_SIZE];
    
    FILE* uart_log_1 = open("uart_log_1.txt", "w");
    FILE* uart_log_2 = open("uart_log_2.txt", "w");

    uart_config_t uart_config_1 = {
        .rate = BAUD_RATE,
        .uart_port = UART_PORT,
        .log_file = uart_log_1,
        .uart_mutex = uart_mutex_1
    };
    uart_config_t uart_config_2 = {
        .rate = BAUD_RATE,
        .uart_port = UART_PORT_2,
        .log_file = uart_log_2,
        .uart_mutex = uart_mutex_2
    };
    if((pthread_create(&uart_thread, NULL, uart_thread, (void*) &uart_config_1)) != 0){
        perror("Failed to create UART thread nr1");
        fclose(udp_log);
        close(uart_stream);
        close(udp_rx_sock);
        return EXIT_FAILURE;
    }
    if((pthread_create(&uart_thread, NULL, uart_thread, (void*) &uart_config_2)) != 0){
        perror("Failed to create UART thread nr2");
        fclose(udp_log);
        close(uart_stream);
        close(udp_rx_sock);
        return EXIT_FAILURE;
    }
    
    while (1) {
        ssize_t msg_len = recv(udp_rx_sock, buffer, BUFF_SIZE, 0);

        if (msg_len > 0) {
            buffer[msg_len] = '\0';
            fprintf(udp_log, "%s", buffer);
            printf("saving udp datagram\n");
            
            ssize_t acquired = 0;
            
            while(!acquired){
                if(pthread_mutex_trylock(&uart_mutex_1) == 0){
                    acquired = 1;
                }
            }
            ssize_t resp = write(uart_stream, buffer, msg_len);
            if(resp < 0) perror(errno);
            pthread_mutex_unlock(&uart_mutex_1);
            ssize_t acquired_2 = 0;
            
            while(!acquired_2){
                if(pthread_mutex_trylock(&uart_mutex_2) == 0){
                    acquired_2 = 1;
                }
            }
            ssize_t resp = write(uart_stream, buffer, msg_len);
            if(resp < 0) perror(errno);
            pthread_mutex_unlock(&uart_mutex_1);
        }
    }
    pthread_join(uart_thread, NULL);
    pthread_mutex_destroy(&uart_mutex_1);
    pthread_mutex_destroy(&uart_mutex_2);
}


void* uart_thread(void* arg) {
    uart_config_t *config = (uart_config_t *)arg;

    pthread_mutex_t uart_mutex = config->uart_mutex;
    int uart_stream = configure_uart(config->uart_port, config->rate);
    if (uart_stream == -1) {
        return NULL;
    }
    
    FILE* uart_log = fopen(config->log_file,  "w+");
    char buffer[UART_BUFF_SIZE];
    int mutex_acuired = 0;
    pthread_mutex_init(&uart_mutex, uart_stream);
    while(!mutex_acuired){
        if(pthread_mutex_trylock(&uart_mutex) == 0){
            puts("mutex acquired");
            mutex_acuired = 1;
        }else{
            puts(errno);
        }
    }
    int bytes_curr_buff = 0;
    int got_data = 0x0;
    mess_data_t curr_mess_data = {0,0};
    char* pointer = buffer;
    while(1){
        int bytes_read = read(uart_stream, pointer+bytes_curr_buff, 16);
        
        if (bytes_read > 0) {
            bytes_curr_buff += bytes_read;
            char* message = buffer;
            //check for id
            // if id, check for lenght
            if(got_data == 0x0){
                curr_mess_data = get_mess_data(message);
                got_data = 0x1;
            }
            if(bytes_curr_buff >= curr_mess_data.lenght){
            // pass if curr buff len<len.
            // else do parsing
            //check for id, then get lenght, then pass till mess lenght is not fully here, if not
                buffer[bytes_curr_buff] = '\0';
                int err;
                if((err = parse_message(message, curr_mess_data)) <0){
                    perror("sth went wrong in uart in thread while parsing");
                }
                got_data = 0x0;
                bytes_curr_buff = 0;
            }
        }
    }
    pthread_mutex_unlock(&uart_mutex);
    fclose(uart_log);
    return NULL;
}

int parse_message(char* message, mess_data_t message_type){   
    char* body = message +5;
    if(checksum(body, message_type.lenght) != *(message + message_type.lenght -1)) {
        perror("corrupted message, checksum not right");
        return -1;
    };
    switch (message_type.mess_type)
    {
    case ET:
        ET_data_t data_ET;
        copy_data(&data_ET, body, sizeof(data_ET));
        //send somewhere or sth
        return 0;
    case PG:
        // trying the move approach for the data transfer.
        PG_data_t data_PG;
        pg_data_u* u_ptr;
        u_ptr = message+5;
        message = NULL;
        return 0;
    case VG:
        VG_data_t data_VG;
        copy_data(&data_VG, body, sizeof(data_VG));
        //add some further parsing/sending elswhere
        return 0;
    case AR:
        AR_data_t data_AR;
        copy_data(&data_AR, body, sizeof(data_AR));
        //test if this will work
        return 0;
    default:
        return -1;
    }

}

u1 checksum(u1 const* src, int count)
{
    u1 res = 0;
    while(count--)
    res = ROT_LEFT(res) ^ *src++;
    return ROT_LEFT(res);
}

int configure_uart(const char* device, int baudrate){
    int uart_filestream = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if(uart_filestream == -1){
        perror("Failed to open UART");
        return EXIT_FAILURE;
    }
    struct termios options;
    tcgetattr(uart_filestream, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_lflag = 0;
    options.c_oflag = 0;
    options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tcsetattr(uart_filestream, TCSANOW, &options);

    return uart_filestream;
}

int spi_communication(char* message, ssize_t mess_len){
    int fd;
    int ret;

    char tx_buff [BUFF_SIZE];
    char rx_buff [BUFF_SIZE];

    struct spi_ioc_transfer transfer = {
        .tx_buf = (unsigned long)tx_buff, // Cast buffer to 64-bit pointer
        .rx_buf = (unsigned long)rx_buff,
        .len = sizeof(tx_buff),                           // Transfer 2 bytes
        .speed_hz = SPI_SPEED,              // Use configured speed
        .bits_per_word = SPI_BITS_PER_WORD, // Use configured bits
        .delay_usecs = 0,                   // No delay after transfer
        .cs_change = 0,                     // Deassert CS after transfer
    };
    

    if((fd = open(SPI_PORT, O_WRONLY)) <0){
        perror("SPI device opening error");
        return -1;
    }

    uint8_t bits = SPI_BITS_PER_WORD;
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret < 0) {
        perror("Failed to set bits per word");
        close(fd);
        return -1;
    }

    uint32_t speed = SPI_SPEED;
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret < 0) {
        perror("Failed to set SPI speed");
        close(fd);
        return 1;
    }

    if((ret = write(fd, message, mess_len))<0){
        perror("failed to write");
    }
    close(fd);

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
    if (ret < 0) {
        perror("Failed to perform SPI transfer");
        close(fd);
        return -1;
    }
}

mess_data_t get_mess_data(char* buff){
    int res;
    u1 len_bytes[3];
    memcpy(len_bytes, buff + 2, 3);
    int len = (len_bytes[0] << 16) | (len_bytes[1] << 8) | len_bytes[2];
    int lenght_log = open("mess_lenght_log.txt", "w");
    if(write(lenght_log, &len, sizeof(len))<0){
        perror("sth went wrong writiong to the lenght log file");
    }
    close(lenght_log);
    if(strncmp(buff, "AI", 2) == 0){
        if(len>=26) return (mess_data_t){len, AI};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "PV", 2) == 0){
        if(len>=35) return (mess_data_t){len, PV};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "::", 2) == 0){
        if(len>=11) return (mess_data_t){len, ET};
        return CORRUPTED_MESS
    }else if(strncmp(buff, "PG", 2) == 0){
        if(len>=30+5) return (mess_data_t){len, PG};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "VG", 2) == 0){
        if(len>=18+5) return (mess_data_t){len, VG};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "AR", 2) == 0){
        if(len>=38) return (mess_data_t){len, AR};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "AV", 2) == 0){
        if(len>=27) return (mess_data_t){len, AV};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "MA", 2) == 0){
        if(len>=38+5) return (mess_data_t){len, MA};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "ha", 2) == 0){
        if(len>=15) return (mess_data_t){len, ha};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "MR", 2) == 0){
        if(len>=37+5) return (mess_data_t){len, MR};
        return CORRUPTED_MESS;
    }
    return NOT_MESS_START;
}

char* copy_data(void* dest, char* source, size_t size) {
    memcpy(dest, source, size);
    return source + size;
}

void init_spi(void) {
    spi_fd = open("/dev/spidev0.0", O_RDWR);
    if (spi_fd < 0) {
        perror("Failed to open SPI device");
        exit(EXIT_FAILURE);
    }

    // Set SPI mode (Mode 0: CPOL=0, CPHA=0)
    uint8_t mode = SPI_MODE_0;
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("Failed to set SPI mode");
        close(spi_fd);
        exit(EXIT_FAILURE);
    }

    // Set bits per word
    uint8_t bits = 8;
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("Failed to set SPI bits per word");
        close(spi_fd);
        exit(EXIT_FAILURE);
    }

    // Set SPI speed (1 MHz)
    uint32_t speed = 1000000;
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("Failed to set SPI speed");
        close(spi_fd);
        exit(EXIT_FAILURE);
    }
}

/** Perform an SPI transfer */
int spi_transfer(uint8_t *tx_buf, uint8_t *rx_buf, int len) {
    struct spi_ioc_transfer transfer = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = len,
        .speed_hz = 0, 
        .delay_usecs = 0,
        .bits_per_word = 0, 
        .cs_change = 0,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer) < 0) {
        perror("SPI transfer failed");
        return -1;
    }
    return 0;
}

void mcp2515_reset(void) {
    uint8_t tx_buf[1] = {0xC0}; // Reset command
    uint8_t rx_buf[1];
    spi_transfer(tx_buf, rx_buf, 1);
}

uint8_t mcp2515_read_reg(uint8_t address) {
    uint8_t tx_buf[3] = {0x03, address, 0x00}; // Read command, address, dummy byte
    uint8_t rx_buf[3];
    spi_transfer(tx_buf, rx_buf, 3);
    return rx_buf[2]; // Data is in the third byte
}

void mcp2515_write_reg(uint8_t address, uint8_t value) {
    uint8_t tx_buf[3] = {0x02, address, value}; // Write command, address, data
    uint8_t rx_buf[3];
    spi_transfer(tx_buf, rx_buf, 3);
}

void mcp2515_init(void) {
    mcp2515_reset();
    usleep(10000);

    // Configure for 500 kbps with 8 MHz clock (example settings)
    mcp2515_write_reg(0x2A, 0x00); // CNF1: SJW=1, BRP=0
    mcp2515_write_reg(0x29, 0x90); // CNF2: BTLMODE=1, PHSEG1=2, PRSEG=1
    mcp2515_write_reg(0x28, 0x02); // CNF3: PHSEG2=3

    mcp2515_write_reg(0x0F, 0x00); // CANCTRL: Normal mode
}

int mcp2515_send_message(uint32_t id, uint8_t dlc, uint8_t *data) {
    if (dlc > 8) return -1; // Max 8 bytes

    pthread_mutex_lock(&spi_mutex);

    // Write 11-bit ID to TXB0SIDH and TXB0SIDL
    uint8_t sidh = (id >> 3) & 0xFF;
    uint8_t sidl = (id & 0x07) << 5;
    mcp2515_write_reg(0x31, sidh); // TXB0SIDH
    mcp2515_write_reg(0x32, sidl); // TXB0SIDL

    mcp2515_write_reg(0x35, dlc); // TXB0DLC

    // Write data to TXB0D0-TXB0D7
    for (int i = 0; i < dlc; i++) {
        mcp2515_write_reg(0x36 + i, data[i]);
    }

    uint8_t tx_buf[1] = {0x81};
    uint8_t rx_buf[1];
    spi_transfer(tx_buf, rx_buf, 1);

    pthread_mutex_unlock(&spi_mutex);
    return 0;
}

uint8_t mcp2515_read_rx_status(void) {
    uint8_t tx_buf[2] = {0xA0, 0x00}; 
    uint8_t rx_buf[2];
    spi_transfer(tx_buf, rx_buf, 2); 
    return rx_buf[1]; 
}

int mcp2515_read_message(uint32_t *id, uint8_t *dlc, uint8_t *data) {
    pthread_mutex_lock(&spi_mutex); 

    uint8_t sidh = mcp2515_read_reg(0x61); // RXB0SIDH
    uint8_t sidl = mcp2515_read_reg(0x62); // RXB0SIDL
    *id = ((uint32_t)sidh << 3) | ((sidl >> 5) & 0x07);

    *dlc = mcp2515_read_reg(0x65) & 0x0F; // Lower 4 bits

    for (int i = 0; i < *dlc && i < 8; i++) {
        data[i] = mcp2515_read_reg(0x66 + i);
    }

    pthread_mutex_unlock(&spi_mutex);
    return 0;
}
// char* parse_geo_pos(char* mess){

//     u1* buff = (u1*)mess;

//     u1 chek = checksum(buff, 30);

//     PG_data_t pos;
    
//     memcpy(&pos.lat, buff, sizeof(f8));
//     buff += sizeof(f8);
//     memcpy(&pos.lon, buff, sizeof(f8));
//     buff += sizeof(f8);
//     memcpy(&pos.alt, buff, sizeof(f8));
//     buff += sizeof(f8);
//     memcpy(&pos.pSigma, buff, sizeof(pos.pSigma));
//     buff += sizeof(pos.pSigma);
//     memcpy(&pos.solType, buff, sizeof(pos.solType));
//     buff += sizeof(pos.solType);
//     memcpy(&pos.cheksum, buff, sizeof(pos.cheksum));
//     buff += sizeof(pos.cheksum);

//     if(pos.cheksum != chek){
//         return NULL;
//     }


//     return buff; // pointer to rest of the string for multiple mess parsing
// }