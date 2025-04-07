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
int spi_fd; 
pthread_mutex_t spi_mutex = PTHREAD_MUTEX_INITIALIZER;
u2 year = 0;
u1 month = 0, day = 0;

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
    
    pthread_t uart_thread_1;
    pthread_t uart_thread_2;
    
    int udp_rx_sock;
    struct sockaddr_in my_addr = {  .sin_family = AF_INET,
                                    .sin_addr.s_addr = INADDR_ANY,
                                    .sin_port = htons(UDP_PORT)};
    char buffer[BUFF_SIZE];

    if((udp_rx_sock = socket(AF_INET, SOCK_DGRAM, 0))<=0){
        perror("failed to create sock");
        return EXIT_FAILURE;
    }

    socklen_t broadcast_enable = 1;
    int opt_result;
    if((setsockopt(udp_rx_sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)))<0){
        perror("Options not set properly");
        return EXIT_FAILURE;
    }

    int result = bind(udp_rx_sock, (struct sockaddr*)&my_addr, sizeof(my_addr));

    int uart_stream1 = configure_uart(UART_PORT, BAUD_RATE);
    if(uart_stream1 == -1) return EXIT_FAILURE;
    int uart_stream2 = configure_uart(UART_PORT_2, BAUD_RATE);
    if(uart_stream2 == -1) return EXIT_FAILURE;
    
    //mcp2515_init();
    //init_spi();

//    if(uart_stream2 == -1) return EXIT_FAILURE;

    FILE* udp_log = fopen("udp_log.txt", "a");
    if (!udp_log) {
        perror("Failed to open udp_log.txt");
        close(uart_stream1);
        close(uart_stream2);
        close(udp_rx_sock);
        return EXIT_FAILURE;
    }

    int uart_1_mess_rd = 0x0;
    int uart_2_mess_rd = 0x0;
    ssize_t msg_len;
    int bytes_read;
    char uart_buff[UART_BUFF_SIZE];
    
    FILE* uart_log_1 = fopen("uart_log_1.txt", "a");
    FILE* uart_log_2 = fopen("uart_log_2.txt", "a");

    uart_config_t uart_config_1 = {
        .uart_port = uart_stream1,
        .log_file = uart_log_1,
        .uart_mutex = uart_mutex_1
    };
    uart_config_t uart_config_2 = {
        .uart_port = uart_stream2,
        .log_file = uart_log_2,
        .uart_mutex = uart_mutex_2
    };
    if((pthread_create(&uart_thread_1, NULL, uart_thread, (void*) &uart_config_1)) != 0){
        perror("Failed to create UART thread nr1");
        fclose(udp_log);
        close(uart_stream1);
        close(uart_stream2);
        close(udp_rx_sock);
        return EXIT_FAILURE;
    }
    if((pthread_create(&uart_thread_2, NULL, uart_thread, (void*) &uart_config_2)) != 0){
        perror("Failed to create UART thread nr2");
        fclose(udp_log);
        close(uart_stream1);
        close(uart_stream2);
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
            ssize_t resp = write(uart_stream1, buffer, msg_len);
            if(resp < 0) perror("failed to write to uart stream");
            pthread_mutex_unlock(&uart_mutex_1);
            ssize_t acquired_2 = 0;
            
            while(!acquired_2){
                if(pthread_mutex_trylock(&uart_mutex_2) == 0){
                    acquired_2 = 1;
                }
            }
            resp = write(uart_stream2, buffer, msg_len);
            if(resp < 0) perror("failed to write to second uart");
            pthread_mutex_unlock(&uart_mutex_2);
        }
    }
    pthread_join(uart_thread_1, NULL);
    pthread_join(uart_thread_2, NULL);
    pthread_mutex_destroy(&uart_mutex_1);
    pthread_mutex_destroy(&uart_mutex_2);
}


void* uart_thread(void* arg) {
    uart_config_t *config = (uart_config_t *)arg;

    pthread_mutex_t uart_mutex = config->uart_mutex;
    int uart_stream = config->uart_port;
    FILE* uart_log = config->log_file;
    char buffer[UART_BUFF_SIZE];
    int mutex_acuired = 0;
    int bytes_curr_buff = 0;
    int got_data = 1;
    mess_data_t curr_mess_data = {0,0};

    while(1){
        while(!mutex_acuired){
            if(pthread_mutex_lock(&uart_mutex) == 0){
                mutex_acuired = 1;
            }else{
                perror("error in acquiring uart mutex");
            }
        }
        int bytes_read = read(uart_stream, buffer + bytes_curr_buff, 1);
        if(buffer[bytes_curr_buff] == 0xa){
            bytes_read = 0;
        }
        if (bytes_read > 0) {
            buffer[bytes_curr_buff+1] = '\0';
            printf("byte read: %#x ", buffer[bytes_curr_buff]);
            bytes_curr_buff += bytes_read;
            if(bytes_curr_buff > 5 ){
                curr_mess_data = get_mess_data(buffer);
            }
            if((bytes_curr_buff >= curr_mess_data.lenght+5) && (curr_mess_data.lenght > 1) ){
                buffer[bytes_curr_buff] = '\0';
                int res = parse_message(buffer, curr_mess_data);

                bytes_curr_buff = 0;
                printf("helo");
                pthread_mutex_unlock(&uart_mutex);
            }
        }else{
            pthread_mutex_unlock(&uart_mutex);
            sleep(0.005);
        }
    }
    pthread_mutex_unlock(&uart_mutex);
    fclose(uart_log);
    return NULL;
}

int parse_message(char* message, mess_data_t message_type){   
    char * file_name = "data_log.csv";
    FILE* log_file = fopen(file_name, "a+");
    if(ftell(log_file) == 0){
        fprintf(log_file, "lat,lon,alt,v_lat,v_lon,v_alt,\n");
    }
    printf("before cheksum calc\nmessage in hex:");
    for(int i = 0; i<11;i++)
    {
        printf("%#x", *(message+i));
    }
    printf("\ncheksum in message: %d\n", (uint8_t)*(message+5+message_type.lenght-1));
    // uint8_t cs = checksum(message + 5, message_type.lenght);
    // if(cs != (uint8_t)*(message + message_type.lenght+5-1)) {
    //     errno = -2;
    //     printf("wrong checksum, func got: %d \n", cs);
    //     fflush(stdout);
    //     return -1;
    // };
    switch (message_type.mess_type)
    {
        case ET:
        data_et_u data_ET;
        for(int i = 5+message_type.lenght; i>5; i--)
        {
            data_ET.bytes[i-5] = *(message + i);    
        }
        // memcpy(&data_ET.data.time, (uint8_t*)(message+5), 4);
        // memcpy(&data_ET.data.checksum, (uint8_t*)(message+5)+4, 1);
        uint8_t cs = checksum(message + 5, message_type.lenght+1);
        printf("time: %d, cheksum: %d, calculated cheksum: %d\n", data_ET.data.time, data_ET.data.checksum, cs);
        
        return 0;
    case PG:
        pg_data_u* data_pg = (pg_data_u*)(message+5);
        f8* tx_buff = (f8*)&data_pg->data.lat;  
        mcp2515_send_message(LAT_PG, 8, (uint8_t*)tx_buff);
        fprintf(log_file, "%f,", *tx_buff);
        tx_buff = (f8*)&data_pg->data.lon;  
        mcp2515_send_message(LON_PG, 8, (uint8_t*)tx_buff);
        fprintf(log_file, "%f,", *tx_buff);
        tx_buff = (f8*)&data_pg->data.alt; 
        mcp2515_send_message(ALT_PG, 8, (uint8_t*)tx_buff);
        fprintf(log_file, "%f,", *tx_buff);
        return 0;
    case VG:
        VG_data_t* data_vg = (VG_data_t*)message;
        tx_buff = (f8*)&data_vg->lat_velocity;
        mcp2515_send_message(LAT_LON_VEL_VG, 8, (uint8_t*)tx_buff);
        fprintf(log_file, "%f,%f,",data_vg->lat_velocity, data_vg->lon_velocity);
        f4* buff = (f4*)&data_vg->alt_velocity;
        
        mcp2515_send_message(ALT_VEL_VG, 4, (uint8_t*)tx_buff);
        fprintf(log_file,"%f,\n",data_vg->alt_velocity);
        return 0;
    case AR:
        data_ar_u* data_ar = (data_ar_u*)message;  // Reinterpret body as union
        if (data_ar->data_AR.flags & 0x01) {    // Check if data is valid (bit 0)
            f8* tx_buff = (f8*)&data_ar->data_AR.pitch;  // Point to pitch
            mcp2515_send_message(PITCH_ROLL_AR, 8, (uint8_t*)tx_buff);  // Send pitch and roll (8 bytes)
        }
        return 0;
    case IM:
        data_im_u *data_im = (data_im_u*)message;
        return 0;
    case RD:
        RD_data_t* data_rd = (RD_data_t*)message;
        year = data_rd->year;
        month = data_rd->month;
        day = data_rd->day;
    default:
        return -1;
    }

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

mess_data_t get_mess_data(char* buff){
    int res;
    uint8_t mess_lenght[4];
    memcpy(mess_lenght, buff+2, 3);
    mess_lenght[3] = '\0';
    printf("\nlenght string: %s", mess_lenght);
    int len = atoi(mess_lenght);
    printf(" len: %d", len);
    FILE* lenght_log = fopen("mess_lenght_log.txt", "a");
    if(fprintf(lenght_log, "%d\t", len)<0){
        perror("sth went wrong writiong to the lenght log file");
    }
    fclose(lenght_log);
    if(strncmp(buff, "::", 2) == 0){
        if(len>=5) {
            printf(" in get mess data ");
            return (mess_data_t){len, ET};
        }
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "PG", 2) == 0){
        if(len>=30+5) return (mess_data_t){len, PG};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "VG", 2) == 0){
        if(len>=18) return (mess_data_t){len, VG};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "AR", 2) == 0){
        if(len>=33) return (mess_data_t){len, AR};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "AV", 2) == 0){
        if(len>=22) return (mess_data_t){len, AV};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "MA", 2) == 0){
        if(len>=38) return (mess_data_t){len, MA};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "ha", 2) == 0){
        if(len>=10) return (mess_data_t){len, ha};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "MR", 2) == 0){
        if(len>=37) return (mess_data_t){len, MR};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "IM", 2) == 0){
        if(len>=20) return(mess_data_t){len, IM};
        return CORRUPTED_MESS;
    }else if(strncmp(buff, "RD", 2) == 0){
        if(len>=10) return (mess_data_t){len, RD};
        return CORRUPTED_MESS;
    }
    return NOT_MESS_START;
}

void init_spi(void) {
    spi_fd = open(SPI_PORT, O_RDWR);
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

u1 checksum(u1 const* src, int count)
{
    u1 res = 0;
    while(count--)
    {
        res = ROT_LEFT(res) ^ *src++;
    }
    int cs = ROT_LEFT(res);
    printf("\nchecksum in function: %d\n", cs);
    return cs;
}