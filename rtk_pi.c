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
#include <linux/spi/spidev.h>
#include "rtk_definitions.h"

// add CAN through spi for sbRIO comms

pthread_mutex_t uart_mutex = PTHREAD_MUTEX_INITIALIZER;

int configure_uart(const char* device, int baudrate);


int main(){
    pthread_t *uart_thread;
    
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
//    int uart_stream2 = configure_uart("dev/ttyAMA1", B9600);
//    if(uart_stream2 == -1) return EXIT_FAILURE;

    FILE* udp_log = fopen("udp_log.txt", "w+");
    if (!udp_log) {
        perror("Failed to open udp_log.txt");
        close(uart_stream);
        close(udp_rx_sock);
        return EXIT_FAILURE;
    }

    ssize_t msg_len;
    int bytes_read1;
    char uart_buff[UART_BUFF_SIZE];
    
    if((pthread_create(&uart_thread, NULL, uart_thread, (void*) uart_buff)) != 0){
        perror("Failed to create UART thread");
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
                if(pthread_mutex_trylock(&uart_mutex) == 0){
                    acquired = 1;
                }
            }
            ssize_t resp = write(uart_stream, buffer, sizeof(buffer));
            if(resp != 0) perror(errno);
            pthread_mutex_unlock(&uart_mutex);
            
        
        }
    }
    pthread_join(uart_thread, NULL);
    pthread_mutex_destroy(&uart_mutex);
}


void* uart_thread(void* arg) {
    int uart_stream = configure_uart(UART_PORT, B115200);
    if (uart_stream == -1) {
        return NULL;
    }
    FILE* uart_log = fopen("uart_log.txt", "w+");
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
    int bytes_read = read(uart_stream, buffer, UART_BUFF_SIZE - 1);
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        if (uart_log) {
            fprintf(uart_log, "%s", buffer);
            fflush(uart_log);
        }
        
        printf("saving uart readings\n");
    }
    pthread_mutex_unlock(&uart_mutex);
    fclose(uart_log);
    return NULL;
}

char* parse_geo_pos(char* mess){

    u1* buff = (u1*)mess;

    u1 chek = checksum(buff, 30);

    geo_pos pos;
    
    memcpy(&pos.lat, buff, sizeof(f8));
    buff += sizeof(f8);
    memcpy(&pos.lon, buff, sizeof(f8));
    buff += sizeof(f8);
    memcpy(&pos.alt, buff, sizeof(f8));
    buff += sizeof(f8);
    memcpy(&pos.pSigma, buff, sizeof(pos.pSigma));
    buff += sizeof(pos.pSigma);
    memcpy(&pos.solType, buff, sizeof(pos.solType));
    buff += sizeof(pos.solType);
    memcpy(&pos.cheksum, buff, sizeof(pos.cheksum));
    buff += sizeof(pos.cheksum);

    if(pos.cheksum != chek){
        return NULL;
    }

    printf("lat: %08x, lon: %08x, alt: %08x, pSig: %04x, solType: %01x", pos.lat, pos.lon, pos.alt, pos.pSigma, pos.solType);

    return buff; // pointer to rest of the string for multiple mess parsing
}

char *parse_message(char* message){
    a1 id[2] = {*message, *message+1 , "\0"};
    int mess_lenght = *(message+2)*100 + *(message+3)*10 + *(message+4);
    int reck = 0;
    u1 check = checksum(message, mess_lenght);
    //check checksum
    u1 calc_checksum = 0;
    for(int i = 0; i<4; i++){
        calc_checksum += *(message+mess_lenght+i);
    } // potentialy a hex
    if(check != calc_checksum){
        return "bad cheksum";
    }


    // parse message depending on type. Define mess types or hard code them in
    // if mess lengh>expected message lengh, ignore the additional data, if smaller
    // ignore the message
    //Change the mutex lock to spinlock


    
}

u1 checksum(u1 const* src, int count)
{
    u1 res = 0;
    while(count--)
    res = ROT_LEFT(res) ^ *src++;
    return ROT_LEFT(res);
}

char *skip_message(char* message, ssize_t mess_lenght){
    
}
// create a function that would skipp incoming message, return a new pointer to function parse message

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

void send_spi(char* message, ssize_t mess_len){
    int fd;
    int ret;

    struct spi_ioc_transfer transfer ={
        
    };
    

    if((fd = open(SPI_PORT, O_WRONLY)) <0){
        perror("SPI device opening error");
        return NULL;
    }

    uint8_t bits = SPI_BITS_PER_WORD;
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret < 0) {
        perror("Failed to set bits per word");
        close(fd);
        return 1;
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
}

char* recieve_spi(){
    char buff[BUFF_SIZE];
    int fd;
    int rec;

    if((fd = open(SPI_PORT, O_RDONLY)) <0 ){
        perror("failed opening spi port\t");
        return NULL;
    }
    if((rec = read(fd, buff, BUFF_SIZE))<0){
        perror("failed reading");
    }
    return buff;
}