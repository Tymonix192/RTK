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
    uart_config_t *config = (uart_config_t *)arg;

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
    int bytes_read = read(uart_stream, buffer, UART_BUFF_SIZE - 1);
    if (bytes_read > 0) {
        //check for id, then get lenght, then pass till mess lenght is not fully here, if not, i will not parse message, no chance
        buffer[bytes_read] = '\0';
        char* message = buffer;
        int err;
        if((err = parse_message) <0){
            perror("sth went wrong in parsing in thread while parsing");
            return NULL;
        }
        printf("message saved");
    }
    pthread_mutex_unlock(&uart_mutex);
    fclose(uart_log);
    return NULL;
}

char* parse_geo_pos(char* mess){

    u1* buff = (u1*)mess;

    u1 chek = checksum(buff, 30);

    geo_pos_t pos;
    
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


    return buff; // pointer to rest of the string for multiple mess parsing
}

int parse_message(char* message){
    a1 id[2] = {*message, *message+1 , "\0"};
    a1 mess_lenght[3];
    a1 len = mess_lenght[0]*16*16 + mess_lenght[1]*16 + mess_lenght[2];
    memcpy(mess_lenght, message+2, 3);
    int reck = 0;
    u1 check = checksum(message, mess_lenght);
    //check checksum
    u1 calc_checksum = 0;
    for(int i = 0; i<4; i++){
        calc_checksum += *(message+len+i);
    } // potentialy a hex
    if(check != calc_checksum){
        return "bad cheksum";
    }

    int mess_type;
    // parse message depending on type. Define mess types or hard code them in
    // if mess lengh>expected message lengh, ignore the additional data, if smaller
    // ignore the message
    //Change the mutex lock to spinlock
    FILE* temp_log = open("/temp_log.txt", "w+");

    geo_pos_t ans;
    switch (mess_type){
        case PV:
            ans.lat = strtod(message+5, &message); 
            message++;
            ans.lon = strtod(message, &message);
            message++;
            ans.alt = strtod(message, &message);
            message++;
            ans.pSigma = strtod(message, &message);
            message++;
            ans.pSigma = atof(message);
            message+= sizeof(f4) + 1;
            ans.solType = *message;
            message++;
            ans.cheksum = *message;

            write(temp_log, &ans, sizeof(geo_pos_t));
        }
    
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

int parse_mess(char* buff){
    int res;
    int len_tab[2];
    strncpy(len_tab, buff+2, 3);
    int len = len_tab[0]*10 + len_tab[1]*10 + len_tab[2];
    if(strncmp(buff, "AI", 2) == 0){
        
    return AI;
    }else if(strncmp(buff, "PV", 2) == 0){
    return PV;
    }else if(strncmp(buff, "::", 2) == 0){
        if(len<11) return -1;
        u4 time;
        for(int i =0; i<5; i++){
            time += *(buff+5+i)*pow(16,4-i);
        }
        printf("mess: AI, reciever time: %f", time);
    return ET;
    }else if(strncmp(buff, "PG", 2) == 0){
        if(len < 35) return -1;
        
        u1 chek = checksum(buff, 30);
        geo_pos_t pos;
    
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
    return PG;
    }else if(strncmp(buff, "VG", 2) == 0){
    return VG;
    }else if(strncmp(buff, "AR", 2) == 0){
    return AR;
    }else if(strncmp(buff, "AV", 2) == 0){
    return AV;
    }else if(strncmp(buff, "MA", 2) == 0){
    return MA;
    }else if(strncmp(buff, "ha", 2) == 0){
    return ha;
    }else if(strncmp(buff, "mr", 2) == 0){
    return mr;
    }else{
        return -1;
    }
}