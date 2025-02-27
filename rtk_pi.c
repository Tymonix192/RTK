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

#define UDP_PORT 8000
#define BUFF_SIZE 1024
#define UART_BUFF_SIZE 256
#define UART_PORT "/dev/serial0"

typedef unsigned char u1;
enum {
    bits = 8,
    lShift = 2,
    rShift = bits - lShift
};
#define ROT_LEFT(val) ((val << lShift) | (val >> rShift))

u1 checksum(u1 const* src, int count);
char* skip_message(char* mess, ssize_t lenght);
void* uart_thread(void* arg);
char* parse_message(char *mess);

pthread_mutex_t uart_mutex = PTHREAD_MUTEX_INITIALIZER;

int configure_uart(const char* device, int baudrate);


int main(){
    pthread_t uart_thread;
    
    int udp_rx_sock;
    struct sockaddr_in my_addr = {  .sin_family = AF_INET,
                                    .sin_addr.s_addr = INADDR_ANY,
                                    .sin_port = htons(UDP_PORT)};
    char buffer[BUFF_SIZE];
    char uart_buff[UART_BUFF_SIZE];

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

    ssize_t msg_len;
    int bytes_read1;
    char uart_buff[UART_BUFF_SIZE];
    
    while (1) {

        pthread_create(&uart_thread, NULL, uart_thread, (void*) uart_buff);
        
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
    pthread_join(&uart_thread, (void*) uart_buff);
    pthread_mutex_destroy(&uart_mutex);
}


void* uart_thread(void* arg) {
    int uart_stream = configure_uart(UART_PORT, B115200);
    if (uart_stream == -1) {
        return NULL;
    }
    FILE* uart_log = open("uart_log.txt", "w+");
    char buffer[UART_BUFF_SIZE];
    int mutex_acuired = 0;
    pthread_mutex_init(&uart_mutex, uart_stream);
    while(!mutex_acuired){
        if(pthread_mutex_trylock(&uart_mutex) == 0){
            puts("mutex acquired");
            mutex_acuired = 1;
        }else{
            perror(errno);
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
    return NULL;
}

char *parse_message(char* message){
    int mess_lenght = *(message+2)*100 + *(message+3)*10 + *(message+4);
    int reck = 0;
    u1 check = checksum(message, mess_lenght);
    //check checksum
    u1 calc_checksum = 0;
    for(int i = 0; i<5; i++){
        calc_checksum += *(message+mess_lenght+i);
    }
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
