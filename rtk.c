#include <stdio.h>
#include <threads.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define UDP_PORT 8080

int main(){

    struct sockaddr_in addr;
    socklen_t addrlen = sizeof(addr);
    int opt = 1;
    char udp_buff[1024];

    if(server_fd = socket(AF_INET, SOCK_DGRAM, 0)<0){
        errno("Wrong socket");
    }
    addr = AF_INET;
    addr.sin_port = htons(UDP_PORT);

}