#ifndef UDP_H
#define UDP_H

#include <stdio.h>   
#include <string.h>   
#include <errno.h>   
#include <stdlib.h>   
#include <unistd.h>   
#include <sys/types.h>   
#include <sys/socket.h>   
#include <netinet/in.h>   
#include <arpa/inet.h> 
#include <string>  
#include <iostream>
  
#define DEST_PORT 10000    

using namespace std;

class udp
{
public:
    int udp_init(const string ip);
    int send_data(const int sock_fd, 
            const char* send_buf);

private:     
    /* socket文件描述符 */
    int sock_fd;
    struct sockaddr_in addr_serv;
    int len;
};
#endif
