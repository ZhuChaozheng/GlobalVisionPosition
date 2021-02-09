#include "udp.h"

int udp::udp_init(string ip)
{
    /* Setup udp socket */ 
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);  
    if(sock_fd < 0)  
    {  
        perror("socket");  
        exit(1);  
    }  
    /* Assign address */  
    memset(&addr_serv_, 0, sizeof(addr_serv_));
    addr_serv_.sin_family = AF_INET;  
    addr_serv_.sin_addr.s_addr = inet_addr(ip.data());    
    addr_serv_.sin_port = htons(DEST_PORT);  
    
    len_ = sizeof(addr_serv_); 
    return sock_fd;  
}
int udp::send_data(const int sock_fd, 
            const char* send_buf) 
{
    int send_num; 
    send_num = sendto(sock_fd, send_buf, 
        6, 0, (struct sockaddr *)&addr_serv_, len_);  

    if(send_num < 0)  
    {  
        printf("sss\n");
        perror("sendto error:");  
        exit(1);  
    }
    close(sock_fd);
    return 1;  
}