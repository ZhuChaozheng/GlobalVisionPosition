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
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;  
    addr_serv.sin_addr.s_addr = inet_addr(ip.data());    
    addr_serv.sin_port = htons(DEST_PORT);  
    
    len = sizeof(addr_serv); 
    return sock_fd;  
}
int udp::send_data(const int sock_fd, 
            const char* send_buf) 
{
    int send_num; 
    send_num = sendto(sock_fd, send_buf, 
        6, 0, (struct sockaddr *)&addr_serv, len);  

    if(send_num < 0)  
    {  
        perror("sendto error:");  
        exit(1);  
    }
    close(sock_fd);
    return 1;  
}