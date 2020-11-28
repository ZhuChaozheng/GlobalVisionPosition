#ifndef COMM_TEST_H
#define COMM_TEST_H

#include <stdio.h>   
#include <string.h>   
#include <errno.h>   
#include <stdlib.h>   
#include <unistd.h>   
#include <sys/types.h>   
#include <sys/socket.h>   
#include <netinet/in.h>   
#include <arpa/inet.h>   
   
  
#define DEST_PORT 10000  
#define DSET_IP_ADDRESS  "192.168.43.112"   

class udp
{
    public:
        udp(void)
        {
            /* Setup udp socket */  
            sock_fd = socket(AF_INET, SOCK_DGRAM, 0);  
            if(sock_fd < 0)  
            {  
                perror("socket");  
                exit(1);  
            }  
    
            /* Assign address */  
            memset(&addr_serv, 0, sizeof(addr_serv));  
            addr_serv.sin_family = AF_INET;  
            addr_serv.sin_addr.s_addr = inet_addr(DSET_IP_ADDRESS);  
            addr_serv.sin_port = htons(DEST_PORT);  
            len = sizeof(addr_serv);
            
        }
        int send_data(const char* send_buf)
        {
            int send_num; 
            //char send_buf[20] = "hey, who are you?";    
      
            //printf("client send: %s\n", send_buf);  
    
            send_num = sendto(sock_fd, send_buf, strlen(send_buf), 0, (struct sockaddr *)&addr_serv, len);  
    
            if(send_num < 0)  
            {  
                perror("sendto error:");  
                exit(1);  
            }
            return 1;  
        }
        void deinit()
        {
            close(sock_fd);
        }
    private:
        /* socket文件描述符 */ 
        int sock_fd;
        struct sockaddr_in addr_serv;
        int len;
};

#endif
