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


void udp::handle_udp_msg(int fd)
{
    char buf[BUFF_LEN];  //buff 1024byte
    socklen_t len;
    int count;
    struct sockaddr_in clent_addr;  //clent_addr, record the sender address
    while(1)
    {
        memset(buf, 0, BUFF_LEN);
        len = sizeof(clent_addr);
        //recvfrom is congestion function, until the data coming
        count = recvfrom(fd, buf, BUFF_LEN, 0, 
                (struct sockaddr*)&clent_addr, &len);  
        if(count == -1)
        {
            printf("recieve data fail!\n");
            return;
        }
        printf("client:%s\n",buf);  
        memset(buf, 0, BUFF_LEN);
        sprintf(buf, "I have recieved %d bytes data!\n", count);  // reply client
        printf("server:%s\n",buf);  
        sendto(fd, buf, BUFF_LEN, 0, (struct sockaddr*)&clent_addr, len);  // send message to client
    }
}


void udp::udp_server_init()
{
    int server_fd, ret;
    struct sockaddr_in ser_addr;

    server_fd = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET:IPV4;SOCK_DGRAM:UDP
    if(server_fd < 0)
    {
        printf("create socket fail!\n");
        return;
    }

    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = htonl(INADDR_ANY); //IP
    ser_addr.sin_port = htons(SERVER_PORT);  //port

    ret = bind(server_fd, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
    if(ret < 0)
    {
        printf("socket bind fail!\n");
        return;
    }

    handle_udp_msg(server_fd);   // handle message

    close(server_fd);
}