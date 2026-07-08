//
// Created by lie on 2024/4/16.
//

#include "socket_send.h"

auto socket_send::init_context(const char *dest_ip, uint16_t port) -> int {
    if ((socketfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        std::cout << "error to open socket" << std::endl;
        return -1;
    }
    memset(&serv_addr, 0, sizeof(serv_addr));

    serv_addr.sin_addr.s_addr = inet_addr(dest_ip);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    struct sockaddr_in cli_addr;
    cli_addr.sin_addr.s_addr = inet_addr("172.16.6.100");
    cli_addr.sin_family = AF_INET;
    cli_addr.sin_port = htons(port + 2000);

    if (bind(socketfd, (struct sockaddr *) &cli_addr, sizeof(struct sockaddr)) < 0) {
        std::cout << "bind error" << std::endl;
        return -1;
    }

    return 0;
}

auto socket_send::send_struct(size_t len, const char *p_buf) -> int {
    socklen_t addrlen = sizeof(serv_addr);

    int len_send = sendto(socketfd, p_buf, len, 0, (struct sockaddr *) &serv_addr, addrlen);
    if (len_send == -1) {
        std::cout << "send error" << std::endl;
        return -1;
    }

    return len_send;
}
