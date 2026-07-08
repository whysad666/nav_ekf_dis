//
// Created by lie on 2024/4/16.
//

#ifndef BOOST_SER_SOCKET_SEND_H
#define BOOST_SER_SOCKET_SEND_H

#include <arpa/inet.h>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

class socket_send {
public:
    struct uwb_data {
        uint32_t ver, id, role;
        double range;
    };
    static const size_t BUF_LEN = 1024;

    socket_send() = default;

    ~socket_send() {
        std::cout << "~socket_send" << std::endl;
        close(socketfd);
    }

    auto init_context(const char *dest_ip, uint16_t port) -> int;

    auto send_struct(size_t len, const char *p_buf) -> int;

private:
    struct sockaddr_in serv_addr;
    int socketfd;
    char buf[BUF_LEN];
};

#endif // BOOST_SER_SOCKET_SEND_H
