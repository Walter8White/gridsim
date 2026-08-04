// Small command helper for LJ-S Ethernet control.
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "LJS8_ErrorCode.h"
#include "LJS8_IF_Linux.h"

static int open_device(int deviceId) {
    LJS8IF_ETHERNET_CONFIG ethernetConfig = {
        {192, 168, 0, 1},
        24691
    };
    return LJS8IF_EthernetOpen(deviceId, &ethernetConfig);
}

static int raw_single_command(unsigned char commandCode) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(24691);
    addr.sin_addr.s_addr = inet_addr("192.168.0.1");

    if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("connect");
        close(sock);
        return 1;
    }

    unsigned char packet[20] = {};
    int tcpLength = 16;
    int bodyLength = 4;
    memcpy(&packet[0], &tcpLength, sizeof(tcpLength));
    packet[4] = 0x05;
    packet[5] = 0x00;
    packet[6] = 0xF0;
    packet[7] = 0x01;
    memcpy(&packet[12], &bodyLength, sizeof(bodyLength));
    packet[16] = commandCode;

    if (send(sock, packet, sizeof(packet), 0) != static_cast<ssize_t>(sizeof(packet))) {
        perror("send");
        close(sock);
        return 1;
    }

    unsigned char reply[256] = {};
    ssize_t n = recv(sock, reply, sizeof(reply), 0);
    if (n < 18) {
        printf("raw 0x%02x: short/no reply (%zd bytes)\n", commandCode, n);
        close(sock);
        return 1;
    }

    printf("raw 0x%02x: body_return=0x%02x controller_status=0x%02x bytes=%zd\n",
           commandCode, reply[17], reply[18], n);
    close(sock);
    return reply[17] == 0 ? 0 : 2;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("usage: %s laser_on|laser_off|clear_memory|trigger|raw <hex_command>\n", argv[0]);
        return 2;
    }

    if (strcmp(argv[1], "raw") == 0) {
        if (argc != 3) {
            printf("usage: %s raw <hex_command>\n", argv[0]);
            return 2;
        }
        unsigned long value = strtoul(argv[2], NULL, 0);
        if (value > 0xff) {
            printf("raw command must fit in one byte\n");
            return 2;
        }
        return raw_single_command(static_cast<unsigned char>(value));
    }

    int deviceId = 0;
    int rc = open_device(deviceId);
    printf("EthernetOpen: 0x%x\n", rc);
    if (rc != LJS8IF_RC_OK) {
        return 1;
    }

    if (strcmp(argv[1], "laser_on") == 0) {
        rc = LJS8IF_ControlLaser(deviceId, 1);
    } else if (strcmp(argv[1], "laser_off") == 0) {
        rc = LJS8IF_ControlLaser(deviceId, 0);
    } else if (strcmp(argv[1], "clear_memory") == 0) {
        rc = LJS8IF_ClearMemory(deviceId);
    } else if (strcmp(argv[1], "trigger") == 0) {
        rc = LJS8IF_Trigger(deviceId);
    } else {
        printf("unknown command: %s\n", argv[1]);
        LJS8IF_CommunicationClose(deviceId);
        return 2;
    }

    printf("%s: 0x%x\n", argv[1], rc);
    LJS8IF_CommunicationClose(deviceId);
    return rc == LJS8IF_RC_OK ? 0 : 1;
}
