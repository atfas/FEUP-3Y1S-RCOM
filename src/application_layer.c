#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

void unpackData(unsigned char* packet, int size, unsigned char* data) {
    int L2 = packet[1], L1 = packet[2];
    int payload = L2*256 + L1;
    memcpy(data, packet +3, payload);
    for (int i = 0; i < payload; i++) data[i] = packet[i+3];
    return;
}

unsigned char* unpackControl(unsigned char* packet, int size, int* fileSize) {
    unsigned char fileSizeLen = packet[2];
    unsigned char tmpFileSize[fileSizeLen];
    memcpy(tmpFileSize, packet +3, fileSizeLen);
    for (int i = 0; i < fileSizeLen; i++)
        *fileSize |= ((unsigned long int) tmpFileSize[i]) << (8*(fileSizeLen -i -1));

    unsigned char fileNameLen = packet[fileSizeLen+4];
    unsigned char *fileName = (unsigned char*)malloc(fileNameLen);
    fileName[fileNameLen] = '\0';
    memcpy(fileName, packet +3 + fileSizeLen +2, fileNameLen);

    return fileName;
}

unsigned char* createData(unsigned char* data, int payloadSize, int* packetSize) {
    *packetSize = payloadSize +3;
    unsigned char* packet = (unsigned char*)malloc(*packetSize);

    int i = 0;
    packet[i] = CTRL_DATA; i++;
    packet[i] = (payloadSize >> 8) & 0xFF; i++;
    packet[i] = payloadSize & 0xFF; i++;
    memcpy(&packet[i], data, payloadSize);

    return packet;
}

unsigned char *createControl(long int fileSize, const char *fileName, int control, int* packetSize) {
    int nameLen = strlen(fileName);
    int fileSizeLen = 0;
    long int tmpFileSize = fileSize;
    while (tmpFileSize > 0) {
        fileSizeLen++;
        tmpFileSize >>= 8;
    }

    *packetSize = fileSizeLen +3 + nameLen + 2;
    unsigned char* packet = (unsigned char*)malloc(*packetSize);
    int i = 0;
    packet[i] = control; i++;
    packet[i] = TYPE_FILESIZE; i++;
    packet[i] = fileSizeLen; i++;
    for (int j = fileSizeLen; j > 0; j--) {
        packet[i] = (fileSize >> ((j-1)*8)) & 0xFF;
        i++;
    }
    packet[i] = TYPE_FILENAME; i++;
    packet[i] = nameLen; i++;
    memcpy(packet+i, fileName, nameLen);

    return packet;
}

unsigned char *getData(FILE *fd, unsigned int fileSize) {
    unsigned char* data = (unsigned char*)malloc(sizeof(unsigned char) *fileSize);
    fread(data, sizeof(unsigned char), fileSize, fd);
    return data;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename) {
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.role = (strcmp(role, "tx") == 0) ? LlTx : LlRx;
    connectionParameters.baudRate = baudRate;
    connectionParameters.timeout = timeout;
    connectionParameters.nRetransmissions = nTries;

    int fd = llopen(connectionParameters);
    if (fd < 0) {
        printf("llopen failed\n");
        return;
    }
    printf("llopen successful\n");

    switch (connectionParameters.role) {
        case LlTx: {
            FILE *file = fopen(filename, "rb");
            if (!file) {
                perror("File not open");
                exit(1);
            }

            long int size = 0;
            while (fgetc(file) != EOF) {
                size++;
            }
            rewind(file);

            int packetSize;
            unsigned char *startPacket = createControl(size, filename, CTRL_START, &packetSize);

            if (llwrite(startPacket, packetSize) == -1) {
                printf("Start packet error\n");
                exit(1);
            }

            unsigned char* data = getData(file, size);
            int rem = size;
            int dataPacketSize;
            printf("Starting to extract %ld bytes\n", size);

            while (rem != 0) {
                int payloadSize = rem > MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : rem;
                unsigned char* chunk = (unsigned char*)malloc(payloadSize);
                memcpy(chunk, data, payloadSize);
                unsigned char *dataPacket = createData(chunk, payloadSize, &dataPacketSize);

                if (llwrite(dataPacket, dataPacketSize) == -1) {
                    printf("Error writing\n");
                    exit(1);
                }
                rem -= payloadSize;
                data += payloadSize;
            }
            unsigned char *endPacket = createControl(size, filename, CTRL_END, &packetSize);

            if (llwrite(endPacket, packetSize) == -1) {
                printf("End packet error\n");
                exit(1);
            }
            break;
        }
        case LlRx: {
            unsigned char *startPacket = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
            int packetSize = llread(startPacket);

            int fileSize = 0;
            unpackControl(startPacket, packetSize, &fileSize);
            printf("Start packet received\n");

            FILE *newFile = fopen("penguin-received.gif", "w");
            unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
            while (1) {
                while ((packetSize = llread(packet)) < 0);
                unsigned char control = packet[0];

                if (control == CTRL_END) {
                    printf("End packet received\n");
                    break;
                }
                else if (control == CTRL_DATA) {
                    unsigned char *data = (unsigned char *)malloc(packetSize -3);
                    unpackData(packet, packetSize, data);
                    fwrite(data, sizeof(unsigned char), packetSize -3, newFile);
                    free(data);
                }
            }
            fclose(newFile);
            break;
        }
        default:
            break;
    }
    llclose();
    printf("Closed\n");
}