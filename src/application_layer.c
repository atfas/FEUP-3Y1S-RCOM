// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/**
 * Builds a control packet (START or END)
 * Returns the size of the packet, or -1 on error
 */
int buildControlPacket(unsigned char *packet, unsigned char controlField, 
                       const char *filename, long fileSize) {
    int idx = 0;
    
    // Control field (START or END)
    packet[idx++] = controlField;
    
    // TLV for file size
    packet[idx++] = TLV_FILE_SIZE;
    
    // Calculate number of bytes needed for file size
    unsigned char sizeBytes[sizeof(long)];
    int numSizeBytes = 0;
    long tempSize = fileSize;
    
    do {
        sizeBytes[numSizeBytes++] = tempSize & 0xFF;
        tempSize >>= 8;
    } while (tempSize > 0);
    
    packet[idx++] = numSizeBytes;
    
    // Write size bytes in big-endian order
    for (int i = numSizeBytes - 1; i >= 0; i--) {
        packet[idx++] = sizeBytes[i];
    }
    
    // TLV for filename
    packet[idx++] = TLV_FILE_NAME;
    int nameLen = strlen(filename);
    packet[idx++] = nameLen;
    memcpy(&packet[idx], filename, nameLen);
    idx += nameLen;
    
    return idx;
}

/**
 * Parses a control packet
 * Returns 0 on success, -1 on error
 */
int parseControlPacket(const unsigned char *packet, int packetSize,
                       char *filename, long *fileSize) {
    if (packetSize < 5) return -1;
    
    int idx = 1; // Skip control field
    
    *fileSize = 0;
    filename[0] = '\0';
    
    while (idx < packetSize) {
        unsigned char type = packet[idx++];
        unsigned char length = packet[idx++];
        
        if (idx + length > packetSize) return -1;
        
        if (type == TLV_FILE_SIZE) {
            // Parse file size (big-endian)
            for (int i = 0; i < length; i++) {
                *fileSize = (*fileSize << 8) | packet[idx++];
            }
        } else if (type == TLV_FILE_NAME) {
            memcpy(filename, &packet[idx], length);
            filename[length] = '\0';
            idx += length;
        } else {
            idx += length; // Skip unknown TLV
        }
    }
    
    return 0;
}

/**
 * Builds a data packet
 * Returns the size of the packet
 */
int buildDataPacket(unsigned char *packet, unsigned int sequenceNum,
                    const unsigned char *data, int dataSize) {
    int idx = 0;
    
    packet[idx++] = DATA_PACKET;
    packet[idx++] = sequenceNum % 256; // Sequence number (mod 256)
    packet[idx++] = (dataSize >> 8) & 0xFF; // L2 (high byte)
    packet[idx++] = dataSize & 0xFF;        // L1 (low byte)
    
    memcpy(&packet[idx], data, dataSize);
    idx += dataSize;
    
    return idx;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename) {
    // Parse command line (already done via function parameters)
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.role = strcmp(role, "tx") ? LlRx : LlTx;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    
    if (llopen(connectionParameters) == -1) {
        perror("applicationLayer: llopen failed\n");
        exit(-1);
    }

    FILE *file = NULL;

    if (connectionParameters.role == LlTx) {
        // ============ TRANSMITTER ============
        
        // Open file for reading (binary mode)
        file = fopen(filename, "rb");
        if (file == NULL) {
            perror("applicationLayer: failed to open file for reading");
            llclose();
            exit(-1);
        }

        // Obtain file size
        fseek(file, 0, SEEK_END);
        long fileSize = ftell(file);
        fseek(file, 0, SEEK_SET);
        
        if (fileSize < 0) {
            perror("applicationLayer: failed to get file size");
            fclose(file);
            llclose();
            exit(-1);
        }

        printf("File '%s' opened. Size: %ld bytes\n", filename, fileSize);

        // Build and send START control packet
        unsigned char controlPacket[MAX_PAYLOAD_SIZE];
        int controlPacketSize = buildControlPacket(controlPacket, CTRL_START, 
                                                   filename, fileSize);
        
        if (controlPacketSize < 0) {
            fprintf(stderr, "applicationLayer: failed to build START packet\n");
            fclose(file);
            llclose();
            exit(-1);
        }
        
        printf("Sending START control packet...\n");
        if (llwrite(controlPacket, controlPacketSize) < 0) {
            fprintf(stderr, "applicationLayer: failed to send START packet\n");
            fclose(file);
            llclose();
            exit(-1);
        }

        // Send file data in fragments
        unsigned char buffer[MAX_PAYLOAD_SIZE];
        unsigned char dataPacket[MAX_PAYLOAD_SIZE + 4];
        unsigned int sequenceNum = 0;
        long bytesSent = 0;

        printf("Sending file data...\n");
        while (!feof(file)) {
            int bytesRead = fread(buffer, 1, MAX_PAYLOAD_SIZE, file);
            if (bytesRead > 0) {
                int dataPacketSize = buildDataPacket(dataPacket, sequenceNum, 
                                                     buffer, bytesRead);
                
                if (llwrite(dataPacket, dataPacketSize) < 0) {
                    fprintf(stderr, "applicationLayer: failed to send data packet %u\n", 
                            sequenceNum);
                    fclose(file);
                    llclose();
                    exit(-1);
                }
                
                bytesSent += bytesRead;
                sequenceNum++;
                
                if (sequenceNum % 10 == 0) {
                    printf("Progress: %ld/%ld bytes (%.1f%%)\n", 
                           bytesSent, fileSize, 
                           (bytesSent * 100.0) / fileSize);
                }
            }
        }

        printf("File transmission complete: %ld bytes sent\n", bytesSent);

        // Build and send END control packet
        controlPacketSize = buildControlPacket(controlPacket, CTRL_END, 
                                              filename, fileSize);
        
        printf("Sending END control packet...\n");
        if (llwrite(controlPacket, controlPacketSize) < 0) {
            fprintf(stderr, "applicationLayer: failed to send END packet\n");
            fclose(file);
            llclose();
            exit(-1);
        }

        fclose(file);
    }
    else if (connectionParameters.role == LlRx) {
        // ============ RECEIVER ============
        
        unsigned char packet[MAX_PAYLOAD_SIZE * 2];
        int packetSize;
        long expectedFileSize = 0;
        long bytesReceived = 0;
        char receivedFilename[256];

        // Receive START control packet
        printf("Waiting for START control packet...\n");
        packetSize = llread(packet);
        if (packetSize < 0 || packet[0] != CTRL_START) {
            fprintf(stderr, "applicationLayer: failed to receive START packet\n");
            llclose();
            exit(-1);
        }

        if (parseControlPacket(packet, packetSize, receivedFilename, 
                              &expectedFileSize) < 0) {
            fprintf(stderr, "applicationLayer: failed to parse START packet\n");
            llclose();
            exit(-1);
        }

        printf("Receiving file '%s', size: %ld bytes\n", 
               receivedFilename, expectedFileSize);

        // Open file for writing
        file = fopen(filename, "wb");
        if (file == NULL) {
            perror("applicationLayer: failed to open file for writing");
            llclose();
            exit(-1);
        }

        // Receive data packets
        printf("Receiving file data...\n");
        while (1) {
            packetSize = llread(packet);
            if (packetSize < 0) {
                fprintf(stderr, "applicationLayer: error reading packet\n");
                fclose(file);
                llclose();
                exit(-1);
            }

            // Check packet type
            if (packet[0] == CTRL_END) {
                printf("Received END control packet\n");
                break;
            } else if (packet[0] == DATA_PACKET) {
                // Parse data packet
                unsigned int seqNum = packet[1];
                int dataSize = (packet[2] << 8) | packet[3];
                
                // Write data to file
                if (fwrite(&packet[4], 1, dataSize, file) != dataSize) {
                    perror("applicationLayer: failed to write to file");
                    fclose(file);
                    llclose();
                    exit(-1);
                }
                
                bytesReceived += dataSize;
                
                if (seqNum % 10 == 0) {
                    printf("Progress: %ld/%ld bytes (%.1f%%)\n", 
                           bytesReceived, expectedFileSize, 
                           (bytesReceived * 100.0) / expectedFileSize);
                }
            } else {
                fprintf(stderr, "applicationLayer: unexpected packet type %d\n", 
                        packet[0]);
            }
        }

        printf("File reception complete: %ld bytes received\n", bytesReceived);
        
        if (bytesReceived != expectedFileSize) {
            fprintf(stderr, "Warning: received %ld bytes, expected %ld bytes\n", 
                    bytesReceived, expectedFileSize);
        }

        fclose(file);
    }

    if (llclose() < 0) {
        perror("applicationLayer: llclose failed\n");
        exit(-1);
    }
    
    printf("Connection closed successfully\n");
}