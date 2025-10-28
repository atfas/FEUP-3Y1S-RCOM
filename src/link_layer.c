// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

int retransmissions = 0;
int timeout = 0;

int alarmTriggered = FALSE;
int alarmCount = 0;

void alarmHandler(int signal) {
    alarmTriggered = TRUE;
    alarmCount++;
    printf("Alarm Triggered\n");
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    LinkLayerState state = START;

    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) {
        perror("openSerialPort");
        exit(-1);
    }
    printf("Serial port %s opened\n", connectionParameters.serialPort);

    retransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

    switch (connectionParameters.role) {
        case LlTx: { // transmitter
            struct sigaction act = {0};
            act.sa_handler = &alarmHandler;
            if (sigaction(SIGALRM, &act, NULL) == -1) {
                perror("sigaction");
                exit(1);
            }

            printf("Alarm configured\n");
            while (connectionParameters.nRetransmissions != 0 && state != STOP) {
                alarm(connectionParameters.timeout);
                alarmTriggered = FALSE;

                unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_SET, ADDR_TR ^ CTRL_SET, FLAG};
                int n = writeBytesSerialPort(FRAME, 5);
                printf("Written %d bytes\n", n);

                while (alarmTriggered == FALSE && state != STOP) {
                    unsigned char byte;
                    int bytes = readByteSerialPort(&byte);

                    if (bytes == 0) continue;

                    //printf("0x%02X\n", byte);

                    switch (state) {
                        case START:
                            if (byte == FLAG) state = FLAG_R;
                            break;
                        case FLAG_R:
                            if (byte == ADDR_TR) state = ADDR_R;
                            else if (byte != FLAG) state = START;
                            break;
                        case ADDR_R:
                            if (byte == CTRL_UA) state = CTRL_R;
                            else if (byte == FLAG) state = FLAG_R;
                            else state = START;
                            break;
                        case CTRL_R:
                            if (byte == (ADDR_TR ^ CTRL_UA)) state = BCC1_OK;
                            else if (byte == FLAG) state = FLAG_R;
                            else state = START;
                            break;
                        case BCC1_OK:
                            if (byte == FLAG) state = STOP;
                            else state = START;
                            break;
                        default: 
                            break;
                    }
                }
                connectionParameters.nRetransmissions--;
            }
            if (state != STOP) return -1;
            break;
        }
        case LlRx: { // receiver
            while (state != STOP) {
                unsigned char byte;
                int bytes = readByteSerialPort(&byte);

                if (bytes == 0) continue;
                //printf("0x%02X %d\n", byte, state);
                
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_R;
                        break;
                    case FLAG_R:
                        if (byte == ADDR_TR) state = ADDR_R;
                        else if (byte != FLAG) state = START;
                        break;
                    case ADDR_R:
                        if (byte == CTRL_SET) state = CTRL_R;
                        else if (byte == FLAG) state = FLAG_R;
                        else state = START;
                        break;
                    case CTRL_R:
                        if (byte == (ADDR_TR ^ CTRL_SET)) state = BCC1_OK;
                        else if (byte == FLAG) state = FLAG_R;
                        else state = START;
                        break;
                    case BCC1_OK:
                        if (byte == FLAG) state = STOP;
                        else state = START;
                        break;
                    default: 
                        break;
                }
            }
            unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_UA, ADDR_TR ^ CTRL_UA, FLAG};
            writeBytesSerialPort(FRAME, 5);
            break;
        }
    }

    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int orderBit = 0;
int llwrite(const unsigned char *buf, int bufSize)
{
    int frameSize = 4 + bufSize + 2;
    unsigned char *frame = (unsigned char *) malloc(frameSize);

    frame[0] = FLAG;
    frame[1] = ADDR_TR;
    frame[2] = CTRL_I(orderBit);
    frame[3] = frame[1] ^ frame[2];
    memcpy(frame+4, buf, bufSize);

    unsigned char BCC2 = 0x00;

    int j = 4;
    for (int i = 0; i < bufSize; i++) {
        BCC2 ^= buf[i];
        if (buf[i] == FLAG || buf[i] == ESC) {
            frame = realloc(frame, ++frameSize);
            frame[j] = ESC;
            j++;
            frame[j] = buf[i] ^ 0x20;
            j++;
        }
        else {
            frame[j] = buf[i];
            j++;
        }
    }

    frame[j] = BCC2;
    frame[j+1] = FLAG;

    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1) {
        perror("sigaction");
        exit(1);
    }

    printf("Alarm configured\n");


    int transmission = 0;
    int rejected = FALSE, accepted = FALSE;
    unsigned char message = 0;
    while (transmission < retransmissions) {
        alarmTriggered = FALSE;
        alarm(timeout);
        rejected = FALSE;
        accepted = FALSE;
        while (alarmTriggered == FALSE && rejected == FALSE && accepted == FALSE) {
            writeBytesSerialPort(frame, frameSize);
            
            LinkLayerState state = START;
            while (alarmTriggered == FALSE && state != STOP) {
                unsigned char byte;
                int bytes = readByteSerialPort(&byte);

                if (bytes == 0) continue;

                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_R;
                        break;
                    case FLAG_R:
                        if (byte == ADDR_TR) state = ADDR_R;
                        else if (byte != FLAG) state = START;
                        break;
                    case ADDR_R:
                        if (byte == CTRL_RR(0) || byte == CTRL_RR(1) || byte == CTRL_REJ(0) || byte == CTRL_REJ(1)) {
                            state = CTRL_R;
                            message = byte;
                        }
                        else if (byte == FLAG) state = FLAG_R;
                        else state = START;
                        break;
                    case CTRL_R:
                        if (byte == (ADDR_TR ^ message)) state = BCC1_OK;
                        else if (byte == FLAG) state = FLAG_R;
                        break;
                    case BCC1_OK:
                        if (byte == FLAG) state = STOP;
                        else state = START;
                        break;
                    default:
                        break;
                }
            }

            if (!message) continue;
            else if (message == CTRL_REJ(orderBit)) rejected = TRUE;
            else if (message == CTRL_RR((orderBit+1)%2)) {
                accepted = TRUE;
                orderBit = (orderBit+1) % 2;
                break;
            }
            else continue;
        }
        if (accepted == TRUE) break;
        transmission++;
    }
    free(frame);
    if (accepted == TRUE) return frameSize;
    else {
        closeSerialPort();
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int orderBitRx = 0;
int llread(unsigned char *packet) {
    LinkLayerState state = START;
    unsigned char control;
    int i = 0;
    int ignore = FALSE;

    while (state != STOP) {
        unsigned char byte;
        int bytes = readByteSerialPort(&byte);

        if (bytes == 0) continue;

        //printf("0x%02X\n", byte);

        switch (state) {
            case START:
                if (byte == FLAG) state = FLAG_R;
                break;
            case FLAG_R:
                if (byte == ADDR_TR) state = ADDR_R;
                else if (byte != FLAG) state = START;
                break;
            case ADDR_R:
                if (byte == CTRL_I(orderBitRx)) {
                    state = CTRL_R;
                    control = byte;
                }
                else if (byte == CTRL_I((orderBitRx+1)%2)) {
                    state = CTRL_R;
                    control = byte;
                    ignore = TRUE;
                }
                else if (byte == FLAG) state = FLAG_R;
                else if (byte == CTRL_DISC) {
                    unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_UA, ADDR_TR ^ CTRL_UA, FLAG};
                    int n = writeBytesSerialPort(FRAME, 5);
                    printf("Received disconnect\n");
                    printf("Written %d bytes\n", n);
                    return 0;
                }
                else state = START;
                break;
            case CTRL_R:
                if (byte == (ADDR_TR ^ control)) state = BCC1_OK;
                else if (byte == FLAG) state = FLAG_R;
                else state = START;
                break;
            case BCC1_OK:
                if (ignore == TRUE) {
                    unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_RR(orderBitRx), ADDR_TR ^ CTRL_RR(orderBitRx), FLAG};
                    writeBytesSerialPort(FRAME, 5);
                    return 0;
                }
                else if (byte == ESC) state = DATA_ESCAPE;
                else if (byte == FLAG) {
                    if (i == 0) {
                        unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_REJ(orderBitRx), ADDR_TR ^ CTRL_REJ(orderBitRx), FLAG};
                        writeBytesSerialPort(FRAME, 5);
                        exit(-1);
                    }
                    unsigned char correct_bcc2 = packet[--i];
                    packet[i] = '\0';
                    unsigned char bcc2 = packet[0];
                    for (unsigned int j = 1; j < i; j++) {
                        bcc2 ^= packet[j];
                    }

                    if (bcc2 == correct_bcc2) {
                        state = STOP;
                        orderBitRx = (orderBitRx + 1)%2;
                        unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_RR(orderBitRx), ADDR_TR ^ CTRL_RR(orderBitRx), FLAG};
                        writeBytesSerialPort(FRAME, 5);
                        return i;
                    }
                    else {
                        printf("Error bcc2");
                        unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_REJ(orderBitRx), ADDR_TR ^ CTRL_REJ(orderBitRx), FLAG};
                        writeBytesSerialPort(FRAME, 5);
                        return -1;
                    }
                }
                else packet[i++] = byte;
                break;
            case DATA_ESCAPE:
                state = BCC1_OK;
                if (byte == (ESC ^ 0x20) || byte == (FLAG ^ 0x20)) {
                    packet[i++] = (byte ^ 0x20);
                }
                else {
                    printf("Error: invalid escape sequence (0x%02X)\n", byte);
                    unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_REJ(orderBitRx), ADDR_TR ^ CTRL_REJ(orderBitRx), FLAG};
                    writeBytesSerialPort(FRAME, 5);
                    return -1;
                }
                break;
            default:
                break;
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    printf("LL: close called\n");
    LinkLayerState state = START;
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1) {
        perror("sigaction");
        exit(1);
    }

    printf("Alarm configured\n");

    int transmission = 0;

    while (transmission < retransmissions && state != STOP) {
        unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_DISC, ADDR_TR ^ CTRL_DISC, FLAG};
        writeBytesSerialPort(FRAME, 5);

        alarm(timeout);
        alarmTriggered = FALSE;
        
        while (alarmTriggered == FALSE && state != STOP) {
            unsigned char byte;
            int bytes = readByteSerialPort(&byte);

            if (bytes == 0) continue;

            //printf("0x%02X\n", byte);

            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_R;
                    break;
                case FLAG_R:
                    if (byte == ADDR_TR) state = ADDR_R;
                    else if (byte != FLAG) state = START;
                    break;
                case ADDR_R:
                    if (byte == CTRL_DISC) state = CTRL_R;
                    else if (byte == FLAG) state = FLAG_R;
                    else state = START;
                    break;
                case CTRL_R:
                    if (byte == (ADDR_TR ^ CTRL_DISC)) state = BCC1_OK;
                    else if (byte == FLAG) state = FLAG_R;
                    else state = START;
                    break;
                case BCC1_OK:
                    if (byte == FLAG) state = STOP;
                    else state = START;
                    break;
                default: 
                    break;
            }
        }
        transmission++;
    }

    if (state != STOP) return -1;
    unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_UA, ADDR_TR ^ CTRL_UA, FLAG};
    writeBytesSerialPort(FRAME, 5);
    return closeSerialPort();
}
