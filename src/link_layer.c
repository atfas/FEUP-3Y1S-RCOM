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

int destuff(unsigned char* destuffed, int frameSize, const unsigned char* frame) {
    int i = 0;
    int j = 0;

    while (i < frameSize) {
        if (frame[i] == ESC) {
            if (i + 1 >= frameSize) break;
            destuffed[j++] = frame[i + 1] ^ 0x20;
            i += 2;
        } else destuffed[j++] = frame[i++];
    }
    return j;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    LinkLayerState state = START;

    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) {
        perror("openSerialPort");
        exit(1);
    }
    printf("Serial port %s opened\n", connectionParameters.serialPort);
    
    unsigned char byte;

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
                unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_SET, ADDR_TR ^ CTRL_SET, FLAG};
                int n = writeBytesSerialPort(FRAME, 5);
                printf("Written %d bytes\n", n);
                
                alarm(connectionParameters.timeout);
                alarmTriggered = FALSE;

                while (alarmTriggered == FALSE && state != STOP) {
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
        case LlRx: {
            while (state != STOP) {
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
        default:
            return -1;
            break;
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

    unsigned char BCC2 = buf[0];

    for (int i = 1; i < bufSize; i++) BCC2 ^= buf[i];

    int j = 4;
    for (int i = 0; i < bufSize; i++) {
        //BCC2 ^= buf[i];
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

    if (BCC2 == FLAG || BCC2 == ESC) {
        frame = realloc(frame, ++frameSize);
        frame[j] = ESC;
        j++;
        frame[j] = BCC2 ^ 0x20;
        j++;
    } else {
        frame[j] = BCC2;
        j++;
    }

    frame[j] = FLAG;

    int transmission = 0;
    int rejected = FALSE, accepted = FALSE;
    while (transmission < retransmissions) {
        alarmTriggered = FALSE;
        alarm(timeout);
        rejected = FALSE;
        accepted = FALSE;
        while (alarmTriggered == FALSE && rejected == FALSE && accepted == FALSE) {
            writeBytesSerialPort(frame, frameSize); //this
            
            LinkLayerState state = START;
            unsigned char byte;
            unsigned char message = 0;
            while (alarmTriggered == FALSE && state != STOP) {
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
            alarm(0);

            if (!message) continue;
            else if (message == CTRL_REJ(orderBit)) rejected = TRUE;
            else if (message == CTRL_RR((orderBit+1)%2)) {
                accepted = TRUE;
                orderBit = (orderBit+1) % 2;
            }
            else continue;
        }
        if (accepted == TRUE) break;
        transmission++;
    }
    free(frame);
    if (accepted == TRUE) return frameSize;
    else {
        llclose();
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int orderBitRx = 0;
int llread(unsigned char *packet) {
    LinkLayerState state = START;
    unsigned char byte;
    unsigned char control;
    int i = 0;

    unsigned char frame[MAX_PAYLOAD_SIZE*2];
    unsigned char destuffed[MAX_PAYLOAD_SIZE];

    while (state != STOP) {
        int bytes = readByteSerialPort(&byte);

        if (bytes == 0) continue;

        //printf("0x%02X\n", byte);

        switch (state) {
            case START:
                i = 0;
                if (byte == FLAG) state = FLAG_R;
                break;
            case FLAG_R:
                if (byte == ADDR_TR) state = ADDR_R;
                else if (byte != FLAG) state = START;
                break;
            case ADDR_R:
                if (byte == CTRL_I(0) || byte == CTRL_I(1) || byte == CTRL_DISC) {
                    state = CTRL_R;
                    control = byte;
                }
                else if (byte == FLAG) state = FLAG_R;
                else state = START;
                break;
            case CTRL_R:
                if (byte == (ADDR_TR ^ control)) state = BCC1_OK;
                else if (byte == FLAG) state = FLAG_R;
                else state = START;
                break;
            case BCC1_OK:
                if (control == CTRL_I((orderBitRx+1)%2)) {
                    unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_RR(orderBitRx), ADDR_TR ^ CTRL_RR(orderBitRx), FLAG};
                    writeBytesSerialPort(FRAME, 5);
                    state = START;
                }
                else if (control == CTRL_DISC) {
                    unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_DISC, ADDR_TR ^ CTRL_DISC, FLAG};
                    int n = writeBytesSerialPort(FRAME, 5);
                    printf("Received disconnect\n");
                    printf("Written %d bytes\n", n);
                    return 0;
                }
                else if (byte == FLAG) {
                    if (i == 0) {
                        unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_REJ(orderBitRx), ADDR_TR ^ CTRL_REJ(orderBitRx), FLAG};
                        writeBytesSerialPort(FRAME, 5);
                        exit(1);
                    }
                    int dataSize = destuff(destuffed, i, frame);

                    unsigned char correct_bcc2 = destuffed[dataSize-1];
                    unsigned char calc_bcc2 = destuffed[0];
                    for (int i = 1; i < dataSize - 1; i++) {
                        calc_bcc2 ^= destuffed[i];
                    }

                    if (correct_bcc2 == calc_bcc2) {
                        if (control == CTRL_I(orderBitRx)) {
                            orderBitRx = (orderBitRx +1) % 2;
                            unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_RR(orderBitRx), ADDR_TR ^ CTRL_RR(orderBitRx), FLAG};
                            writeBytesSerialPort(FRAME, 5);
                            memcpy(packet, destuffed, dataSize - 1);
                            return dataSize - 1;
                        }
                    } else {
                        printf("BCC2 error\n");
                        unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_REJ(orderBitRx), ADDR_TR ^ CTRL_REJ(orderBitRx), FLAG};
                        writeBytesSerialPort(FRAME, 5);
                        state = START;
                    }
                }
                else {
                    if (i < MAX_PAYLOAD_SIZE*2)
                        frame[i++] = byte;
                    else {
                        printf("Frame overflow\n");
                        i = 0;
                        state = START;
                    }
                }
                break;
            default:
                break;
            }
        }
    printf("No frame received\n");
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
    unsigned char byte;

    while (transmission < retransmissions && state != STOP) {
        unsigned char FRAME[5] = {FLAG, ADDR_TR, CTRL_DISC, ADDR_TR ^ CTRL_DISC, FLAG};
        writeBytesSerialPort(FRAME, 5);

        alarm(timeout);
        alarmTriggered = FALSE;
        
        while (alarmTriggered == FALSE && state != STOP) {
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