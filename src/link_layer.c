// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

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

    switch (connectionParameters.role) {
        case LlTx: {
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

                    printf("0x%02X\n", byte);

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
                            if (byte == (ADDR_TR ^ CTRL_UA)) state = BCC_OK;
                            else if (byte == FLAG) state = FLAG_R;
                            else state = START;
                            break;
                        case BCC_OK:
                            if (byte == FLAG) state = STOP;
                            else state = START;
                            break;
                        default: 
                            break;
                    }
                }
                connectionParameters.nRetransmissions--;
            }
            break;
        }
        case LlRx: {
            while (state != STOP) {
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
                        if (byte == CTRL_SET) state = CTRL_R;
                        else if (byte == FLAG) state = FLAG_R;
                        else state = START;
                        break;
                    case CTRL_R:
                        if (byte == (ADDR_TR ^ CTRL_SET)) state = BCC_OK;
                        else if (byte == FLAG) state = FLAG_R;
                        else state = START;
                        break;
                    case BCC_OK:
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
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}
