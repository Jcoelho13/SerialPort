// Link layer protocol implementation
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <signal.h>
#include "../include/link_layer.h"
#include <string.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

/*The following section includes the definitions of Supervision (S) and Unnumbered (U) Frames */
#define C_0 0x00
#define C_1 0x40
#define FLAG 0x7E
#define ESCAPE 0x7D

#define A_T 0x03
#define A_R 0x01

#define CF_SET 0x03
#define CF_UA 0x07
#define CF_RR0 0x05
#define CF_RR1 0x85
#define CF_REJ0 0x01
#define CF_REJ1 0x81
#define CF_DISC 0x0B

/*The following section includes the definitions of the possible states of the state machine*/

typedef enum {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP, READING_DATA, BYTE_DESTUFFING} State;

LinkLayerRole role;

int timeout;
int n_retransmissions;
int fd;
int frame_no = 0;

int alarmState = FALSE;

void alarmHandler(int signal) {
    alarmState = TRUE;
}

unsigned char set[5] = {FLAG, A_T, CF_SET, A_T ^ CF_SET, FLAG};
unsigned char ua[5] = {FLAG, A_T, CF_UA, A_T ^ CF_UA, FLAG};
unsigned char rr0[5] = {FLAG, A_T, CF_RR0, A_T ^ CF_RR0, FLAG};
unsigned char rr1[5] = {FLAG, A_T, CF_RR1, A_T ^ CF_RR1, FLAG};
unsigned char rej0[5] = {FLAG, A_T, CF_REJ0, A_T ^ CF_REJ0, FLAG};
unsigned char rej1[5] = {FLAG, A_T, CF_REJ1, A_T ^ CF_REJ1, FLAG};
unsigned char disc[5] = {FLAG, A_T, CF_DISC, A_T ^ CF_DISC, FLAG};

struct termios pre, pos;

int llopen(LinkLayer connectionParameters)
{
    printf("%d\n", connectionParameters.role);
    State state = START;
    printf("starting state: %d\n", state);

    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY );

    if (fd < 0) {
        perror(connectionParameters.serialPort);
        return -1;
    }

    n_retransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

    if ( tcgetattr(fd,&pre) == -1) {
        perror("tcgetattr");
        return -1;
    }

    memset(&pos,0,sizeof(pos));
    pos.c_oflag = 0;
    pos.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    pos.c_iflag = IGNPAR;
    pos.c_cc[VTIME] = 0;
    if (connectionParameters.role == 0){
        pos.c_cc[VMIN] = 0;
    }
    else if (connectionParameters.role == 1){
        pos.c_cc[VMIN] = 1;
    }
    else{
        printf("Invalid role.\n");
        return -1;
    }

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &pos) == -1){
        perror("tcsetattr");
        return -1;
    }    

    unsigned char check;

    switch (connectionParameters.role){
        case LlTx: {
            while (connectionParameters.nRetransmissions > 0 && state != STOP) {
                if (write(fd, set, 5) < 0) {
                    perror("write");
                    return -1;
                }
                else{
                    printf("sent %d | %d | %d | %d | %d\n", set[0], set[1], set[2], set[3], set[4]);
                }
                (void) signal(SIGALRM, alarmHandler);
                alarm(timeout);
                alarmState = FALSE;

                while (alarmState == FALSE && state != STOP){
                    if(read(fd, &check, 1) > 0){
                        printf("read byte %d\n", check);
                        switch(state){
                            case START:
                                if(check == FLAG)
                                {
                                    state = FLAG_RCV;
                                    printf("Reached FLAG_RCV\n");
                                }
                                break;
                            case FLAG_RCV:
                                if(check == A_T)
                                    state = A_RCV;
                                else if(check != FLAG)
                                    state = START;
                                break;
                            case A_RCV:
                                if(check == CF_UA)
                                    state = C_RCV;
                                else if(check == FLAG)
                                    state = FLAG_RCV;
                                else
                                    state = START;
                                break;
                            case C_RCV:
                                if(check == (A_T ^ CF_UA))
                                {
                                    state = BCC_OK;
                                    printf("Reached BCC OK \n");
                                }
                                else if(check == FLAG)
                                    state = FLAG_RCV;
                                else
                                    state = START;
                                break;
                            case BCC_OK:
                                if(check == FLAG) {
                                    state = STOP;
                                    printf("Reached stop\n");
                                }
                                else
                                    state = START;
                                break;
                            default:
                                break;
                        }
                    }
                }
                    connectionParameters.nRetransmissions--;
            }
            if (state == STOP)
                printf("Connection established.\n");
            else{
                printf("Connection failed.\n");
                return -1;
            }
            break;
        }
        case LlRx: {
            while (state != STOP) {
                if(read(fd, &check, 1) > 0){
                    switch(state){
                        case START:
                            if(check == FLAG)
                                state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if(check == A_T)
                                state = A_RCV;
                            else if(check != FLAG)
                                state = START;
                            break;
                        case A_RCV:
                            if(check == CF_SET)
                                state = C_RCV;
                            else if(check == FLAG)
                                state = FLAG_RCV;
                            else
                                state = START;
                            break;
                        case C_RCV:
                            if(check == (A_T ^ CF_SET))
                                state = BCC_OK;
                            else if(check == FLAG)
                                state = FLAG_RCV;
                            else
                                state = START;
                            break;
                        case BCC_OK:
                            if(check == FLAG)
                                state = STOP;
                            else
                                state = START;
                            break;
                        default:
                            break;
                    }
                }
            }
            printf("Read set frame\n");
            if (write(fd, ua, 5) < 0) {
                perror("write");
                return -1;
            }
            else printf("Sent UA\n");
            break;
        }
        default:
            return -1;
            break;
    }
    
    return fd;
}

////////////////////////////////////////////////
//                  LLWRITE                   //
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    printf("Entered write\n");
    int actualSize = bufSize;
    unsigned char BCC2 = buf[0];

    printf("\nSending %i bytes...\n\n", bufSize-3);

    for (int i = 1; i < bufSize; i++) {
        if (buf[i] == FLAG || buf[i] == ESCAPE) {
            //printf("Found FLAG/ESCAPE on byte %i\n", i);
            actualSize += 1;
        }
        BCC2 = BCC2 ^ buf[i];
    }

    //printf("Buffer size with stuffing: %i\n", actualSize);

    int frameSize = actualSize + 6;
    //printf("Frame Size: %d\n", frameSize);
    unsigned char i_frame[frameSize];
    i_frame[0] = FLAG;
    i_frame[1] = A_T;
    i_frame[2] = frame_no ? C_1 : C_0;
    i_frame[3] = i_frame[1] ^ i_frame[2];

    /* byte stuffing */
        int j = 4;
    for (int i = 0; i < bufSize; i++) {
        if (buf[i] == FLAG) {
            //printf("Found flag on byte %i. Stuffing...\n", i);
            i_frame[j++] = 0x7D;
            i_frame[j++] = 0x5E;
        }
        else if (buf[i] == ESCAPE) {
            //printf("Found escape on byte %i. Stuffing...\n", i);
            i_frame[j++] = 0x7D;
            i_frame[j++] = 0x5D;
        }
        else {
            i_frame[j++] = buf[i];
        }
    }

    i_frame[frameSize - 2] = BCC2;
    i_frame[frameSize - 1] = FLAG;

    State state = START;
    int rejected = 0;
    int received = 0;

    while (n_retransmissions > 0 && received == 0) {
        if (write(fd, i_frame, frameSize) < 0) {
            perror("write");
            return -1;
        }
        else {
            printf("Sent frame\n");
        }
        (void) signal(SIGALRM, alarmHandler);
        alarm(timeout);
        alarmState = FALSE;
        unsigned char byte;

        while (alarmState == FALSE && !rejected && !received){
            if(read(fd, &byte, 1) > 0){
                //printf("Received byte %02X\n", byte);
                switch(state){
                    case START:
                        if(byte == FLAG)
                            state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if(byte == A_R)
                            state = A_RCV;
                        else if(byte != FLAG)
                            state = START;
                        break;
                    case A_RCV:
                        if(byte == CF_REJ0 || byte == CF_REJ1) {
                            rejected = 1;
                            state = C_RCV;
                        }
                        else if(byte == CF_RR0 || byte == CF_RR1) {
                            received = 1;
                            frame_no = (frame_no + 1) % 2; //passa de 0 para 1 ou 1 para 0
                            state = C_RCV;
                        }
                        else if(byte == FLAG)
                            state = FLAG_RCV;
                        else
                            state = START;
                        break;
                    case C_RCV:
                        if(byte == (A_R ^ CF_UA))
                            state = BCC_OK;
                        else if(byte == FLAG)
                            state = FLAG_RCV;
                        else
                            state = START;
                        break;
                    case BCC_OK:
                        if(byte == FLAG) {
                            //printf("Reached STOP\n");
                            state = STOP;
                        }
                        else
                            state = START;
                        break;
                    default:
                        break;
                }
            }
        }
            if (received) {break;}
        n_retransmissions--;
        printf("Timeout\n");
    }

    if (received) return frameSize;

    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    State state = START;
    unsigned char byte;
    int control_field = 0;
    int i = 0; // Index for data packet
    int frame_no;

    while (state != STOP) {
        if (read(fd, &byte, 1) > 0) {
            //printf("Received byte %02X\n", byte);
            switch(state) {
                case START:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                        //printf("Reached start\n");
                    }
                    break;
                case FLAG_RCV:
                    if (byte == A_T) {
                        state = A_RCV;
                        //printf("Reached address\n");
                    }
                    else if (byte == FLAG) {
                        state = FLAG_RCV;
                    }
                    else {state = START;}
                    break;
                case A_RCV:
                    if (byte == C_0) {
                        frame_no = 0;
                        state = C_RCV;
                        control_field = byte;
                        //printf("Reading frame %d\n", frame_no);
                    }
                    else if (byte == C_1) {
                        frame_no = 1;
                        state = C_RCV;
                        control_field = byte;
                        //printf("Reading frame %d\n", frame_no);
                    }
                    else if (byte == FLAG) {
                        state = FLAG_RCV;
                    }
                    else if (byte == CF_DISC) {
                        unsigned char disc[5] = {FLAG, A_R, CF_DISC, A_R ^ CF_DISC, FLAG};
                        if(write(fd, disc, 5)<0){
                            //printf("Error in llread\n");
                            return -1;
                        };
                        return 0;
                    }
                    else {state = START;}
                    break;
                case C_RCV:
                    if (byte == (A_T ^ control_field)) {
                        state = READING_DATA;
                    }
                    else if (byte == FLAG) {state = FLAG_RCV;}
                    else {state = START;}
                    break;
                case READING_DATA: // Passed BCC1, now reading data
                    //printf("REACHED READING DATA\n");
                    if (byte == ESCAPE) { // Reached a stuffed byte
                        state = BYTE_DESTUFFING;
                    }
                    else if (byte == FLAG) { // Reached end of frame
                        //printf("Reached end of packet (FLAG)\n");
                        unsigned char BCC2 = packet[i - 1]; // Access bcc2 byte
                        i--;
                        packet[i] = '\0'; // Closes packet and takes out bcc2 (not part of the data)
                        unsigned char control_check = packet[0];

                        for (int p = 1; p < i; p++) {
                            control_check ^= packet[p];
                        }
                        if (BCC2 == control_check) {
                            //printf("BCC2 checked\n");
                            state = STOP;


                            if (frame_no == 0) { // Write RR(1)
                                unsigned char response[5] = {FLAG, A_R, CF_RR1, A_R^CF_RR1, FLAG};
                                write(fd, response, 5);
                            }


                            else if (frame_no == 1) { // Write RR(0)
                                unsigned char response[5] = {FLAG, A_R, CF_RR0, A_R^CF_RR0, FLAG};
                                write(fd, response, 5);
                            }


                            return i;
                        }
                        else {
                            if (frame_no == 0) { // Write REJ(0)
                                unsigned char response[5] = {FLAG, A_R, CF_REJ0, A_R^CF_REJ0, FLAG};
                                write(fd, response, 5);
                            }
                            else if (frame_no == 1) { // Write REJ(1)
                                unsigned char response[5] = {FLAG, A_R, CF_REJ1, A_R^CF_REJ1, FLAG};
                                write(fd, response, 5);
                            }
                            return -1;
                        }
                    }
                    else {
                        packet[i] = byte;
                        i++;
                    }
                    break;
                case BYTE_DESTUFFING:
                    printf("Destuffing...\n");
                    if (byte == 0x5E) {
                        packet[i] = FLAG;
                        i++;
                    }
                    if (byte == 0x5D) {
                        packet[i] = ESCAPE;
                        i++;
                    }
                    state = READING_DATA;
                    break;

                default:
                    break;
            }
        }
    }

    return -1;
}

int llclose(int showStatistics)
{

    State state = START;
    int decrement = n_retransmissions;
    unsigned char check;
    (void) signal(SIGALRM, alarmHandler);

    while(state != STOP && decrement > 0){

        if (write(fd, disc, 5) < 0) {
            perror("write");
            return -1;
        }
        else printf("Sent DISC\n");

        alarm(timeout);
        alarmState = FALSE;

        while(state != STOP && alarmState == FALSE){
            if(read(fd, &check, 1) > 0){
                switch(state){
                    case START:
                        if(check == FLAG)
                            state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if(check == A_R)
                            state = A_RCV;
                        else if(check != FLAG)
                            state = START;
                        break;
                    case A_RCV:
                        if(check == CF_DISC)
                            state = C_RCV;
                        else if(check == FLAG)
                            state = FLAG_RCV;
                        else
                            state = START;
                        break;
                    case C_RCV:
                        if(check == (A_R ^ CF_DISC))
                            state = BCC_OK;
                        else if(check == FLAG)
                            state = FLAG_RCV;
                        else
                            state = START;
                        break;
                    case BCC_OK:
                        if(check == FLAG)
                            state = STOP;
                        else
                            state = START;
                        break;
                    default:
                        break;
                }
            }
        }
        decrement--;
    }

    if (state != STOP){
        perror("Error closing.\n");
        return -1;
    }

    if (write(fd, ua, 5) < 0) {
        perror("write UA");
        return -1;
    }
    else printf("Sent UA\n");

    return close(fd);
}