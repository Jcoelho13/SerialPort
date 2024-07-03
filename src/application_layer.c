// Application layer protocol implementation

#include "../include/application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer parameters;
    strcpy(parameters.serialPort,serialPort);
    if (strcmp(role, "rx")==0) {
        parameters.role = LlRx;
        printf("i am RX\n");}
    else if (strcmp(role, "tx")==0){
        parameters.role = LlTx;
        printf("i am Tx\n");
    } 
    else exit(-1);
    parameters.baudRate = baudRate;
    parameters.nRetransmissions = nTries;
    parameters.timeout = timeout;

    int fd = llopen(parameters);
    if (fd < 0) {
        perror("Error opening\n");
        exit(-1);
    }
    if(parameters.role == LlTx){
        int penguin = open(filename, O_RDONLY);
        if (penguin == -1) {
            perror("Error opening file\n");
            exit(-1);
        }

        unsigned long long int fileSize = lseek(penguin, 0, SEEK_END); 
        lseek(penguin, 0, SEEK_SET);

        unsigned int size = 7;   
        unsigned char controlPacket[MAX_PAYLOAD_SIZE];
        controlPacket[0] = 2;
        controlPacket[1] = 0x00;
        controlPacket[2] = 4;
        controlPacket[3] = (fileSize >> 24) & 0xFF;
        controlPacket[4] = (fileSize >> 16) & 0xFF;
        controlPacket[5] = (fileSize >> 8) & 0xFF;
        controlPacket[6] = fileSize & 0xFF;


        if(llwrite(controlPacket, size) < 0){
            perror("Error in control packet\n");
            exit(-1);
        }

        unsigned char buffer[MAX_PAYLOAD_SIZE];

        long long int bytesRd = 0;
        long long int packetIndex = 1;

        while ((bytesRd = read(penguin, buffer, MAX_PAYLOAD_SIZE)) > 0) {
            unsigned char dataPacket[MAX_PAYLOAD_SIZE];

            dataPacket[0] = 1;
            dataPacket[1] = (unsigned char)((bytesRd >> 8) & 0xFF);
            dataPacket[2] = (unsigned char)(bytesRd & 0xFF);
            memcpy(&dataPacket[3], buffer, bytesRd);

            if (llwrite(dataPacket, bytesRd + 3) < 0) {
                perror("data packet send error");
                exit(EXIT_FAILURE);
            }

            packetIndex++;
        }

        unsigned char controlPacketEND[MAX_PAYLOAD_SIZE];
        controlPacketEND[0] = 3;
        controlPacketEND[1] = 0x00;
        controlPacketEND[2] = 4;
        controlPacketEND[3] = (fileSize >> 24) & 0xFF;
        controlPacketEND[4] = (fileSize >> 16) & 0xFF;
        controlPacketEND[5] = (fileSize >> 8) & 0xFF;
        controlPacketEND[6] = (fileSize) & 0xFF;

        if(llwrite(controlPacketEND, size) < 0){
            perror("Error in control packet end\n");
            exit(-1);
        }
        if(llclose(1) < 0){
            perror("Error closing serial port\n");
            exit(-1);
        }
    }
    else if(parameters.role == LlRx){

        unsigned char packet[MAX_PAYLOAD_SIZE];
        int pSize;
        /*     read first packet (start packet)     */
        pSize = llread(packet);
        if(packet[0] != 2){
                perror("Error receiving control packet 0\n");
                exit(-1);
            }
            if(packet[1] != 0x00){
                perror("Error receiving control packet 1\n");
                exit(-1);
            }
            if(packet[2] != 4){
                perror("Error receiving control packet 2\n");
                exit(-1);
            }
        printf("Start packet size: %d\n", pSize);

       
       /*   parse file size   */
        unsigned char fileSizeBytes = packet[2];
        unsigned char content[fileSizeBytes];
        for (int i = 0; i < fileSizeBytes; i++) {
            content[i] = packet[i + 3];
        }
        unsigned long int fileSize = 0;
        for (int j = 0; j < fileSizeBytes; j++) {
            fileSize |= (content[j] << (8 * (fileSizeBytes - j - 1)));
        }
        printf("File Size: %lu\n", fileSize);

        /*   parse file name   */
        const char name[] = "penguin-received.gif";
        printf("File Name: %s\n", name);
        int newPenguin = open(name, O_WRONLY | O_CREAT | O_TRUNC, 0666);      
        printf("file: %s\n", name);  
        if(newPenguin == -1){
            perror("Error opening file\n");
            exit(-1);
        }

        unsigned char buffer[MAX_PAYLOAD_SIZE];
        long long int bytesRd = 0;
        long long int packetIndex = 0;
        while(packetIndex<fileSize){
            unsigned char dataPacket[MAX_PAYLOAD_SIZE + 3];
            if(llread(dataPacket) < 0){
                perror("Error reading data packet\n");
                exit(-1);
            }
            if(dataPacket[0] != 1){
                perror("Error receiving data packet 0\n");
                exit(-1);
            }
            int packetSize = (dataPacket[1] << 8) | dataPacket[2];
            memcpy(buffer, &dataPacket[3], packetSize);
            bytesRd += packetSize;
            packetIndex += write(newPenguin, buffer, packetSize);
            printf("Packet index: %lld\n", packetIndex);
            printf("Bytes read: %lld\n", bytesRd);
            printf("Packet size: %d\n", packetSize);
            printf("File size: %lu\n", fileSize);
        }

                            /*   END PACKET   */
            int packetSize;
            unsigned char controlPacketEnd[MAX_PAYLOAD_SIZE];
            while (1) {
                packetSize = llread(controlPacketEnd);
                if (packetSize == 0) {break;}
                else if (controlPacketEnd[0] != 3 || controlPacketEnd[1] != 0 || controlPacketEnd[2] != 4) {
                    perror("not end packet\n");
                }
            }
            
            unsigned long long int fileSizeEnd = ((controlPacketEnd[3] << 24) | (controlPacketEnd[4] << 16) | (controlPacketEnd[5] << 8) | controlPacketEnd[6]);

            if(fileSize != fileSizeEnd){
                perror("Error receiving control packet end\n");
                exit(-1);
            }
            close(newPenguin);
    }
    else exit(-1);

    return;
}