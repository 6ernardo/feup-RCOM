// Application layer protocol implementation

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "application_layer.h"
#include "link_layer.h"

#define BUFSIZE 512
#define FRAGMENTSIZE 256
#define C_START 0x02
#define C_END 0x03
#define T_SIZE 0x00
#define T_NAME 0x01
#define C_DATA 0x01

unsigned char packet[300];

int TLV(unsigned char *address, unsigned char* T, unsigned char* L, unsigned char** V){
    *T=address[0];
    *L=address[1];
    *V=address+2;
    return 2 + *L; //returns size of the TLV.
}

long int buildControlPacket(int ctrl, unsigned char *buffer, FILE *file){
    //get file size in bytes
    fseek(file, 0, SEEK_END);
    long int fileSize=ftell(file);
    fseek(file,0, SEEK_SET);

    //build control packet
    if(ctrl){
        buffer[0] = C_START;
    }
    else buffer[0] = C_END;
    buffer[1] = T_SIZE;
    buffer[2] = sizeof(long);
    *((long*)(buffer+3))=fileSize;

    return fileSize;
}

void buildDataPacket(unsigned char *buffer, int i, unsigned long bytes){
    buffer[0]=C_DATA;
    buffer[1]=i;
    buffer[2]=bytes>>8;
    buffer[3]=bytes%256;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename)
{
    LinkLayer link_struct;

    strcpy(link_struct.serialPort, serialPort);

    if(strcmp(role, "tx") == 0)
    {
        link_struct.role= LlTx;
    } else if(strcmp(role, "rx")==0) {
        link_struct.role = LlRx;
    } else {
        perror("LinkLayer Role not valid.\n");
    }

    link_struct.baudRate = baudRate;
    link_struct.nRetransmissions = nTries;
    link_struct.timeout = timeout;

    printf("Establishing the connection.\n");
    int connection_fd = llopen(link_struct);
    if(connection_fd <=0)
    {
        perror("Could not establish connection.\n");
        exit(-1);
    }
    else printf("Connection established.\n");

    //everything correct until here

    //sleep(5);
  
    if(link_struct.role==LlTx){

        printf("\nExecuting llwrite:\n\n");

        //open file
        FILE *file = fopen(filename, "r");
        
        if(!file){
            printf("Couldn't open file.\n");
            exit(-1);
        }

        //build control packet

        long int fileSize = buildControlPacket(TRUE, packet, file);

        int fail = FALSE;

        //send control packet
        if(llwrite(packet, 10)==-1){
            printf("llwrite failed.\n");
            fail = TRUE;
            exit(-1);
        }

        unsigned long sent = 0;

        for(unsigned char i=0; sent<fileSize && fail == FALSE; ++i){
            size_t aux;
            
            if(fileSize-sent>1024) aux = 1024;
            else aux = fileSize - sent;

            unsigned long bytes = fread(packet+4, 1, aux, file);

            if(bytes!=aux){
                printf("File read failure.\n");
                fail=TRUE;
                break;
            }

            //build and send data packets
            unsigned char packet1[300];

            buildDataPacket(packet1, i, bytes);

            if(llwrite(packet1, bytes+4)==-1){
            printf("llwrite failed.\n");
            fail = TRUE;
            break;
            }
            else{
                printf("Data packet sent successfully.\n");
            }
        }

        if(!fail){
            //build and sent control packet
            buildControlPacket(FALSE, packet, file);

            if(llwrite(packet, 1)==-1){
                printf("llwrite failed.\n");
            }
        }

        fclose(file);
        //app layer llwrite finished
    }
    
    //app layer llread

    else if(link_struct.role == LlRx) {
        unsigned long filesize=0;
        
        printf("\nExecuting llread:\n\n");
        
        int bytes_read = llread(packet);
        unsigned char T,L,*V;
        
        if(packet[0] == C_START){
            int offset=1;           
            while(offset<bytes_read){
                
                offset+=TLV(offset+packet, &T, &L, &V);

                if(T==T_SIZE){
                    filesize=*((unsigned long*)V);
                }
            }
            
            FILE* file = fopen(filename, "w");
            if(!file) {
                printf("Could not open file to write in.\n");
                return;
            }

            unsigned long recieved_bytes=0;
            
            while(recieved_bytes<filesize){
                if(llread(packet)<1){
                    printf("llread failure.\n");
                    break;
                }

                if(packet[0]==C_END){
                    printf("Disconnected before end of file.\n");
                    break;
                }
                
                if(packet[0]==C_DATA){
                    unsigned long size=packet[3]+packet[2]*256;
                    fwrite(packet+4,1,size,file);
                    recieved_bytes+=size;
                }
            }
            fclose(file);
        }else{
            printf("Transmission didn't start with a start packet.\n");
            for(unsigned int i=0;i<10;++i)
                printf("%i ",packet[i]);
        }
    }

    //under this line (call to llclose) also correct

    printf("Terminating the connection.\n");
    llclose(1);
}
