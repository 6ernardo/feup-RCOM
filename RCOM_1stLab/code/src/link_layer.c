// Link layer protocol implementation

#include "link_layer.h"

#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define BAUDRATE B38400

#define BUF_SIZE 256

#define FLAG 0x7e
#define ADDRESS_TX 0x03
#define ADDRESS_RX 0x01
#define ESCAPE 0x7d
#define ESCAPE_FLAG 0x5e
#define ESCAPE_ESCAPE 0x5d

#define CTRL_SET 0x03
#define CTRL_DISC 0x0b
#define CTRL_UA 0x07
#define CTRL_RR(R) (R)%2?0b10000101:0b00000101
#define CTRL_REJ(R) (R)%2?0b10000001:0b00000001
#define CTRL_DATA(S) (S)%2?0b01000000:0b00000000

//GLOBALDATA

typedef enum {START, FLAGRCV, ARCV, CRCV, BCCOK1, DATARCV, ESCRCV, BCCOK2, REJRCV, STOP} state;
typedef struct {
    state state_c;
    unsigned char address;
    unsigned char control;
    unsigned char bcc;
    unsigned char *data;
    unsigned int datasize;
} State;

State state_var;

LinkLayer ll;
struct termios oldtio;
struct termios newtio;

int fd;

int flagDISC = FALSE;

unsigned char buf[256];
unsigned char DataFlag=0;

unsigned long BUFsize=0;
// unsigned char *BUF=NULL; is this used?

//ALARM
int alarmEnabled = FALSE;
int alarmTries = 0;

void alarmHandler(int signal){
    alarmEnabled = FALSE;
    alarmTries++;

    printf("Alarm #%d\n", alarmTries);
}

//STATE MACHINE
void state_machine(unsigned char buffer, State* state){
    if(state->state_c==REJRCV || state->state_c==STOP){
        state->state_c=START;
    }
    
    if(state->state_c==START){
        if(buffer==FLAG){
            state->state_c=FLAGRCV;
        }
    }
    else if(state->state_c==FLAGRCV){
        state->datasize=0;
        if(buffer==ADDRESS_TX || buffer==ADDRESS_RX){
            state->state_c=ARCV;
            state->address=buffer;
        }
        else if(buffer==FLAG){
            state->state_c=FLAGRCV;
        }
        else{
            state->state_c=START;
        }
    }
    else if(state->state_c==ARCV){
        if(buffer==CTRL_SET || buffer==CTRL_UA || buffer==CTRL_DISC
        || buffer==CTRL_REJ(0) || buffer==CTRL_REJ(1) || buffer==CTRL_RR(0) 
        || buffer==CTRL_RR(1) || buffer==CTRL_DATA(0) || buffer==CTRL_DATA(1)){
            state->state_c=CRCV;
            state->control=buffer;
            state->bcc=state->address ^ state->control;
        }
        else if(buffer==FLAG){
            state->state_c=FLAGRCV;
        }
        else{
            state->state_c=START;
        }
    }
    else if(state->state_c==CRCV){
        if(buffer==state->bcc){
            state->state_c=BCCOK1;
        }
        else if(state->state_c==FLAGRCV){
            state->state_c=FLAGRCV;
        }
        else{
            state->state_c=START;
        }
    }
    else if(state->state_c==BCCOK1){
        if(buffer==FLAG){
            if(state->control==CTRL_DATA(0) || state->control==CTRL_DATA(1)){
                state->state_c=FLAGRCV;
            }
            else{
                state->state_c=STOP;
            }
        }
        else if(state->control==CTRL_DATA(0) || state->control==CTRL_DATA(1)){
            if(state->data!=NULL){
                state->datasize=0;
                if(buffer==ESCAPE){
                    state->state_c=ESCRCV;
                    state->bcc=0;
                }
                else{
                    state->data[state->datasize++]=buffer;
                    state->bcc=buffer;
                    state->state_c=DATARCV;
                }
            }
        }
        else{
            state->state_c=START;
        }
    }
    else if(state->state_c==DATARCV){
        if(buffer==ESCAPE){
            state->state_c=ESCRCV;
        }
        else if(buffer==FLAG){
            state->state_c=REJRCV;
        }
        else if(buffer==state->bcc){
            state->state_c=BCCOK2;
        }
        else{
            state->data[state->datasize++]=buffer;
            state->bcc^=buffer;
        }
    }
    else if(state->state_c==ESCRCV){
        if(buffer==FLAG){
            state->state_c=REJRCV;
        }
        else if(buffer==ESCAPE_FLAG){
            if(state->bcc==FLAG){
                state->state_c=BCCOK2;
            }
            else{
                state->bcc^=FLAG;
                state->data[state->datasize++]=FLAG;
                state->state_c=DATARCV;
            }
        }
        else if(buffer==ESCAPE_ESCAPE){
            if(state->bcc==ESCAPE){
                state->state_c=BCCOK2;
            }
            else{
                state->bcc^=ESCAPE;
                state->data[state->datasize++]=ESCAPE;
                state->state_c=DATARCV;
            }
        }
        else{
            state->state_c=START;
        }
    }
    else if(state->state_c==BCCOK2){
        if(buffer==FLAG){
            state->state_c=STOP;
        }
        else if(buffer==0){
            state->data[state->datasize++]=state->bcc;
            state->bcc=0;
        }
        else if(buffer==ESCAPE){
            state->data[state->datasize++]=state->bcc;
            state->bcc=0;
            state->state_c=ESCRCV;
        }
        else{
            state->data[state->datasize++]=state->bcc;
            state->data[state->datasize++]=buffer;
            state->bcc=buffer;
            state->state_c=DATARCV;
        }
    }
}

//LLOPEN_TX
int llopen_tx(LinkLayer transmitter){
    
    signal(SIGALRM, alarmHandler);

    int flagUA = FALSE;
    alarmTries = 0;

    while(alarmTries <= transmitter.nRetransmissions && !flagUA){
        if(!alarmEnabled){

            //build SET frame
            unsigned char BCC_SET = ADDRESS_TX ^ CTRL_SET;
            unsigned char set[5] = {FLAG, ADDRESS_TX, CTRL_SET, BCC_SET, FLAG};

            //send set
            int s = write(fd, set, 5);
            
            if(s!=5){
                perror("SET FRAME not sent.\n");
                exit(-1);
            }

            printf("TX:Sent SET frame.\n");

            //set alarm
            alarm(transmitter.timeout);
            alarmEnabled = TRUE;

            while(alarmEnabled && !flagUA){
                //read from receiver
                int s1 = read(fd, buf, 128);
                //printf("s1: %d\n", s1);

                for(int i=0; i<s1; i++){
                    state_machine(buf[i], &state_var);
                    //printf("STATE AFTER MACHINE: %d\n", state);

                    if(state_var.state_c == STOP && state_var.control==CTRL_UA){
                        flagUA = TRUE;
                    }
                }
            }

        }

    }

    if(flagUA){
        printf("TX:Received UA frame.\n");
        alarm(0);
        alarmEnabled = FALSE;
        return 1;
    }
   
    return -1;
}

//LLOPEN_RX
int llopen_rx(LinkLayer receiver){
    int flagSET = FALSE;

    while(!flagSET){
        //read from transmitter
        int s = read(fd, buf, 128);
        if(s<0) return -1;

        for(int i=0; i<s; i++){
            state_machine(buf[i], &state_var);
            //printf("STATE AFTER MACHINE: %d\n", state);
            if(state_var.state_c == STOP && state_var.control==CTRL_SET) flagSET = TRUE;
        }
    }

    if(flagSET){
        printf("RX:Received SET frame.\n");

        //building and sending UA
        unsigned char BCC_UA = ADDRESS_RX ^ CTRL_UA;
        unsigned char ua[5] = {FLAG, ADDRESS_RX, CTRL_UA, BCC_UA, FLAG};

        int s1 = write(fd, ua, 5);

        if(s1!=5){
            perror("UA frame not sent.\n");
            exit(-1);
        }

        printf("RX:Sent UA frame.\n");

        return 1;
    }

    return -1;
}

//STUFFING

int stuffing(const unsigned char *buffer, int bufSize, unsigned char* D, unsigned char *bcc)
{
    int size=0;

    for(unsigned int i=0; i<bufSize; i++)
    {
        if(bcc!=NULL) *bcc^=buffer[i];
        if(buffer[i]==FLAG){
            D[size++]=ESCAPE;
            D[size++]=ESCAPE_FLAG;
            break;
        }
        if(buffer[i]==ESCAPE){
            D[size++]=ESCAPE;
            D[size++]=ESCAPE_ESCAPE;
            break;
        }
        //if(buf[i]==CTRL_DATA(0) || buf[i]==CTRL_DATA(1)) printf("------------------------Data\n");
        D[size++]=buffer[i];
        //printf("buf->>>%x\nd---->>>>>>%x\n",buf[i], D[size-1]);
    }

    return size;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters){
    ll=connectionParameters;
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 1; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    if(connectionParameters.role == LlRx){
        if(llopen_rx(connectionParameters)>0) return 1;
    }
    else if(connectionParameters.role == LlTx){
        if(llopen_tx(connectionParameters)>0) return 1;
    }
    else {
        perror("LinkLayer not valid\n");
    }

    return -1;
}


////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize){
    //build data frame
    unsigned char data[bufSize*2+100];

    data[0]=FLAG;
    data[1]=ADDRESS_TX;
    data[2]=CTRL_DATA(DataFlag);
    data[3]=data[1] ^ data[2];

    int offset=0;
    unsigned char bcc=0;
    for(unsigned int i=0;i<bufSize;++i){
        offset+=stuffing(buf+i,1,data+offset+4,&bcc);
    }
    offset+=stuffing(&bcc,1,data+offset+4,NULL);
    data[4+offset]=FLAG;
    int j= 5+offset;

    //BIT OF CODE THAT IM NOT SURE IS NECESSARY HERE, WILL TEST AND MIGHT PUT LATER

    //printf("address:%x\ncontrol: %x\nbcc: %x\n",data[1], data[2], data[3]);
    //unsigned char *stdata=NULL;

    alarmTries=0;
    int flagRR = FALSE;
    state_var.data=NULL;

    signal(SIGALRM, alarmHandler);
    
    while(alarmTries <= ll.nRetransmissions && !flagRR){
        if(!alarmEnabled){
            //send Data Frame
            int bytes=write(fd, data, j);
            if(bytes==-1){
                perror("llwrite():Write error.\n");
            }

            printf("TX:Sent DATA Frame.\n");

            //set alarm
            alarm(ll.timeout);
            alarmEnabled = TRUE;

            while(alarmEnabled && !flagRR){
                int bytes_read = read(fd, buf, 256);
                if(bytes_read<0){
                    return -1;
                } 

                for(int i=0; i<bytes_read; ++i){
                    state_machine(buf[i], &state_var);
                    if(state_var.state_c==STOP){
                        if(state_var.address==ADDRESS_TX && state_var.control==CTRL_RR(DataFlag)){
                            flagRR = TRUE;
                        }
                        else if(state_var.address==ADDRESS_TX && state_var.control==CTRL_REJ(DataFlag)){
                            alarmTries=0;
                            printf("Requesting retransmission, reset alarm tries.\n");
                        }
                    }
                }
            }
        }
    }

    if(flagRR){
        printf("TX:Received RR frame.\n");
        alarm(0);
        alarmEnabled = FALSE;
        DataFlag=DataFlag?0:1;
        return 0;
    }

    return -1;
} //llwrite still very similar to previous versions, but I everything seems correct so I wont change anything for now

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet){
    int flagData = FALSE;

    state_var.data=packet;

    unsigned char rej[5];
    unsigned char rr[5];
    unsigned char ua[5];
    
    while(!flagData){
        //read from transmitter
        int bytes_read = read(fd, buf, 256);
        if(bytes_read<0) return -1;

        //printf("buf2: %x\nCTRLDATA:%x\n\n", buf[2], CTRL_DATA(DataFlag));

        for(int i=0; i<bytes_read && !flagData; ++i){
            state_machine(buf[i],&state_var);
            //printf(" RX: buf[i]: %x. state:%d\n", buf[i], state_var.state_c);
            
            if(state_var.state_c==REJRCV && state_var.address==ADDRESS_TX){
                //build and send REJ frame
                rej[0]=FLAG;
                rej[1]=ADDRESS_TX;
                if(state_var.control==CTRL_DATA(0)) rej[2]=CTRL_REJ(0);
                else state_var.control=CTRL_REJ(1);
                rej[3]= rej[1] ^ rej[2];
                rej[4]=FLAG;
                
                write(fd, rej, 5);
                printf(" RX: Sent REJ frame.\n");
            }
            if(state_var.state_c==STOP && state_var.address==ADDRESS_TX){
                if(state_var.control==CTRL_SET){
                    //build and send UA frame
                    ua[0]=FLAG;
                    ua[1]=ADDRESS_TX;
                    ua[2]=CTRL_UA;
                    ua[3]= ua[1] ^ ua[2];
                    ua[4]=FLAG;
                    
                    write(fd, ua, 5);
                    printf(" RX: Sent UA frame.\n");
                }
                else if(state_var.control==CTRL_DATA(DataFlag)){
                    DataFlag=DataFlag?0:1;
                    //build and send RR frame
                    rr[0]=FLAG;
                    rr[1]=ADDRESS_TX;
                    rr[2]=CTRL_RR(DataFlag);
                    rr[3]= rr[1] ^ rr[2];
                    rr[4]=FLAG;
                    
                    write(fd, rr, 5);
                    printf(" RX: Sent RR frame.\n");
                    printf( "RX: datasize: %d\n", state_var.datasize);

                    return state_var.datasize;
                }
                else{
                    //build and send RR frame
                    rr[0]=FLAG;
                    rr[1]=ADDRESS_TX;
                    rr[2]=CTRL_RR(DataFlag);
                    rr[3]= rr[1] ^ rr[2];
                    rr[4]=FLAG;

                    write(fd,rr,5);
                    printf(" RX: Sent RR frame, requesting retransmission\n");
                }
            }
            if(state_var.control==CTRL_DISC){
                //send REJ
                rej[0]=FLAG;
                rej[1]=ADDRESS_TX;
                if(state_var.control==CTRL_DATA(0)) rej[2]=CTRL_REJ(0);
                else state_var.control=CTRL_REJ(1);
                rej[3]= rej[1] ^ rej[2];
                rej[4]=FLAG;

                flagDISC = TRUE;
                
                write(fd, rej, 5);
                printf(" RX: Received DISC. Sent REJ frame.\n");
            }

        }
    }

    return 0;
}


//LLCLOSE_TX
int llclose_tx(LinkLayer transmitter, int fd){

    signal(SIGALRM,alarmHandler);

    int flagDISCtx = FALSE;
    alarmTries = 0;

    while(alarmTries <= transmitter.nRetransmissions && !flagDISCtx){
        if(!alarmEnabled){
            //build DISC frame
            unsigned char BCC_DISC = ADDRESS_TX ^ CTRL_DISC;
            unsigned char disc[5] = {FLAG, ADDRESS_TX, CTRL_DISC, BCC_DISC, FLAG};

            //send DISC
            int s = write(fd, disc, 5);

            if(s!=5){
                perror("DISC FRAME not sent.\n");
                exit(-1);
            }

            printf(" TX:Sent DISC frame.\n");

            //set alarm
            alarm(transmitter.timeout);
            alarmEnabled = TRUE;

            while(alarmEnabled && !flagDISCtx){
                //read from receiver
                int s1 = read(fd, buf, 128);

                for(int i=0; i<s1; i++){
                    state_machine(buf[i], &state_var);
                    if(state_var.state_c == STOP && state_var.control==CTRL_DISC){
                        flagDISCtx = TRUE;
                    }
                }
            }
        }
    }

    if(flagDISCtx){
        //build UA frame
        unsigned char BCC_UA = ADDRESS_TX ^ CTRL_UA;
        unsigned char ua[5] = {FLAG, ADDRESS_TX, CTRL_UA, BCC_UA, FLAG};

        //send UA
        int s2 = write(fd, ua, 5);

        if(s2!=5){
            perror("UA FRAME not sent.\n");
            exit(-1);
        }

        printf(" TX:Received DISC and sent UA.\n");

        return 1;
    }

    return -1;
} 

//LLCLOSE_RX
int llclose_rx(LinkLayer receiver, int fd){
    int flagUA = FALSE;

    while(!flagDISC){
        //read from transmitter
        int s = read(fd, buf, 128);
        if(s<0) return -1;

        for(int i=0; i<s; i++){
            state_machine(buf[i], &state_var);
            if(state_var.state_c == STOP && state_var.control==CTRL_DISC){
                flagDISC = TRUE;
            }
        }
    }

    if(flagDISC){
        //build DISC
        unsigned char BCC_DISC = ADDRESS_RX ^ CTRL_DISC;
        unsigned char disc[5] = {FLAG, ADDRESS_RX, CTRL_DISC, BCC_DISC, FLAG};

        //send DISC
        int s1 = write(fd, disc, 5);

        if(s1!=5){
            perror("DISC FRAME not sent.\n");
            exit(-1);
        }

        printf("RX:Received DISC and sent DISC.\n");
    }


    while(!flagUA){
        //read from transmitter
        int s = read(fd, buf, 128);
        if(s<0) return -1;

        for(int i=0; i<s; i++){
            state_machine(buf[i], &state_var);
            if(state_var.state_c == STOP && state_var.control==CTRL_UA){
                flagUA = TRUE;
            }
        }
    }

    if(flagUA){
        printf("RX:Received UA.\n");
        return 1;
    }

    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    if(ll.role == LlTx){ //Transmitter
        if(llclose_tx(ll, fd)<0) return -1;
    }
    else if(ll.role == LlRx){ //Receiver
        if(llclose_rx(ll, fd)<0) return -1;
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 1;
}