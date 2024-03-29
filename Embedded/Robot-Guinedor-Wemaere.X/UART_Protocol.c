#include <xc.h>
#include "UART_Protocol.h"
#include "CB_RX1.h"
#include "CB_TX1.h"
#include "UART.h"
#include "main.h"
#include "IO.h"
#include "Asservissement.h"
#include "QEI.h"
#include "Utilities.h"
#include "robot.h"

int msgDecodedFunction = 0;
int msgDecodedPayloadLength = 0;
unsigned char msgDecodedPayload[128];
int msgDecodedPayloadIndex = 0;
int reset;
unsigned char receivedChecksum;

typedef enum {
    Waiting,
            
    FunctionMSB,
    FunctionLSB,
    PayloadLengthMSB,
    PayloadLengthLSB,
    Payload,
    CheckSum
    
} StateReception;

StateReception rcvState = Waiting;


unsigned char UartCalculateChecksum(int msgFunction,int msgPayloadLength, unsigned char* msgPayload)
{
    int i;
    unsigned char  checksum = 0xFE;
    checksum ^= msgFunction>>8;
    checksum ^= msgFunction>>0;
    checksum ^= msgPayloadLength>>8;
    checksum ^= msgPayloadLength>>0;
    for (i = 0 ; i < msgPayloadLength; i++)
    {
        checksum ^= msgPayload[i];
    }
    return checksum;
}

void UartEncodeAndSendMessage(int msgFunction,int msgPayloadLength, unsigned char* msgPayload)
{
    int pos=0,i;
    unsigned char trame[6+msgPayloadLength];
    trame[pos++]=0xFE;
    trame[pos++]= msgFunction>>8;
    trame[pos++]= msgFunction>>0;
    trame[pos++]= msgPayloadLength>>8;
    trame[pos++]=msgPayloadLength>>0;
    for(i=0;i<msgPayloadLength;i++)
    {
        trame[pos++]=msgPayload[i];
    }
    trame[pos++]= UartCalculateChecksum(msgFunction, msgPayloadLength, msgPayload);
    SendMessage(trame,pos);
           
}

void UartDecodeMessage(unsigned char c)
{
    switch (rcvState)
    {
        case Waiting:
            if (c == 0xFE)
            {
                rcvState = FunctionMSB;
            }
            msgDecodedFunction = 0;
            msgDecodedPayloadIndex = 0;
            msgDecodedPayloadLength = 0;
        break;

        case FunctionMSB:
            msgDecodedFunction = (unsigned char)(c << 8);
            rcvState = FunctionLSB;
        break;

        case FunctionLSB:
            msgDecodedFunction += (unsigned char)(c << 0);
            rcvState = PayloadLengthMSB;
        break;

        case PayloadLengthMSB:
            msgDecodedPayloadLength = (unsigned char)(c << 8);
            rcvState = PayloadLengthLSB;
        break;

        case PayloadLengthLSB:
            msgDecodedPayloadLength += (unsigned char)(c << 0);
            if(msgDecodedPayloadLength != 0)
            {
                msgDecodedPayloadIndex = 0;
                rcvState = Payload;
            }
            else 
            {
                rcvState = CheckSum;
            }
        break;

        case Payload:
            msgDecodedPayload[msgDecodedPayloadIndex] = c;
            msgDecodedPayloadIndex++;
            if(msgDecodedPayloadIndex >= msgDecodedPayloadLength)
            {
                rcvState = CheckSum;
            }
        break;
           
        case CheckSum: 
            receivedChecksum = c;
            unsigned char calculatedChecksum = UartCalculateChecksum( msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            if (calculatedChecksum == receivedChecksum)
            {
                UartProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            }
            rcvState = Waiting;
        break;
        
        
            
            

        default:
            rcvState = Waiting;
        break;
        
        
        
       
    }
}

double model, Kp, Ki, Kd, KpMax, KiMax, KdMax;
        
void UartProcessDecodedMessage(unsigned char function,unsigned char payloadLength, unsigned char * payload)
{
    switch(function)
    {
        case SET_ROBOT_STATE:
            SetRobotState(payload[0]);
        
        case SET_ROBOT_MANUAL_CONTROL:
            SetRobotAutoControlState(payload[0]);
            LED_BLEUE=1;
            LED_ORANGE=1;
        break;
            
        case RESET_ODO:
            reset=1;
        break;
        
        case ASSERVISSEMENT: 
            model = getFloat(payload, 0);
            Kp = getFloat(payload, 4);
            Ki = getDouble(payload, 8);
            Kd = getDouble(payload, 12);
            KpMax = getDouble(payload, 16);
            KiMax = getDouble(payload, 20);
            KdMax = getDouble(payload, 24);
            
            
            if (model == 1) {
                SetupPidAsservissement(&robotState.PidX, Kp, Ki, Kd, KpMax, KiMax, KdMax);
            } 
            else if (model == 0) {
                SetupPidAsservissement(&robotState.PidTheta, Kp, Ki, Kd, KpMax, KiMax, KdMax);
            }
            
        break;
    }
}

